//! TMR2 time driver for embassy-time (the `time-driver-tim2` feature;
//! alternative implementation).
//!
//! TMR2 on the AT32F435 is a 32-bit timer (TMR5 shares the same register
//! layout): PSC divides the clock down to TICK_HZ and PR is set to u32::MAX
//! for free-running; the overflow interrupt maintains the 64-bit high bits
//! (`period`), and compare channel C1 acts as a single alarm (c1dt = low 32
//! bits of the alarm timestamp). The queue is the same as the systick driver's
//! (`Queue`).
//!
//! WARNING: The `TMR2` interrupt (vector `TMR2`) is owned by this driver: applications
//! must not `bind_interrupts!` for it again.
//! Verified on the AT32F435CGT7 EVB with concurrent timers expiring in
//! deadline order.

use core::cell::{Cell, RefCell};
use core::task::Waker;

use critical_section::Mutex as CsMutex;
use embassy_time_driver::{time_driver_impl, Driver, TICK_HZ};
use embassy_time_queue_utils::Queue;

use super::scale;

fn tmr() -> &'static crate::pac::tmr2::RegisterBlock {
    unsafe { &*crate::pac::TMR2::PTR }
}

struct TimerDriver {
    /// Number of 32-bit counter overflows (high bits of the tick).
    period: CsMutex<Cell<u32>>,
    /// Current alarm timestamp (in ticks); u64::MAX when no alarm is pending.
    alarm: CsMutex<Cell<u64>>,
    queue: CsMutex<RefCell<Queue>>,
}

impl TimerDriver {
    const fn new() -> Self {
        Self {
            period: CsMutex::new(Cell::new(0)),
            alarm: CsMutex::new(Cell::new(u64::MAX)),
            queue: CsMutex::new(RefCell::new(Queue::new())),
        }
    }
}

time_driver_impl!(static DRIVER: TimerDriver = TimerDriver::new());

impl TimerDriver {
    /// Configures TMR2: PSC divider, PR at full range, overflow/C1 compare
    /// interrupts, and starts counting.
    pub(crate) fn init(&'static self) {
        <crate::peripherals::TMR2 as crate::peripheral::ClockControl>::enable_and_reset();
        let t = tmr();

        // Divide the timer clock (2*PCLK1 when the APB1 prescaler > 1) down to TICK_HZ
        let timer_freq = crate::rcc::clocks().pclk1_tim.0;
        let tick_hz = u32::try_from(TICK_HZ).expect("tick rate exceeds the TMR2 clock");
        assert!(
            tick_hz != 0 && timer_freq >= tick_hz && timer_freq.is_multiple_of(tick_hz),
            "TMR2 clock must be an integer multiple of the time driver tick rate"
        );
        let psc = timer_freq / tick_hz - 1;
        assert!(
            psc <= 0xFFFF,
            "TMR2 prescaler exceeds 16 bits (clock tree misconfigured)"
        );

        t.ctrl1().modify(|_, w| {
            w.tmren().clear_bit(); // stop the timer while configuring
            w.pmen().set_bit(); // TMR2/5 requires explicitly enabling 32-bit mode (official tmr_32_bit_function_enable)
            w
        });
        t.div().write(|w| w.set(psc as u16));
        t.pr().write(|w| w.set(u32::MAX));
        // Load the buffered prescaler immediately, then clear the update flag.
        t.swevt().write(|w| w.ovfswtr().overflow());
        // The queue arms C1 when it has a deadline.
        t.ists().write(|w| {
            w.ovfif().clear();
            w.c1if().clear();
            w
        });
        t.iden().modify(|_, w| {
            w.ovfien().set_bit();
            w.c1ien().clear_bit();
            w
        });
        t.cctrl().modify(|_, w| w.c1en().set_bit());
        t.ctrl1().modify(|_, w| w.tmren().set_bit());

        cortex_m::peripheral::NVIC::unpend(crate::pac::Interrupt::TMR2);
        unsafe { cortex_m::peripheral::NVIC::unmask(crate::pac::Interrupt::TMR2) };
    }

    fn on_interrupt(&self) {
        critical_section::with(|cs| {
            let t = tmr();
            let status = t.ists().read();
            if status.ovfif().bit_is_set() {
                t.ists().write(|w| w.ovfif().clear());
                let period = self.period.borrow(cs);
                period.set(period.get().wrapping_add(1));
            }
            if status.c1if().bit_is_set() {
                t.ists().write(|w| w.c1if().clear());
            }
            if self.now_locked(cs) >= self.alarm.borrow(cs).get() {
                self.dispatch_and_arm(cs, &mut self.queue.borrow_ref_mut(cs));
            }
        });
    }

    fn now_locked(&self, cs: critical_section::CriticalSection<'_>) -> u64 {
        let t = tmr();
        let mut period = self.period.borrow(cs).get();
        let mut counter = t.cval().read().bits();
        if t.ists().read().ovfif().bit_is_set() {
            period = period.wrapping_add(1);
            counter = t.cval().read().bits();
        }
        (u64::from(period) << 32) | u64::from(counter)
    }

    fn dispatch_and_arm(&self, cs: critical_section::CriticalSection<'_>, queue: &mut Queue) {
        let t = tmr();
        loop {
            let next = queue.next_expiration(self.now_locked(cs));
            self.alarm.borrow(cs).set(next);
            t.iden().modify(|_, w| w.c1ien().clear_bit());
            if next == u64::MAX {
                return;
            }
            t.c1dt().write(|w| w.set(next as u32));
            t.ists().write(|w| w.c1if().clear());
            t.iden().modify(|_, w| w.c1ien().set_bit());
            // The compare match can pass while changing the register.
            if self.now_locked(cs) < next {
                return;
            }
        }
    }
}

impl Driver for TimerDriver {
    fn now(&self) -> u64 {
        critical_section::with(|cs| self.now_locked(cs))
    }

    fn schedule_wake(&self, at: u64, waker: &Waker) {
        critical_section::with(|cs| {
            let mut q = self.queue.borrow_ref_mut(cs);
            q.schedule_wake(at, waker);
            self.dispatch_and_arm(cs, &mut q);
        });
    }
}

/// Current time (microseconds, converted from TICK_HZ), for defmt timestamps
/// and the like.
pub fn now_us() -> u64 {
    scale(DRIVER.now(), TICK_HZ, 1_000_000)
}

/// Driver entry point (called by `embassy::init()`).
pub(crate) fn init() {
    DRIVER.init();
}

/// TMR2 interrupt handler (owned by the HAL; do not bind it again in applications).
#[cfg(feature = "rt")]
#[cortex_m_rt::interrupt]
fn TMR2() {
    // cortex-m-rt's #[interrupt] macro validates the interrupt's existence via
    // `interrupt::TMR2`, so an `interrupt` name must be in scope (this HAL's
    // interrupt module already exports the IRQ variant).
    use crate::interrupt;
    DRIVER.on_interrupt();
}
