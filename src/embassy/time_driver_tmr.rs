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
//! WARNING: Compile-checked only (the evb defaults to the systick path); not
//! regression-tested on hardware - if you switch this feature on, test it on
//! the board first.

use core::cell::{Cell, RefCell};
use core::sync::atomic::{AtomicU32, Ordering};
use core::task::Waker;

use critical_section::Mutex as CsMutex;
use embassy_time_driver::{time_driver_impl, Driver, TICK_HZ};
use embassy_time_queue_utils::Queue;

fn tmr() -> &'static crate::pac::tmr2::RegisterBlock {
    unsafe { &*crate::pac::TMR2::PTR }
}

struct TimerDriver {
    /// Number of 32-bit counter overflows (high bits of the tick).
    period: AtomicU32,
    /// Current alarm timestamp (in ticks); u64::MAX when no alarm is pending.
    alarm: CsMutex<Cell<u64>>,
    queue: CsMutex<RefCell<Queue>>,
}

impl TimerDriver {
    const fn new() -> Self {
        Self {
            period: AtomicU32::new(0),
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
        <crate::peripherals::TMR2 as crate::peripheral::RccPeripheral>::enable_and_reset();
        let t = tmr();

        // Divide the timer clock (2*PCLK1 when the APB1 prescaler > 1) down to TICK_HZ
        let timer_freq = crate::rcc::clocks().pclk1_tim.0;
        let psc = timer_freq / TICK_HZ as u32 - 1;
        assert!(psc <= 0xFFFF, "TMR2 prescaler exceeds 16 bits (clock tree misconfigured)");

        t.ctrl1().modify(|_, w| {
            w.tmren().clear_bit(); // stop the timer while configuring
            w.pmen().set_bit(); // TMR2/5 requires explicitly enabling 32-bit mode (official tmr_32_bit_function_enable)
            w
        });
        t.div().write(|w| w.set(psc as u16));
        t.pr().write(|w| w.set(u32::MAX));
        // Clear stale flags (write 0 to clear), enable overflow + C1 compare
        // interrupts, enable the compare channel
        t.ists().write(|w| {
            w.ovfif().clear();
            w.c1if().clear();
            w
        });
        t.iden().modify(|_, w| {
            w.ovfien().set_bit();
            w.c1ien().set_bit();
            w
        });
        t.cctrl().modify(|_, w| w.c1en().set_bit());
        t.ctrl1().modify(|_, w| w.tmren().set_bit());

        unsafe {
            cortex_m::peripheral::NVIC::unmask(crate::pac::Interrupt::TMR2);
        }
    }

    fn on_interrupt(&self) {
        let t = tmr();
        let ists = t.ists().read();
        if ists.ovfif().bit_is_set() {
            t.ists().write(|w| w.ovfif().clear());
            self.period.fetch_add(1, Ordering::Relaxed);
        }
        if ists.c1if().bit_is_set() {
            t.ists().write(|w| w.c1if().clear());
            critical_section::with(|cs| {
                let ts = self.alarm.borrow(cs).get();
                if self.now() >= ts {
                    self.queue.borrow_ref_mut(cs).next_expiration(self.now());
                }
            });
        }
    }

    /// Current 64-bit tick (tick rate = TICK_HZ).
    ///
    /// When the overflow flag is pending but the ISR has not run yet (NVIC
    /// latency), the high bits are period+1 - the reading stays monotonic,
    /// with an error bounded by the ISR latency.
    fn now(&self) -> u64 {
        let t = tmr();
        let period = self.period.load(Ordering::Relaxed);
        let cval = t.cval().read().bits();
        let pending = t.ists().read().ovfif().bit_is_set();
        let hi = if pending { period + 1 } else { period };
        ((hi as u64) << 32) | cval as u64
    }

    /// Sets the C1 compare alarm; returns false if the alarm timestamp has
    /// already passed (the caller should dispatch the queue immediately).
    fn set_alarm(&self, cs: critical_section::CriticalSection<'_>, timestamp: u64) -> bool {
        self.alarm.borrow(cs).set(timestamp);
        if timestamp <= self.now() {
            return false;
        }
        // cval is a 32-bit tick: compare against the low 32 bits of the
        // timestamp directly (the high bits are covered by the overflow count)
        tmr().c1dt().write(|w| w.set(timestamp as u32));
        true
    }
}

impl Driver for TimerDriver {
    fn now(&self) -> u64 {
        self.now()
    }

    fn schedule_wake(&self, at: u64, waker: &Waker) {
        critical_section::with(|cs| {
            let mut q = self.queue.borrow_ref_mut(cs);
            q.schedule_wake(at, waker);
            let next = q.next_expiration(self.now());
            if !self.set_alarm(cs, next) {
                // Alarm already expired: dispatch another round immediately
                let next = q.next_expiration(self.now());
                let _ = self.set_alarm(cs, next);
            }
        });
    }
}

/// Current time (microseconds, converted from TICK_HZ), for defmt timestamps
/// and the like.
pub fn now_us() -> u64 {
    DRIVER.now() * 1_000_000 / TICK_HZ
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
