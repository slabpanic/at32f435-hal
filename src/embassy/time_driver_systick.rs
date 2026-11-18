//! SysTick time driver for embassy-time (the HAL default; ported from the
//! proven evb_at32f435cgt7 implementation).
//!
//! A 1 ms SysTick interrupt maintains the millisecond counter (a critical-
//! section protected `Cell<u64>`, because the thumbv7em target has no 64-bit
//! atomics); `now()` uses CVR to interpolate within the millisecond down to
//! microseconds. The waker queue uses embassy-time-queue-utils' `Queue`, and
//! due dispatching happens in the SysTick exception. Keep SysTick service
//! latency below one period because the NVIC records only one pending wrap.
//!
//! Reference: embassy-time/src/driver_mock.rs (queue usage of the official
//! mock driver).

use core::cell::{Cell, RefCell};
use core::sync::atomic::{AtomicU32, Ordering};
use core::task::Waker;

use critical_section::Mutex as CsMutex;
use embassy_time_driver::{time_driver_impl, Driver, TICK_HZ};
use embassy_time_queue_utils::Queue;

use super::scale;

/// SysTick reload value (1 ms period), written by `init()` from the current
/// system clock.
static RELOAD: AtomicU32 = AtomicU32::new(0);
const PERIOD_HZ: u32 = 1_000;
const SYST_CVR: *const u32 = 0xE000_E018 as *const u32;
const SCB_ICSR: *const u32 = 0xE000_ED04 as *const u32;
const PENDSTSET: u32 = 1 << 26;

/// Millisecond counter: the thumbv7em target has no 64-bit atomics (a Rust
/// target spec limitation), so a critical-section protected Cell is used
/// instead (updated from the 1 kHz interrupt; the overhead is negligible).
struct TimeDriver {
    ms: CsMutex<Cell<u64>>,
    queue: CsMutex<RefCell<Queue>>,
}

impl TimeDriver {
    const fn new() -> Self {
        Self {
            ms: CsMutex::new(Cell::new(0)),
            queue: CsMutex::new(RefCell::new(Queue::new())),
        }
    }
}

time_driver_impl!(static DRIVER: TimeDriver = TimeDriver::new());

impl TimeDriver {
    /// Configures SysTick: Core clock source, 1 ms period, interrupt enabled.
    /// Called once from `hal::init()`.
    pub(crate) fn init(&'static self) {
        let hclk = crate::rcc::clocks().hclk.0;
        assert!(
            hclk >= 2 * PERIOD_HZ,
            "HCLK is too slow for the SysTick time driver"
        );
        let reload = hclk / PERIOD_HZ - 1;
        RELOAD.store(reload, Ordering::Relaxed);

        // The executor's #[embassy_executor::main] has already taken
        // cortex_m::Peripherals (NVIC); steal a copy here to get SYST
        // (core peripherals have no take mechanism, and their register blocks
        // do not overlap).
        let mut syst = unsafe { cortex_m::Peripherals::steal() }.SYST;
        syst.disable_counter();
        syst.set_clock_source(cortex_m::peripheral::syst::SystClkSource::Core);
        syst.set_reload(reload);
        syst.clear_current();
        syst.enable_counter();
        syst.enable_interrupt();
    }

    fn now_at_rate(&self, hz: u64) -> u64 {
        let (ms, elapsed_cycles) = critical_section::with(|cs| {
            let ms = self.ms.borrow(cs).get();
            let reload = RELOAD.load(Ordering::Relaxed);
            if reload == 0 {
                return (ms, 0);
            }
            // Sample CVR again if hardware wrapped before we checked the
            // pending bit. SysTick cannot advance the software count in
            // this critical section.
            let mut current = unsafe { SYST_CVR.read_volatile() };
            let pending = unsafe { SCB_ICSR.read_volatile() } & PENDSTSET != 0;
            if pending {
                current = unsafe { SYST_CVR.read_volatile() };
            }
            let elapsed = if current == 0 { 0 } else { reload - current };
            (ms + u64::from(pending), elapsed)
        });
        // Count cycles at the actual AHB rate so an HCLK which is not a
        // multiple of PERIOD_HZ does not accrue a fixed error every tick.
        let hclk = crate::rcc::clocks().hclk.0;
        let period_cycles = RELOAD.load(Ordering::Relaxed) + 1;
        if hz == 1_000_000 && hclk.is_multiple_of(1_000_000) {
            return ms * 1_000 + u64::from(elapsed_cycles / (hclk / 1_000_000));
        }
        let cycles = ms * u64::from(period_cycles) + u64::from(elapsed_cycles);
        scale(cycles, u64::from(hclk), hz)
    }
}

impl Driver for TimeDriver {
    fn now(&self) -> u64 {
        self.now_at_rate(TICK_HZ)
    }

    fn schedule_wake(&self, at: u64, waker: &Waker) {
        critical_section::with(|cs| {
            let mut q = self.queue.borrow_ref_mut(cs);
            q.schedule_wake(at, waker);
            // Wake immediately for anything already due (mirrors the official
            // mock driver)
            q.next_expiration(self.now());
        });
    }
}

/// Current time (microseconds, converted from TICK_HZ), for defmt timestamps
/// and the like.
pub fn now_us() -> u64 {
    DRIVER.now_at_rate(1_000_000)
}

/// Driver entry point (called by `embassy::init()`).
pub(crate) fn init() {
    DRIVER.init();
}

/// SysTick exception: increments the millisecond counter and dispatches due
/// wakers (the executor does the scheduling via PendSV).
// This exception is owned by the HAL - applications must not define their own
// `#[exception] fn SysTick`.
#[cfg(feature = "rt")]
#[cortex_m_rt::exception]
fn SysTick() {
    critical_section::with(|cs| {
        let ms = DRIVER.ms.borrow(cs).get() + 1;
        DRIVER.ms.borrow(cs).set(ms);
        DRIVER
            .queue
            .borrow_ref_mut(cs)
            .next_expiration(DRIVER.now());
    });
}
