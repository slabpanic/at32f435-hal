//! SysTick time driver for embassy-time (the HAL default; ported from the
//! proven evb_at32f435cgt7 implementation).
//!
//! A 1 ms SysTick interrupt maintains the millisecond counter (a critical-
//! section protected `Cell<u64>`, because the thumbv7em target has no 64-bit
//! atomics); `now()` uses CVR to interpolate within the millisecond down to
//! microseconds. The waker queue uses embassy-time-queue-utils' `Queue`, and
//! due dispatching happens in the SysTick exception.
//!
//! Reference: embassy-time/src/driver_mock.rs (queue usage of the official
//! mock driver).

use core::cell::{Cell, RefCell};
use core::sync::atomic::{AtomicU32, Ordering};
use core::task::Waker;

use critical_section::Mutex as CsMutex;
use embassy_time_driver::{time_driver_impl, Driver, TICK_HZ};
use embassy_time_queue_utils::Queue;

/// SysTick reload value (1 ms period), written by `init()` from the current
/// system clock.
static RELOAD: AtomicU32 = AtomicU32::new(0);

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
        let sysclk = crate::rcc::clocks().sysclk.0;
        let reload = sysclk / 1_000 - 1;
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

    #[inline]
    fn now_parts() -> (u64, u64) {
        // ms and CVR must be read together atomically (otherwise interrupt
        // tearing makes the reading jump backwards), hence the critical section.
        critical_section::with(|cs| {
            let ms = DRIVER.ms.borrow(cs).get();
            let reload = RELOAD.load(Ordering::Relaxed);
            // SysTick counts down: the smaller CVR is, the more of this
            // millisecond has elapsed.
            // Read SYST_CVR @ 0xE000E018 via a direct address to avoid stealing
            // the core peripheral singleton.
            let cvr = unsafe { (0xE000_E018 as *const u32).read_volatile() };
            // Microseconds elapsed within the current millisecond
            // (ticks_per_us = (reload+1)/1000 = sysclk/1e6)
            let ticks_per_us = (reload + 1) / 1_000;
            let sub_us = ((reload + 1 - cvr.min(reload + 1)) / ticks_per_us) as u64;
            (ms, sub_us)
        })
    }
}

impl Driver for TimeDriver {
    fn now(&self) -> u64 {
        let (ms, sub_us) = Self::now_parts();
        (ms * 1_000 + sub_us) * TICK_HZ / 1_000_000
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
    DRIVER.now() * 1_000_000 / TICK_HZ
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
            .next_expiration(ms * 1_000 * TICK_HZ / 1_000_000);
    });
}
