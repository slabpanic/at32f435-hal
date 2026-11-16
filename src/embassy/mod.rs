//! Embassy integration: time driver selection and initialization.
//!
//! Two drivers, selected via features (mutually exclusive; enabling both
//! triggers a `compile_error!`):
//! - `time-driver-systick` (default): Cortex-M SysTick, 1 ms interrupt + CVR
//!   interpolation; occupies no peripheral timer. The SysTick exception is
//!   defined by this HAL (applications can no longer define their own `SysTick`
//!   exception).
//! - `time-driver-tim2`: TMR2 32-bit timer (period counting + a single alarm on
//!   the C1 compare channel), suitable when SysTick is used for other purposes.
//!   The `TMR2` interrupt is owned by this HAL; applications must not bind it.
//!
//! Applications must bring their own `embassy-time` dependency (pick one
//! `generic-queue-N` feature to set the queue capacity).

#[cfg(feature = "time-driver-systick")]
#[path = "time_driver_systick.rs"]
mod time_driver;

#[cfg(all(feature = "time-driver-tim2", not(feature = "time-driver-systick")))]
#[path = "time_driver_tmr.rs"]
mod time_driver;

pub(crate) fn init() {
    time_driver::init();
}

/// Current time (microseconds). Convenience API for defmt timestamps and the like:
/// `defmt::timestamp!("{=u64:us}", at32f435_hal::embassy::now_us());`
pub fn now_us() -> u64 {
    time_driver::now_us()
}
