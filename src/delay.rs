//! Blocking delay (coarse-grained): approximate cycle-count loops based on
//! `cortex_m::asm::delay`.
//!
//! For precise timing, use embassy-time (`Timer::after_*`); this module is for
//! early startup or simple waits before the embassy driver has been initialized.

use crate::rcc::clocks;

/// Blocks for `us` microseconds (approximate; `asm::delay` takes about 4 cycles
/// per loop, with a conservative margin included).
pub fn delay_us(us: u32) {
    let cycles = (us as u64) * clocks().sysclk.0 as u64 / 1_000_000;
    cortex_m::asm::delay(cycles.min(u32::MAX as u64) as u32);
}

/// Blocks for `ms` milliseconds.
pub fn delay_ms(ms: u32) {
    for _ in 0..ms {
        delay_us(1_000);
    }
}

/// Unit struct implementing embedded-hal 1.0 `DelayNs` (delay_us/delay_ms are
/// its default methods).
#[derive(Copy, Clone, Debug, Default)]
pub struct Delay;

impl embedded_hal::delay::DelayNs for Delay {
    #[inline]
    fn delay_ns(&mut self, ns: u32) {
        let cycles = (ns as u64) * clocks().sysclk.0 as u64 / 1_000_000_000;
        cortex_m::asm::delay(cycles.min(u32::MAX as u64) as u32);
    }
}
