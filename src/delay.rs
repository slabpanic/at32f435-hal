//! Blocking delay (coarse-grained): cycle-count loops based on
//! `cortex_m::asm::delay`.
//!
//! For precise timing, use embassy-time (`Timer::after_*`); this module is for
//! early startup or simple waits before the embassy driver has been
//! initialized.
//!
//! Timing model (hw-measured on the evb @ 288 MHz, 2026-11-18):
//! `asm::delay` decrements a register with `subs` + conditional branch, ~3
//! cycles per iteration on the Cortex-M4F pipeline - the requested cycle
//! count is therefore divided by 3 to hit the requested wall time (the old
//! code passed the full cycle count and ran 3x long).

use crate::rcc::clocks;

/// Cortex-M4F `asm::delay` loop cost in cycles (subs + branch, measured).
const ASM_DELAY_CYCLES_PER_ITER: u64 = 3;

/// Blocks for `us` microseconds (cycle-count accurate to the loop model above).
pub fn delay_us(us: u32) {
    delay_cycles(u64::from(us) * u64::from(clocks().hclk.0), 1_000_000);
}

/// Blocks for `ms` milliseconds.
pub fn delay_ms(ms: u32) {
    delay_cycles(u64::from(ms) * u64::from(clocks().hclk.0), 1_000);
}

fn delay_cycles(scaled_cycles: u64, units_per_second: u64) {
    let mut iterations = scaled_cycles.div_ceil(units_per_second * ASM_DELAY_CYCLES_PER_ITER);
    while iterations != 0 {
        let chunk = iterations.min(u64::from(u32::MAX)) as u32;
        cortex_m::asm::delay(chunk);
        iterations -= u64::from(chunk);
    }
}

/// Unit struct implementing embedded-hal 1.0 `DelayNs` (delay_us/delay_ms are
/// its default methods).
#[derive(Copy, Clone, Debug, Default)]
pub struct Delay;

impl embedded_hal::delay::DelayNs for Delay {
    #[inline]
    fn delay_ns(&mut self, ns: u32) {
        delay_cycles(u64::from(ns) * u64::from(clocks().hclk.0), 1_000_000_000);
    }
}
