//! Independent watchdog WDT (free watchdog, LICK clock domain).
//!
//! - Clock: LICK ~ 40 kHz (actually 30~60 kHz; only suitable as coarse-grained
//!   timeout protection, do not use it for precise timing).
//! - Timeout = RLD * DIV / f_LICK; RLD is 12-bit (<=4095), DIV in {/4../256};
//!   the longest timeout is about 4095 * 256 / 40 kHz ~ 26.2 s.
//! - Writing `DIV/RLD` requires unlocking first (CMD=0x5555); feeding = CMD=0xAAAA,
//!   start = CMD=0xCCCC; **once enabled it cannot be stopped** (it can only be
//!   reconfigured after a reset).
//!
//! ```ignore
//! let mut wdt = hal::wdg::IndependentWdt::start(p.WDT, 2_000); // 2s timeout
//! loop {
//!     wdt.feed();
//!     // ...
//! }
//! ```

use embassy_hal_internal::Peri;

use crate::pac;
use crate::time::Hertz;

/// Nominal LICK frequency (40 kHz; actually +/-50%, only used for coarse-grained
/// timeout estimation).
pub const LICK_FREQ: Hertz = Hertz(40_000);

/// Upper limit of the RLD register bit width.
const RLD_MAX: u64 = 0xFFF;

/// Independent watchdog handle: holds the singleton proof; `feed()` feeds the
/// watchdog.
pub struct IndependentWdt {
    _peri: Peri<'static, crate::peripherals::WDT>,
}

impl IndependentWdt {
    /// Starts the watchdog: automatically selects a divider (/4../256) so that
    /// the reload value fits within the 12-bit range.
    ///
    /// # Panics
    ///
    /// Panics if `timeout_ms` exceeds the reachable range (> ~26 s) or is 0.
    pub fn start(peri: Peri<'static, crate::peripherals::WDT>, timeout_ms: u32) -> Self {
        assert!(timeout_ms > 0, "WDT timeout must be > 0");
        let ticks = (LICK_FREQ.0 as u64) * (timeout_ms as u64) / 1_000;

        // Smallest divider so that RLD <= 0xFFF (a smaller divider gives higher resolution)
        const DIVS: [(u32, u8); 7] = [(4, 0), (8, 1), (16, 2), (32, 3), (64, 4), (128, 5), (256, 6)];
        let Some((div, div_code)) = DIVS
            .into_iter()
            .find(|(_, code)| ticks >> (code + 2) <= RLD_MAX)
        else {
            panic!("WDT timeout too long: max is about 26.2 s (4095 * 256 / 40 kHz)");
        };
        let rld = (ticks / div as u64).max(1);

        let wdt = unsafe { &*pac::WDT::PTR };
        // Unlock -> configure divider/reload -> wait for the write to take effect -> enable
        wdt.cmd().write(|w| w.cmd().unlock());
        wdt.div().write(|w| {
            match div_code {
                0 => w.div().div4(),
                1 => w.div().div8(),
                2 => w.div().div16(),
                3 => w.div().div32(),
                4 => w.div().div64(),
                5 => w.div().div128(),
                _ => w.div().div256(),
            };
            w
        });
        wdt.rld().write(|w| w.rld().set(rld as u16));
        while wdt.sts().read().divf().bit_is_set() || wdt.sts().read().rldf().bit_is_set() {}
        wdt.cmd().write(|w| w.cmd().enable());

        Self { _peri: peri }
    }

    /// Starts with an explicit divider and reload value (bypasses the automatic
    /// conversion; timeout = rld * div / 40 kHz).
    ///
    /// # Panics
    ///
    /// Panics if `rld` exceeds 12 bits.
    pub fn start_raw(
        peri: Peri<'static, crate::peripherals::WDT>,
        div_code: u8,
        rld: u16,
    ) -> Self {
        assert!(rld as u64 <= RLD_MAX, "RLD exceeds 12 bits");
        assert!(div_code <= 6, "DIV code must be 0..=6 (0x7 and 0x6 are both /256)");
        let wdt = unsafe { &*pac::WDT::PTR };
        wdt.cmd().write(|w| w.cmd().unlock());
        wdt.div().write(|w| {
            match div_code {
                0 => w.div().div4(),
                1 => w.div().div8(),
                2 => w.div().div16(),
                3 => w.div().div32(),
                4 => w.div().div64(),
                5 => w.div().div128(),
                _ => w.div().div256(),
            };
            w
        });
        wdt.rld().write(|w| w.rld().set(rld));
        while wdt.sts().read().divf().bit_is_set() || wdt.sts().read().rldf().bit_is_set() {}
        wdt.cmd().write(|w| w.cmd().enable());
        Self { _peri: peri }
    }

    /// Feeds the watchdog (CMD = 0xAAAA).
    pub fn feed(&mut self) {
        let wdt = unsafe { &*pac::WDT::PTR };
        wdt.cmd().write(|w| w.cmd().reload());
    }
}
