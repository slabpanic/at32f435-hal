//! Window watchdog WWDT (PCLK1 clock domain, RM 15).
//!
//! A 7-bit down counter clocked at PCLK1/4096/2^DIV. A system reset occurs when
//! either:
//! - the counter drops from 0x40 to 0x3F (timeout), or
//! - the counter is reloaded while it is still above the window value WIN.
//!
//! Refresh is therefore only legal while `0x40 <= CNT < WIN`; the legal window
//! opens `window_timeout` after the last reload and closes `timeout` after it.
//! Once enabled, WWDT cannot be stopped (only a system reset clears it).
//!
//! Timeout = T_PCLK1 * 4096 * 2^DIV * (`CNT[5:0]` + 1) seconds; the counter is
//! loaded with RLD (0x40..=0x7F, bit 6 always kept 1 - loading `CNT[6]`=0 into
//! CTRL while WWDTEN is being set resets the chip immediately, per the RM).
//!
//! ```ignore
//! let mut wwdt = hal::wwdt::WindowWdt::start(p.WWDT, 2_000, 500)?;
//! // refresh is allowed 0.5-2 s after each feed; outside that window it resets
//! loop {
//!     wwdt.feed();
//!     ...
//! }
//! ```

use embassy_hal_internal::Peri;

use crate::pac;
use crate::peripheral::RccPeripheral;

/// WWDT counter reload/counter range: bit 6 must be 1.
const CNT_MIN: u8 = 0x40;
const CNT_MAX: u8 = 0x7F;

/// Window configuration error.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[non_exhaustive]
pub enum WwdtError {
    /// Timeout too short: the PCLK1/4096/2^DIV counter granularity cannot reach
    /// the requested duration even with `CNT[5:0]`+1 = 1.
    TimeoutTooShort,
    /// Timeout too long: exceeds the reachable maximum (0x40 tick * 64 steps at
    /// the slowest divider).
    TimeoutTooLong,
    /// The window duration must be strictly smaller than the timeout (a window
    /// >= timeout would make every refresh illegal).
    WindowNotSmallerThanTimeout,
}

/// Window watchdog handle.
pub struct WindowWdt {
    /// Reload value written on `feed()` (kept so feed() can restore it exactly).
    rld: u8,
    _peri: Peri<'static, crate::peripherals::WWDT>,
}

impl WindowWdt {
    /// Starts the window watchdog: picks the divider automatically and computes
    /// the reload count from `timeout_ms`, then the window value from
    /// `window_ms` (refresh becomes legal `window_ms` after each feed).
    ///
    /// `window_ms` must be smaller than `timeout_ms`; `window_ms = 0` is allowed
    /// and disables the window check (WIN = 0x7F, refresh legal any time before
    /// the timeout).
    ///
    /// The WWDT clock starts at PCLK1/4096 (the APB1 clock is enabled by
    /// `RccPeripheral` before this runs).
    pub fn start(
        peri: Peri<'static, crate::peripherals::WWDT>,
        timeout_ms: u32,
        window_ms: u32,
    ) -> Result<Self, WwdtError> {
        assert!(timeout_ms > 0, "WWDT timeout must be > 0");
        assert!(
            window_ms < timeout_ms,
            "WWDT window must be smaller than the timeout"
        );

        let pclk1_hz = <crate::peripherals::WWDT as RccPeripheral>::frequency().0;
        // One counter tick (CNT step) = 4096 * 2^DIV / pclk1, in ns.
        // Find the smallest DIV so that the tick * 64 (CNT[5:0] range) covers
        // the timeout: i.e. tick >= timeout/64.
        const DIVS: [u32; 4] = [4_096, 8_192, 16_384, 32_768];
        // Timeout in ns (u64 math: 4096 * 64 * 1e9 / 144e6 ~ 1.8e12 fits).
        let timeout_ns = (timeout_ms as u64) * 1_000_000;
        let mut div_code = 0u8;
        let mut tick_ns = 0u64;
        let mut found = false;
        for (i, div) in DIVS.into_iter().enumerate() {
            tick_ns = (div as u64) * 1_000_000_000 / pclk1_hz as u64;
            // 64 counter steps max (CNT[5:0]+1 <= 64)
            if tick_ns * 64 >= timeout_ns {
                div_code = i as u8;
                found = true;
                break;
            }
        }
        if !found {
            return Err(WwdtError::TimeoutTooLong);
        }
        if timeout_ns < tick_ns {
            return Err(WwdtError::TimeoutTooShort);
        }
        // CNT[5:0] = steps - 1 (the counter resets when it leaves 0x40, so the
        // last tick brings 0x41 -> 0x40 and the timeout fires one tick later)
        let steps = timeout_ns / tick_ns;
        let low = (steps.max(1) - 1).min(0x3F) as u8;
        let rld = CNT_MIN | low;

        // Window: counter value below which refresh is legal. The counter
        // counts down from RLD to 0x40; refresh is illegal while CNT >= WIN.
        // window_ns / tick gives the number of full ticks elapsed at which the
        // refresh window opens: WIN = CNT_at_that_point = RLD - ticks.
        let win = if window_ms == 0 {
            CNT_MAX
        } else {
            let window_ns = (window_ms as u64) * 1_000_000;
            let ticks = window_ns / tick_ns + 1; // open slightly late: safer to refresh early? No -
                                                 // opening late makes the legal window smaller, which is the safe
                                                 // side (refreshing too early resets, refreshing late is fine).
            let win = (rld as u64).saturating_sub(ticks).max(CNT_MIN as u64) as u8;
            win.min(CNT_MAX)
        };

        Ok(Self::start_raw(peri, div_code, rld, win))
    }

    /// Starts with explicit register values.
    ///
    /// - `div_code`: 0..=3 -> PCLK1 / 4096 / 8192 / 16384 / 32768.
    /// - `rld`: reload value, 0x40..=0x7F (bit 6 forced to 1).
    /// - `win`: window value 0x40..=0x7F; refresh legal while CNT < WIN.
    ///   Use 0x7F for "no window" (refresh legal until the timeout).
    pub fn start_raw(
        peri: Peri<'static, crate::peripherals::WWDT>,
        div_code: u8,
        rld: u8,
        win: u8,
    ) -> Self {
        assert!(div_code <= 3, "WWDT DIV code must be 0..=3");
        let rld = rld | CNT_MIN;
        let win = win.max(CNT_MIN);
        assert!(
            (CNT_MIN..=CNT_MAX).contains(&rld),
            "WWDT reload must be 0x40..=0x7F"
        );

        <crate::peripherals::WWDT as crate::peripheral::ClockControl>::enable_and_reset();
        let wwdt = unsafe { &*pac::WWDT::PTR };
        // CFG: divider + window (+ reload interrupt if ever needed). These are
        // rw1s/rw fields; writing once here is enough.
        wwdt.cfg().modify(|_, w| {
            match div_code {
                0 => w.div().div4096(),
                1 => w.div().div8192(),
                2 => w.div().div16384(),
                _ => w.div().div32768(),
            };
            unsafe { w.win().bits(win) };
            w
        });
        // CTRL: load the counter and enable. Bit 6 (CNT[6]) stays 1: setting
        // WWDTEN while CNT[6] = 0 resets the chip immediately (RM 15.3).
        unsafe { wwdt.ctrl().write(|w| w.bits((rld as u32) | 0x80)) };

        Self { rld, _peri: peri }
    }

    /// Feeds the watchdog: reloads the counter with the configured reload value.
    ///
    /// Only legal while the counter is inside the window (`CNT[6:0] < WIN` and
    /// still >= 0x40); refreshing outside the window triggers a system reset.
    pub fn feed(&mut self) {
        let wwdt = unsafe { &*pac::WWDT::PTR };
        // WWDTEN is rw1s: keep writing 1; CNT[6] stays 1.
        unsafe { wwdt.ctrl().write(|w| w.bits((self.rld as u32) | 0x80)) };
    }

    /// Current counter value (7 bits, counts down).
    pub fn counter(&self) -> u8 {
        let wwdt = unsafe { &*pac::WWDT::PTR };
        (wwdt.ctrl().read().bits() & 0x7F) as u8
    }

    /// Reload-counter interrupt flag (set when the counter reaches 0x40):
    /// with `RLDIEN` this is the "refresh now" interrupt just before the reset.
    pub fn reload_flag(&self) -> bool {
        let wwdt = unsafe { &*pac::WWDT::PTR };
        wwdt.sts().read().rldf().bit_is_set()
    }

    /// Clears the reload-counter interrupt flag.
    pub fn clear_reload_flag(&self) {
        let wwdt = unsafe { &*pac::WWDT::PTR };
        wwdt.sts().write(|w| w.rldf().clear());
    }
}
