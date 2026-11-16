//! Basic frequency/time types (a minimal implementation, the same as ch32-hal's
//! `time.rs`).

/// Frequency, in Hz.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Ord, PartialOrd)]
#[repr(transparent)]
pub struct Hertz(pub u32);

impl Hertz {
    /// Creates a frequency value from Hz.
    pub const fn hz(hz: u32) -> Self {
        Self(hz)
    }

    /// Creates a frequency value from kHz.
    pub const fn khz(khz: u32) -> Self {
        Self(khz * 1_000)
    }

    /// Creates a frequency value from MHz.
    pub const fn mhz(mhz: u32) -> Self {
        Self(mhz * 1_000_000)
    }
}

impl From<Hertz> for u32 {
    fn from(hz: Hertz) -> Self {
        hz.0
    }
}

impl core::ops::Mul<u32> for Hertz {
    type Output = Hertz;
    fn mul(self, rhs: u32) -> Hertz {
        Hertz(self.0 * rhs)
    }
}

impl core::ops::Div<u32> for Hertz {
    type Output = Hertz;
    fn div(self, rhs: u32) -> Hertz {
        Hertz(self.0 / rhs)
    }
}
