//! DEBUG module (RM 30): low-power debug clock control and debug-halt
//! peripheral freezing.
//!
//! The DEBUG block sits at 0xE004_2000 (Cortex-M4 private peripheral region),
//! is never clock-gated, and its registers are reset by PORESET only (they
//! survive a system reset).
//!
//! Two register groups:
//!
//! - `DEBUG_CTRL`: keeps clocks running in low-power modes so that a debugger
//!   (and RTT) stays alive while the MCU sleeps (RM 30.4.2). With
//!   `sleep_debug` set, Sleep mode keeps all clocks running; with
//!   `deepsleep_debug`, Deepsleep mode keeps the system clock fed from HICK
//!   (the application must reconfigure the clocks on wake, which
//!   [`crate::rcc`] does automatically on its step-up path); with
//!   `standby_debug`, Standby keeps the 1.2 V digital domain powered from
//!   HICK.
//! - `DEBUG_APB1_PAUSE` / `DEBUG_APB2_PAUSE`: freeze individual peripherals'
//!   clocks while the core is halted in debug, so timers/watchdogs/CAN do not
//!   run away under the debugger (RM 30.4.3 / 30.4.4).
//!
//! Note: the write restrictions only apply "while the core is in reset" -
//! from software these registers are plain read-write.
//!
//! Usage:
//! ```ignore
//! let dbg = hal::debug::Debug::new(p.DEBUG);
//! dbg.set_low_power_debug(true, true, false); // keep RTT alive in sleep/deepsleep
//! dbg.set_apb1_pause(Apb1Pause::new().with_wdt(true).with_wwdt(true));
//! ```

use embassy_hal_internal::Peri;

use crate::pac;

/// Handle to the DEBUG block (single instance, not clock-gated).
pub struct Debug {
    _peri: Peri<'static, crate::peripherals::DEBUG>,
}

impl Debug {
    /// Take the DEBUG singleton. There is no RCC gate to enable.
    pub fn new(peri: Peri<'static, crate::peripherals::DEBUG>) -> Self {
        Self { _peri: peri }
    }

    fn block(&self) -> &'static pac::debug::RegisterBlock {
        unsafe { &*pac::DEBUG::PTR }
    }

    /// Raw IDCODE (RM 30.4.1): revision in the upper halfword, device ID in
    /// the lower one (e.g. 0x7008_4552 on AT32F437VMT7).
    pub fn idcode(&self) -> u32 {
        self.block().idcode().read().bits()
    }

    /// Device ID (lower halfword of [`Debug::idcode`]).
    pub fn device_id(&self) -> u16 {
        self.block().idcode().read().bits() as u16
    }

    /// Revision ID (upper halfword of [`Debug::idcode`]).
    pub fn revision_id(&self) -> u16 {
        (self.block().idcode().read().bits() >> 16) as u16
    }

    /// Low-power debug control (RM 30.4.2). Each flag keeps the corresponding
    /// mode clocked so a debugger / RTT stays alive; wake-up from Deepsleep
    /// with `deepsleep_debug` leaves the system clock on HICK, which the
    /// application must reconfigure (the HAL's automatic step-up path does).
    pub fn set_low_power_debug(&mut self, sleep: bool, deepsleep: bool, standby: bool) {
        self.block().ctrl().modify(|_, w| {
            w.sleep_debug().bit(sleep);
            w.deepsleep_debug().bit(deepsleep);
            w.standby_debug().bit(standby);
            w
        });
    }

    /// Read back the low-power debug control bits (sleep, deepsleep, standby).
    pub fn low_power_debug(&self) -> (bool, bool, bool) {
        let r = self.block().ctrl().read();
        (
            r.sleep_debug().bit_is_set(),
            r.deepsleep_debug().bit_is_set(),
            r.standby_debug().bit_is_set(),
        )
    }

    /// Freeze the selected APB1 peripherals while the core is halted.
    pub fn set_apb1_pause(&mut self, flags: Apb1Pause) {
        self.block()
            .apb1_pause()
            .write(|w| unsafe { w.bits(flags.0) });
    }

    /// Read back the APB1 pause word.
    pub fn apb1_pause(&self) -> Apb1Pause {
        Apb1Pause(self.block().apb1_pause().read().bits())
    }

    /// Freeze the selected APB2 peripherals while the core is halted.
    pub fn set_apb2_pause(&mut self, flags: Apb2Pause) {
        self.block()
            .apb2_pause()
            .write(|w| unsafe { w.bits(flags.0) });
    }

    /// Read back the APB2 pause word.
    pub fn apb2_pause(&self) -> Apb2Pause {
        Apb2Pause(self.block().apb2_pause().read().bits())
    }
}

/// APB1 debug-halt freeze flags (RM 30.4.3). A set bit pauses that
/// peripheral's clock while the core is halted in debug.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct Apb1Pause(u32);

impl Apb1Pause {
    /// No peripherals paused (reset value).
    pub const fn new() -> Self {
        Self(0)
    }

    /// Pause all timers and watchdogs (everything except the ERTC 512 Hz
    /// output and the I2C SMBus timeouts).
    pub const fn timers_and_wdts() -> Self {
        Self(
            (1 << 0)
                | (1 << 1)
                | (1 << 2)
                | (1 << 3)
                | (1 << 4)
                | (1 << 5)
                | (1 << 6)
                | (1 << 7)
                | (1 << 8)
                | (1 << 10)
                | (1 << 11)
                | (1 << 12)
                | (1 << 25)
                | (1 << 26),
        )
    }

    /// A `|`-style builder: returns a copy with `tmr2` set/unset.
    pub const fn with_tmr2(mut self, on: bool) -> Self {
        self.0 = set_bit_u32(self.0, 0, on);
        self
    }

    pub const fn with_tmr3(mut self, on: bool) -> Self {
        self.0 = set_bit_u32(self.0, 1, on);
        self
    }

    pub const fn with_tmr4(mut self, on: bool) -> Self {
        self.0 = set_bit_u32(self.0, 2, on);
        self
    }

    pub const fn with_tmr5(mut self, on: bool) -> Self {
        self.0 = set_bit_u32(self.0, 3, on);
        self
    }

    pub const fn with_tmr6(mut self, on: bool) -> Self {
        self.0 = set_bit_u32(self.0, 4, on);
        self
    }

    pub const fn with_tmr7(mut self, on: bool) -> Self {
        self.0 = set_bit_u32(self.0, 5, on);
        self
    }

    pub const fn with_tmr12(mut self, on: bool) -> Self {
        self.0 = set_bit_u32(self.0, 6, on);
        self
    }

    pub const fn with_tmr13(mut self, on: bool) -> Self {
        self.0 = set_bit_u32(self.0, 7, on);
        self
    }

    pub const fn with_tmr14(mut self, on: bool) -> Self {
        self.0 = set_bit_u32(self.0, 8, on);
        self
    }

    /// Pause the ERTC calendar while halted.
    pub const fn with_ertc(mut self, on: bool) -> Self {
        self.0 = set_bit_u32(self.0, 10, on);
        self
    }

    /// Pause the window watchdog while halted.
    pub const fn with_wwdt(mut self, on: bool) -> Self {
        self.0 = set_bit_u32(self.0, 11, on);
        self
    }

    /// Pause the independent watchdog while halted.
    pub const fn with_wdt(mut self, on: bool) -> Self {
        self.0 = set_bit_u32(self.0, 12, on);
        self
    }

    /// Freeze the ERTC 512 Hz output clock while halted.
    pub const fn with_ertc512(mut self, on: bool) -> Self {
        self.0 = set_bit_u32(self.0, 15, on);
        self
    }

    /// Stop the I2C1 SMBus timeout counter while halted.
    pub const fn with_i2c1_smbus_timeout(mut self, on: bool) -> Self {
        self.0 = set_bit_u32(self.0, 24, on);
        self
    }

    /// Stop the I2C2 SMBus timeout counter while halted.
    pub const fn with_i2c2_smbus_timeout(mut self, on: bool) -> Self {
        self.0 = set_bit_u32(self.0, 27, on);
        self
    }

    /// Stop the I2C3 SMBus timeout counter while halted.
    pub const fn with_i2c3_smbus_timeout(mut self, on: bool) -> Self {
        self.0 = set_bit_u32(self.0, 28, on);
        self
    }

    /// Stop CAN1 reception while halted.
    pub const fn with_can1(mut self, on: bool) -> Self {
        self.0 = set_bit_u32(self.0, 25, on);
        self
    }

    /// Stop CAN2 reception while halted.
    pub const fn with_can2(mut self, on: bool) -> Self {
        self.0 = set_bit_u32(self.0, 26, on);
        self
    }

    /// Raw register word (for read-back round trips).
    pub const fn bits(self) -> u32 {
        self.0
    }

    /// Build from a raw register word.
    pub const fn from_bits(bits: u32) -> Self {
        Self(bits)
    }
}

impl core::ops::BitOr for Apb1Pause {
    type Output = Self;

    fn bitor(self, rhs: Self) -> Self {
        Self(self.0 | rhs.0)
    }
}

/// APB2 debug-halt freeze flags (RM 30.4.4): the advanced/generic timers
/// TMR1/8/20 and TMR9/10/11.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct Apb2Pause(u32);

impl Apb2Pause {
    /// No peripherals paused (reset value).
    pub const fn new() -> Self {
        Self(0)
    }

    /// Pause all APB2 timers.
    pub const fn timers() -> Self {
        Self((1 << 0) | (1 << 1) | (1 << 6) | (1 << 16) | (1 << 17) | (1 << 18))
    }

    pub const fn with_tmr1(mut self, on: bool) -> Self {
        self.0 = set_bit_u32(self.0, 0, on);
        self
    }

    pub const fn with_tmr8(mut self, on: bool) -> Self {
        self.0 = set_bit_u32(self.0, 1, on);
        self
    }

    pub const fn with_tmr20(mut self, on: bool) -> Self {
        self.0 = set_bit_u32(self.0, 6, on);
        self
    }

    pub const fn with_tmr9(mut self, on: bool) -> Self {
        self.0 = set_bit_u32(self.0, 16, on);
        self
    }

    pub const fn with_tmr10(mut self, on: bool) -> Self {
        self.0 = set_bit_u32(self.0, 17, on);
        self
    }

    pub const fn with_tmr11(mut self, on: bool) -> Self {
        self.0 = set_bit_u32(self.0, 18, on);
        self
    }

    /// Raw register word (for read-back round trips).
    pub const fn bits(self) -> u32 {
        self.0
    }

    /// Build from a raw register word.
    pub const fn from_bits(bits: u32) -> Self {
        Self(bits)
    }
}

impl core::ops::BitOr for Apb2Pause {
    type Output = Self;

    fn bitor(self, rhs: Self) -> Self {
        Self(self.0 | rhs.0)
    }
}

const fn set_bit_u32(word: u32, bit: u8, on: bool) -> u32 {
    if on {
        word | (1 << bit)
    } else {
        word & !(1 << bit)
    }
}
