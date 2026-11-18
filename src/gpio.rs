//! GPIO: `Pin`/`AnyPin`/`Flex`/`Input`/`Output`/`OutputOpenDrain` (ch32-hal structure).
//!
//! AT32F435 pin model (differs from the old F1/F4 conventions; the PAC is
//! authoritative):
//! - Mode `IOMC` (CFGR, 2 bits per pin): Input=0 / Output=1 / Multiplexed=2 / Analog=3
//! - Output type `OMODE` (push-pull/open-drain), drive strength `ODRVR`
//!   (Normal/Large), pull-up/down `PULL`
//! - Alternate-function number `MUXL/MUXH` (4 bits per pin, matching the
//!   "alternate function table" mux index in the datasheet)
//! - Atomic set/clear via `SCR` (write-1-to-act, write-only register)
//!
//! **WARNING: Always use `modify()` on mode registers**: the reset value of
//! GPIOA_CFGR, 0xA800_0000, has its top 8 bits set by the multiplexed modes of
//! PA13(SWDIO)/PA14(SWCLK)/PA15(JTDI); a full-register `write()` would clear
//! them, and the firmware would disable SWD itself (observed on the evb board).

use core::convert::Infallible;

use embassy_hal_internal::{impl_peripheral, Peri, PeripheralType};

use crate::pac;

/// GPIO port base address (GPIOA @ 0x4002_0000, port stride 0x400).
const GPIO_PORT_BASE: usize = 0x4002_0000;
const GPIO_PORT_STRIDE: usize = 0x400;

type RegisterBlock = pac::gpioa::RegisterBlock;

#[inline]
fn block(port: u8) -> &'static RegisterBlock {
    unsafe { &*((GPIO_PORT_BASE + GPIO_PORT_STRIDE * port as usize) as *const _) }
}

/// Logic level.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Level {
    Low,
    High,
}

/// Pull-up/pull-down.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Pull {
    None,
    Up,
    Down,
}

/// Output "speed". AT32 has no separate slew-rate/speed register, so this maps
/// onto the ODRVR drive strength: Low/Medium -> Normal, High -> Large.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Speed {
    Low,
    Medium,
    High,
}

impl From<Speed> for Drive {
    fn from(s: Speed) -> Drive {
        match s {
            Speed::Low | Speed::Medium => Drive::Normal,
            Speed::High => Drive::Large,
        }
    }
}

/// Output type.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum OutputType {
    PushPull,
    OpenDrain,
}

/// ODRVR drive strength.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Drive {
    /// Normal drive capability.
    Normal,
    /// Large (high) drive capability.
    Large,
}

/// Alternate-function (AF) mode pin parameters: output type + drive strength + pull.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct AfType {
    output_type: OutputType,
    drive: Drive,
    pull: Pull,
}

impl AfType {
    /// Alternate-function output (TX/MOSI/PWM etc.).
    pub const fn output(output_type: OutputType, drive: Drive, pull: Pull) -> Self {
        Self {
            output_type,
            drive,
            pull,
        }
    }

    /// Alternate-function input (RX/MISO/capture etc.).
    pub const fn input(pull: Pull) -> Self {
        Self {
            output_type: OutputType::PushPull,
            drive: Drive::Normal,
            pull,
        }
    }
}

/// GPIO mode field written by the pin configuration methods.
#[derive(Copy, Clone)]
pub(crate) enum Iomc {
    Input,
    Output,
    Multiplexed,
    Analog,
}

/// Low-level pin register operations (shared within the crate; methods also
/// work on `AnyPin`).
pub(crate) trait SealedPin {
    /// Port number *16 + pin number (A=0..H=7).
    fn pin_port(&self) -> u8;

    #[inline]
    fn pin(&self) -> u8 {
        self.pin_port() & 0x0F
    }

    #[inline]
    fn port(&self) -> u8 {
        self.pin_port() >> 4
    }

    #[inline]
    fn block(&self) -> &'static RegisterBlock {
        block(self.port())
    }

    /// Write IOMC + PULL (always `modify`, preserving the AF bits of other
    /// pins such as SWD).
    #[inline]
    fn set_iomc(&self, iomc: Iomc, pull: Pull) {
        let rb = self.block();
        let n = self.pin();
        rb.cfgr().modify(|_, w| {
            match iomc {
                Iomc::Input => w.iomc(n).input(),
                Iomc::Output => w.iomc(n).output(),
                Iomc::Multiplexed => w.iomc(n).multiplexed(),
                Iomc::Analog => w.iomc(n).analog(),
            };
            w
        });
        rb.pull().modify(|_, w| {
            match pull {
                Pull::None => w.pull(n).no_pull(),
                Pull::Up => w.pull(n).pull_up(),
                Pull::Down => w.pull(n).pull_down(),
            };
            w
        });
    }

    #[inline]
    fn set_output_type(&self, ot: OutputType) {
        let n = self.pin();
        self.block().omode().modify(|_, w| {
            match ot {
                OutputType::PushPull => w.om(n).push_pull(),
                OutputType::OpenDrain => w.om(n).open_drain(),
            };
            w
        });
    }

    #[inline]
    fn set_drive(&self, drive: Drive) {
        let n = self.pin();
        self.block().odrvr().modify(|_, w| {
            match drive {
                Drive::Normal => w.odrv(n).normal(),
                Drive::Large => w.odrv(n).large(),
            };
            w
        });
    }

    /// Write the alternate-function number (MUXL covers pins 0-7, MUXH pins
    /// 8-15, 4 bits per pin).
    #[inline]
    fn set_mux(&self, af: u8) {
        let rb = self.block();
        let n = self.pin();
        assert!(af < 16, "alternate function number exceeds 4 bits");
        if n < 8 {
            rb.muxl().modify(|_, w| unsafe { w.mux(n).bits(af) });
        } else {
            rb.muxh().modify(|_, w| unsafe { w.mux(n - 8).bits(af) });
        }
    }

    /// Atomic set high (SCR.IOSB, write-1-to-act).
    #[inline]
    fn set_high(&self) {
        self.block().scr().write(|w| w.iosb(self.pin()).set_());
    }

    /// Atomic set low (SCR.IOCB, write-1-to-act).
    #[inline]
    fn set_low(&self) {
        self.block().scr().write(|w| w.iocb(self.pin()).clear());
    }

    /// Output latch state (ODT).
    #[inline]
    fn is_set_high(&self) -> bool {
        self.block().odt().read().odt(self.pin()).bit_is_set()
    }

    /// Pin input level (IDT).
    #[inline]
    fn is_high(&self) -> bool {
        self.block().idt().read().idt(self.pin()).bit_is_set()
    }

    /// Configure as output: write the level first, then switch the mode, to
    /// avoid a glitch during the transition.
    #[inline]
    fn set_as_output(&self, ot: OutputType, drive: Drive, level: Level, pull: Pull) {
        critical_section::with(|_| {
            match level {
                Level::High => self.set_high(),
                Level::Low => self.set_low(),
            }
            self.set_output_type(ot);
            self.set_drive(drive);
            self.set_iomc(Iomc::Output, pull);
        });
    }

    #[inline]
    fn set_as_input(&self, pull: Pull) {
        critical_section::with(|_| {
            self.set_iomc(Iomc::Input, pull);
        });
    }

    /// Configure as alternate function: write the AF number + output
    /// type/drive strength + IOMC.
    #[inline]
    fn set_as_af(&self, af: u8, af_type: AfType) {
        critical_section::with(|_| {
            self.set_mux(af);
            self.set_output_type(af_type.output_type);
            self.set_drive(af_type.drive);
            self.set_iomc(Iomc::Multiplexed, af_type.pull);
        });
    }

    /// Disconnect: analog state (no digital peripheral attached, no pull).
    #[inline]
    fn set_as_analog(&self) {
        critical_section::with(|_| {
            self.set_iomc(Iomc::Analog, Pull::None);
        });
    }
}

/// Pin trait (public): implemented by both singleton pin types and `AnyPin`.
#[allow(private_bounds)]
pub trait Pin: SealedPin + Into<AnyPin> + PeripheralType + Sized + 'static {
    /// The EXINT channel for this pin (line number = pin number).
    type ExtiChannel: crate::exti::Channel;

    /// Pin number (0-15).
    fn pin(&self) -> u8 {
        SealedPin::pin(self)
    }

    /// Port number (A=0..H=7).
    fn port(&self) -> u8 {
        SealedPin::port(self)
    }

    /// Degrade into a type-erased pin (handy for passing into tasks or storing
    /// across function boundaries).
    fn degrade(self) -> AnyPin {
        AnyPin {
            pin_port: SealedPin::pin_port(&self),
        }
    }
}

/// Type-erased pin (`pin_port` encodes port*16+pin).
pub struct AnyPin {
    pin_port: u8,
}

impl_peripheral!(AnyPin);

impl SealedPin for AnyPin {
    #[inline]
    fn pin_port(&self) -> u8 {
        self.pin_port
    }
}

// AnyPin is itself a Pin: drivers (Output::new etc.) can directly accept
// already-degraded pins
impl Pin for AnyPin {
    type ExtiChannel = crate::exti::AnyChannel;
}

/// Trait impls for all 128 pins (PA0..PH15).
macro_rules! impl_pin {
    ($pin_name:ident, $port:expr, $num:expr, $exti:ident) => {
        impl SealedPin for crate::peripherals::$pin_name {
            #[inline]
            fn pin_port(&self) -> u8 {
                $port * 16 + $num
            }
        }

        impl Pin for crate::peripherals::$pin_name {
            type ExtiChannel = crate::peripherals::$exti;
        }

        // Per-pin From impl (no blanket impl: it would conflict with the core
        // reflexive `From<T> for T`; AnyPin->AnyPin is already covered by that
        // reflexive impl)
        impl From<crate::peripherals::$pin_name> for AnyPin {
            #[inline]
            fn from(x: crate::peripherals::$pin_name) -> Self {
                x.degrade()
            }
        }
    };
}

crate::foreach_pin!(impl_pin);

// ---------------------------------------------------------------------------
// Runtime-mode drivers: Flex / Input / Output / OutputOpenDrain
// ---------------------------------------------------------------------------

/// Pin with switchable modes: change between input/output/analog at any time.
pub struct Flex<'d> {
    pin: Peri<'d, AnyPin>,
}

impl<'d> Flex<'d> {
    /// Take ownership of the pin (configured as a floating input).
    pub fn new(pin: Peri<'d, impl Pin>) -> Self {
        let pin: Peri<'d, AnyPin> = pin.into();
        pin.set_as_input(Pull::None);
        Self { pin }
    }

    pub fn set_as_input(&mut self, pull: Pull) {
        self.pin.set_as_input(pull);
    }

    pub fn set_as_output(&mut self, level: Level, speed: Speed) {
        self.pin
            .set_as_output(OutputType::PushPull, speed.into(), level, Pull::None);
    }

    pub fn set_as_output_open_drain(&mut self, level: Level, speed: Speed) {
        self.pin
            .set_as_output(OutputType::OpenDrain, speed.into(), level, Pull::None);
    }

    pub fn set_as_analog(&mut self) {
        self.pin.set_as_analog();
    }

    pub fn is_high(&self) -> bool {
        self.pin.is_high()
    }

    pub fn is_low(&self) -> bool {
        !self.is_high()
    }

    pub fn set_level(&mut self, level: Level) {
        match level {
            Level::High => self.pin.set_high(),
            Level::Low => self.pin.set_low(),
        }
    }

    pub fn set_high(&mut self) {
        self.pin.set_high();
    }

    pub fn set_low(&mut self) {
        self.pin.set_low();
    }

    pub fn is_set_high(&self) -> bool {
        self.pin.is_set_high()
    }

    pub fn toggle(&mut self) {
        if self.pin.is_set_high() {
            self.pin.set_low();
        } else {
            self.pin.set_high();
        }
    }
}

/// Input pin.
pub struct Input<'d> {
    pin: Peri<'d, AnyPin>,
}

impl<'d> Input<'d> {
    /// Take ownership of the pin and configure it as an input.
    pub fn new(pin: Peri<'d, impl Pin>, pull: Pull) -> Self {
        let pin: Peri<'d, AnyPin> = pin.into();
        pin.set_as_input(pull);
        Self { pin }
    }

    /// Pin number (crate-internal: ExtiInput needs it to line up with the
    /// EXINT line number).
    #[cfg(feature = "exti")]
    pub(crate) fn pin_no(&self) -> u8 {
        SealedPin::pin(&*self.pin)
    }

    /// Port number (crate-internal).
    #[cfg(feature = "exti")]
    pub(crate) fn port_no(&self) -> u8 {
        SealedPin::port(&*self.pin)
    }

    pub fn is_high(&self) -> bool {
        self.pin.is_high()
    }

    pub fn is_low(&self) -> bool {
        !self.is_high()
    }

    pub fn get_level(&self) -> Level {
        if self.is_high() {
            Level::High
        } else {
            Level::Low
        }
    }
}

/// Push-pull output pin.
pub struct Output<'d> {
    pin: Peri<'d, AnyPin>,
}

impl<'d> Output<'d> {
    /// Take ownership of the pin and configure it as a push-pull output,
    /// immediately driving `level` (level written before the mode switch, no
    /// glitch).
    pub fn new(pin: Peri<'d, impl Pin>, level: Level, speed: Speed) -> Self {
        let pin: Peri<'d, AnyPin> = pin.into();
        pin.set_as_output(OutputType::PushPull, speed.into(), level, Pull::None);
        Self { pin }
    }

    pub fn set_high(&mut self) {
        self.pin.set_high();
    }

    pub fn set_low(&mut self) {
        self.pin.set_low();
    }

    pub fn set_level(&mut self, level: Level) {
        match level {
            Level::High => self.pin.set_high(),
            Level::Low => self.pin.set_low(),
        }
    }

    pub fn is_set_high(&self) -> bool {
        self.pin.is_set_high()
    }

    pub fn is_set_low(&self) -> bool {
        !self.is_set_high()
    }

    pub fn toggle(&mut self) {
        if self.is_set_high() {
            self.set_low();
        } else {
            self.set_high();
        }
    }
}

/// Open-drain output pin (can also read the pin level back).
pub struct OutputOpenDrain<'d> {
    pin: Peri<'d, AnyPin>,
}

impl<'d> OutputOpenDrain<'d> {
    pub fn new(pin: Peri<'d, impl Pin>, level: Level, speed: Speed) -> Self {
        let pin: Peri<'d, AnyPin> = pin.into();
        pin.set_as_output(OutputType::OpenDrain, speed.into(), level, Pull::None);
        Self { pin }
    }

    pub fn set_high(&mut self) {
        self.pin.set_high();
    }

    pub fn set_low(&mut self) {
        self.pin.set_low();
    }

    pub fn set_level(&mut self, level: Level) {
        match level {
            Level::High => self.pin.set_high(),
            Level::Low => self.pin.set_low(),
        }
    }

    pub fn is_set_high(&self) -> bool {
        self.pin.is_set_high()
    }

    pub fn is_set_low(&self) -> bool {
        !self.is_set_high()
    }

    pub fn toggle(&mut self) {
        if self.is_set_high() {
            self.set_low();
        } else {
            self.set_high();
        }
    }

    pub fn is_high(&self) -> bool {
        self.pin.is_high()
    }

    pub fn is_low(&self) -> bool {
        !self.is_high()
    }
}

// ---------------------------------------------------------------------------
// embedded-hal 1.0 blocking trait impls
// ---------------------------------------------------------------------------

macro_rules! impl_eh_output {
    ($ty:ident) => {
        impl embedded_hal::digital::OutputPin for $ty<'_> {
            #[inline]
            fn set_low(&mut self) -> Result<(), Self::Error> {
                self.set_low();
                Ok(())
            }

            #[inline]
            fn set_high(&mut self) -> Result<(), Self::Error> {
                self.set_high();
                Ok(())
            }
        }

        impl embedded_hal::digital::StatefulOutputPin for $ty<'_> {
            #[inline]
            fn is_set_high(&mut self) -> Result<bool, Self::Error> {
                Ok($ty::is_set_high(self))
            }

            #[inline]
            fn is_set_low(&mut self) -> Result<bool, Self::Error> {
                Ok($ty::is_set_low(self))
            }

            #[inline]
            fn toggle(&mut self) -> Result<(), Self::Error> {
                $ty::toggle(self);
                Ok(())
            }
        }
    };
}

impl_eh_output!(Output);
impl_eh_output!(OutputOpenDrain);

macro_rules! impl_eh_input {
    ($ty:ident) => {
        impl embedded_hal::digital::InputPin for $ty<'_> {
            #[inline]
            fn is_high(&mut self) -> Result<bool, Self::Error> {
                Ok($ty::is_high(self))
            }

            #[inline]
            fn is_low(&mut self) -> Result<bool, Self::Error> {
                Ok($ty::is_low(self))
            }
        }
    };
}

impl_eh_input!(Input);
impl_eh_input!(OutputOpenDrain);
impl_eh_input!(Flex);

// ErrorType can only be implemented once per driver type (the macros above
// expand per trait)
impl embedded_hal::digital::ErrorType for Output<'_> {
    type Error = Infallible;
}
impl embedded_hal::digital::ErrorType for OutputOpenDrain<'_> {
    type Error = Infallible;
}
impl embedded_hal::digital::ErrorType for Input<'_> {
    type Error = Infallible;
}
impl embedded_hal::digital::ErrorType for Flex<'_> {
    type Error = Infallible;
}
