//! Timers (general-purpose 4-channel TMR2/3/4/5): instance abstraction + PWM
//! (ch32-hal `timer/` structure).
//!
//! - [`GeneralInstance`]: timer instance trait; `PwmRegs` abstracts over the
//!   width difference between the TMR2/5 (32-bit) and TMR3/4 (16-bit)
//!   register blocks.
//! - [`SimplePwm`] / [`PwmPin`]: PWM output
//!   (`PwmPin::new_ch1(p.PA0)` -> `SimplePwm::new(p.TMR2, Some(ch1), .., freq)`).
//!
//! - [`TimerBase`]: TMR6/7 time base (overflow IRQ + async wait)
//! - [`InputCapture`] / [`OnePulse`] / [`Qei`]:
//!   capture, single-pulse and encoder counting on TMR2-5
//! - [`ComplementaryPwm`]: TMR1/8/20 complementary outputs with
//!   dead time, brake (software + input) and the repetition counter
//!
//! PWM also supports the one- and two-channel TMR9 through TMR14.
//! The infrared timer is in [`crate::irtmr`].

mod advanced;
mod base;
mod general;
mod low_level;
mod simple_pwm;

use crate::peripheral::RccPeripheral;
use low_level::PwmRegs;

pub use advanced::{
    AdvancedBrkPin, AdvancedCh1CPin, AdvancedCh1Pin, AdvancedCh2CPin, AdvancedCh2Pin,
    AdvancedCh3CPin, AdvancedCh3Pin, AdvancedCh4Pin, AdvancedInstance, BrakeConfig,
    ComplementaryChannel, ComplementaryPwm,
};
pub use base::{InterruptHandler as BaseInterruptHandler, TimerBase};
pub use general::{
    CaptureConfig, CaptureError, CaptureInstance, Edge, EventDivider, Filter, InputCapture,
    InterruptHandler as GeneralInterruptHandler, OnePulse, Qei,
};
pub use simple_pwm::{Polarity, PwmPin, SimplePwm};

/// Timer channel.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Channel {
    Ch1,
    Ch2,
    Ch3,
    Ch4,
}

impl Channel {
    /// Channel index (0..3).
    #[inline]
    pub fn index(&self) -> usize {
        match self {
            Channel::Ch1 => 0,
            Channel::Ch2 => 1,
            Channel::Ch3 => 2,
            Channel::Ch4 => 3,
        }
    }
}

/// General-purpose timer instance usable for PWM.
#[allow(private_bounds)]
pub trait GeneralInstance: RccPeripheral + embassy_hal_internal::PeripheralType + 'static {
    /// Register-block handle.
    type Regs: PwmRegs;

    /// Register-block pointer.
    fn regs() -> &'static Self::Regs;

    /// Counter bit width (32 = TMR2/5, 16 = TMR3/4).
    const BITS: u8;
}

macro_rules! impl_general_timer {
    ($inst:ident, $block:ident, $bits:expr) => {
        impl GeneralInstance for crate::peripherals::$inst {
            type Regs = crate::pac::$block::RegisterBlock;

            #[inline]
            fn regs() -> &'static Self::Regs {
                unsafe { &*crate::pac::$inst::PTR }
            }

            const BITS: u8 = $bits;
        }
    };
}

#[cfg(not(all(feature = "embassy", feature = "time-driver-tim2")))]
impl_general_timer!(TMR2, tmr2, 32);
impl_general_timer!(TMR5, tmr2, 32);
impl_general_timer!(TMR3, tmr3, 16);
impl_general_timer!(TMR4, tmr3, 16);
impl_general_timer!(TMR9, tmr9, 16);
impl_general_timer!(TMR12, tmr9, 16);
impl_general_timer!(TMR10, tmr10, 16);
impl_general_timer!(TMR11, tmr10, 16);
impl_general_timer!(TMR13, tmr10, 16);
impl_general_timer!(TMR14, tmr10, 16);

crate::pin_trait!(Channel1Pin, GeneralInstance);
crate::pin_trait!(Channel2Pin, GeneralInstance);
crate::pin_trait!(Channel3Pin, GeneralInstance);
crate::pin_trait!(Channel4Pin, GeneralInstance);

// AF numbers: mux1 = TMR1/TMR2 family, mux2 = TMR3/4/5 family
// ([V] checked against Artery's official examples; [I] = DS function table +
// STM32 family-pattern inference)
#[cfg(not(all(feature = "embassy", feature = "time-driver-tim2")))]
crate::pin_trait_impl!(crate::timer::Channel1Pin, TMR2, PA0, 1); // [V] encoder_tmr2 example
#[cfg(not(all(feature = "embassy", feature = "time-driver-tim2")))]
crate::pin_trait_impl!(crate::timer::Channel1Pin, TMR2, PA15, 1); // [I]
#[cfg(not(all(feature = "embassy", feature = "time-driver-tim2")))]
crate::pin_trait_impl!(crate::timer::Channel2Pin, TMR2, PA1, 1); // [V]
#[cfg(not(all(feature = "embassy", feature = "time-driver-tim2")))]
crate::pin_trait_impl!(crate::timer::Channel2Pin, TMR2, PB3, 1); // [V] DS function table
#[cfg(not(all(feature = "embassy", feature = "time-driver-tim2")))]
crate::pin_trait_impl!(crate::timer::Channel3Pin, TMR2, PA2, 1); // [I]
#[cfg(not(all(feature = "embassy", feature = "time-driver-tim2")))]
crate::pin_trait_impl!(crate::timer::Channel3Pin, TMR2, PB10, 1); // [I]
#[cfg(not(all(feature = "embassy", feature = "time-driver-tim2")))]
crate::pin_trait_impl!(crate::timer::Channel4Pin, TMR2, PA3, 1); // [I]
#[cfg(not(all(feature = "embassy", feature = "time-driver-tim2")))]
crate::pin_trait_impl!(crate::timer::Channel4Pin, TMR2, PB11, 1); // [I]

crate::pin_trait_impl!(crate::timer::Channel1Pin, TMR3, PA6, 2); // [V] pwm_output_tmr3 example
crate::pin_trait_impl!(crate::timer::Channel1Pin, TMR3, PB4, 2); // [I]
crate::pin_trait_impl!(crate::timer::Channel2Pin, TMR3, PA7, 2); // [V]
crate::pin_trait_impl!(crate::timer::Channel2Pin, TMR3, PB5, 2); // [I]
crate::pin_trait_impl!(crate::timer::Channel3Pin, TMR3, PB0, 2); // [V]
crate::pin_trait_impl!(crate::timer::Channel3Pin, TMR3, PC8, 2); // [I]
crate::pin_trait_impl!(crate::timer::Channel4Pin, TMR3, PB1, 2); // [V]
crate::pin_trait_impl!(crate::timer::Channel4Pin, TMR3, PC9, 2); // [I]

crate::pin_trait_impl!(crate::timer::Channel1Pin, TMR4, PB6, 2); // [I]
crate::pin_trait_impl!(crate::timer::Channel1Pin, TMR4, PD12, 2); // [I]
crate::pin_trait_impl!(crate::timer::Channel2Pin, TMR4, PB7, 2); // [I]
crate::pin_trait_impl!(crate::timer::Channel2Pin, TMR4, PD13, 2); // [I]
crate::pin_trait_impl!(crate::timer::Channel3Pin, TMR4, PB8, 2); // [I]
crate::pin_trait_impl!(crate::timer::Channel3Pin, TMR4, PD14, 2); // [I]
crate::pin_trait_impl!(crate::timer::Channel4Pin, TMR4, PB9, 2); // [I]
crate::pin_trait_impl!(crate::timer::Channel4Pin, TMR4, PD15, 2); // [I]

crate::pin_trait_impl!(crate::timer::Channel1Pin, TMR5, PA0, 2); // [I] DS function table
crate::pin_trait_impl!(crate::timer::Channel2Pin, TMR5, PA1, 2); // [I] DS function table
crate::pin_trait_impl!(crate::timer::Channel3Pin, TMR5, PA2, 2); // [I] DS function table
crate::pin_trait_impl!(crate::timer::Channel4Pin, TMR5, PA3, 2); // [I] DS function table

// Small timers (2-ch TMR9/12, 1-ch TMR10/11/13/14); mux from RM V2.09 table
// 6-1/6-2 page-image read ([V] 2026-11-17): TMR9 = mux3, TMR10/11 = mux3,
// TMR12 = mux9, TMR13/14 = mux9
crate::pin_trait_impl!(crate::timer::Channel1Pin, TMR9, PA2, 3); // [V] p114
crate::pin_trait_impl!(crate::timer::Channel2Pin, TMR9, PA3, 3); // [V] p114
crate::pin_trait_impl!(crate::timer::Channel1Pin, TMR12, PB14, 9); // [V] p117
crate::pin_trait_impl!(crate::timer::Channel2Pin, TMR12, PB15, 9); // [V] p117
crate::pin_trait_impl!(crate::timer::Channel1Pin, TMR10, PB8, 3); // [V] p116
crate::pin_trait_impl!(crate::timer::Channel1Pin, TMR11, PB9, 3); // [V] p116
crate::pin_trait_impl!(crate::timer::Channel1Pin, TMR13, PA6, 9); // [V] p115
crate::pin_trait_impl!(crate::timer::Channel1Pin, TMR14, PA7, 9); // [V] p115

// Frequency configuration shared by the up-counting time base and PWM
// drivers. Choose the smallest prescaler that fits PR to retain resolution.
fn frequency_dividers(
    clock: crate::time::Hertz,
    frequency: crate::time::Hertz,
    max_period: u32,
) -> (u16, u32) {
    assert!(frequency.0 > 0, "timer frequency must be > 0");
    assert!(
        frequency.0 <= clock.0 / 2,
        "timer period must contain at least two ticks"
    );
    let ticks = clock.0.div_ceil(frequency.0);
    let prescaler = ticks.div_ceil(max_period);
    let div = u16::try_from(prescaler - 1).expect("timer prescaler exceeds 16 bits");
    let period = ticks.div_ceil(prescaler);
    (div, period - 1)
}
