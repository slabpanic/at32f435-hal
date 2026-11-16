//! Timers (general-purpose 4-channel TMR2/3/4/5): instance abstraction + PWM
//! (ch32-hal `timer/` structure).
//!
//! - [`GeneralInstance`]: timer instance trait; `PwmRegs` abstracts over the
//!   width difference between the TMR2/5 (32-bit) and TMR3/4 (16-bit)
//!   register blocks.
//! - [`simple_pwm::SimplePwm`] / [`simple_pwm::PwmPin`]: PWM output
//!   (`PwmPin::new_ch1(p.PA0)` -> `SimplePwm::new(p.TMR2, Some(ch1), .., freq)`).
//!
//! The advanced timers TMR1/8/20 (which need the POEN master-output enable)
//! and the 2-channel timers TMR9..14 are not covered yet.

mod simple_pwm;

use crate::peripheral::RccPeripheral;

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

/// Register-block access abstraction: smooths over the width difference
/// between TMR2/5 (32-bit) and TMR3/4 (16-bit).
///
/// All methods are thin register-level wrappers; the driver layer only deals
/// with u32/u16 semantic interfaces.
pub(crate) trait PwmRegs {
    /// Prescaler (DIV, writing 0 means no division).
    fn set_div(&self, v: u16);
    /// Period reload value (PR).
    fn set_pr(&self, v: u32);
    /// Read back the period reload value.
    fn read_pr(&self) -> u32;
    /// Write the channel compare value (CDT).
    fn write_cdt(&self, ch: Channel, v: u32);
    /// Software-generate the overflow event (loads the shadow registers,
    /// equivalent to STM32's UG).
    fn swevt_ovf(&self);
    /// Counter enable (CTRL1.TMREN).
    fn set_tmren(&self, en: bool);
    /// Period preload enable (CTRL1.PRBEN).
    fn set_prben(&self, en: bool);
    /// Configure the channel for PWM output: direction=output, output control
    /// = PWM mode (0b110=PWM1 / 0b111=PWM2), compare preload (OBEN) enabled.
    ///
    /// WARNING: OSEN (output channel switch) must stay at its reset value 0.
    /// Setting it disconnects the channel output entirely (all three channels
    /// went dead on hardware; the official library never sets it either).
    fn set_pwm_channel(&self, ch: Channel, octrl_bits: u8);
    /// Channel enable (CCTRL.CEN).
    fn set_cen(&self, ch: Channel, en: bool);
    /// Channel polarity (CCTRL.CP: false = active high, true = active low).
    fn set_cp(&self, ch: Channel, active_low: bool);
    /// 32-bit counting mode (CTRL1.PMEN, only present on TMR2/5; TMR3/4 have
    /// no such bit, empty impl).
    fn set_pmen(&self, en: bool);
    /// Clear the overflow flag (gets set after a software-generated update event).
    fn clear_ovf_flag(&self);
}

impl PwmRegs for crate::pac::tmr2::RegisterBlock {
    #[inline]
    fn set_pmen(&self, en: bool) {
        self.ctrl1().modify(|_, w| {
            if en {
                w.pmen().set_bit();
            } else {
                w.pmen().clear_bit();
            }
            w
        });
    }

    #[inline]
    fn clear_ovf_flag(&self) {
        self.ists().write(|w| w.ovfif().clear());
    }

    #[inline]
    fn set_div(&self, v: u16) {
        self.div().write(|w| w.set(v));
    }

    #[inline]
    fn set_pr(&self, v: u32) {
        self.pr().write(|w| w.set(v));
    }

    #[inline]
    fn read_pr(&self) -> u32 {
        self.pr().read().bits()
    }

    #[inline]
    fn write_cdt(&self, ch: Channel, v: u32) {
        match ch {
            Channel::Ch1 => self.c1dt().write(|w| w.set(v)),
            Channel::Ch2 => self.c2dt().write(|w| w.set(v)),
            Channel::Ch3 => self.c3dt().write(|w| w.set(v)),
            Channel::Ch4 => self.c4dt().write(|w| w.set(v)),
        };
    }

    #[inline]
    fn swevt_ovf(&self) {
        self.swevt().write(|w| w.ovfswtr().overflow());
    }

    #[inline]
    fn set_tmren(&self, en: bool) {
        self.ctrl1().modify(|_, w| {
            if en {
                w.tmren().enable();
            } else {
                w.tmren().disable();
            }
            w
        });
    }

    #[inline]
    fn set_prben(&self, en: bool) {
        self.ctrl1().modify(|_, w| {
            if en {
                w.prben().enable();
            } else {
                w.prben().disable();
            }
            w
        });
    }

    #[inline]
    fn set_pwm_channel(&self, ch: Channel, octrl_bits: u8) {
        match ch {
            Channel::Ch1 => self.cm1_output().modify(|_, w| {
                w.c1c().output();
                unsafe {
                    w.c1octrl().bits(octrl_bits);
                }
                w.c1oben().enable();
                w
            }),
            Channel::Ch2 => self.cm1_output().modify(|_, w| {
                w.c2c().output();
                unsafe {
                    w.c2octrl().bits(octrl_bits);
                }
                w.c2oben().enable();
                w
            }),
            Channel::Ch3 => self.cm2_output().modify(|_, w| {
                w.c3c().output();
                unsafe {
                    w.c3octrl().bits(octrl_bits);
                }
                w.c3oben().enable();
                w
            }),
            Channel::Ch4 => self.cm2_output().modify(|_, w| {
                w.c4c().output();
                unsafe {
                    w.c4octrl().bits(octrl_bits);
                }
                w.c4oben().enable();
                w
            }),
        };
    }

    #[inline]
    fn set_cen(&self, ch: Channel, en: bool) {
        let n = ch.index() as u8;
        self.cctrl().modify(|_, w| {
            if en {
                w.cen(n).enable();
            } else {
                w.cen(n).disable();
            }
            w
        });
    }

    #[inline]
    fn set_cp(&self, ch: Channel, active_low: bool) {
        let n = ch.index() as u8;
        self.cctrl().modify(|_, w| {
            if active_low {
                w.cp(n).set_bit();
            } else {
                w.cp(n).clear_bit();
            }
            w
        });
    }
}

impl PwmRegs for crate::pac::tmr3::RegisterBlock {
    // 16-bit register block: same shape as above; the width truncates at the
    // write site (the driver layer guarantees the value range)

    #[inline]
    fn set_pmen(&self, _en: bool) {
        // TMR3/4 have no PMEN bit (always 16-bit counting)
    }

    #[inline]
    fn clear_ovf_flag(&self) {
        self.ists().write(|w| w.ovfif().clear());
    }

    #[inline]
    fn set_div(&self, v: u16) {
        self.div().write(|w| w.set(v));
    }

    #[inline]
    fn set_pr(&self, v: u32) {
        self.pr().write(|w| w.set(v as u16));
    }

    #[inline]
    fn read_pr(&self) -> u32 {
        self.pr().read().bits() as u32
    }

    #[inline]
    fn write_cdt(&self, ch: Channel, v: u32) {
        match ch {
            Channel::Ch1 => self.c1dt().write(|w| w.set(v as u16)),
            Channel::Ch2 => self.c2dt().write(|w| w.set(v as u16)),
            Channel::Ch3 => self.c3dt().write(|w| w.set(v as u16)),
            Channel::Ch4 => self.c4dt().write(|w| w.set(v as u16)),
        };
    }

    #[inline]
    fn swevt_ovf(&self) {
        self.swevt().write(|w| w.ovfswtr().overflow());
    }

    #[inline]
    fn set_tmren(&self, en: bool) {
        self.ctrl1().modify(|_, w| {
            if en {
                w.tmren().enable();
            } else {
                w.tmren().disable();
            }
            w
        });
    }

    #[inline]
    fn set_prben(&self, en: bool) {
        self.ctrl1().modify(|_, w| {
            if en {
                w.prben().enable();
            } else {
                w.prben().disable();
            }
            w
        });
    }

    #[inline]
    fn set_pwm_channel(&self, ch: Channel, octrl_bits: u8) {
        match ch {
            Channel::Ch1 => self.cm1_output().modify(|_, w| {
                w.c1c().output();
                unsafe {
                    w.c1octrl().bits(octrl_bits);
                }
                w.c1oben().enable();
                w
            }),
            Channel::Ch2 => self.cm1_output().modify(|_, w| {
                w.c2c().output();
                unsafe {
                    w.c2octrl().bits(octrl_bits);
                }
                w.c2oben().enable();
                w
            }),
            Channel::Ch3 => self.cm2_output().modify(|_, w| {
                w.c3c().output();
                unsafe {
                    w.c3octrl().bits(octrl_bits);
                }
                w.c3oben().enable();
                w
            }),
            Channel::Ch4 => self.cm2_output().modify(|_, w| {
                w.c4c().output();
                unsafe {
                    w.c4octrl().bits(octrl_bits);
                }
                w.c4oben().enable();
                w
            }),
        };
    }

    #[inline]
    fn set_cen(&self, ch: Channel, en: bool) {
        let n = ch.index() as u8;
        self.cctrl().modify(|_, w| {
            if en {
                w.cen(n).enable();
            } else {
                w.cen(n).disable();
            }
            w
        });
    }

    #[inline]
    fn set_cp(&self, ch: Channel, active_low: bool) {
        let n = ch.index() as u8;
        self.cctrl().modify(|_, w| {
            if active_low {
                w.cp(n).set_bit();
            } else {
                w.cp(n).clear_bit();
            }
            w
        });
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

impl_general_timer!(TMR2, tmr2, 32);
impl_general_timer!(TMR5, tmr2, 32);
impl_general_timer!(TMR3, tmr3, 16);
impl_general_timer!(TMR4, tmr3, 16);

crate::pin_trait!(Channel1Pin, GeneralInstance);
crate::pin_trait!(Channel2Pin, GeneralInstance);
crate::pin_trait!(Channel3Pin, GeneralInstance);
crate::pin_trait!(Channel4Pin, GeneralInstance);

// AF numbers: mux1 = TMR1/TMR2 family, mux2 = TMR3/4/5 family
// ([V] checked against Artery's official examples; [I] = DS function table +
// STM32 family-pattern inference)
crate::pin_trait_impl!(crate::timer::Channel1Pin, TMR2, PA0, 1); // [V] encoder_tmr2 example
crate::pin_trait_impl!(crate::timer::Channel1Pin, TMR2, PA15, 1); // [I]
crate::pin_trait_impl!(crate::timer::Channel2Pin, TMR2, PA1, 1); // [V]
crate::pin_trait_impl!(crate::timer::Channel2Pin, TMR2, PB3, 1); // [V] DS function table
crate::pin_trait_impl!(crate::timer::Channel3Pin, TMR2, PA2, 1); // [I]
crate::pin_trait_impl!(crate::timer::Channel3Pin, TMR2, PB10, 1); // [I]
crate::pin_trait_impl!(crate::timer::Channel4Pin, TMR2, PA3, 1); // [I]
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
