use super::Channel;

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

impl PwmRegs for crate::pac::tmr9::RegisterBlock {
    // 2-channel timers TMR9/12: same control shape as the 4-channel blocks,
    // but only C1DT/C2DT and one CM1_OUTPUT register exist. Ch3/Ch4 are
    // compile-time unused (SimplePwm ch3/ch4 stay None) and their helpers are
    // no-ops.

    #[inline]
    fn set_pmen(&self, _en: bool) {
        // TMR9/12 have no PMEN bit (always 16-bit counting)
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
            Channel::Ch1 => {
                self.cdt(0).write(|w| w.set(v as u16));
            }
            Channel::Ch2 => {
                self.cdt(1).write(|w| w.set(v as u16));
            }
            _ => {}
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
            Channel::Ch1 => {
                self.cm1_output().modify(|_, w| {
                    // C1C: 0 = output mode (no enum variant on this block)
                    unsafe {
                        w.c1c().bits(0);
                        w.c1octrl().bits(octrl_bits);
                    }
                    w.c1oben().enable();
                    w
                });
            }
            Channel::Ch2 => {
                self.cm1_output().modify(|_, w| {
                    unsafe {
                        w.c2c().bits(0);
                        w.c2octrl().bits(octrl_bits);
                    }
                    w.c2oben().enable();
                    w
                });
            }
            _ => {}
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

impl PwmRegs for crate::pac::tmr10::RegisterBlock {
    // 1-channel timers TMR10/11/13/14: only C1DT and the C1 fields.

    #[inline]
    fn set_pmen(&self, _en: bool) {
        // No PMEN bit (always 16-bit counting)
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
        if ch == Channel::Ch1 {
            self.cdt(0).write(|w| w.set(v as u16));
        }
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
        if ch == Channel::Ch1 {
            self.cm1_output().modify(|_, w| {
                unsafe {
                    w.c1c().bits(0);
                    w.c1octrl().bits(octrl_bits);
                }
                w.c1oben().enable()
            });
        }
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
