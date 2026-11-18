//! Advanced timer driver: TMR1/TMR8/TMR20 (RM 14.4) - complementary PWM
//! outputs (CHx + CHxC) with dead-time insertion, the primary-output enable
//! (BRK.OEN, the AT32 equivalent of STM32's MOE), brake (software + input),
//! the repetition counter and the 16-bit time base.
//!
//! Hardware notes (RM 14.4.4.18, verified on the evb):
//! - Outputs gate on BRK.OEN in addition to CCTRL.CxEN/CxCEN; PWM never
//!   reaches the pins while OEN = 0.
//! - `DTC[7:0]` encodes the dead time against TDTS = timer clock (CLKDIV = 00):
//!   `0xx` -> `DTC[7:0]` * TDTS, `10x` -> (64 + `DTC[5:0]`) * 2 * TDTS,
//!   `110` -> (32 + `DTC[4:0]`) * 8 * TDTS, `111` -> (32 + `DTC[4:0]`) * 16 * TDTS.
//! - SWEVT.BRKSWTR triggers a brake event from software, so the brake path is
//!   testable without wiring the BRK input pin.
//! - WPC (write protect) is left at 0 so the brake/dead-time can be
//!   reconfigured; set it via `lock_config()` once tuning is done.
//! - CxOSEN (output channel switch) must stay 0 - see docs/pitfalls.md.

use embassy_hal_internal::Peri;

use crate::gpio::SealedPin;
use crate::pac;
use crate::peripheral::RccPeripheral;
use crate::time::Hertz;

/// Complementary output channel (CHx + CHxC pair).
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum ComplementaryChannel {
    Ch1,
    Ch2,
    Ch3,
}

impl ComplementaryChannel {
    /// Channel index (0..2).
    pub const fn index(self) -> usize {
        match self {
            ComplementaryChannel::Ch1 => 0,
            ComplementaryChannel::Ch2 => 1,
            ComplementaryChannel::Ch3 => 2,
        }
    }
}

/// Brake configuration (TMRx_BRK).
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
pub struct BrakeConfig {
    /// Brake input polarity (BRKV: false = active low, true = active high).
    pub active_high: bool,
    /// Outputs in the frozen state while the timer is disabled and OEN = 1
    /// (FCSOEN): true = output the idle level, false = keep the outputs off.
    pub idle_output_enable: bool,
    /// Outputs in the frozen state while OEN = 0 (FCSODIS): true = output the
    /// idle level, false = off.
    pub disable_output_enable: bool,
    /// Re-enable OEN automatically at the next overflow after a brake
    /// (AOEN).
    pub auto_recover: bool,
}

/// Advanced timer instance trait (TMR1/TMR8/TMR20, 16-bit counters).
#[allow(private_bounds)]
pub trait AdvancedInstance: RccPeripheral + embassy_hal_internal::PeripheralType + 'static {
    /// Register block.
    fn regs() -> &'static pac::tmr1::RegisterBlock;
}

macro_rules! impl_advanced_instance {
    ($inst:ident) => {
        impl AdvancedInstance for crate::peripherals::$inst {
            #[inline]
            fn regs() -> &'static pac::tmr1::RegisterBlock {
                unsafe { &*pac::$inst::PTR }
            }
        }
    };
}

impl_advanced_instance!(TMR1);
impl_advanced_instance!(TMR8);
impl_advanced_instance!(TMR20);

crate::pin_trait!(AdvancedBrkPin, AdvancedInstance);
crate::pin_trait!(AdvancedCh1Pin, AdvancedInstance);
crate::pin_trait!(AdvancedCh2Pin, AdvancedInstance);
crate::pin_trait!(AdvancedCh3Pin, AdvancedInstance);
crate::pin_trait!(AdvancedCh4Pin, AdvancedInstance);
crate::pin_trait!(AdvancedCh1CPin, AdvancedInstance);
crate::pin_trait!(AdvancedCh2CPin, AdvancedInstance);
crate::pin_trait!(AdvancedCh3CPin, AdvancedInstance);

// AF numbers verified against RM V2.09 tables 6-1..6-8.
crate::pin_trait_impl!(crate::timer::AdvancedBrkPin, TMR1, PA6, 1);
crate::pin_trait_impl!(crate::timer::AdvancedBrkPin, TMR1, PB12, 1);
crate::pin_trait_impl!(crate::timer::AdvancedBrkPin, TMR1, PE15, 1);
crate::pin_trait_impl!(crate::timer::AdvancedCh1Pin, TMR1, PA8, 1);
crate::pin_trait_impl!(crate::timer::AdvancedCh1Pin, TMR1, PE9, 1);
crate::pin_trait_impl!(crate::timer::AdvancedCh1CPin, TMR1, PA7, 1);
crate::pin_trait_impl!(crate::timer::AdvancedCh1CPin, TMR1, PE8, 1);
crate::pin_trait_impl!(crate::timer::AdvancedCh2Pin, TMR1, PA9, 1);
crate::pin_trait_impl!(crate::timer::AdvancedCh2Pin, TMR1, PE11, 1);
crate::pin_trait_impl!(crate::timer::AdvancedCh2CPin, TMR1, PB0, 1);
crate::pin_trait_impl!(crate::timer::AdvancedCh2CPin, TMR1, PE10, 1);
crate::pin_trait_impl!(crate::timer::AdvancedCh3Pin, TMR1, PA10, 1);
crate::pin_trait_impl!(crate::timer::AdvancedCh3Pin, TMR1, PE13, 1);
crate::pin_trait_impl!(crate::timer::AdvancedCh3CPin, TMR1, PB1, 1);
crate::pin_trait_impl!(crate::timer::AdvancedCh3CPin, TMR1, PE12, 1);
crate::pin_trait_impl!(crate::timer::AdvancedCh4Pin, TMR1, PA11, 1);
crate::pin_trait_impl!(crate::timer::AdvancedCh4Pin, TMR1, PE14, 1);

crate::pin_trait_impl!(crate::timer::AdvancedCh1Pin, TMR8, PC6, 2);
crate::pin_trait_impl!(crate::timer::AdvancedCh2Pin, TMR8, PC7, 2);
crate::pin_trait_impl!(crate::timer::AdvancedCh3Pin, TMR8, PC8, 2);
crate::pin_trait_impl!(crate::timer::AdvancedCh4Pin, TMR8, PC9, 2);
crate::pin_trait_impl!(crate::timer::AdvancedCh1CPin, TMR8, PA5, 3);
crate::pin_trait_impl!(crate::timer::AdvancedCh2CPin, TMR8, PA7, 3);

crate::pin_trait_impl!(crate::timer::AdvancedCh1Pin, TMR20, PB2, 2);
crate::pin_trait_impl!(crate::timer::AdvancedCh1Pin, TMR20, PF12, 2);
crate::pin_trait_impl!(crate::timer::AdvancedCh2Pin, TMR20, PC2, 2);
crate::pin_trait_impl!(crate::timer::AdvancedCh2Pin, TMR20, PE3, 2);
crate::pin_trait_impl!(crate::timer::AdvancedCh3Pin, TMR20, PB0, 2);
crate::pin_trait_impl!(crate::timer::AdvancedCh4Pin, TMR20, PF3, 2);
crate::pin_trait_impl!(crate::timer::AdvancedCh4Pin, TMR20, PE1, 2);

/// Complementary PWM with dead time, brake and repetition counter.
pub struct ComplementaryPwm<'d, T: AdvancedInstance> {
    _peri: Peri<'d, T>,
    _pins: [Option<Peri<'d, crate::gpio::AnyPin>>; 7],
    period: u32,
    dead_time_ticks: u32,
}

impl<'d, T: AdvancedInstance> ComplementaryPwm<'d, T> {
    /// Configures the timer for `update_freq` overflow frequency with the
    /// dead time and brake settings; channel pairs are added afterwards with
    /// `enable_ch1_pair`/`enable_ch2_pair`/`enable_ch3_pair`.
    ///
    /// `dead_time_ns` is inserted between each CHx rising edge and the
    /// corresponding CHxC rising edge. Call `enable_output()` (or rely on
    /// `brake.auto_recover`) before anything reaches the pins.
    ///
    /// # Panics
    ///
    /// Panics if the frequency/dead time cannot be represented by the 16-bit
    /// time base or the 8-bit DTC field.
    pub fn new(
        peri: Peri<'d, T>,
        update_freq: Hertz,
        dead_time_ns: u32,
        brake: BrakeConfig,
    ) -> Self {
        // Leave a representable compare value for 100% duty.
        let (divider, period) =
            super::frequency_dividers(T::frequency(), update_freq, u16::MAX.into());
        let (dtc, dead_ticks) =
            dead_time(T::frequency(), dead_time_ns).expect("dead time out of DTC range");
        T::enable_and_reset();
        let r = T::regs();

        r.div().write(|w| w.set(divider));
        r.pr().write(|w| w.set(period as u16));
        r.rpr().write(|w| w.set(0u8));
        // Load DIV/PR shadows once.
        r.swevt().write(|w| w.ovfswtr().overflow());

        // BRK register: write everything in one shot (WPC stays 0 so later
        // tuning is allowed).
        r.brk().modify(|_, w| {
            if brake.auto_recover {
                w.aoen().set_bit();
            }
            if brake.active_high {
                w.brkv().set_bit();
            } else {
                w.brkv().clear_bit();
            }
            if brake.idle_output_enable {
                w.fcsoen().set_bit();
            }
            if brake.disable_output_enable {
                w.fcsodis().set_bit();
            }
            unsafe { w.dtc().bits(dtc) };
            w.oen().clear_bit();
            w
        });

        r.ctrl1().modify(|_, w| w.tmren().set_bit());

        Self {
            _peri: peri,
            _pins: [None, None, None, None, None, None, None],
            period,
            dead_time_ticks: dead_ticks,
        }
    }

    /// Wires the brake input pin (PA6/PB12/PE15 at mux 1) with a pull so the
    /// idle level matches `BrakeConfig::active_high` (a floating brake pin
    /// with active-low polarity holds the brake asserted permanently).
    pub fn enable_brake_pin(
        &mut self,
        pin: Peri<'d, impl AdvancedBrkPin<T>>,
        pull: crate::gpio::Pull,
    ) -> &mut Self {
        let af = pin.af_num();
        let pin = Peri::into(pin);
        pin.set_as_af(af, crate::gpio::AfType::input(pull));
        self._pins[6] = Some(pin);
        self
    }

    /// Enables the CH1 + CH1C complementary pair.
    pub fn enable_ch1_pair(
        &mut self,
        main: Peri<'d, impl AdvancedCh1Pin<T>>,
        comp: Peri<'d, impl AdvancedCh1CPin<T>>,
    ) -> &mut Self {
        let (main_af, comp_af) = (main.af_num(), comp.af_num());
        let (main, comp) = (Peri::into(main), Peri::into(comp));
        main.set_as_af(main_af, AF_OUT);
        comp.set_as_af(comp_af, AF_OUT);
        self._pins[0] = Some(main);
        self._pins[1] = Some(comp);
        let r = T::regs();
        r.cm1_output().modify(|_, w| {
            w.c1c().output();
            unsafe { w.c1octrl().bits(0b110) };
            w.c1oben().set_bit();
            w
        });
        r.cctrl().modify(|_, w| {
            w.c1en().set_bit();
            w.c1cen().set_bit();
            w
        });
        self
    }

    /// Enables the CH2 + CH2C complementary pair.
    pub fn enable_ch2_pair(
        &mut self,
        main: Peri<'d, impl AdvancedCh2Pin<T>>,
        comp: Peri<'d, impl AdvancedCh2CPin<T>>,
    ) -> &mut Self {
        let (main_af, comp_af) = (main.af_num(), comp.af_num());
        let (main, comp) = (Peri::into(main), Peri::into(comp));
        main.set_as_af(main_af, AF_OUT);
        comp.set_as_af(comp_af, AF_OUT);
        self._pins[2] = Some(main);
        self._pins[3] = Some(comp);
        let r = T::regs();
        r.cm1_output().modify(|_, w| {
            w.c2c().output();
            unsafe { w.c2octrl().bits(0b110) };
            w.c2oben().set_bit();
            w
        });
        r.cctrl().modify(|_, w| {
            w.c2en().set_bit();
            w.c2cen().set_bit();
            w
        });
        self
    }

    /// Enables the CH3 + CH3C complementary pair.
    pub fn enable_ch3_pair(
        &mut self,
        main: Peri<'d, impl AdvancedCh3Pin<T>>,
        comp: Peri<'d, impl AdvancedCh3CPin<T>>,
    ) -> &mut Self {
        let (main_af, comp_af) = (main.af_num(), comp.af_num());
        let (main, comp) = (Peri::into(main), Peri::into(comp));
        main.set_as_af(main_af, AF_OUT);
        comp.set_as_af(comp_af, AF_OUT);
        self._pins[4] = Some(main);
        self._pins[5] = Some(comp);
        let r = T::regs();
        r.cm2_output().modify(|_, w| {
            w.c3c().output();
            unsafe { w.c3octrl().bits(0b110) };
            w.c3oben().set_bit();
            w
        });
        r.cctrl().modify(|_, w| {
            w.c3en().set_bit();
            w.c3cen().set_bit();
            w
        });
        self
    }

    /// The configured period (PR) in timer ticks.
    pub fn period(&self) -> u32 {
        self.period
    }

    /// The configured dead time in tDTS ticks (independent of DIV).
    pub fn dead_time_ticks(&self) -> u32 {
        self.dead_time_ticks
    }

    /// Sets the duty of a complementary channel (compare value in ticks).
    pub fn set_duty(&mut self, ch: ComplementaryChannel, duty: u32) {
        assert!(duty <= self.period + 1, "duty exceeds period");
        let r = T::regs();
        let duty = duty as u16;
        match ch {
            ComplementaryChannel::Ch1 => r.c1dt().write(|w| w.set(duty)),
            ComplementaryChannel::Ch2 => r.c2dt().write(|w| w.set(duty)),
            ComplementaryChannel::Ch3 => r.c3dt().write(|w| w.set(duty)),
        };
    }

    /// Reads back the compare value of a channel.
    pub fn duty(&self, ch: ComplementaryChannel) -> u32 {
        let r = T::regs();
        match ch {
            ComplementaryChannel::Ch1 => r.c1dt().read().bits() as u32,
            ComplementaryChannel::Ch2 => r.c2dt().read().bits() as u32,
            ComplementaryChannel::Ch3 => r.c3dt().read().bits() as u32,
        }
    }

    /// Updates the dead time (DTC re-encoding; requires WPC = 0).
    pub fn set_dead_time(&mut self, dead_time_ns: u32) {
        let r = T::regs();
        let (dtc, ticks) =
            dead_time(T::frequency(), dead_time_ns).expect("dead time out of DTC range");
        r.brk().modify(|_, w| unsafe { w.dtc().bits(dtc) });
        self.dead_time_ticks = ticks;
    }

    /// Sets the repetition counter: the overflow event fires only after
    /// `n + 1` counter periods (0 = every period).
    pub fn set_repetition(&mut self, n: u8) {
        T::regs().rpr().write(|w| w.set(n));
    }

    /// Master-mode TRGOUT source (CTRL2.PTOS): 0 = reset, 1 = enable,
    /// 2 = overflow event, 3 = compare pulse, 4..7 = C1..C4ORAW.
    /// The TRGOUT signal feeds the other timers' internal trigger inputs
    /// (ISx), enabling capture chains and slave-clock configurations.
    pub fn set_master_output(&mut self, sel: u8) {
        assert!(sel <= 7, "PTOS must be 0..=7");
        T::regs()
            .ctrl2()
            .modify(|_, w| unsafe { w.ptos().bits(sel) });
    }

    /// Enables the brake input with the given configuration.
    pub fn enable_brake_input(&mut self, cfg: BrakeConfig) {
        let r = T::regs();
        r.brk().modify(|_, w| {
            if cfg.active_high {
                w.brkv().set_bit();
            } else {
                w.brkv().clear_bit();
            }
            w.brken().set_bit();
            if cfg.auto_recover {
                w.aoen().set_bit();
            }
            if cfg.idle_output_enable {
                w.fcsoen().set_bit();
            }
            if cfg.disable_output_enable {
                w.fcsodis().set_bit();
            }
            w
        });
    }

    /// Triggers a brake event from software (SWEVT.BRKSWTR): with
    /// `BRKEN = 1` this clears OEN and shuts the outputs down; the flag and
    /// OEN state can be read back via `brake_flag()`/`output_enabled()`.
    pub fn trigger_brake_software(&mut self) {
        T::regs().swevt().write(|w| w.brkswtr().set_bit());
    }

    /// Whether the brake flag (ISTS.BRKIF) is pending.
    pub fn brake_flag(&self) -> bool {
        T::regs().ists().read().brkif().bit_is_set()
    }

    /// Clears the brake flag.
    pub fn clear_brake_flag(&self) {
        T::regs().ists().write(|w| w.brkif().clear());
    }

    /// Whether the primary output enable (BRK.OEN) is active.
    pub fn output_enabled(&self) -> bool {
        T::regs().brk().read().oen().bit_is_set()
    }

    /// Re-enables the primary output after a brake (BRK.OEN = 1); with
    /// `auto_recover` (AOEN) the hardware does this at the next overflow.
    pub fn enable_output(&mut self) {
        T::regs().brk().modify(|_, w| w.oen().set_bit());
    }

    /// Current counter value.
    pub fn now(&self) -> u16 {
        T::regs().cval().read().bits()
    }

    /// Write-protects the brake/dead-time configuration (BRK.WPC; only a
    /// system reset clears it). Level 3 protects DTC/AOEN/BRKEN/BRKV/OEN and
    /// CTRL2/CIOS; level 2 additionally CCTRL polarities + FCSO bits; level 1
    /// additionally the CxOCTRL/CxOBEN bits.
    pub fn lock_config(&mut self, level: u8) {
        assert!(level <= 3, "WPC level must be 0..=3");
        T::regs()
            .brk()
            .modify(|_, w| unsafe { w.wpc().bits(level) });
    }

    /// Whether the overflow flag is pending.
    pub fn overflow_flag(&self) -> bool {
        T::regs().ists().read().ovfif().bit_is_set()
    }

    /// Clears the overflow flag.
    pub fn clear_overflow_flag(&self) {
        T::regs().ists().write(|w| w.ovfif().clear());
    }
}

/// AF output pin configuration shared by all advanced-timer pins (Large
/// drive = Speed::High).
const AF_OUT: crate::gpio::AfType = crate::gpio::AfType::output(
    crate::gpio::OutputType::PushPull,
    crate::gpio::Drive::Large,
    crate::gpio::Pull::None,
);

// CLKDIV is left at reset (tDTS = tCK_INT). DIV prescales the counter
// without affecting dead time. Round up to avoid undershooting a requested
// minimum dead time at the gaps between DTC encoding ranges (RM 14.4.4).
fn dead_time(clock: Hertz, ns: u32) -> Option<(u8, u32)> {
    let ticks = (u64::from(clock.0) * u64::from(ns)).div_ceil(1_000_000_000);
    let (code, actual) = match ticks {
        0..=127 => (ticks, ticks),
        128..=254 => (0x80 + ticks.div_ceil(2) - 64, ticks.div_ceil(2) * 2),
        255..=504 => (0xC0 + ticks.div_ceil(8) - 32, ticks.div_ceil(8) * 8),
        505..=1008 => (0xE0 + ticks.div_ceil(16) - 32, ticks.div_ceil(16) * 16),
        _ => return None,
    };
    Some((code as u8, actual as u32))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dead_time_is_rounded_up_to_the_next_encodable_value() {
        let clock = Hertz(250_000_000);
        assert_eq!(dead_time(clock, 1), Some((1, 1)));
        assert_eq!(dead_time(clock, 508), Some((127, 127)));
        assert_eq!(dead_time(clock, 1017), Some((0xc0, 256)));
        assert_eq!(dead_time(clock, 2017), Some((0xe0, 512)));
        assert_eq!(dead_time(clock, 4033), None);
    }
}
