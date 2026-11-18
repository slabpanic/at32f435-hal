//! IRTMR infrared interface (RM 23): hardware-ORed IR output composed of a
//! low-frequency envelope (TMR10_C1OUT) and a high-frequency carrier
//! (TMR11_C1OUT), polarity-selected by SCFG_CFG1.IR_POL and routed to
//! PA13 (mux 1) or PB9 (mux 0).
//!
//! There is no dedicated IRTMR register block on this family - the driver
//! configures the two timers' CH1 PWM outputs plus the SCFG polarity bit:
//!
//! ```ignore
//! // 38 kHz carrier, 1 kHz envelope, 1/3 duty bursts:
//! let ir = hal::irtmr::IrTmr::new(
//!     p.TMR10, p.TMR11, p.PB9,
//!     hal::irtmr::ChannelPwm::new(Hertz::hz(1_000), 1, 3),
//!     hal::irtmr::ChannelPwm::new(Hertz::hz(38_000), 1, 2),
//! );
//! ir.set_polarity(false);
//! ```
//!
//! WARNING: PA13 is SWDIO - wiring IR onto PA13 unhooks SWD exactly like the
//! GPIO pitfall in docs/pitfalls.md; prefer PB9.

use embassy_hal_internal::Peri;

use crate::gpio::{AfType, Drive, OutputType, Pull};
use crate::pac;
use crate::peripheral::RccPeripheral;
use crate::time::Hertz;

/// One IR channel configuration: a CH1 PWM output (PWM mode A) at `freq`
/// with `num/den` duty.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct ChannelPwm {
    pub freq: Hertz,
    pub duty_num: u32,
    pub duty_den: u32,
}

impl ChannelPwm {
    /// PWM at `freq` with duty = num/den (0..=1).
    pub const fn new(freq: Hertz, num: u32, den: u32) -> Self {
        Self {
            freq,
            duty_num: num,
            duty_den: den,
        }
    }
}

/// IR output pin.
pub trait IrOutPin: crate::gpio::Pin {
    /// The alternate-function number for IR_OUT on this pin.
    fn ir_af_num(&self) -> u8;
}

impl IrOutPin for crate::peripherals::PB9 {
    #[inline]
    fn ir_af_num(&self) -> u8 {
        0
    }
}

impl IrOutPin for crate::peripherals::PA13 {
    #[inline]
    fn ir_af_num(&self) -> u8 {
        1
    }
}

/// IR output handle: owns the envelope timer, the carrier timer and the pin.
pub struct IrTmr<'d> {
    _env: Peri<'d, crate::peripherals::TMR10>,
    _carrier: Peri<'d, crate::peripherals::TMR11>,
    _pin: Peri<'d, crate::gpio::AnyPin>,
}

impl<'d> IrTmr<'d> {
    /// Configures both timers' CH1 as PWM mode A outputs, wires the pin to
    /// IR_OUT and starts both counters.
    pub fn new(
        env: Peri<'d, crate::peripherals::TMR10>,
        carrier: Peri<'d, crate::peripherals::TMR11>,
        pin: Peri<'d, impl IrOutPin>,
        envelope: ChannelPwm,
        carrier_pwm: ChannelPwm,
    ) -> Self {
        <crate::peripherals::TMR10 as crate::peripheral::ClockControl>::enable_and_reset();
        <crate::peripherals::TMR11 as crate::peripheral::ClockControl>::enable_and_reset();
        config_channel::<crate::peripherals::TMR10>(&envelope);
        config_channel::<crate::peripherals::TMR11>(&carrier_pwm);

        pin.set_as_af(
            pin.ir_af_num(),
            AfType::output(OutputType::PushPull, Drive::Large, Pull::None),
        );

        // SCFG clock is enabled by hal::init; keep the polarity at reset.
        Self {
            _env: env,
            _carrier: carrier,
            _pin: pin.into(),
        }
    }

    /// IR output polarity (SCFG_CFG1.IR_POL: false = not inverted).
    pub fn set_polarity(&self, inverted: bool) {
        let scfg = unsafe { &*pac::SCFG::PTR };
        scfg.cfg1().modify(|_, w| {
            if inverted {
                w.ir_pol().set_bit();
            } else {
                w.ir_pol().clear_bit();
            }
            w
        });
    }

    /// Whether the polarity bit is set.
    pub fn polarity_inverted(&self) -> bool {
        let scfg = unsafe { &*pac::SCFG::PTR };
        scfg.cfg1().read().ir_pol().bit_is_set()
    }

    /// Reconfigures the envelope PWM (TMR10 CH1).
    pub fn set_envelope(&mut self, cfg: ChannelPwm) {
        config_channel::<crate::peripherals::TMR10>(&cfg);
    }

    /// Reconfigures the carrier PWM (TMR11 CH1).
    pub fn set_carrier(&mut self, cfg: ChannelPwm) {
        config_channel::<crate::peripherals::TMR11>(&cfg);
    }

    /// Stops both timers (IR output goes idle).
    pub fn stop(&mut self) {
        <crate::peripherals::TMR10 as crate::peripheral::ClockControl>::disable();
        <crate::peripherals::TMR11 as crate::peripheral::ClockControl>::disable();
    }
}

/// Register-block accessor for the IR channel timers (both have the same
/// 1-channel layout).
trait IrChannelRegs {
    fn regs() -> &'static pac::tmr10::RegisterBlock;
}

impl IrChannelRegs for crate::peripherals::TMR10 {
    #[inline]
    fn regs() -> &'static pac::tmr10::RegisterBlock {
        unsafe { &*pac::TMR10::PTR }
    }
}

impl IrChannelRegs for crate::peripherals::TMR11 {
    #[inline]
    fn regs() -> &'static pac::tmr10::RegisterBlock {
        unsafe { &*pac::TMR11::PTR }
    }
}

/// Configures one IR channel timer: CH1 = PWM mode A output with the
/// requested frequency/duty, counter running.
fn config_channel<T: IrChannelRegs + RccPeripheral>(cfg: &ChannelPwm) {
    let r = <T as IrChannelRegs>::regs();
    assert!(
        cfg.duty_den > 0 && cfg.duty_num <= cfg.duty_den,
        "duty out of range"
    );
    let clk = T::frequency().0;
    let total = (clk + cfg.freq.0 / 2) / cfg.freq.0.max(1);
    let div1 = total.div_ceil(65536).clamp(1, 65536);
    let pr1 = (total / div1).clamp(1, 65536);
    r.div().write(|w| w.set((div1 - 1) as u16));
    r.pr().write(|w| w.set((pr1 - 1) as u16));
    let duty = ((pr1 as u64) * cfg.duty_num as u64 / cfg.duty_den as u64) as u16;
    r.c1dt().write(|w| w.set(duty));
    // PWM mode A (C1OCTRL = 110) + compare preload; OSEN untouched (pitfall).
    r.cm1_output().modify(|_, w| {
        w.c1c().output();
        unsafe { w.c1octrl().bits(0b110) };
        w.c1oben().set_bit();
        w
    });
    r.cctrl().modify(|_, w| {
        w.c1en().set_bit();
        w
    });
    r.ctrl1().modify(|_, w| w.tmren().set_bit());
    // Kick the shadow registers.
    r.swevt().write(|w| w.ovfswtr().overflow());
}
