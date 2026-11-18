//! Simple PWM output (ch32-hal `timer/simple_pwm.rs` structure).
//!
//! ```ignore
//! use at32f435_hal as hal;
//! use hal::timer::{PwmPin, SimplePwm};
//! use hal::time::Hertz;
//!
//! let ch1 = PwmPin::new_ch1(p.PA0); // the AF number comes from the pin trait table
//! let mut pwm = SimplePwm::new(p.TMR2, Some(ch1), None, None, None, Hertz(1_000));
//! pwm.set_duty(hal::timer::Channel::Ch1, pwm.get_max_duty() / 2);
//! ```
//!
//! Output is fixed to PWM mode 1 (active level while CNT < CDT) with period
//! preload enabled; the polarity can be flipped with
//! [`SimplePwm::set_polarity`] (use [`Polarity::ActiveLow`] for loads such as
//! common-anode LEDs that light up when driven low).

use core::marker::PhantomData;

use embassy_hal_internal::Peri;

use super::{Channel, GeneralInstance, PwmRegs};
use crate::gpio::{AfType, AnyPin, OutputType, Pull, Speed};
use crate::time::Hertz;

/// CxOCTRL encoding of PWM mode 1 (output active while CNT < CDT).
const OCTRL_PWM1: u8 = 0b110;

/// PWM output polarity.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Polarity {
    /// High level is the active level (duty cycles map to a higher output).
    ActiveHigh,
    /// Low level is the active level (for loads lit at low level, such as
    /// common-anode LEDs).
    ActiveLow,
}

/// PWM channel pin: configuring the AF (push-pull, high speed) happens at
/// construction.
pub struct PwmPin<'d, T, C> {
    _pin: Peri<'d, AnyPin>,
    phantom: PhantomData<(T, C)>,
}

macro_rules! pwm_pin_channel {
    ($new_chx:ident, $channel:ident, $pin_trait:ident, $doc:expr) => {
        impl<'d, T: GeneralInstance> PwmPin<'d, T, $channel> {
            #[doc = $doc]
            pub fn $new_chx(pin: Peri<'d, impl crate::timer::$pin_trait<T>>) -> Self {
                pin.set_as_af(
                    pin.af_num(),
                    AfType::output(OutputType::PushPull, Speed::High.into(), Pull::None),
                );
                Self {
                    _pin: pin.into(),
                    phantom: PhantomData,
                }
            }
        }
    };
}

pwm_pin_channel!(
    new_ch1,
    Channel1,
    Channel1Pin,
    "Create a new channel 1 PWM pin instance."
);
pwm_pin_channel!(
    new_ch2,
    Channel2,
    Channel2Pin,
    "Create a new channel 2 PWM pin instance."
);
pwm_pin_channel!(
    new_ch3,
    Channel3,
    Channel3Pin,
    "Create a new channel 3 PWM pin instance."
);
pwm_pin_channel!(
    new_ch4,
    Channel4,
    Channel4Pin,
    "Create a new channel 4 PWM pin instance."
);

// Channel marker types (PwmPin generic parameters)
pub struct Channel1;
pub struct Channel2;
pub struct Channel3;
pub struct Channel4;

/// Simple PWM: owns a timer instance and a number of configured channels.
///
/// Construction enables and resets the timer, configures each given channel
/// for PWM mode 1, sets the frequency, and starts counting.
pub struct SimplePwm<'d, T: GeneralInstance> {
    #[allow(dead_code)]
    peri: Peri<'d, T>,
    /// Whether each channel was provided by the caller (determines the
    /// cen/enable set).
    channels: [bool; 4],
}

impl<'d, T: GeneralInstance> SimplePwm<'d, T> {
    /// Create the PWM: takes the timer singleton and up to 4 channel pins,
    /// and starts immediately at `freq`.
    pub fn new(
        peri: Peri<'d, T>,
        ch1: Option<PwmPin<'d, T, Channel1>>,
        ch2: Option<PwmPin<'d, T, Channel2>>,
        ch3: Option<PwmPin<'d, T, Channel3>>,
        ch4: Option<PwmPin<'d, T, Channel4>>,
        freq: Hertz,
    ) -> Self {
        T::enable_and_reset();
        let regs = T::regs();
        regs.set_tmren(false);
        // WARNING: TMR2/5 default to non-32-bit mode: PMEN must be set (official
        // library tmr_32_bit_function_enable) to use 32-bit counting/period,
        // otherwise PR is truncated by hardware (observed: masked to the low
        // 18 bits)
        regs.set_pmen(T::BITS == 32);
        regs.set_prben(true);

        let channels = [ch1.is_some(), ch2.is_some(), ch3.is_some(), ch4.is_some()];

        let mut this = Self { peri, channels };
        this.set_frequency(freq);

        for (i, &present) in channels.iter().enumerate() {
            if !present {
                continue;
            }
            let ch = Channel::Ch1;
            let ch = match i {
                1 => Channel::Ch2,
                2 => Channel::Ch3,
                3 => Channel::Ch4,
                _ => ch,
            };
            regs.set_pwm_channel(ch, OCTRL_PWM1);
            regs.set_cp(ch, false); // default active high
            regs.set_cen(ch, true);
        }
        regs.set_tmren(true);
        this
    }

    /// Set the PWM frequency (allocates the prescaler and period reload
    /// automatically; the OVF event latches the shadow registers).
    pub fn set_frequency(&mut self, freq: Hertz) {
        // Leave a representable compare value for 100% duty.
        let max_period = ((1u64 << T::BITS) - 1) as u32;
        let (prescaler, period) = super::frequency_dividers(T::frequency(), freq, max_period);
        let regs = T::regs();
        regs.set_tmren(false);
        regs.set_div(prescaler);
        regs.set_pr(period);
        regs.swevt_ovf(); // generate an update event to load the shadow registers
        regs.clear_ovf_flag(); // the software update event sets OVFIF; clear it to avoid false detection
        regs.set_tmren(true);
    }

    /// Current max duty cycle (= PR + 1).
    pub fn get_max_duty(&self) -> u32 {
        T::regs().read_pr() + 1
    }

    /// Set the channel duty cycle (0..=get_max_duty()).
    pub fn set_duty(&mut self, channel: Channel, duty: u32) {
        assert!(duty <= self.get_max_duty(), "duty cycle exceeds max_duty");
        T::regs().write_cdt(channel, duty);
    }

    /// Flip the channel output polarity (use [`Polarity::ActiveLow`] for
    /// loads lit at low level, e.g. common-anode).
    pub fn set_polarity(&mut self, channel: Channel, polarity: Polarity) {
        T::regs().set_cp(channel, polarity == Polarity::ActiveLow);
    }

    /// Enable the channel output (CEN).
    pub fn enable(&mut self, channel: Channel) {
        T::regs().set_cen(channel, true);
    }

    /// Disable the channel output (clearing CEN returns the pin to the
    /// inactive level).
    pub fn disable(&mut self, channel: Channel) {
        T::regs().set_cen(channel, false);
    }

    /// Whether the channel was provided in the constructor.
    pub fn has_channel(&self, channel: Channel) -> bool {
        self.channels[channel.index()]
    }
}
