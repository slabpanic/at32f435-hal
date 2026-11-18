//! ADC driver: ADC1/2/3 + the ADCCOM common block (RM 18, ch32-hal `adc/`
//! structure).
//!
//! The AT32F435 ADC uses ordinary/preempted channel groups (not the STM32
//! regular/injected naming). This driver covers the ordinary group:
//!
//! - [`Adc::blocking_read`]: single-channel blocking conversion (polling).
//! - [`Adc::blocking_read_sequence`]: multi-channel sequence (SQEN + OSQ +
//!   software trigger; up to 16 channels).
//! - [`Adc::blocking_read_vts`]/[`Adc::blocking_read_vintrv`]/[`Adc::blocking_read_vbat`]:
//!   the ADC1-internal channels (temperature sensor, 1.2 V reference, VBAT/4).
//! - [`Adc::scan_dma`]: async sequence scan with the results DMA'd to a
//!   buffer (one DMA request per conversion; needs the channel IRQ binding).
//! - [`Adc::blocking_read_preempted`]: the preempted (injected) group, up to
//!   4 channels with per-slot data offsets (PCDTOx).
//! - [`Adc::enable_oversampling`]: hardware oversampler (2x..256x + shift).
//! - [`Adc::set_voltage_monitor`]: out-of-range detection (VMHB/VMLB).
//! - [`Adc::set_ordinary_trigger`]/[`Adc::set_preempted_trigger`]: hardware
//!   triggers (timer TRGOUT/CH events, EXINT lines 11/15).
//!
//! Clock: ADCCLK = HCLK/(ADCDIV+2), shared by all three ADCs, capped at
//! 80 MHz and not above PCLK2. Power-up sequence (RM 18.4.2.1): set ADCDIV ->
//! ADCEN -> wait RDY -> calibrate at 12-bit -> set the final resolution.
//! EOCSFEN is set permanently so OCCE marks every ordinary conversion
//! (per-conversion polling in sequence mode + active overrun detection).
//!
//! ```ignore
//! let mut adc = hal::adc::Adc::new(p.ADC1, hal::adc::Config::default());
//! let v = adc.blocking_read(p.PA4.reborrow())?;   // ADC12_IN4
//! let temp_c = adc.blocking_read_vts(hal::adc::SampleTime::Cycles247_5);
//! ```

mod pins;

use core::marker::PhantomData;

use embassy_hal_internal::{drop::OnDrop, Peri};

use crate::dma::{Channel, Config as DmaConfig, DmaError, Width};
// (the ADC channel enum below is AdcChannel to avoid clashing with the DMA Channel)
use crate::gpio::Pin;
use crate::pac;
use crate::peripheral::RccPeripheral;
use crate::rcc;
use crate::time::Hertz;

/// Conversion resolution (CTRL1.CRSEL).
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
pub enum Resolution {
    /// 12-bit (also the only resolution calibratable).
    #[default]
    Bits12,
    /// 10-bit.
    Bits10,
    /// 8-bit.
    Bits8,
    /// 6-bit.
    Bits6,
}

impl Resolution {
    const fn code(self) -> u8 {
        match self {
            Resolution::Bits12 => 0,
            Resolution::Bits10 => 1,
            Resolution::Bits8 => 2,
            Resolution::Bits6 => 3,
        }
    }

    /// Full-scale count.
    pub const fn full_scale(self) -> u32 {
        match self {
            Resolution::Bits12 => 4095,
            Resolution::Bits10 => 1023,
            Resolution::Bits8 => 255,
            Resolution::Bits6 => 63,
        }
    }
}

/// Per-channel sample time (SPT1/SPT2.CSPTx, 3 bits per channel).
#[derive(Copy, Clone, Debug, Eq, PartialEq, PartialOrd, Ord, Default)]
pub enum SampleTime {
    /// 2.5 cycles (fast channels 0/1/10/11/12/13 only).
    #[default]
    Cycles2_5,
    /// 6.5 cycles.
    Cycles6_5,
    /// 12.5 cycles.
    Cycles12_5,
    /// 24.5 cycles.
    Cycles24_5,
    /// 47.5 cycles.
    Cycles47_5,
    /// 92.5 cycles.
    Cycles92_5,
    /// 247.5 cycles.
    Cycles247_5,
    /// 640.5 cycles.
    Cycles640_5,
}

impl SampleTime {
    const fn code(self) -> u8 {
        match self {
            SampleTime::Cycles2_5 => 0,
            SampleTime::Cycles6_5 => 1,
            SampleTime::Cycles12_5 => 2,
            SampleTime::Cycles24_5 => 3,
            SampleTime::Cycles47_5 => 4,
            SampleTime::Cycles92_5 => 5,
            SampleTime::Cycles247_5 => 6,
            SampleTime::Cycles640_5 => 7,
        }
    }
}

/// Analog channel number (0-15 external, 16-18 internal on ADC1).
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[non_exhaustive]
pub enum AdcChannel {
    Ch0,
    Ch1,
    Ch2,
    Ch3,
    Ch4,
    Ch5,
    Ch6,
    Ch7,
    Ch8,
    Ch9,
    Ch10,
    Ch11,
    Ch12,
    Ch13,
    Ch14,
    Ch15,
    /// ADC1 internal temperature sensor (IN16).
    Vts,
    /// ADC1 internal 1.2 V reference (IN17).
    Vintrv,
    /// ADC1 internal VBAT/4 (IN18).
    Vbat,
}

impl AdcChannel {
    /// Numeric channel index written to OSQ/SPT.
    pub const fn number(self) -> u8 {
        match self {
            AdcChannel::Ch0 => 0,
            AdcChannel::Ch1 => 1,
            AdcChannel::Ch2 => 2,
            AdcChannel::Ch3 => 3,
            AdcChannel::Ch4 => 4,
            AdcChannel::Ch5 => 5,
            AdcChannel::Ch6 => 6,
            AdcChannel::Ch7 => 7,
            AdcChannel::Ch8 => 8,
            AdcChannel::Ch9 => 9,
            AdcChannel::Ch10 => 10,
            AdcChannel::Ch11 => 11,
            AdcChannel::Ch12 => 12,
            AdcChannel::Ch13 => 13,
            AdcChannel::Ch14 => 14,
            AdcChannel::Ch15 => 15,
            AdcChannel::Vts => 16,
            AdcChannel::Vintrv => 17,
            AdcChannel::Vbat => 18,
        }
    }
}

/// A GPIO pin usable as an analog input on `T`.
pub trait AdcPin<T: Instance>: Pin {
    /// The channel this pin feeds.
    fn channel(&self) -> AdcChannel;
}

/// External trigger source for ordinary channel conversions
/// (CTRL2.OCTESEL, RM table 18-1).
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[non_exhaustive]
pub enum OrdinaryTrigger {
    Tmr1Ch1,
    Tmr1Ch2,
    Tmr1Ch3,
    Tmr1Trgout,
    Tmr1Trgout2,
    Tmr2Ch1,
    Tmr2Ch2,
    Tmr2Ch3,
    Tmr2Ch4,
    Tmr2Trgout,
    Tmr3Ch1,
    Tmr3Ch4,
    Tmr3Trgout,
    Tmr4Ch1,
    Tmr4Ch4,
    Tmr4Trgout,
    Tmr5Ch1,
    Tmr5Ch2,
    Tmr5Ch3,
    Tmr6Trgout,
    Tmr7Trgout,
    Tmr8Ch1,
    Tmr8Trgout,
    Tmr8Trgout2,
    Tmr20Ch1,
    Tmr20Ch2,
    Tmr20Ch3,
    Tmr20Trgout,
    Tmr20Trgout2,
    /// EXINT line 11 external pin.
    Exint11,
}

impl OrdinaryTrigger {
    const fn code(self) -> u8 {
        match self {
            OrdinaryTrigger::Tmr1Ch1 => 0x00,
            OrdinaryTrigger::Tmr1Ch2 => 0x01,
            OrdinaryTrigger::Tmr1Ch3 => 0x02,
            OrdinaryTrigger::Tmr2Ch2 => 0x03,
            OrdinaryTrigger::Tmr2Ch3 => 0x04,
            OrdinaryTrigger::Tmr2Ch4 => 0x05,
            OrdinaryTrigger::Tmr2Trgout => 0x06,
            OrdinaryTrigger::Tmr3Ch1 => 0x07,
            OrdinaryTrigger::Tmr3Trgout => 0x08,
            OrdinaryTrigger::Tmr4Ch4 => 0x09,
            OrdinaryTrigger::Tmr5Ch1 => 0x0a,
            OrdinaryTrigger::Tmr5Ch2 => 0x0b,
            OrdinaryTrigger::Tmr5Ch3 => 0x0c,
            OrdinaryTrigger::Tmr8Ch1 => 0x0d,
            OrdinaryTrigger::Tmr8Trgout => 0x0e,
            OrdinaryTrigger::Exint11 => 0x0f,
            OrdinaryTrigger::Tmr20Trgout => 0x10,
            OrdinaryTrigger::Tmr20Trgout2 => 0x11,
            OrdinaryTrigger::Tmr20Ch1 => 0x12,
            OrdinaryTrigger::Tmr20Ch2 => 0x13,
            OrdinaryTrigger::Tmr20Ch3 => 0x14,
            OrdinaryTrigger::Tmr8Trgout2 => 0x15,
            OrdinaryTrigger::Tmr1Trgout2 => 0x16,
            OrdinaryTrigger::Tmr4Trgout => 0x17,
            OrdinaryTrigger::Tmr6Trgout => 0x18,
            OrdinaryTrigger::Tmr3Ch4 => 0x19,
            OrdinaryTrigger::Tmr4Ch1 => 0x1a,
            OrdinaryTrigger::Tmr1Trgout => 0x1b,
            OrdinaryTrigger::Tmr2Ch1 => 0x1c,
            OrdinaryTrigger::Tmr7Trgout => 0x1e,
        }
    }
}

/// External trigger source for preempted channel conversions
/// (CTRL2.PCTESEL, RM table 18-2).
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[non_exhaustive]
pub enum PreemptedTrigger {
    Tmr1Ch3,
    Tmr1Ch4,
    Tmr1Trgout,
    Tmr1Trgout2,
    Tmr2Ch1,
    Tmr2Trgout,
    Tmr3Ch1,
    Tmr3Ch2,
    Tmr3Ch3,
    Tmr3Ch4,
    Tmr3Trgout,
    Tmr4Ch1,
    Tmr4Ch2,
    Tmr4Ch3,
    Tmr4Ch4,
    Tmr4Trgout,
    Tmr5Ch4,
    Tmr5Trgout,
    Tmr6Trgout,
    Tmr7Trgout,
    Tmr8Ch2,
    Tmr8Ch3,
    Tmr8Ch4,
    Tmr8Trgout,
    Tmr8Trgout2,
    Tmr20Ch2,
    Tmr20Ch4,
    Tmr20Trgout,
    Tmr20Trgout2,
    /// EXINT line 15 external pin.
    Exint15,
}

impl PreemptedTrigger {
    const fn code(self) -> u8 {
        match self {
            PreemptedTrigger::Tmr1Ch4 => 0x00,
            PreemptedTrigger::Tmr1Trgout => 0x01,
            PreemptedTrigger::Tmr2Ch1 => 0x02,
            PreemptedTrigger::Tmr2Trgout => 0x03,
            PreemptedTrigger::Tmr3Ch2 => 0x04,
            PreemptedTrigger::Tmr3Ch4 => 0x05,
            PreemptedTrigger::Tmr4Ch1 => 0x06,
            PreemptedTrigger::Tmr4Ch2 => 0x07,
            PreemptedTrigger::Tmr4Ch3 => 0x08,
            PreemptedTrigger::Tmr4Trgout => 0x09,
            PreemptedTrigger::Tmr5Ch4 => 0x0a,
            PreemptedTrigger::Tmr5Trgout => 0x0b,
            PreemptedTrigger::Tmr8Ch2 => 0x0c,
            PreemptedTrigger::Tmr8Ch3 => 0x0d,
            PreemptedTrigger::Tmr8Ch4 => 0x0e,
            PreemptedTrigger::Exint15 => 0x0f,
            PreemptedTrigger::Tmr20Trgout => 0x10,
            PreemptedTrigger::Tmr20Trgout2 => 0x11,
            PreemptedTrigger::Tmr20Ch4 => 0x12,
            PreemptedTrigger::Tmr1Trgout2 => 0x13,
            PreemptedTrigger::Tmr8Trgout => 0x14,
            PreemptedTrigger::Tmr8Trgout2 => 0x15,
            PreemptedTrigger::Tmr3Ch3 => 0x16,
            PreemptedTrigger::Tmr3Trgout => 0x17,
            PreemptedTrigger::Tmr3Ch1 => 0x18,
            PreemptedTrigger::Tmr6Trgout => 0x19,
            PreemptedTrigger::Tmr4Ch4 => 0x1a,
            PreemptedTrigger::Tmr1Ch3 => 0x1b,
            PreemptedTrigger::Tmr20Ch2 => 0x1c,
            PreemptedTrigger::Tmr7Trgout => 0x1e,
        }
    }
}

/// External trigger edge (CTRL2.OCETE / PCETE).
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
pub enum TriggerEdge {
    /// No edge detection (external trigger disabled).
    #[default]
    Disabled,
    Rising,
    Falling,
    Both,
}

impl TriggerEdge {
    const fn code(self) -> u8 {
        match self {
            TriggerEdge::Disabled => 0,
            TriggerEdge::Rising => 1,
            TriggerEdge::Falling => 2,
            TriggerEdge::Both => 3,
        }
    }
}

/// Oversampling ratio (OVSP.OSRSEL). `shift` returns the right shift that
/// averages the accumulated sum back to the resolution scale.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
pub enum OversampleRatio {
    #[default]
    X2,
    X4,
    X8,
    X16,
    X32,
    X64,
    X128,
    X256,
}

impl OversampleRatio {
    const fn code(self) -> u8 {
        match self {
            OversampleRatio::X2 => 0,
            OversampleRatio::X4 => 1,
            OversampleRatio::X8 => 2,
            OversampleRatio::X16 => 3,
            OversampleRatio::X32 => 4,
            OversampleRatio::X64 => 5,
            OversampleRatio::X128 => 6,
            OversampleRatio::X256 => 7,
        }
    }

    /// log2 of the ratio (1 for X2 .. 8 for X256).
    pub const fn shift(self) -> u8 {
        match self {
            OversampleRatio::X2 => 1,
            OversampleRatio::X4 => 2,
            OversampleRatio::X8 => 3,
            OversampleRatio::X16 => 4,
            OversampleRatio::X32 => 5,
            OversampleRatio::X64 => 6,
            OversampleRatio::X128 => 7,
            OversampleRatio::X256 => 8,
        }
    }
}

/// Behavior when a preempted conversion interrupts an ordinary oversampled
/// conversion (OVSP.OOSRSEL).
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
pub enum OversampleRestart {
    /// Keep the accumulated sum and resume where it stopped.
    #[default]
    Continue,
    /// Clear the accumulated sum and redo the channel.
    Restart,
}

/// Whether all sub-conversions of an ordinary oversampled conversion run on
/// one trigger or each needs its own trigger (OVSP.OOSTREN). Triggered mode
/// cannot be combined with repeat mode (RPEN).
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
pub enum OversampleTrigger {
    /// One trigger completes the whole oversampled conversion.
    #[default]
    SingleTrigger,
    /// Every sub-conversion needs its own trigger.
    Triggered,
}

/// Oversampler configuration (RM 18.4.5). While the oversampler is enabled
/// the hardware ignores DTALIGN and PCDTOx and produces right-aligned
/// 16-bit results.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct Oversampling {
    pub ratio: OversampleRatio,
    /// Right shift (0-8) applied to the accumulated sum before it lands in
    /// the data register.
    pub shift: u8,
    pub restart: OversampleRestart,
    pub trigger: OversampleTrigger,
    /// Oversample the ordinary group.
    pub ordinary: bool,
    /// Oversample the preempted group.
    pub preempted: bool,
}

/// Channel set monitored by the voltage monitor (CTRL1.VMSGEN/VMCSEL).
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum VmTarget {
    /// All channels.
    All,
    /// One channel (internal channels allowed).
    Single(AdcChannel),
}

/// ADC configuration.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct Config {
    /// Conversion resolution.
    pub resolution: Resolution,
    /// ADCCLK target. The prescaler is computed from HCLK (ADCCLK =
    /// HCLK/(ADCDIV+2), max 80 MHz and not above PCLK2). None = the highest
    /// ADCCLK satisfying both limits. All active ADC instances must use the
    /// same prescaler because ADCCLK is shared.
    pub adcclk: Option<Hertz>,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            resolution: Resolution::Bits12,
            adcclk: None,
        }
    }
}

/// ADC conversion error.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[non_exhaustive]
pub enum Error {
    /// Overrun: the previous conversion data was not consumed (OCCO).
    Overrun,
    /// More channels than the 16-slot ordinary sequence allows.
    TooManyChannels,
    /// A DMA transfer failed.
    Dma(DmaError),
}

impl From<DmaError> for Error {
    fn from(e: DmaError) -> Self {
        Error::Dma(e)
    }
}

trait SealedInstance {
    fn regs() -> &'static pac::adc1::RegisterBlock;
}

/// ADC instance trait.
#[allow(private_bounds)]
pub trait Instance:
    SealedInstance + RccPeripheral + embassy_hal_internal::PeripheralType + 'static
{
}

macro_rules! impl_adc_instance {
    ($inst:ident) => {
        impl SealedInstance for crate::peripherals::$inst {
            #[inline]
            fn regs() -> &'static pac::adc1::RegisterBlock {
                unsafe { &*pac::$inst::PTR }
            }
        }

        impl Instance for crate::peripherals::$inst {}
    };
}

impl_adc_instance!(ADC1);
impl_adc_instance!(ADC2);
impl_adc_instance!(ADC3);

crate::dma_trait!(RxDma, Instance);

/// ADCCLK prescaler value (0..15 -> HCLK/(2+code)).
fn adcdiv_for(hclk: u32, pclk2: u32, target: Option<u32>) -> u8 {
    let limit = target.unwrap_or(80_000_000).min(80_000_000).min(pclk2);
    assert!(limit != 0, "ADC clock target must be nonzero");
    let divisor = hclk.div_ceil(limit).max(2);
    assert!(divisor <= 17, "ADC clock target is below HCLK/17");
    (divisor - 2) as u8
}

/// ADC driver handle.
pub struct Adc<'d, T: Instance> {
    _peri: Peri<'d, T>,
    resolution: Resolution,
    sequence_armed: bool,
    _mode: PhantomData<T>,
}

impl<'d, T: Instance> Adc<'d, T> {
    /// Powers up the ADC and calibrates it (calibration runs at 12-bit per the
    /// RM, then the configured resolution is applied).
    ///
    /// Panics if another active ADC uses a different shared prescaler.
    pub fn new(peri: Peri<'d, T>, config: Config) -> Self {
        let clocks = rcc::clocks();
        let div = adcdiv_for(clocks.hclk.0, clocks.pclk2.0, config.adcclk.map(|c| c.0));
        T::enable_and_reset();
        let r = T::regs();
        critical_section::with(|_| {
            let com = unsafe { &*pac::ADCCOM::PTR };
            let gates = unsafe { &*pac::CRM::PTR }.apb2en().read();
            let adc_on = |enabled, regs: *const pac::adc1::RegisterBlock| {
                enabled && unsafe { &*regs }.ctrl2().read().adcen().bit_is_set()
            };
            if adc_on(gates.adc1().bit_is_set(), pac::ADC1::PTR)
                || adc_on(gates.adc2().bit_is_set(), pac::ADC2::PTR)
                || adc_on(gates.adc3().bit_is_set(), pac::ADC3::PTR)
            {
                assert_eq!(
                    com.cctrl().read().adcdiv().bits(),
                    div,
                    "active ADCs must share ADCCLK"
                );
            } else {
                com.cctrl().modify(|_, w| unsafe { w.adcdiv().bits(div) });
            }

            // OCCE after every ordinary conversion, including sequence DMA.
            r.ctrl2()
                .modify(|_, w| w.eocsfen().set_bit().adcen().set_bit());
        });
        while !r.sts().read().rdy().bit_is_set() {}
        // Calibrate, then apply the requested resolution.
        r.ctrl2().modify(|_, w| w.adcal().set_bit());
        while r.ctrl2().read().adcal().bit_is_set() {}
        r.ctrl1()
            .modify(|_, w| unsafe { w.crsel().bits(config.resolution.code()) });

        Self {
            _peri: peri,
            resolution: config.resolution,
            sequence_armed: false,
            _mode: PhantomData,
        }
    }

    /// Configured resolution.
    pub fn resolution(&self) -> Resolution {
        self.resolution
    }

    fn r(&self) -> &'static pac::adc1::RegisterBlock {
        T::regs()
    }

    /// Sets the sample time of one channel (SPT1 covers 10-18, SPT2 0-9).
    fn set_sample_time(&self, ch: u8, sample: SampleTime) {
        let code = sample.code();
        let r = self.r();
        if ch >= 10 {
            r.spt1()
                .modify(|_, w| unsafe { w.cspt(ch - 10).bits(code) });
        } else {
            r.spt2().modify(|_, w| unsafe { w.cspt(ch).bits(code) });
        }
    }

    /// Writes a channel into ordinary sequence slot 1..16.
    fn set_sequence_slot(&self, slot: usize, ch: u8) {
        let r = self.r();
        match slot {
            0..=5 => r
                .osq3()
                .modify(|_, w| unsafe { w.osn(slot as u8).bits(ch) }),
            6..=11 => r
                .osq2()
                .modify(|_, w| unsafe { w.osn(slot as u8 - 6).bits(ch) }),
            _ => r
                .osq1()
                .modify(|_, w| unsafe { w.osn(slot as u8 - 12).bits(ch) }),
        };
    }

    /// Clears the overrun flag (STS.OCCO). While OCCO is set the ADC has
    /// stopped converting (RM 18.4.6.2) - e.g. after hardware-triggered
    /// conversions nobody consumed; software-triggered reads clear it as
    /// recovery.
    pub fn clear_overrun(&mut self) {
        self.r().sts().modify(|_, w| w.occo().clear_bit());
    }

    /// Single-channel blocking conversion (sequence mode disabled).
    ///
    /// `pin` is configured as an analog input on first use of the channel.
    pub fn blocking_read(&mut self, pin: Peri<'_, impl AdcPin<T>>, sample: SampleTime) -> u16 {
        let ch = pin.channel();
        pin.set_as_analog();
        self.read_channel(ch.number(), sample)
    }

    /// Blocking conversion of a raw channel number (internal channels included).
    pub fn read_channel(&mut self, ch: u8, sample: SampleTime) -> u16 {
        let r = self.r();
        self.leave_sequence_mode();
        self.set_sample_time(ch, sample);
        // Non-sequence mode: OSN1 holds the channel.
        self.set_sequence_slot(0, ch);
        // Software trigger + wait for the conversion (OCCE also clears by
        // reading ODT). A stale overrun would keep the ADC halted, so clear
        // it before triggering.
        r.sts()
            .modify(|_, w| w.occe().clear_bit().occo().clear_bit());
        r.ctrl2().modify(|_, w| w.ocswtrg().triggered());
        while !r.sts().read().occe().bit_is_set() {}
        r.odt().read().odt().bits()
    }

    /// Blocking multi-channel sequence: triggers once and polls per
    /// conversion (OCCE sets per conversion; reading ODT clears it).
    /// `buf.len()` channels max 16.
    pub fn blocking_read_sequence(
        &mut self,
        channels: &[AdcChannel],
        sample: SampleTime,
        buf: &mut [u16],
    ) -> Result<(), Error> {
        if channels.len() != buf.len() {
            return Err(Error::TooManyChannels);
        }
        if channels.is_empty() || channels.len() > 16 {
            return Err(Error::TooManyChannels);
        }
        let r = self.r();
        for (i, ch) in channels.iter().enumerate() {
            self.set_sample_time(ch.number(), sample);
            self.set_sequence_slot(i, ch.number());
        }
        // Sequence mode: OLEN = count - 1; per-conversion polling (no
        // OCCEIEN, so the flag sets after every conversion).
        r.ctrl1().modify(|_, w| {
            w.sqen().set_bit();
            w
        });
        r.osq1()
            .modify(|_, w| unsafe { w.oclen().bits(channels.len() as u8 - 1) });
        self.sequence_armed = true;
        r.sts()
            .modify(|_, w| w.occe().clear_bit().occo().clear_bit());
        r.ctrl2().modify(|_, w| w.ocswtrg().triggered());
        for out in buf.iter_mut() {
            while !r.sts().read().occe().bit_is_set() {}
            *out = r.odt().read().odt().bits();
        }
        Ok(())
    }

    /// Async sequence scan with DMA: one DMA request per conversion fills
    /// `buf`. `rx_dma` is an existing channel handle (its IRQ must be bound
    /// with `bind_interrupts!`; `listen()` is called here). Switches the
    /// driver into sequence mode.
    ///
    /// # Safety
    ///
    /// Use DMA-accessible memory. If the pending future is forgotten, keep
    /// the receive buffer exclusively available to DMA until the channel stops.
    pub async unsafe fn scan_dma<RXD: RxDma<T>>(
        &mut self,
        channels: &[AdcChannel],
        sample: SampleTime,
        buf: &mut [u16],
        rx_dma: &mut Channel<'_, RXD>,
    ) -> Result<(), Error> {
        if channels.is_empty() || channels.len() > 16 || channels.len() != buf.len() {
            return Err(Error::TooManyChannels);
        }
        let r = self.r();
        for (i, ch) in channels.iter().enumerate() {
            self.set_sample_time(ch.number(), sample);
            self.set_sequence_slot(i, ch.number());
        }
        r.ctrl1().modify(|_, w| {
            w.sqen().set_bit();
            w
        });
        r.osq1()
            .modify(|_, w| unsafe { w.oclen().bits(channels.len() as u8 - 1) });
        self.sequence_armed = true;

        let dma = rx_dma;
        unsafe {
            dma.configure_peripheral_to_memory(
                RXD::request(),
                self.odt_addr(),
                buf.as_mut_ptr() as *mut u8,
                buf.len() as u16,
                DmaConfig::peripheral_to_memory(Width::Bits16),
            )?;
        }
        // DMA requests: one per ordinary conversion (OCDMAEN).
        r.ctrl2().modify(|_, w| w.ocdmaen().set_bit());
        r.sts()
            .modify(|_, w| w.occe().clear_bit().occo().clear_bit());
        let _disable_request = OnDrop::new(|| {
            r.ctrl2().modify(|_, w| w.ocdmaen().clear_bit());
        });
        dma.start();
        r.ctrl2().modify(|_, w| w.ocswtrg().triggered());
        let res = dma.wait_transfer().await;
        if r.sts().read().occo().bit_is_set() {
            r.sts().modify(|_, w| w.occo().clear_bit());
            res?;
            return Err(Error::Overrun);
        }
        res.map_err(Error::from)
    }

    /// Data register address (DMA peripheral side).
    fn odt_addr(&self) -> u32 {
        self.r().odt() as *const _ as u32
    }

    /// Selects the external trigger for ordinary conversions
    /// (CTRL2.OCTESEL + OCETE, RM 18.4.2.2). `None` disables edge-triggered
    /// conversion (software trigger only).
    pub fn set_ordinary_trigger(&mut self, source: Option<OrdinaryTrigger>, edge: TriggerEdge) {
        let code = source.map(|s| s.code()).unwrap_or(0);
        let e = edge.code();
        self.r().ctrl2().modify(|_, w| unsafe {
            w.octesel_l()
                .bits(code & 0xf)
                .octesel_h()
                .bit(code & 0x10 != 0)
                .ocete()
                .bits(e)
        });
    }

    /// Selects the external trigger for preempted conversions
    /// (CTRL2.PCTESEL + PCETE, RM 18.4.2.2).
    pub fn set_preempted_trigger(&mut self, source: Option<PreemptedTrigger>, edge: TriggerEdge) {
        let code = source.map(|s| s.code()).unwrap_or(0);
        let e = edge.code();
        self.r().ctrl2().modify(|_, w| unsafe {
            w.pctesel_l()
                .bits(code & 0xf)
                .pctesel_h()
                .bit(code & 0x10 != 0)
                .pcete()
                .bits(e)
        });
    }

    /// Blocking preempted-group conversion (RM 18.4.3): arms the 1..4-slot
    /// preempted sequence, software-triggers it, waits for PCCE and copies
    /// the results into `out`.
    ///
    /// The RM starts the sequence at PSN(4 - PCLEN); the hardware fills
    /// PDT1..PDT4 in CONVERSION ORDER (hw-verified: PSN3/PSN4 selected, data
    /// in PDT1/PDT2), so `out[i]` is the i-th conversion and
    /// [`Adc::set_preempted_offset`] with slot `i` shifts it. With a nonzero
    /// offset the value minus offset can go negative (SIGN in the upper
    /// bits; read as `data as i16`).
    pub fn blocking_read_preempted(
        &mut self,
        channels: &[AdcChannel],
        sample: SampleTime,
        out: &mut [u16],
    ) -> Result<(), Error> {
        if channels.is_empty() || channels.len() > 4 || channels.len() != out.len() {
            return Err(Error::TooManyChannels);
        }
        let r = self.r();
        let len = channels.len() as u8;
        // The sequence starts at PSN(4 - PCLEN) (1-based), so the i-th
        // channel sits at 0-based PSN slot (4 - len) + i. The DATA, however,
        // lands in PDT1..PDTN by conversion order (hw-verified).
        let base = (4 - len) as usize;
        for (i, ch) in channels.iter().enumerate() {
            self.set_sample_time(ch.number(), sample);
            r.psq()
                .modify(|_, w| unsafe { w.psn((base + i) as u8).bits(ch.number()) });
        }
        r.psq().modify(|_, w| unsafe { w.pclen().bits(len - 1) });
        // SQEN gates the preempted sequence too (RM 18.4.3.1): with it off a
        // trigger converts PSN4 only. Turn it on for the multi-slot sequence
        // and restore the previous state afterwards (sequence_armed keeps the
        // ordinary reads' bookkeeping consistent). Clear a stale overrun so
        // the ADC is not halted.
        let seq_was = self.sequence_armed;
        r.ctrl1().modify(|_, w| w.sqen().set_bit());
        self.sequence_armed = true;
        r.sts()
            .modify(|_, w| w.pcce().clear_bit().occo().clear_bit());
        r.ctrl2().modify(|_, w| w.pcswtrg().triggered());
        while !r.sts().read().pcce().bit_is_set() {}
        for (i, o) in out.iter_mut().enumerate() {
            *o = r.pdt(i).read().pdt().bits();
        }
        if !seq_was {
            r.ctrl1().modify(|_, w| w.sqen().clear_bit());
            self.sequence_armed = false;
        }
        Ok(())
    }

    /// Raw preempted data register readback (slot 0..3 = PDT1..4).
    pub fn preempted_data(&self, slot: usize) -> u16 {
        self.r().pdt(slot).read().pdt().bits()
    }

    /// Sets the offset subtracted from the raw value of preempted data
    /// register `slot` (0..3 = PCDTO1..4; PCDTOx applies to PDTx, which is
    /// filled in conversion order - see [`Adc::blocking_read_preempted`]).
    pub fn set_preempted_offset(&mut self, slot: usize, offset: u16) {
        self.r()
            .pcdto(slot)
            .modify(|_, w| unsafe { w.dto().bits(offset) });
    }

    /// Enables the hardware oversampler (RM 18.4.5). Every read of the
    /// affected group then averages `ratio` conversions (right-aligned
    /// 16-bit result; DTALIGN/PCDTOx are ignored while enabled).
    pub fn enable_oversampling(&mut self, cfg: Oversampling) {
        let shift = cfg.shift.min(8);
        self.r().ovsp().modify(|_, w| unsafe {
            w.oosen()
                .bit(cfg.ordinary)
                .posen()
                .bit(cfg.preempted)
                .osrsel()
                .bits(cfg.ratio.code())
                .osssel()
                .bits(shift)
                .oostren()
                .bit(cfg.trigger == OversampleTrigger::Triggered)
                .oosrsel()
                .bit(cfg.restart == OversampleRestart::Restart)
        });
    }

    /// Disables the oversampler.
    pub fn disable_oversampling(&mut self) {
        self.r()
            .ovsp()
            .modify(|_, w| w.oosen().clear_bit().posen().clear_bit());
    }

    /// Sets the voltage-monitor bounds and channel set (RM 18.4.7). The
    /// bounds compare against the raw conversion counts (12-bit, or the full
    /// 16-bit accumulated value while the oversampler is on); CRSEL,
    /// DTALIGN and PCDTOx have no effect on the comparison.
    pub fn set_voltage_monitor(&mut self, high: u16, low: u16, target: VmTarget) {
        let r = self.r();
        r.vmb(0).write(|w| unsafe { w.vmb().bits(high) });
        r.vmb(1).write(|w| unsafe { w.vmb().bits(low) });
        r.ctrl1().modify(|_, w| match target {
            VmTarget::All => w.vmsgen().clear_bit(),
            VmTarget::Single(ch) => {
                w.vmsgen().set_bit();
                unsafe { w.vmcsel().bits(ch.number()) }
            }
        });
    }

    /// Enables voltage monitoring on ordinary conversions (CTRL1.OCVMEN).
    pub fn enable_ordinary_vm(&mut self, on: bool) {
        self.r().ctrl1().modify(|_, w| w.ocvmen().bit(on));
    }

    /// Enables voltage monitoring on preempted conversions (CTRL1.PCVMEN).
    pub fn enable_preempted_vm(&mut self, on: bool) {
        self.r().ctrl1().modify(|_, w| w.pcvmen().bit(on));
    }

    /// True while the monitored conversion was outside the VMHB/VMLB bounds
    /// (STS.VMOR).
    pub fn vm_out_of_range(&self) -> bool {
        self.r().sts().read().vmor().bit_is_set()
    }

    /// Clears the voltage-monitor flag.
    pub fn clear_vm_flag(&mut self) {
        self.r().sts().modify(|_, w| w.vmor().clear_bit());
    }

    /// Aborts the running conversion sequence (ADABRT) and waits for the
    /// hardware to release the bit before other ADC operations (RM 18.4.4).
    pub fn abort(&mut self) {
        let r = self.r();
        r.ctrl2().modify(|_, w| w.adabrt().set_bit());
        while r.ctrl2().read().adabrt().bit_is_set() {}
    }

    /// Repeat mode (CTRL2.RPEN): one trigger re-converts the single channel
    /// or the whole ordinary sequence until ADCEN is cleared. Cannot be
    /// combined with partitioned mode.
    pub fn set_repeat_mode(&mut self, on: bool) {
        self.r().ctrl2().modify(|_, w| w.rpen().bit(on));
    }

    /// Preempted auto-conversion (CTRL1.PCAUTOEN, RM 18.4.3.2): the
    /// preempted sequence runs right after each ordinary conversion/sequence.
    pub fn set_preempted_auto(&mut self, on: bool) {
        self.r().ctrl1().modify(|_, w| w.pcautoen().bit(on));
    }

    /// Ordinary partitioned mode (CTRL1.OCPEN + OCPCNT, RM 18.4.3.4): each
    /// trigger converts `count` (1..8) ordinary slots - one subgroup - and
    /// rotates over the full sequence. `None` disables. Cannot be combined
    /// with repeat mode.
    pub fn set_ordinary_partition(&mut self, count: Option<u8>) {
        self.r().ctrl1().modify(|_, w| match count {
            None => w.ocpen().clear_bit(),
            Some(c) => unsafe {
                let n = c.clamp(1, 8) - 1;
                w.ocpcnt().bits(n).ocpen().set_bit()
            },
        });
    }

    /// Preempted partitioned mode (CTRL1.PCPEN): each preempted trigger
    /// converts exactly one PSN slot, rotating over the sequence.
    pub fn enable_preempted_partition(&mut self, on: bool) {
        self.r().ctrl1().modify(|_, w| w.pcpen().bit(on));
    }

    /// Leaves sequence mode (single-channel reads use OSN1 only).
    fn leave_sequence_mode(&mut self) {
        if self.sequence_armed {
            self.r().ctrl1().modify(|_, w| {
                w.sqen().clear_bit();
                w
            });
            self.sequence_armed = false;
        }
    }

    /// Blocks while reading the internal temperature sensor (IN16; ADC1 only
    /// on this family - ADC2/3 pin 16-18 are VSS/VREF).
    pub fn blocking_read_vts(&mut self, sample: SampleTime) -> u16 {
        self.enable_itsrv();
        // Sensor power-up time (~10 us at typical clocks; a short spin is
        // enough since conversions take longer than that anyway).
        let mut left = 10_000u32;
        while left > 0 {
            core::hint::spin_loop();
            left -= 1;
        }
        self.read_channel(
            AdcChannel::Vts.number(),
            sample.max(SampleTime::Cycles640_5),
        )
    }

    /// Blocks while reading the internal 1.2 V reference (IN17; ADC1 only).
    pub fn blocking_read_vintrv(&mut self, sample: SampleTime) -> u16 {
        self.enable_itsrv();
        self.read_channel(
            AdcChannel::Vintrv.number(),
            sample.max(SampleTime::Cycles640_5),
        )
    }

    /// Blocks while reading VBAT/4 (IN18; ADC1 only). Disables the VBAT
    /// divider afterwards (it draws current continuously once enabled).
    pub fn blocking_read_vbat(&mut self, sample: SampleTime) -> u16 {
        self.enable_vbat(true);
        let v = self.read_channel(
            AdcChannel::Vbat.number(),
            sample.max(SampleTime::Cycles640_5),
        );
        self.enable_vbat(false);
        v
    }

    /// Enables the internal temperature sensor + 1.2 V reference (ITSRVEN) in
    /// the shared common block.
    fn enable_itsrv(&mut self) {
        let com = unsafe { &*pac::ADCCOM::PTR };
        critical_section::with(|_| com.cctrl().modify(|_, w| w.itsrven().set_bit()));
    }

    /// Enables/disables the VBAT/4 divider (VBATEN, shared common block).
    fn enable_vbat(&mut self, on: bool) {
        let com = unsafe { &*pac::ADCCOM::PTR };
        critical_section::with(|_| com.cctrl().modify(|_, w| w.vbaten().bit(on)));
    }
}

impl<T: Instance> Drop for Adc<'_, T> {
    fn drop(&mut self) {
        T::regs().ctrl2().modify(|_, w| w.adcen().clear_bit());
    }
}
