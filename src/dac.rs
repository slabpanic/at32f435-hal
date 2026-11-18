//! DAC (RM 19): two 12-bit channels (DAC1_OUT = PA4, DAC2_OUT = PA5) with
//! software/timer/EXTI9 triggers, hardware noise/triangle wave generation,
//! DMA feed and dual-channel synchronized updates.
//!
//! Data paths per channel: 12-bit right (DTH12R), 12-bit left (DTH12L) and
//! 8-bit right (DTH8R) holding registers, all reflected in the read-only
//! output register (ODT). Without a trigger (TRGEN = 0) a holding-register
//! write reaches the output after one APB cycle; with TRGEN = 1 the transfer
//! waits for the selected event (`software_trigger()` / timer TRGOUT).
//!
//! The same pins are ADC12_IN4/IN5, so a DAC -> pin -> ADC loopback is a
//! self-contained hardware test (used by `phase5_test` on the evb).
//!
//! ```ignore
//! let mut dac = hal::dac::Dac::new(p.DAC);
//! dac.enable_channel(hal::dac::Channel::Ch1, &hal::dac::ChannelConfig::buffered());
//! dac.set_value_12bit(hal::dac::Channel::Ch1, 0x800); // ~VDDA/2 (no trigger)
//! ```

use embassy_hal_internal::Peri;

use crate::{pac, peripherals};

/// DAC channel.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Channel {
    /// DAC1 (PA4 / ADC12_IN4).
    Ch1,
    /// DAC2 (PA5 / ADC12_IN5).
    Ch2,
}

impl Channel {
    const fn shift(self) -> u32 {
        match self {
            Channel::Ch1 => 0,
            Channel::Ch2 => 16,
        }
    }
}

/// Conversion trigger (`CTRL.DTxTRGSEL[2:0]`, official `dac_trigger_select_type`).
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
pub enum Trigger {
    /// TMR6 TRGOUT.
    #[default]
    Tmr6,
    /// TMR8 TRGOUT.
    Tmr8,
    /// TMR7 TRGOUT.
    Tmr7,
    /// TMR5 TRGOUT.
    Tmr5,
    /// TMR2 TRGOUT.
    Tmr2,
    /// TMR4 TRGOUT.
    Tmr4,
    /// EXTI line 9 event.
    Exti9,
    /// Software trigger (write 1 to SWTRG.DTxSWTRG).
    Software,
}

impl Trigger {
    const fn code(self) -> u8 {
        match self {
            Trigger::Tmr6 => 0,
            Trigger::Tmr8 => 1,
            Trigger::Tmr7 => 2,
            Trigger::Tmr5 => 3,
            Trigger::Tmr2 => 4,
            Trigger::Tmr4 => 5,
            Trigger::Exti9 => 6,
            Trigger::Software => 7,
        }
    }
}

/// Hardware wave generation (`CTRL.DTxNM[1:0]` + `DTxNBSEL[3:0]` amplitude).
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
pub enum WaveGen {
    /// No wave generation (static output).
    #[default]
    None,
    /// Pseudo-noise; `amplitude` is `NBSEL[3:0]` (LSB count 1..=15 scaled by VREF/2^12).
    Noise { amplitude: u8 },
    /// Triangle; `amplitude` is `NBSEL[3:0]` (peak = amplitude * (VREF/4096) * ...).
    Triangle { amplitude: u8 },
}

impl WaveGen {
    const fn nm(self) -> u8 {
        match self {
            WaveGen::None => 0,
            WaveGen::Noise { .. } => 1,
            WaveGen::Triangle { .. } => 2,
        }
    }

    const fn nbsel(self) -> u8 {
        match self {
            WaveGen::None => 0,
            WaveGen::Noise { amplitude } => amplitude & 0x0F,
            WaveGen::Triangle { amplitude } => amplitude & 0x0F,
        }
    }
}

/// Data alignment (which holding register the value is written to).
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
pub enum Alignment {
    /// 12-bit right-aligned (`DTH12R[11:0]`).
    #[default]
    Right12,
    /// 12-bit left-aligned (`DTH12L[15:4]`).
    Left12,
    /// 8-bit right-aligned (`DTH8R[7:0]`, lower 4 bits of the 12-bit value dropped).
    Right8,
}

/// Per-channel configuration.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct ChannelConfig {
    /// Conversion trigger; `None` = untriggered (holding writes reach the
    /// output immediately).
    pub trigger: Option<Trigger>,
    /// Output buffer on the DAC pin (default on: stronger drive, slight
    /// rail-to-rail headroom loss).
    pub output_buffer: bool,
    /// Hardware wave generation.
    pub wave: WaveGen,
}

impl ChannelConfig {
    /// Buffered output, no trigger, no wave (static voltage source).
    pub const fn buffered() -> Self {
        Self {
            trigger: None,
            output_buffer: true,
            wave: WaveGen::None,
        }
    }
}

/// DAC controller (both channels share one register block).
pub struct Dac {
    _peri: Peri<'static, peripherals::DAC>,
}

impl Dac {
    /// Enables the DAC APB clock and leaves both channels disabled.
    pub fn new(peri: Peri<'static, peripherals::DAC>) -> Self {
        <peripherals::DAC as crate::peripheral::ClockControl>::enable_and_reset();
        Self { _peri: peri }
    }

    /// Enables a channel with the given configuration. The pin must be put
    /// into analog mode by the caller (`Peri::set_as_analog` on PA4/PA5).
    pub fn enable_channel(&mut self, ch: Channel, cfg: &ChannelConfig) {
        let r = unsafe { &*pac::DAC::PTR };
        let mut ctrl = r.ctrl().read().bits();
        let s = ch.shift();
        ctrl &= !(0x3FFF << s); // EN/OBDIS/TRGEN/TRGSEL[5:3]/NM[7:6]/NBSEL[11:8] of this channel
        if cfg.output_buffer {
            // OBDIS = 0 keeps the buffer on.
        } else {
            ctrl |= 1 << (1 + s);
        }
        if let Some(t) = cfg.trigger {
            ctrl |= 1 << (2 + s); // TRGEN
            ctrl |= (t.code() as u32) << (3 + s);
        }
        ctrl |= (cfg.wave.nm() as u32) << (6 + s);
        ctrl |= (cfg.wave.nbsel() as u32) << (8 + s);
        r.ctrl().write(|w| unsafe { w.bits(ctrl) });
        r.ctrl().modify(|_, w| match ch {
            Channel::Ch1 => w.d1en().set_bit(),
            Channel::Ch2 => w.d2en().set_bit(),
        });
    }

    /// Enables/disables the channel's output buffer/gain (CTRL.DTxOBDIS).
    pub fn set_output_buffer(&mut self, ch: Channel, enabled: bool) {
        let r = unsafe { &*pac::DAC::PTR };
        r.ctrl().modify(|_, w| match (ch, enabled) {
            (Channel::Ch1, true) => w.d1obdis().clear_bit(),
            (Channel::Ch1, false) => w.d1obdis().set_bit(),
            (Channel::Ch2, true) => w.d2obdis().clear_bit(),
            (Channel::Ch2, false) => w.d2obdis().set_bit(),
        });
    }

    /// Disables a channel (register state preserved for re-enable).
    pub fn disable_channel(&mut self, ch: Channel) {
        let r = unsafe { &*pac::DAC::PTR };
        r.ctrl().modify(|_, w| match ch {
            Channel::Ch1 => w.d1en().clear_bit(),
            Channel::Ch2 => w.d2en().clear_bit(),
        });
    }

    /// Writes a value to the channel's holding register in the requested
    /// alignment. Without a trigger this reaches the output after one APB
    /// cycle; with a trigger it waits for `software_trigger()` / the event.
    pub fn set_value(&mut self, ch: Channel, value: u16, align: Alignment) {
        let value = match align {
            Alignment::Right12 => (value & 0x0FFF) as u32,
            Alignment::Left12 => ((value & 0x0FFF) as u32) << 4,
            Alignment::Right8 => ((value >> 4) & 0xFF) as u32,
        };
        // Holding registers: D1DTH12R/L/8R at 0x08/0x0C/0x10, D2DTH12R/L/8R
        // at 0x14/0x18/0x1C (per-channel stride 0x0C; the PAC emits a
        // distinct type per register, so dispatch by address).
        let addr = pac::DAC::PTR as usize
            + 0x08
            + 0x0C * (ch as usize)
            + match align {
                Alignment::Right12 => 0,
                Alignment::Left12 => 4,
                Alignment::Right8 => 8,
            };
        unsafe { (addr as *mut u32).write_volatile(value) };
    }

    /// 12-bit right-aligned convenience wrapper.
    pub fn set_value_12bit(&mut self, ch: Channel, value: u16) {
        self.set_value(ch, value, Alignment::Right12);
    }

    /// Dual-channel synchronized update: writes both holding registers
    /// (12-bit right) in one word so both outputs transfer together under a
    /// shared trigger (or untriggered, in the same APB cycle).
    pub fn set_value_dual_12bit(&mut self, ch1: u16, ch2: u16) {
        let r = unsafe { &*pac::DAC::PTR };
        let v = ((ch1 & 0x0FFF) as u32) | (((ch2 & 0x0FFF) as u32) << 16);
        r.ddth12r().write(|w| unsafe { w.bits(v) });
    }

    /// Fires the software trigger (only meaningful with
    /// `Trigger::Software` + `TRGEN`).
    pub fn software_trigger(&mut self, ch: Channel) {
        // SWTRG is write-only (bit 0 = DAC1, bit 1 = DAC2): write 1 to fire.
        let bit = match ch {
            Channel::Ch1 => 0x1,
            Channel::Ch2 => 0x2,
        };
        unsafe { ((pac::DAC::PTR as usize + 0x04) as *mut u32).write_volatile(bit) };
    }

    /// Reads the channel's current output data (ODT, 12-bit).
    pub fn output_value(&self, ch: Channel) -> u16 {
        let r = unsafe { &*pac::DAC::PTR };
        match ch {
            Channel::Ch1 => (r.d1odt().read().bits() & 0x0FFF) as u16,
            Channel::Ch2 => (r.d2odt().read().bits() & 0x0FFF) as u16,
        }
    }

    /// Enables the DMA request for a channel. The DMA stream should target
    /// the holding register address matching the desired alignment - see
    /// [`Dac::dma_data_register_address`].
    pub fn enable_dma(&mut self, ch: Channel) {
        let r = unsafe { &*pac::DAC::PTR };
        r.ctrl().modify(|_, w| match ch {
            Channel::Ch1 => w.d1dmaen().set_bit(),
            Channel::Ch2 => w.d2dmaen().set_bit(),
        });
    }

    /// Disables the DMA request.
    pub fn disable_dma(&mut self, ch: Channel) {
        let r = unsafe { &*pac::DAC::PTR };
        r.ctrl().modify(|_, w| match ch {
            Channel::Ch1 => w.d1dmaen().clear_bit(),
            Channel::Ch2 => w.d2dmaen().clear_bit(),
        });
    }

    /// The holding-register address for `ch`/`align` - wire this into a DMA
    /// stream's memory->peripheral transfer to stream waveform samples.
    pub fn dma_data_register_address(ch: Channel, align: Alignment) -> u32 {
        let r = unsafe { &*pac::DAC::PTR };
        match (ch, align) {
            (Channel::Ch1, Alignment::Right12) => r.d1dth12r() as *const _ as u32,
            (Channel::Ch1, Alignment::Left12) => r.d1dth12l() as *const _ as u32,
            (Channel::Ch1, Alignment::Right8) => r.d1dth8r() as *const _ as u32,
            (Channel::Ch2, Alignment::Right12) => r.d2dth12r() as *const _ as u32,
            (Channel::Ch2, Alignment::Left12) => r.d2dth12l() as *const _ as u32,
            (Channel::Ch2, Alignment::Right8) => r.d2dth8r() as *const _ as u32,
        }
    }

    /// DMA underrun flag (STS.DMAUDRx): the DMA request was not served in
    /// time for the trigger.
    pub fn dma_underrun_flag(&self, ch: Channel) -> bool {
        let r = unsafe { &*pac::DAC::PTR };
        let sts = r.sts().read();
        match ch {
            Channel::Ch1 => sts.dmaudr1().bit_is_set(),
            Channel::Ch2 => sts.dmaudr2().bit_is_set(),
        }
    }

    /// Clears the DMA underrun flag (STS is write-0-to-clear: write the
    /// read-back value with the target bit cleared, official `dac_flag_clear`).
    pub fn clear_dma_underrun(&mut self, ch: Channel) {
        let r = unsafe { &*pac::DAC::PTR };
        let cur = r.sts().read().bits();
        let bit = match ch {
            Channel::Ch1 => 1 << 13,
            Channel::Ch2 => 1 << 29,
        };
        r.sts().write(|w| unsafe { w.bits(cur & !bit) });
    }
}
