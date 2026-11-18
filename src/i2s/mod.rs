//! I2S audio mode driver for SPI1-4 (RM 13.3) and the I2S2_EXT/I2S3_EXT
//! full-duplex companion blocks.
//!
//! The SPI blocks switch into I2S mode via I2SCTRL.MSEL; only I2S2/I2S3 have
//! an EXT companion (slave-side second data channel) for full duplex. The
//! kernel clock is the parent SPI's APB clock (SPI1/SPI4: pclk2, SPI2/SPI3
//! and their EXTs: pclk1).
//!
//! - [`I2s`]: half-duplex master/slave TX or RX, blocking. 16-bit data
//!   formats move one DT halfword per channel frame ([`I2s::blocking_write16`]
//!   / [`I2s::blocking_read16`]); 24/32-bit data formats move two DT
//!   halfwords per channel frame ([`I2s::blocking_write32`] /
//!   [`I2s::blocking_read32`]), packed per protocol (see [`pack_sample32`]).
//! - [`I2sFullDuplex`]: main block as master + its EXT block as slave in the
//!   opposite direction ([`I2sFullDuplex::blocking_exchange16`]).
//! - Sample-rate divider per RM 13.3.5 / table 13-1 ([`Config::divider`]):
//!   MCK = 256x fs (128x for PCM); CK = MCK/8 (16-bit channels) or /4
//!   (32-bit channels) when MCK is on, otherwise SCLK/(32|64 or 16|32)x fs.
//! - Flags shared with SPI mode: TDBE/RDBF/ROERR plus TUERR (TX underflow)
//!   and ACS (channel side). No CRC in I2S mode.
//!
//! Pin mapping (RM tables 6-1..6-7, verified 2026-11-18): SD = the SPIx_MOSI
//! pins, WS = the SPIx_CS pins, CK = the SPIx_SCK pins, EXTSD has its own
//! mux rows (PB14(6)/PC2(6) for I2S2, PB4(7)/PC11(5) for I2S3), MCK has
//! dedicated rows (e.g. PB0(5)/PB6(5)/PC4(5) for I2S1).
//!
//! ```ignore
//! let mut i2s = hal::i2s::I2s::new_master_tx(
//!     p.SPI1, p.PA7, p.PA4, p.PA5, Some(p.PB0),
//!     hal::i2s::Config {
//!         protocol: hal::i2s::Protocol::Philips,
//!         format: hal::i2s::DataFormat::Data16Channel32,
//!         sample_rate: 48_000,
//!         mclk_output: true,
//!         ..Default::default()
//!     },
//! );
//! i2s.blocking_write16(&samples)?;
//! ```

mod full_duplex;
pub use full_duplex::I2sFullDuplex;

mod dma_transfer;

use embassy_hal_internal::Peri;

use crate::dma::{self, ChannelInstance};
use crate::gpio::{AfType, OutputType, Pull, Speed};
use crate::pac;
use crate::peripheral::RccPeripheral;
use crate::time::Hertz;

/// I2S audio protocol (I2SCTRL.STDSEL + PCMFSSEL).
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
pub enum Protocol {
    /// Philips standard (WS delayed one CK from the data edge).
    #[default]
    Philips,
    /// MSB-justified (left-aligned) standard.
    Msb,
    /// LSB-justified (right-aligned) standard.
    Lsb,
    /// PCM standard, short frame sync.
    PcmShort,
    /// PCM standard, long frame sync.
    PcmLong,
}

impl Protocol {
    const fn stdsel(self) -> u8 {
        match self {
            Protocol::Philips => 0,
            Protocol::Msb => 1,
            Protocol::Lsb => 2,
            Protocol::PcmShort | Protocol::PcmLong => 3,
        }
    }

    const fn pcm_long(self) -> bool {
        matches!(self, Protocol::PcmLong)
    }

    /// PCM family (affects the divider multiplier and MCK ratio).
    pub const fn is_pcm(self) -> bool {
        matches!(self, Protocol::PcmShort | Protocol::PcmLong)
    }
}

/// Data width / channel length combination (I2SCTRL.DBN + CBN).
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
pub enum DataFormat {
    /// 16-bit data in 16-bit channel frames (one DT write per frame).
    #[default]
    Data16Channel16,
    /// 16-bit data in 32-bit channel frames (one DT write; the other half is
    /// hardware-zero-filled per protocol alignment).
    Data16Channel32,
    /// 24-bit data in 32-bit channel frames (two DT writes per frame).
    Data24Channel32,
    /// 32-bit data in 32-bit channel frames (two DT writes per frame).
    Data32Channel32,
}

impl DataFormat {
    const fn dbn(self) -> u8 {
        match self {
            DataFormat::Data16Channel16 | DataFormat::Data16Channel32 => 0,
            DataFormat::Data24Channel32 => 1,
            DataFormat::Data32Channel32 => 2,
        }
    }

    /// 32-bit channel frames (affects the divider multiplier and CK/MCK ratio).
    pub const fn cbn(self) -> bool {
        matches!(
            self,
            DataFormat::Data16Channel32 | DataFormat::Data24Channel32 | DataFormat::Data32Channel32
        )
    }

    /// Two DT halfwords per channel frame?
    pub const fn two_writes(self) -> bool {
        matches!(
            self,
            DataFormat::Data24Channel32 | DataFormat::Data32Channel32
        )
    }
}

/// Operation mode (I2SCTRL.OPERSEL).
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
pub enum Mode {
    /// Slave transmitter.
    #[default]
    SlaveTx,
    /// Slave receiver.
    SlaveRx,
    /// Master transmitter (generates CK/WS and optionally MCK).
    MasterTx,
    /// Master receiver (generates CK/WS).
    MasterRx,
}

impl Mode {
    const fn is_master(self) -> bool {
        matches!(self, Mode::MasterTx | Mode::MasterRx)
    }

    const fn is_tx(self) -> bool {
        matches!(self, Mode::SlaveTx | Mode::MasterTx)
    }
}

/// CK steady-state level (I2SCTRL.CLKPOL). Philips standard requires the
/// CK idle low; the justified/PCM standards vary by device convention.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
pub enum ClockPolarity {
    /// CK idles low (I2S Philips default).
    #[default]
    IdleLow,
    /// CK idles high.
    IdleHigh,
}

/// I2S configuration.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct Config {
    pub protocol: Protocol,
    pub format: DataFormat,
    /// Target audio sample rate (fs). The actual rate is
    /// [`Config::divider`]'s `real_fs` (divider granularity).
    pub sample_rate: u32,
    pub clock_polarity: ClockPolarity,
    /// Output the 256x fs (128x for PCM) master clock on the MCK pin
    /// (master modes only).
    pub mclk_output: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            protocol: Protocol::Philips,
            format: DataFormat::Data16Channel16,
            sample_rate: 48_000,
            clock_polarity: ClockPolarity::IdleLow,
            mclk_output: false,
        }
    }
}

/// Computed sample-rate divider (I2SCLK.DIV9_8/DIV7_0/ODD) and the resulting
/// real sample rate.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct Divider {
    /// 10-bit divider register value (2..=1023; degenerate configs clamp to
    /// 2, the same fallback the official library uses).
    pub div: u16,
    /// ODD bit.
    pub odd: bool,
    /// Real sample rate after divider rounding.
    pub real_fs: u32,
}

impl Config {
    /// Computes the sample-rate divider for a kernel clock `sclk`
    /// (RM 13.3.5: idx = round(sclk / (multiplier x fs)); multiplier 256/128
    /// with MCK, 32/64 (16/32-bit channels) without MCK for the frame
    /// protocols and 32/64, 16/32 for PCM; idx = 2 x DIV + ODD).
    pub fn divider(&self, sclk: Hertz) -> Divider {
        let pcm = self.protocol.is_pcm();
        let ch32 = self.format.cbn();
        let num: u64 = match (pcm, self.mclk_output, ch32) {
            (false, true, _) => 256,
            (true, true, _) => 128,
            (false, false, false) => 32,
            (false, false, true) => 64,
            (true, false, false) => 16,
            (true, false, true) => 32,
        };
        // Fixed-point rounding, same as the official library: idx10 =
        // round(sclk / (num x fs)) x 10.
        let idx10 = ((sclk.0 as u64 / num) * 10 / self.sample_rate.max(1) as u64) + 5;
        let idx = (idx10 / 10) as u32;
        let odd = idx & 1 != 0;
        let div = ((idx - u32::from(odd)) / 2) as u16;
        let idx_eff: u64 = if (2..=0x3FF).contains(&div) {
            (2 * div as u64) + u64::from(odd)
        } else {
            4
        };
        Divider {
            div: if idx_eff == 4 { 2 } else { div },
            odd: if idx_eff == 4 { false } else { odd },
            real_fs: (sclk.0 as u64 / (idx_eff * num)) as u32,
        }
    }
}

/// Packs one 24/32-bit sample into the two DT halfwords of a 32-bit channel
/// frame. LSB-justified right-aligns the sample; the other protocols
/// left-align it (RM 13.3.4 data-write table).
pub fn pack_sample32(sample: u32, format: DataFormat, protocol: Protocol) -> [u16; 2] {
    match format {
        DataFormat::Data32Channel32 => [sample as u16, (sample >> 16) as u16],
        DataFormat::Data24Channel32 => {
            if protocol == Protocol::Lsb {
                // right-aligned: first halfword low byte = sample[23:16]
                // (high byte hardware-zeroed), second halfword = sample[15:0]
                [((sample >> 16) & 0xFF) as u16, sample as u16]
            } else {
                // left-aligned: first halfword = sample[15:0], second
                // halfword high byte = sample[23:16] (low byte hardware-zeroed)
                [sample as u16, (((sample >> 16) & 0xFF) as u16) * 256]
            }
        }
        // 16-bit data formats use one write; the caller must not pack.
        _ => [sample as u16, 0],
    }
}

/// Unpacks one 24/32-bit sample from the two received DT halfwords (inverse
/// of [`pack_sample32`]).
pub fn unpack_sample32(hw: [u16; 2], format: DataFormat, protocol: Protocol) -> u32 {
    match format {
        DataFormat::Data32Channel32 => (hw[0] as u32) | ((hw[1] as u32) << 16),
        DataFormat::Data24Channel32 => {
            if protocol == Protocol::Lsb {
                (hw[0] as u32 & 0xFF) << 16 | hw[1] as u32
            } else {
                (hw[0] as u32) | ((hw[1] as u32 >> 8) << 16)
            }
        }
        _ => hw[0] as u32,
    }
}

/// I2S operation error.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[non_exhaustive]
pub enum Error {
    /// RX overrun (ROERR): frames were discarded.
    Overrun,
    /// TX underflow (TUERR): the shift register starved (software too slow
    /// for the running CK).
    Underflow,
    /// Polling timeout while waiting for a flag.
    Timeout,
}

#[cfg(feature = "defmt")]
impl defmt::Format for Error {
    fn format(&self, fmt: defmt::Formatter) {
        match self {
            Error::Overrun => defmt::write!(fmt, "Overrun"),
            Error::Underflow => defmt::write!(fmt, "Underflow"),
            Error::Timeout => defmt::write!(fmt, "Timeout"),
        }
    }
}

trait SealedInstance {
    fn regs() -> &'static pac::spi1::RegisterBlock;
}

/// I2S-capable instance: SPI1-4 plus the I2S2_EXT/I2S3_EXT companions (the
/// EXTs share their parent SPI's CRM gate and NVIC line).
#[allow(private_bounds)]
pub trait Instance:
    SealedInstance + RccPeripheral + embassy_hal_internal::PeripheralType + 'static
{
}

macro_rules! impl_i2s_instance {
    ($inst:ident) => {
        impl SealedInstance for crate::peripherals::$inst {
            #[inline]
            fn regs() -> &'static pac::spi1::RegisterBlock {
                unsafe { &*pac::$inst::PTR }
            }
        }
        impl Instance for crate::peripherals::$inst {}
    };
}

impl_i2s_instance!(SPI1);
impl_i2s_instance!(SPI2);
impl_i2s_instance!(SPI3);
impl_i2s_instance!(SPI4);
impl_i2s_instance!(I2S2_EXT);
impl_i2s_instance!(I2S3_EXT);

/// Marker for main blocks that own an EXT companion (I2S2/I2S3 only).
pub trait HasExt: Instance {
    /// The companion block type.
    type Ext: Instance;
}

impl HasExt for crate::peripherals::SPI2 {
    type Ext = crate::peripherals::I2S2_EXT;
}

impl HasExt for crate::peripherals::SPI3 {
    type Ext = crate::peripherals::I2S3_EXT;
}

crate::pin_trait!(I2sCkPin, Instance);
crate::pin_trait!(I2sWsPin, Instance);
crate::pin_trait!(I2sSdPin, Instance);
crate::pin_trait!(I2sMckPin, Instance);
crate::pin_trait!(I2sExtSdPin, Instance);

crate::dma_trait!(TxDma, Instance);
crate::dma_trait!(RxDma, Instance);

// ---------------------------------------------------------------------------
// AF pin tables (mux numbers verified against RM V2.09 tables 6-1..6-7,
// page-image read 2026-11-18; [V] = also reachable on the evb, [I] = table
// row only)
// ---------------------------------------------------------------------------

// I2S1 (SPI1)
crate::pin_trait_impl!(crate::i2s::I2sCkPin, SPI1, PA5, 5); // [I]
crate::pin_trait_impl!(crate::i2s::I2sCkPin, SPI1, PB3, 5); // [I]
crate::pin_trait_impl!(crate::i2s::I2sCkPin, SPI1, PE13, 4); // [I]
crate::pin_trait_impl!(crate::i2s::I2sWsPin, SPI1, PA4, 5); // [V] evb test
crate::pin_trait_impl!(crate::i2s::I2sWsPin, SPI1, PA15, 5); // [I]
crate::pin_trait_impl!(crate::i2s::I2sWsPin, SPI1, PE12, 4); // [I]
crate::pin_trait_impl!(crate::i2s::I2sSdPin, SPI1, PA7, 5); // [V] evb test
crate::pin_trait_impl!(crate::i2s::I2sSdPin, SPI1, PB5, 5); // [I]
crate::pin_trait_impl!(crate::i2s::I2sSdPin, SPI1, PE15, 4); // [I]
crate::pin_trait_impl!(crate::i2s::I2sSdPin, SPI1, PG1, 5); // [I]
crate::pin_trait_impl!(crate::i2s::I2sMckPin, SPI1, PB0, 5); // [V] evb test
crate::pin_trait_impl!(crate::i2s::I2sMckPin, SPI1, PB6, 5); // [I]
crate::pin_trait_impl!(crate::i2s::I2sMckPin, SPI1, PC4, 5); // [I]

// I2S2 (SPI2)
crate::pin_trait_impl!(crate::i2s::I2sCkPin, SPI2, PA9, 5); // [I]
crate::pin_trait_impl!(crate::i2s::I2sCkPin, SPI2, PB10, 5); // [I]
crate::pin_trait_impl!(crate::i2s::I2sCkPin, SPI2, PB13, 5); // [I]
crate::pin_trait_impl!(crate::i2s::I2sCkPin, SPI2, PC7, 5); // [I]
crate::pin_trait_impl!(crate::i2s::I2sCkPin, SPI2, PD1, 6); // [I]
crate::pin_trait_impl!(crate::i2s::I2sCkPin, SPI2, PD3, 5); // [I]
crate::pin_trait_impl!(crate::i2s::I2sWsPin, SPI2, PA11, 5); // [I]
crate::pin_trait_impl!(crate::i2s::I2sWsPin, SPI2, PB9, 5); // [I]
crate::pin_trait_impl!(crate::i2s::I2sWsPin, SPI2, PB12, 5); // [I] evb flash CS net (avoid: flash)
crate::pin_trait_impl!(crate::i2s::I2sWsPin, SPI2, PD0, 7); // [I]
crate::pin_trait_impl!(crate::i2s::I2sWsPin, SPI2, PD1, 7); // [I]
crate::pin_trait_impl!(crate::i2s::I2sSdPin, SPI2, PA10, 5); // [I]
crate::pin_trait_impl!(crate::i2s::I2sSdPin, SPI2, PB15, 5); // [I] evb flash MOSI net (avoid: flash)
crate::pin_trait_impl!(crate::i2s::I2sSdPin, SPI2, PC1, 7); // [I]
crate::pin_trait_impl!(crate::i2s::I2sSdPin, SPI2, PC3, 5); // [I]
crate::pin_trait_impl!(crate::i2s::I2sSdPin, SPI2, PD4, 6); // [I]
crate::pin_trait_impl!(crate::i2s::I2sMckPin, SPI2, PA3, 5); // [I]
crate::pin_trait_impl!(crate::i2s::I2sMckPin, SPI2, PA6, 6); // [I]
crate::pin_trait_impl!(crate::i2s::I2sMckPin, SPI2, PC6, 5); // [I]
crate::pin_trait_impl!(crate::i2s::I2sExtSdPin, I2S2_EXT, PB14, 6); // [I] evb flash MISO net (avoid: flash)
crate::pin_trait_impl!(crate::i2s::I2sExtSdPin, I2S2_EXT, PC2, 6); // [I]

// I2S3 (SPI3)
crate::pin_trait_impl!(crate::i2s::I2sCkPin, SPI3, PB3, 6); // [V] evb test
crate::pin_trait_impl!(crate::i2s::I2sCkPin, SPI3, PB12, 7); // [I] evb flash CS net (avoid: flash)
crate::pin_trait_impl!(crate::i2s::I2sCkPin, SPI3, PC10, 6); // [I]
crate::pin_trait_impl!(crate::i2s::I2sWsPin, SPI3, PA4, 6); // [I]
crate::pin_trait_impl!(crate::i2s::I2sWsPin, SPI3, PA15, 6); // [V] evb test
crate::pin_trait_impl!(crate::i2s::I2sSdPin, SPI3, PB5, 6); // [V] evb test
crate::pin_trait_impl!(crate::i2s::I2sSdPin, SPI3, PB2, 7); // [I]
crate::pin_trait_impl!(crate::i2s::I2sSdPin, SPI3, PC1, 5); // [I]
crate::pin_trait_impl!(crate::i2s::I2sSdPin, SPI3, PC12, 6); // [I]
crate::pin_trait_impl!(crate::i2s::I2sSdPin, SPI3, PD6, 5); // [I]
crate::pin_trait_impl!(crate::i2s::I2sMckPin, SPI3, PB10, 6); // [V] evb test
crate::pin_trait_impl!(crate::i2s::I2sMckPin, SPI3, PC7, 6); // [I]
crate::pin_trait_impl!(crate::i2s::I2sExtSdPin, I2S3_EXT, PB4, 7); // [V] evb test (jumper to PB5 for loopback)
crate::pin_trait_impl!(crate::i2s::I2sExtSdPin, I2S3_EXT, PC11, 5); // [I]

// I2S4 (SPI4, no EXT companion)
crate::pin_trait_impl!(crate::i2s::I2sCkPin, SPI4, PB7, 6); // [I]
crate::pin_trait_impl!(crate::i2s::I2sCkPin, SPI4, PB13, 6); // [I]
crate::pin_trait_impl!(crate::i2s::I2sCkPin, SPI4, PE2, 5); // [I]
crate::pin_trait_impl!(crate::i2s::I2sCkPin, SPI4, PG11, 6); // [I]
crate::pin_trait_impl!(crate::i2s::I2sWsPin, SPI4, PB6, 6); // [I]
crate::pin_trait_impl!(crate::i2s::I2sWsPin, SPI4, PB12, 6); // [I]
crate::pin_trait_impl!(crate::i2s::I2sWsPin, SPI4, PE4, 5); // [I]
crate::pin_trait_impl!(crate::i2s::I2sWsPin, SPI4, PE11, 5); // [I]
crate::pin_trait_impl!(crate::i2s::I2sWsPin, SPI4, PG14, 6); // [I]
crate::pin_trait_impl!(crate::i2s::I2sSdPin, SPI4, PB9, 6); // [I]
crate::pin_trait_impl!(crate::i2s::I2sSdPin, SPI4, PA1, 5); // [I]
crate::pin_trait_impl!(crate::i2s::I2sSdPin, SPI4, PE6, 5); // [I]
crate::pin_trait_impl!(crate::i2s::I2sSdPin, SPI4, PE14, 5); // [I]
crate::pin_trait_impl!(crate::i2s::I2sSdPin, SPI4, PG13, 6); // [I]
crate::pin_trait_impl!(crate::i2s::I2sMckPin, SPI4, PA10, 6); // [I]
crate::pin_trait_impl!(crate::i2s::I2sMckPin, SPI4, PC8, 5); // [I]

// ---------------------------------------------------------------------------
// Register helpers (shared by the blocking paths)
// ---------------------------------------------------------------------------

/// Loop-count bound for flag waits (same budget as the spi/i2c drivers).
const DEFAULT_SPIN: u32 = 1 << 24;

const STS_RDBF: u32 = 1 << 0;
const STS_ACS: u32 = 1 << 2;
const STS_TUERR: u32 = 1 << 3;
const STS_ROERR: u32 = 1 << 6;
const STS_BF: u32 = 1 << 7;

/// Clears ROERR with the SPI-mode sequence: read DT, then read STS.
fn clear_overrun(r: &pac::spi1::RegisterBlock) {
    let _ = r.dt().read();
    let _ = r.sts().read();
}

/// Waits for RDBF; reports overrun.
fn wait_rdbf(r: &pac::spi1::RegisterBlock, left: &mut u32) -> Result<(), Error> {
    while r.sts().read().bits() & STS_RDBF == 0 {
        if *left == 0 {
            return Err(Error::Timeout);
        }
        *left -= 1;
        if r.sts().read().bits() & STS_ROERR != 0 {
            clear_overrun(r);
            return Err(Error::Overrun);
        }
    }
    Ok(())
}

/// Waits for one channel-frame boundary (an ACS flip). This is the only
/// usable TX pacing signal: TDBE latches DT into the next-frame buffer
/// immediately (it reads 1 throughout a TX burst, hw-verified), so
/// back-to-back DT writes overwrite the pending frame and skip samples.
/// Note: reading STS clears TUERR, so the underflow check must be done by
/// the caller on its own single read.
fn wait_frame_boundary(r: &pac::spi1::RegisterBlock, left: &mut u32) -> Result<(), Error> {
    let a0 = r.sts().read().bits() & STS_ACS != 0;
    loop {
        let sts = r.sts().read().bits();
        if (sts & STS_ACS != 0) != a0 {
            return Ok(());
        }
        if *left == 0 {
            return Err(Error::Timeout);
        }
        *left -= 1;
        if sts & STS_ROERR != 0 {
            clear_overrun(r);
            return Err(Error::Overrun);
        }
    }
}

/// Register-level I2S setup shared by all constructors: called with the
/// peripheral clock enabled and both I2SEN and SPIEN cleared.
fn configure(r: &pac::spi1::RegisterBlock, sclk: Hertz, mode: Mode, config: &Config, mck: bool) {
    // Make sure the block is idle in both personalities (RM: I2S and SPI
    // share the enable domain; reconfiguring MSEL requires the block off).
    r.ctrl1().modify(|_, w| w.spien().disable());
    r.i2sctrl().modify(|_, w| w.en().clear_bit());
    // Leave I2S mode... no - ENTER I2S mode (MSEL = 1) with all selectors.
    r.i2sctrl().modify(|_, w| {
        match config.format.cbn() {
            false => w.cbn().bit16(),
            true => w.cbn().bit32(),
        };
        match config.format.dbn() {
            0 => w.dbn().bit16(),
            1 => w.dbn().bit24(),
            _ => w.dbn().bit32(),
        };
        match config.clock_polarity {
            ClockPolarity::IdleLow => w.clkpol().low(),
            ClockPolarity::IdleHigh => w.clkpol().high(),
        };
        unsafe {
            w.stdsel().bits(config.protocol.stdsel());
        }
        match config.protocol.pcm_long() {
            false => w.pcmfssel().short(),
            true => w.pcmfssel().long(),
        };
        match mode {
            Mode::SlaveTx => w.opersel().slave_trasmission(),
            Mode::SlaveRx => w.opersel().slave_reception(),
            Mode::MasterTx => w.opersel().master_trasmission(),
            Mode::MasterRx => w.opersel().master_reception(),
        };
        w.msel().set_bit();
        w
    });
    let divider = config.divider(sclk);
    r.i2sclk().modify(|_, w| {
        match divider.odd {
            false => w.odd().double(),
            true => w.odd().double_plus_one(),
        };
        unsafe {
            w.div7_0().bits((divider.div & 0xFF) as u8);
            w.div9_8().bits(((divider.div >> 8) & 0x3) as u8);
        }
        match mck {
            false => w.mclkoe().disable(),
            true => w.mclkoe().enable(),
        };
        w
    });
}

/// Data register address (DMA peripheral side).
fn dt_addr(r: &pac::spi1::RegisterBlock) -> u32 {
    r.dt() as *const _ as u32
}

// ---------------------------------------------------------------------------
// I2s: half-duplex blocking driver
// ---------------------------------------------------------------------------

/// Blocking I2S endpoint (master or slave, TX or RX).
pub struct I2s<'d, T: Instance> {
    _peri: Peri<'d, T>,
    _sd: Peri<'d, crate::gpio::AnyPin>,
    _ws: Peri<'d, crate::gpio::AnyPin>,
    _ck: Peri<'d, crate::gpio::AnyPin>,
    _mck: Option<Peri<'d, crate::gpio::AnyPin>>,
    mode: Mode,
    config: Config,
}

impl<'d, T: Instance> I2s<'d, T> {
    /// Master transmitter: drives SD (data out), WS and CK; no MCK output.
    pub fn new_master_tx(
        peri: Peri<'d, T>,
        sd: Peri<'d, impl I2sSdPin<T>>,
        ws: Peri<'d, impl I2sWsPin<T>>,
        ck: Peri<'d, impl I2sCkPin<T>>,
        config: Config,
    ) -> Self {
        Self::new_inner(peri, sd, ws, ck, None, Mode::MasterTx, config)
    }

    /// Master transmitter with the 256x fs (128x PCM) master clock output on
    /// the MCK pin (overrides `config.mclk_output`).
    pub fn new_master_tx_with_mck(
        peri: Peri<'d, T>,
        sd: Peri<'d, impl I2sSdPin<T>>,
        ws: Peri<'d, impl I2sWsPin<T>>,
        ck: Peri<'d, impl I2sCkPin<T>>,
        mck: Peri<'d, impl I2sMckPin<T>>,
        config: Config,
    ) -> Self {
        let mck_pin = new_pin!(
            mck,
            AfType::output(OutputType::PushPull, Speed::High.into(), Pull::None)
        );
        Self::new_inner(peri, sd, ws, ck, mck_pin, Mode::MasterTx, config)
    }

    /// Master receiver: drives WS/CK, SD is an input; no MCK output.
    pub fn new_master_rx(
        peri: Peri<'d, T>,
        sd: Peri<'d, impl I2sSdPin<T>>,
        ws: Peri<'d, impl I2sWsPin<T>>,
        ck: Peri<'d, impl I2sCkPin<T>>,
        config: Config,
    ) -> Self {
        Self::new_inner(peri, sd, ws, ck, None, Mode::MasterRx, config)
    }

    /// Master receiver with the master clock output on the MCK pin.
    pub fn new_master_rx_with_mck(
        peri: Peri<'d, T>,
        sd: Peri<'d, impl I2sSdPin<T>>,
        ws: Peri<'d, impl I2sWsPin<T>>,
        ck: Peri<'d, impl I2sCkPin<T>>,
        mck: Peri<'d, impl I2sMckPin<T>>,
        config: Config,
    ) -> Self {
        let mck_pin = new_pin!(
            mck,
            AfType::output(OutputType::PushPull, Speed::High.into(), Pull::None)
        );
        Self::new_inner(peri, sd, ws, ck, mck_pin, Mode::MasterRx, config)
    }

    /// Slave transmitter: SD is an output, WS/CK are inputs from the master.
    pub fn new_slave_tx(
        peri: Peri<'d, T>,
        sd: Peri<'d, impl I2sSdPin<T>>,
        ws: Peri<'d, impl I2sWsPin<T>>,
        ck: Peri<'d, impl I2sCkPin<T>>,
        config: Config,
    ) -> Self {
        Self::new_inner(peri, sd, ws, ck, None, Mode::SlaveTx, config)
    }

    /// Slave receiver: SD, WS and CK are inputs from the master.
    pub fn new_slave_rx(
        peri: Peri<'d, T>,
        sd: Peri<'d, impl I2sSdPin<T>>,
        ws: Peri<'d, impl I2sWsPin<T>>,
        ck: Peri<'d, impl I2sCkPin<T>>,
        config: Config,
    ) -> Self {
        Self::new_inner(peri, sd, ws, ck, None, Mode::SlaveRx, config)
    }

    fn new_inner(
        peri: Peri<'d, T>,
        sd: Peri<'d, impl I2sSdPin<T>>,
        ws: Peri<'d, impl I2sWsPin<T>>,
        ck: Peri<'d, impl I2sCkPin<T>>,
        mck_pin: Option<Peri<'d, crate::gpio::AnyPin>>,
        mode: Mode,
        config: Config,
    ) -> Self {
        let out_af = || AfType::output(OutputType::PushPull, Speed::High.into(), Pull::None);
        let in_af = || AfType::input(Pull::None);
        let sd_pin = new_pin!(sd, if mode.is_tx() { out_af() } else { in_af() });
        let ws_pin = new_pin!(ws, if mode.is_master() { out_af() } else { in_af() });
        let ck_pin = new_pin!(ck, if mode.is_master() { out_af() } else { in_af() });
        T::enable_and_reset();
        let sclk = T::frequency();
        let r = T::regs();
        configure(
            r,
            sclk,
            mode,
            &config,
            config.mclk_output && mck_pin.is_some(),
        );
        r.i2sctrl().modify(|_, w| w.en().set_bit());
        Self {
            _peri: peri,
            _sd: sd_pin.unwrap(),
            _ws: ws_pin.unwrap(),
            _ck: ck_pin.unwrap(),
            _mck: mck_pin,
            mode,
            config,
        }
    }

    /// Configured mode.
    pub fn mode(&self) -> Mode {
        self.mode
    }

    /// The configuration (use [`Config::divider`] with [`RccPeripheral::frequency`]
    /// for the computed real sample rate).
    pub fn config(&self) -> Config {
        self.config
    }

    /// Channel side of the next frame: false = left, true = right (STS.ACS).
    pub fn channel_right(&self) -> bool {
        T::regs().sts().read().bits() & STS_ACS != 0
    }

    /// TX underflow flag (STS.TUERR, read-and-clear: any STS read clears it).
    /// Meaningful on slave TX endpoints (the master's CK runs regardless of
    /// software); on master TX the clock is demand-gated on pending data, so
    /// underflow cannot happen there (hw-verified).
    pub fn underflow(&self) -> bool {
        let r = T::regs();
        let u = r.sts().read().bits() & STS_TUERR != 0;
        if u {
            clear_overrun(r);
        }
        u
    }

    /// Blocking transmit of 16-bit samples (valid for the Data16Channel16 and
    /// Data16Channel32 formats: one DT halfword per channel frame; the
    /// hardware zero-fills the unused half per protocol alignment).
    pub fn blocking_write16(&mut self, samples: &[u16]) -> Result<(), Error> {
        if self.config.format.two_writes() {
            return Err(Error::Underflow);
        }
        let r = T::regs();
        let mut left = DEFAULT_SPIN;
        for (i, &s) in samples.iter().enumerate() {
            if i > 0 {
                // One channel frame per write: wait the boundary so the
                // previous value shifts out before the next one latches.
                wait_frame_boundary(r, &mut left)?;
            }
            r.dt().write(|w| unsafe { w.bits(s) });
        }
        self.wait_tx_idle()
    }

    /// Blocking transmit of 24/32-bit samples (Data24Channel32 /
    /// Data32Channel32 formats: packed into two DT halfwords per channel
    /// frame via [`pack_sample32`]).
    pub fn blocking_write32(&mut self, samples: &[u32]) -> Result<(), Error> {
        if !self.config.format.two_writes() {
            return Err(Error::Underflow);
        }
        let r = T::regs();
        let mut left = DEFAULT_SPIN;
        for (i, &s) in samples.iter().enumerate() {
            if i > 0 {
                // Both halfwords of one channel frame go out back-to-back
                // (they must land inside the same frame window); the frame
                // boundary wait goes between samples.
                wait_frame_boundary(r, &mut left)?;
            }
            for hw in pack_sample32(s, self.config.format, self.config.protocol) {
                r.dt().write(|w| unsafe { w.bits(hw) });
            }
        }
        self.wait_tx_idle()
    }

    /// Blocking receive of 16-bit samples (16-bit data formats).
    pub fn blocking_read16(&mut self, out: &mut [u16]) -> Result<(), Error> {
        if self.config.format.two_writes() {
            return Err(Error::Underflow);
        }
        let r = T::regs();
        let mut left = DEFAULT_SPIN;
        for o in out.iter_mut() {
            wait_rdbf(r, &mut left)?;
            *o = r.dt().read().bits();
        }
        Ok(())
    }

    /// Blocking receive of 24/32-bit samples (two halfwords per frame,
    /// unpacked via [`unpack_sample32`]).
    pub fn blocking_read32(&mut self, out: &mut [u32]) -> Result<(), Error> {
        if !self.config.format.two_writes() {
            return Err(Error::Underflow);
        }
        let r = T::regs();
        let mut left = DEFAULT_SPIN;
        for o in out.iter_mut() {
            let mut hw = [0u16; 2];
            for slot in hw.iter_mut() {
                wait_rdbf(r, &mut left)?;
                *slot = r.dt().read().bits();
            }
            *o = unpack_sample32(hw, self.config.format, self.config.protocol);
        }
        Ok(())
    }

    /// Best-effort wait for the transmitter to drain (BF clear). TDBE reads
    /// 1 throughout I2S TX (the DT latch is immediate, hw-verified) and the
    /// master TX clock is demand-gated on pending data, so this only waits
    /// out a possibly-still-shifting last frame; BF may legitimately never
    /// set between frames.
    pub fn wait_tx_idle(&self) -> Result<(), Error> {
        if !self.mode.is_tx() {
            return Ok(());
        }
        let r = T::regs();
        let mut left = 1 << 16;
        while r.sts().read().bits() & STS_BF != 0 {
            if left == 0 {
                return Err(Error::Timeout);
            }
            left -= 1;
        }
        Ok(())
    }

    /// Disables the I2S block (I2SEN = 0; the block keeps its configuration
    /// for re-enabling). For TX the caller should [`I2s::wait_tx_idle`] (or a
    /// blocking write that returns) first; the RM's per-protocol CK-period
    /// waits before shutdown are covered by the BF/TDBE wait plus the register
    /// write latency at audio rates.
    pub fn disable(&mut self) {
        T::regs().i2sctrl().modify(|_, w| w.en().clear_bit());
    }

    /// Re-enables the I2S block after [`I2s::disable`].
    pub fn enable(&mut self) {
        T::regs().i2sctrl().modify(|_, w| w.en().set_bit());
    }

    /// Reconfigures protocol/format/sample rate in place (I2SEN off, fields
    /// rewritten, I2SEN on; pins and mode unchanged). MCK output requires the
    /// MCK pin to be present at construction (`config.mclk_output` alone is
    /// not enough).
    pub fn reconfigure(&mut self, config: Config) {
        let r = T::regs();
        r.i2sctrl().modify(|_, w| w.en().clear_bit());
        configure(
            r,
            T::frequency(),
            self.mode,
            &config,
            config.mclk_output && self._mck.is_some(),
        );
        r.i2sctrl().modify(|_, w| w.en().set_bit());
        self.config = config;
    }
}
