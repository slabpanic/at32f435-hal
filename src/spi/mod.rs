//! SPI master driver: SPI1-4 (RM 13, ch32-hal `spi/` structure).
//!
//! - [`Spi`]: blocking, byte-at-a-time polling driver implementing
//!   embedded-hal 1.0 `SpiBus` (also usable as the register-level core of the
//!   DMA driver).
//! - [`SpiDma`]: DMA-driven driver (async `write`/`read`/`transfer` +
//!   blocking fallbacks). The DMA channel IRQs must be bound with
//!   `bind_interrupts!`; the driver enables them itself.
//! - Software CS: the driver configures the controller for software NSS with
//!   the internal level high (mandatory for master operation, otherwise the
//!   controller raises MMERR); chip-select GPIOs are owned by the application
//!   (or an `embedded-hal-bus` `SpiDevice` on top of the blocking `SpiBus`).
//! - Hardware CS (`new_blocking_hwcs`/`new_hwcs`): the controller drives the
//!   `SPIx_CS` AF pin low while SPIEN is set and releases it after SPIEN is
//!   cleared and the transmit completes (RM 13.2.3). [`Spi::cs_assert`] /
//!   [`Spi::cs_release`] (or [`Spi::blocking_transaction`]) frame one bus
//!   transaction; the `SpiBus` impl asserts/releases around every method call.
//! - CRC transfer (RM 13.2.5): `set_crc_polynomial` + `enable_crc` arm the
//!   hardware CRC units. On the blocking driver
//!   [`Spi::blocking_write_with_crc`] appends the TCRC frame after the data
//!   (NTC handled internally); on the DMA driver the hardware appends and
//!   checks the CRC automatically when CCEN is set. `crc_tx_value` /
//!   `crc_rx_value` read the accumulators, `crc_error` reports CCERR.
//! - Divider: MDIV picks /2../256, /512, /1024 from PCLK, plus the /3 path
//!   (MDIV3EN). The constructor picks the divider that meets the requested
//!   SCK frequency (actual SCK never higher than requested).
//! - ROERR (receive overrun) is cleared with the read-DT-then-read-STS
//!   sequence (RM 13.2.11).
//!
//! ```ignore
//! let mut spi = hal::spi::Spi::new_blocking(
//!     p.SPI1, p.PA5, p.PA6, p.PA7, hal::spi::Config::default(),
//! );
//! spi.blocking_write(&[0xA5, 0x5A])?;
//! ```
//!
//! DMA example:
//! ```ignore
//! bind_interrupts!(struct Irqs {
//!     DMA1_CHANNEL4 => hal::dma::InterruptHandler<hal::peripherals::DMA1_CH4>;
//!     DMA1_CHANNEL5 => hal::dma::InterruptHandler<hal::peripherals::DMA1_CH5>;
//! });
//! let mut spi = hal::spi::SpiDma::new(
//!     p.SPI1, p.PA5, p.PA6, p.PA7,
//!     p.DMA1_CH5, p.DMA1_CH4, hal::spi::Config::default(),
//! );
//! // SAFETY: tx is in DMA-accessible memory and remains valid until completion.
//! unsafe { spi.write(&tx).await? };
//! ```

mod dma;
mod pins;
pub use dma::SpiDma;

use embassy_hal_internal::Peri;

use crate::dma::{Channel, Config as DmaConfig, DmaError, Width};
use crate::gpio::{AfType, OutputType, Pull, Speed};
use crate::pac;
use crate::peripheral::RccPeripheral;
use crate::time::Hertz;

/// SPI mode types (same as the usart driver).
pub mod mode {
    /// Blocking mode (byte-at-a-time polling).
    pub struct Blocking;
    /// Async mode (DMA-driven; see [`super::SpiDma`]).
    pub struct Async;

    pub trait SealedMode {}
    impl SealedMode for Blocking {}
    impl SealedMode for Async {}

    /// Mode marker trait.
    pub trait Mode: SealedMode {}
    impl Mode for Blocking {}
    impl Mode for Async {}
}

/// SPI clock polarity/phase combination (MODE0..MODE3).
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
pub enum Mode {
    /// CPOL = 0, CPHA = 0 (clock idle low, sample on first edge).
    #[default]
    Mode0,
    /// CPOL = 0, CPHA = 1 (clock idle low, sample on second edge).
    Mode1,
    /// CPOL = 1, CPHA = 0 (clock idle high, sample on second edge).
    Mode2,
    /// CPOL = 1, CPHA = 1 (clock idle high, sample on first edge).
    Mode3,
}

/// Bit transmission order.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
pub enum BitOrder {
    /// MSB first (reset default).
    #[default]
    MsbFirst,
    /// LSB first.
    LsbFirst,
}

/// Frame size.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
pub enum FrameSize {
    /// 8-bit frames (reset default).
    #[default]
    Bits8,
    /// 16-bit frames.
    Bits16,
}

impl FrameSize {
    /// Data width for DMA transfers.
    pub(crate) const fn dma_width(self) -> Width {
        match self {
            FrameSize::Bits8 => Width::Bits8,
            FrameSize::Bits16 => Width::Bits16,
        }
    }
}

/// SPI configuration.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct Config {
    /// Clock polarity/phase.
    pub mode: Mode,
    /// Bit order.
    pub bit_order: BitOrder,
    /// Frame size (8/16 bit).
    pub frame_size: FrameSize,
    /// Target SCK frequency. The actual SCK is the highest PCLK/MDIV value
    /// that does not exceed this (MDIV = 2/3/4/../256/512/1024).
    pub sck_freq: Hertz,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            mode: Mode::Mode0,
            bit_order: BitOrder::MsbFirst,
            frame_size: FrameSize::Bits8,
            sck_freq: Hertz(1_000_000),
        }
    }
}

/// SPI transfer error.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[non_exhaustive]
pub enum Error {
    /// Receive overrun (ROERR): data beyond the requested size was discarded.
    Overrun,
    /// Mode fault (MMERR): NSS pulled low while operating as master.
    ModeFault,
    /// A DMA buffer is not aligned to the configured frame width.
    InvalidBuffer,
    /// Unequal nonempty TX/RX buffers would require two hardware CRC frames.
    UnequalBuffersWithCrc,
    /// A DMA transfer failed.
    Dma(DmaError),
}

impl From<DmaError> for Error {
    fn from(e: DmaError) -> Self {
        Error::Dma(e)
    }
}

impl embedded_hal::spi::Error for Error {
    fn kind(&self) -> embedded_hal::spi::ErrorKind {
        match self {
            Error::Overrun => embedded_hal::spi::ErrorKind::Overrun,
            Error::ModeFault => embedded_hal::spi::ErrorKind::ModeFault,
            Error::InvalidBuffer | Error::UnequalBuffersWithCrc | Error::Dma(_) => {
                embedded_hal::spi::ErrorKind::Other
            }
        }
    }
}

trait SealedInstance {
    fn regs() -> &'static pac::spi1::RegisterBlock;
}

/// SPI instance trait.
#[allow(private_bounds)]
pub trait Instance:
    SealedInstance + RccPeripheral + embassy_hal_internal::PeripheralType + 'static
{
}

macro_rules! impl_spi_instance {
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

impl_spi_instance!(SPI1);
impl_spi_instance!(SPI2);
impl_spi_instance!(SPI3);
impl_spi_instance!(SPI4);

crate::pin_trait!(SckPin, Instance);
crate::pin_trait!(MisoPin, Instance);
crate::pin_trait!(MosiPin, Instance);
crate::pin_trait!(CsPin, Instance);

dma_trait!(TxDma, Instance);
dma_trait!(RxDma, Instance);

/// DMAMUX routing impls: any physical channel can serve any request line, so
/// every (instance, channel) pair is wired with the instance's request ID.
macro_rules! impl_spi_dma_routes {
    ($($ch:ident),*) => {
        $(
            impl TxDma<crate::peripherals::SPI1> for crate::peripherals::$ch {
                #[inline] fn request() -> crate::dma::Request { crate::dma::Request::Spi1Tx }
            }
            impl RxDma<crate::peripherals::SPI1> for crate::peripherals::$ch {
                #[inline] fn request() -> crate::dma::Request { crate::dma::Request::Spi1Rx }
            }
            impl TxDma<crate::peripherals::SPI2> for crate::peripherals::$ch {
                #[inline] fn request() -> crate::dma::Request { crate::dma::Request::Spi2Tx }
            }
            impl RxDma<crate::peripherals::SPI2> for crate::peripherals::$ch {
                #[inline] fn request() -> crate::dma::Request { crate::dma::Request::Spi2Rx }
            }
            impl TxDma<crate::peripherals::SPI3> for crate::peripherals::$ch {
                #[inline] fn request() -> crate::dma::Request { crate::dma::Request::Spi3Tx }
            }
            impl RxDma<crate::peripherals::SPI3> for crate::peripherals::$ch {
                #[inline] fn request() -> crate::dma::Request { crate::dma::Request::Spi3Rx }
            }
            impl TxDma<crate::peripherals::SPI4> for crate::peripherals::$ch {
                #[inline] fn request() -> crate::dma::Request { crate::dma::Request::Spi4Tx }
            }
            impl RxDma<crate::peripherals::SPI4> for crate::peripherals::$ch {
                #[inline] fn request() -> crate::dma::Request { crate::dma::Request::Spi4Rx }
            }
        )*
    };
}

impl_spi_dma_routes!(
    DMA1_CH1, DMA1_CH2, DMA1_CH3, DMA1_CH4, DMA1_CH5, DMA1_CH6, DMA1_CH7, //
    DMA2_CH1, DMA2_CH2, DMA2_CH3, DMA2_CH4, DMA2_CH5, DMA2_CH6, DMA2_CH7
);

/// Register-level setup shared by all constructors: called with the peripheral
/// clock enabled and the controller disabled (SPIEN = 0). `hw_cs` selects the
/// hardware CS output (HWCSOE = 1, SWCSEN = 0, RM 13.2.3) instead of the
/// software CS with internal level high.
fn configure(r: &pac::spi1::RegisterBlock, config: &Config, pclk: Hertz, hw_cs: bool) {
    // Leave I2S mode (reset default; kept for clarity, matches spi_init()).
    r.i2sctrl().modify(|_, w| {
        w.msel().clear_bit();
        w.en().clear_bit();
        w
    });

    // Divider: smallest MDIV value with pclk/div <= requested sck_freq
    // (so the actual SCK never exceeds the request).
    // MDIV encoding (CTRL1[5:3] + CTRL2[8]): 0=/2, 1=/4 ... 7=/256;
    // CTRL2[8] (MDIV[3]) set adds /512 (0b1000) and /1024 (0b1001);
    // MDIV3EN (CTRL2 bit9) forces /3.
    const DIVISORS: [(u32, u8, bool, bool); 11] = [
        // (div, mdiv_l, mdiv_h, mdiv3en)
        (2, 0, false, false),
        (3, 0, false, true),
        (4, 1, false, false),
        (8, 2, false, false),
        (16, 3, false, false),
        (32, 4, false, false),
        (64, 5, false, false),
        (128, 6, false, false),
        (256, 7, false, false),
        (512, 0, true, false),
        (1024, 1, true, false),
    ];
    let (_, mdiv_l, mdiv_h, mdiv3en) = DIVISORS
        .iter()
        .find(|(d, _, _, _)| pclk.0 / d <= config.sck_freq.0)
        .copied()
        .unwrap_or((1024, 1, true, false));

    r.ctrl1().modify(|_, w| {
        match config.mode {
            Mode::Mode0 | Mode::Mode1 => w.clkpol().clear_bit(),
            Mode::Mode2 | Mode::Mode3 => w.clkpol().set_bit(),
        };
        match config.mode {
            Mode::Mode0 | Mode::Mode2 => w.clkpha().clear_bit(),
            Mode::Mode1 | Mode::Mode3 => w.clkpha().set_bit(),
        };
        match config.bit_order {
            BitOrder::MsbFirst => w.ltf().clear_bit(),
            BitOrder::LsbFirst => w.ltf().set_bit(),
        };
        match config.frame_size {
            FrameSize::Bits8 => w.fbn().clear_bit(),
            FrameSize::Bits16 => w.fbn().set_bit(),
        };
        unsafe {
            w.mdiv2_0().bits(mdiv_l);
        }
        w
    });
    r.ctrl2().modify(|_, w| {
        if mdiv3en {
            w.mdiv3en().set_bit();
        } else {
            w.mdiv3en().clear_bit();
        }
        w.mdiv3().bit(mdiv_h);
        // Hardware CS output drives the SPIx_CS pin while SPIEN is set;
        // otherwise CS pins are application-owned (software CS).
        if hw_cs {
            w.hwcsoe().set_bit();
        } else {
            w.hwcsoe().clear_bit();
        }
        w
    });
    r.ctrl1().modify(|_, w| {
        // Master, full duplex. Software CS: internal level high (swcsil = 1
        // keeps the controller out of mode fault). Hardware CS output:
        // SWCSEN = 0, the controller owns the SPIx_CS pin.
        w.msten().set_bit();
        w.slben().disable();
        w.slbtd().clear_bit();
        w.ora().clear_bit();
        if hw_cs {
            w.swcsen().disable();
            w.swcsil().clear_bit();
        } else {
            w.swcsen().enable();
            w.swcsil().set_bit();
        }
        w
    });
}

/// Data register address (DMA peripheral side).
fn dt_addr(r: &pac::spi1::RegisterBlock) -> u32 {
    r.dt() as *const _ as u32
}

/// Clears ROERR with the RM sequence: read DT, then read STS.
fn clear_overrun<T: Instance>() {
    let r = T::regs();
    let _ = r.dt().read();
    let _ = r.sts().read();
}

/// Flushes any stale received bytes (call after transmit-only operations).
fn drain_rx<T: Instance>() {
    let r = T::regs();
    while r.sts().read().rdbf().bit_is_set() {
        let _ = r.dt().read();
    }
    if r.sts().read().roerr().bit_is_set() {
        clear_overrun::<T>();
    }
}

/// Waits for the shift register to empty (BF clear) and reports error flags.
fn wait_idle<T: Instance>() -> Result<(), Error> {
    let r = T::regs();
    while r.sts().read().bf().bit_is_set() {}
    let sts = r.sts().read();
    if sts.roerr().bit_is_set() {
        clear_overrun::<T>();
        return Err(Error::Overrun);
    }
    if sts.mmerr().bit_is_set() {
        // MMERR is read-only; it clears when the controller is disabled.
        r.ctrl1().modify(|_, w| w.spien().disable());
        r.ctrl1().modify(|_, w| w.spien().enable());
        return Err(Error::ModeFault);
    }
    Ok(())
}

/// Blocking, full-duplex exchange of one element (8- or 16-bit per config).
#[inline]
fn blocking_transfer_element<T: Instance>(sent: u16) -> u16 {
    let r = T::regs();
    while !r.sts().read().tdbe().bit_is_set() {}
    r.dt().write(|w| unsafe { w.bits(sent) });
    while !r.sts().read().rdbf().bit_is_set() {}
    r.dt().read().bits()
}

/// Blocking SPI master (no DMA).
pub struct Spi<'d, T: Instance> {
    /// Ownership proof of the SPI singleton (register access goes via T::regs()).
    _peri: Peri<'d, T>,
    _sck: Peri<'d, crate::gpio::AnyPin>,
    _miso: Peri<'d, crate::gpio::AnyPin>,
    _mosi: Peri<'d, crate::gpio::AnyPin>,
    _cs: Option<Peri<'d, crate::gpio::AnyPin>>,
    frame: FrameSize,
    hw_cs: bool,
}

impl<'d, T: Instance> Spi<'d, T> {
    /// Blocking SPI master: configures the pins (SCK/MOSI as AF push-pull,
    /// MISO as AF input) and the controller.
    pub fn new_blocking(
        peri: Peri<'d, T>,
        sck: Peri<'d, impl SckPin<T>>,
        miso: Peri<'d, impl MisoPin<T>>,
        mosi: Peri<'d, impl MosiPin<T>>,
        config: Config,
    ) -> Self {
        let sck_pin = new_pin!(
            sck,
            AfType::output(OutputType::PushPull, Speed::High.into(), Pull::None)
        );
        let miso_pin = new_pin!(miso, AfType::input(Pull::None));
        let mosi_pin = new_pin!(
            mosi,
            AfType::output(OutputType::PushPull, Speed::High.into(), Pull::None)
        );
        Self::new_inner(
            peri,
            sck_pin.unwrap(),
            miso_pin.unwrap(),
            mosi_pin.unwrap(),
            None,
            config,
            false,
        )
    }

    /// Blocking SPI master with the hardware CS output on the `SPIx_CS` AF
    /// pin. The controller drives the pin low while SPIEN is set and releases
    /// it after SPIEN is cleared and the transmit completes (RM 13.2.3);
    /// frame bus transactions with [`Spi::blocking_transaction`] or the
    /// `cs_assert`/`cs_release` pair (the `SpiBus` impl frames each method
    /// call automatically). The release Hi-Zs the pin, so the CS pin gets
    /// the internal pull-up: without one (external or this internal) the
    /// slave stays selected while SPIEN is off (hw-validated on the evb).
    pub fn new_blocking_hwcs(
        peri: Peri<'d, T>,
        sck: Peri<'d, impl SckPin<T>>,
        miso: Peri<'d, impl MisoPin<T>>,
        mosi: Peri<'d, impl MosiPin<T>>,
        cs: Peri<'d, impl CsPin<T>>,
        config: Config,
    ) -> Self {
        let sck_pin = new_pin!(
            sck,
            AfType::output(OutputType::PushPull, Speed::High.into(), Pull::None)
        );
        let miso_pin = new_pin!(miso, AfType::input(Pull::None));
        let mosi_pin = new_pin!(
            mosi,
            AfType::output(OutputType::PushPull, Speed::High.into(), Pull::None)
        );
        let cs_pin = new_pin!(
            cs,
            AfType::output(OutputType::PushPull, Speed::High.into(), Pull::Up)
        );
        Self::new_inner(
            peri,
            sck_pin.unwrap(),
            miso_pin.unwrap(),
            mosi_pin.unwrap(),
            Some(cs_pin.unwrap()),
            config,
            true,
        )
    }

    fn new_inner(
        peri: Peri<'d, T>,
        sck: Peri<'d, crate::gpio::AnyPin>,
        miso: Peri<'d, crate::gpio::AnyPin>,
        mosi: Peri<'d, crate::gpio::AnyPin>,
        cs: Option<Peri<'d, crate::gpio::AnyPin>>,
        config: Config,
        hw_cs: bool,
    ) -> Self {
        T::enable_and_reset();
        let r = T::regs();
        configure(r, &config, T::frequency(), hw_cs);
        // Enable the controller last (in hwcs mode this asserts CS).
        if !hw_cs {
            r.ctrl1().modify(|_, w| w.spien().enable());
        }
        Self {
            _peri: peri,
            _sck: sck,
            _miso: miso,
            _mosi: mosi,
            _cs: cs,
            frame: config.frame_size,
            hw_cs,
        }
    }

    /// Configured frame size.
    pub fn frame_size(&self) -> FrameSize {
        self.frame
    }

    /// Asserts the hardware CS (SPIEN on). Only meaningful in hwcs mode;
    /// the pin stays low until [`Spi::cs_release`].
    pub fn cs_assert(&mut self) {
        T::regs().ctrl1().modify(|_, w| w.spien().enable());
    }

    /// Releases the hardware CS: waits for the shift register to empty, then
    /// clears SPIEN (RM 13.2.3 requires transmit-complete AND SPIEN off
    /// before the pin is released).
    pub fn cs_release(&mut self) {
        let r = T::regs();
        while r.sts().read().bf().bit_is_set() {}
        r.ctrl1().modify(|_, w| w.spien().disable());
    }

    /// Runs `f` inside one hardware-CS-framed transaction: CS asserted,
    /// `f` executed (use the ordinary `blocking_*` methods inside), CS
    /// released. In software-CS mode this simply runs `f`.
    pub fn blocking_transaction<R>(&mut self, f: impl FnOnce(&mut Self) -> R) -> R {
        if self.hw_cs {
            self.cs_assert();
        }
        let r = f(self);
        if self.hw_cs {
            self.cs_release();
        }
        r
    }

    /// CRC polynomial (CPOLY). Call while the controller is disabled (e.g.
    /// right after construction) - the value is latched by the CRC units.
    pub fn set_crc_polynomial(&mut self, poly: u16) {
        T::regs().cpoly().write(|w| unsafe { w.bits(poly) });
    }

    /// Enables the CRC calculation units (CTRL1.CCEN). The hardware resets
    /// TCRC/RCRC to their initial value **only on the 0 -> 1 edge of CCEN**
    /// (re-writing 1 while CCEN is already set is a no-op), so enabling
    /// forces the transition; call between transactions. Accumulator width
    /// follows the frame size: 8-bit frames accumulate in 8 bits with
    /// `CPOLY[7:0]` (a 16-bit poly degenerates to its low byte!), 16-bit
    /// frames accumulate in 16 bits with the full CPOLY (hw-validated
    /// against CRC-16/XMODEM per-word).
    pub fn enable_crc(&mut self, en: bool) {
        let r = T::regs();
        if en {
            r.ctrl1().modify(|_, w| w.ccen().disable());
        }
        r.ctrl1().modify(|_, w| {
            if en {
                w.ccen().enable();
            } else {
                w.ccen().disable();
            }
            w
        });
    }

    /// Transmit CRC accumulator (TCRC) - also its initial value right after
    /// [`Spi::enable_crc`].
    pub fn crc_tx_value(&self) -> u16 {
        T::regs().tcrc().read().bits()
    }

    /// Receive CRC accumulator (RCRC).
    pub fn crc_rx_value(&self) -> u16 {
        T::regs().rcrc().read().bits()
    }

    /// CRC receive-check error flag (STS.CCERR, write-0-to-clear): the CRC
    /// frame received from the slave did not match RCRC. Cleared by this
    /// call.
    pub fn crc_error(&self) -> bool {
        let r = T::regs();
        let err = r.sts().read().ccerr().bit_is_set();
        if err {
            r.sts().modify(|_, w| w.ccerr().clear());
        }
        err
    }

    /// Blocking write with the CRC frame appended (CPU transmit mode,
    /// RM 13.2.5, same order as the official `crc_transfer_polling` example):
    /// sends `buf`; the last data element is written with NTC armed before it
    /// finishes shifting, and the hardware then transmits the TCRC frame
    /// automatically (no extra DT write - that would be sent as a data frame).
    /// The slave's CRC frame lands in DT and is drained. Requires
    /// [`Spi::enable_crc`] beforehand; check the slave's CRC afterwards via
    /// [`Spi::crc_error`].
    pub fn blocking_write_with_crc(&mut self, buf: &[u8]) -> Result<(), Error> {
        if buf.is_empty() {
            return Err(Error::Overrun);
        }
        let r = T::regs();
        for &b in &buf[..buf.len() - 1] {
            let _ = blocking_transfer_element::<T>(b as u16);
        }
        // Last data element + NTC before it finishes shifting.
        while !r.sts().read().tdbe().bit_is_set() {}
        r.dt()
            .write(|w| unsafe { w.bits(buf[buf.len() - 1] as u16) });
        r.ctrl1().modify(|_, w| w.ntc().crc());
        // The last data frame's received value...
        while !r.sts().read().rdbf().bit_is_set() {}
        let _ = r.dt().read().bits();
        // ...then the CRC frame (hardware-sent on TX; the slave's CRC value
        // is received on RX and lands in DT).
        while !r.sts().read().rdbf().bit_is_set() {}
        let _ = r.dt().read().bits();
        wait_idle::<T>()?;
        // NTC self-clears on hardware, but keep the register consistent.
        r.ctrl1().modify(|_, w| w.ntc().normal());
        drain_rx::<T>();
        Ok(())
    }

    /// Blocking read: sends 0xFF (idle-high bus) while clocking in `buf`.
    pub fn blocking_read(&mut self, buf: &mut [u8]) -> Result<(), Error> {
        for b in buf.iter_mut() {
            *b = blocking_transfer_element::<T>(0xFF) as u8;
        }
        wait_idle::<T>()
    }

    /// Blocking write; the received bytes are discarded (overrun tolerated,
    /// ROERR is cleared).
    pub fn blocking_write(&mut self, buf: &[u8]) -> Result<(), Error> {
        for &b in buf {
            let _ = blocking_transfer_element::<T>(b as u16);
        }
        wait_idle::<T>()?;
        drain_rx::<T>();
        Ok(())
    }

    /// Blocking full-duplex transfer: writes `write` while filling `read`.
    /// The transfer runs for max(len) elements; elements past the end of one
    /// buffer use 0xFF (TX side) / are discarded (RX side).
    pub fn blocking_transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Error> {
        let n = read.len().max(write.len());
        for i in 0..n {
            let sent = write.get(i).copied().unwrap_or(0xFF) as u16;
            let got = blocking_transfer_element::<T>(sent) as u8;
            if let Some(r) = read.get_mut(i) {
                *r = got;
            }
        }
        wait_idle::<T>()
    }

    /// Blocking full-duplex transfer over one buffer (sent bytes are replaced
    /// in place by the received bytes).
    pub fn blocking_transfer_in_place(&mut self, buf: &mut [u8]) -> Result<(), Error> {
        for b in buf.iter_mut() {
            let got = blocking_transfer_element::<T>(*b as u16) as u8;
            *b = got;
        }
        wait_idle::<T>()
    }

    /// Blocking wait until the bus goes idle (all bytes shifted out).
    pub fn blocking_flush(&mut self) -> Result<(), Error> {
        wait_idle::<T>()
    }
}

impl<T: Instance> embedded_hal::spi::ErrorType for Spi<'_, T> {
    type Error = Error;
}

impl<T: Instance> embedded_hal::spi::SpiBus for Spi<'_, T> {
    // In hwcs mode every method call is one CS-framed transaction; in
    // software-CS mode `blocking_transaction` just runs the op (CS stays
    // application-owned across calls, per the SpiBus contract).
    fn read(&mut self, buf: &mut [u8]) -> Result<(), Self::Error> {
        self.blocking_transaction(|s| Spi::blocking_read(s, buf))
    }

    fn write(&mut self, buf: &[u8]) -> Result<(), Self::Error> {
        self.blocking_transaction(|s| Spi::blocking_write(s, buf))
    }

    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        self.blocking_transaction(|s| Spi::blocking_transfer(s, read, write))
    }

    fn transfer_in_place(&mut self, buf: &mut [u8]) -> Result<(), Self::Error> {
        self.blocking_transaction(|s| Spi::blocking_transfer_in_place(s, buf))
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Spi::blocking_flush(self)
    }
}
