//! Serial driver: USART1/2/3/6, UART4/5/7/8 (shared register layout,
//! ch32-hal `usart/` structure).
//!
//! - Modes: [`mode::Async`] (interrupt-driven byte-by-byte + waker, no DMA) /
//!   [`mode::Blocking`] (busy-wait polling).
//! - DMA: [`Uart::write_dma`]/[`Uart::read_dma`] move the data over a DMA1/2
//!   channel via DMAMUX (CTRL3.DMATEN/DMAREN); the TX path also waits for TDC
//!   so the shift register drains (RS485 DE turnaround).
//! - Hardware flow control (USART1/2/3/6): [`Uart::enable_flow_control`]
//!   (CTRL3.RTSEN/CTSEN, RTS/CTS pins).
//! - RS485 driver enable (USART1/2/3/6): [`Uart::enable_rs485`] drives DE on
//!   the RTS pin with configurable polarity + TSDT/TCDT delays (CTRL1, 1/16
//!   baud units); single-wire half-duplex via
//!   [`Uart::set_single_wire_half_duplex`].
//! - Async usage:
//!   ```ignore
//!   bind_interrupts!(struct Irqs { USART1 => hal::usart::InterruptHandler<hal::peripherals::USART1>; });
//!   let mut uart = hal::usart::Uart::new(p.USART1, p.PA9, p.PA10, Irqs, config)?;
//!   uart.write(&[b'a']).await?; let mut buf = [0u8; 16]; uart.read(&mut buf).await?;
//!   ```
//! - Baud rate: BAUDR = round(fCK / baud) (Artery's official `usart_init`
//!   formula; fCK = the APB bus clock the instance hangs off,
//!   USART1/6 = PCLK2, the rest = PCLK1, provided by `RccPeripheral::frequency()`).
//! - Flag semantics: `rdbf/tdc` are cleared by writing 0; `perr/ferr/nerr/roerr`
//!   are read-only (the ROERR clear sequence = read STS then read DT). The ISR
//!   latches the error bits into `State.rx_errors` and the read side consumes
//!   them from the latch, avoiding a race between the clear sequence and data
//!   reads.
//! - The pin trait table covers all eight instances; every entry is verified
//!   against the RM V2.09 IOMUX tables 6-1..6-8 (GPIOA..GPIOH).

mod dma;
mod pins;

use core::convert::Infallible;
use core::future::poll_fn;
use core::marker::PhantomData;
use core::sync::atomic::{AtomicU8, Ordering};
use core::task::Poll;

use embassy_hal_internal::Peri;
use embassy_sync::waitqueue::AtomicWaker;

use crate::dma::DmaError;
use crate::gpio::{AfType, OutputType, Pull, Speed};
use crate::interrupt::typelevel::{Binding, Handler, Interrupt as _};
use crate::peripheral::RccPeripheral;
use crate::time::Hertz;
use crate::{interrupt, pac};

/// Serial mode types (same as ch32-hal).
pub mod mode {
    /// Blocking mode (busy-wait polling, no interrupt binding needed).
    pub struct Blocking;
    /// Async mode (interrupt + waker; pass the binding produced by
    /// `bind_interrupts!` at construction).
    pub struct Async;

    pub trait SealedMode {}
    impl SealedMode for Blocking {}
    impl SealedMode for Async {}

    /// Mode marker trait.
    pub trait Mode: SealedMode {}
    impl Mode for Blocking {}
    impl Mode for Async {}
}

/// Number of data bits.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum DataBits {
    Data7,
    Data8,
    Data9,
}

/// Parity bit.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Parity {
    ParityNone,
    ParityEven,
    ParityOdd,
}

/// Number of stop bits.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum StopBits {
    /// 1 bit
    STOP1,
    /// 0.5 bit
    STOP0_5,
    /// 2 bits
    STOP2,
    /// 1.5 bits
    STOP1_5,
}

/// Serial configuration (defaults to 115200 8N1).
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct Config {
    pub baudrate: u32,
    pub data_bits: DataBits,
    pub parity: Parity,
    pub stop_bits: StopBits,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            baudrate: 115_200,
            data_bits: DataBits::Data8,
            parity: Parity::ParityNone,
            stop_bits: StopBits::STOP1,
        }
    }
}

/// Receive error.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[non_exhaustive]
pub enum Error {
    Framing,
    Noise,
    Overrun,
    Parity,
    /// A DMA transfer failed.
    Dma(DmaError),
}

impl From<DmaError> for Error {
    fn from(e: DmaError) -> Self {
        Error::Dma(e)
    }
}

impl embedded_hal_nb::serial::Error for Error {
    fn kind(&self) -> embedded_hal_nb::serial::ErrorKind {
        match self {
            Error::Framing => embedded_hal_nb::serial::ErrorKind::FrameFormat,
            Error::Noise => embedded_hal_nb::serial::ErrorKind::Noise,
            Error::Overrun => embedded_hal_nb::serial::ErrorKind::Overrun,
            Error::Parity => embedded_hal_nb::serial::ErrorKind::Parity,
            Error::Dma(_) => embedded_hal_nb::serial::ErrorKind::Other,
        }
    }
}

/// Configuration error.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[non_exhaustive]
pub enum ConfigError {
    /// Baud rate too low (divider exceeds 16 bits).
    BaudrateTooLow,
    /// Baud rate too high (divider < 1).
    BaudrateTooHigh,
}

// Bit latches of State.rx_errors (mapping the STS error bits 0..3)
const ERR_PERR: u8 = 1 << 0;
const ERR_FERR: u8 = 1 << 1;
const ERR_NERR: u8 = 1 << 2;
const ERR_ROERR: u8 = 1 << 3;
const _ERR_ALL: u8 = ERR_PERR | ERR_FERR | ERR_NERR | ERR_ROERR;

pub(crate) struct State {
    pub(crate) rx_waker: AtomicWaker,
    pub(crate) tx_waker: AtomicWaker,
    /// STS error bits observed by the ISR (PERR/FERR/NERR/ROERR), consumed by
    /// the read side.
    pub(crate) rx_errors: AtomicU8,
}

impl State {
    pub(crate) const fn new() -> Self {
        Self {
            rx_waker: AtomicWaker::new(),
            tx_waker: AtomicWaker::new(),
            rx_errors: AtomicU8::new(0),
        }
    }
}

trait SealedInstance {
    fn regs() -> &'static pac::usart1::RegisterBlock;
    fn state() -> &'static State;
}

/// Serial instance trait (peripheral singleton -> register block / per-instance
/// state / interrupt number).
#[allow(private_bounds)]
pub trait Instance:
    SealedInstance + RccPeripheral + embassy_hal_internal::PeripheralType + Send + 'static
{
    /// This instance's interrupt (typelevel).
    type Interrupt: interrupt::typelevel::Interrupt;
}

macro_rules! impl_usart_instance {
    ($inst:ident, $irq:ident) => {
        impl SealedInstance for crate::peripherals::$inst {
            #[inline]
            fn regs() -> &'static pac::usart1::RegisterBlock {
                unsafe { &*pac::$inst::PTR }
            }

            #[inline]
            fn state() -> &'static State {
                static STATE: State = State::new();
                &STATE
            }
        }

        impl Instance for crate::peripherals::$inst {
            type Interrupt = interrupt::typelevel::$irq;
        }
    };
}

impl_usart_instance!(USART1, USART1);
impl_usart_instance!(USART2, USART2);
impl_usart_instance!(USART3, USART3);
impl_usart_instance!(USART6, USART6);
impl_usart_instance!(UART4, UART4);
impl_usart_instance!(UART5, UART5);
impl_usart_instance!(UART7, UART7);
impl_usart_instance!(UART8, UART8);

crate::pin_trait!(TxPin, Instance);
crate::pin_trait!(RxPin, Instance);
// USART CTS input (USART1/2/3/6 only; UART4/5/7/8 have no CTS/RTS pins).
crate::pin_trait!(CtsPin, Instance);
// USART RTS output; also the RS485 driver-enable (DE) output pin.
crate::pin_trait!(RtsPin, Instance);

crate::dma_trait!(TxDma, Instance);
crate::dma_trait!(RxDma, Instance);

/// RS485 driver-enable polarity (CTRL3.DEP).
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
pub enum DePolarity {
    /// DE asserted high while transmitting.
    #[default]
    ActiveHigh,
    /// DE asserted low while transmitting.
    ActiveLow,
}

/// Interrupt handler: bind it to the instance's IRQ vector via `bind_interrupts!`.
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let r = T::regs();
        let s = T::state();
        let sts = r.sts().read();
        let ctrl1 = r.ctrl1().read();

        // Receive errors (only ours when the RDBF or ERR interrupt is enabled)
        let err_bits = (u8::from(sts.perr().bit_is_set()) * ERR_PERR)
            | (u8::from(sts.ferr().bit_is_set()) * ERR_FERR)
            | (u8::from(sts.nerr().bit_is_set()) * ERR_NERR)
            | (u8::from(sts.roerr().bit_is_set()) * ERR_ROERR);
        if err_bits != 0 && (ctrl1.rdbfien().bit_is_set() || r.ctrl3().read().errien().bit_is_set())
        {
            // Latch the error bits for the read side to consume; disable the
            // RX interrupt to stop repeated false reports
            s.rx_errors.fetch_or(err_bits, Ordering::Relaxed);
            r.ctrl1().modify(|_, w| w.rdbfien().disable());
            r.ctrl3().modify(|_, w| w.errien().disable());
            s.rx_waker.wake();
        }

        if ctrl1.rdbfien().bit_is_set() && sts.rdbf().bit_is_set() {
            // Disable the IE so the flag (which only clears on a DT read)
            // cannot keep the IRQ line asserted without a reader.
            r.ctrl1().modify(|_, w| w.rdbfien().disable());
            s.rx_waker.wake();
        }
        if ctrl1.tdbeien().bit_is_set() && sts.tdbe().bit_is_set() {
            r.ctrl1().modify(|_, w| w.tdbeien().disable());
            s.tx_waker.wake();
        }
        if ctrl1.tdcien().bit_is_set() && sts.tdc().bit_is_set() {
            r.ctrl1().modify(|_, w| w.tdcien().disable());
            s.tx_waker.wake();
        }
    }
}

/// Compute the baud-rate divider: BAUDR = round(fCK / baud) (same formula as
/// Artery's `usart_init`).
fn baudrate_divisor(clock_hz: u32, baudrate: u32) -> Result<u16, ConfigError> {
    if baudrate == 0 {
        return Err(ConfigError::BaudrateTooLow);
    }
    let div = ((clock_hz as u64) + (baudrate as u64) / 2) / (baudrate as u64);
    if div > 0xFFFF {
        return Err(ConfigError::BaudrateTooLow);
    }
    if div == 0 {
        return Err(ConfigError::BaudrateTooHigh);
    }
    Ok(div as u16)
}

/// Register-level configuration (uen=0 -> parameters -> baudr -> ten/ren -> uen=1).
fn configure(
    r: &pac::usart1::RegisterBlock,
    config: &Config,
    fck: Hertz,
    enable_tx: bool,
    enable_rx: bool,
) -> Result<(), ConfigError> {
    let div = baudrate_divisor(fck.0, config.baudrate)?;

    r.ctrl1().modify(|_, w| w.uen().disable());

    // Data bits: DBN[1:0] (dbn_l = bit12 low, dbn_h = bit28 high):
    // 7 bits=00, 8 bits=01, 9 bits=10
    let (dbn_l, dbn_h) = match config.data_bits {
        DataBits::Data7 => (false, false),
        DataBits::Data8 => (true, false),
        DataBits::Data9 => (false, true),
    };
    r.ctrl1().modify(|_, w| {
        w.dbn0().bit(dbn_l);
        w.dbn1().bit(dbn_h);
        w
    });
    // Stop bits (ctrl2.STOPBN[1:0]): 1=00, 0.5=01, 2=10, 1.5=11
    let stopbn = match config.stop_bits {
        StopBits::STOP1 => 0u8,
        StopBits::STOP0_5 => 1,
        StopBits::STOP2 => 2,
        StopBits::STOP1_5 => 3,
    };
    r.ctrl2().modify(|_, w| unsafe {
        w.stopbn().bits(stopbn);
        w
    });
    // Parity bit
    r.ctrl1().modify(|_, w| {
        match config.parity {
            Parity::ParityNone => {
                w.pen().disable();
            }
            Parity::ParityEven => {
                w.pen().enable();
                w.psel().clear_bit();
            }
            Parity::ParityOdd => {
                w.pen().enable();
                w.psel().set_bit();
            }
        }
        w
    });

    r.baudr().write(|w| w.set(div));

    r.ctrl1().modify(|_, w| {
        if enable_tx {
            w.ten().enable();
        }
        if enable_rx {
            w.ren().enable();
        }
        w
    });
    r.ctrl1().modify(|_, w| w.uen().enable());
    Ok(())
}

/// Collect the STS error bits into the latch and map them to `Error` (also
/// runs the ROERR clear sequence).
fn take_rx_errors<T: Instance>() -> Result<(), Error> {
    let s = T::state();
    let latched = s.rx_errors.swap(0, Ordering::Relaxed);
    if latched == 0 {
        return Ok(());
    }
    // ROERR clear sequence: read STS then read DT (discard the bad byte)
    let r = T::regs();
    let _ = r.sts().read();
    let _ = r.dt().read();
    Err(if latched & ERR_PERR != 0 {
        Error::Parity
    } else if latched & ERR_FERR != 0 {
        Error::Framing
    } else if latched & ERR_NERR != 0 {
        Error::Noise
    } else {
        Error::Overrun
    })
}

// ---------------------------------------------------------------------------
// Uart full-duplex
// ---------------------------------------------------------------------------

/// Full-duplex serial port.
pub struct Uart<'d, T: Instance, M: mode::Mode> {
    #[allow(dead_code)]
    peri: Peri<'d, T>,
    _tx_pin: Option<Peri<'d, crate::gpio::AnyPin>>,
    _rx_pin: Option<Peri<'d, crate::gpio::AnyPin>>,
    _mode: PhantomData<M>,
}

impl<'d, T: Instance> Uart<'d, T, mode::Async> {
    /// Async full-duplex (`_irq` takes the binding struct produced by
    /// `bind_interrupts!`).
    pub fn new(
        peri: Peri<'d, T>,
        tx: Peri<'d, impl TxPin<T>>,
        rx: Peri<'d, impl RxPin<T>>,
        _irq: impl Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        let tx_pin = new_pin!(
            tx,
            AfType::output(OutputType::PushPull, Speed::High.into(), Pull::None)
        );
        let rx_pin = new_pin!(rx, AfType::input(Pull::None));
        let this = Self::new_inner(peri, tx_pin, rx_pin, config, true, true)?;
        // RX interrupt only: TDBE/TDC are set at reset and re-assert while
        // idle, so leaving their IEs on would storm the NVIC; the async
        // write/flush futures arm them while pending instead.
        let r = T::regs();
        r.ctrl1().modify(|_, w| w.rdbfien().enable());
        r.ctrl3().modify(|_, w| w.errien().enable());
        T::Interrupt::enable();
        Ok(this)
    }

    /// Split into independent Tx / Rx halves (each holds a copy of the
    /// singleton: Tx/Rx share the same register block anyway).
    pub fn split(self) -> (UartTx<'d, T, mode::Async>, UartRx<'d, T, mode::Async>) {
        let Self {
            peri,
            _tx_pin,
            _rx_pin,
            _mode,
        } = self;
        let tx_peri = unsafe { peri.clone_unchecked() };
        (
            UartTx {
                peri: tx_peri,
                _pin: _tx_pin,
                _mode: PhantomData,
            },
            UartRx {
                peri,
                _pin: _rx_pin,
                _mode: PhantomData,
            },
        )
    }
}

impl<'d, T: Instance> Uart<'d, T, mode::Blocking> {
    /// Blocking full-duplex (no interrupts).
    pub fn new_blocking(
        peri: Peri<'d, T>,
        tx: Peri<'d, impl TxPin<T>>,
        rx: Peri<'d, impl RxPin<T>>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        let tx_pin = new_pin!(
            tx,
            AfType::output(OutputType::PushPull, Speed::High.into(), Pull::None)
        );
        let rx_pin = new_pin!(rx, AfType::input(Pull::None));
        Self::new_inner(peri, tx_pin, rx_pin, config, true, true)
    }
}

impl<'d, T: Instance, M: mode::Mode> Uart<'d, T, M> {
    fn new_inner(
        peri: Peri<'d, T>,
        tx_pin: Option<Peri<'d, crate::gpio::AnyPin>>,
        rx_pin: Option<Peri<'d, crate::gpio::AnyPin>>,
        config: Config,
        enable_tx: bool,
        enable_rx: bool,
    ) -> Result<Self, ConfigError> {
        T::enable_and_reset();
        let r = T::regs();
        configure(r, &config, T::frequency(), enable_tx, enable_rx)?;
        Ok(Self {
            peri,
            _tx_pin: tx_pin,
            _rx_pin: rx_pin,
            _mode: PhantomData,
        })
    }

    /// Enables RTS/CTS hardware flow control (CTRL3.RTSEN/CTSEN). The pins
    /// are switched to the USART alternate function here; UART4/5/7/8 have
    /// no flow-control pins (their CtsPin/RtsPin traits are not implemented).
    pub fn enable_flow_control(
        &mut self,
        cts: Peri<'_, impl CtsPin<T>>,
        rts: Peri<'_, impl RtsPin<T>>,
    ) {
        cts.set_as_af(cts.af_num(), AfType::input(Pull::None));
        rts.set_as_af(
            rts.af_num(),
            AfType::output(OutputType::PushPull, Speed::High.into(), Pull::None),
        );
        T::regs().ctrl3().modify(|_, w| {
            w.rtsen().enable();
            w.ctsen().enable();
            w
        });
    }

    /// Enables RS485 driver-enable mode (RM 12.4.5): DE comes out of the RTS
    /// pin while transmitting, with the requested polarity. `start_delay` /
    /// `complete_delay` are the TSDT/TCDT turnaround delays in 1/16 baud
    /// periods (0..31, clamped). The TX path waits for TDC before returning,
    /// so the hardware applies the complete-delay before DE deasserts.
    pub fn enable_rs485(
        &mut self,
        rts: Peri<'_, impl RtsPin<T>>,
        polarity: DePolarity,
        start_delay: u8,
        complete_delay: u8,
    ) {
        rts.set_as_af(
            rts.af_num(),
            AfType::output(OutputType::PushPull, Speed::High.into(), Pull::None),
        );
        let r = T::regs();
        r.ctrl1().modify(|_, w| unsafe {
            w.tsdt()
                .bits(start_delay.min(31))
                .tcdt()
                .bits(complete_delay.min(31))
        });
        r.ctrl3().modify(|_, w| {
            w.rs485en().bit(true);
            w.dep().bit(polarity == DePolarity::ActiveLow);
            w
        });
    }

    /// Single-wire half-duplex mode (CTRL3.SLBEN): TX and RX share the TX
    /// pin; the receiver is fed from the TX pin, so the USART receives its
    /// own transmissions.
    pub fn set_single_wire_half_duplex(&mut self, on: bool) {
        T::regs().ctrl3().modify(|_, w| w.slben().bit(on));
    }

    /// Blocking write (works in both modes).
    pub fn blocking_write(&mut self, buf: &[u8]) -> Result<(), Infallible> {
        for &b in buf {
            blocking_write_byte::<T>(b);
        }
        Ok(())
    }

    /// Blocking wait until transmission fully ends (TDC).
    pub fn blocking_flush(&mut self) -> Result<(), Infallible> {
        let r = T::regs();
        while !r.sts().read().tdc().bit_is_set() {}
        Ok(())
    }

    /// Blocking read (returns the corresponding Error on failure; the ROERR
    /// sequence is handled).
    pub fn blocking_read(&mut self, buf: &mut [u8]) -> Result<(), Error> {
        for b in buf.iter_mut() {
            *b = blocking_read_byte::<T>()?;
        }
        Ok(())
    }
}

impl<'d, T: Instance> Uart<'d, T, mode::Async> {
    /// Async write of all bytes (waits for TDBE byte by byte).
    pub async fn write(&mut self, buf: &[u8]) -> Result<(), Error> {
        for &b in buf {
            poll_fn(|cx| -> Poll<Result<(), Error>> {
                let r = T::regs();
                T::state().tx_waker.register(cx.waker());
                if r.sts().read().tdbe().bit_is_set() {
                    r.dt().write(|w| unsafe { w.bits(b as u16) });
                    Poll::Ready(Ok(()))
                } else {
                    // Buffer full: arm the TDBE interrupt until the ISR
                    // consumes it (it disables the IE when it wakes us).
                    r.ctrl1().modify(|_, w| w.tdbeien().enable());
                    Poll::Pending
                }
            })
            .await?;
        }
        Ok(())
    }

    /// Async wait until transmission fully ends (TDC).
    pub async fn flush(&mut self) -> Result<(), Error> {
        poll_fn(|cx| -> Poll<Result<(), Error>> {
            let r = T::regs();
            T::state().tx_waker.register(cx.waker());
            if r.sts().read().tdc().bit_is_set() {
                Poll::Ready(Ok(()))
            } else {
                r.ctrl1().modify(|_, w| w.tdcien().enable());
                Poll::Pending
            }
        })
        .await
    }

    /// Async read filling `buf` (waits for RDBF byte by byte; errors are
    /// reported via the latch).
    pub async fn read(&mut self, buf: &mut [u8]) -> Result<(), Error> {
        for b in buf.iter_mut() {
            take_rx_errors::<T>()?;
            *b = poll_fn(|cx| {
                let r = T::regs();
                T::state().rx_waker.register(cx.waker());
                if r.sts().read().rdbf().bit_is_set() {
                    Poll::Ready(r.dt().read().bits() as u8)
                } else {
                    // Arm the RDBF interrupt while pending (the ISR disables
                    // it when it wakes us).
                    r.ctrl1().modify(|_, w| w.rdbfien().enable());
                    Poll::Pending
                }
            })
            .await;
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// UartTx / UartRx halves
// ---------------------------------------------------------------------------

/// Transmit half.
pub struct UartTx<'d, T: Instance, M: mode::Mode> {
    #[allow(dead_code)]
    peri: Peri<'d, T>,
    _pin: Option<Peri<'d, crate::gpio::AnyPin>>,
    _mode: PhantomData<M>,
}

/// Receive half.
pub struct UartRx<'d, T: Instance, M: mode::Mode> {
    #[allow(dead_code)]
    peri: Peri<'d, T>,
    _pin: Option<Peri<'d, crate::gpio::AnyPin>>,
    _mode: PhantomData<M>,
}

impl<'d, T: Instance> UartTx<'d, T, mode::Async> {
    /// TX-only (async).
    pub fn new(
        peri: Peri<'d, T>,
        tx: Peri<'d, impl TxPin<T>>,
        _irq: impl Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        let tx_pin = new_pin!(
            tx,
            AfType::output(OutputType::PushPull, Speed::High.into(), Pull::None)
        );
        T::enable_and_reset();
        let r = T::regs();
        configure(r, &config, T::frequency(), true, false)?;
        r.ctrl1().modify(|_, w| {
            w.tdbeien().enable();
            w.tdcien().enable();
            w
        });
        T::Interrupt::enable();
        Ok(Self {
            peri,
            _pin: tx_pin,
            _mode: PhantomData,
        })
    }

    /// Async write of all bytes.
    pub async fn write(&mut self, buf: &[u8]) -> Result<(), Error> {
        for &b in buf {
            poll_fn(|cx| -> Poll<Result<(), Error>> {
                let r = T::regs();
                T::state().tx_waker.register(cx.waker());
                if r.sts().read().tdbe().bit_is_set() {
                    r.dt().write(|w| unsafe { w.bits(b as u16) });
                    Poll::Ready(Ok(()))
                } else {
                    // Buffer full: arm the TDBE interrupt until the ISR
                    // consumes it (it disables the IE when it wakes us).
                    r.ctrl1().modify(|_, w| w.tdbeien().enable());
                    Poll::Pending
                }
            })
            .await?;
        }
        Ok(())
    }

    /// Async wait for TDC.
    pub async fn flush(&mut self) -> Result<(), Error> {
        poll_fn(|cx| -> Poll<Result<(), Error>> {
            let r = T::regs();
            T::state().tx_waker.register(cx.waker());
            if r.sts().read().tdc().bit_is_set() {
                Poll::Ready(Ok(()))
            } else {
                r.ctrl1().modify(|_, w| w.tdcien().enable());
                Poll::Pending
            }
        })
        .await
    }
}

impl<'d, T: Instance> UartRx<'d, T, mode::Async> {
    /// RX-only (async).
    pub fn new(
        peri: Peri<'d, T>,
        rx: Peri<'d, impl RxPin<T>>,
        _irq: impl Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        let rx_pin = new_pin!(rx, AfType::input(Pull::None));
        T::enable_and_reset();
        let r = T::regs();
        configure(r, &config, T::frequency(), false, true)?;
        r.ctrl1().modify(|_, w| w.rdbfien().enable());
        r.ctrl3().modify(|_, w| w.errien().enable());
        T::Interrupt::enable();
        Ok(Self {
            peri,
            _pin: rx_pin,
            _mode: PhantomData,
        })
    }

    /// Async read filling `buf`.
    pub async fn read(&mut self, buf: &mut [u8]) -> Result<(), Error> {
        for b in buf.iter_mut() {
            take_rx_errors::<T>()?;
            *b = poll_fn(|cx| {
                let r = T::regs();
                T::state().rx_waker.register(cx.waker());
                if r.sts().read().rdbf().bit_is_set() {
                    Poll::Ready(r.dt().read().bits() as u8)
                } else {
                    // Arm the RDBF interrupt while pending (the ISR disables
                    // it when it wakes us).
                    r.ctrl1().modify(|_, w| w.rdbfien().enable());
                    Poll::Pending
                }
            })
            .await;
        }
        Ok(())
    }
}

impl<'d, T: Instance> UartTx<'d, T, mode::Blocking> {
    /// TX-only (blocking).
    pub fn new_blocking(
        peri: Peri<'d, T>,
        tx: Peri<'d, impl TxPin<T>>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        let tx_pin = new_pin!(
            tx,
            AfType::output(OutputType::PushPull, Speed::High.into(), Pull::None)
        );
        T::enable_and_reset();
        configure(T::regs(), &config, T::frequency(), true, false)?;
        Ok(Self {
            peri,
            _pin: tx_pin,
            _mode: PhantomData,
        })
    }

    /// Blocking write.
    pub fn blocking_write(&mut self, buf: &[u8]) -> Result<(), Infallible> {
        for &b in buf {
            blocking_write_byte::<T>(b);
        }
        Ok(())
    }

    /// Blocking wait for TDC.
    pub fn blocking_flush(&mut self) -> Result<(), Infallible> {
        while !T::regs().sts().read().tdc().bit_is_set() {}
        Ok(())
    }
}

impl<'d, T: Instance> UartRx<'d, T, mode::Blocking> {
    /// RX-only (blocking).
    pub fn new_blocking(
        peri: Peri<'d, T>,
        rx: Peri<'d, impl RxPin<T>>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        let rx_pin = new_pin!(rx, AfType::input(Pull::None));
        T::enable_and_reset();
        configure(T::regs(), &config, T::frequency(), false, true)?;
        Ok(Self {
            peri,
            _pin: rx_pin,
            _mode: PhantomData,
        })
    }

    /// Blocking read.
    pub fn blocking_read(&mut self, buf: &mut [u8]) -> Result<(), Error> {
        for b in buf.iter_mut() {
            *b = blocking_read_byte::<T>()?;
        }
        Ok(())
    }
}

#[inline]
fn blocking_write_byte<T: Instance>(b: u8) {
    let r = T::regs();
    while !r.sts().read().tdbe().bit_is_set() {}
    r.dt().write(|w| unsafe { w.bits(b as u16) });
}

#[inline]
fn blocking_read_byte<T: Instance>() -> Result<u8, Error> {
    // Consume latched errors first (runs the ROERR clear sequence if any)
    let s = T::state();
    if s.rx_errors.load(Ordering::Relaxed) != 0 {
        take_rx_errors::<T>()?;
    }
    let r = T::regs();
    loop {
        let sts = r.sts().read();
        if sts.roerr().bit_is_set() {
            // Overrun: read STS + read DT to clear, and discard
            let _ = r.dt().read();
            return Err(Error::Overrun);
        }
        if sts.perr().bit_is_set() {
            let _ = r.dt().read();
            return Err(Error::Parity);
        }
        if sts.ferr().bit_is_set() || sts.nerr().bit_is_set() {
            let _ = r.dt().read();
            return Err(Error::Framing);
        }
        if sts.rdbf().bit_is_set() {
            return Ok(r.dt().read().bits() as u8);
        }
    }
}

// ---------------------------------------------------------------------------
// Trait impls (Blocking mode: embedded-hal-nb serial [Read/Write, nb::Result
// semantics] + core::fmt::Write)
// ---------------------------------------------------------------------------

type NbResult<T, E> = core::result::Result<T, embedded_hal_nb::nb::Error<E>>;

/// Single-byte non-blocking read with nb semantics (errors reported
/// immediately; the ROERR sequence is handled).
fn nb_read_byte<T: Instance>() -> NbResult<u8, Error> {
    use embedded_hal_nb::nb::Error as NbError;
    let latched = T::state().rx_errors.swap(0, Ordering::Relaxed);
    if latched != 0 {
        let _ = take_rx_errors::<T>();
        return Err(NbError::Other(Error::Overrun));
    }
    let r = T::regs();
    let sts = r.sts().read();
    if sts.roerr().bit_is_set() {
        let _ = r.dt().read(); // ROERR clear sequence: read STS + read DT
        Err(NbError::Other(Error::Overrun))
    } else if sts.perr().bit_is_set() {
        let _ = r.dt().read();
        Err(NbError::Other(Error::Parity))
    } else if sts.ferr().bit_is_set() || sts.nerr().bit_is_set() {
        let _ = r.dt().read();
        Err(NbError::Other(Error::Framing))
    } else if sts.rdbf().bit_is_set() {
        Ok(r.dt().read().bits() as u8)
    } else {
        Err(NbError::WouldBlock)
    }
}

/// Single-byte non-blocking write with nb semantics.
fn nb_write_byte<T: Instance>(byte: u8) -> NbResult<(), Error> {
    let r = T::regs();
    if r.sts().read().tdbe().bit_is_set() {
        r.dt().write(|w| unsafe { w.bits(byte as u16) });
        Ok(())
    } else {
        Err(embedded_hal_nb::nb::Error::WouldBlock)
    }
}

fn nb_flush<T: Instance>() -> NbResult<(), Error> {
    if T::regs().sts().read().tdc().bit_is_set() {
        Ok(())
    } else {
        Err(embedded_hal_nb::nb::Error::WouldBlock)
    }
}

impl<T: Instance> embedded_hal_nb::serial::ErrorType for UartTx<'_, T, mode::Blocking> {
    type Error = Error;
}

impl<T: Instance> embedded_hal_nb::serial::Write for UartTx<'_, T, mode::Blocking> {
    fn write(&mut self, byte: u8) -> NbResult<(), Self::Error> {
        nb_write_byte::<T>(byte)
    }

    fn flush(&mut self) -> NbResult<(), Self::Error> {
        nb_flush::<T>()
    }
}

impl<T: Instance> embedded_hal_nb::serial::ErrorType for UartRx<'_, T, mode::Blocking> {
    type Error = Error;
}

impl<T: Instance> embedded_hal_nb::serial::Read for UartRx<'_, T, mode::Blocking> {
    fn read(&mut self) -> NbResult<u8, Self::Error> {
        nb_read_byte::<T>()
    }
}

impl<T: Instance> embedded_hal_nb::serial::ErrorType for Uart<'_, T, mode::Blocking> {
    type Error = Error;
}

impl<T: Instance> embedded_hal_nb::serial::Write for Uart<'_, T, mode::Blocking> {
    fn write(&mut self, byte: u8) -> NbResult<(), Self::Error> {
        nb_write_byte::<T>(byte)
    }

    fn flush(&mut self) -> NbResult<(), Self::Error> {
        nb_flush::<T>()
    }
}

impl<T: Instance> embedded_hal_nb::serial::Read for Uart<'_, T, mode::Blocking> {
    fn read(&mut self) -> NbResult<u8, Self::Error> {
        nb_read_byte::<T>()
    }
}

impl<T: Instance> core::fmt::Write for UartTx<'_, T, mode::Blocking> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.blocking_write(s.as_bytes())
            .map_err(|_| core::fmt::Error)?;
        Ok(())
    }
}

impl<T: Instance> core::fmt::Write for Uart<'_, T, mode::Blocking> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.blocking_write(s.as_bytes())
            .map_err(|_| core::fmt::Error)?;
        Ok(())
    }
}
