//! DMA controller driver (RM 9): DMA1/DMA2, 7 channels each, with the built-in
//! DMAMUX request router (ch32-hal `dma/` structure).
//!
//! Every channel can be routed to any peripheral request line through the
//! DMAMUX (per-DMA-instance embedded at DMA_BASE+0x104). `Request` holds the
//! official request-line IDs (firmware library `dmamux_requst_id_sel_type`).
//!
//! Per-channel singletons (`peripherals::DMA1_CH1` ... `DMA2_CH7`) own the
//! channel exclusively; the parent DMA controller clock is enabled by the
//! channel constructor.
//!
//! Blocking usage (mem-to-mem):
//! ```ignore
//! let mut ch = hal::dma::Channel::new(p.DMA1_CH1);
//! // SAFETY: The two DMA-accessible buffers are disjoint and live through the call.
//! unsafe { ch.mem_to_mem(dst.as_mut_ptr(), src.as_ptr(), src.len())? };
//! ```
//!
//! Async usage (peripheral transfer, e.g. SPI):
//! ```ignore
//! bind_interrupts!(struct Irqs {
//!     DMA1_CHANNEL4 => hal::dma::InterruptHandler<hal::peripherals::DMA1_CH4>;
//! });
//! let mut tx_dma = hal::dma::Channel::new(p.DMA1_CH4);
//! tx_dma.listen();                       // enable FDTIEN + NVIC
//! // Configure and start the channel with valid DMA endpoints.
//! // ... then await the channel (see spi/i2c drivers for the full pattern)
//! ```

use core::future::poll_fn;
use core::marker::PhantomData;
use core::sync::atomic::{compiler_fence, Ordering};
use core::task::{Context, Poll};
use embassy_hal_internal::drop::OnDrop;

use embassy_hal_internal::Peri;
use embassy_sync::waitqueue::AtomicWaker;

use crate::interrupt::typelevel::{Handler, Interrupt as _};
use crate::{interrupt, pac};

/// One waker per channel (DMA1 has 7 channels, DMA2 has 7).
const CHANNEL_TOTAL: usize = 14;

static WAKERS: [AtomicWaker; CHANNEL_TOTAL] = [
    AtomicWaker::new(),
    AtomicWaker::new(),
    AtomicWaker::new(),
    AtomicWaker::new(),
    AtomicWaker::new(),
    AtomicWaker::new(),
    AtomicWaker::new(),
    AtomicWaker::new(),
    AtomicWaker::new(),
    AtomicWaker::new(),
    AtomicWaker::new(),
    AtomicWaker::new(),
    AtomicWaker::new(),
    AtomicWaker::new(),
];

#[inline]
fn waker(dma_index: usize, ch_index: usize) -> &'static AtomicWaker {
    &WAKERS[dma_index * 7 + ch_index]
}

/// DMAMUX request line (values from the official firmware library
/// `dmamux_requst_id_sel_type`; written to `DMA_MUXCxCTRL.REQSEL[6:0]`).
///
/// Only the request lines of the peripherals implemented by this HAL are
/// listed; extend as drivers are added (SDIO/QSPI/DVP/EDMA paths later).
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u8)]
#[allow(dead_code)]
pub enum Request {
    /// No request line: mem-to-memory transfer (M2M bit drives the channel).
    MemToMem = 0,
    Adc1 = 0x05,
    Adc2 = 0x24,
    Adc3 = 0x25,
    Dac1 = 0x06,
    Dac2 = 0x29,
    Tmr6Overflow = 0x08,
    Tmr7Overflow = 0x09,
    Spi1Rx = 0x0A,
    Spi1Tx = 0x0B,
    Spi2Rx = 0x0C,
    Spi2Tx = 0x0D,
    Spi3Rx = 0x0E,
    Spi3Tx = 0x0F,
    Spi4Rx = 0x6A,
    Spi4Tx = 0x6B,
    I2s2ExtRx = 0x6E,
    I2s2ExtTx = 0x6F,
    I2s3ExtRx = 0x70,
    I2s3ExtTx = 0x71,
    I2c1Rx = 0x10,
    I2c1Tx = 0x11,
    I2c2Rx = 0x12,
    I2c2Tx = 0x13,
    I2c3Rx = 0x14,
    I2c3Tx = 0x15,
    Usart1Rx = 0x18,
    Usart1Tx = 0x19,
    Usart2Rx = 0x1A,
    Usart2Tx = 0x1B,
    Usart3Rx = 0x1C,
    Usart3Tx = 0x1D,
    Uart4Rx = 0x1E,
    Uart4Tx = 0x1F,
    Uart5Rx = 0x20,
    Uart5Tx = 0x21,
    Sdio1 = 0x27,
    Sdio2 = 0x67,
    Qspi1 = 0x28,
    Qspi2 = 0x68,
    Usart6Rx = 0x72,
    Usart6Tx = 0x73,
    Uart7Rx = 0x74,
    Uart7Tx = 0x75,
    Uart8Rx = 0x76,
    Uart8Tx = 0x77,
}

/// Transfer data width (both sides programmed independently).
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Width {
    Bits8,
    Bits16,
    Bits32,
}

impl Width {
    /// Width in bytes.
    pub const fn bytes(self) -> usize {
        match self {
            Width::Bits8 => 1,
            Width::Bits16 => 2,
            Width::Bits32 => 4,
        }
    }
}

/// Channel priority (arbitration among enabled channels; equal priorities are
/// resolved by channel number, lower first).
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
pub enum Priority {
    Low,
    Medium,
    High,
    #[default]
    VeryHigh,
}

/// Transfer direction (CTRL.DTD).
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Direction {
    /// Peripheral -> memory (P2M).
    PeripheralToMemory,
    /// Memory -> peripheral (M2P).
    MemoryToPeripheral,
}

/// Per-transfer configuration (everything except the addresses/counts).
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct Config {
    /// Transfer direction.
    pub direction: Direction,
    /// Peripheral-side data width.
    pub peripheral_width: Width,
    /// Memory-side data width.
    pub memory_width: Width,
    /// Increment the peripheral address after each transfer.
    pub peripheral_increment: bool,
    /// Increment the memory address after each transfer.
    pub memory_increment: bool,
    /// Loop (circular) mode: the channel reloads automatically at completion.
    pub circular: bool,
    /// Arbitration priority.
    pub priority: Priority,
}

impl Config {
    /// Configuration for memory -> peripheral transfers (memory increments,
    /// peripheral is a single register, same width on both sides).
    pub const fn mem_to_peripheral(width: Width) -> Self {
        Self {
            direction: Direction::MemoryToPeripheral,
            peripheral_width: width,
            memory_width: width,
            peripheral_increment: false,
            memory_increment: true,
            circular: false,
            priority: Priority::VeryHigh,
        }
    }

    /// Configuration for peripheral -> memory transfers.
    pub const fn peripheral_to_memory(width: Width) -> Self {
        Self {
            direction: Direction::PeripheralToMemory,
            peripheral_width: width,
            memory_width: width,
            peripheral_increment: false,
            memory_increment: true,
            circular: false,
            priority: Priority::VeryHigh,
        }
    }
}

/// Channel transfer error.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[non_exhaustive]
pub enum DmaError {
    /// The channel reported DTERRF (transfer error).
    TransferError,
    /// Count does not fit the 16-bit DTCNT register.
    CountTooLarge,
}

trait SealedChannel {}

/// DMA channel instance trait (per-channel singleton).
#[allow(private_bounds)]
pub trait ChannelInstance: SealedChannel + embassy_hal_internal::PeripheralType + 'static {
    /// Parent DMA controller index (0 = DMA1, 1 = DMA2).
    const DMA_INDEX: usize;
    /// Channel index inside the controller (0..6, i.e. channel1..channel7).
    const CH_INDEX: usize;
    /// The channel's own NVIC interrupt.
    type Interrupt: interrupt::typelevel::Interrupt;
}

/// Which boundary the circular transfer just crossed.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum CircularEvent {
    /// HDTF: the first half of the buffer is ready for processing.
    FirstHalf,
    /// FDTF: the second half is ready (the counter reloaded).
    SecondHalf,
}

/// DMA channel handle.
pub struct Channel<'d, T: ChannelInstance> {
    _peri: Peri<'d, T>,
    _mode: PhantomData<T>,
}

impl<'d, T: ChannelInstance> Channel<'d, T> {
    /// Takes ownership of a channel singleton: enables the parent DMA clock,
    /// selects the flexible request-mapping table (TBL_SEL) and leaves the
    /// channel disabled.
    pub fn new(peri: Peri<'d, T>) -> Self {
        critical_section::with(|_| {
            // The clock and mapping mode belong to the entire controller.
            // Resetting it here would abort transfers on other channels.
            let crm = unsafe { &*pac::CRM::PTR };
            crm.ahben1().modify(|_, w| match T::DMA_INDEX {
                0 => w.dma1().set_bit(),
                _ => w.dma2().set_bit(),
            });
            let _ = crm.ahben1().read();
            Self::block()
                .dma_muxsel()
                .modify(|_, w| w.tbl_sel().set_bit());
        });
        Self::stop_transfer();
        Self::ch().ctrl().reset();
        Self {
            _peri: peri,
            _mode: PhantomData,
        }
    }

    #[inline]
    fn block() -> &'static pac::dma1::RegisterBlock {
        match T::DMA_INDEX {
            0 => unsafe { &*pac::DMA1::PTR },
            _ => unsafe { &*pac::DMA2::PTR },
        }
    }

    /// Channel registers (channel cluster at 0x08 + 20*index).
    #[inline]
    pub(crate) fn ch() -> &'static pac::dma1::channel::Channel {
        Self::block().channel(T::CH_INDEX)
    }

    /// Writes the DMAMUX request line for this channel (flexible table).
    pub fn set_request(&mut self, request: Request) {
        assert!(
            !self.is_enabled(),
            "stop the DMA channel before reconfiguring it"
        );
        let req = request as u8;
        match T::CH_INDEX {
            0 => Self::block()
                .muxc1ctrl()
                .modify(|_, w| unsafe { w.reqsel().bits(req) }),
            1 => Self::block()
                .muxc2ctrl()
                .modify(|_, w| unsafe { w.reqsel().bits(req) }),
            2 => Self::block()
                .muxc3ctrl()
                .modify(|_, w| unsafe { w.reqsel().bits(req) }),
            3 => Self::block()
                .muxc4ctrl()
                .modify(|_, w| unsafe { w.reqsel().bits(req) }),
            4 => Self::block()
                .muxc5ctrl()
                .modify(|_, w| unsafe { w.reqsel().bits(req) }),
            5 => Self::block()
                .muxc6ctrl()
                .modify(|_, w| unsafe { w.reqsel().bits(req) }),
            _ => Self::block()
                .muxc7ctrl()
                .modify(|_, w| unsafe { w.reqsel().bits(req) }),
        };
    }

    /// Applies the transfer configuration (channel must be disabled).
    pub fn set_config(&mut self, cfg: Config) {
        assert!(
            !self.is_enabled(),
            "stop the DMA channel before reconfiguring it"
        );
        let ch = Self::ch();
        ch.ctrl().modify(|_, w| {
            w.m2m().disable();
            match cfg.direction {
                Direction::PeripheralToMemory => w.dtd().peripherals(),
                Direction::MemoryToPeripheral => w.dtd().memory(),
            };
            match cfg.peripheral_width {
                Width::Bits8 => w.pwidth().bit8(),
                Width::Bits16 => w.pwidth().bit16(),
                Width::Bits32 => w.pwidth().bit32(),
            };
            match cfg.memory_width {
                Width::Bits8 => w.mwidth().bit8(),
                Width::Bits16 => w.mwidth().bit16(),
                Width::Bits32 => w.mwidth().bit32(),
            };
            if cfg.peripheral_increment {
                w.pincm().enable();
            } else {
                w.pincm().disable();
            }
            if cfg.memory_increment {
                w.mincm().enable();
            } else {
                w.mincm().disable();
            }
            if cfg.circular {
                w.lm().enable();
            } else {
                w.lm().disable();
            }
            match cfg.priority {
                Priority::Low => w.chpl().low(),
                Priority::Medium => w.chpl().medium(),
                Priority::High => w.chpl().high(),
                Priority::VeryHigh => w.chpl().very_high(),
            };
            w
        });
    }

    /// Peripheral-side base address.
    pub fn set_peripheral_address(&mut self, addr: u32) {
        assert!(
            !self.is_enabled(),
            "stop the DMA channel before reconfiguring it"
        );
        Self::ch().paddr().write(|w| unsafe { w.bits(addr) });
    }

    /// Memory-side base address.
    pub fn set_memory_address(&mut self, addr: u32) {
        assert!(
            !self.is_enabled(),
            "stop the DMA channel before reconfiguring it"
        );
        Self::ch().maddr().write(|w| unsafe { w.bits(addr) });
    }

    /// Transfer count (number of transfers, not bytes).
    pub fn set_count(&mut self, count: u16) {
        assert!(
            !self.is_enabled(),
            "stop the DMA channel before reconfiguring it"
        );
        Self::ch().dtcnt().write(|w| unsafe { w.bits(count) });
    }

    /// Enables the channel and starts the configured transfer.
    ///
    /// # Safety
    ///
    /// The configured endpoints, alignment, widths and count must describe a
    /// valid transfer. Keep memory accessible to DMA for the entire operation.
    /// Stop or reconfigure the channel before reusing an expired address.
    pub unsafe fn start(&mut self) {
        compiler_fence(Ordering::Release);
        Self::ch().ctrl().modify(|_, w| w.chen().enable());
    }

    /// Disables the channel and waits for it to stop.
    pub fn stop(&mut self) {
        Self::disable_channel();
    }

    fn disable_channel() {
        let ctrl = Self::ch().ctrl();
        ctrl.modify(|_, w| w.chen().disable());
        while ctrl.read().chen().is_enabled() {}
        cortex_m::asm::dsb();
        compiler_fence(Ordering::Acquire);
    }

    /// Stop a one-shot transfer on success, error or future drop.
    pub(crate) fn stop_transfer() {
        T::Interrupt::disable();
        Self::disable_channel();
        Self::ch()
            .ctrl()
            .modify(|_, w| w.fdtien().disable().hdtien().disable().dterrien().disable());
        Self::clear_all_flags();
        T::Interrupt::unpend();
    }

    /// Whether the channel is still transferring.
    pub fn is_enabled(&self) -> bool {
        Self::ch().ctrl().read().chen().is_enabled()
    }

    const FULL: u32 = 1 << 1;
    const HALF: u32 = 1 << 2;
    const ERROR: u32 = 1 << 3;

    fn flags() -> u32 {
        (Self::block().sts().read().bits() >> (4 * T::CH_INDEX)) & 0x0F
    }

    fn clear_flags(mask: u32) {
        Self::block()
            .clr()
            .write(|w| unsafe { w.bits(mask << (4 * T::CH_INDEX)) });
    }

    /// Clears all flags (global/half/full/error) of this channel.
    pub fn clear_all_flags() {
        Self::clear_flags(1);
    }

    /// Whether the half-transfer flag is set.
    pub fn half_transfer_flag(&self) -> bool {
        Self::flags() & Self::HALF != 0
    }

    /// Whether the full-transfer flag is set.
    pub fn full_transfer_flag(&self) -> bool {
        Self::flags() & Self::FULL != 0
    }

    /// Whether the transfer error flag is set.
    pub fn error_flag(&self) -> bool {
        Self::flags() & Self::ERROR != 0
    }

    /// Enables the transfer-complete interrupt and unmask the NVIC line (call
    /// once; the async `wait_transfer` relies on it).
    pub fn listen(&mut self) {
        Self::ch().ctrl().modify(|_, w| {
            w.fdtien().enable();
            w.hdtien().enable();
            w.dterrien().enable();
            w
        });
        T::Interrupt::unpend();
        T::Interrupt::enable();
    }

    /// Disables the channel interrupt in the NVIC.
    pub fn unlisten(&mut self) {
        Self::ch().ctrl().modify(|_, w| {
            w.fdtien().disable();
            w.hdtien().disable();
            w.dterrien().disable();
            w
        });
        T::Interrupt::disable();
    }

    /// Configures the channel for memory -> peripheral and arms it (request +
    /// config + addresses + count). `count` is in transfers.
    ///
    /// # Safety
    ///
    /// `mem_buf` must be valid for `count * memory_width.bytes()` bytes of
    /// reads and live until the transfer completes or the channel is stopped.
    /// The peripheral address must be valid for writes of the configured width.
    pub unsafe fn configure_memory_to_peripheral(
        &mut self,
        request: Request,
        peripheral_addr: u32,
        mem_buf: *const u8,
        count: u16,
        cfg: Config,
    ) -> Result<(), DmaError> {
        self.configure_common(request, peripheral_addr, mem_buf as u32, count, cfg)
    }

    /// Configures the channel for peripheral -> memory and arms it.
    ///
    /// # Safety
    ///
    /// `mem_buf` must be valid for `count * memory_width.bytes()` bytes of
    /// writes and live until the transfer completes or the channel is stopped.
    /// The peripheral address must be valid for reads of the configured width.
    pub unsafe fn configure_peripheral_to_memory(
        &mut self,
        request: Request,
        peripheral_addr: u32,
        mem_buf: *mut u8,
        count: u16,
        cfg: Config,
    ) -> Result<(), DmaError> {
        self.configure_common(request, peripheral_addr, mem_buf as u32, count, cfg)
    }

    fn configure_common(
        &mut self,
        request: Request,
        peripheral_addr: u32,
        mem_addr: u32,
        count: u16,
        cfg: Config,
    ) -> Result<(), DmaError> {
        if count == 0 {
            return Err(DmaError::CountTooLarge);
        }
        Self::stop_transfer();
        self.set_request(request);
        self.set_config(cfg);
        self.set_peripheral_address(peripheral_addr);
        self.set_memory_address(mem_addr);
        self.set_count(count);
        Ok(())
    }

    /// Blocking mem-to-memory transfer (src -> dst, same width, both sides
    /// incrementing). Uses no request line (M2M + full-transfer trigger).
    ///
    /// # Safety
    ///
    /// Both regions must be DMA-accessible, aligned to the hardware transfer
    /// width and valid for `count` initialized reads/writes without overlap.
    pub unsafe fn mem_to_mem<TW: Copy>(
        &mut self,
        dst: *mut TW,
        src: *const TW,
        count: usize,
    ) -> Result<(), DmaError> {
        if count == 0 {
            return Ok(());
        }
        self.prepare_mem_to_mem(dst, src, count)?;
        self.wait_done_blocking()
    }

    /// Async mem-to-memory: configure, start, await completion (requires
    /// `listen()` + the channel IRQ binding).
    ///
    /// # Safety
    ///
    /// Same requirements as [`Channel::mem_to_mem`]. If the pending future is
    /// forgotten, both regions must remain valid until the channel is stopped.
    pub async unsafe fn mem_to_mem_async<TW: Copy>(
        &mut self,
        dst: *mut TW,
        src: *const TW,
        count: usize,
    ) -> Result<(), DmaError> {
        if count == 0 {
            return Ok(());
        }
        self.prepare_mem_to_mem(dst, src, count)?;
        self.wait_transfer().await
    }

    unsafe fn prepare_mem_to_mem<TW: Copy>(
        &mut self,
        dst: *mut TW,
        src: *const TW,
        count: usize,
    ) -> Result<(), DmaError> {
        let width = match core::mem::size_of::<TW>() {
            1 => Width::Bits8,
            2 => Width::Bits16,
            4 => Width::Bits32,
            _ => return Err(DmaError::CountTooLarge),
        };
        let count = u16::try_from(count).map_err(|_| DmaError::CountTooLarge)?;
        let cfg = Config {
            direction: Direction::PeripheralToMemory,
            peripheral_width: width,
            memory_width: width,
            peripheral_increment: true,
            memory_increment: true,
            circular: false,
            priority: Priority::VeryHigh,
        };
        Self::stop_transfer();
        self.set_config(cfg);
        self.set_peripheral_address(src as u32);
        self.set_memory_address(dst as u32);
        self.set_count(count);
        // M2M mode: the channel triggers itself, no DMAMUX request needed.
        Self::ch().ctrl().modify(|_, w| w.m2m().enable());
        self.start();
        Ok(())
    }

    /// Blocking wait for transfer completion (FDTF) or error.
    pub fn wait_done_blocking(&mut self) -> Result<(), DmaError> {
        let result = loop {
            if let Poll::Ready(result) = Self::completion() {
                break result;
            }
        };
        Self::stop_transfer();
        result
    }

    fn completion() -> Poll<Result<(), DmaError>> {
        let flags = Self::flags();
        if flags & Self::ERROR != 0 {
            Poll::Ready(Err(DmaError::TransferError))
        } else if flags & Self::FULL != 0 {
            Poll::Ready(Ok(()))
        } else {
            Poll::Pending
        }
    }

    /// Waits for and consumes one circular-mode boundary (half or full).
    /// If both are pending, the half flag is consumed first. The caller must
    /// keep up with the DMA because the hardware does not count missed events.
    /// Bind the channel IRQ with `bind_interrupts!`.
    pub async fn wait_circular_event(&mut self) -> Result<CircularEvent, DmaError> {
        self.listen();
        poll_fn(|cx| {
            waker(T::DMA_INDEX, T::CH_INDEX).register(cx.waker());
            let flags = Self::flags();
            if flags & Self::ERROR != 0 {
                Self::stop_transfer();
                Poll::Ready(Err(DmaError::TransferError))
            } else if flags & Self::HALF != 0 {
                Self::clear_flags(Self::HALF);
                compiler_fence(Ordering::Acquire);
                Poll::Ready(Ok(CircularEvent::FirstHalf))
            } else if flags & Self::FULL != 0 {
                Self::clear_flags(Self::FULL);
                compiler_fence(Ordering::Acquire);
                Poll::Ready(Ok(CircularEvent::SecondHalf))
            } else {
                Poll::Pending
            }
        })
        .await
    }

    pub(crate) fn enable_transfer_irq() {
        T::Interrupt::disable();
        Self::ch()
            .ctrl()
            .modify(|_, w| w.fdtien().enable().dterrien().enable().hdtien().disable());
        T::Interrupt::unpend();
        T::Interrupt::enable();
    }

    pub(crate) fn poll_transfer(cx: &mut Context<'_>) -> Poll<Result<(), DmaError>> {
        waker(T::DMA_INDEX, T::CH_INDEX).register(cx.waker());
        Self::completion()
    }

    /// Waits for a one-shot transfer to finish and stops the channel.
    /// Bind the channel IRQ with `bind_interrupts!`.
    pub async fn wait_transfer(&mut self) -> Result<(), DmaError> {
        let _stop = OnDrop::new(Self::stop_transfer);
        Self::enable_transfer_irq();
        poll_fn(Self::poll_transfer).await
    }
}

/// DMA channel interrupt handler: bind one per channel via `bind_interrupts!`.
pub struct InterruptHandler<T: ChannelInstance> {
    _phantom: PhantomData<T>,
}

impl<T: ChannelInstance> Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let flags = Channel::<T>::flags();
        if flags & (Channel::<T>::FULL | Channel::<T>::HALF | Channel::<T>::ERROR) != 0 {
            // Keep the event pending for the waiter. Disable its interrupt
            // source to avoid re-entering the IRQ before the task can run.
            Channel::<T>::ch().ctrl().modify(|_, w| {
                if flags & Channel::<T>::FULL != 0 {
                    w.fdtien().disable();
                }
                if flags & Channel::<T>::HALF != 0 {
                    w.hdtien().disable();
                }
                if flags & Channel::<T>::ERROR != 0 {
                    w.dterrien().disable();
                }
                w
            });
            waker(T::DMA_INDEX, T::CH_INDEX).wake();
        }
    }
}

macro_rules! impl_dma_channel {
    ($ch:ident, $dma_index:expr, $ch_index:expr, $irq:ident) => {
        impl SealedChannel for crate::peripherals::$ch {}

        impl ChannelInstance for crate::peripherals::$ch {
            const DMA_INDEX: usize = $dma_index;
            const CH_INDEX: usize = $ch_index;
            type Interrupt = interrupt::typelevel::$irq;
        }
    };
}

impl_dma_channel!(DMA1_CH1, 0, 0, DMA1_CHANNEL1);
impl_dma_channel!(DMA1_CH2, 0, 1, DMA1_CHANNEL2);
impl_dma_channel!(DMA1_CH3, 0, 2, DMA1_CHANNEL3);
impl_dma_channel!(DMA1_CH4, 0, 3, DMA1_CHANNEL4);
impl_dma_channel!(DMA1_CH5, 0, 4, DMA1_CHANNEL5);
impl_dma_channel!(DMA1_CH6, 0, 5, DMA1_CHANNEL6);
impl_dma_channel!(DMA1_CH7, 0, 6, DMA1_CHANNEL7);
impl_dma_channel!(DMA2_CH1, 1, 0, DMA2_CHANNEL1);
impl_dma_channel!(DMA2_CH2, 1, 1, DMA2_CHANNEL2);
impl_dma_channel!(DMA2_CH3, 1, 2, DMA2_CHANNEL3);
impl_dma_channel!(DMA2_CH4, 1, 3, DMA2_CHANNEL4);
impl_dma_channel!(DMA2_CH5, 1, 4, DMA2_CHANNEL5);
impl_dma_channel!(DMA2_CH6, 1, 5, DMA2_CHANNEL6);
impl_dma_channel!(DMA2_CH7, 1, 6, DMA2_CHANNEL7);
