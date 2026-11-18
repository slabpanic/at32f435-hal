//! EDMA controller (RM 29): the 8-stream high-throughput DMA with per-stream
//! FIFOs, bursts and 2D/link-list extensions, dedicated to the high-bandwidth
//! peripherals (SDIO/QSPI/DVP request map).
//!
//! Each stream has its own NVIC line (`EDMA_STREAM1..8`) and its own flag
//! cluster (STS1 holds streams 1-4, STS2 streams 5-8; flags are
//! write-1-to-clear via CLR1/CLR2). Requests route through the on-chip DMAMUX:
//! set `MUXSEL.TBL_SEL`, then the per-stream `MUXSxCTRL.REQSEL` value (RM
//! table 29-3; e.g. ADC1 = 0x05, SDIO1 = 0x27, QSPI1 = 0x28, DVP = 0x69).
//! Memory-to-memory transfers need no request line - `DTD = 0b10`
//! self-triggers with `paddr` as the source and `m0addr` as the destination.
//!
//! The 4-word FIFO needs threshold and memory burst to match
//! (`FTHSEL*4 = MBURST*MWIDTH*M`, RM 29.3.6); mismatching them raises FERR and
//! the stream disables itself. Direct mode (FEN = 0) avoids the issue for
//! single-transfer setups.
//!
//! ```ignore
//! let mut s = hal::edma::Stream::new(p.EDMA_STREAM1);
//! unsafe { s.mem_to_mem(&mut dst, &src, 128)? };
//! ```
//!
//! Register layout (RM 29.4): stream clusters at `0x10 + 0x18*n` with
//! CTRL/DTCNT/PADDR/M0ADDR/M1ADDR/FCTRL; STS1/STS2/CLR1/CLR2 at 0x00..0x0C
//! (6 flag bits per stream: FERRF@+0, reserved, DMERRF@+2, DTERRF@+3,
//! HDTF@+4, FDTF@+5); MUXSEL at 0x13C and MUXSxCTRL at 0x140 + 4*n. The PAC
//! generates distinct register types per stream, so the driver accesses the
//! clusters through computed addresses with volatile word operations (the
//! RM 29.4 word-access rule applies to EDMA registers).

use core::future::poll_fn;
use core::marker::PhantomData;
use core::sync::atomic::{compiler_fence, Ordering};
use core::task::Poll;
use embassy_hal_internal::drop::OnDrop;

use embassy_sync::waitqueue::AtomicWaker;

use crate::interrupt::typelevel::{Handler, Interrupt as _};
use crate::{interrupt, pac, Peri};

// Register offsets (RM 29.4).
const STS1: usize = 0x00;
const STS2: usize = 0x04;
const CLR1: usize = 0x08;
const CLR2: usize = 0x0C;
const S_CTRL: usize = 0x10; // stream1 CTRL; cluster stride 0x18
const S_DTCNT: usize = 0x04;
const S_PADDR: usize = 0x08;
const S_M0ADDR: usize = 0x0C;
const S_M1ADDR: usize = 0x10;
const S_FCTRL: usize = 0x14;
const MUXSEL: usize = 0x13C;
const MUXS_CTRL: usize = 0x140;

#[inline]
unsafe fn rd(addr: usize) -> u32 {
    (addr as *const u32).read_volatile()
}

#[inline]
unsafe fn wr(addr: usize, v: u32) {
    (addr as *mut u32).write_volatile(v);
}

#[inline]
unsafe fn rmw(addr: usize, f: impl FnOnce(u32) -> u32) {
    wr(addr, f(rd(addr)));
}

// CTRL bits (identical layout for every stream).
const SEN: u32 = 1 << 0;
const DMERRIEN: u32 = 1 << 1;
const DTERRIEN: u32 = 1 << 2;
const FDTIEN: u32 = 1 << 4;
const FERRIEN: u32 = 1 << 7;
const LM: u32 = 1 << 8;
const PINCM: u32 = 1 << 9;
const MINCM: u32 = 1 << 10;

/// Data transfer width.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
pub enum Width {
    #[default]
    Bits8,
    Bits16,
    Bits32,
}

impl Width {
    const fn code(self) -> u8 {
        match self {
            Width::Bits8 => 0,
            Width::Bits16 => 1,
            Width::Bits32 => 2,
        }
    }

    /// Width in bytes.
    pub const fn bytes(self) -> usize {
        match self {
            Width::Bits8 => 1,
            Width::Bits16 => 2,
            Width::Bits32 => 4,
        }
    }
}

/// Arbitration priority among streams (`CTRL.SPL[17:16]`).
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
pub enum Priority {
    #[default]
    Low,
    Medium,
    High,
    VeryHigh,
}

impl Priority {
    const fn code(self) -> u8 {
        match self {
            Priority::Low => 0,
            Priority::Medium => 1,
            Priority::High => 2,
            Priority::VeryHigh => 3,
        }
    }
}

/// Transfer direction (`CTRL.DTD[7:6]`).
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
pub enum Direction {
    /// Peripheral -> memory (P2M).
    #[default]
    PeripheralToMemory,
    /// Memory -> peripheral (M2P).
    MemoryToPeripheral,
    /// Memory -> memory: `paddr` is the source, `m0addr` the destination, no
    /// DMAMUX request needed.
    MemoryToMemory,
}

impl Direction {
    const fn code(self) -> u8 {
        match self {
            Direction::PeripheralToMemory => 0,
            Direction::MemoryToPeripheral => 1,
            Direction::MemoryToMemory => 2,
        }
    }
}

/// FIFO threshold (`FCTRL.FTHSEL[1:0]`). Must satisfy
/// `FTHSEL*4 = MBURST*MWIDTH*M` (RM 29.3.6) or the stream self-disables with
/// FERR on enable.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
pub enum FifoThreshold {
    #[default]
    Quarter,
    Half,
    ThreeQuarters,
    Full,
}

impl FifoThreshold {
    const fn code(self) -> u8 {
        match self {
            FifoThreshold::Quarter => 0,
            FifoThreshold::Half => 1,
            FifoThreshold::ThreeQuarters => 2,
            FifoThreshold::Full => 3,
        }
    }
}

/// AHB burst size (`CTRL.PBURST[22:21]` / `CTRL.MBURST[24:23]`).
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
pub enum Burst {
    #[default]
    Single,
    Inc4,
    Inc8,
    Inc16,
}

impl Burst {
    const fn code(self) -> u8 {
        match self {
            Burst::Single => 0,
            Burst::Inc4 => 1,
            Burst::Inc8 => 2,
            Burst::Inc16 => 3,
        }
    }
}

/// Per-transfer configuration.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct Config {
    pub direction: Direction,
    pub peripheral_width: Width,
    pub memory_width: Width,
    pub peripheral_increment: bool,
    pub memory_increment: bool,
    /// Loop (circular) mode: reload at completion (CTRL.LM).
    pub circular: bool,
    pub priority: Priority,
    /// FIFO mode (FCTRL.FEN); false = direct mode (FIFO bypassed).
    pub fifo_mode: bool,
    pub fifo_threshold: FifoThreshold,
    pub peripheral_burst: Burst,
    pub memory_burst: Burst,
}

impl Config {
    /// Direct-mode peripheral -> memory (memory increments, FIFO off).
    pub const fn peripheral_to_memory(width: Width) -> Self {
        Self {
            direction: Direction::PeripheralToMemory,
            peripheral_width: width,
            memory_width: width,
            peripheral_increment: false,
            memory_increment: true,
            circular: false,
            priority: Priority::VeryHigh,
            fifo_mode: false,
            fifo_threshold: FifoThreshold::Quarter,
            peripheral_burst: Burst::Single,
            memory_burst: Burst::Single,
        }
    }

    /// Direct-mode memory -> peripheral.
    pub const fn mem_to_peripheral(width: Width) -> Self {
        Self {
            direction: Direction::MemoryToPeripheral,
            peripheral_width: width,
            memory_width: width,
            peripheral_increment: false,
            memory_increment: true,
            circular: false,
            priority: Priority::VeryHigh,
            fifo_mode: false,
            fifo_threshold: FifoThreshold::Quarter,
            peripheral_burst: Burst::Single,
            memory_burst: Burst::Single,
        }
    }
}

/// Stream transfer error.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[non_exhaustive]
pub enum EdmaError {
    /// DTERRF: transfer error on the AHB bus.
    TransferError,
    /// DMERRF: direct-mode error.
    DirectModeError,
    /// FERRF: FIFO threshold/burst mismatch (stream self-disabled).
    FifoError,
    /// Count does not fit the 16-bit DTCNT.
    CountTooLarge,
}

#[cfg(feature = "defmt")]
impl defmt::Format for EdmaError {
    fn format(&self, fmt: defmt::Formatter) {
        match self {
            EdmaError::TransferError => defmt::write!(fmt, "TransferError"),
            EdmaError::DirectModeError => defmt::write!(fmt, "DirectModeError"),
            EdmaError::FifoError => defmt::write!(fmt, "FifoError"),
            EdmaError::CountTooLarge => defmt::write!(fmt, "CountTooLarge"),
        }
    }
}

static WAKERS: [AtomicWaker; 8] = [
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
fn waker(stream: usize) -> &'static AtomicWaker {
    &WAKERS[stream]
}

trait SealedStream {}

/// EDMA stream instance trait (per-stream singleton).
#[allow(private_bounds)]
pub trait StreamInstance: SealedStream + embassy_hal_internal::PeripheralType + 'static {
    /// Stream index inside the controller (0..7, i.e. stream1..stream8).
    const STREAM_INDEX: usize;
    /// The stream's own NVIC interrupt.
    type Interrupt: interrupt::typelevel::Interrupt;
}

/// EDMA stream handle.
pub struct Stream<'d, T: StreamInstance> {
    _peri: Peri<'d, T>,
    _mode: PhantomData<T>,
}

impl<'d, T: StreamInstance> Stream<'d, T> {
    #[inline]
    fn base() -> usize {
        pac::EDMA::PTR as usize
    }

    /// Stream cluster base (CTRL at +0, DTCNT +4, PADDR +8, M0 +0xC,
    /// M1 +0x10, FCTRL +0x14).
    #[inline]
    fn stream_base() -> usize {
        Self::base() + S_CTRL + 0x18 * T::STREAM_INDEX
    }

    /// Takes ownership of a stream singleton: enables the EDMA clock, resets
    /// the stream registers and leaves the stream disabled.
    pub fn new(peri: Peri<'d, T>) -> Self {
        critical_section::with(|_| {
            let crm = unsafe { &*pac::CRM::PTR };
            crm.ahben1().modify(|_, w| w.edma().set_bit());
            let _ = crm.ahben1().read();
        });
        Self::stop_transfer();
        // Reset only this stream. Preserve the FCTRL initialization used by
        // the vendor's edma_reset routine.
        let sb = Self::stream_base();
        unsafe {
            wr(sb, 0);
            wr(sb + S_DTCNT, 0);
            wr(sb + S_PADDR, 0);
            wr(sb + S_M0ADDR, 0);
            wr(sb + S_M1ADDR, 0);
            wr(sb + S_FCTRL, 0x21);
        }
        Self::clear_all_flags();
        Self {
            _peri: peri,
            _mode: PhantomData,
        }
    }

    /// Enables the flexible DMAMUX mapping table and writes this stream's
    /// request line (`reqsel`, RM table 29-3). Not needed for mem2mem.
    pub fn set_request(&mut self, reqsel: u8) {
        assert!(
            !self.is_enabled(),
            "stop the DMA stream before reconfiguring it"
        );
        let b = Self::base();
        critical_section::with(|_| unsafe { rmw(b + MUXSEL, |v| v | 1) });
        unsafe { wr(b + MUXS_CTRL + 4 * T::STREAM_INDEX, u32::from(reqsel)) };
    }

    /// Applies the transfer configuration (stream must be disabled).
    pub fn set_config(&mut self, cfg: Config) {
        assert!(
            !self.is_enabled(),
            "stop the DMA stream before reconfiguring it"
        );
        let sb = Self::stream_base();
        let mut ctrl = 0u32;
        ctrl |= (cfg.direction.code() as u32) << 6;
        ctrl |= if cfg.peripheral_increment { PINCM } else { 0 };
        ctrl |= if cfg.memory_increment { MINCM } else { 0 };
        ctrl |= (cfg.peripheral_width.code() as u32) << 11;
        ctrl |= (cfg.memory_width.code() as u32) << 13;
        ctrl |= if cfg.circular { LM } else { 0 };
        ctrl |= (cfg.priority.code() as u32) << 16;
        ctrl |= (cfg.peripheral_burst.code() as u32) << 21;
        ctrl |= (cfg.memory_burst.code() as u32) << 23;
        let fctrl = (u32::from(cfg.fifo_mode) << 2) | (cfg.fifo_threshold.code() as u32);
        unsafe {
            wr(sb, ctrl);
            wr(sb + S_FCTRL, fctrl);
        }
    }

    /// Peripheral-side base address (the SOURCE for mem2mem transfers).
    pub fn set_peripheral_address(&mut self, addr: u32) {
        assert!(
            !self.is_enabled(),
            "stop the DMA stream before reconfiguring it"
        );
        unsafe { wr(Self::stream_base() + S_PADDR, addr) };
    }

    /// Memory0 base address (the DESTINATION for mem2mem transfers).
    pub fn set_memory_address(&mut self, addr: u32) {
        assert!(
            !self.is_enabled(),
            "stop the DMA stream before reconfiguring it"
        );
        unsafe { wr(Self::stream_base() + S_M0ADDR, addr) };
    }

    /// Memory1 base address (double-buffer mode; not configured by this
    /// driver's helpers).
    pub fn set_memory1_address(&mut self, addr: u32) {
        assert!(
            !self.is_enabled(),
            "stop the DMA stream before reconfiguring it"
        );
        unsafe { wr(Self::stream_base() + S_M1ADDR, addr) };
    }

    /// Transfer count in transfer units (DTCNT, 16-bit).
    pub fn set_count(&mut self, count: u16) {
        assert!(
            !self.is_enabled(),
            "stop the DMA stream before reconfiguring it"
        );
        unsafe { wr(Self::stream_base() + S_DTCNT, count as u32) };
    }

    /// Enables the stream and starts the configured transfer.
    ///
    /// # Safety
    ///
    /// The configured endpoints, alignment, widths, bursts and count must
    /// describe a valid transfer. Keep memory accessible to EDMA until the
    /// operation ends. Stop or reconfigure before reusing an expired address.
    pub unsafe fn start(&mut self) {
        compiler_fence(Ordering::Release);
        rmw(Self::stream_base(), |v| v | SEN);
    }

    /// Disables the stream and waits for it to stop.
    pub fn stop(&mut self) {
        Self::disable_stream();
    }

    fn disable_stream() {
        let addr = Self::stream_base();
        unsafe {
            rmw(addr, |v| v & !SEN);
            while rd(addr) & SEN != 0 {}
        }
        cortex_m::asm::dsb();
        compiler_fence(Ordering::Acquire);
    }

    fn stop_transfer() {
        T::Interrupt::disable();
        Self::disable_stream();
        // Mask interrupt sources before clearing the status flags.
        let addr = Self::stream_base();
        unsafe {
            rmw(addr, |v| v & !(FDTIEN | DTERRIEN | DMERRIEN));
            rmw(addr + S_FCTRL, |v| v & !FERRIEN);
        }
        Self::clear_all_flags();
        T::Interrupt::unpend();
    }

    /// Whether the stream is still enabled.
    pub fn is_enabled(&self) -> bool {
        unsafe { rd(Self::stream_base()) & SEN != 0 }
    }

    /// Clears every flag of this stream (FERR/DMERR/DTERR/HDT/FDT).
    pub fn clear_all_flags() {
        let b = Self::base();
        let n = T::STREAM_INDEX;
        // 6-bit flag block per stream: FERRF@0, DMERRF@2, DTERRF@3, HDTF@4,
        // FDTF@5 (the official EDMA_STREAMx_INT_MASK = 0x3D).
        let mask = 0x3D << (6 * (n % 4));
        let clr = if n < 4 { CLR1 } else { CLR2 };
        unsafe { wr(b + clr, mask) };
    }

    const FIFO_ERROR: u32 = 1 << 0;
    const DIRECT_ERROR: u32 = 1 << 2;
    const TRANSFER_ERROR: u32 = 1 << 3;
    const FULL: u32 = 1 << 5;

    fn flags() -> u32 {
        let n = T::STREAM_INDEX;
        let sts = unsafe { rd(Self::base() + if n < 4 { STS1 } else { STS2 }) };
        (sts >> (6 * (n % 4))) & 0x3F
    }

    /// Reads the full-transfer flag (FDTF).
    pub fn full_transfer_flag(&self) -> bool {
        Self::flags() & Self::FULL != 0
    }

    /// Reads the transfer-error flag (DTERRF).
    pub fn transfer_error_flag(&self) -> bool {
        Self::flags() & Self::TRANSFER_ERROR != 0
    }

    /// Reads the FIFO error flag (FERRF).
    pub fn fifo_error_flag(&self) -> bool {
        Self::flags() & Self::FIFO_ERROR != 0
    }

    /// Enables the completion/error interrupts and unmasks the NVIC line.
    pub fn listen(&mut self) {
        let addr = Self::stream_base();
        T::Interrupt::disable();
        unsafe {
            rmw(addr, |v| v | FDTIEN | DTERRIEN | DMERRIEN);
            rmw(addr + S_FCTRL, |v| v | FERRIEN);
        }
        T::Interrupt::unpend();
        T::Interrupt::enable();
    }

    /// Disables the stream interrupts in the NVIC.
    pub fn unlisten(&mut self) {
        T::Interrupt::disable();
        let addr = Self::stream_base();
        unsafe {
            rmw(addr, |v| v & !(FDTIEN | DTERRIEN | DMERRIEN));
            rmw(addr + S_FCTRL, |v| v & !FERRIEN);
        }
    }

    /// Configures the stream for peripheral -> memory and arms it.
    ///
    /// # Safety
    ///
    /// `mem_buf` must be valid for `count * memory_width.bytes()` bytes of
    /// writes and live until the transfer completes or the stream stops.
    pub unsafe fn configure_peripheral_to_memory(
        &mut self,
        reqsel: u8,
        peripheral_addr: u32,
        mem_buf: *mut u8,
        count: u16,
        cfg: Config,
    ) -> Result<(), EdmaError> {
        if count == 0 {
            return Err(EdmaError::CountTooLarge);
        }
        Self::stop_transfer();
        self.set_request(reqsel);
        self.set_config(cfg);
        self.set_peripheral_address(peripheral_addr);
        self.set_memory_address(mem_buf as u32);
        self.set_count(count);
        Ok(())
    }

    /// Configures the stream for memory -> peripheral and arms it.
    ///
    /// # Safety
    ///
    /// `mem_buf` must be valid for `count * memory_width.bytes()` bytes of
    /// reads and live until the transfer completes or the stream stops.
    pub unsafe fn configure_memory_to_peripheral(
        &mut self,
        reqsel: u8,
        peripheral_addr: u32,
        mem_buf: *const u8,
        count: u16,
        cfg: Config,
    ) -> Result<(), EdmaError> {
        if count == 0 {
            return Err(EdmaError::CountTooLarge);
        }
        Self::stop_transfer();
        self.set_request(reqsel);
        self.set_config(cfg);
        self.set_peripheral_address(peripheral_addr);
        self.set_memory_address(mem_buf as u32);
        self.set_count(count);
        Ok(())
    }

    /// Blocking mem-to-memory transfer (src -> dst, same width on both
    /// sides, both incrementing, direct mode).
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
    ) -> Result<(), EdmaError> {
        if count == 0 {
            return Ok(());
        }
        self.prepare_mem_to_mem(dst, src, count)?;
        self.wait_done_blocking()
    }

    /// Async mem-to-memory (requires `listen()` + the stream IRQ binding).
    ///
    /// # Safety
    ///
    /// Same requirements as [`Stream::mem_to_mem`]. If the pending future is
    /// forgotten, both regions must remain valid until the stream is stopped.
    pub async unsafe fn mem_to_mem_async<TW: Copy>(
        &mut self,
        dst: *mut TW,
        src: *const TW,
        count: usize,
    ) -> Result<(), EdmaError> {
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
    ) -> Result<(), EdmaError> {
        let width = match core::mem::size_of::<TW>() {
            1 => Width::Bits8,
            2 => Width::Bits16,
            4 => Width::Bits32,
            _ => return Err(EdmaError::CountTooLarge),
        };
        let count = u16::try_from(count).map_err(|_| EdmaError::CountTooLarge)?;
        Self::stop_transfer();
        self.set_config(Config {
            direction: Direction::MemoryToMemory,
            peripheral_width: width,
            memory_width: width,
            peripheral_increment: true,
            memory_increment: true,
            circular: false,
            priority: Priority::VeryHigh,
            // Direct mode only moves full words on the AHB; sub-word mem2mem
            // needs the FIFO (verified on-silicon: byte mem2mem with FEN=0
            // never completes, with FEN=1 it does).
            fifo_mode: width != Width::Bits32,
            fifo_threshold: FifoThreshold::Quarter,
            peripheral_burst: Burst::Single,
            memory_burst: Burst::Single,
        });
        // M2M: paddr = source, m0addr = destination, no DMAMUX request.
        self.set_peripheral_address(src as u32);
        self.set_memory_address(dst as u32);
        self.set_count(count);
        self.start();
        Ok(())
    }

    fn completion(poll_count: bool) -> Poll<Result<(), EdmaError>> {
        let flags = Self::flags();
        if flags & Self::TRANSFER_ERROR != 0 {
            Poll::Ready(Err(EdmaError::TransferError))
        } else if flags & Self::DIRECT_ERROR != 0 {
            Poll::Ready(Err(EdmaError::DirectModeError))
        } else if flags & Self::FIFO_ERROR != 0 {
            Poll::Ready(Err(EdmaError::FifoError))
        } else if flags & Self::FULL != 0 {
            Poll::Ready(Ok(()))
        } else if poll_count && unsafe { rd(Self::stream_base() + S_DTCNT) } == 0 {
            // Sub-word mem2mem can drain without setting FDTF. Allow the
            // last FIFO/AHB write to settle before shutting down the stream.
            cortex_m::asm::delay(64);
            Poll::Ready(Ok(()))
        } else {
            Poll::Pending
        }
    }

    fn uses_count_completion() -> bool {
        let ctrl = unsafe { rd(Self::stream_base()) };
        let direction = (ctrl >> 6) & 0b11;
        let memory_width = (ctrl >> 13) & 0b11;
        direction == u32::from(Direction::MemoryToMemory.code()) && memory_width != 0b10
    }

    /// Blocking one-shot wait for completion or a controller error.
    pub fn wait_done_blocking(&mut self) -> Result<(), EdmaError> {
        let poll_count = Self::uses_count_completion();
        let result = loop {
            if let Poll::Ready(result) = Self::completion(poll_count) {
                break result;
            }
        };
        Self::stop_transfer();
        result
    }

    /// Waits for a one-shot transfer and stops the stream. Bind its IRQ with
    /// `bind_interrupts!`. Sub-word mem2mem also checks the count because
    /// that hardware path can finish without generating a completion IRQ.
    pub async fn wait_transfer(&mut self) -> Result<(), EdmaError> {
        let _stop = OnDrop::new(Self::stop_transfer);
        let poll_count = Self::uses_count_completion();
        self.listen();
        poll_fn(|cx| {
            waker(T::STREAM_INDEX).register(cx.waker());
            let result = Self::completion(poll_count);
            if poll_count && result.is_pending() {
                cx.waker().wake_by_ref();
            }
            result
        })
        .await
    }
}

/// EDMA stream interrupt handler: bind one per stream via `bind_interrupts!`.
pub struct InterruptHandler<T: StreamInstance> {
    _phantom: PhantomData<T>,
}

impl<T: StreamInstance> Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let flags = Stream::<T>::flags();
        if flags
            & (Stream::<T>::FULL
                | Stream::<T>::TRANSFER_ERROR
                | Stream::<T>::DIRECT_ERROR
                | Stream::<T>::FIFO_ERROR)
            != 0
        {
            // The waiter consumes the flags. Mask the sources so the
            // interrupt does not fire again while the task is waking up.
            let addr = Stream::<T>::stream_base();
            rmw(addr, |v| v & !(FDTIEN | DTERRIEN | DMERRIEN));
            rmw(addr + S_FCTRL, |v| v & !FERRIEN);
            waker(T::STREAM_INDEX).wake();
        }
    }
}

macro_rules! impl_edma_stream {
    ($stream:ident, $index:expr, $irq:ident) => {
        impl SealedStream for crate::peripherals::$stream {}

        impl StreamInstance for crate::peripherals::$stream {
            const STREAM_INDEX: usize = $index;
            type Interrupt = interrupt::typelevel::$irq;
        }
    };
}

impl_edma_stream!(EDMA_STREAM1, 0, EDMA_STREAM1);
impl_edma_stream!(EDMA_STREAM2, 1, EDMA_STREAM2);
impl_edma_stream!(EDMA_STREAM3, 2, EDMA_STREAM3);
impl_edma_stream!(EDMA_STREAM4, 3, EDMA_STREAM4);
impl_edma_stream!(EDMA_STREAM5, 4, EDMA_STREAM5);
impl_edma_stream!(EDMA_STREAM6, 5, EDMA_STREAM6);
impl_edma_stream!(EDMA_STREAM7, 6, EDMA_STREAM7);
impl_edma_stream!(EDMA_STREAM8, 7, EDMA_STREAM8);
