//! USB host mode for OTGFS1 / OTGFS2 (Synopsys OTG FS core).
//!
//! The host stack comes from [`embassy_usb_synopsys_otg::host`] (slave-mode /
//! PIO host: TX data is pushed into the non-periodic FIFO and RX packets are
//! popped from the global RX FIFO on the RXFLVL interrupt). This module is the
//! AT32 adaptation layer on top of it, plus a standard enumeration helper.
//!
//! AT32 specifics handled here (all verified against the Artery official host
//! stack `usbh_core.c` / `at32f435_437_usb.c`, register bits cross-checked with
//! RM 21.6.3/21.6.4 and with hardware register dumps):
//!
//! 1. The core-id dispatch inside the crate's host init (`configure_as_host`)
//!    reads the ID register at offset 0x3C - on AT32 that is the GUID, which
//!    reads 0x2000, so the crate takes its "core v2" GCCFG path. In the v2 bit
//!    layout bit21 is VBDEN, which the crate clears - on AT32 bit21 is VBUSIG,
//!    so that write disables the VBUS override. [`UsbHost`] therefore re-writes
//!    the AT32 GCCFG patch (`PWRDOWN = 1` PHY power-up + `VBUSIG = 1` no VBUS
//!    sensing wired, same as the device side) after every
//!    `wait_for_device_event` call, and once in `new()` before the stack runs.
//! 2. `GUSBCFG.USBTIM` = 0x5: the 16-bit UTMI+ MAC turn-around time (RM
//!    21.6.3.4; also the reset value, so this is a re-affirm, not a change).
//! 3. `HFIR` is deliberately NOT written before the port is enabled: RM
//!    21.6.4.2 only allows writing it once PRTENA is set (earlier writes are
//!    silently ignored - hw-verified, HFIR stayed at its 0xEA60 reset value).
//!    The stack writes 48000 after the port comes up, which is correct.
//! 4. The crate sizes the host FIFOs as RX=half, NP-TX=quarter, P-TX=quarter.
//!    For the AT32 320-word FIFO that means the non-periodic TX FIFO is only
//!    80 words = 320 B, which caps OUT transfer sizes at 320 B. After the core
//!    is initialized, [`UsbHost`] retunes the split to RX=128 / NP-TX=160 /
//!    P-TX=32 words, so a full 512-byte OUT transfer fits the NP FIFO (IN
//!    transfers are unlimited: the RX FIFO only ever holds one packet at a
//!    time because the RXFLVL handler drains it per packet).
//!
//! Usage (evb jumper loopback: OTGFS1 host <-> OTGFS2 device, PA11->PB14
//! D-, PA12->PB15 D+):
//! ```ignore
//! use at32f435_hal::{bind_interrupts, usb::host::{UsbHost, HostInterruptHandler,
//!     enumerate, pipe, UsbHostAllocator, UsbHostController}};
//!
//! bind_interrupts!(struct Irqs {
//!     OTGFS1 => HostInterruptHandler<hal::peripherals::USB_OTG1>;
//! });
//! let mut host = UsbHost::new(p.USB_OTG1, p.PA12, p.PA11, Irqs);
//! let ev = host.wait_for_device_event().await; // Connected(speed) / Disconnected
//! if let DeviceEvent::Connected(speed) = ev {
//!     let info = enumerate(&host.allocator(), speed).await?;
//!     // info.vid / info.pid / info.max_packet_size0 ...
//! }
//! ```
//!
//! Transfer size limits (slave-mode host, both verified against the official
//! FIFO layout): a single OUT transfer must fit the non-periodic TX FIFO
//! (160 words = 640 B after the retune); interrupt/isochronous OUT transfers
//! must fit the periodic TX FIFO (32 words = 128 B). IN transfers are not
//! limited by the FIFO. Bulk/interrupt pipes are allocated through
//! [`UsbHostAllocator::alloc_pipe`] with type-level type/direction markers.

use core::marker::PhantomData;

use embassy_usb_synopsys_otg::host::{on_host_interrupt, OtgHost, OtgHostInstance};
use embassy_usb_synopsys_otg::PhyType;

// Re-exports for consumers (also used within this module): the pipe/direction/
// endpoint types and the trait surface of the host API all come from
// embassy-usb-driver.
pub use embassy_usb_driver::host::{
    pipe, DeviceEvent, HostError, PipeError, TimeoutConfig, UsbHostAllocator, UsbHostController,
    UsbPipe,
};
pub use embassy_usb_driver::{Direction, EndpointAddress, EndpointInfo, EndpointType, Speed};

use crate::gpio::{AfType, OutputType, Pull, Speed as PinSpeed};
use crate::interrupt::typelevel::{Binding, Handler, Interrupt as _};
use crate::Peri;

use critical_section;

use super::Instance;

// Host FIFO retune (difference #4 in the module docs): total 320 words.
const HOST_RX_FIFO_WORDS: u16 = 128;
const HOST_NPTX_FIFO_WORDS: u16 = 160;
// Periodic FIFO gets what is left (32 words).

/// Host-mode interrupt handler: bind it to the `OTGFS1` / `OTGFS2` vectors via
/// `bind_interrupts!`.
pub struct HostInterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> Handler<T::Interrupt> for HostInterruptHandler<T> {
    unsafe fn on_interrupt() {
        on_host_interrupt(T::regs(), &T::host_state());
    }
}

/// USB host controller driver (one per instance; the stack itself lives in
/// embassy-usb-synopsys-otg).
pub struct UsbHost<'d, T: Instance> {
    inner: OtgHost<'d>,
    fifo_tuned: bool,
    _phantom: PhantomData<&'d mut T>,
}

impl<'d, T: Instance> UsbHost<'d, T> {
    /// Build the FS host driver. The core and the 48 MHz control clock are
    /// configured here; mode switching, FIFO setup and interrupt arming happen
    /// lazily on the first [`UsbHostController::wait_for_device_event`].
    pub fn new(
        _peri: Peri<'d, T>,
        dp: Peri<'d, impl super::DpPin<T>>,
        dm: Peri<'d, impl super::DmPin<T>>,
        _irq: impl Binding<T::Interrupt, HostInterruptHandler<T>> + 'd,
    ) -> Self {
        // The host drives the bus (and provides the pull-downs internally), so
        // the pins get the same push-pull mux as in device mode.
        dp.set_as_af(
            dp.af_num(),
            AfType::output(OutputType::PushPull, PinSpeed::High.into(), Pull::None),
        );
        dm.set_as_af(
            dm.af_num(),
            AfType::output(OutputType::PushPull, PinSpeed::High.into(), Pull::None),
        );

        // USB control clock 48 MHz (shared with the other OTG core, idempotent).
        super::configure_usb_48m();

        // NVIC: clear pending first, then enable (the ISR stays quiet until the
        // stack arms GINTMSK on the first wait_for_device_event).
        T::Interrupt::unpend();
        T::Interrupt::enable();

        // CRM clock enable + peripheral reset pulse.
        T::enable_and_reset();

        let r = T::regs();
        // Wait for the AHB host to be idle after the reset pulse (RM 21.5.1).
        while !r.grstctl().read().ahbidl() {}

        // AT32 patches (module doc differences #1 - #3): GCCFG PWRDOWN + VBUSIG,
        // GUSBCFG TRDT = 0x5, HFIR = 48000. The crate's host init skips its own
        // GCCFG dispatch on AT32 (the CID address is repurposed), so this patch
        // must run here - it survives until the stack configures the core
        // because no soft reset happens in between.
        T::host_preinit();

        Self {
            inner: OtgHost::new(OtgHostInstance {
                regs: T::regs(),
                state: T::host_state(),
                fifo_depth_words: T::FIFO_DEPTH_WORDS,
                phy_type: PhyType::InternalFullSpeed,
            }),
            fifo_tuned: false,
            _phantom: PhantomData,
        }
    }

    /// Retune the host FIFO split once the stack has initialized the core
    /// (module doc difference #4). Safe because at event-delivery time no
    /// channel is active; the flush discards nothing.
    fn tune_fifos(&mut self) {
        if self.fifo_tuned {
            return;
        }
        self.fifo_tuned = true;

        let r = T::regs();
        critical_section::with(|_| {
            r.grxfsiz().modify(|w| w.set_rxfd(HOST_RX_FIFO_WORDS));
            r.hnptxfsiz().write(|w| {
                w.set_sa(HOST_RX_FIFO_WORDS);
                w.set_fd(HOST_NPTX_FIFO_WORDS);
            });
            r.hptxfsiz().write(|w| {
                w.set_sa(HOST_RX_FIFO_WORDS + HOST_NPTX_FIFO_WORDS);
                w.set_fd(T::FIFO_DEPTH_WORDS - HOST_RX_FIFO_WORDS - HOST_NPTX_FIFO_WORDS);
            });
        });

        // Flush everything so the new boundaries take effect immediately.
        r.grstctl().write(|w| {
            w.set_rxfflsh(true);
            w.set_txfflsh(true);
            w.set_txfnum(0x10);
        });
        while {
            let x = r.grstctl().read();
            x.rxfflsh() || x.txfflsh()
        } {}

        // The driver never uses SOF notifications (frame scheduling reads
        // HFNUM directly); without this the core would raise a 1 kHz interrupt.
        r.gintmsk().modify(|w| w.set_sofm(false));
    }
}

impl<'d, T: Instance> UsbHostController<'d> for UsbHost<'d, T> {
    type Allocator = <OtgHost<'d> as UsbHostController<'d>>::Allocator;

    fn allocator(&self) -> Self::Allocator {
        self.inner.allocator()
    }

    async fn wait_for_device_event(&mut self) -> DeviceEvent {
        let ev = self.inner.wait_for_device_event().await;
        // The stack initializes the core lazily inside the first call; its
        // core-id dispatch reads GUID@0x3C (= 0x2000 on AT32) and takes the
        // GCCFG v2 path, whose VBDEN write lands on AT32's VBUSIG (both are
        // bit21) and clears it. Re-apply the AT32 patch after every call so
        // the PHY power-up and the VBUS override are always in effect.
        T::host_preinit();
        // The FIFO retune can only happen after the lazy init.
        self.tune_fifos();
        ev
    }

    async fn bus_reset(&mut self) {
        self.inner.bus_reset().await;
    }
}

/// Standard device enumeration on top of the pipe API: GET_DESCRIPTOR(Device)
/// (8 bytes for bMaxPacketSize0, then the full 18 at the learned size),
/// SET_ADDRESS, optional GET_DESCRIPTOR(Config), SET_CONFIGURATION.
///
/// If `config_buf` is `Some`, the complete configuration descriptor (index 0,
/// up to the buffer length) is written into it and the byte count is returned
/// as `info.config_len`.
///
/// After this returns the device answers at `info.address`; bulk/interrupt
/// pipes for its endpoints can be allocated from the same allocator.
pub async fn enumerate<'d, A: UsbHostAllocator<'d>>(
    alloc: &A,
    speed: Speed,
    config_buf: Option<&mut [u8]>,
) -> Result<DeviceInfo, HostError> {
    // First pass at the minimum EP0 size (8 B): this cannot exceed the device's
    // bMaxPacketSize0, which is not known yet (official usbh flow: GET 8 bytes,
    // re-open the control channels at the learned size, then read the full
    // descriptor).
    let mps0 = {
        let mut ctl = alloc_control_pipe(alloc, 0, 8)?;
        let mut buf = [0u8; 8];
        let req = setup_req(0x80, 0x06, 0x0100, 0, 8);
        let n = ctl.control_in(&req, &mut buf).await?;
        if n < 8 {
            return Err(HostError::InvalidDescriptor);
        }
        drop(ctl);
        buf[7].clamp(8, 64)
    };

    let mut ctl = alloc_control_pipe(alloc, 0, mps0 as u16)?;

    // Full device descriptor (18 bytes).
    let mut full = [0u8; 18];
    {
        let req = setup_req(0x80, 0x06, 0x0100, 0, 18);
        let n = ctl.control_in(&req, &mut full).await?;
        if n < 18 {
            return Err(HostError::InvalidDescriptor);
        }
    }

    let info = DeviceInfo {
        speed,
        address: 0,
        bcd_usb: u16::from_le_bytes([full[2], full[3]]),
        device_class: full[4],
        device_subclass: full[5],
        device_protocol: full[6],
        vid: u16::from_le_bytes([full[8], full[9]]),
        pid: u16::from_le_bytes([full[10], full[11]]),
        max_packet_size0: mps0,
        num_configs: full[17],
        config_len: 0,
    };

    // SET_ADDRESS: the new address takes effect after the status phase, so the
    // request still goes out on address 0.
    let addr: u8 = 1;
    {
        let req = setup_req(0x00, 0x05, addr as u16, 0, 0);
        ctl.control_out(&req, &[]).await?;
    }
    // Re-open the pipe on the new address (the pipe bakes the address in).
    drop(ctl);
    let mut ctl = alloc_control_pipe(alloc, addr, mps0 as u16)?;

    // Configuration descriptor: header first, then the full length.
    let mut cfg = [0u8; 9];
    {
        let req = setup_req(0x80, 0x06, 0x0200, 0, 9);
        let n = ctl.control_in(&req, &mut cfg).await?;
        if n < 9 {
            return Err(HostError::InvalidDescriptor);
        }
    }
    let total = u16::from_le_bytes([cfg[2], cfg[3]]) as usize;

    let mut config_len = 0usize;
    if let Some(buf) = config_buf {
        let len = total.min(buf.len());
        let req = setup_req(0x80, 0x06, 0x0200, 0, len as u16);
        let n = ctl.control_in(&req, &mut buf[..len]).await?;
        config_len = n;
    }

    // SET_CONFIGURATION (index 0 value) when the device has at least one.
    if info.num_configs > 0 {
        let req = setup_req(0x00, 0x09, cfg[5] as u16, 0, 0);
        ctl.control_out(&req, &[]).await?;
    }

    Ok(DeviceInfo {
        address: addr,
        config_len,
        ..info
    })
}

/// Fetch a string descriptor by index and convert it from UTF-16LE to ASCII
/// (non-ASCII characters are dropped). Returns the converted string, which
/// borrows `buf`.
pub async fn get_string<'d, 'buf, A: UsbHostAllocator<'d>>(
    alloc: &A,
    addr: u8,
    max_packet_size0: u8,
    index: u8,
    buf: &'buf mut [u8],
) -> Result<&'buf str, HostError> {
    if index == 0 || buf.len() < 2 {
        return Err(HostError::Other("bad string request"));
    }
    let mut ctl = alloc_control_pipe(alloc, addr, max_packet_size0 as u16)?;
    let req = setup_req(0x80, 0x06, 0x0300 | index as u16, 0x0409, buf.len() as u16);
    let n = ctl.control_in(&req, buf).await?;
    if n < 2 {
        return Err(HostError::InvalidDescriptor);
    }
    // UTF-16LE payload starts at byte 2: bLength = 2 + 2 * chars.
    let mut out = 0usize;
    let mut i = 2usize;
    while i + 1 < n && out < buf.len() {
        let ch = u16::from_le_bytes([buf[i], buf[i + 1]]);
        if (0x20..0x7F).contains(&ch) {
            buf[out] = ch as u8;
            out += 1;
        }
        i += 2;
    }
    match core::str::from_utf8(&buf[..out]) {
        Ok(s) => Ok(s),
        Err(_) => Err(HostError::InvalidDescriptor),
    }
}

/// Summary of the device descriptor after [`enumerate`].
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DeviceInfo {
    /// Bus speed reported by the root port.
    pub speed: Speed,
    /// Device address assigned during enumeration (1 after a fresh connect).
    pub address: u8,
    /// bcdUSB (2.00 etc).
    pub bcd_usb: u16,
    pub device_class: u8,
    pub device_subclass: u8,
    pub device_protocol: u8,
    /// idVendor.
    pub vid: u16,
    /// idProduct.
    pub pid: u16,
    /// bMaxPacketSize0.
    pub max_packet_size0: u8,
    /// bNumConfigurations.
    pub num_configs: u8,
    /// Bytes written into the caller's config buffer, if one was passed.
    pub config_len: usize,
}

/// Allocate a control pipe (in+out) for the given device address.
fn alloc_control_pipe<'d, A: UsbHostAllocator<'d>>(
    alloc: &A,
    addr: u8,
    max_packet_size: u16,
) -> Result<A::Pipe<pipe::Control, pipe::InOut>, HostError> {
    alloc.alloc_pipe::<pipe::Control, pipe::InOut>(
        addr,
        &EndpointInfo {
            // The direction field is meaningless for control pipes.
            addr: EndpointAddress::from_parts(0, Direction::In),
            ep_type: EndpointType::Control,
            max_packet_size,
            interval_ms: 0,
        },
        None,
    )
}

/// Build a standard setup packet (little-endian wValue/wIndex/wLength).
pub fn setup_req(
    bm_request_type: u8,
    b_request: u8,
    w_value: u16,
    w_index: u16,
    w_length: u16,
) -> [u8; 8] {
    [
        bm_request_type,
        b_request,
        w_value as u8,
        (w_value >> 8) as u8,
        w_index as u8,
        (w_index >> 8) as u8,
        w_length as u8,
        (w_length >> 8) as u8,
    ]
}
