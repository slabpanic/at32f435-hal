//! USB OTG driver: OTGFS1 (PA11/PA12) / OTGFS2 (PB14/PB15), Synopsys OTG FS
//! core, device and host mode.
//!
//! The protocol stack comes from [`embassy_usb_synopsys_otg`] (the same one as
//! embassy-stm32, with raw-pointer register definitions, no PAC dependency);
//! this module only adapts the instances: register pointers, FIFO depth
//! (320 words), the USB 48 MHz clock configuration, NVIC binding, and
//! AT32-specific register differences.
//!
//! - Device mode ([`Driver`], [`InterruptHandler`]): the stack's device half,
//!   as consumed by `embassy-usb` class drivers (CDC-ACM, HID, MSC, ...).
//! - Host mode ([`host::UsbHost`], [`host::HostInterruptHandler`]): the
//!   stack's host half, as consumed by the `embassy_usb_driver::host` traits;
//!   plus an [`host::enumerate`] helper for the standard device enumeration.
//!
//! Verified differences between AT32F435 and STM32F4 OTG_FS (RM 21.3 / PAC
//! cross-check):
//! 1. **The USB control clock must be 48 MHz +/- 0.25%** (independent of the AHB
//!    clock): CRM_MISC1.HICK_TO_USB selects HICK 48M pass-through, or divides
//!    the PLL down via CRM_MISC2.USBDIV (half-step divider encoding, 0xB=/6).
//!    `Bus` configures this automatically on its first poll based on the
//!    current clocks; AHB must be > 30 MHz (RM 21.3.1).
//! 2. **GCCFG.VBUSIG (bit21)** replaces STM32's NOVBUSSENS: it must be set to 1
//!    when VBUS sensing is not wired, otherwise the core considers VBUS
//!    invalid and does not connect to the bus. This module writes it
//!    automatically after `config_v1()` (same as Artery's official
//!    `usbd_init` with vbusig=TRUE).
//! 3. **USBTRDTIM** is calibrated for the 16-bit UTMI+ MAC (RM 21.6.3.4: 0x5;
//!    same value as Artery's official `usbd_enumdone_handler`), which differs
//!    from embassy-stm32's STM32 table (0x6).
//! 4. GUSBCFG has no PHYSEL (the built-in FS PHY is selected implicitly); the
//!    crate's physel write lands on a reserved bit and is a no-op. The crate's
//!    novbussens/vbusa/vbusb writes also land on AT32 reserved bits - no
//!    handling needed.
//! 5. Pin AF numbers (RM table 21-1): OTGFS1_D+ / D- = PA11/PA12 mux 0xA;
//!    OTGFS2_D+ / D- = PB14/PB15 mux 0xC (not the same origin as STM32 AF10, and
//!    the two instances use different numbers).
//!
//! Usage (same as the evb CDC-ACM example):
//! ```ignore
//! use at32f435_hal::{bind_interrupts, usb::{Config, Driver, InterruptHandler}};
//!
//! bind_interrupts!(struct Irqs {
//!     OTGFS1 => InterruptHandler<hal::peripherals::USB_OTG1>;
//! });
//! static EP_OUT: StaticCell<[u8; 512]> = StaticCell::new();
//! let driver = Driver::new(
//!     p.USB_OTG1, p.PA12, p.PA11, Irqs,
//!     EP_OUT.init([0u8; 512]), Config::default(),
//! );
//! // -> embassy_usb::Builder + class (CdcAcmClass etc.)
//! ```
//!
//! `ep_out_buffer` is a staging area for received packets and must be >= the sum
//! of the max packet sizes of all OUT endpoints (256-512 bytes is enough for
//! full-speed CDC); endpoint allocation errors out if it is too small.

pub mod host;

use core::marker::PhantomData;

use embassy_usb_driver::{EndpointAddress, EndpointAllocError, EndpointType, Event, Unsupported};
use embassy_usb_synopsys_otg::host::{HostState, HostStateStorage};
use embassy_usb_synopsys_otg::otg_v1::vals::Dspd;
use embassy_usb_synopsys_otg::otg_v1::Otg;
pub use embassy_usb_synopsys_otg::Config;
use embassy_usb_synopsys_otg::{
    on_interrupt as on_interrupt_impl, Bus as OtgBus, ControlPipe, Driver as OtgDriver, Endpoint,
    In, OtgInstance, Out, PhyType, State, StateStorage,
};

use crate::gpio::{AfType, OutputType, Pull, Speed};
use crate::interrupt::typelevel::{Binding, Handler, Interrupt as _};
use crate::peripheral::RccPeripheral;
use crate::{interrupt, pac, Peri};

/// Extra RX FIFO reserve (embassy-stm32's rule-of-thumb value: RXFLVL status
/// word + back-to-back packet buffering).
const RX_FIFO_EXTRA_SIZE_WORDS: u16 = 30;

/// Interrupt handler: bind it to the `OTGFS1` (IRQ 67) / `OTGFS2` (IRQ 77)
/// vectors via `bind_interrupts!`.
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        on_interrupt_impl(T::regs(), &T::state());
    }
}

/// USB device driver (one per instance; the stack itself lives in
/// embassy-usb-synopsys-otg).
pub struct Driver<'d, T: Instance> {
    _phantom: PhantomData<&'d mut T>,
    inner: OtgDriver<'d>,
}

impl<'d, T: Instance> Driver<'d, T> {
    /// Build the FS device driver (built-in full-speed PHY; the D+ / D- pins
    /// get their mux configured automatically via the pin trait table).
    pub fn new(
        _peri: Peri<'d, T>,
        dp: Peri<'d, impl DpPin<T>>,
        dm: Peri<'d, impl DmPin<T>>,
        _irq: impl Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        ep_out_buffer: &'d mut [u8],
        config: Config,
    ) -> Self {
        dp.set_as_af(
            dp.af_num(),
            AfType::output(OutputType::PushPull, Speed::High.into(), Pull::None),
        );
        dm.set_as_af(
            dm.af_num(),
            AfType::output(OutputType::PushPull, Speed::High.into(), Pull::None),
        );

        let instance = OtgInstance {
            regs: T::regs(),
            state: T::state(),
            fifo_depth_words: T::FIFO_DEPTH_WORDS,
            extra_rx_fifo_words: RX_FIFO_EXTRA_SIZE_WORDS,
            phy_type: PhyType::InternalFullSpeed,
            calculate_trdt_fn: calculate_trdt::<T>,
        };

        Self {
            _phantom: PhantomData,
            inner: OtgDriver::new(ep_out_buffer, instance, config),
        }
    }
}

impl<'d, T: Instance> embassy_usb_driver::Driver<'d> for Driver<'d, T> {
    type EndpointOut = Endpoint<'d, Out>;
    type EndpointIn = Endpoint<'d, In>;
    type ControlPipe = ControlPipe<'d>;
    type Bus = Bus<'d, T>;

    fn alloc_endpoint_in(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointIn, EndpointAllocError> {
        self.inner
            .alloc_endpoint_in(ep_type, ep_addr, max_packet_size, interval_ms)
    }

    fn alloc_endpoint_out(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointOut, EndpointAllocError> {
        self.inner
            .alloc_endpoint_out(ep_type, ep_addr, max_packet_size, interval_ms)
    }

    fn start(self, control_max_packet_size: u16) -> (Self::Bus, Self::ControlPipe) {
        let (bus, cp) = self.inner.start(control_max_packet_size);
        (
            Bus {
                _phantom: PhantomData,
                inner: bus,
                inited: false,
            },
            cp,
        )
    }
}

/// USB bus (held by the `Builder`): on the first poll, configures the 48M
/// clock, RCC, the core, and the AT32 patches.
pub struct Bus<'d, T: Instance> {
    _phantom: PhantomData<&'d mut T>,
    inner: OtgBus<'d>,
    inited: bool,
}

impl<'d, T: Instance> Bus<'d, T> {
    fn init(&mut self) {
        // USB control clock 48 MHz (must precede core startup; RM 21.3.1)
        configure_usb_48m();

        // NVIC: clear pending first, then enable (same order as
        // embassy-stm32's common_init)
        T::Interrupt::unpend();
        T::Interrupt::enable();

        // CRM clock enable + peripheral reset pulse (AHBEN2/AHBRST2.otgfs1 =
        // bit7; OTG2 = AHBEN1 bit29)
        T::enable_and_reset();

        let r = T::regs();

        // Wait for the AHB host to be idle (RM 21.5.1 init flow)
        while !r.grstctl().read().ahbidl() {}

        // Force device mode (GUSBCFG.FDEVMODE@bit30, same bit layout as core
        // v1 on AT32) + wait for CMOD
        self.inner.configure_as_device();

        // Core ID: AT32's GUID lives at 0x3C - the Synopsys-standard CID at
        // 0x40 is repurposed on AT32 (reads back 0x4F54400A on silicon), so
        // the PAC's guid register must be used.
        // Silicon reads 0x0000_2000 (RM 21.6.3.13 says "reset value 0x1000",
        // which does not match silicon).
        //
        // embassy-stm32 routes by CID (0x2000 -> config_v2v3); on AT32 the key
        // differing bits of that path (NOVBUSSENS/VBDEN/BVALOEN) do not exist
        // or are meaningless, while the net effect of config_v1 (PWRDOWN=1 +
        // remaining reserved bits ignored) matches Artery's official
        // usb_global_init/usbd_init exactly, so we always take config_v1
        // (VBUSIG patch below).
        let core_id = T::core_id();
        match core_id {
            0x0000_1000 | 0x0000_1100 | 0x0000_1200 | 0x0000_2000 | 0x0000_2100 | 0x0000_2300 => {
                self.inner.config_v1()
            }
            _ => panic!("unknown USB core id {:#x}", core_id),
        }

        // AT32 difference: GCCFG.VBUSIG (bit21) - must be set to 1 when VBUS
        // sensing is not wired (RM 21.6.3.12)
        T::set_vbusig();
    }

    fn disable(&mut self) {
        if !self.inited {
            return;
        }
        T::Interrupt::disable();
        T::disable();
        self.inited = false;
    }
}

impl<'d, T: Instance> embassy_usb_driver::Bus for Bus<'d, T> {
    async fn poll(&mut self) -> Event {
        if !self.inited {
            self.init();
            self.inited = true;
        }
        self.inner.poll().await
    }

    fn endpoint_set_stalled(&mut self, ep_addr: EndpointAddress, stalled: bool) {
        self.inner.endpoint_set_stalled(ep_addr, stalled)
    }

    fn endpoint_is_stalled(&mut self, ep_addr: EndpointAddress) -> bool {
        self.inner.endpoint_is_stalled(ep_addr)
    }

    fn endpoint_set_enabled(&mut self, ep_addr: EndpointAddress, enabled: bool) {
        self.inner.endpoint_set_enabled(ep_addr, enabled)
    }

    async fn enable(&mut self) {
        self.inner.enable().await
    }

    async fn disable(&mut self) {
        self.inner.disable().await;
        Bus::disable(self);
    }

    async fn remote_wakeup(&mut self) -> Result<(), Unsupported> {
        self.inner.remote_wakeup().await
    }
}

impl<'d, T: Instance> Drop for Bus<'d, T> {
    fn drop(&mut self) {
        Bus::disable(self);
    }
}

/// Configure the USB control clock to 48 MHz +/- 0.25% (RM 21.3.1).
///
/// - SCLK=PLL: USBDIV (`CRM_MISC2[15:12]`, half-step divider encoding) divides
///   the PLL output (in this HAL, PLL output = sysclk) down to 48 MHz;
/// - SCLK=HICK: HICK_TO_USB (CRM_MISC1 bit13) passes HICK 48M straight through;
/// - SCLK=HEXT driven directly: 48M cannot be derived, so error out.
///
/// Note: USBDIV is independent of the other MISC1/MISC2 fields (AUTO_STEP_EN,
/// reserved values 0xD, etc.); always use read-modify-write.
pub(crate) fn configure_usb_48m() {
    let crm = unsafe { &*pac::CRM::PTR };
    let sel = crm.cfg().read().sclksel();
    if sel.is_pll() {
        // MISC2.USBDIV encoding (RM 4.3.23): the divider factor of value v =
        // DIV_X2[v] / 2.
        // 288 MHz -> /6 -> v = 0xB.
        const DIV_X2: [u16; 13] = [3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 14];
        let pll = crate::rcc::clocks().sysclk.0;
        let code = DIV_X2
            .iter()
            .position(|&dx2| 24_000_000 * dx2 as u32 == pll)
            .map(|i| i as u8)
            .unwrap_or_else(|| {
                panic!(
                    "cannot derive USB 48 MHz from PLL {} Hz (no matching USBDIV step)",
                    pll
                )
            });
        crm.misc2().modify(|_, w| unsafe { w.usbdiv().bits(code) });
        crm.misc1().modify(|_, w| w.hick_to_usb().clear_bit());
    } else if sel.is_hick() {
        crm.misc1().modify(|_, w| w.hick_to_usb().set_bit());
    } else {
        panic!("SCLK driven directly by HEXT cannot provide the USB 48 MHz control clock; use PLL or HICK");
    }
}

/// AT32 USBTRDTIM: 16-bit UTMI+ MAC interface (RM 21.6.3.4: 0x5, based on
/// AHB >= 30 MHz; same value as Artery's official usbd_enumdone_handler).
/// embassy-stm32's STM32 table (>=32M -> 0x6) does not apply to AT32 and is not
/// used.
fn calculate_trdt<T: Instance>(_speed: Dspd) -> u8 {
    let hclk = T::frequency().0;
    if hclk >= 30_000_000 {
        0x5
    } else {
        panic!(
            "USB requires AHB > 30 MHz (RM 21.3.1); current HCLK = {} Hz",
            hclk
        );
    }
}

trait SealedInstance {
    /// Dedicated FIFO SRAM = 1280 B = 320 words (RM 21.1, same as STM32F4
    /// OTG_FS).
    const FIFO_DEPTH_WORDS: u16;

    fn regs() -> Otg;
    fn state() -> State<'static>;

    /// Host-mode state storage (16 channels, RM 21.6.2 / official library
    /// USB_HOST_CHANNEL_NUM = 16).
    fn host_state() -> HostState<'static>;

    /// AT32-specific patches applied after the core is enabled, before the
    /// host stack starts (PHY power-up, VBUSIG, TRDT, frame interval).
    fn host_preinit();
    /// Core ID (AT32 GUID@0x3C, reads 0x2000 on silicon; the crate's cid()
    /// reads 0x40, which is wrong here).
    fn core_id() -> u32;
    /// AT32 difference patch: set GCCFG.VBUSIG (module doc difference #2).
    fn set_vbusig();
}

/// USB OTGFS instance trait (peripheral singleton -> registers / state /
/// interrupt number).
#[allow(private_bounds)]
pub trait Instance:
    SealedInstance + RccPeripheral + embassy_hal_internal::PeripheralType + 'static
{
    /// This instance's interrupt (typelevel, used by `bind_interrupts!`).
    type Interrupt: interrupt::typelevel::Interrupt;
}

impl SealedInstance for crate::peripherals::USB_OTG1 {
    const FIFO_DEPTH_WORDS: u16 = 320;

    fn regs() -> Otg {
        unsafe { Otg::from_ptr(pac::USB_OTG1_GLOBAL::PTR as *mut ()) }
    }

    fn state() -> State<'static> {
        static STATE: StateStorage<8> = StateStorage::new();
        STATE.as_state()
    }

    fn host_state() -> HostState<'static> {
        static HOST_STATE: HostStateStorage<16> = HostStateStorage::new();
        HOST_STATE.as_host_state()
    }

    fn host_preinit() {
        let g = unsafe { &*pac::USB_OTG1_GLOBAL::PTR };
        // PHY power-up (official usb_global_init pwrdown=TRUE) + VBUS sensing
        // ignored (module doc difference #2).
        g.gccfg()
            .modify(|_, w| w.pwrdown().set_bit().vbusig().set_bit());
        // 16-bit UTMI+ MAC turn-around time (RM 21.6.3.4: 0x5, AHB >= 30 MHz;
        // also the reset value - this write is a belt-and-braces re-affirm).
        g.gusbcfg()
            .modify(|_, w| unsafe { w.usbtrdtim().bits(0x5) });
        // NOTE: HFIR must NOT be written here - RM 21.6.4.2 allows writing it
        // only after the port enable bit is set (earlier writes are ignored;
        // hw-verified). The host stack writes 48000 after the port comes up.
    }

    fn core_id() -> u32 {
        let g = unsafe { &*pac::USB_OTG1_GLOBAL::PTR };
        g.guid().read().bits()
    }

    fn set_vbusig() {
        let g = unsafe { &*pac::USB_OTG1_GLOBAL::PTR };
        g.gccfg().modify(|_, w| w.vbusig().set_bit());
    }
}

impl Instance for crate::peripherals::USB_OTG1 {
    type Interrupt = interrupt::typelevel::OTGFS1;
}

impl SealedInstance for crate::peripherals::USB_OTG2 {
    const FIFO_DEPTH_WORDS: u16 = 320;

    fn regs() -> Otg {
        unsafe { Otg::from_ptr(pac::USB_OTG2_GLOBAL::PTR as *mut ()) }
    }

    fn state() -> State<'static> {
        static STATE: StateStorage<8> = StateStorage::new();
        STATE.as_state()
    }

    fn host_state() -> HostState<'static> {
        static HOST_STATE: HostStateStorage<16> = HostStateStorage::new();
        HOST_STATE.as_host_state()
    }

    fn host_preinit() {
        let g = unsafe { &*pac::USB_OTG2_GLOBAL::PTR };
        // Same patch set as USB_OTG1 (see that impl for register references).
        g.gccfg()
            .modify(|_, w| w.pwrdown().set_bit().vbusig().set_bit());
        g.gusbcfg()
            .modify(|_, w| unsafe { w.usbtrdtim().bits(0x5) });
    }

    fn core_id() -> u32 {
        let g = unsafe { &*pac::USB_OTG2_GLOBAL::PTR };
        g.guid().read().bits()
    }

    fn set_vbusig() {
        let g = unsafe { &*pac::USB_OTG2_GLOBAL::PTR };
        g.gccfg().modify(|_, w| w.vbusig().set_bit());
    }
}

impl Instance for crate::peripherals::USB_OTG2 {
    type Interrupt = interrupt::typelevel::OTGFS2;
}

crate::pin_trait!(DpPin, Instance);
crate::pin_trait!(DmPin, Instance);

// AF numbers per RM table 21-1: OTGFS1_D+ / D- = mux 0xA, OTGFS2_D+ / D- = mux 0xC
// (the two instances use different numbers).
crate::pin_trait_impl!(crate::usb::DpPin, USB_OTG1, PA12, 0xA);
crate::pin_trait_impl!(crate::usb::DmPin, USB_OTG1, PA11, 0xA);
crate::pin_trait_impl!(crate::usb::DpPin, USB_OTG2, PB15, 0xC);
crate::pin_trait_impl!(crate::usb::DmPin, USB_OTG2, PB14, 0xC);
