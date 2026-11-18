//! General-timer extensions for TMR2/3/4/5 (RM 14.2): input capture, single
//! pulse (one-cycle) mode and QEI encoder counting.
//!
//! All three drivers sit on top of the same register block shape as the PWM
//! driver (`GeneralInstance`), reusing its `Channel`/pin traits.
//!
//! ```ignore
//! // input capture on TMR2 CH1 (PA0), blocking:
//! let mut cap = hal::timer::InputCapture::new(p.TMR2, Default::default());
//! cap.enable_ch1(p.PA0);
//! let tick = cap.blocking_capture(Channel::Ch1)?;
//!
//! // single pulse on TMR2 CH2 (PA1): one PWM burst per trigger or start():
//! let mut op = hal::timer::OnePulse::new(
//!     p.TMR2, Hertz::hz(1_000), 400, Channel::Ch2, p.PA1, None, // None = start on start()
//! );
//! op.start();
//! op.wait_done_blocking();
//!
//! // QEI encoder on TMR3 CH1/CH2:
//! let mut qei = hal::timer::Qei::new(p.TMR3, p.PA6, p.PA7);
//! let count = qei.count();
//! ```

use core::future::poll_fn;
use core::marker::PhantomData;
use core::task::Poll;

use embassy_hal_internal::Peri;
use embassy_sync::waitqueue::AtomicWaker;

use crate::gpio::AnyPin;
use crate::gpio::SealedPin;
use crate::interrupt;
use crate::interrupt::typelevel::{Handler, Interrupt as _};
use crate::peripheral::RccPeripheral;
use crate::time::Hertz;

use super::{Channel, GeneralInstance};

/// Capture edge (input polarity, CCTRL.CxP for input channels).
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
pub enum Edge {
    /// Rising edge (C1P/C1CP = 00).
    #[default]
    Rising,
    /// Falling edge (C1P/C1CP = 01).
    Falling,
    /// Both edges (C1P/C1CP = 11).
    Both,
}

/// Input capture filter (CxDF, 4 bits): N consecutive equal samples before the
/// edge is accepted (sampling clock = timer clock / CLKDIV).
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
pub struct Filter(pub u8);

impl Filter {
    /// No filtering (fDTS clock, N=1).
    pub const NONE: Filter = Filter(0);
}

/// Input capture divider (CxIDIV): events between captures.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
pub enum EventDivider {
    /// Capture on every valid edge.
    #[default]
    Every,
    /// Every 2nd valid edge.
    Div2,
    /// Every 4th valid edge.
    Div4,
    /// Every 8th valid edge.
    Div8,
}

impl EventDivider {
    const fn code(self) -> u8 {
        match self {
            EventDivider::Every => 0,
            EventDivider::Div2 => 1,
            EventDivider::Div4 => 2,
            EventDivider::Div8 => 3,
        }
    }
}

/// Capture configuration shared by all channels.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
pub struct CaptureConfig {
    /// Capture edge for subsequently enabled channels.
    pub edge: Edge,
    /// Digital filter.
    pub filter: Filter,
    /// Event divider.
    pub divider: EventDivider,
}

/// Capture error.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[non_exhaustive]
pub enum CaptureError {
    /// The channel was never enabled with a pin.
    ChannelNotEnabled,
    /// The timer does not support the requested configuration.
    Unsupported,
}

/// Register-level abstraction over the TMR2 (32-bit-capable) and TMR3/4/5
/// (16-bit) blocks for capture/slave-mode/one-cycle configuration.
pub(crate) trait CapRegs {
    /// Slave mode control: SMSSEL + STIS.
    fn set_slave(&self, smsel: u8, stis: u8);
    /// External clock mode B enable (ECMBEN).
    #[allow(dead_code)]
    fn set_ecmben(&self, on: bool);
    /// CTRL1 bits: single-cycle (OCMEN) + start.
    fn set_ocmen(&self, on: bool);
    fn set_tmren(&self, on: bool);
    fn counter_running(&self) -> bool;
    /// Configure a channel as capture input: `stci` selects the internal
    /// trigger input (CxC = 11) instead of the pin (CxC = 01).
    fn set_capture_channel(
        &self,
        ch: Channel,
        stci: bool,
        edge: Edge,
        filter: Filter,
        divider: EventDivider,
    );
    fn enable_capture_channel(&self, ch: Channel, on: bool);
    /// Enables the channel interrupt (IDEN.CxIEN).
    fn listen_channel(&self, ch: Channel, on: bool);
    /// Capture interrupt flag (ISTS.CxIF).
    fn capture_flag(&self, ch: Channel) -> bool;
    /// Captured value (CxDT; reading C1DT..C4DT clears the CxIF flag).
    fn capture_value(&self, ch: Channel) -> u32;
    /// Encoder mode A/B/C: SMSSEL 001/010/011.
    fn set_encoder_mode(&self, mode: u8);
    /// Period (PR) for the one-cycle pulse.
    fn set_period(&self, pr: u32);
    /// Prescaler (DIV).
    fn set_divider(&self, div: u32);
    /// Replace only the STIS field of STCTRL (keep the slave mode).
    fn set_slave_trigger_input(&self, stis: u8);
    /// Enable/disable a channel output (CCTRL.CxEN).
    fn enable_output_channel(&self, ch: Channel, on: bool);
    /// Counter value (CVAL).
    fn counter(&self) -> u32;
    /// Counter direction flag (CTRL1.OWCDIR: true = down).
    fn direction_down(&self) -> bool;
    /// Overflow flag (ISTS.OVFIF).
    fn overflow_flag(&self) -> bool;
    /// Clears the overflow flag.
    fn clear_overflow_flag(&self);
    /// Compare value (CxDT) - used as the pulse width when the channel is in
    /// output mode.
    fn set_compare(&self, ch: Channel, v: u32);
    /// Configure a channel as PWM output (mode A) with preload, like the PWM
    /// driver but exposed for the one-cycle pulse.
    fn set_pwm_output_channel(&self, ch: Channel);
    /// Overflow software event (loads DIV/PR shadows).
    fn generate_overflow(&self);
}

impl CapRegs for crate::pac::tmr2::RegisterBlock {
    #[inline]
    fn set_slave(&self, smsel: u8, stis: u8) {
        self.stctrl().modify(|_, w| unsafe {
            w.smsel().bits(smsel);
            w.stis().bits(stis);
            w
        });
    }

    #[inline]
    fn set_ecmben(&self, on: bool) {
        self.stctrl().modify(|_, w| {
            if on {
                w.ecmben().set_bit();
            } else {
                w.ecmben().clear_bit();
            }
            w
        });
    }

    #[inline]
    fn set_ocmen(&self, on: bool) {
        self.ctrl1().modify(|_, w| {
            if on {
                w.ocmen().set_bit();
            } else {
                w.ocmen().clear_bit();
            }
            w
        });
    }

    #[inline]
    fn set_tmren(&self, on: bool) {
        self.ctrl1().modify(|_, w| {
            if on {
                w.tmren().set_bit();
            } else {
                w.tmren().clear_bit();
            }
            w
        });
    }

    #[inline]
    fn counter_running(&self) -> bool {
        self.ctrl1().read().tmren().is_enabled()
    }

    #[inline]
    fn set_capture_channel(
        &self,
        ch: Channel,
        stci: bool,
        edge: Edge,
        filter: Filter,
        divider: EventDivider,
    ) {
        let code = if stci { 0b11 } else { 0b01 };
        let filt = filter.0 & 0x0F;
        let div = divider.code();
        match ch {
            Channel::Ch1 => self.cm1_input().modify(|_, w| unsafe {
                w.c1c().bits(code);
                w.c1df().bits(filt);
                w.c1idiv().bits(div);
                w
            }),
            Channel::Ch2 => self.cm1_input().modify(|_, w| unsafe {
                w.c2c().bits(code);
                w.c2df().bits(filt);
                w.c2idiv().bits(div);
                w
            }),
            Channel::Ch3 => self.cm2_input().modify(|_, w| unsafe {
                w.c3c().bits(code);
                w.c3df().bits(filt);
                w.c3idiv().bits(div);
                w
            }),
            Channel::Ch4 => self.cm2_input().modify(|_, w| unsafe {
                w.c4c().bits(code);
                w.c4df().bits(filt);
                w.c4idiv().bits(div);
                w
            }),
        };
        // Input polarity: CxP (even channels share pairs on CCTRL, so use the
        // dedicated CxP bit per channel; CxCP only matters for complementary
        // outputs) - write the rising/falling/both code into CxP.
        // CxP lives at bit n*4+1; the complementary-polarity bit CxCP at
        // n*4+3 exists in the register (RM table for TMR2-5 CCTRL) even
        // though the PAC models it only for the advanced timers - it defines
        // the second active edge for captures, so "Both edges" needs it.
        let n = ch.index() as u32;
        let mut bits = self.cctrl().read().bits();
        bits &= !(1 << (n * 4 + 1));
        bits &= !(1 << (n * 4 + 3));
        match edge {
            Edge::Rising => {}
            Edge::Falling => bits |= 1 << (n * 4 + 1),
            Edge::Both => {
                bits |= 1 << (n * 4 + 1);
                bits |= 1 << (n * 4 + 3);
            }
        }
        self.cctrl().write(|w| unsafe { w.bits(bits) });
    }

    #[inline]
    fn enable_capture_channel(&self, ch: Channel, on: bool) {
        let n = ch.index() as u8;
        self.cctrl().modify(|_, w| {
            if on {
                w.cen(n).enable();
            } else {
                w.cen(n).disable();
            }
            w
        });
    }

    #[inline]
    fn listen_channel(&self, ch: Channel, on: bool) {
        self.iden().modify(|_, w| {
            match ch {
                Channel::Ch1 => {
                    if on {
                        w.c1ien().enable()
                    } else {
                        w.c1ien().disable()
                    }
                }
                Channel::Ch2 => {
                    if on {
                        w.c2ien().enable()
                    } else {
                        w.c2ien().disable()
                    }
                }
                Channel::Ch3 => {
                    if on {
                        w.c3ien().enable()
                    } else {
                        w.c3ien().disable()
                    }
                }
                Channel::Ch4 => {
                    if on {
                        w.c4ien().enable()
                    } else {
                        w.c4ien().disable()
                    }
                }
            };
            w
        });
    }

    #[inline]
    fn capture_flag(&self, ch: Channel) -> bool {
        let sts = self.ists().read();
        match ch {
            Channel::Ch1 => sts.c1if().bit_is_set(),
            Channel::Ch2 => sts.c2if().bit_is_set(),
            Channel::Ch3 => sts.c3if().bit_is_set(),
            Channel::Ch4 => sts.c4if().bit_is_set(),
        }
    }

    #[inline]
    fn capture_value(&self, ch: Channel) -> u32 {
        // Reading CxDT clears the CxIF flag (RM 14.2.4.5).
        match ch {
            Channel::Ch1 => self.c1dt().read().bits(),
            Channel::Ch2 => self.c2dt().read().bits(),
            Channel::Ch3 => self.c3dt().read().bits(),
            Channel::Ch4 => self.c4dt().read().bits(),
        }
    }

    #[inline]
    fn set_encoder_mode(&self, mode: u8) {
        self.stctrl().modify(|_, w| unsafe { w.smsel().bits(mode) });
    }

    #[inline]
    fn set_period(&self, pr: u32) {
        self.pr().write(|w| w.set(pr));
    }

    #[inline]
    fn set_divider(&self, div: u32) {
        self.div().write(|w| w.set(div as u16));
    }

    #[inline]
    fn set_slave_trigger_input(&self, stis: u8) {
        self.stctrl().modify(|_, w| unsafe { w.stis().bits(stis) });
    }

    #[inline]
    fn enable_output_channel(&self, ch: Channel, on: bool) {
        self.enable_capture_channel(ch, on);
    }

    #[inline]
    fn counter(&self) -> u32 {
        self.cval().read().bits()
    }

    #[inline]
    fn direction_down(&self) -> bool {
        self.ctrl1().read().owcdir().bit_is_set()
    }

    #[inline]
    fn overflow_flag(&self) -> bool {
        self.ists().read().ovfif().bit_is_set()
    }

    #[inline]
    fn clear_overflow_flag(&self) {
        self.ists().write(|w| w.ovfif().clear());
    }

    #[inline]
    fn set_compare(&self, ch: Channel, v: u32) {
        match ch {
            Channel::Ch1 => self.c1dt().write(|w| w.set(v)),
            Channel::Ch2 => self.c2dt().write(|w| w.set(v)),
            Channel::Ch3 => self.c3dt().write(|w| w.set(v)),
            Channel::Ch4 => self.c4dt().write(|w| w.set(v)),
        };
    }

    #[inline]
    fn set_pwm_output_channel(&self, ch: Channel) {
        // PWM mode A (CxOCTRL = 110) + output buffer (CxOBEN); OSEN stays 0
        // (the register-level pitfall documented in docs/pitfalls.md).
        match ch {
            Channel::Ch1 => self.cm1_output().modify(|_, w| {
                w.c1c().output();
                unsafe { w.c1octrl().bits(0b110) };
                w.c1oben().enable();
                w
            }),
            Channel::Ch2 => self.cm1_output().modify(|_, w| {
                w.c2c().output();
                unsafe { w.c2octrl().bits(0b110) };
                w.c2oben().enable();
                w
            }),
            Channel::Ch3 => self.cm2_output().modify(|_, w| {
                w.c3c().output();
                unsafe { w.c3octrl().bits(0b110) };
                w.c3oben().enable();
                w
            }),
            Channel::Ch4 => self.cm2_output().modify(|_, w| {
                w.c4c().output();
                unsafe { w.c4octrl().bits(0b110) };
                w.c4oben().enable();
                w
            }),
        };
    }

    #[inline]
    fn generate_overflow(&self) {
        self.swevt().write(|w| w.ovfswtr().overflow());
    }
}

impl CapRegs for crate::pac::tmr3::RegisterBlock {
    #[inline]
    fn set_slave(&self, smsel: u8, stis: u8) {
        self.stctrl().modify(|_, w| unsafe {
            w.smsel().bits(smsel);
            w.stis().bits(stis);
            w
        });
    }

    #[inline]
    fn set_ecmben(&self, on: bool) {
        self.stctrl().modify(|_, w| {
            if on {
                w.ecmben().set_bit();
            } else {
                w.ecmben().clear_bit();
            }
            w
        });
    }

    #[inline]
    fn set_ocmen(&self, on: bool) {
        self.ctrl1().modify(|_, w| {
            if on {
                w.ocmen().set_bit();
            } else {
                w.ocmen().clear_bit();
            }
            w
        });
    }

    #[inline]
    fn set_tmren(&self, on: bool) {
        self.ctrl1().modify(|_, w| {
            if on {
                w.tmren().set_bit();
            } else {
                w.tmren().clear_bit();
            }
            w
        });
    }

    #[inline]
    fn counter_running(&self) -> bool {
        self.ctrl1().read().tmren().is_enabled()
    }

    #[inline]
    fn set_capture_channel(
        &self,
        ch: Channel,
        stci: bool,
        edge: Edge,
        filter: Filter,
        divider: EventDivider,
    ) {
        let code = if stci { 0b11 } else { 0b01 };
        let filt = filter.0 & 0x0F;
        let div = divider.code();
        match ch {
            Channel::Ch1 => self.cm1_input().modify(|_, w| unsafe {
                w.c1c().bits(code);
                w.c1df().bits(filt);
                w.c1idiv().bits(div);
                w
            }),
            Channel::Ch2 => self.cm1_input().modify(|_, w| unsafe {
                w.c2c().bits(code);
                w.c2df().bits(filt);
                w.c2idiv().bits(div);
                w
            }),
            Channel::Ch3 => self.cm2_input().modify(|_, w| unsafe {
                w.c3c().bits(code);
                w.c3df().bits(filt);
                w.c3idiv().bits(div);
                w
            }),
            Channel::Ch4 => self.cm2_input().modify(|_, w| unsafe {
                w.c4c().bits(code);
                w.c4df().bits(filt);
                w.c4idiv().bits(div);
                w
            }),
        };
        // Same raw-bit handling as the TMR2 block (CxP at n*4+1, CxCP at
        // n*4+3 for the "both edges" case; see the TMR2 impl comment).
        let n = ch.index() as u32;
        let mut bits = self.cctrl().read().bits();
        bits &= !(1 << (n * 4 + 1));
        bits &= !(1 << (n * 4 + 3));
        match edge {
            Edge::Rising => {}
            Edge::Falling => bits |= 1 << (n * 4 + 1),
            Edge::Both => {
                bits |= 1 << (n * 4 + 1);
                bits |= 1 << (n * 4 + 3);
            }
        }
        self.cctrl().write(|w| unsafe { w.bits(bits) });
    }

    #[inline]
    fn enable_capture_channel(&self, ch: Channel, on: bool) {
        let n = ch.index() as u8;
        self.cctrl().modify(|_, w| {
            if on {
                w.cen(n).enable();
            } else {
                w.cen(n).disable();
            }
            w
        });
    }

    #[inline]
    fn listen_channel(&self, ch: Channel, on: bool) {
        self.iden().modify(|_, w| {
            match ch {
                Channel::Ch1 => {
                    if on {
                        w.c1ien().enable()
                    } else {
                        w.c1ien().disable()
                    }
                }
                Channel::Ch2 => {
                    if on {
                        w.c2ien().enable()
                    } else {
                        w.c2ien().disable()
                    }
                }
                Channel::Ch3 => {
                    if on {
                        w.c3ien().enable()
                    } else {
                        w.c3ien().disable()
                    }
                }
                Channel::Ch4 => {
                    if on {
                        w.c4ien().enable()
                    } else {
                        w.c4ien().disable()
                    }
                }
            };
            w
        });
    }

    #[inline]
    fn capture_flag(&self, ch: Channel) -> bool {
        let sts = self.ists().read();
        match ch {
            Channel::Ch1 => sts.c1if().bit_is_set(),
            Channel::Ch2 => sts.c2if().bit_is_set(),
            Channel::Ch3 => sts.c3if().bit_is_set(),
            Channel::Ch4 => sts.c4if().bit_is_set(),
        }
    }

    #[inline]
    fn capture_value(&self, ch: Channel) -> u32 {
        match ch {
            Channel::Ch1 => self.c1dt().read().bits() as u32,
            Channel::Ch2 => self.c2dt().read().bits() as u32,
            Channel::Ch3 => self.c3dt().read().bits() as u32,
            Channel::Ch4 => self.c4dt().read().bits() as u32,
        }
    }

    #[inline]
    fn set_encoder_mode(&self, mode: u8) {
        self.stctrl().modify(|_, w| unsafe { w.smsel().bits(mode) });
    }

    #[inline]
    fn set_period(&self, pr: u32) {
        self.pr().write(|w| w.set(pr as u16));
    }

    #[inline]
    fn set_divider(&self, div: u32) {
        self.div().write(|w| w.set(div as u16));
    }

    #[inline]
    fn set_slave_trigger_input(&self, stis: u8) {
        self.stctrl().modify(|_, w| unsafe { w.stis().bits(stis) });
    }

    #[inline]
    fn enable_output_channel(&self, ch: Channel, on: bool) {
        self.enable_capture_channel(ch, on);
    }

    #[inline]
    fn counter(&self) -> u32 {
        self.cval().read().bits() as u32
    }

    #[inline]
    fn direction_down(&self) -> bool {
        self.ctrl1().read().owcdir().bit_is_set()
    }

    #[inline]
    fn overflow_flag(&self) -> bool {
        self.ists().read().ovfif().bit_is_set()
    }

    #[inline]
    fn clear_overflow_flag(&self) {
        self.ists().write(|w| w.ovfif().clear());
    }

    #[inline]
    fn set_compare(&self, ch: Channel, v: u32) {
        match ch {
            Channel::Ch1 => self.c1dt().write(|w| w.set(v as u16)),
            Channel::Ch2 => self.c2dt().write(|w| w.set(v as u16)),
            Channel::Ch3 => self.c3dt().write(|w| w.set(v as u16)),
            Channel::Ch4 => self.c4dt().write(|w| w.set(v as u16)),
        };
    }

    #[inline]
    fn set_pwm_output_channel(&self, ch: Channel) {
        match ch {
            Channel::Ch1 => self.cm1_output().modify(|_, w| {
                w.c1c().output();
                unsafe { w.c1octrl().bits(0b110) };
                w.c1oben().enable();
                w
            }),
            Channel::Ch2 => self.cm1_output().modify(|_, w| {
                w.c2c().output();
                unsafe { w.c2octrl().bits(0b110) };
                w.c2oben().enable();
                w
            }),
            Channel::Ch3 => self.cm2_output().modify(|_, w| {
                w.c3c().output();
                unsafe { w.c3octrl().bits(0b110) };
                w.c3oben().enable();
                w
            }),
            Channel::Ch4 => self.cm2_output().modify(|_, w| {
                w.c4c().output();
                unsafe { w.c4octrl().bits(0b110) };
                w.c4oben().enable();
                w
            }),
        };
    }

    #[inline]
    fn generate_overflow(&self) {
        self.swevt().write(|w| w.ovfswtr().overflow());
    }
}

pub(crate) struct GeneralState {
    pub(crate) waker: AtomicWaker,
}

impl GeneralState {
    pub(crate) const fn new() -> Self {
        Self {
            waker: AtomicWaker::new(),
        }
    }
}

trait SealedCapture {
    fn cap_regs() -> &'static dyn CapRegs;
    fn state() -> &'static GeneralState;
}

/// General timer instance usable for capture / one-pulse / QEI (TMR2-5).
#[allow(private_bounds)]
pub trait CaptureInstance:
    SealedCapture + GeneralInstance + RccPeripheral + embassy_hal_internal::PeripheralType + 'static
{
    /// The timer's interrupt (typelevel).
    type Interrupt: interrupt::typelevel::Interrupt;
}

macro_rules! impl_capture_instance {
    ($inst:ident, $block:ident, $irq:ident) => {
        impl SealedCapture for crate::peripherals::$inst {
            #[inline]
            fn cap_regs() -> &'static dyn CapRegs {
                unsafe { &*(crate::pac::$inst::PTR as *const crate::pac::$block::RegisterBlock) }
            }

            #[inline]
            fn state() -> &'static GeneralState {
                static STATE: GeneralState = GeneralState::new();
                &STATE
            }
        }

        impl CaptureInstance for crate::peripherals::$inst {
            type Interrupt = interrupt::typelevel::$irq;
        }
    };
}

#[cfg(not(all(feature = "embassy", feature = "time-driver-tim2")))]
impl_capture_instance!(TMR2, tmr2, TMR2);
impl_capture_instance!(TMR3, tmr3, TMR3);
impl_capture_instance!(TMR4, tmr3, TMR4);
impl_capture_instance!(TMR5, tmr2, TMR5);

/// General timer update/capture interrupt handler: bind it with
/// `bind_interrupts!(struct Irqs { TMR2 => hal::timer::general::InterruptHandler<hal::peripherals::TMR2>; });`
///
/// Wakes the waiting futures without clearing flags (the consumers clear them
/// by reading the CxDT register); the listen bit is turned off here so the
/// IRQ does not re-fire before the consumer runs.
pub struct InterruptHandler<T: CaptureInstance> {
    _phantom: PhantomData<T>,
}

impl<T: CaptureInstance> Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let regs = T::cap_regs();
        let any = regs.capture_flag(Channel::Ch1)
            || regs.capture_flag(Channel::Ch2)
            || regs.capture_flag(Channel::Ch3)
            || regs.capture_flag(Channel::Ch4);
        if any {
            // Stop the IRQ from re-firing; the waiting future re-arms.
            regs.listen_channel(Channel::Ch1, false);
            regs.listen_channel(Channel::Ch2, false);
            regs.listen_channel(Channel::Ch3, false);
            regs.listen_channel(Channel::Ch4, false);
            T::state().waker.wake();
        }
    }
}

// ---------------------------------------------------------------------------
// InputCapture
// ---------------------------------------------------------------------------

/// Input capture driver over one timer and up to four channels.
pub struct InputCapture<'d, T: CaptureInstance> {
    _peri: Peri<'d, T>,
    _pins: [Option<Peri<'d, AnyPin>>; 4],
    cfg: CaptureConfig,
}

impl<'d, T: CaptureInstance> InputCapture<'d, T> {
    /// Configures the timer in capture mode (counter running on the internal
    /// clock, channels disabled until `enable_chN` is called).
    pub fn new(peri: Peri<'d, T>, cfg: CaptureConfig) -> Self {
        T::enable_and_reset();
        // Timer clocked by CK_INT: slave mode off, counter up, running.
        T::cap_regs().set_slave(0b000, 0b000);
        T::cap_regs().set_tmren(true);
        Self {
            _peri: peri,
            _pins: [None, None, None, None],
            cfg,
        }
    }

    /// Enables channel 1 capture from its pin.
    pub fn enable_ch1(&mut self, pin: Peri<'d, impl super::Channel1Pin<T>>) -> &mut Self {
        self.enable_pin_erased(Channel::Ch1, Peri::into(pin))
    }

    /// Enables channel 2 capture from its pin.
    pub fn enable_ch2(&mut self, pin: Peri<'d, impl super::Channel2Pin<T>>) -> &mut Self {
        self.enable_pin_erased(Channel::Ch2, Peri::into(pin))
    }

    /// Enables channel 3 capture from its pin.
    pub fn enable_ch3(&mut self, pin: Peri<'d, impl super::Channel3Pin<T>>) -> &mut Self {
        self.enable_pin_erased(Channel::Ch3, Peri::into(pin))
    }

    /// Enables channel 4 capture from its pin.
    pub fn enable_ch4(&mut self, pin: Peri<'d, impl super::Channel4Pin<T>>) -> &mut Self {
        self.enable_pin_erased(Channel::Ch4, Peri::into(pin))
    }

    fn enable_pin_erased(&mut self, ch: Channel, pin: Peri<'d, AnyPin>) -> &mut Self {
        pin.set_as_input(crate::gpio::Pull::None);
        self._pins[ch.index()] = Some(pin);
        let regs = T::cap_regs();
        regs.set_capture_channel(ch, false, self.cfg.edge, self.cfg.filter, self.cfg.divider);
        regs.enable_capture_channel(ch, true);
        self
    }

    /// Captures the internal trigger input (STCI) on `ch` instead of the
    /// channel pin: with a master timer routing TRGOUT to this timer's
    /// internal trigger, every master overflow edge captures the counter.
    pub fn capture_internal_trigger(&mut self, ch: Channel) -> &mut Self {
        let regs = T::cap_regs();
        regs.set_capture_channel(ch, true, self.cfg.edge, self.cfg.filter, self.cfg.divider);
        regs.enable_capture_channel(ch, true);
        self
    }

    /// Changes the capture edge for `ch` (also valid while running).
    pub fn set_edge(&mut self, ch: Channel, edge: Edge) {
        T::cap_regs().set_capture_channel(ch, false, edge, self.cfg.filter, self.cfg.divider);
    }

    /// Selects the internal trigger input (STIS) for slave-mode operations:
    /// 0..=3 = ITR0..ITR3 (master timer TRGOUT interconnect), 4 = C1 raw
    /// input, 5/6 = filtered inputs, 7 = external pin.
    pub fn set_trigger_input(&mut self, stis: u8) {
        // Keep the slave mode field, replace only the trigger selection.
        T::cap_regs().set_slave_trigger_input(stis);
    }

    /// Sets the counter prescaler (DIV register; the counter ticks at
    /// fCK / (div + 1)). Useful to slow the counter down so captured deltas
    /// stay meaningful.
    pub fn set_clock_divider(&mut self, div: u32) {
        T::cap_regs().set_divider(div);
    }

    /// Blocking capture: waits for the next edge on `ch` and returns the
    /// captured counter value (busy-poll, no IRQ needed).
    pub fn blocking_capture(&mut self, ch: Channel) -> Result<u32, CaptureError> {
        if self._pins[ch.index()].is_none() {
            // STCI captures have no pin.
            if !T::cap_regs().capture_flag(ch) && !self.stci_armed(ch) {
                return Err(CaptureError::ChannelNotEnabled);
            }
        }
        let regs = T::cap_regs();
        while !regs.capture_flag(ch) {}
        Ok(regs.capture_value(ch)) // reading CxDT clears the flag
    }

    fn stci_armed(&self, ch: Channel) -> bool {
        // Internal-trigger captures are armed by capture_internal_trigger();
        // the CCTRL enable bit reflects that. Cheap readback check.
        T::cap_regs().capture_flag(ch)
    }

    /// Async capture: waits for the next edge on `ch` (requires the timer IRQ
    /// bound with [`InterruptHandler<T>`]).
    pub async fn capture(&mut self, ch: Channel) -> u32 {
        let regs = T::cap_regs();
        regs.listen_channel(ch, true);
        T::Interrupt::unpend();
        T::Interrupt::enable();
        poll_fn(|cx| {
            T::state().waker.register(cx.waker());
            if regs.capture_flag(ch) {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await;
        regs.capture_value(ch) // reading clears the flag
    }

    /// Captured value without waiting (the flag is left pending).
    pub fn peek(&self, ch: Channel) -> Option<u32> {
        let regs = T::cap_regs();
        if regs.capture_flag(ch) {
            Some(regs.capture_value(ch))
        } else {
            None
        }
    }

    /// Counter value (useful together with `capture_internal_trigger` to
    /// measure intervals between trigger edges).
    pub fn now(&self) -> u32 {
        T::cap_regs().counter()
    }
}

// ---------------------------------------------------------------------------
// OnePulse (single-cycle mode)
// ---------------------------------------------------------------------------

/// One pulse generator (RM 14.2.3.6 "single cycle mode"): the channel outputs
/// one PWM burst of `period` ticks with `compare` duty; the counter stops
/// itself after the burst (TMREN auto-clears).
pub struct OnePulse<'d, T: CaptureInstance> {
    _peri: Peri<'d, T>,
    _pin: Peri<'d, AnyPin>,
    ch: Channel,
}

impl<'d, T: CaptureInstance> OnePulse<'d, T> {
    /// Configures a one-cycle PWM burst on channel 1.
    ///
    /// `trigger_stis`: `Some(n)` starts the pulse from internal trigger input
    /// `n` (slave mode "trigger": SMSEL=110); `None` starts it from `start()`.
    pub fn new_ch1(
        peri: Peri<'d, T>,
        pin: Peri<'d, impl super::Channel1Pin<T>>,
        tick_freq: Hertz,
        period_ticks: u16,
        compare_ticks: u16,
        trigger_stis: Option<u8>,
    ) -> Self {
        let af = pin.af_num();
        Self::new_impl(
            peri,
            Channel::Ch1,
            Peri::into(pin),
            af,
            tick_freq,
            period_ticks,
            compare_ticks,
            trigger_stis,
        )
    }

    /// Configures a one-cycle PWM burst on channel 2.
    pub fn new_ch2(
        peri: Peri<'d, T>,
        pin: Peri<'d, impl super::Channel2Pin<T>>,
        tick_freq: Hertz,
        period_ticks: u16,
        compare_ticks: u16,
        trigger_stis: Option<u8>,
    ) -> Self {
        let af = pin.af_num();
        Self::new_impl(
            peri,
            Channel::Ch2,
            Peri::into(pin),
            af,
            tick_freq,
            period_ticks,
            compare_ticks,
            trigger_stis,
        )
    }

    /// Configures a one-cycle PWM burst on channel 3.
    pub fn new_ch3(
        peri: Peri<'d, T>,
        pin: Peri<'d, impl super::Channel3Pin<T>>,
        tick_freq: Hertz,
        period_ticks: u16,
        compare_ticks: u16,
        trigger_stis: Option<u8>,
    ) -> Self {
        let af = pin.af_num();
        Self::new_impl(
            peri,
            Channel::Ch3,
            Peri::into(pin),
            af,
            tick_freq,
            period_ticks,
            compare_ticks,
            trigger_stis,
        )
    }

    /// Configures a one-cycle PWM burst on channel 4.
    pub fn new_ch4(
        peri: Peri<'d, T>,
        pin: Peri<'d, impl super::Channel4Pin<T>>,
        tick_freq: Hertz,
        period_ticks: u16,
        compare_ticks: u16,
        trigger_stis: Option<u8>,
    ) -> Self {
        let af = pin.af_num();
        Self::new_impl(
            peri,
            Channel::Ch4,
            Peri::into(pin),
            af,
            tick_freq,
            period_ticks,
            compare_ticks,
            trigger_stis,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn new_impl(
        peri: Peri<'d, T>,
        ch: Channel,
        pin: Peri<'d, AnyPin>,
        af: u8,
        tick_freq: Hertz,
        period_ticks: u16,
        compare_ticks: u16,
        trigger_stis: Option<u8>,
    ) -> Self {
        assert!(compare_ticks <= period_ticks, "compare must be <= period");
        assert!(period_ticks >= 2, "period must be >= 2 ticks");
        T::enable_and_reset();
        let regs = T::cap_regs();
        // Tick frequency: DIV = fCK/tick - 1.
        let clk = T::frequency().0;
        let div1 = (clk / tick_freq.0.max(1)).clamp(1, 65536);
        regs.set_divider(div1 - 1);
        regs.set_period(period_ticks as u32 - 1);
        regs.set_compare(ch, compare_ticks as u32);
        regs.set_pwm_output_channel(ch);
        regs.enable_output_channel(ch, true);
        regs.set_ocmen(true);
        match trigger_stis {
            Some(n) => {
                // Trigger mode: the pulse starts on the TRGIN rising edge.
                regs.set_slave(0b110, n);
            }
            None => {
                regs.set_slave(0b000, 0b000);
            }
        }
        // Generate an overflow once so the preload (C1DT) transfers to the
        // shadow register before the pulse.
        regs.generate_overflow();
        pin.set_as_af(
            af,
            crate::gpio::AfType::output(
                crate::gpio::OutputType::PushPull,
                crate::gpio::Speed::High.into(),
                crate::gpio::Pull::None,
            ),
        );
        Self {
            _peri: peri,
            _pin: pin,
            ch,
        }
    }

    /// Starts the pulse (TMREN = 1). With a trigger configured the counter
    /// waits for the TRGIN edge instead.
    pub fn start(&mut self) {
        T::cap_regs().set_tmren(true);
    }

    /// True while the pulse is running (TMREN auto-clears at the end).
    pub fn is_running(&self) -> bool {
        T::cap_regs().counter_running()
    }

    /// Blocking wait until the pulse finished (TMREN cleared by hardware).
    pub fn wait_done_blocking(&mut self) {
        let regs = T::cap_regs();
        if !regs.counter_running() {
            return; // never started (trigger mode) or already done
        }
        while regs.counter_running() {}
        regs.listen_channel(self.ch, false);
    }

    /// Async wait until the pulse finished (needs the timer IRQ binding).
    pub async fn wait_done(&mut self) {
        let regs = T::cap_regs();
        poll_fn(|cx| {
            T::state().waker.register(cx.waker());
            if !regs.counter_running() {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await;
    }
}

// ---------------------------------------------------------------------------
// QEI (quadrature encoder interface)
// ---------------------------------------------------------------------------

/// Quadrature encoder interface (RM 14.2.3.2): encoder mode A counts CH1
/// edges (CH2 resolves direction), direction flag in CTRL1.OWCDIR.
pub struct Qei<'d, T: CaptureInstance> {
    _peri: Peri<'d, T>,
    _ch1: Peri<'d, AnyPin>,
    _ch2: Peri<'d, AnyPin>,
    last: u16,
    accum: i64,
}

impl<'d, T: CaptureInstance> Qei<'d, T> {
    /// Configures encoder mode A on CH1/CH2.
    pub fn new(
        peri: Peri<'d, T>,
        ch1: Peri<'d, impl super::Channel1Pin<T>>,
        ch2: Peri<'d, impl super::Channel2Pin<T>>,
    ) -> Self {
        T::enable_and_reset();
        let regs = T::cap_regs();
        regs.set_capture_channel(
            Channel::Ch1,
            false,
            Edge::Rising,
            Filter::NONE,
            EventDivider::Every,
        );
        regs.set_capture_channel(
            Channel::Ch2,
            false,
            Edge::Rising,
            Filter::NONE,
            EventDivider::Every,
        );
        regs.enable_capture_channel(Channel::Ch1, true);
        regs.enable_capture_channel(Channel::Ch2, true);
        regs.set_encoder_mode(0b001); // encoder mode A
        regs.set_tmren(true);
        ch1.set_as_input(crate::gpio::Pull::None);
        ch2.set_as_input(crate::gpio::Pull::None);
        let ch1 = Peri::into(ch1);
        let ch2 = Peri::into(ch2);
        Self {
            _peri: peri,
            _ch1: ch1,
            _ch2: ch2,
            last: 0,
            accum: 0,
        }
    }

    /// Current accumulated count (wrap-safe across the 16-bit counter).
    pub fn count(&mut self) -> i64 {
        let now = T::cap_regs().counter() as u16;
        let delta = (now as i16).wrapping_sub(self.last as i16);
        self.accum += delta as i64;
        self.last = now;
        self.accum
    }

    /// Current count direction (CTRL1.OWCDIR: false = up, true = down).
    pub fn direction_down(&self) -> bool {
        T::cap_regs().direction_down()
    }

    /// Count overflow flag (missed steps indicator, cleared on read).
    pub fn take_overflowed(&self) -> bool {
        let r = T::cap_regs();
        let f = r.overflow_flag();
        if f {
            r.clear_overflow_flag();
        }
        f
    }
}
