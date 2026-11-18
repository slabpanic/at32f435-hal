//! Basic timer time base: TMR6/TMR7 (RM 14.1) - 16-bit up-counter, DIV prescaler
//! and PR period, overflow interrupt / software event, single-cycle mode.
//!
//! Update frequency = fCK / ((DIV+1) * (PR+1)) where fCK is the APB1 timer
//! clock (2x PCLK1 when the APB1 divider is > 1, provided by
//! `RccPeripheral::frequency`).
//!
//! ```ignore
//! let mut tmr = hal::timer::TimerBase::new(p.TMR6, Hertz::hz(1_000));
//! tmr.listen(); // needs bind_interrupts! on TMR6
//! while tmr.wait_update().await { /* every 1 ms */ }
//! ```

use core::future::poll_fn;
use core::marker::PhantomData;
use core::task::Poll;

use embassy_hal_internal::Peri;
use embassy_sync::waitqueue::AtomicWaker;

use crate::interrupt::typelevel::{Handler, Interrupt as _};
use crate::peripheral::RccPeripheral;
use crate::time::Hertz;
use crate::{interrupt, pac};

pub(crate) struct BaseState {
    pub(crate) waker: AtomicWaker,
}

impl BaseState {
    pub(crate) const fn new() -> Self {
        Self {
            waker: AtomicWaker::new(),
        }
    }
}

trait SealedBase {
    fn regs() -> &'static pac::tmr6::RegisterBlock;
    fn state() -> &'static BaseState;
}

/// Basic timer instance trait (TMR6/TMR7).
#[allow(private_bounds)]
pub trait BaseInstance:
    SealedBase + RccPeripheral + embassy_hal_internal::PeripheralType + 'static
{
    /// The timer's update interrupt (typelevel).
    type Interrupt: interrupt::typelevel::Interrupt;
}

macro_rules! impl_base_instance {
    ($inst:ident, $irq:ident) => {
        impl SealedBase for crate::peripherals::$inst {
            #[inline]
            fn regs() -> &'static pac::tmr6::RegisterBlock {
                unsafe { &*pac::$inst::PTR }
            }

            #[inline]
            fn state() -> &'static BaseState {
                static STATE: BaseState = BaseState::new();
                &STATE
            }
        }

        impl BaseInstance for crate::peripherals::$inst {
            type Interrupt = interrupt::typelevel::$irq;
        }
    };
}

impl_base_instance!(TMR6, TMR6);
impl_base_instance!(TMR7, TMR7);

/// Basic timer time base.
pub struct TimerBase<'d, T: BaseInstance> {
    _peri: Peri<'d, T>,
    _mode: PhantomData<T>,
}

impl<'d, T: BaseInstance> TimerBase<'d, T> {
    /// Configures the timer for the requested overflow frequency and starts
    /// counting. DIV/PR are loaded through a software overflow event (PRBEN is
    /// off, so they take effect immediately).
    ///
    /// # Panics
    ///
    /// Panics if the frequency cannot be reached (needs 2 <= cycles-per-update
    /// <= 65536 * 65536).
    pub fn new(peri: Peri<'d, T>, update_freq: Hertz) -> Self {
        let (divider, period) = super::frequency_dividers(T::frequency(), update_freq, 65_536);
        T::enable_and_reset();
        let r = T::regs();
        r.div().write(|w| w.set(divider));
        r.pr().write(|w| w.set(period as u16));
        // Generate an overflow event to load the shadow registers.
        r.swevt().write(|w| w.ovfswtr().overflow());
        r.iden().write(|w| w.ovfien().disable());
        r.ctrl1().modify(|_, w| {
            w.ovfen().clear_bit(); // allow overflow events
            w.tmren().set_bit();
            w
        });
        Self {
            _peri: peri,
            _mode: PhantomData,
        }
    }

    fn r() -> &'static pac::tmr6::RegisterBlock {
        T::regs()
    }

    /// Current counter value.
    pub fn now(&self) -> u16 {
        Self::r().cval().read().bits()
    }

    /// Reconfigures the overflow frequency (immediate, PRBEN off).
    pub fn set_frequency(&mut self, update_freq: Hertz) {
        let (divider, period) = super::frequency_dividers(T::frequency(), update_freq, 65_536);
        let r = Self::r();
        r.div().write(|w| w.set(divider));
        r.pr().write(|w| w.set(period as u16));
        r.swevt().write(|w| w.ovfswtr().overflow());
    }

    /// Enables the update interrupt (requires the `bind_interrupts!` binding
    /// of [`InterruptHandler<T>`]).
    pub fn listen(&mut self) {
        Self::r().iden().modify(|_, w| w.ovfien().enable());
        T::Interrupt::unpend();
        T::Interrupt::enable();
    }

    /// Disables the update interrupt.
    pub fn unlisten(&mut self) {
        Self::r().iden().modify(|_, w| w.ovfien().disable());
        T::Interrupt::disable();
    }

    /// Stops the counter (TMREN = 0).
    pub fn stop(&mut self) {
        Self::r().ctrl1().modify(|_, w| w.tmren().disable());
    }

    /// Resumes the counter.
    pub fn resume(&mut self) {
        Self::r().ctrl1().modify(|_, w| w.tmren().enable());
    }

    /// Single-cycle mode: the counter stops after the current period
    /// (CTRL1.OCMEN = 1); `resume()` restarts one period.
    pub fn set_single_cycle(&mut self, on: bool) {
        Self::r().ctrl1().modify(|_, w| {
            if on {
                w.ocmen().set_bit();
            } else {
                w.ocmen().clear_bit();
            }
            w
        });
    }

    /// Whether the overflow flag is pending.
    pub fn overflow_flag(&self) -> bool {
        Self::r().ists().read().ovfif().bit_is_set()
    }

    /// Clears the overflow flag.
    pub fn clear_overflow_flag(&self) {
        Self::r().ists().write(|w| w.ovfif().clear());
    }

    /// Waits (async) for the next overflow event; requires `listen()`. The
    /// flag is consumed here (IST is rw0c) so the IRQ does not re-fire.
    pub async fn wait_update(&mut self) {
        let r = Self::r();
        r.iden().modify(|_, w| w.ovfien().enable());
        poll_fn(|cx| {
            T::state().waker.register(cx.waker());
            if r.ists().read().ovfif().bit_is_set() {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await;
        r.ists().write(|w| w.ovfif().clear());
    }

    /// Blocking wait for the next overflow flag (busy-poll, no IRQ needed).
    pub fn blocking_wait(&self) {
        let r = Self::r();
        while !r.ists().read().ovfif().bit_is_set() {}
        r.ists().write(|w| w.ovfif().clear());
    }
}

/// Basic timer update interrupt handler: bind it with
/// `bind_interrupts!(struct Irqs { TMR6 => hal::timer::base::InterruptHandler<hal::peripherals::TMR6>; });`
pub struct InterruptHandler<T: BaseInstance> {
    _phantom: PhantomData<T>,
}

impl<T: BaseInstance> Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let r = T::regs();
        if r.ists().read().ovfif().bit_is_set() {
            // Stop the IRQ from re-firing; the waiter consumes the flag.
            r.iden().modify(|_, w| w.ovfien().disable());
            T::state().waker.wake();
        }
    }
}
