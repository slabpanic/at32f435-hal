use super::*;

use core::cell::RefCell;
use core::future::poll_fn;
use core::task::Poll;

use critical_section::Mutex;
use embassy_hal_internal::drop::OnDrop;
use embassy_sync::waitqueue::MultiWakerRegistration;

use crate::interrupt::typelevel::{Binding, Handler, Interrupt as _};

pub(super) struct AsyncState {
    inner: Mutex<RefCell<MailboxState>>,
}

struct MailboxState {
    // One claim per in-flight transmit. Keep it until that future consumes
    // its result so another sender cannot reuse the completion record.
    claimed: u8,
    completed: [Option<Result<(), Error>>; 3],
    tx_wakers: MultiWakerRegistration<8>,
    rx_wakers: [MultiWakerRegistration<4>; 2],
}

impl AsyncState {
    pub(super) const fn new() -> Self {
        Self {
            inner: Mutex::new(RefCell::new(MailboxState {
                claimed: 0,
                completed: [None; 3],
                tx_wakers: MultiWakerRegistration::new(),
                rx_wakers: [const { MultiWakerRegistration::new() }; 2],
            })),
        }
    }
}

/// TX-complete interrupt handler for CANx_TX.
pub struct TxInterruptHandler<T: Instance> {
    _phantom: core::marker::PhantomData<T>,
}

impl<T: Instance> Handler<T::TxInterrupt> for TxInterruptHandler<T> {
    unsafe fn on_interrupt() {
        critical_section::with(|cs| {
            let r = T::regs();
            let status = r.tsts().read();
            let mut state = T::async_state().inner.borrow_ref_mut(cs);
            let mut any_complete = false;
            for (mailbox, (complete, success)) in [
                (status.tm0tcf().bit_is_set(), status.tm0tsf().bit_is_set()),
                (status.tm1tcf().bit_is_set(), status.tm1tsf().bit_is_set()),
                (status.tm2tcf().bit_is_set(), status.tm2tsf().bit_is_set()),
            ]
            .into_iter()
            .enumerate()
            {
                if complete {
                    clear_tx_completion(r, mailbox);
                    if state.claimed & (1 << mailbox) != 0 {
                        state.completed[mailbox] = Some(if success {
                            Ok(())
                        } else {
                            Err(Error::TxFailed)
                        });
                    }
                    any_complete = true;
                }
            }
            if any_complete {
                state.tx_wakers.wake();
            }
        });
    }
}

fn clear_tx_completion(r: &pac::can1::RegisterBlock, mailbox: usize) {
    r.tsts().write(|w| match mailbox {
        0 => w.tm0tcf().clear(),
        1 => w.tm1tcf().clear(),
        _ => w.tm2tcf().clear(),
    });
}

/// RX-FIFO0 interrupt handler. RF0MIEN is level-triggered on RF0MN != 0
/// (RM 20.7.1.6), so the handler must switch it off whenever a frame is
/// pending - otherwise the line re-asserts after every handler exit and the
/// core starves (same storm class as the ERTC/async-UART findings). The
/// receive future re-arms RFMIEN on its next poll after draining.
pub struct Rx0InterruptHandler<T: Instance> {
    _phantom: core::marker::PhantomData<T>,
}

impl<T: Instance> Handler<T::Rx0Interrupt> for Rx0InterruptHandler<T> {
    unsafe fn on_interrupt() {
        on_rx_interrupt::<T>(0);
    }
}

/// RX-FIFO1 message/full/overflow interrupt handler (see
/// [`Rx0InterruptHandler`]).
pub struct Rx1InterruptHandler<T: Instance> {
    _phantom: core::marker::PhantomData<T>,
}

impl<T: Instance> Handler<T::Rx1Interrupt> for Rx1InterruptHandler<T> {
    unsafe fn on_interrupt() {
        on_rx_interrupt::<T>(1);
    }
}

fn on_rx_interrupt<T: Instance>(fifo: usize) {
    critical_section::with(|cs| {
        let r = T::regs();
        let status = r.rf(fifo).read();
        if status.ff().bit_is_set() || status.of().bit_is_set() {
            r.rf(fifo).write(|w| {
                if status.ff().bit_is_set() {
                    w.ff().clear();
                }
                if status.of().bit_is_set() {
                    w.of().clear();
                }
                w
            });
        }
        if status.mn().bits() != 0 {
            r.inten().modify(|_, w| {
                if fifo == 0 {
                    w.rf0mien().clear_bit()
                } else {
                    w.rf1mien().clear_bit()
                }
            });
            T::async_state().inner.borrow_ref_mut(cs).rx_wakers[fifo].wake();
        }
    });
}

/// Interrupt-driven CAN controller.
/// Transmits from multiple tasks share the hardware mailboxes. Receivers
/// may share either FIFO and receive each pending frame once.
pub struct CanAsync<'d, T: Instance> {
    can: Can<'d, T>,
}

impl<'d, T: Instance> CanAsync<'d, T> {
    /// Creates an async CAN controller (same pins/configuration as
    /// [`Can::new`]) and binds the TX/RX0/RX1 interrupts.
    pub fn new(
        peri: Peri<'d, T>,
        tx: Peri<'d, impl TxPin<T>>,
        rx: Peri<'d, impl RxPin<T>>,
        config: Config,
        _tx_irq: impl Binding<T::TxInterrupt, TxInterruptHandler<T>> + 'd,
        _rx0_irq: impl Binding<T::Rx0Interrupt, Rx0InterruptHandler<T>> + 'd,
        _rx1_irq: impl Binding<T::Rx1Interrupt, Rx1InterruptHandler<T>> + 'd,
    ) -> Self {
        let can = Can::new(peri, tx, rx, config);
        critical_section::with(|cs| {
            let mut state = T::async_state().inner.borrow_ref_mut(cs);
            state.claimed = 0;
            state.completed = [None; 3];
        });
        let r = T::regs();
        // Controller interrupt sources: TX complete + FIFO overflow for both
        // FIFOs (self-clearing flags - safe to keep enabled). RFMIEN is
        // level-triggered on MN != 0 and is armed per receive() instead.
        r.inten().modify(|_, w| {
            w.tcien().set_bit();
            w.rf0oien().set_bit();
            w.rf1oien().set_bit();
            w
        });
        T::TxInterrupt::unpend();
        T::TxInterrupt::enable();
        T::Rx0Interrupt::unpend();
        T::Rx0Interrupt::enable();
        T::Rx1Interrupt::unpend();
        T::Rx1Interrupt::enable();
        Self { can }
    }

    /// Accesses the blocking driver while no async operation is borrowed.
    pub fn blocking(&mut self) -> &mut Can<'d, T> {
        &mut self.can
    }

    /// Transmits a frame. Dropping the future releases its software claim.
    /// The controller may still finish sending a frame already in a mailbox.
    pub async fn transmit(&self, frame: &Frame) -> Result<(), Error> {
        let state = &T::async_state().inner;
        let r = T::regs();
        let mailbox = poll_fn(|cx| {
            critical_section::with(|cs| {
                let mut state = state.borrow_ref_mut(cs);
                let status = r.tsts().read();
                for mailbox in 0..3 {
                    if state.claimed & (1 << mailbox) == 0 && mailbox_empty(&status, mailbox) {
                        clear_tx_completion(r, mailbox);
                        state.claimed |= 1 << mailbox;
                        state.completed[mailbox] = None;
                        write_mailbox(r, mailbox, frame);
                        return Poll::Ready(mailbox);
                    }
                }
                state.tx_wakers.register(cx.waker());
                Poll::Pending
            })
        })
        .await;
        let _release = OnDrop::new(|| {
            critical_section::with(|cs| {
                let mut state = state.borrow_ref_mut(cs);
                state.claimed &= !(1 << mailbox);
                state.tx_wakers.wake();
            });
        });
        poll_fn(|cx| {
            critical_section::with(|cs| {
                let mut state = state.borrow_ref_mut(cs);
                if let Some(result) = state.completed[mailbox].take() {
                    Poll::Ready(result)
                } else {
                    state.tx_wakers.register(cx.waker());
                    Poll::Pending
                }
            })
        })
        .await
    }

    /// Receives a frame from the selected FIFO, waiting for an interrupt if
    /// it is empty.
    pub async fn receive(&self, fifo: Fifo) -> Result<Frame, Error> {
        let fifo = match fifo {
            Fifo::Fifo0 => 0,
            Fifo::Fifo1 => 1,
        };
        let r = T::regs();
        poll_fn(|cx| {
            critical_section::with(|cs| {
                if r.rf(fifo).read().mn().bits() != 0 {
                    return Poll::Ready(Ok(read_fifo_frame(r, fifo)));
                }
                T::async_state().inner.borrow_ref_mut(cs).rx_wakers[fifo].register(cx.waker());
                r.inten().modify(|_, w| {
                    if fifo == 0 {
                        w.rf0mien().set_bit()
                    } else {
                        w.rf1mien().set_bit()
                    }
                });
                Poll::Pending
            })
        })
        .await
    }
}
