//! EXINT external interrupt/event: `ExtiInput` async edge/level waiting
//! (ch32-hal `exti.rs` structure).
//!
//! The AT32F435 EXINT has 23 lines (0-22), of which **0-15 map to GPIO**
//! (line number = pin number, port selection in SCFG.EXINTC); lines 16-22 are
//! internal wakeup sources (ERTC/OTG etc., not covered by this module).
//! The pending flag `intsts` is **cleared by writing 1**; each line can have
//! its rising/falling polarity configured independently (`polcfg1/polcfg2`).
//!
//! Wakeup protocol (ch32-hal's consumed-check, race-free):
//! 1. Future construction: mask the line -> set polarity/port -> write 1 to
//!    clear any leftover pending -> enable the line;
//! 2. ISR ([`on_irq`]): read `intsts & inten` (only consume lines this module
//!    enabled) -> write 1 to clear pending -> **clear the line's enable bit**
//!    (the "consumed" signal) -> wake the waker;
//! 3. `poll`: sees the enable bit of its line cleared -> Ready; `Drop`: re-mask.
//!
//! With the `exti` feature (enabled by default), the 7 EXINT vectors
//! (EXINT0..4, EXINT9_5, EXINT15_10) are registered by the HAL and call
//! [`on_irq`] - the application must not `bind_interrupts!` them again;
//! disable the feature if you need custom binding (`Channel`/`AnyChannel`
//! types remain available regardless).

use embassy_hal_internal::{impl_peripheral, PeripheralType};

trait SealedChannel {}

/// EXINT channel trait (one singleton type per GPIO-mapped line).
#[allow(private_bounds)]
pub trait Channel: SealedChannel + PeripheralType + Sized {
    /// Line number (0-15, equal to the GPIO pin number).
    fn number(&self) -> u8;

    /// Degrade into a type-erased channel.
    fn degrade(self) -> AnyChannel {
        AnyChannel {
            number: self.number(),
        }
    }
}

/// Type-erased EXINT channel.
pub struct AnyChannel {
    number: u8,
}

impl_peripheral!(AnyChannel);

impl SealedChannel for AnyChannel {}

impl Channel for AnyChannel {
    #[inline]
    fn number(&self) -> u8 {
        self.number
    }
}

macro_rules! impl_exti {
    ($type:ident, $number:expr) => {
        impl SealedChannel for crate::peripherals::$type {}
        impl Channel for crate::peripherals::$type {
            #[inline]
            fn number(&self) -> u8 {
                $number
            }
        }
    };
}

mod _channels {
    use super::*;

    impl_exti!(EXINT0, 0);
    impl_exti!(EXINT1, 1);
    impl_exti!(EXINT2, 2);
    impl_exti!(EXINT3, 3);
    impl_exti!(EXINT4, 4);
    impl_exti!(EXINT5, 5);
    impl_exti!(EXINT6, 6);
    impl_exti!(EXINT7, 7);
    impl_exti!(EXINT8, 8);
    impl_exti!(EXINT9, 9);
    impl_exti!(EXINT10, 10);
    impl_exti!(EXINT11, 11);
    impl_exti!(EXINT12, 12);
    impl_exti!(EXINT13, 13);
    impl_exti!(EXINT14, 14);
    impl_exti!(EXINT15, 15);
}

/// Driver part (ExtiInput / on_irq / vector entries), gated by the `exti`
/// feature; when disabled, the `Channel`/`AnyChannel` types remain usable
/// (see the `on_irq` implementation for the protocol when binding vectors yourself).
#[cfg(feature = "exti")]
mod driver {
    use core::future::Future;
    use core::marker::PhantomData;
    use core::pin::Pin;
    use core::task::{Context, Poll};

    use embassy_sync::waitqueue::AtomicWaker;

    use crate::gpio::{Input, Pin as GpioPin, Pull};
    use crate::pac;

    use super::Channel;

    /// Number of GPIO-mapped EXINT lines (lines 16-22 are internal events).
    const GPIO_LINES: usize = 16;

    // Repeating a const array relies on const context (no Copy requirement);
    // the static suggested by clippy is not viable here
    #[allow(clippy::declare_interior_mutable_const)]
    const NEW_AW: AtomicWaker = AtomicWaker::new();
    static EXINT_WAKERS: [AtomicWaker; GPIO_LINES] = [NEW_AW; GPIO_LINES];

    #[inline]
    fn exint() -> &'static pac::exint::RegisterBlock {
        unsafe { &*pac::EXINT::PTR }
    }

    #[inline]
    fn scfg() -> &'static pac::scfg::RegisterBlock {
        unsafe { &*pac::SCFG::PTR }
    }

    /// External input pin: configure it for EXINT edge triggering, then use
    /// `wait_for_*` for async waiting.
    pub struct ExtiInput<'d> {
        pin: Input<'d>,
    }

    impl<'d> core::marker::Unpin for ExtiInput<'d> {}

    impl<'d> ExtiInput<'d> {
        /// Take ownership of the pin and its EXINT channel (the line number
        /// must equal the pin number, guaranteed by the assertion).
        pub fn new<T: GpioPin>(
            pin: crate::Peri<'d, T>,
            ch: crate::Peri<'d, T::ExtiChannel>,
            pull: Pull,
        ) -> Self {
            assert_eq!(
                GpioPin::pin(&*pin),
                ch.number(),
                "EXINT line number must equal the GPIO pin number"
            );
            Self {
                pin: Input::new(pin, pull),
            }
        }

        pub fn is_high(&self) -> bool {
            self.pin.is_high()
        }

        pub fn is_low(&self) -> bool {
            self.pin.is_low()
        }

        pub fn get_level(&self) -> crate::gpio::Level {
            self.pin.get_level()
        }

        /// Wait until the pin goes high (returns immediately if already high).
        pub async fn wait_for_high(&mut self) {
            if self.is_high() {
                return;
            }
            ExtiInputFuture::new(self.pin.pin_no(), self.pin.port_no(), true, false).await
        }

        /// Wait until the pin goes low (returns immediately if already low).
        pub async fn wait_for_low(&mut self) {
            if self.is_low() {
                return;
            }
            ExtiInputFuture::new(self.pin.pin_no(), self.pin.port_no(), false, true).await
        }

        pub async fn wait_for_rising_edge(&mut self) {
            ExtiInputFuture::new(self.pin.pin_no(), self.pin.port_no(), true, false).await
        }

        pub async fn wait_for_falling_edge(&mut self) {
            ExtiInputFuture::new(self.pin.pin_no(), self.pin.port_no(), false, true).await
        }

        pub async fn wait_for_any_edge(&mut self) {
            ExtiInputFuture::new(self.pin.pin_no(), self.pin.port_no(), true, true).await
        }
    }

    /// One EXINT edge wait: constructing it arms the line; after wakeup, Drop
    /// re-masks it automatically.
    pub struct ExtiInputFuture<'a> {
        pin: u8,
        _lifetime: PhantomData<&'a ()>,
    }

    impl<'a> ExtiInputFuture<'a> {
        fn new(pin: u8, port: u8, rising: bool, falling: bool) -> Self {
            assert!(pin < GPIO_LINES as u8, "EXINT GPIO lines are numbered 0-15");
            let n = pin as usize;
            let nu8 = pin;
            critical_section::with(|_| {
                let ex = exint();

                // Mask this line first (shield against spurious triggers while
                // changing polarity/port)
                ex.inten()
                    .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << n)) });

                // Polarity: polcfg1 = rising (rp), polcfg2 = falling (fp)
                ex.polcfg1().modify(|_, w| {
                    if rising {
                        w.rp(nu8).enable();
                    } else {
                        w.rp(nu8).disable();
                    }
                    w
                });
                ex.polcfg2().modify(|_, w| {
                    if falling {
                        w.fp(nu8).enable();
                    } else {
                        w.fp(nu8).disable();
                    }
                    w
                });

                // Port selection: SCFG.EXINTC{1..4}, 4 lines per register, 4
                // bits per line, value = port number (A=0..H=7)
                // Fields are named after the global line number (EXINTC2 =
                // exint4..exint7); SPEC types differ, so each register must be
                // expanded separately
                match n / 4 {
                    0 => scfg().exintc1().modify(|_, w| unsafe {
                        match n {
                            0 => w.exint0().bits(port),
                            1 => w.exint1().bits(port),
                            2 => w.exint2().bits(port),
                            _ => w.exint3().bits(port),
                        };
                        w
                    }),
                    1 => scfg().exintc2().modify(|_, w| unsafe {
                        match n {
                            4 => w.exint4().bits(port),
                            5 => w.exint5().bits(port),
                            6 => w.exint6().bits(port),
                            _ => w.exint7().bits(port),
                        };
                        w
                    }),
                    2 => scfg().exintc3().modify(|_, w| unsafe {
                        match n {
                            8 => w.exint8().bits(port),
                            9 => w.exint9().bits(port),
                            10 => w.exint10().bits(port),
                            _ => w.exint11().bits(port),
                        };
                        w
                    }),
                    _ => scfg().exintc4().modify(|_, w| unsafe {
                        match n {
                            12 => w.exint12().bits(port),
                            13 => w.exint13().bits(port),
                            14 => w.exint14().bits(port),
                            _ => w.exint15().bits(port),
                        };
                        w
                    }),
                };

                // Clear any leftover pending (write-1-to-clear), then enable the line
                ex.intsts().write(|w| w.line(nu8).clear());
                ex.inten()
                    .modify(|r, w| unsafe { w.bits(r.bits() | (1 << n)) });
            });
            Self {
                pin,
                _lifetime: PhantomData,
            }
        }
    }

    impl<'a> Future for ExtiInputFuture<'a> {
        type Output = ();

        fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
            let n = self.pin as usize;
            EXINT_WAKERS[n].register(cx.waker());
            // "consumed" = this line's enable bit was cleared by on_irq
            // (avoids the race of reading pending directly)
            if exint().inten().read().bits() & (1 << n) == 0 {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        }
    }

    impl<'a> Drop for ExtiInputFuture<'a> {
        fn drop(&mut self) {
            let n = self.pin as usize;
            critical_section::with(|_| {
                let ex = exint();
                ex.inten()
                    .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << n)) });
            });
        }
    }

    /// EXINT interrupt catch-all: clears pending, consumes the enable bits,
    /// wakes waiting tasks.
    ///
    /// Called by the vector entries of the `exti` feature; call this function
    /// too when binding EXINT vectors yourself.
    ///
    /// # Safety
    ///
    /// Must only be called from EXINT interrupt context.
    pub unsafe fn on_irq() {
        let ex = exint();
        // Only handle GPIO lines that are "pending and enabled" (0-15);
        // lines 16-22 belong to other wakeup-source vectors
        let bits = ex.intsts().read().bits() & ex.inten().read().bits() & 0xFFFF;
        if bits == 0 {
            return;
        }
        // Write 1 to clear pending; clear the enable bits (consumed signal)
        ex.intsts().write(|w| unsafe { w.bits(bits) });
        ex.inten()
            .modify(|r, w| unsafe { w.bits(r.bits() & !bits) });
        for (n, waker) in EXINT_WAKERS.iter().enumerate() {
            if bits & (1 << n) != 0 {
                waker.wake();
            }
        }
    }

    /// Enable the NVIC for the 7 EXINT vectors (called by `hal::init()`).
    pub(crate) fn init() {
        use cortex_m::peripheral::NVIC;
        use pac::Interrupt;
        unsafe {
            NVIC::unmask(Interrupt::EXINT0);
            NVIC::unmask(Interrupt::EXINT1);
            NVIC::unmask(Interrupt::EXINT2);
            NVIC::unmask(Interrupt::EXINT3);
            NVIC::unmask(Interrupt::EXINT4);
            NVIC::unmask(Interrupt::EXINT9_5);
            NVIC::unmask(Interrupt::EXINT15_10);
        }
    }
}

#[cfg(feature = "exti")]
pub(crate) use driver::init;
#[cfg(feature = "exti")]
pub use driver::{on_irq, ExtiInput, ExtiInputFuture};

/// Vector entries under `exti` + `rt` (owned by the HAL; applications must not
/// bind them again).
#[cfg(all(feature = "exti", feature = "rt"))]
mod irq {
    use crate::interrupt;

    #[crate::interrupt]
    unsafe fn EXINT0() {
        unsafe { super::driver::on_irq() };
    }
    #[crate::interrupt]
    unsafe fn EXINT1() {
        unsafe { super::driver::on_irq() };
    }
    #[crate::interrupt]
    unsafe fn EXINT2() {
        unsafe { super::driver::on_irq() };
    }
    #[crate::interrupt]
    unsafe fn EXINT3() {
        unsafe { super::driver::on_irq() };
    }
    #[crate::interrupt]
    unsafe fn EXINT4() {
        unsafe { super::driver::on_irq() };
    }
    #[crate::interrupt]
    unsafe fn EXINT9_5() {
        unsafe { super::driver::on_irq() };
    }
    #[crate::interrupt]
    unsafe fn EXINT15_10() {
        unsafe { super::driver::on_irq() };
    }
}
