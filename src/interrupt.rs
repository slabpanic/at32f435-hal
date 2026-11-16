//! Interrupt infrastructure: typelevel IRQ zero-sized types + the
//! [`typelevel::Handler`]/[`typelevel::Binding`] traits (used by
//! `bind_interrupts!`), backed by the cortex-m NVIC.
//!
//! Usage (same as ch32-hal/embassy-stm32):
//! ```ignore
//! use at32f435_hal::{bind_interrupts, interrupt};
//!
//! bind_interrupts!(struct Irqs {
//!     TMR2 => at32f435_hal::embassy::time_driver::on_irq;  // example
//! });
//! // The driver holds Irqs as proof the IRQ is bound, so it can safely
//! // enable/operate the interrupt from within a task.
//! ```

pub use crate::pac::Interrupt;

/// IRQ names are directly available (`interrupt::TMR2` etc.): when the
/// cortex-m-rt `#[interrupt]` macro expands, it validates the interrupt
/// exists via `interrupt::<IRQ>`, so this module re-exports all enum variants.
#[allow(unused_imports)]
pub use crate::pac::Interrupt::*;

/// The NVIC itself (use together with this module's typelevel IRQ types for
/// priority/pending operations; enable/disable can also be done directly with
/// `typelevel::Interrupt::enable()/disable()`).
pub use cortex_m::peripheral::NVIC;

/// Typelevel interrupt types and traits.
pub mod typelevel {
    use cortex_m::peripheral::NVIC;

    use crate::pac::Interrupt as PacInterrupt;

    /// Common interface of the typelevel IRQ types.
    pub trait Interrupt {
        /// The corresponding PAC interrupt-number enum value.
        const IRQ: PacInterrupt;

        /// Enable this interrupt in the NVIC (unmask after setting the
        /// default lowest priority 0).
        fn enable() {
            unsafe { NVIC::unmask(Self::IRQ) }
        }

        /// Mask this interrupt in the NVIC.
        fn disable() {
            NVIC::mask(Self::IRQ);
        }

        /// Clear the NVIC pending state (a stale pending left over from a
        /// reset would fire immediately after enable, so clear it first).
        fn unpend() {
            NVIC::unpend(Self::IRQ);
        }
    }

    /// Interrupt-handler callback implemented by drivers.
    pub trait Handler<I: Interrupt> {
        /// Called from the vector handler of this IRQ (interrupt context).
        ///
        /// # Safety
        ///
        /// Runs in interrupt context: must not block, must not use
        /// non-reentrant blocking locks; the implementation must itself
        /// guarantee that peripheral-register accesses are consistent with
        /// the runtime state.
        unsafe fn on_interrupt();
    }

    /// Ownership proof of an (IRQ, Handler) combination by a
    /// "struct XxxIrqs" type.
    ///
    /// `bind_interrupts!` generates an unsafe impl for each binding; a driver
    /// holding the binding struct has exclusive access to that IRQ.
    ///
    /// # Safety
    ///
    /// Only one impl of a given (IRQ, Handler) combination may exist in the
    /// program - otherwise the same vector would generate duplicate
    /// `#[interrupt]` handler functions (link-time symbol conflicts) or
    /// duplicated wakeup logic.
    pub unsafe trait Binding<I: Interrupt, H: Handler<I>> {}

    macro_rules! irq_type {
        ($name:ident) => {
            #[allow(non_camel_case_types)]
            #[derive(Copy, Clone, Debug)]
            pub struct $name(());
            impl Interrupt for $name {
                const IRQ: PacInterrupt = PacInterrupt::$name;
            }
        };
    }

    // Full table: one-to-one with the pac::Interrupt enum (AT32F435 has 115
    // vectors in total).
    irq_type!(WWDT);
    irq_type!(PVM);
    irq_type!(TAMPER);
    irq_type!(ERTC_WKUP);
    irq_type!(FLASH);
    irq_type!(CRM);
    irq_type!(EXINT0);
    irq_type!(EXINT1);
    irq_type!(EXINT2);
    irq_type!(EXINT3);
    irq_type!(EXINT4);
    irq_type!(EDMA_STREAM1);
    irq_type!(EDMA_STREAM2);
    irq_type!(EDMA_STREAM3);
    irq_type!(EDMA_STREAM4);
    irq_type!(EDMA_STREAM5);
    irq_type!(EDMA_STREAM6);
    irq_type!(EDMA_STREAM7);
    irq_type!(ADC);
    irq_type!(CAN1_TX);
    irq_type!(CAN1_RX0);
    irq_type!(CAN_RX1);
    irq_type!(CAN_SE);
    irq_type!(EXINT9_5);
    irq_type!(TMR1_BRK_TMR9);
    irq_type!(TMR1_OVF_TMR10);
    irq_type!(TMR1_TRG_HALL_TMR11);
    irq_type!(TMR1_CH);
    irq_type!(TMR2);
    irq_type!(TMR3);
    irq_type!(TMR4);
    irq_type!(I2C1_EVT);
    irq_type!(I2C1_ERR);
    irq_type!(I2C2_EVT);
    irq_type!(I2C2_ERR);
    irq_type!(SPI1);
    irq_type!(SPI2);
    irq_type!(USART1);
    irq_type!(USART2);
    irq_type!(USART3);
    irq_type!(EXINT15_10);
    irq_type!(ERTCALARM);
    irq_type!(OTGFS1_WKUP);
    irq_type!(TMR8_BRK_TMR12);
    irq_type!(TMR8_OVF_TMR13);
    irq_type!(TMR8_TRG_HALL_TMR14);
    irq_type!(TMR8_CH);
    irq_type!(EDMA_STREAM8);
    irq_type!(XMC);
    irq_type!(SDIO1);
    irq_type!(TMR5);
    irq_type!(SPI3);
    irq_type!(UART4);
    irq_type!(UART5);
    irq_type!(TMR6);
    irq_type!(TMR7);
    irq_type!(DMA1_CHANNEL1);
    irq_type!(DMA1_CHANNEL2);
    irq_type!(DMA1_CHANNEL3);
    irq_type!(DMA1_CHANNEL4);
    irq_type!(DMA1_CHANNEL5);
    irq_type!(EMAC_WKUP);
    irq_type!(CAN2_TX);
    irq_type!(CAN2_RX0);
    irq_type!(CAN2_RX1);
    irq_type!(CAN2_SE);
    irq_type!(OTGFS1);
    irq_type!(DMA1_CHANNEL6);
    irq_type!(DMA1_CHANNEL7);
    irq_type!(USART6);
    irq_type!(I2C3_EVT);
    irq_type!(I2C3_ERR);
    irq_type!(OTGFS2_WKUP);
    irq_type!(OTGFS2);
    irq_type!(DVP);
    irq_type!(UART7);
    irq_type!(UART8);
    irq_type!(SPI4);
    irq_type!(QSPI2);
    irq_type!(QSPI1);
    irq_type!(SDIO2);
    irq_type!(TMR20_BRK);
    irq_type!(TMR20_OVF);
    irq_type!(TMR20_TRG_HALL);
    irq_type!(TMR20_CH);
    irq_type!(DMA2_CHANNEL1);
    irq_type!(DMA2_CHANNEL2);
    irq_type!(DMA2_CHANNEL3);
    irq_type!(DMA2_CHANNEL4);
    irq_type!(DMA2_CHANNEL5);
    irq_type!(DMA2_CHANNEL6);
    irq_type!(DMA2_CHANNEL7);
}
