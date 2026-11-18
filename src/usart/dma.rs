use embassy_hal_internal::drop::OnDrop;

use crate::dma::{Channel, Config as DmaConfig, DmaError, Width};

use super::{mode, Error, Instance, RxDma, TxDma, Uart};

impl<'d, T: Instance> Uart<'d, T, mode::Async> {
    /// DMA transmit (CTRL3.DMATEN): streams `buf` over a DMA1/2 channel to
    /// the DT register and waits for TDC (shift register drained) before
    /// returning - with RS485 enabled, DE deasserts before this future
    /// resolves. `tx_dma`'s channel IRQ must be bound with `bind_interrupts!`.
    ///
    /// # Safety
    ///
    /// Use DMA-accessible memory. If the pending future is forgotten, keep
    /// the buffer available to DMA until the channel stops.
    pub async unsafe fn write_dma<TXD: TxDma<T>>(
        &mut self,
        buf: &[u8],
        tx_dma: &mut Channel<'_, TXD>,
    ) -> Result<(), Error> {
        if buf.is_empty() {
            return Ok(());
        }
        let r = T::regs();
        let dt_addr = r.dt() as *const _ as u32;
        unsafe {
            tx_dma.configure_memory_to_peripheral(
                TXD::request(),
                dt_addr,
                buf.as_ptr(),
                u16::try_from(buf.len()).map_err(|_| DmaError::CountTooLarge)?,
                DmaConfig::mem_to_peripheral(Width::Bits8),
            )?;
        }
        r.sts().modify(|_, w| w.tdc().clear_bit());
        r.ctrl3().modify(|_, w| w.dmaten().enable());
        let _disable_request = OnDrop::new(|| {
            r.ctrl3().modify(|_, w| w.dmaten().disable());
        });
        tx_dma.start();
        let res = tx_dma.wait_transfer().await;
        if res.is_ok() {
            // Wait for the last byte to leave the shift register (TDC), so
            // DE has deasserted and reusing the DMA channel is safe.
            while !r.sts().read().tdc().bit_is_set() {}
        }
        res.map_err(Error::from)
    }

    /// DMA receive (CTRL3.DMAREN): fills `buf` with one frame batch and
    /// returns when the transfer count hits zero. `rx_dma`'s channel IRQ
    /// must be bound with `bind_interrupts!`.
    ///
    /// # Safety
    ///
    /// Use DMA-accessible memory. If the pending future is forgotten, keep
    /// the buffer available to DMA until the channel stops.
    pub async unsafe fn read_dma<RXD: RxDma<T>>(
        &mut self,
        buf: &mut [u8],
        rx_dma: &mut Channel<'_, RXD>,
    ) -> Result<(), Error> {
        if buf.is_empty() {
            return Ok(());
        }
        let r = T::regs();
        let dt_addr = r.dt() as *const _ as u32;
        unsafe {
            rx_dma.configure_peripheral_to_memory(
                RXD::request(),
                dt_addr,
                buf.as_mut_ptr(),
                u16::try_from(buf.len()).map_err(|_| DmaError::CountTooLarge)?,
                DmaConfig::peripheral_to_memory(Width::Bits8),
            )?;
        }
        r.sts().modify(|_, w| w.rdbf().clear_bit());
        r.ctrl3().modify(|_, w| w.dmaren().enable());
        let _disable_request = OnDrop::new(|| {
            r.ctrl3().modify(|_, w| w.dmaren().disable());
        });
        rx_dma.start();
        let res = rx_dma.wait_transfer().await;
        res.map_err(Error::from)
    }
}

// DMA routing: any DMA1/2 channel can serve any USART/UART request via the
// DMAMUX (8 instances, TX + RX).
macro_rules! impl_usart_dma_routes {
    ($($ch:ident),*) => {
        $(
            impl TxDma<crate::peripherals::USART1> for crate::peripherals::$ch {
                #[inline] fn request() -> crate::dma::Request { crate::dma::Request::Usart1Tx }
            }
            impl RxDma<crate::peripherals::USART1> for crate::peripherals::$ch {
                #[inline] fn request() -> crate::dma::Request { crate::dma::Request::Usart1Rx }
            }
            impl TxDma<crate::peripherals::USART2> for crate::peripherals::$ch {
                #[inline] fn request() -> crate::dma::Request { crate::dma::Request::Usart2Tx }
            }
            impl RxDma<crate::peripherals::USART2> for crate::peripherals::$ch {
                #[inline] fn request() -> crate::dma::Request { crate::dma::Request::Usart2Rx }
            }
            impl TxDma<crate::peripherals::USART3> for crate::peripherals::$ch {
                #[inline] fn request() -> crate::dma::Request { crate::dma::Request::Usart3Tx }
            }
            impl RxDma<crate::peripherals::USART3> for crate::peripherals::$ch {
                #[inline] fn request() -> crate::dma::Request { crate::dma::Request::Usart3Rx }
            }
            impl TxDma<crate::peripherals::USART6> for crate::peripherals::$ch {
                #[inline] fn request() -> crate::dma::Request { crate::dma::Request::Usart6Tx }
            }
            impl RxDma<crate::peripherals::USART6> for crate::peripherals::$ch {
                #[inline] fn request() -> crate::dma::Request { crate::dma::Request::Usart6Rx }
            }
            impl TxDma<crate::peripherals::UART4> for crate::peripherals::$ch {
                #[inline] fn request() -> crate::dma::Request { crate::dma::Request::Uart4Tx }
            }
            impl RxDma<crate::peripherals::UART4> for crate::peripherals::$ch {
                #[inline] fn request() -> crate::dma::Request { crate::dma::Request::Uart4Rx }
            }
            impl TxDma<crate::peripherals::UART5> for crate::peripherals::$ch {
                #[inline] fn request() -> crate::dma::Request { crate::dma::Request::Uart5Tx }
            }
            impl RxDma<crate::peripherals::UART5> for crate::peripherals::$ch {
                #[inline] fn request() -> crate::dma::Request { crate::dma::Request::Uart5Rx }
            }
            impl TxDma<crate::peripherals::UART7> for crate::peripherals::$ch {
                #[inline] fn request() -> crate::dma::Request { crate::dma::Request::Uart7Tx }
            }
            impl RxDma<crate::peripherals::UART7> for crate::peripherals::$ch {
                #[inline] fn request() -> crate::dma::Request { crate::dma::Request::Uart7Rx }
            }
            impl TxDma<crate::peripherals::UART8> for crate::peripherals::$ch {
                #[inline] fn request() -> crate::dma::Request { crate::dma::Request::Uart8Tx }
            }
            impl RxDma<crate::peripherals::UART8> for crate::peripherals::$ch {
                #[inline] fn request() -> crate::dma::Request { crate::dma::Request::Uart8Rx }
            }
        )*
    };
}

impl_usart_dma_routes!(
    DMA1_CH1, DMA1_CH2, DMA1_CH3, DMA1_CH4, DMA1_CH5, DMA1_CH6, DMA1_CH7, //
    DMA2_CH1, DMA2_CH2, DMA2_CH3, DMA2_CH4, DMA2_CH5, DMA2_CH6, DMA2_CH7
);
