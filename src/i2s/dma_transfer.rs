use super::*;

// ---------------------------------------------------------------------------
// DMA plumbing (request routing; the async data paths ride the same Channel
// abstraction as the SPI driver)
// ---------------------------------------------------------------------------

impl<'d, T: Instance> I2s<'d, T> {
    /// Arms and starts a TX DMA transfer of halfwords into DT (16-bit formats:
    /// one halfword per frame; 24/32-bit formats: the caller supplies the
    /// pre-packed halfword stream, see [`pack_sample32`]). The channel stays
    /// armed - await its completion through [`dma::Channel::wait_transfer`]
    /// (the channel IRQ must be bound with `bind_interrupts!`).
    ///
    /// # Safety
    ///
    /// Keep DMA-accessible buffer memory valid until the channel completes
    /// or is stopped. The buffer borrow ends before the hardware transfer.
    pub unsafe fn write_dma_hw<X: ChannelInstance + TxDma<T>>(
        &mut self,
        ch: &mut dma::Channel<'d, X>,
        hw: &[u16],
    ) -> Result<(), Error> {
        let count = u16::try_from(hw.len()).map_err(|_| Error::Timeout)?;
        if count == 0 {
            return Err(Error::Timeout);
        }
        // Peripheral-side DMA request enable (CTRL2.DMATEN, shared with the
        // SPI personality) must be on or the I2S never raises the request.
        T::regs().ctrl2().modify(|_, w| w.dmaten().set_bit());
        unsafe {
            ch.configure_memory_to_peripheral(
                X::request(),
                dt_addr(T::regs()),
                hw.as_ptr() as *const u8,
                count,
                dma::Config::mem_to_peripheral(dma::Width::Bits16),
            )
            .map_err(|_| Error::Timeout)?;
            ch.start();
        }
        Ok(())
    }

    /// Arms and starts an RX DMA transfer of halfwords from DT (await via
    /// [`dma::Channel::wait_transfer`]).
    ///
    /// # Safety
    ///
    /// Keep DMA-accessible buffer memory valid until the channel completes
    /// or is stopped. The buffer borrow ends before the hardware transfer.
    pub unsafe fn read_dma_hw<X: ChannelInstance + RxDma<T>>(
        &mut self,
        ch: &mut dma::Channel<'d, X>,
        buf: &mut [u16],
    ) -> Result<(), Error> {
        let count = u16::try_from(buf.len()).map_err(|_| Error::Timeout)?;
        if count == 0 {
            return Err(Error::Timeout);
        }
        T::regs().ctrl2().modify(|_, w| w.dmaren().set_bit());
        unsafe {
            ch.configure_peripheral_to_memory(
                X::request(),
                dt_addr(T::regs()),
                buf.as_mut_ptr() as *mut u8,
                count,
                dma::Config::peripheral_to_memory(dma::Width::Bits16),
            )
            .map_err(|_| Error::Timeout)?;
            ch.start();
        }
        Ok(())
    }
}

/// DMAMUX request routing impls: main blocks use the parent SPI request
/// lines, the EXT blocks their own (RM table 9-3: 110-113).
macro_rules! impl_i2s_dma_routes {
    ($($ch:ident),*) => {
        $(
            impl TxDma<crate::peripherals::SPI1> for crate::peripherals::$ch {
                #[inline] fn request() -> crate::dma::Request { crate::dma::Request::Spi1Tx }
            }
            impl RxDma<crate::peripherals::SPI1> for crate::peripherals::$ch {
                #[inline] fn request() -> crate::dma::Request { crate::dma::Request::Spi1Rx }
            }
            impl TxDma<crate::peripherals::SPI2> for crate::peripherals::$ch {
                #[inline] fn request() -> crate::dma::Request { crate::dma::Request::Spi2Tx }
            }
            impl RxDma<crate::peripherals::SPI2> for crate::peripherals::$ch {
                #[inline] fn request() -> crate::dma::Request { crate::dma::Request::Spi2Rx }
            }
            impl TxDma<crate::peripherals::SPI3> for crate::peripherals::$ch {
                #[inline] fn request() -> crate::dma::Request { crate::dma::Request::Spi3Tx }
            }
            impl RxDma<crate::peripherals::SPI3> for crate::peripherals::$ch {
                #[inline] fn request() -> crate::dma::Request { crate::dma::Request::Spi3Rx }
            }
            impl TxDma<crate::peripherals::SPI4> for crate::peripherals::$ch {
                #[inline] fn request() -> crate::dma::Request { crate::dma::Request::Spi4Tx }
            }
            impl RxDma<crate::peripherals::SPI4> for crate::peripherals::$ch {
                #[inline] fn request() -> crate::dma::Request { crate::dma::Request::Spi4Rx }
            }
            impl TxDma<crate::peripherals::I2S2_EXT> for crate::peripherals::$ch {
                #[inline] fn request() -> crate::dma::Request { crate::dma::Request::I2s2ExtTx }
            }
            impl RxDma<crate::peripherals::I2S2_EXT> for crate::peripherals::$ch {
                #[inline] fn request() -> crate::dma::Request { crate::dma::Request::I2s2ExtRx }
            }
            impl TxDma<crate::peripherals::I2S3_EXT> for crate::peripherals::$ch {
                #[inline] fn request() -> crate::dma::Request { crate::dma::Request::I2s3ExtTx }
            }
            impl RxDma<crate::peripherals::I2S3_EXT> for crate::peripherals::$ch {
                #[inline] fn request() -> crate::dma::Request { crate::dma::Request::I2s3ExtRx }
            }
        )*
    };
}

impl_i2s_dma_routes!(
    DMA1_CH1, DMA1_CH2, DMA1_CH3, DMA1_CH4, DMA1_CH5, DMA1_CH6, DMA1_CH7, //
    DMA2_CH1, DMA2_CH2, DMA2_CH3, DMA2_CH4, DMA2_CH5, DMA2_CH6, DMA2_CH7
);
