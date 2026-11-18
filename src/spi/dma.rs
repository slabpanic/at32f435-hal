use super::*;
use core::{future::poll_fn, task::Poll};
use embassy_hal_internal::drop::OnDrop;

// ---------------------------------------------------------------------------
// SpiDma: DMA-driven SPI (async)
// ---------------------------------------------------------------------------

/// Idle byte clocked out during receive-only DMA transfers (safe for both
/// CPOL settings: the receiver ignores it). Lives in flash (.rodata): the DMA
/// only reads it.
static DMA_DUMMY_BYTE: u8 = 0xFF;
static DMA_DUMMY_HALFWORD: u16 = 0xFFFF;

// One aligned discard slot per DMA channel. Only the owner of that channel
// may arm DMA into its slot, and the CPU never creates references to it.
static mut DMA_RX_SINK: [u16; 14] = [0; 14];

/// DMA-driven SPI master: async `write`/`read`/`transfer` plus the blocking
/// byte-at-a-time fallbacks. The TX/RX DMA channel IRQs must be bound via
/// `bind_interrupts!` (the constructor enables them).
///
/// With CRC enabled ([`SpiDma::enable_crc`]) the hardware automatically
/// appends the TCRC frame after the DMA'd data and checks the received CRC
/// (RM 13.2.5 DMA mode); the receive CRC frame lands in DT and is drained by
/// the driver.
pub struct SpiDma<'d, T: Instance, TX: TxDma<T>, RX: RxDma<T>> {
    /// Ownership proof of the SPI singleton (register access goes via T::regs()).
    _peri: Peri<'d, T>,
    _sck: Peri<'d, crate::gpio::AnyPin>,
    _miso: Peri<'d, crate::gpio::AnyPin>,
    _mosi: Peri<'d, crate::gpio::AnyPin>,
    _cs: Option<Peri<'d, crate::gpio::AnyPin>>,
    tx_dma: Channel<'d, TX>,
    rx_dma: Channel<'d, RX>,
    frame: FrameSize,
    hw_cs: bool,
}

impl<'d, T: Instance, TXD: TxDma<T>, RXD: RxDma<T>> SpiDma<'d, T, TXD, RXD> {
    /// DMA SPI master. `tx_dma`/`rx_dma` are routed to the instance's TX/RX
    /// request lines automatically; their completion interrupts are enabled.
    pub fn new(
        peri: Peri<'d, T>,
        sck: Peri<'d, impl SckPin<T>>,
        miso: Peri<'d, impl MisoPin<T>>,
        mosi: Peri<'d, impl MosiPin<T>>,
        tx_dma: Peri<'d, TXD>,
        rx_dma: Peri<'d, RXD>,
        config: Config,
    ) -> SpiDma<'d, T, TXD, RXD> {
        let sck_pin = new_pin!(
            sck,
            AfType::output(OutputType::PushPull, Speed::High.into(), Pull::None)
        );
        let miso_pin = new_pin!(miso, AfType::input(Pull::None));
        let mosi_pin = new_pin!(
            mosi,
            AfType::output(OutputType::PushPull, Speed::High.into(), Pull::None)
        );
        Self::new_inner(
            peri,
            sck_pin.unwrap(),
            miso_pin.unwrap(),
            mosi_pin.unwrap(),
            None,
            tx_dma,
            rx_dma,
            config,
            false,
        )
    }

    /// DMA SPI master with the hardware CS output on the `SPIx_CS` AF pin
    /// (RM 13.2.3). Frame bus transactions with [`SpiDma::transaction`] or
    /// the `cs_assert`/`cs_release` pair: the async data methods only run
    /// while CS is asserted (SPIEN on). The release Hi-Zs the pin, so the
    /// CS pin gets the internal pull-up (see [`Spi::new_blocking_hwcs`]).
    #[allow(clippy::too_many_arguments)]
    pub fn new_hwcs(
        peri: Peri<'d, T>,
        sck: Peri<'d, impl SckPin<T>>,
        miso: Peri<'d, impl MisoPin<T>>,
        mosi: Peri<'d, impl MosiPin<T>>,
        cs: Peri<'d, impl CsPin<T>>,
        tx_dma: Peri<'d, TXD>,
        rx_dma: Peri<'d, RXD>,
        config: Config,
    ) -> SpiDma<'d, T, TXD, RXD> {
        let sck_pin = new_pin!(
            sck,
            AfType::output(OutputType::PushPull, Speed::High.into(), Pull::None)
        );
        let miso_pin = new_pin!(miso, AfType::input(Pull::None));
        let mosi_pin = new_pin!(
            mosi,
            AfType::output(OutputType::PushPull, Speed::High.into(), Pull::None)
        );
        let cs_pin = new_pin!(
            cs,
            AfType::output(OutputType::PushPull, Speed::High.into(), Pull::Up)
        );
        Self::new_inner(
            peri,
            sck_pin.unwrap(),
            miso_pin.unwrap(),
            mosi_pin.unwrap(),
            Some(cs_pin.unwrap()),
            tx_dma,
            rx_dma,
            config,
            true,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn new_inner(
        peri: Peri<'d, T>,
        sck: Peri<'d, crate::gpio::AnyPin>,
        miso: Peri<'d, crate::gpio::AnyPin>,
        mosi: Peri<'d, crate::gpio::AnyPin>,
        cs: Option<Peri<'d, crate::gpio::AnyPin>>,
        tx_dma: Peri<'d, TXD>,
        rx_dma: Peri<'d, RXD>,
        config: Config,
        hw_cs: bool,
    ) -> SpiDma<'d, T, TXD, RXD> {
        let mut tx = Channel::new(tx_dma);
        let mut rx = Channel::new(rx_dma);
        T::enable_and_reset();
        let r = T::regs();
        configure(r, &config, T::frequency(), hw_cs);
        // Route the mux requests (TX side and RX side) before enabling SPI,
        // and let the controller raise DMA requests (CTRL2.DMATEN/DMAREN; the
        // DMA channels themselves are armed per transfer).
        tx.set_request(TXD::request());
        rx.set_request(RXD::request());
        r.ctrl2().modify(|_, w| {
            w.dmaten().set_bit();
            w.dmaren().set_bit();
            w
        });
        if !hw_cs {
            r.ctrl1().modify(|_, w| w.spien().enable());
        }
        Self {
            _peri: peri,
            _sck: sck,
            _miso: miso,
            _mosi: mosi,
            _cs: cs,
            tx_dma: tx,
            rx_dma: rx,
            frame: config.frame_size,
            hw_cs,
        }
    }
}

impl<'d, T: Instance, TX: TxDma<T>, RX: RxDma<T>> SpiDma<'d, T, TX, RX> {
    /// Asserts the hardware CS (SPIEN on). Only meaningful in hwcs mode; the
    /// pin stays low until [`SpiDma::cs_release`].
    pub fn cs_assert(&mut self) {
        T::regs().ctrl1().modify(|_, w| w.spien().enable());
    }

    /// Releases the hardware CS: waits for the shift register to empty, then
    /// clears SPIEN (RM 13.2.3 requires transmit-complete AND SPIEN off
    /// before the pin is released).
    pub fn cs_release(&mut self) {
        let r = T::regs();
        while r.sts().read().bf().bit_is_set() {}
        r.ctrl1().modify(|_, w| w.spien().disable());
    }

    /// Runs `f` inside one hardware-CS-framed transaction: CS asserted, `f`
    /// awaited (an async closure; use the ordinary async methods inside),
    /// CS released. In software-CS mode this just awaits `f`.
    pub async fn transaction<R>(&mut self, f: impl AsyncFnOnce(&mut Self) -> R) -> R {
        let hw_cs = self.hw_cs;
        if hw_cs {
            self.cs_assert();
        }
        let _release = OnDrop::new(|| {
            if hw_cs {
                let _ = wait_idle::<T>();
                T::regs().ctrl1().modify(|_, w| w.spien().disable());
            }
        });
        f(self).await
    }

    /// CRC polynomial (CPOLY). Call while no transfer is running.
    pub fn set_crc_polynomial(&mut self, poly: u16) {
        T::regs().cpoly().write(|w| unsafe { w.bits(poly) });
    }

    /// Enables the CRC calculation units (CTRL1.CCEN; forces the 0 -> 1
    /// edge that resets TCRC/RCRC - see [`Spi::enable_crc`]). With CCEN set,
    /// the DMA transfer methods automatically send/check the CRC frame after
    /// the DMA'd data (RM 13.2.5); the receive CRC frame lands in DT and is
    /// drained by the driver.
    pub fn enable_crc(&mut self, en: bool) {
        let r = T::regs();
        if en {
            r.ctrl1().modify(|_, w| w.ccen().disable());
        }
        r.ctrl1().modify(|_, w| {
            if en {
                w.ccen().enable();
            } else {
                w.ccen().disable();
            }
            w
        });
    }

    /// Transmit CRC accumulator (TCRC).
    pub fn crc_tx_value(&self) -> u16 {
        T::regs().tcrc().read().bits()
    }

    /// Receive CRC accumulator (RCRC).
    pub fn crc_rx_value(&self) -> u16 {
        T::regs().rcrc().read().bits()
    }

    /// CRC receive-check error flag (STS.CCERR, write-0-to-clear); cleared
    /// by this call.
    pub fn crc_error(&self) -> bool {
        let r = T::regs();
        let err = r.sts().read().ccerr().bit_is_set();
        if err {
            r.sts().modify(|_, w| w.ccerr().clear());
        }
        err
    }

    // Safety: Any supplied pointer describes count aligned frames and remains
    // DMA-accessible until the transfer completes or the channels stop.
    async unsafe fn transfer_frames(
        &mut self,
        rx: Option<*mut u8>,
        tx: Option<*const u8>,
        len: usize,
    ) -> Result<(), Error> {
        if len == 0 {
            return Ok(());
        }
        let width = self.frame.dma_width();
        let count = (len / width.bytes()) as u16;
        let data_reg = dt_addr(T::regs());
        let idle = match width {
            Width::Bits8 => &DMA_DUMMY_BYTE as *const u8,
            _ => (&DMA_DUMMY_HALFWORD as *const u16).cast(),
        };
        let sink_index = 7 * RX::DMA_INDEX + RX::CH_INDEX;
        let sink = core::ptr::addr_of_mut!(DMA_RX_SINK)
            .cast::<u16>()
            .add(sink_index)
            .cast();
        self.rx_dma.configure_peripheral_to_memory(
            RX::request(),
            data_reg,
            rx.unwrap_or(sink),
            count,
            DmaConfig {
                memory_increment: rx.is_some(),
                ..DmaConfig::peripheral_to_memory(width)
            },
        )?;
        self.tx_dma.configure_memory_to_peripheral(
            TX::request(),
            data_reg,
            tx.unwrap_or(idle),
            count,
            DmaConfig {
                memory_increment: tx.is_some(),
                ..DmaConfig::mem_to_peripheral(width)
            },
        )?;

        // Drain any stale receive data before arming the next transfer.
        drain_rx::<T>();
        let _stop = OnDrop::new(|| {
            Channel::<TX>::stop_transfer();
            Channel::<RX>::stop_transfer();
        });
        self.rx_dma.start();
        self.tx_dma.start();
        Channel::<RX>::enable_transfer_irq();
        Channel::<TX>::enable_transfer_irq();
        // Poll both directions so that either channel can report an error.
        poll_fn(|cx| {
            let rx = Channel::<RX>::poll_transfer(cx)?;
            let tx = Channel::<TX>::poll_transfer(cx)?;
            if rx.is_ready() && tx.is_ready() {
                Poll::Ready(Ok::<(), DmaError>(()))
            } else {
                Poll::Pending
            }
        })
        .await?;
        wait_idle::<T>()?;
        drain_rx::<T>();
        Ok(())
    }

    /// Async read, clocking the idle value on MOSI.
    ///
    /// # Safety
    ///
    /// Use DMA-accessible memory. If the pending future is forgotten, keep
    /// the buffer exclusively available to DMA until the RX channel stops.
    pub async unsafe fn read(&mut self, buf: &mut [u8]) -> Result<(), Error> {
        check_buffer(self.frame, buf.as_ptr(), buf.len())?;
        self.transfer_frames(Some(buf.as_mut_ptr()), None, buf.len())
            .await
    }

    /// Async write, discarding the received frames.
    ///
    /// # Safety
    ///
    /// Use DMA-accessible memory. If the pending future is forgotten, keep
    /// the buffer valid for reads until the TX channel stops.
    pub async unsafe fn write(&mut self, buf: &[u8]) -> Result<(), Error> {
        check_buffer(self.frame, buf.as_ptr(), buf.len())?;
        self.transfer_frames(None, Some(buf.as_ptr()), buf.len())
            .await
    }

    /// Full-duplex DMA transfer. The longer side finishes with idle TX
    /// frames or discarded RX frames once the shorter side runs out.
    /// With CRC enabled, nonempty buffers must have equal lengths so the
    /// hardware appends a single CRC frame for the transaction.
    ///
    /// # Safety
    ///
    /// The buffers must satisfy the contracts of [`Self::read`] and
    /// [`Self::write`].
    pub async unsafe fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Error> {
        check_buffer(self.frame, read.as_ptr(), read.len())?;
        check_buffer(self.frame, write.as_ptr(), write.len())?;
        if !read.is_empty()
            && !write.is_empty()
            && read.len() != write.len()
            && T::regs().ctrl1().read().ccen().is_enabled()
        {
            return Err(Error::UnequalBuffersWithCrc);
        }
        let common = read.len().min(write.len());
        let (read_head, read_tail) = read.split_at_mut(common);
        let (write_head, write_tail) = write.split_at(common);
        self.transfer_frames(
            Some(read_head.as_mut_ptr()),
            Some(write_head.as_ptr()),
            common,
        )
        .await?;
        if !read_tail.is_empty() {
            self.transfer_frames(Some(read_tail.as_mut_ptr()), None, read_tail.len())
                .await?;
        } else if !write_tail.is_empty() {
            self.transfer_frames(None, Some(write_tail.as_ptr()), write_tail.len())
                .await?;
        }
        Ok(())
    }

    /// Transfers outgoing frames in place, replacing them with incoming frames.
    ///
    /// # Safety
    ///
    /// The buffer must satisfy the contracts of [`Self::read`] and
    /// [`Self::write`].
    pub async unsafe fn transfer_in_place(&mut self, buf: &mut [u8]) -> Result<(), Error> {
        check_buffer(self.frame, buf.as_ptr(), buf.len())?;
        let ptr = buf.as_mut_ptr();
        // A received frame can reach DMA only after the matching TX frame
        // has been read from the buffer and clocked out.
        self.transfer_frames(Some(ptr), Some(ptr.cast_const()), buf.len())
            .await
    }

    /// Blocking passthroughs (handy in mixed sync/async code).
    pub fn blocking_write(&mut self, buf: &[u8]) -> Result<(), Error> {
        for &b in buf {
            let _ = blocking_transfer_element::<T>(b as u16);
        }
        wait_idle::<T>()?;
        drain_rx::<T>();
        Ok(())
    }

    pub fn blocking_read(&mut self, buf: &mut [u8]) -> Result<(), Error> {
        for b in buf.iter_mut() {
            *b = blocking_transfer_element::<T>(0xFF) as u8;
        }
        wait_idle::<T>()
    }

    pub fn blocking_flush(&mut self) -> Result<(), Error> {
        wait_idle::<T>()
    }
}

impl<T: Instance, TX: TxDma<T>, RX: RxDma<T>> embedded_hal::spi::ErrorType
    for SpiDma<'_, T, TX, RX>
{
    type Error = Error;
}

impl<T: Instance, TX: TxDma<T>, RX: RxDma<T>> embedded_hal::spi::SpiBus for SpiDma<'_, T, TX, RX> {
    fn read(&mut self, buf: &mut [u8]) -> Result<(), Self::Error> {
        SpiDma::blocking_read(self, buf)
    }

    fn write(&mut self, buf: &[u8]) -> Result<(), Self::Error> {
        SpiDma::blocking_write(self, buf)
    }

    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        // Polling fallback (the async path needs an executor context).
        let n = read.len().max(write.len());
        for i in 0..n {
            let sent = write.get(i).copied().unwrap_or(0xFF) as u16;
            let got = blocking_transfer_element::<T>(sent) as u8;
            if let Some(r) = read.get_mut(i) {
                *r = got;
            }
        }
        wait_idle::<T>()
    }

    fn transfer_in_place(&mut self, buf: &mut [u8]) -> Result<(), Self::Error> {
        for b in buf.iter_mut() {
            let got = blocking_transfer_element::<T>(*b as u16) as u8;
            *b = got;
        }
        wait_idle::<T>()
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        SpiDma::blocking_flush(self)
    }
}

fn check_buffer(frame: FrameSize, ptr: *const u8, len: usize) -> Result<(), Error> {
    let width = frame.dma_width().bytes();
    if len != 0 && (!len.is_multiple_of(width) || !(ptr as usize).is_multiple_of(width)) {
        return Err(Error::InvalidBuffer);
    }
    u16::try_from(len / width).map_err(|_| DmaError::CountTooLarge)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::{check_buffer, DmaError, Error, FrameSize};

    #[test]
    fn checks_dma_frame_width_and_counter_before_starting() {
        let aligned = [0u16; 2];
        let ptr = aligned.as_ptr().cast::<u8>();
        assert_eq!(check_buffer(FrameSize::Bits16, ptr, 4), Ok(()));
        assert_eq!(
            check_buffer(FrameSize::Bits16, ptr, 3),
            Err(Error::InvalidBuffer)
        );
        assert_eq!(
            check_buffer(FrameSize::Bits16, ptr.wrapping_add(1), 2),
            Err(Error::InvalidBuffer)
        );
        assert_eq!(
            check_buffer(FrameSize::Bits8, ptr, 65_536),
            Err(Error::Dma(DmaError::CountTooLarge))
        );
    }
}
