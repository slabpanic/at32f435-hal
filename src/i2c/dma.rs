use super::*;
use crate::dma::ChannelInstance;
use embassy_hal_internal::drop::OnDrop;

// ---------------------------------------------------------------------------
// I2cDma: DMA-driven master (async)
// ---------------------------------------------------------------------------

/// DMA-driven I2C master. Bind both controller interrupts and the two DMA
/// channel interrupts with `bind_interrupts!`.
pub struct I2cDma<'d, T: Instance, TX: TxDma<T>, RX: RxDma<T>> {
    _peri: Peri<'d, T>,
    _scl: Peri<'d, crate::gpio::AnyPin>,
    _sda: Peri<'d, crate::gpio::AnyPin>,
    tx_dma: Channel<'d, TX>,
    rx_dma: Channel<'d, RX>,
}

impl<'d, T: Instance, TXD: TxDma<T>, RXD: RxDma<T>> I2cDma<'d, T, TXD, RXD> {
    /// Creates the DMA master with bindings for the EVT and ERR vectors.
    pub fn new(
        peri: Peri<'d, T>,
        scl: Peri<'d, impl SclPin<T>>,
        sda: Peri<'d, impl SdaPin<T>>,
        tx_dma: Peri<'d, TXD>,
        rx_dma: Peri<'d, RXD>,
        _irq: impl Binding<T::Interrupt, InterruptHandler<T>>
            + Binding<T::ErrorInterrupt, ErrorInterruptHandler<T>>
            + 'd,
        config: Config,
    ) -> Self {
        let scl_pin = new_pin!(
            scl,
            AfType::output(OutputType::OpenDrain, Drive::Normal, config.pull)
        );
        let sda_pin = new_pin!(
            sda,
            AfType::output(OutputType::OpenDrain, Drive::Normal, config.pull)
        );
        let tx = Channel::new(tx_dma);
        let rx = Channel::new(rx_dma);
        T::enable_and_reset();
        configure::<T>(&config);
        T::Interrupt::unpend();
        T::ErrorInterrupt::unpend();
        T::Interrupt::enable();
        T::ErrorInterrupt::enable();
        Self {
            _peri: peri,
            _scl: scl_pin.unwrap(),
            _sda: sda_pin.unwrap(),
            tx_dma: tx,
            rx_dma: rx,
        }
    }

    fn validate(addr: u8, len: usize) -> Result<(), Error> {
        SlaveAddr::Addr7(addr).validate()?;
        if !(1..=255).contains(&len) {
            return Err(Error::Overrun);
        }
        Ok(())
    }

    /// Writes one 1..=255 byte buffer and generates STOP.
    ///
    /// # Safety
    ///
    /// Use DMA-accessible memory. If the pending future is forgotten, keep
    /// the buffer valid for reads until the TX channel stops.
    pub async unsafe fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
        Self::validate(addr, bytes.len())?;
        wait_bus_free::<T>(DEFAULT_SPIN)?;
        self.write_phase(addr, bytes, true).await
    }

    /// Reads into one 1..=255 byte buffer and generates STOP.
    ///
    /// # Safety
    ///
    /// Use DMA-accessible memory. If the pending future is forgotten, keep
    /// the buffer exclusively available to DMA until the RX channel stops.
    pub async unsafe fn read(&mut self, addr: u8, buf: &mut [u8]) -> Result<(), Error> {
        Self::validate(addr, buf.len())?;
        wait_bus_free::<T>(DEFAULT_SPIN)?;
        self.read_phase(addr, buf).await
    }

    /// Writes, issues a repeated START, then reads before generating STOP.
    ///
    /// # Safety
    ///
    /// The buffers must satisfy the contracts of [`Self::write`] and
    /// [`Self::read`].
    pub async unsafe fn write_read(
        &mut self,
        addr: u8,
        bytes: &[u8],
        buf: &mut [u8],
    ) -> Result<(), Error> {
        Self::validate(addr, bytes.len())?;
        Self::validate(addr, buf.len())?;
        wait_bus_free::<T>(DEFAULT_SPIN)?;
        self.write_phase(addr, bytes, false).await?;
        self.read_phase(addr, buf).await
    }

    async unsafe fn write_phase(
        &mut self,
        addr: u8,
        bytes: &[u8],
        auto_stop: bool,
    ) -> Result<(), Error> {
        self.tx_dma.configure_memory_to_peripheral(
            TXD::request(),
            T::regs().txdt() as *const _ as u32,
            bytes.as_ptr(),
            bytes.len() as u16,
            DmaConfig::mem_to_peripheral(Width::Bits8),
        )?;
        self.tx_dma.start();
        run_phase::<T, TXD>(addr, bytes.len(), false, auto_stop).await
    }

    async unsafe fn read_phase(&mut self, addr: u8, buf: &mut [u8]) -> Result<(), Error> {
        self.rx_dma.configure_peripheral_to_memory(
            RXD::request(),
            T::regs().rxdt() as *const _ as u32,
            buf.as_mut_ptr(),
            buf.len() as u16,
            DmaConfig::peripheral_to_memory(Width::Bits8),
        )?;
        self.rx_dma.start();
        run_phase::<T, RXD>(addr, buf.len(), true, true).await
    }
}

fn mask_requests<T: Instance>() {
    critical_section::with(|_| {
        T::regs().ctrl1().modify(|_, w| {
            w.ackfailien()
                .clear_bit()
                .errien()
                .clear_bit()
                .stopien()
                .clear_bit()
                .tdcien()
                .clear_bit()
                .dmaren()
                .clear_bit()
                .dmaten()
                .clear_bit()
        });
    });
}

async fn run_phase<T: Instance, D: ChannelInstance>(
    addr: u8,
    len: usize,
    read: bool,
    auto_stop: bool,
) -> Result<(), Error> {
    let r = T::regs();
    let abort = OnDrop::new(|| {
        mask_requests::<T>();
        Channel::<D>::stop_transfer();
        recover::<T>(status_error(r.sts().read().bits()).err(), DEFAULT_SPIN);
    });
    clear_error_flags::<T>();
    let done = if auto_stop { STS_STOPF } else { STS_TDC };
    Channel::<D>::enable_transfer_irq();
    critical_section::with(|_| {
        r.ctrl1().modify(|_, w| {
            w.ackfailien()
                .set_bit()
                .errien()
                .set_bit()
                .stopien()
                .bit(auto_stop)
                .tdcien()
                .bit(!auto_stop)
        });
        program_chunk::<T>(SlaveAddr::Addr7(addr), read, len, auto_stop, true, false);
        r.ctrl1()
            .modify(|_, w| w.dmaren().bit(read).dmaten().bit(!read));
    });

    poll_fn(|cx| {
        T::waker().register(cx.waker());
        let status = r.sts().read().bits();
        status_error(status)?;
        if Channel::<D>::poll_transfer(cx)?.is_ready() && status & done != 0 {
            Poll::Ready(Ok::<(), Error>(()))
        } else {
            Poll::Pending
        }
    })
    .await?;
    mask_requests::<T>();
    Channel::<D>::stop_transfer();
    if auto_stop {
        r.clr().write(|w| w.stopc().set_bit());
        r.ctrl2().reset();
    }
    abort.defuse();
    Ok(())
}

/// Event interrupt handler for I2Cx_EVT (NACK and end of phase).
pub struct InterruptHandler<T: Instance>(core::marker::PhantomData<T>);

impl<T: Instance> Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let r = T::regs();
        let status = r.sts().read().bits();
        critical_section::with(|_| {
            r.ctrl1().modify(|_, w| {
                if status & STS_ACKFAIL != 0 {
                    w.ackfailien().clear_bit();
                }
                if status & STS_STOPF != 0 {
                    w.stopien().clear_bit();
                }
                if status & (STS_TDC | STS_TCRLD) != 0 {
                    w.tdcien().clear_bit();
                }
                w
            })
        });
        T::waker().wake();
    }
}

/// Error interrupt handler for I2Cx_ERR.
pub struct ErrorInterruptHandler<T: Instance>(core::marker::PhantomData<T>);

impl<T: Instance> Handler<T::ErrorInterrupt> for ErrorInterruptHandler<T> {
    unsafe fn on_interrupt() {
        critical_section::with(|_| T::regs().ctrl1().modify(|_, w| w.errien().clear_bit()));
        T::waker().wake();
    }
}

// DMA routing impls (any physical channel can serve any request line).
macro_rules! impl_i2c_dma_routes {
    ($($ch:ident),*) => {
        $(
            impl TxDma<crate::peripherals::I2C1> for crate::peripherals::$ch {
                #[inline] fn request() -> crate::dma::Request { crate::dma::Request::I2c1Tx }
            }
            impl RxDma<crate::peripherals::I2C1> for crate::peripherals::$ch {
                #[inline] fn request() -> crate::dma::Request { crate::dma::Request::I2c1Rx }
            }
            impl TxDma<crate::peripherals::I2C2> for crate::peripherals::$ch {
                #[inline] fn request() -> crate::dma::Request { crate::dma::Request::I2c2Tx }
            }
            impl RxDma<crate::peripherals::I2C2> for crate::peripherals::$ch {
                #[inline] fn request() -> crate::dma::Request { crate::dma::Request::I2c2Rx }
            }
            impl TxDma<crate::peripherals::I2C3> for crate::peripherals::$ch {
                #[inline] fn request() -> crate::dma::Request { crate::dma::Request::I2c3Tx }
            }
            impl RxDma<crate::peripherals::I2C3> for crate::peripherals::$ch {
                #[inline] fn request() -> crate::dma::Request { crate::dma::Request::I2c3Rx }
            }
        )*
    };
}

impl_i2c_dma_routes!(
    DMA1_CH1, DMA1_CH2, DMA1_CH3, DMA1_CH4, DMA1_CH5, DMA1_CH6, DMA1_CH7, //
    DMA2_CH1, DMA2_CH2, DMA2_CH3, DMA2_CH4, DMA2_CH5, DMA2_CH6, DMA2_CH7
);
