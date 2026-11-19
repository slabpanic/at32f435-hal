//! I2C master driver: I2C1-3 (RM 11, ch32-hal `i2c/` structure).
//!
//! The AT32F435 I2C is the counter-based generation (STM32F0/F7 style):
//! transfers are programmed with CTRL2 (SADDR/DIR/CNT/RLDEN/ASTOPEN/GENSTART)
//! and clocked by CLKCTRL (DIV/SDAD/SCLD/SCLH/SCLL). Master transfers use
//! TDIS/RDBF for data and TDC/STOPF for the end of each phase.
//!
//! - [`I2c`]: blocking master (embedded-hal 1.0 `I2c`), byte-at-a-time with a
//!   loop-count timeout. Transfers over 255 bytes use the reload mode
//!   (RLDEN + TCRLD chunking, RM 11.4.2) instead of erroring. 10-bit
//!   addressing (`blocking_write_10bit`/`blocking_read_10bit`, RM 11
//!   figures 11-8/11-9) and SMBus master PEC transfers
//!   (`blocking_smbus_write_pec`/`blocking_smbus_read_pec`, RM 11.4.6 - the
//!   hardware appends/checks the PEC byte) live on the blocking driver; the
//!   SMBus timeout detector comes via `configure_smbus_timeout`.
//! - [`I2cDma`]: DMA-driven master (async). Data moves over DMA (TXDT/RXDT);
//!   the EVT and ERR handlers wake the transfer on bus events and errors.
//! - Timing: CLKCTRL is computed from PCLK1 (tHIGH = (SCLH+1)*(DIV+1)*Ti2c,
//!   tLOW = (SCLL+1)*(DIV+1)*Ti2c, RM 11.4.3). Pass `Config::clkctrl` to set
//!   the raw register value instead (e.g. from Artery's timing tool).
//!
//! Pins are open-drain alternate functions. Use external pull-ups for a
//! wired bus, or internal pull-ups for a probe with the pins unconnected.
//!
//! ```ignore
//! let mut i2c = hal::i2c::I2c::new_blocking(
//!     p.I2C1, p.PB6, p.PB7, hal::i2c::Config {
//!         pull: hal::gpio::Pull::Up,
//!         ..Default::default()
//!     },
//! );
//! let acknowledged = i2c.blocking_probe(0x50)?;
//! ```

mod dma;
mod pins;
pub use dma::{ErrorInterruptHandler, I2cDma, InterruptHandler};

use core::future::poll_fn;
use core::task::Poll;

use embassy_hal_internal::Peri;
use embassy_sync::waitqueue::AtomicWaker;

use crate::dma::{Channel, Config as DmaConfig, Width};
use crate::gpio::{AfType, Drive, OutputType, Pull};
use crate::interrupt::typelevel::{Binding, Handler, Interrupt as _};
use crate::peripheral::RccPeripheral;
use crate::time::Hertz;
use crate::{interrupt, pac};

/// Bus operation error.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[non_exhaustive]
pub enum Error {
    /// Bus error (BUSERR, e.g. misplaced START/STOP).
    Bus,
    /// Arbitration lost (ARLOST).
    ArbitrationLost,
    /// The addressed device did not acknowledge (ACKFAIL).
    NoAcknowledge,
    /// Overflow/underflow (OUF): software too slow for the configured speed.
    Overrun,
    /// Polling timeout while waiting for a flag.
    Timeout,
    /// Slave address outside the selected addressing mode.
    InvalidAddress,
    /// A DMA transfer failed.
    Dma(crate::dma::DmaError),
}

impl From<crate::dma::DmaError> for Error {
    fn from(e: crate::dma::DmaError) -> Self {
        Error::Dma(e)
    }
}

impl embedded_hal::i2c::Error for Error {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        use embedded_hal::i2c::ErrorKind;
        match self {
            Error::Bus => ErrorKind::Bus,
            Error::ArbitrationLost => ErrorKind::ArbitrationLoss,
            Error::NoAcknowledge => {
                ErrorKind::NoAcknowledge(embedded_hal::i2c::NoAcknowledgeSource::Unknown)
            }
            Error::Overrun => ErrorKind::Overrun,
            Error::Timeout | Error::InvalidAddress | Error::Dma(_) => ErrorKind::Other,
        }
    }
}

/// I2C configuration.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct Config {
    /// SCL frequency target (the actual value depends on the CLKCTRL rounding).
    pub freq: Hertz,
    /// Raw CLKCTRL register value overriding the computed timing (the four
    /// sub-fields: `DIV[31:24]` `SDAD[19:16]` `SCLD[15:12]` `SCLH[15:8]` `SCLL[7:0]`).
    pub clkctrl: Option<u32>,
    /// Pin pull for SCL/SDA: external pull-ups are the proper solution, but
    /// `Pull::Up` (internal, weak) lets short probes run on bare headers.
    pub pull: Pull,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            freq: Hertz(100_000),
            clkctrl: None,
            pull: Pull::None,
        }
    }
}

/// Computes the CLKCTRL register value for the requested SCL frequency
/// (RM 11.4.3 formulas; targets ~64 ticks per SCL period).
pub fn compute_clkctrl(pclk: Hertz, freq: Hertz) -> u32 {
    let pclk_hz = pclk.0;
    let period_ns = 1_000_000_000u64 / freq.0.max(1) as u64;
    // DIV+1: aim for 64 ticks per period, clamped to the field range.
    let div1 = (pclk_hz as u64 * 64 / (freq.0.max(1) as u64 * 1_000_000)).clamp(1, 256) as u32;
    let tick_ns = 1_000_000_000u64 * div1 as u64 / pclk_hz as u64;
    // High/low split per the I2C spec duty targets (incl. rise/fall margin).
    let (high_ns, low_ns) = if freq.0 > 100_000 {
        (period_ns * 26 / 100, period_ns * 55 / 100) // fast mode
    } else {
        (period_ns * 40 / 100, period_ns * 47 / 100) // standard mode
    };
    let sclh = ((high_ns / tick_ns).max(1) as u32 - 1).min(255);
    let scll = ((low_ns / tick_ns).max(1) as u32 - 1).min(255);
    // Data hold ~150 ns, data setup per the speed class (SM 250 ns / FM 100 ns).
    let sdad = (((150_000u64 / tick_ns).max(1) as u32) - 1).min(15);
    let scld_setup = if freq.0 > 100_000 {
        100_000u64
    } else {
        250_000
    };
    let scld = (((scld_setup / tick_ns).max(1) as u32) - 1).min(15);
    (div1.wrapping_sub(1) << 24) | (sdad << 16) | (scld << 12) | (sclh << 8) | scll
}

trait SealedInstance {
    fn regs() -> &'static pac::i2c1::RegisterBlock;
    fn waker() -> &'static AtomicWaker;
}

/// I2C instance trait (EVT interrupt is the driver's handshake source).
#[allow(private_bounds)]
pub trait Instance:
    SealedInstance + RccPeripheral + embassy_hal_internal::PeripheralType + 'static
{
    /// This instance's EVT interrupt (typelevel).
    type Interrupt: interrupt::typelevel::Interrupt;
    /// This instance's error interrupt.
    type ErrorInterrupt: interrupt::typelevel::Interrupt;
}

macro_rules! impl_i2c_instance {
    ($inst:ident, $irq:ident, $err:ident) => {
        impl SealedInstance for crate::peripherals::$inst {
            #[inline]
            fn regs() -> &'static pac::i2c1::RegisterBlock {
                unsafe { &*pac::$inst::PTR }
            }

            #[inline]
            fn waker() -> &'static AtomicWaker {
                static WAKER: AtomicWaker = AtomicWaker::new();
                &WAKER
            }
        }

        impl Instance for crate::peripherals::$inst {
            type Interrupt = interrupt::typelevel::$irq;
            type ErrorInterrupt = interrupt::typelevel::$err;
        }
    };
}

impl_i2c_instance!(I2C1, I2C1_EVT, I2C1_ERR);
impl_i2c_instance!(I2C2, I2C2_EVT, I2C2_ERR);
impl_i2c_instance!(I2C3, I2C3_EVT, I2C3_ERR);

crate::pin_trait!(SclPin, Instance);
crate::pin_trait!(SdaPin, Instance);

dma_trait!(TxDma, Instance);
dma_trait!(RxDma, Instance);

// Flag bits of I2C_STS (RM 11.7.7, cross-checked against the official
// library I2C_STS_* masks and the PAC field positions; 2026-11-17). The
// original constants had ADDRF/ACKFAIL/STOPF shifted onto TDC/TMOUT/ALERTF -
// the open-bus probe test never noticed because it only exercised ARLOST.
const STS_TDIS: u32 = 1 << 1;
const STS_RDBF: u32 = 1 << 2;
const STS_ACKFAIL: u32 = 1 << 4;
const STS_STOPF: u32 = 1 << 5;
const STS_TDC: u32 = 1 << 6;
const STS_TCRLD: u32 = 1 << 7;
const STS_BUSERR: u32 = 1 << 8;
const STS_ARLOST: u32 = 1 << 9;
const STS_OUF: u32 = 1 << 10;
const STS_PECERR: u32 = 1 << 11;
const STS_TMOUT: u32 = 1 << 12;

fn status_error(sts: u32) -> Result<(), Error> {
    if sts & STS_BUSERR != 0 {
        Err(Error::Bus)
    } else if sts & STS_ARLOST != 0 {
        Err(Error::ArbitrationLost)
    } else if sts & STS_ACKFAIL != 0 {
        Err(Error::NoAcknowledge)
    } else if sts & STS_OUF != 0 {
        Err(Error::Overrun)
    } else if sts & STS_TMOUT != 0 {
        Err(Error::Timeout)
    } else {
        Ok(())
    }
}

/// Clears the write-1-to-clear error flags (CLR register).
fn clear_error_flags<T: Instance>() {
    let r = T::regs();
    r.clr().write(|w| {
        w.buserrc().set_bit();
        w.arlostc().set_bit();
        w.ackfailc().set_bit();
        w.oufc().set_bit();
        w.tmoutc().set_bit();
        w.pecerrc().set_bit();
        w
    });
}

// Leave the controller ready for the next operation after an error. NACK
// already generates STOP and arbitration loss has relinquished the bus.
fn recover<T: Instance>(error: Option<Error>, spin: u32) {
    let r = T::regs();
    let timeout_config = if error == Some(Error::Timeout) {
        let config = r.timeout().read().bits();
        // Temporarily mask timeout sources so they cannot interrupt cleanup.
        r.timeout()
            .write(|w| unsafe { w.bits(config & !((1 << 15) | (1 << 31))) });
        Some(config)
    } else {
        None
    };
    if r.sts().read().busyf().bit_is_set() && error != Some(Error::ArbitrationLost) {
        if error != Some(Error::NoAcknowledge) {
            r.ctrl2().modify(|_, w| w.genstop().set_bit());
        }
        for _ in 0..spin.min(1 << 16) {
            if r.sts().read().stopf().bit_is_set() {
                break;
            }
        }
    }
    r.clr().write(|w| w.stopc().set_bit());
    clear_error_flags::<T>();
    if error == Some(Error::Timeout) {
        // A clock timeout can leave the controller busy after GENSTOP. Reset
        // its transaction state while releasing both open-drain lines.
        r.ctrl1().modify(|_, w| w.i2cen().disable());
        r.ctrl2().reset();
        r.ctrl1().modify(|_, w| w.i2cen().enable());
        r.clr().write(|w| w.stopc().set_bit());
        clear_error_flags::<T>();
        for _ in 0..spin.min(1 << 16) {
            if r.sts().read().busyf().bit_is_clear() {
                break;
            }
        }
    }
    if r.sts().read().busyf().bit_is_clear() {
        // The vendor middleware flushes a partially filled transmit register
        // after an aborted operation. TDBE is set by writing a one.
        r.sts().modify(|_, w| w.tdbe().set_bit());
    }
    if let Some(config) = timeout_config {
        r.timeout().write(|w| unsafe { w.bits(config) });
    }
}

/// Registers the computed/raw CLKCTRL timing and enables the controller.
fn configure<T: Instance>(config: &Config) {
    let r = T::regs();
    r.ctrl1().modify(|_, w| w.i2cen().disable());
    let clkctrl = config
        .clkctrl
        .unwrap_or_else(|| compute_clkctrl(T::frequency(), config.freq));
    r.clkctrl().write(|w| unsafe { w.bits(clkctrl) });
    // Full clock stretching (STRETCH default), analog+digital filter off
    // (kept at reset; i2c_init(dfilter) maps onto CTRL1.DFLT).
    r.ctrl1().modify(|_, w| w.i2cen().enable());
}

/// Waits for a set of STS bits with a loop-count timeout.
fn wait_flags<T: Instance>(mask: u32, spin: u32) -> Result<(), Error> {
    for _ in 0..spin {
        let status = T::regs().sts().read().bits();
        status_error(status)?;
        if status & mask != 0 {
            return Ok(());
        }
    }
    Err(Error::Timeout)
}

/// Slave address for the blocking transfer primitives.
#[derive(Copy, Clone)]
enum SlaveAddr {
    /// 7-bit address (CTRL2.ADDR10 = 0).
    Addr7(u8),
    /// 10-bit address (CTRL2.ADDR10 = 1; the hardware sends the
    /// 11110 A9 A8 W header plus the A7..A0 byte, RM 11.4.3).
    Addr10(u16),
}

impl SlaveAddr {
    fn validate(self) -> Result<(), Error> {
        match self {
            Self::Addr7(0..=0x7f) | Self::Addr10(0..=0x3ff) => Ok(()),
            _ => Err(Error::InvalidAddress),
        }
    }

    fn saddr(self) -> u16 {
        match self {
            SlaveAddr::Addr7(a) => u16::from(a) << 1,
            SlaveAddr::Addr10(a) => a,
        }
    }

    fn is_10bit(self) -> bool {
        matches!(self, SlaveAddr::Addr10(_))
    }
}

/// Programs CTRL2 for one transfer chunk. Reload chunking for transfers
/// longer than 255 bytes: RLDEN = 1 with CNT = min(remaining, 255) while a
/// further chunk follows, RLDEN = 0 with CNT = remaining for the final
/// (auto-stop) chunk. `genstart` arms the START condition - set for every
/// transfer start (initial or repeated), clear for mid-transfer reloads.
fn program_chunk<T: Instance>(
    slave: SlaveAddr,
    dir: bool,
    remaining: usize,
    astopen: bool,
    genstart: bool,
    pec: bool,
) {
    let r = T::regs();
    let (rlden, cnt) = if remaining > 255 {
        (true, 255)
    } else {
        (false, remaining)
    };
    r.ctrl2().write(|w| unsafe {
        w.saddr().bits(slave.saddr());
        if slave.is_10bit() {
            w.addr10().set_bit();
        }
        if dir {
            w.dir().set_bit();
        }
        if genstart {
            w.genstart().set_bit();
        }
        if pec {
            w.pecten().set_bit();
        }
        if astopen && !rlden {
            w.astopen().set_bit();
        }
        w.rlden().bit(rlden);
        w.cnt().bits(cnt as u8);
        w
    });
}

/// Waits for an idle bus (BUSF clear) with a loop-count timeout.
fn wait_bus_free<T: Instance>(spin: u32) -> Result<(), Error> {
    for _ in 0..spin {
        if !T::regs().sts().read().busyf().bit_is_set() {
            return Ok(());
        }
    }
    Err(Error::Timeout)
}

fn wait_stop<T: Instance>(spin: u32) -> Result<(), Error> {
    wait_flags::<T>(STS_STOPF, spin)?;
    T::regs().clr().write(|w| w.stopc().set_bit());
    Ok(())
}

fn operation_len(op: &embedded_hal::i2c::Operation<'_>) -> usize {
    match op {
        embedded_hal::i2c::Operation::Read(buf) => buf.len(),
        embedded_hal::i2c::Operation::Write(buf) => buf.len(),
    }
}

fn is_read(op: &embedded_hal::i2c::Operation<'_>) -> bool {
    matches!(op, embedded_hal::i2c::Operation::Read(_))
}

fn blocking_transaction<T: Instance>(
    slave: SlaveAddr,
    ops: &mut [embedded_hal::i2c::Operation<'_>],
    spin: u32,
) -> Result<(), Error> {
    slave.validate()?;
    if ops.is_empty() {
        return Ok(());
    }
    wait_bus_free::<T>(spin)?;
    let r = T::regs();
    let result = (|| {
        let mut ops = ops;
        while let Some(first) = ops.first() {
            let read = is_read(first);
            let group_len = ops.iter().take_while(|op| is_read(op) == read).count();
            let final_group = group_len == ops.len();
            let (group, remaining_ops) = ops.split_at_mut(group_len);
            let mut remaining = group.iter().try_fold(0usize, |len, op| {
                len.checked_add(operation_len(op)).ok_or(Error::Overrun)
            })?;
            program_chunk::<T>(slave, read, remaining, final_group, true, false);

            let mut chunk_left = remaining.min(255);
            for op in group {
                let len = operation_len(op);
                for index in 0..len {
                    if chunk_left == 0 {
                        wait_flags::<T>(STS_TCRLD, spin)?;
                        program_chunk::<T>(slave, read, remaining, final_group, false, false);
                        chunk_left = remaining.min(255);
                    }
                    match op {
                        embedded_hal::i2c::Operation::Read(buf) => {
                            wait_flags::<T>(STS_RDBF, spin)?;
                            buf[index] = r.rxdt().read().bits() as u8;
                        }
                        embedded_hal::i2c::Operation::Write(buf) => {
                            wait_flags::<T>(STS_TDIS, spin)?;
                            r.txdt().write(|w| unsafe { w.bits(u32::from(buf[index])) });
                        }
                    }
                    remaining -= 1;
                    chunk_left -= 1;
                }
            }
            if final_group {
                wait_stop::<T>(spin)?;
                r.ctrl2().reset();
            } else {
                wait_flags::<T>(STS_TDC, spin)?;
            }
            ops = remaining_ops;
        }
        Ok(())
    })();
    if let Err(error) = result {
        recover::<T>(Some(error), spin);
    }
    result
}

/// Blocking I2C master.
pub struct I2c<'d, T: Instance> {
    _peri: Peri<'d, T>,
    _scl: Peri<'d, crate::gpio::AnyPin>,
    _sda: Peri<'d, crate::gpio::AnyPin>,
    spin: u32,
    last_pec_value: u8,
}

// Loop-count bound for flag waits: each iteration is a register read plus
// arithmetic (~10 cycles), so 1<<24 ~ 0.6 s at 288 MHz.
const DEFAULT_SPIN: u32 = 1 << 24;

impl<'d, T: Instance> I2c<'d, T> {
    /// Blocking I2C master (open-drain AF pins, external pull-ups expected).
    pub fn new_blocking(
        peri: Peri<'d, T>,
        scl: Peri<'d, impl SclPin<T>>,
        sda: Peri<'d, impl SdaPin<T>>,
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
        T::enable_and_reset();
        configure::<T>(&config);
        Self {
            _peri: peri,
            _scl: scl_pin.unwrap(),
            _sda: sda_pin.unwrap(),
            spin: DEFAULT_SPIN,
            last_pec_value: 0,
        }
    }

    /// Override the flag-wait loop budget (default ~0.6 s worth of cycles).
    pub fn set_timeout_spins(&mut self, spin: u32) {
        self.spin = spin;
    }

    /// Blocking write of `bytes` to `addr` (START -> addr+W -> data -> STOP).
    /// Transfers longer than 255 bytes use the reload mode (RLDEN, TCRLD
    /// chunking, RM 11.4.2) instead of erroring.
    pub fn blocking_write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
        blocking_transaction::<T>(
            SlaveAddr::Addr7(addr),
            &mut [embedded_hal::i2c::Operation::Write(bytes)],
            self.spin,
        )
    }

    /// Blocking read of `buf.len()` bytes from `addr` (reload chunking for
    /// lengths over 255).
    pub fn blocking_read(&mut self, addr: u8, buf: &mut [u8]) -> Result<(), Error> {
        blocking_transaction::<T>(
            SlaveAddr::Addr7(addr),
            &mut [embedded_hal::i2c::Operation::Read(buf)],
            self.spin,
        )
    }

    /// Writes, issues a repeated START, then reads before generating STOP.
    pub fn blocking_write_read(
        &mut self,
        addr: u8,
        bytes: &[u8],
        buf: &mut [u8],
    ) -> Result<(), Error> {
        use embedded_hal::i2c::Operation::{Read, Write};
        blocking_transaction::<T>(
            SlaveAddr::Addr7(addr),
            &mut [Write(bytes), Read(buf)],
            self.spin,
        )
    }

    /// Blocking write to a 10-bit-addressed slave (CTRL2.ADDR10 = 1; the
    /// hardware sends the 11110 A9 A8 W header plus the A7..A0 byte).
    /// Successful transfers still need validation with a 10-bit target.
    pub fn blocking_write_10bit(&mut self, addr10: u16, bytes: &[u8]) -> Result<(), Error> {
        blocking_transaction::<T>(
            SlaveAddr::Addr10(addr10),
            &mut [embedded_hal::i2c::Operation::Write(bytes)],
            self.spin,
        )
    }

    /// Blocking read from a 10-bit-addressed slave (RM 11 figure 11-9,
    /// READH10 = 0): address-only write phase, then a repeated START
    /// re-sending the header byte with r = 1. Successful transfers still
    /// need validation with a 10-bit target.
    pub fn blocking_read_10bit(&mut self, addr10: u16, buf: &mut [u8]) -> Result<(), Error> {
        blocking_transaction::<T>(
            SlaveAddr::Addr10(addr10),
            &mut [embedded_hal::i2c::Operation::Read(buf)],
            self.spin,
        )
    }

    /// SMBus master write with PEC (RM 11.4.6, figure 11-14): enables PECEN,
    /// programs CNT = len+1 with PECTEN; the hardware appends the PEC byte
    /// (CRC-8 over address + data) after the data automatically. The captured
    /// value is available through [`I2c::last_pec_value`]. len + 1 must not
    /// exceed 255.
    pub fn blocking_smbus_write_pec(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
        if bytes.is_empty() || bytes.len() + 1 > 255 {
            return Err(Error::Overrun);
        }
        SlaveAddr::Addr7(addr).validate()?;
        self.set_pec_enabled(true);
        let r = T::regs();
        let res = (|| {
            wait_bus_free::<T>(self.spin)?;
            program_chunk::<T>(
                SlaveAddr::Addr7(addr),
                false,
                bytes.len() + 1,
                true,
                true,
                true,
            );
            // Software writes len bytes; hardware auto-sends the PEC as the
            // final (Nth) byte.
            for &b in bytes {
                wait_flags::<T>(STS_TDIS, self.spin)?;
                r.txdt().write(|w| unsafe { w.bits(b as u32) });
            }
            wait_stop::<T>(self.spin)
        })();
        if let Err(error) = res {
            recover::<T>(Some(error), self.spin);
        } else {
            // I2CEN = 0 resets PECVAL, so preserve the result before disabling PEC.
            self.last_pec_value = self.pec_value();
        }
        let pec_error = res.is_ok() && self.pec_error();
        self.set_pec_enabled(false);
        res?;
        if pec_error {
            Err(Error::Bus)
        } else {
            Ok(())
        }
    }

    /// SMBus master read with PEC (RM 11.4.6): CNT = len+1; the received
    /// Nth byte is the PEC and is auto-NACKed and checked by the hardware
    /// (PECERR on mismatch). len + 1 must not exceed 255.
    pub fn blocking_smbus_read_pec(&mut self, addr: u8, buf: &mut [u8]) -> Result<(), Error> {
        if buf.is_empty() || buf.len() + 1 > 255 {
            return Err(Error::Overrun);
        }
        SlaveAddr::Addr7(addr).validate()?;
        self.set_pec_enabled(true);
        let r = T::regs();
        let res = (|| {
            wait_bus_free::<T>(self.spin)?;
            program_chunk::<T>(
                SlaveAddr::Addr7(addr),
                true,
                buf.len() + 1,
                true,
                true,
                true,
            );
            for b in buf.iter_mut() {
                wait_flags::<T>(STS_RDBF, self.spin)?;
                *b = r.rxdt().read().bits() as u8;
            }
            // The final byte is the PEC: read it out (hardware already
            // compared it and auto-NACKed it).
            wait_flags::<T>(STS_RDBF, self.spin)?;
            let _ = r.rxdt().read().bits();
            wait_stop::<T>(self.spin)
        })();
        if let Err(error) = res {
            recover::<T>(Some(error), self.spin);
        } else {
            // I2CEN = 0 resets PECVAL, so preserve the result before disabling PEC.
            self.last_pec_value = self.pec_value();
        }
        let pec_error = res.is_ok() && self.pec_error();
        self.set_pec_enabled(false);
        res?;
        if pec_error {
            Err(Error::Bus)
        } else {
            Ok(())
        }
    }

    /// Enables/disables the PEC calculation unit (CTRL1.PECEN; toggled with
    /// the controller briefly disabled, as the bit requires I2CEN = 0).
    pub fn set_pec_enabled(&mut self, en: bool) {
        let r = T::regs();
        r.ctrl1().modify(|_, w| w.i2cen().disable());
        r.ctrl1().modify(|_, w| {
            if en {
                w.pecen().set_bit();
            } else {
                w.pecen().clear_bit();
            }
            w
        });
        r.ctrl1().modify(|_, w| w.i2cen().enable());
    }

    /// PEC calculation value (PEC register).
    pub fn pec_value(&self) -> u8 {
        T::regs().pec().read().pecval().bits()
    }

    /// PEC value captured after the most recent completed SMBus transfer.
    pub fn last_pec_value(&self) -> u8 {
        self.last_pec_value
    }

    /// PEC receive error flag (STS.PECERR; cleared by this call).
    pub fn pec_error(&self) -> bool {
        let r = T::regs();
        let err = r.sts().read().bits() & STS_PECERR != 0;
        if err {
            r.clr().write(|w| w.pecerrc().set_bit());
        }
        err
    }

    /// Configures the SMBus timeout detector (RM 11.7.6 I2C_TIMEOUT):
    /// `to_ticks` = TOTIME (timeout = (TOTIME+1) x 2048 x Ti2c in low mode /
    /// (TOTIME+1) x 4 x Ti2c in high mode), `detect_high` selects the high
    /// level mode (TOMODE), `ext_ticks` = EXTTIME for the cumulative
    /// clock-low-extend timeout (0 disables EXTEN).
    pub fn configure_smbus_timeout(&mut self, to_ticks: u16, detect_high: bool, ext_ticks: u16) {
        T::regs().timeout().write(|w| unsafe {
            w.totime().bits(to_ticks & 0x0FFF);
            if detect_high {
                w.tomoed().set_bit();
            }
            w.toen().set_bit();
            w.exttime().bits(ext_ticks);
            if ext_ticks > 0 {
                w.exten().set_bit();
            }
            w
        });
    }

    /// Probes a 7-bit address with an address-only write. Used for bus scans; also exercises START/STOP
    /// generation on hardware.
    pub fn blocking_probe(&mut self, addr: u8) -> Result<bool, Error> {
        match self.blocking_write(addr, &[]) {
            Ok(()) => Ok(true),
            Err(Error::NoAcknowledge) => Ok(false),
            Err(e) => Err(e),
        }
    }
}

impl<T: Instance> embedded_hal::i2c::ErrorType for I2c<'_, T> {
    type Error = Error;
}

impl<T: Instance> embedded_hal::i2c::I2c for I2c<'_, T> {
    fn read(&mut self, address: u8, read: &mut [u8]) -> Result<(), Self::Error> {
        I2c::blocking_read(self, address, read)
    }

    fn write(&mut self, address: u8, write: &[u8]) -> Result<(), Self::Error> {
        I2c::blocking_write(self, address, write)
    }

    fn write_read(
        &mut self,
        address: u8,
        write: &[u8],
        read: &mut [u8],
    ) -> Result<(), Self::Error> {
        I2c::blocking_write_read(self, address, write, read)
    }

    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        blocking_transaction::<T>(SlaveAddr::Addr7(address), operations, self.spin)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn address_field_encodes_both_bus_address_sizes() {
        let addr7 = SlaveAddr::Addr7(0x50);
        let addr10 = SlaveAddr::Addr10(0x350);
        assert_eq!(addr7.validate(), Ok(()));
        assert_eq!(addr7.saddr(), 0xa0);
        assert_eq!(addr10.validate(), Ok(()));
        assert_eq!(addr10.saddr(), 0x350);
        assert_eq!(
            SlaveAddr::Addr7(0x80).validate(),
            Err(Error::InvalidAddress)
        );
        assert_eq!(
            SlaveAddr::Addr10(0x400).validate(),
            Err(Error::InvalidAddress)
        );
    }
}
