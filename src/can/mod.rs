//! Blocking and interrupt-driven CAN 2.0B for CAN1/CAN2 (RM 20).
//!
//! The AT32F435 CAN is a bxCAN-style controller: 3 transmit mailboxes,
//! 2 receive FIFOs and 28 identifier filter banks per controller (RM 20.6.4;
//! the Artery library exposes only banks 0..13 but the hardware has 0..27).
//!
//! Bit timing: bit time = 1 (SYNC) + BTS1 + BTS2 time quanta with
//! TQ = BRDIV / PCLK1. Use [`BitTiming::from_baud`] to compute segments from
//! a target baud rate (sample point near 87.5%), or fill [`BitTiming`]
//! manually for exotic settings.
//!
//! Filters take pre-formatted 32-bit register words (RM 20.6.4 tables):
//! 32-bit: `FFB1[31:21]` = `SID[10:0]` or `EID[28:18]`, `FFB1[20:3]` = `EID[17:0]`,
//! `FFB1[2]` = IDE, `FFB1[1]` = RTR; FFB2 holds the mask (mask mode) or a second
//! identifier (list mode). The [`filter_word`] helpers build those words.
//!
//! ```ignore
//! let mut can = hal::can::Can::new(
//!     p.CAN1, p.PA12, p.PA11,
//!     hal::can::Config::baud(hal::rcc::clocks().pclk1, 500_000),
//! );
//! can.add_filter(hal::can::FilterEntry::std_mask32(0, hal::can::Fifo::Fifo0, 0x123, 0x7FF));
//! can.transmit(&frame, None)?;
//! let frame = can.receive(hal::can::Fifo::Fifo0, None)?;
//! ```

mod async_driver;
pub use async_driver::{CanAsync, Rx0InterruptHandler, Rx1InterruptHandler, TxInterruptHandler};

use async_driver::AsyncState;

use embassy_hal_internal::Peri;

use crate::gpio::{AfType, OutputType, Pull, Speed};
use crate::interrupt;
use crate::pac;
use crate::peripheral::RccPeripheral;
use crate::time::Hertz;

/// CAN operation error.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[non_exhaustive]
pub enum Error {
    /// A flag wait timed out (controller not reacting, e.g. bus stuck).
    Timeout,
    /// All three transmit mailboxes stayed busy for the whole wait.
    NoEmptyMailbox,
    /// The frame was not transmitted (arbitration lost or bus error).
    TxFailed,
    /// The baud target is not reachable with the legal segment ranges.
    BadTiming,
}

#[cfg(feature = "defmt")]
impl defmt::Format for Error {
    fn format(&self, fmt: defmt::Formatter) {
        match self {
            Error::Timeout => defmt::write!(fmt, "Timeout"),
            Error::NoEmptyMailbox => defmt::write!(fmt, "NoEmptyMailbox"),
            Error::TxFailed => defmt::write!(fmt, "TxFailed"),
            Error::BadTiming => defmt::write!(fmt, "BadTiming"),
        }
    }
}

/// Identifier.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Id {
    /// 11-bit standard identifier.
    Standard(u16),
    /// 29-bit extended identifier.
    Extended(u32),
}

/// Receive FIFO selector.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Fifo {
    Fifo0,
    Fifo1,
}

/// A CAN frame (data or remote, classic 2.0B payload).
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct Frame {
    pub id: Id,
    /// True for a remote transmission request (no data payload).
    pub remote: bool,
    /// Payload length 0..=8 (data length code of the frame).
    pub len: usize,
    /// Payload bytes (only `len` are valid).
    pub data: [u8; 8],
}

impl Frame {
    /// New data frame with `len` payload bytes (len is clamped to 0..=8).
    pub fn new(id: Id, data: &[u8]) -> Self {
        let len = data.len().min(8);
        let mut buf = [0u8; 8];
        buf[..len].copy_from_slice(&data[..len]);
        Self {
            id,
            remote: false,
            len,
            data: buf,
        }
    }

    /// New remote frame requesting `len` bytes.
    pub fn new_remote(id: Id, len: usize) -> Self {
        Self {
            id,
            remote: true,
            len: len.min(8),
            data: [0u8; 8],
        }
    }
}

/// Operating mode (BTMG.LBEN / BTMG.LOEN test modes).
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
pub enum Mode {
    /// Normal bus communication.
    #[default]
    Normal,
    /// Loopback: TX feeds RX internally, the pin still drives the bus.
    Loopback,
    /// Silent (listen-only): the controller never drives dominant bits.
    Silent,
    /// Silent loopback: fully disconnected from the bus, TX->RX internal.
    SilentLoopback,
}

/// Bit timing segments (time-quantum counts, each >= 1).
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct BitTiming {
    /// Bit-rate prescaler BRDIV (1..=4096).
    pub prescaler: u16,
    /// Resynchronization adjust width RSAW (1..=4 TQ).
    pub sync_jump: u8,
    /// Bit segment 1 before the sample point (1..=16 TQ, includes the
    /// propagation segment).
    pub seg1: u8,
    /// Bit segment 2 after the sample point (1..=8 TQ).
    pub seg2: u8,
}

impl BitTiming {
    /// Searches prescaler/segments for the closest bit rate with the sample
    /// point near 87.5% (CAN recommendation). `pclk` is the CAN kernel clock
    /// (PCLK1); `baud` the target bit rate.
    pub fn from_baud(pclk: Hertz, baud: u32) -> Result<Self, Error> {
        if baud == 0 {
            return Err(Error::BadTiming);
        }
        let mut best: Option<(BitTiming, u32, i32)> = None;
        for div in 1..=4096u32 {
            let tq_total = (pclk.0 + baud / 2) / baud / div;
            if !(4..=25).contains(&tq_total) {
                continue;
            }
            // seg2 grows from 1 to keep the sample point late; seg1 = rest.
            for seg2 in 1..=8u32 {
                let seg1 = tq_total - 1 - seg2;
                if !(1..=16).contains(&seg1) {
                    continue;
                }
                let actual = pclk.0 / (div * tq_total);
                let err = actual.abs_diff(baud);
                // Sample point = (1 + seg1) / total, target 87.5%.
                let sp = (1000 * (1 + seg1)) / tq_total;
                let sp_err = (sp as i32 - 875).abs();
                let score = err as i64 * 4 + sp_err as i64;
                let better = match best {
                    None => true,
                    Some((_, _, s)) => (score as i32) < s,
                };
                if better {
                    best = Some((
                        Self {
                            prescaler: div as u16,
                            sync_jump: 2,
                            seg1: seg1 as u8,
                            seg2: seg2 as u8,
                        },
                        actual,
                        score as i32,
                    ));
                }
            }
        }
        best.map(|(t, _, _)| t).ok_or(Error::BadTiming)
    }

    const fn tq_total(&self) -> u32 {
        1 + self.seg1 as u32 + self.seg2 as u32
    }

    /// Effective bit rate for a kernel clock `pclk`.
    pub const fn baud(&self, pclk: Hertz) -> u32 {
        pclk.0 / (self.prescaler as u32 * self.tq_total())
    }
}

/// Controller configuration.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct Config {
    pub mode: Mode,
    pub timing: BitTiming,
    /// Automatic exit from bus-off after 129 idle bit times (MCTRL.AEBOEN).
    pub auto_bus_off: bool,
    /// Prohibit retransmission of a failed frame (MCTRL.PRSFEN).
    pub no_retransmit: bool,
}

impl Config {
    /// Default config for `baud` on kernel clock `pclk`.
    pub fn baud(pclk: Hertz, baud: u32) -> Result<Self, Error> {
        Ok(Self {
            mode: Mode::Normal,
            timing: BitTiming::from_baud(pclk, baud)?,
            auto_bus_off: false,
            no_retransmit: false,
        })
    }
}

/// Filter identifier word/mask formatter (RM 20.6.4 32-bit layout).
pub mod filter_word {
    /// 32-bit filter word for a standard identifier (IDE = 0).
    pub const fn std(id: u16, remote: bool) -> u32 {
        ((id as u32 & 0x7FF) << 21) | ((remote as u32) << 1)
    }

    /// 32-bit filter word for an extended identifier (IDE = 1).
    pub const fn ext(id: u32, remote: bool) -> u32 {
        (((id >> 18) & 0x7FF) << 21) | ((id & 0x3_FFFF) << 3) | (1 << 2) | ((remote as u32) << 1)
    }
}

/// Filter mode.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
pub enum FilterMode {
    /// FFB2 is a mask over FFB1 (accept ID ranges).
    #[default]
    Mask,
    /// FFB1 and FFB2 are two identifiers that must match exactly.
    List,
}

/// One filter bank configuration (RM 20.6.4; banks 0..27 per controller).
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct FilterEntry {
    /// Filter bank number 0..=27.
    pub bank: u8,
    /// Receive FIFO that passing frames land in.
    pub fifo: Fifo,
    pub mode: FilterMode,
    /// Activate the bank after writing (FACFG bit).
    pub activate: bool,
    /// First 32-bit register value (identifier in mask mode).
    pub ffb1: u32,
    /// Second 32-bit register value (mask in mask mode; second ID in list).
    pub ffb2: u32,
}

impl FilterEntry {
    /// Accept-all filter for standard frames on `bank` (32-bit mask, all
    /// bits masked off).
    pub fn accept_all_std(bank: u8, fifo: Fifo) -> Self {
        Self {
            bank,
            fifo,
            mode: FilterMode::Mask,
            activate: true,
            ffb1: 0,
            ffb2: 0,
        }
    }

    /// 32-bit mask filter for one standard identifier (`mask` zeroes the
    /// don't-care bits, e.g. 0x7FF for an exact match of `id`).
    pub fn std_mask32(bank: u8, fifo: Fifo, id: u16, mask: u16) -> Self {
        Self {
            bank,
            fifo,
            mode: FilterMode::Mask,
            activate: true,
            ffb1: filter_word::std(id, false),
            ffb2: filter_word::std(mask, false),
        }
    }

    /// 32-bit mask filter for one extended identifier.
    pub fn ext_mask32(bank: u8, fifo: Fifo, id: u32, mask: u32) -> Self {
        Self {
            bank,
            fifo,
            mode: FilterMode::Mask,
            activate: true,
            ffb1: filter_word::ext(id, false),
            ffb2: filter_word::ext(mask, false),
        }
    }
}

trait SealedInstance {
    fn regs() -> &'static pac::can1::RegisterBlock;
    fn async_state() -> &'static AsyncState;
}

/// CAN1/CAN2 instance trait.
#[allow(private_bounds)]
pub trait Instance:
    SealedInstance + RccPeripheral + embassy_hal_internal::PeripheralType + 'static
{
    /// This instance's TX-complete interrupt (typelevel).
    type TxInterrupt: interrupt::typelevel::Interrupt;
    /// This instance's RX-FIFO0 interrupt (typelevel).
    type Rx0Interrupt: interrupt::typelevel::Interrupt;
    /// This instance's RX-FIFO1 interrupt (typelevel).
    type Rx1Interrupt: interrupt::typelevel::Interrupt;
}

macro_rules! impl_can_instance {
    ($inst:ident, $tx:ident, $rx0:ident, $rx1:ident) => {
        impl SealedInstance for crate::peripherals::$inst {
            #[inline]
            fn regs() -> &'static pac::can1::RegisterBlock {
                unsafe { &*pac::$inst::PTR }
            }
            #[inline]
            fn async_state() -> &'static AsyncState {
                static STATE: AsyncState = AsyncState::new();
                &STATE
            }
        }
        impl Instance for crate::peripherals::$inst {
            type TxInterrupt = interrupt::typelevel::$tx;
            type Rx0Interrupt = interrupt::typelevel::$rx0;
            type Rx1Interrupt = interrupt::typelevel::$rx1;
        }
    };
}

// Note: the CAN1 RX1 / status-error vectors are named CAN_RX1 / CAN_SE in
// the PAC interrupt enum (shared naming); CAN2 has its own CAN2_RX1/CAN2_SE.
impl_can_instance!(CAN1, CAN1_TX, CAN1_RX0, CAN_RX1);
impl_can_instance!(CAN2, CAN2_TX, CAN2_RX0, CAN2_RX1);

crate::pin_trait!(TxPin, Instance);
crate::pin_trait!(RxPin, Instance);

// Verified against RM V2.09 tables 6-1/6-2/6-5/6-8 (page-image read,
// docs/af_table_notes.md): all CAN signals live on MUX 9.
crate::pin_trait_impl!(crate::can::TxPin, CAN1, PA12, 9); // [V] p115
crate::pin_trait_impl!(crate::can::TxPin, CAN1, PB9, 9); // [V] p117
crate::pin_trait_impl!(crate::can::TxPin, CAN1, PD1, 9); // [V] p121
crate::pin_trait_impl!(crate::can::TxPin, CAN1, PG1, 9); // [V] p127
crate::pin_trait_impl!(crate::can::RxPin, CAN1, PA11, 9); // [V] p115
crate::pin_trait_impl!(crate::can::RxPin, CAN1, PB8, 9); // [V] p117
crate::pin_trait_impl!(crate::can::RxPin, CAN1, PD0, 9); // [V] p121
crate::pin_trait_impl!(crate::can::RxPin, CAN1, PG0, 9); // [V] p127
crate::pin_trait_impl!(crate::can::TxPin, CAN2, PB6, 9); // [V] p117
crate::pin_trait_impl!(crate::can::TxPin, CAN2, PB13, 9); // [V] p117
crate::pin_trait_impl!(crate::can::TxPin, CAN2, PG12, 9); // [V] p127
crate::pin_trait_impl!(crate::can::RxPin, CAN2, PB5, 9); // [V] p117
crate::pin_trait_impl!(crate::can::RxPin, CAN2, PB12, 9); // [V] p117
crate::pin_trait_impl!(crate::can::RxPin, CAN2, PG11, 9); // [V] p127

// Loop-count bound for flag waits (~0.6 s at 288 MHz, same budget as i2c).
const DEFAULT_SPIN: u32 = 1 << 24;

const FZC_TIMEOUT: u32 = 0xFFFF;

/// Writes one frame into transmit mailbox `mailbox` (clearing stale
/// completion flags first) and requests transmission. Used by the blocking
/// and the async transmit paths.
fn write_mailbox(r: &pac::can1::RegisterBlock, mailbox: usize, frame: &Frame) {
    let mb = r.mailbox(mailbox);
    // Clear any stale completion flags (write-1-to-clear).
    r.tsts().modify(|_, w| {
        match mailbox {
            0 => {
                w.tm0tcf().clear();
                w.tm0tsf().clear();
            }
            1 => {
                w.tm1tcf().clear();
                w.tm1tsf().clear();
            }
            _ => {
                w.tm2tcf().clear();
                w.tm2tsf().clear();
            }
        }
        w
    });
    // Identifier + frame type + data/remote.
    mb.tmi().modify(|_, w| {
        match frame.id {
            Id::Standard(id) => unsafe {
                w.idsel().clear_bit();
                w.sid().bits(id & 0x7FF);
            },
            Id::Extended(id) => unsafe {
                w.idsel().set_bit();
                w.eid().bits(id & 0x3_FFFF);
                w.sid().bits(((id >> 18) & 0x7FF) as u16);
            },
        };
        if frame.remote {
            w.frsel().set_bit();
        } else {
            w.frsel().clear_bit();
        }
        w
    });
    mb.tmc()
        .modify(|_, w| unsafe { w.dtbl().bits(frame.len as u8) });
    mb.tmdtl().write(|w| unsafe {
        w.tmdt0().bits(frame.data[0]);
        w.tmdt1().bits(frame.data[1]);
        w.tmdt2().bits(frame.data[2]);
        w.tmdt3().bits(frame.data[3]);
        w
    });
    mb.tmdth().write(|w| unsafe {
        w.tmdt4().bits(frame.data[4]);
        w.tmdt5().bits(frame.data[5]);
        w.tmdt6().bits(frame.data[6]);
        w.tmdt7().bits(frame.data[7]);
        w
    });
    // Request transmission.
    mb.tmi().modify(|_, w| w.sr().transmit());
}

/// Reads one frame from receive FIFO `n` (must have MN > 0) and releases the
/// slot. Used by the blocking and the async receive paths.
fn read_fifo_frame(r: &pac::can1::RegisterBlock, n: usize) -> Frame {
    let f = r.fifo(n);
    let rfi = f.rfi().read();
    let id = if rfi.idi().bit_is_set() {
        let low = rfi.eid().bits();
        let high = rfi.sid().bits() as u32;
        Id::Extended((high << 18) | low)
    } else {
        Id::Standard(rfi.sid().bits())
    };
    let rfc = f.rfc().read();
    let len = (rfc.dtl().bits() as usize).min(8);
    let mut data = [0u8; 8];
    let dtl = f.rfdtl().read();
    data[0] = dtl.rfdt0().bits();
    data[1] = dtl.rfdt1().bits();
    data[2] = dtl.rfdt2().bits();
    data[3] = dtl.rfdt3().bits();
    let dth = f.rfdth().read();
    data[4] = dth.rfdt4().bits();
    data[5] = dth.rfdt5().bits();
    data[6] = dth.rfdt6().bits();
    data[7] = dth.rfdt7().bits();
    let remote = rfi.fri().bit_is_set();
    // Release the FIFO slot (write 1 to RFxR).
    r.rf(n).modify(|_, w| w.r().release());
    Frame {
        id,
        remote,
        len,
        data,
    }
}

/// Hardware-empty state of transmit mailbox `mailbox` (TMxEF).
fn mailbox_empty(tsts: &pac::can1::tsts::R, mailbox: usize) -> bool {
    match mailbox {
        0 => tsts.tm0ef().bit_is_set(),
        1 => tsts.tm1ef().bit_is_set(),
        _ => tsts.tm2ef().bit_is_set(),
    }
}

/// Blocking CAN handle.
pub struct Can<'d, T: Instance> {
    _peri: Peri<'d, T>,
    _tx: Peri<'d, crate::gpio::AnyPin>,
    _rx: Peri<'d, crate::gpio::AnyPin>,
    spin: u32,
}

impl<'d, T: Instance> Can<'d, T> {
    /// Creates a blocking CAN controller with the given TX/RX AF pins.
    pub fn new(
        peri: Peri<'d, T>,
        tx: Peri<'d, impl TxPin<T>>,
        rx: Peri<'d, impl RxPin<T>>,
        config: Config,
    ) -> Self {
        let tx_pin = new_pin!(
            tx,
            AfType::output(OutputType::PushPull, Speed::High.into(), Pull::None)
        );
        let rx_pin = new_pin!(rx, AfType::input(Pull::None));
        T::enable_and_reset();
        let can = Self {
            _peri: peri,
            _tx: tx_pin.unwrap(),
            _rx: rx_pin.unwrap(),
            spin: DEFAULT_SPIN,
        };
        can.configure(&config);
        can
    }

    /// Override the flag-wait loop budget (see [`crate::i2c::I2c`]).
    pub fn set_timeout_spins(&mut self, spin: u32) {
        self.spin = spin;
    }

    fn regs(&self) -> &'static pac::can1::RegisterBlock {
        T::regs()
    }

    /// Applies mode/timing/config (freeze -> BTMG/MCTRL -> unfreeze).
    pub fn configure(&self, config: &Config) {
        let r = self.regs();
        // Freeze for configuration (exit doze first).
        r.mctrl().modify(|_, w| w.dzen().clear_bit());
        r.mctrl().modify(|_, w| w.fzen().set_bit());
        let mut left = FZC_TIMEOUT;
        while r.msts().read().fzc().bit_is_clear() {
            left -= 1;
            if left == 0 {
                return;
            }
        }
        r.btmg().modify(|_, w| unsafe {
            w.brdiv().bits(config.timing.prescaler - 1);
            w.rsaw().bits(config.timing.sync_jump - 1);
            w.bts1().bits(config.timing.seg1 - 1);
            w.bts2().bits(config.timing.seg2 - 1);
            w.lben().clear_bit();
            w.loen().clear_bit();
            w
        });
        let (lben, loen) = match config.mode {
            Mode::Normal => (false, false),
            Mode::Loopback => (true, false),
            Mode::Silent => (false, true),
            Mode::SilentLoopback => (true, true),
        };
        r.btmg().modify(|_, w| {
            if lben {
                w.lben().set_bit();
            }
            if loen {
                w.loen().set_bit();
            }
            w
        });
        r.mctrl().modify(|_, w| {
            w.ttcen().clear_bit();
            if config.auto_bus_off {
                w.aeboen().set_bit();
            }
            w.aeden().clear_bit();
            if config.no_retransmit {
                w.prsfen().set_bit();
            }
            w.mdrsel().clear_bit();
            w.mmssr().clear_bit();
            w
        });
        // Leave freeze (normal communication).
        r.mctrl().modify(|_, w| w.fzen().clear_bit());
        left = FZC_TIMEOUT;
        while r.msts().read().fzc().bit_is_set() {
            left -= 1;
            if left == 0 {
                break;
            }
        }
    }

    /// Configures one filter bank (must be called while frames are not being
    /// received; the hardware briefs the filter into config mode itself).
    pub fn add_filter(&self, entry: &FilterEntry) {
        assert!(entry.bank < 28, "filter bank 0..27");
        let r = self.regs();
        let bit = 1u32 << entry.bank;
        // Enter filter configuration mode.
        r.fctrl().modify(|_, w| w.fcs().configuration());
        // Deactivate while modifying.
        r.facfg().modify(|r, w| unsafe { w.bits(r.bits() & !bit) });
        match entry.mode {
            FilterMode::Mask => {
                r.fmcfg().modify(|r, w| unsafe { w.bits(r.bits() & !bit) });
            }
            FilterMode::List => {
                r.fmcfg().modify(|r, w| unsafe { w.bits(r.bits() | bit) });
            }
        }
        // 32-bit scale only (the 16-bit packing is a subset of the same
        // registers; use ffb1/ffb2 fields for custom 16-bit words).
        r.fbwcfg().modify(|r, w| unsafe { w.bits(r.bits() | bit) });
        let bank = r.filter_bank(entry.bank as usize);
        bank.ffb(0).write(|w| unsafe { w.bits(entry.ffb1) });
        bank.ffb(1).write(|w| unsafe { w.bits(entry.ffb2) });
        match entry.fifo {
            Fifo::Fifo0 => {
                r.frf().modify(|r, w| unsafe { w.bits(r.bits() & !bit) });
            }
            Fifo::Fifo1 => {
                r.frf().modify(|r, w| unsafe { w.bits(r.bits() | bit) });
            }
        }
        if entry.activate {
            r.facfg().modify(|r, w| unsafe { w.bits(r.bits() | bit) });
        }
        // Back to working condition.
        r.fctrl().modify(|_, w| w.fcs().active());
    }

    /// Queues a frame for transmission. Waits up to the timeout budget for a
    /// free mailbox; returns `NoEmptyMailbox` if all three stay busy.
    pub fn transmit(&self, frame: &Frame, timeout_spins: Option<u32>) -> Result<(), Error> {
        let r = self.regs();
        let budget = timeout_spins.unwrap_or(self.spin);
        // Pick an empty mailbox (TM0EF..TM2EF: 1 = empty).
        let mut wait = budget;
        let mailbox = loop {
            let tsts = r.tsts().read();
            if tsts.tm0ef().bit_is_set() {
                break 0usize;
            }
            if tsts.tm1ef().bit_is_set() {
                break 1;
            }
            if tsts.tm2ef().bit_is_set() {
                break 2;
            }
            if wait == 0 {
                return Err(Error::NoEmptyMailbox);
            }
            wait -= 1;
        };
        write_mailbox(r, mailbox, frame);
        // Wait for completion (TMxTCF) and read the result (TMxTSF set =
        // success per the Artery status semantics).
        let mut left = budget;
        loop {
            let tsts = r.tsts().read();
            let (tcf, tsf) = match mailbox {
                0 => (tsts.tm0tcf().bit_is_set(), tsts.tm0tsf().bit_is_set()),
                1 => (tsts.tm1tcf().bit_is_set(), tsts.tm1tsf().bit_is_set()),
                _ => (tsts.tm2tcf().bit_is_set(), tsts.tm2tsf().bit_is_set()),
            };
            if tcf {
                r.tsts().modify(|_, w| {
                    match mailbox {
                        0 => {
                            w.tm0tcf().clear();
                            w.tm0tsf().clear();
                            w.tm0alf().clear();
                            w.tm0tef().clear();
                        }
                        1 => {
                            w.tm1tcf().clear();
                            w.tm1tsf().clear();
                            w.tm1alf().clear();
                            w.tm1tef().clear();
                        }
                        _ => {
                            w.tm2tcf().clear();
                            w.tm2tsf().clear();
                            w.tm2alf().clear();
                            w.tm2tef().clear();
                        }
                    }
                    w
                });
                return if tsf { Ok(()) } else { Err(Error::TxFailed) };
            }
            if left == 0 {
                // Cancel the request.
                r.tsts().modify(|_, w| {
                    match mailbox {
                        0 => {
                            w.tm0ct().set_bit();
                        }
                        1 => {
                            w.tm1ct().set_bit();
                        }
                        _ => {
                            w.tm2ct().set_bit();
                        }
                    }
                    w
                });
                return Err(Error::Timeout);
            }
            left -= 1;
        }
    }

    /// Receives one pending frame from `fifo`. Blocks until a frame arrives
    /// (or `timeout_spins` elapses).
    pub fn receive(&self, fifo: Fifo, timeout_spins: Option<u32>) -> Result<Frame, Error> {
        let r = self.regs();
        let n = match fifo {
            Fifo::Fifo0 => 0usize,
            Fifo::Fifo1 => 1,
        };
        let mut left = timeout_spins.unwrap_or(self.spin);
        while r.rf(n).read().mn().bits() == 0 {
            if left == 0 {
                return Err(Error::Timeout);
            }
            left -= 1;
        }
        Ok(read_fifo_frame(r, n))
    }

    /// Number of pending frames in `fifo`.
    pub fn pending(&self, fifo: Fifo) -> u8 {
        self.regs()
            .rf(match fifo {
                Fifo::Fifo0 => 0,
                Fifo::Fifo1 => 1,
            })
            .read()
            .mn()
            .bits()
    }

    /// Enters doze (low-power) mode; returns false if the controller did not
    /// acknowledge in time.
    pub fn enter_doze(&self) -> bool {
        let r = self.regs();
        r.mctrl().modify(|_, w| w.fzen().clear_bit());
        r.mctrl().modify(|_, w| w.dzen().set_bit());
        let mut left = FZC_TIMEOUT;
        while r.msts().read().dzc().bit_is_clear() {
            left -= 1;
            if left == 0 {
                return false;
            }
        }
        true
    }

    /// Leaves doze mode (synchronous wake-up, needs bus activity unless
    /// `auto_wakeup` was configured).
    pub fn exit_doze(&self) -> bool {
        let r = self.regs();
        r.mctrl().modify(|_, w| w.dzen().clear_bit());
        let mut left = FZC_TIMEOUT;
        while r.msts().read().dzc().bit_is_set() {
            left -= 1;
            if left == 0 {
                return false;
            }
        }
        true
    }

    /// Transmit/receive error counters (ESTS.TEC / ESTS.REC).
    pub fn error_counters(&self) -> (u8, u8) {
        let ests = self.regs().ests().read();
        (ests.tec().bits(), ests.rec().bits())
    }

    /// Bus-off flag (ESTS.BOF).
    pub fn bus_off(&self) -> bool {
        self.regs().ests().read().bof().bit_is_set()
    }
}
