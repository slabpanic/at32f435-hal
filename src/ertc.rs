//! ERTC real-time clock (RM 17): calendar with alarm A/B, the periodic wakeup
//! timer, timestamp and tamper detection, and the 20 backup registers (BPR)
//! on the battery-backed domain.
//!
//! Clock: LEXT (32.768 kHz on the evb) / LICK / divided HEXT, selected by
//! CRM_BPDC.ERTCSEL. With DIVA = 127, DIVB = 255 the 1 Hz calendar tick
//! (ck_b) comes out of the 32.768 kHz LEXT.
//!
//! Write protection: every configuration register is protected by ERTC_WP
//! (0xCA, 0x53 unlock sequence) behind PWC_CTRL.BPWEN - the driver handles
//! both. Config changes (calendar, DIV) additionally require init mode
//! (STS.IMEN); alarm/wakeup registers use their own ALAWF/ALBWF/WATWF
//! allow-write flags. STS flags are write-0-to-clear (rw0c).
//!
//! ```ignore
//! let mut ertc = hal::ertc::Ertc::new(p.ERTC, hal::ertc::ClockSource::Lext)?;
//! ertc.set_calendar(hal::ertc::Timestamp { year: 26, month: 11, day: 17,
//!     weekday: hal::ertc::Weekday::Tuesday, hour: 12, minute: 34, second: 56 })?;
//! let t = ertc.calendar();
//! ```

use core::sync::atomic::{AtomicBool, AtomicU8, Ordering};

use embassy_hal_internal::Peri;
use embassy_sync::waitqueue::AtomicWaker;

use crate::pac;
use crate::time::Hertz;

/// LEXT frequency on boards with a 32.768 kHz crystal.
pub const LEXT_FREQ: Hertz = Hertz(32_768);

static WAKER: AtomicWaker = AtomicWaker::new();
/// Set by the ERTC interrupt handler when any of the enabled events fires.
static EVENTS: AtomicU8 = AtomicU8::new(0);

const EV_ALARM_A: u8 = 1 << 0;
const EV_ALARM_B: u8 = 1 << 1;
const EV_WAKEUP: u8 = 1 << 2;
const EV_TIMESTAMP: u8 = 1 << 3;
const EV_TAMPER: u8 = 1 << 4;

/// ERTC interrupt events (the family shares one vector for all ERTC sources).
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[non_exhaustive]
pub enum Event {
    AlarmA,
    AlarmB,
    Wakeup,
    Timestamp,
    Tamper,
}

impl Event {
    const fn bit(self) -> u8 {
        match self {
            Event::AlarmA => EV_ALARM_A,
            Event::AlarmB => EV_ALARM_B,
            Event::Wakeup => EV_WAKEUP,
            Event::Timestamp => EV_TIMESTAMP,
            Event::Tamper => EV_TAMPER,
        }
    }
}

/// ERTC clock source (CRM_BPDC.ERTCSEL).
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum ClockSource {
    /// LEXT 32.768 kHz crystal (recommended).
    Lext,
    /// LICK ~40 kHz internal RC (coarse).
    Lick,
    /// HEXT divided by `div` (BPDC.ERTCSEL = 0b11 with CRM_CFG.ERTCDIV =
    /// `div`, divisor 2..=32, f_ertc <= 1 MHz). `hext_hz` is the crystal
    /// frequency in Hz; the calendar dividers are derived from it.
    Hext { div: u8, hext_hz: u32 },
}

/// Day of week (DATE.WK field, 1 = Monday .. 7 = Sunday per the RM).
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
pub enum Weekday {
    #[default]
    Monday,
    Tuesday,
    Wednesday,
    Thursday,
    Friday,
    Saturday,
    Sunday,
}

impl Weekday {
    const fn code(self) -> u8 {
        match self {
            Weekday::Monday => 1,
            Weekday::Tuesday => 2,
            Weekday::Wednesday => 3,
            Weekday::Thursday => 4,
            Weekday::Friday => 5,
            Weekday::Saturday => 6,
            Weekday::Sunday => 7,
        }
    }

    const fn from_code(v: u8) -> Self {
        match v {
            1 => Weekday::Monday,
            2 => Weekday::Tuesday,
            3 => Weekday::Wednesday,
            4 => Weekday::Thursday,
            5 => Weekday::Friday,
            6 => Weekday::Saturday,
            _ => Weekday::Sunday,
        }
    }
}

/// A calendar timestamp (BCD decoded by the hardware, decimal here).
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
pub struct Timestamp {
    /// Year (0..=99, offset from the century base chosen by the application;
    /// the hardware stores 2 digits only).
    pub year: u8,
    /// Month 1..=12.
    pub month: u8,
    /// Day of month 1..=31.
    pub day: u8,
    pub weekday: Weekday,
    /// Hours 0..=23 (24-hour mode).
    pub hour: u8,
    pub minute: u8,
    pub second: u8,
}

/// Alarm match fields (ERTC_ALA/ALB MASKx bits: masked fields are ignored).
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
pub struct AlarmMask {
    /// Ignore the day-of-month / weekday field (MASK4).
    pub day: bool,
    /// Ignore the hours field (MASK3).
    pub hours: bool,
    /// Ignore the minutes field (MASK2).
    pub minutes: bool,
    /// Ignore the seconds field (MASK1).
    pub seconds: bool,
}

/// Alarm selector.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Alarm {
    A,
    B,
}

/// Tamper pin active edge (TAMP.TP1EDG).
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
pub enum TamperEdge {
    /// Falling edge on the tamper pin.
    #[default]
    Falling,
    /// Rising edge on the tamper pin.
    Rising,
}

/// Smooth calibration window period (SCAL.CAL16/CAL8).
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
pub enum SmoothCalPeriod {
    /// 32-second calibration window (default; 512-pulse steps).
    #[default]
    Sec32,
    /// 16-second window (1024-pulse steps, 2x resolution).
    Sec16,
    /// 8-second window (2048-pulse steps, 4x resolution).
    Sec8,
}

/// Wakeup timer clock source (CTRL.WATCLK).
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum WakeupClock {
    /// ERTC_CLK / 16.
    ErtcClkDiv16,
    /// ERTC_CLK / 8.
    ErtcClkDiv8,
    /// ERTC_CLK / 4.
    ErtcClkDiv4,
    /// ERTC_CLK / 2.
    ErtcClkDiv2,
    /// ck_b (usually 1 Hz), 16-bit counter.
    CkB,
    /// ck_b with the 17-bit counter extension (VAL + 2^16).
    CkB16bit,
}

impl WakeupClock {
    const fn code(self) -> u8 {
        match self {
            WakeupClock::ErtcClkDiv16 => 0b000,
            WakeupClock::ErtcClkDiv8 => 0b001,
            WakeupClock::ErtcClkDiv4 => 0b010,
            WakeupClock::ErtcClkDiv2 => 0b011,
            WakeupClock::CkB => 0b100,
            WakeupClock::CkB16bit => 0b110,
        }
    }
}

/// Calendar/alarm configuration error.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[non_exhaustive]
pub enum ErtcError {
    /// A configuration sync timed out (calendar not ready).
    Timeout,
    /// A field is out of the calendar range.
    OutOfRange,
    /// The requested divider configuration cannot reach 1 Hz.
    BadDivider,
}

#[cfg(feature = "defmt")]
impl defmt::Format for ErtcError {
    fn format(&self, fmt: defmt::Formatter) {
        match self {
            ErtcError::Timeout => defmt::write!(fmt, "Timeout"),
            ErtcError::OutOfRange => defmt::write!(fmt, "OutOfRange"),
            ErtcError::BadDivider => defmt::write!(fmt, "BadDivider"),
        }
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for Event {
    fn format(&self, fmt: defmt::Formatter) {
        match self {
            Event::AlarmA => defmt::write!(fmt, "AlarmA"),
            Event::AlarmB => defmt::write!(fmt, "AlarmB"),
            Event::Wakeup => defmt::write!(fmt, "Wakeup"),
            Event::Timestamp => defmt::write!(fmt, "Timestamp"),
            Event::Tamper => defmt::write!(fmt, "Tamper"),
        }
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for Weekday {
    fn format(&self, fmt: defmt::Formatter) {
        match self {
            Weekday::Monday => defmt::write!(fmt, "Mon"),
            Weekday::Tuesday => defmt::write!(fmt, "Tue"),
            Weekday::Wednesday => defmt::write!(fmt, "Wed"),
            Weekday::Thursday => defmt::write!(fmt, "Thu"),
            Weekday::Friday => defmt::write!(fmt, "Fri"),
            Weekday::Saturday => defmt::write!(fmt, "Sat"),
            Weekday::Sunday => defmt::write!(fmt, "Sun"),
        }
    }
}

static INITIALIZED: AtomicBool = AtomicBool::new(false);

fn ertc() -> &'static pac::ertc::RegisterBlock {
    unsafe { &*pac::ERTC::PTR }
}

/// Unlocks the ERTC write protection (PWC BPWEN + ERTC_WP 0xCA/0x53).
fn unlock() {
    crate::rcc::enable_pwc_clock();
    let pwc = unsafe { &*pac::PWC::PTR };
    critical_section::with(|_| pwc.ctrl().modify(|_, w| w.bpwen().set_bit()));
    // WP key sequence; a wrong key re-activates the protection. RM 17.4
    // requires 32-bit-word accesses to ERTC registers - the PAC models WP
    // as u8, whose svd2rust accessors emit byte stores the hardware ignores,
    // so write through a word pointer instead.
    let wp = ertc().wp() as *const _ as *mut u32;
    unsafe {
        wp.write_volatile(0xCA);
        wp.write_volatile(0x53);
    }
}

fn lock() {
    // Any wrong key re-activates the protection; write a non-key value.
    let wp = ertc().wp() as *const _ as *mut u32;
    unsafe { wp.write_volatile(0x00) };
}

/// Arms the internal EXINT line that routes an ERTC event to the NVIC
/// (RM 17.3.9: alarm A/B = line 17, timestamp/tamper = line 21, periodic
/// wakeup = line 22; rising edge, interrupt mode). Without this the ERTC
/// interrupt can wake sleep mode but never deepsleep/standby.
fn arm_exint_line(n: u8) {
    critical_section::with(|_| {
        let ex = unsafe { &*pac::EXINT::PTR };
        // Rising edge: polcfg1 = rising (rp), polcfg2 = falling (fp).
        ex.polcfg1().modify(|_, w| w.rp(n).enable());
        ex.polcfg2().modify(|_, w| w.fp(n).disable());
        // Clear any leftover pending (write-1-to-clear), then enable the line.
        ex.intsts().write(|w| w.line(n).clear());
        ex.inten()
            .modify(|r, w| unsafe { w.bits(r.bits() | (1 << n)) });
    });
}

/// BCD helpers (tens digit, units digit) for the split decimal fields.
const fn tens(v: u8) -> u8 {
    v / 10
}
const fn units(v: u8) -> u8 {
    v % 10
}

// Spin budgets (register-read loops, ~15 cycles/iter at 288 MHz):
// - LEXT crystal start-up can take seconds (typ 0.25-2 s)
// - leaving init mode: UPDF sets up to 2 ck_b cycles later (= 2 s at 1 Hz)
// - other sync flags take 2-3 ERTC clock cycles (fast, but keep margin)
const LEXT_READY_TIMEOUT: u32 = 1 << 27;
const UPDF_TIMEOUT: u32 = 1 << 27;
const SYNC_TIMEOUT: u32 = 1 << 20;

/// Real-time clock handle.
pub struct Ertc {
    _peri: Peri<'static, crate::peripherals::ERTC>,
}

impl Ertc {
    /// Enables the battery-domain interface and initializes the ERTC clock
    /// (source + DIVA/DIVB) and calendar. If the calendar is already running
    /// (STS.IMEN clear, i.e. the battery domain was kept alive by VBAT), the
    /// clock configuration is skipped and the running time is preserved.
    ///
    /// The default divider targets 1 Hz ck_b from LEXT (DIVA = 127, DIVB =
    /// 255); the LICK and HEXT paths pick dividers from their nominal
    /// frequencies.
    pub fn new(
        peri: Peri<'static, crate::peripherals::ERTC>,
        source: ClockSource,
    ) -> Result<Self, ErtcError> {
        unlock();

        let crm = unsafe { &*pac::CRM::PTR };
        match source {
            ClockSource::Lext => {
                // Enable LEXT (CRM_BPDC) and wait ready.
                crm.bpdc().modify(|_, w| w.lexten().set_bit());
                let mut left = LEXT_READY_TIMEOUT;
                while crm.bpdc().read().lextstbl().bit_is_clear() {
                    left -= 1;
                    if left == 0 {
                        return Err(ErtcError::Timeout);
                    }
                }
                // ERTCSEL: 0b00 = NOCLOCK, 0b01 = LEXT, 0b10 = LICK,
                // 0b11 = HEXT/ERTCDIV (per crm_ertc_clock_select encoding).
                crm.bpdc().modify(|_, w| unsafe {
                    w.ertcsel().bits(0b01);
                    w.ertcen().set_bit();
                    w
                });
            }
            ClockSource::Lick => {
                // Enable LICK (CRM_CTRLSTS) and wait ready.
                crm.ctrlsts().modify(|_, w| w.licken().set_bit());
                let mut left = LEXT_READY_TIMEOUT;
                while crm.ctrlsts().read().lickstbl().bit_is_clear() {
                    left -= 1;
                    if left == 0 {
                        return Err(ErtcError::Timeout);
                    }
                }
                crm.bpdc().modify(|_, w| unsafe {
                    w.ertcsel().bits(0b10);
                    w.ertcen().set_bit();
                    w
                });
            }
            ClockSource::Hext { div, hext_hz } => {
                assert!((2..=32).contains(&div), "HEXT ERTC divisor must be 2..=32");
                crm.cfg().modify(|_, w| unsafe { w.ertcdiv().bits(div) });
                crm.bpdc().modify(|_, w| unsafe {
                    w.ertcsel().bits(0b11);
                    w.ertcen().set_bit();
                    w
                });
                let _ = hext_hz; // consumed below in the divider block
            }
        }

        let r = ertc();
        // Enter init mode (calendar stopped) only when needed: if the battery
        // domain kept the calendar running, IMEN is clear - preserve the
        // running time in that case.
        let already_running = r.sts().read().imen().bit_is_clear();
        if !already_running || !INITIALIZED.load(Ordering::Relaxed) {
            r.sts().modify(|_, w| w.imen().set_bit());
            let mut left = SYNC_TIMEOUT;
            // IMF (bit 6) = init mode entered; INITF (bit 4) means
            // "DATE year != 0" and stays clear on a fresh calendar.
            while r.sts().read().imf().bit_is_clear() {
                left -= 1;
                if left == 0 {
                    lock();
                    return Err(ErtcError::Timeout);
                }
            }
            // Dividers for 1 Hz ck_b: ck_b = f_ertc / ((DIVA+1) * (DIVB+1)).
            match source {
                ClockSource::Lext => r.div().modify(|_, w| unsafe {
                    w.diva().bits(127);
                    w.divb().bits(255);
                    w
                }),
                ClockSource::Lick => {
                    // ~40 kHz: 40000 / (126 * 317) ~ 1.0015 Hz.
                    r.div().modify(|_, w| unsafe {
                        w.diva().bits(125);
                        w.divb().bits(316);
                        w
                    })
                }
                ClockSource::Hext { div, hext_hz } => {
                    // f_ertc = HEXT/div; DIVA = 127, DIVB = f_ertc/128 - 1.
                    let f_ertc = hext_hz / div as u32;
                    let divb = (f_ertc / 128).saturating_sub(1);
                    if divb == 0 || divb > 32767 {
                        lock();
                        return Err(ErtcError::BadDivider);
                    }
                    r.div().modify(|_, w| unsafe {
                        w.diva().bits(127);
                        w.divb().bits(divb as u16);
                        w
                    })
                }
            };
            // 24-hour format, calendar reset to year 00, January, 1st, Monday.
            r.time().write(|w| unsafe {
                w.ht().bits(0);
                w.hu().bits(0);
                w.mt().bits(0);
                w.mu().bits(0);
                w.st().bits(0);
                w.su().bits(0);
                w
            });
            r.date().write(|w| unsafe {
                w.yt().bits(0);
                w.yu().bits(0);
                w.mt().clear_bit();
                w.mu().bits(1); // January
                w.dt().bits(0);
                w.du().bits(1); // 1st
                w.wk().bits(1); // Monday
                w
            });
            r.sts().modify(|_, w| w.imen().clear_bit());
            let mut left = UPDF_TIMEOUT;
            while r.sts().read().updf().bit_is_clear() {
                left -= 1;
                if left == 0 {
                    lock();
                    return Err(ErtcError::Timeout);
                }
            }
            INITIALIZED.store(true, Ordering::Relaxed);
        }
        lock();
        Ok(Self { _peri: peri })
    }

    /// Sets the calendar (also usable to correct a running clock: the driver
    /// handles init mode + write protection around it).
    pub fn set_calendar(&mut self, ts: Timestamp) -> Result<(), ErtcError> {
        if ts.month == 0
            || ts.month > 12
            || ts.day == 0
            || ts.day > 31
            || ts.hour > 23
            || ts.minute > 59
            || ts.second > 59
            || ts.year > 99
        {
            return Err(ErtcError::OutOfRange);
        }
        unlock();
        let r = ertc();
        r.sts().modify(|_, w| w.imen().set_bit());
        let mut left = SYNC_TIMEOUT;
        while r.sts().read().imf().bit_is_clear() {
            left -= 1;
            if left == 0 {
                lock();
                return Err(ErtcError::Timeout);
            }
        }
        let (ht, hu) = (tens(ts.hour), units(ts.hour));
        let (mt, mu) = (tens(ts.minute), units(ts.minute));
        let (st, su) = (tens(ts.second), units(ts.second));
        r.time().write(|w| unsafe {
            w.ht().bits(ht);
            w.hu().bits(hu);
            w.mt().bits(mt);
            w.mu().bits(mu);
            w.st().bits(st);
            w.su().bits(su);
            w
        });
        r.date().write(|w| unsafe {
            w.yt().bits(tens(ts.year));
            w.yu().bits(units(ts.year));
            if ts.month >= 10 {
                w.mt().set_bit();
            }
            w.mu().bits(units(ts.month));
            w.dt().bits(tens(ts.day));
            w.du().bits(units(ts.day));
            w.wk().bits(ts.weekday.code());
            w
        });
        r.sts().modify(|_, w| w.imen().clear_bit());
        let mut left = UPDF_TIMEOUT;
        while r.sts().read().updf().bit_is_clear() {
            left -= 1;
            if left == 0 {
                lock();
                return Err(ErtcError::Timeout);
            }
        }
        lock();
        Ok(())
    }

    /// Reads the current calendar (synchronized shadow registers; the final
    /// DATE read releases the shadow lock taken by the TIME read).
    pub fn calendar(&self) -> Timestamp {
        let r = ertc();
        let dt = r.date().read();
        let tm = r.time().read();
        let _ = r.date().read();
        Timestamp {
            year: from_bcd(dt.yt().bits(), dt.yu().bits()),
            month: from_bcd(u8::from(dt.mt().bit_is_set()), dt.mu().bits()),
            day: from_bcd(dt.dt().bits(), dt.du().bits()),
            weekday: Weekday::from_code(dt.wk().bits()),
            hour: from_bcd(tm.ht().bits(), tm.hu().bits()),
            minute: from_bcd(tm.mt().bits(), tm.mu().bits()),
            second: from_bcd(tm.st().bits(), tm.su().bits()),
        }
    }

    /// Sub-second register (free-running DIVB down-counter, ck_b ticks).
    pub fn subsecond(&self) -> u16 {
        let r = ertc();
        let v = r.sbs().read().bits();
        let _ = r.date().read(); // release the shadow lock
        v
    }

    /// Sets the dividers directly (init mode; usually `new` already picked
    /// the right values).
    pub fn set_dividers(&mut self, diva: u8, divb: u16) -> Result<(), ErtcError> {
        assert!(diva <= 127 && divb <= 32767, "DIVA 7 bits / DIVB 15 bits");
        unlock();
        let r = ertc();
        r.sts().modify(|_, w| w.imen().set_bit());
        let mut left = SYNC_TIMEOUT;
        while r.sts().read().imf().bit_is_clear() {
            left -= 1;
            if left == 0 {
                lock();
                return Err(ErtcError::Timeout);
            }
        }
        r.div().modify(|_, w| unsafe {
            w.diva().bits(diva);
            w.divb().bits(divb);
            w
        });
        r.sts().modify(|_, w| w.imen().clear_bit());
        lock();
        Ok(())
    }

    /// Configures an alarm: fires when the (unmasked) calendar fields match.
    /// The day field is filled with the current day of month (use
    /// `AlarmMask::day` for an every-hour/daily-style alarm).
    pub fn set_alarm(
        &mut self,
        which: Alarm,
        mask: AlarmMask,
        hour: u8,
        minute: u8,
        second: u8,
    ) -> Result<(), ErtcError> {
        if hour > 23 || minute > 59 || second > 59 {
            return Err(ErtcError::OutOfRange);
        }
        unlock();
        let r = ertc();
        // Disable the alarm and wait for its allow-write flag.
        match which {
            Alarm::A => {
                r.ctrl().modify(|_, w| w.alaen().clear_bit());
            }
            Alarm::B => {
                r.ctrl().modify(|_, w| w.alben().clear_bit());
            }
        }
        let mut left = SYNC_TIMEOUT;
        loop {
            let ok = match which {
                Alarm::A => r.sts().read().alawf().bit_is_set(),
                Alarm::B => r.sts().read().albwf().bit_is_set(),
            };
            if ok || left == 0 {
                break;
            }
            left -= 1;
        }
        if left == 0 {
            lock();
            return Err(ErtcError::Timeout);
        }
        let (ht, hu) = (tens(hour), units(hour));
        let (mt, mu) = (tens(minute), units(minute));
        let (st, su) = (tens(second), units(second));
        let d = ertc().date().read();
        let (dt, du) = (d.dt().bits(), d.du().bits());
        // ALA layout: MASK4/DT/DU/WKSEL/MASK3/AMPM/HT/HU/MASK2/MT/MU/MASK1/ST/SU.
        match which {
            Alarm::A => {
                r.ala().write(|w| unsafe {
                    w.ht().bits(ht);
                    w.hu().bits(hu);
                    w.mt().bits(mt);
                    w.mu().bits(mu);
                    w.st().bits(st);
                    w.su().bits(su);
                    w.dt().bits(dt);
                    w.du().bits(du);
                    w.ampm().clear_bit(); // 24-hour format
                    w.wksel().clear_bit(); // day-of-month semantics
                    if mask.day {
                        w.mask4().set_bit();
                    }
                    if mask.hours {
                        w.mask3().set_bit();
                    }
                    if mask.minutes {
                        w.mask2().set_bit();
                    }
                    if mask.seconds {
                        w.mask1().set_bit();
                    }
                    w
                });
            }
            Alarm::B => {
                r.alb().write(|w| unsafe {
                    w.ht().bits(ht);
                    w.hu().bits(hu);
                    w.mt().bits(mt);
                    w.mu().bits(mu);
                    w.st().bits(st);
                    w.su().bits(su);
                    w.dt().bits(dt);
                    w.du().bits(du);
                    w.ampm().clear_bit();
                    w.wksel().clear_bit();
                    if mask.day {
                        w.mask4().set_bit();
                    }
                    if mask.hours {
                        w.mask3().set_bit();
                    }
                    if mask.minutes {
                        w.mask2().set_bit();
                    }
                    if mask.seconds {
                        w.mask1().set_bit();
                    }
                    w
                });
            }
        }
        match which {
            Alarm::A => {
                r.ctrl().modify(|_, w| {
                    w.alaen().set_bit();
                    w.alaien().set_bit();
                    w
                });
            }
            Alarm::B => {
                r.ctrl().modify(|_, w| {
                    w.alben().set_bit();
                    w.albien().set_bit();
                    w
                });
            }
        }
        lock();
        Ok(())
    }

    /// Disables an alarm.
    pub fn disable_alarm(&mut self, which: Alarm) -> Result<(), ErtcError> {
        unlock();
        let r = ertc();
        match which {
            Alarm::A => {
                r.ctrl().modify(|_, w| w.alaen().clear_bit());
            }
            Alarm::B => {
                r.ctrl().modify(|_, w| w.alben().clear_bit());
            }
        }
        let mut left = SYNC_TIMEOUT;
        loop {
            let ok = match which {
                Alarm::A => r.sts().read().alawf().bit_is_set(),
                Alarm::B => r.sts().read().albwf().bit_is_set(),
            };
            if ok || left == 0 {
                break;
            }
            left -= 1;
        }
        lock();
        if left == 0 {
            Err(ErtcError::Timeout)
        } else {
            Ok(())
        }
    }

    /// Configures the periodic wakeup timer. `period_ticks` is in units of
    /// the selected `clock` (counter counts down from the value to 0).
    pub fn set_wakeup(&mut self, clock: WakeupClock, period_ticks: u16) -> Result<(), ErtcError> {
        unlock();
        let r = ertc();
        r.ctrl().modify(|_, w| w.waten().clear_bit());
        let mut left = SYNC_TIMEOUT;
        while r.sts().read().watwf().bit_is_clear() {
            left -= 1;
            if left == 0 {
                lock();
                return Err(ErtcError::Timeout);
            }
        }
        // WATCLK lives in CTRL[2:0]; the WAT register is the bare counter.
        r.ctrl()
            .modify(|_, w| unsafe { w.watclk().bits(clock.code()) });
        r.wat().write(|w| unsafe { w.bits(period_ticks) });
        r.ctrl().modify(|_, w| {
            w.waten().set_bit();
            w.watien().set_bit();
            w
        });
        lock();
        Ok(())
    }

    /// Stops the wakeup timer.
    pub fn disable_wakeup(&mut self) -> Result<(), ErtcError> {
        unlock();
        let r = ertc();
        r.ctrl().modify(|_, w| w.waten().clear_bit());
        let mut left = SYNC_TIMEOUT;
        while r.sts().read().watwf().bit_is_clear() {
            left -= 1;
            if left == 0 {
                break;
            }
        }
        lock();
        if left == 0 {
            Err(ErtcError::Timeout)
        } else {
            Ok(())
        }
    }

    /// Configures smooth calendar calibration (RM 17.3.4): adds or removes
    /// clock pulses over a 32/16/8-second cycle to trim the 1 Hz calendar
    /// tick against a reference.
    ///
    /// - `period`: calibration window (32 s default).
    /// - `add_512`: when true, inserts 512 ERTC_CLK pulses per window
    ///   (positive correction); when false, `dec` pulses are removed.
    /// - `dec`: number of ERTC_CLK pulses suppressed per window (0-511,
    ///   negative correction). Only meaningful when `add_512` is false.
    ///
    /// With the default 32768 Hz LEXT, `dec` of 1 removes ~0.95 ppm per
    /// window; combined with `add_512` the correction range is
    /// -487.1 ppm to +487.1 ppm.
    pub fn set_smooth_calibration(
        &mut self,
        period: SmoothCalPeriod,
        add_512: bool,
        dec: u16,
    ) -> Result<(), ErtcError> {
        assert!(dec <= 511, "DEC field is 9 bits (0-511)");
        unlock();
        let r = ertc();
        // Wait for the previous calibration update to complete (CALUPDF,
        // write-0-to-clear - it sets after the SCAL write lands).
        let mut left = SYNC_TIMEOUT;
        while r.sts().read().calupdf().bit_is_clear() {
            left -= 1;
            if left == 0 {
                lock();
                return Err(ErtcError::Timeout);
            }
        }
        let cal16 = period == SmoothCalPeriod::Sec16;
        let cal8 = period == SmoothCalPeriod::Sec8;
        let scal = (u32::from(cal16) << 13)
            | (u32::from(cal8) << 12)
            | (u32::from(add_512) << 11)
            | (dec as u32 & 0x1FF);
        r.scal().write(|w| unsafe { w.bits(scal) });
        lock();
        Ok(())
    }

    /// Disables smooth calibration (all correction pulses removed).
    pub fn disable_smooth_calibration(&mut self) {
        unlock();
        ertc().scal().write(|w| unsafe { w.bits(0) });
        lock();
    }

    /// Enables tamper pin 1 detection: selects the active edge, enables the
    /// pin and its interrupt. A tamper event clears the backup registers
    /// unless configured otherwise (RM 17).
    pub fn enable_tamper(&mut self, edge: TamperEdge) {
        unlock();
        let r = ertc();
        r.tamp().modify(|_, w| {
            match edge {
                TamperEdge::Falling => w.tp1edg().clear_bit(),
                TamperEdge::Rising => w.tp1edg().set_bit(),
            };
            w.tp1en().set_bit();
            w.tpien().set_bit();
            w
        });
        lock();
    }

    /// Reads one of the 20 battery-backed backup registers (BPR, 16-bit
    /// values; readable without unlocking, written with `write_bpr`).
    pub fn read_bpr(&self, index: usize) -> u16 {
        assert!(index < 20, "BPR index 0..19");
        ertc().bprdt(index).read().bits() as u16
    }

    /// Writes one of the 20 backup registers (unlocks the battery domain
    /// first; BPR survives resets and runs on VBAT).
    pub fn write_bpr(&self, index: usize, value: u16) {
        assert!(index < 20, "BPR index 0..19");
        unlock();
        ertc()
            .bprdt(index)
            .write(|w| unsafe { w.bits(value as u32) });
        lock();
    }

    /// Enables the ERTC interrupt for `events` and unmasks the NVIC lines.
    /// Alarms go through the ERTCALARM vector, tamper/timestamp through
    /// TAMPER, and the wakeup timer through ERTC_WKUP (bind the handlers
    /// used by the application with `bind_interrupts!`).
    pub fn listen(&mut self, events: &[Event]) {
        unlock();
        let r = ertc();
        let mut need_alarm = false;
        let mut need_tamper = false;
        let mut need_wakeup = false;
        for ev in events {
            match ev {
                Event::AlarmA => {
                    need_alarm = true;
                    r.ctrl().modify(|_, w| w.alaien().set_bit());
                }
                Event::AlarmB => {
                    need_alarm = true;
                    r.ctrl().modify(|_, w| w.albien().set_bit());
                }
                Event::Wakeup => {
                    need_wakeup = true;
                    r.ctrl().modify(|_, w| w.watien().set_bit());
                }
                Event::Timestamp => {
                    need_tamper = true;
                    r.ctrl().modify(|_, w| {
                        w.tsen().set_bit();
                        w.tsien().set_bit();
                        w
                    });
                }
                Event::Tamper => {
                    need_tamper = true;
                    r.tamp().modify(|_, w| w.tpien().set_bit());
                }
            }
        }
        // EXINT routing for the deepsleep wake path (RM 17.3.9: alarms =
        // line 17, timestamp/tamper = line 21, periodic wakeup = line 22).
        for ev in events {
            match ev {
                Event::AlarmA | Event::AlarmB => arm_exint_line(17),
                Event::Timestamp | Event::Tamper => arm_exint_line(21),
                Event::Wakeup => arm_exint_line(22),
            }
        }
        lock();
        use cortex_m::peripheral::NVIC;
        if need_alarm {
            NVIC::unpend(pac::Interrupt::ERTCALARM);
            unsafe { NVIC::unmask(pac::Interrupt::ERTCALARM) };
        }
        if need_tamper {
            NVIC::unpend(pac::Interrupt::TAMPER);
            unsafe { NVIC::unmask(pac::Interrupt::TAMPER) };
        }
        if need_wakeup {
            NVIC::unpend(pac::Interrupt::ERTC_WKUP);
            unsafe { NVIC::unmask(pac::Interrupt::ERTC_WKUP) };
        }
    }

    /// Waits (async) for any of the `events` and returns the one that fired;
    /// requires `listen()` + `bind_interrupts!` bindings for the vectors the
    /// events use (`ERTCALARM => AlarmHandler`, `TAMPER => InterruptHandler`,
    /// `ERTC_WKUP => WakeupHandler`). Competing events are all consumed.
    pub async fn wait_event(&self, events: &[Event]) -> Event {
        let want: u8 = events.iter().fold(0, |acc, e| acc | e.bit());
        core::future::poll_fn(|cx| {
            WAKER.register(cx.waker());
            let got = EVENTS.load(Ordering::Relaxed);
            if got & want != 0 {
                Poll::Ready(got)
            } else {
                Poll::Pending
            }
        })
        .await;
        // Consume: pick the first matching event.
        let got = EVENTS.swap(0, Ordering::Relaxed);
        for ev in [
            Event::AlarmA,
            Event::AlarmB,
            Event::Wakeup,
            Event::Timestamp,
            Event::Tamper,
        ] {
            if got & ev.bit() != 0 {
                return ev;
            }
        }
        unreachable!("wait_event returned with no matching event")
    }

    /// Clears the pending flags of `events` (mostly redundant: `on_irq`
    /// already clears every flag it observes).
    ///
    /// STS event flags are write-0-to-clear, but a read-modify-write would
    /// also write 0 to the hardware-managed allow-write flags (ALAWF/ALBWF/
    /// WATWF), and on-silicon that bulk clear suppresses the next hardware
    /// flag set (the alarm event still pulses EXINT17, but ALAF never
    /// latches). This follows the official `ertc_flag_clear` instead: write 1
    /// everywhere except the target flag bits (plus IMEN, preserved).
    pub fn clear_events(&self, events: &[Event]) {
        let mut flags = 0u32;
        for ev in events {
            match ev {
                Event::AlarmA => flags |= 1 << 8,
                Event::AlarmB => flags |= 1 << 9,
                Event::Wakeup => flags |= 1 << 10,
                Event::Timestamp => flags |= 1 << 11,
                Event::Tamper => flags |= (1 << 13) | (1 << 14),
            }
        }
        let imen = ertc().sts().read().imen().bit_is_set() as u32;
        unsafe {
            ertc()
                .sts()
                .write(|w| w.bits(!(flags | 0x80) | (imen << 7)));
        }
    }
}

const fn from_bcd(tens: u8, units: u8) -> u8 {
    tens * 10 + units
}

/// ERTC interrupt handler: bind it with
/// `bind_interrupts!(struct Irqs { TAMPER => hal::ertc::InterruptHandler; });`
/// (the family funnels every ERTC event into the TAMPER vector).
#[allow(non_snake_case)]
#[unsafe(no_mangle)]
pub extern "C" fn on_irq() {
    // The ERTC event flags ARE the level source of their EXINT lines: while a
    // flag is set the line stays asserted and the vector re-fires forever,
    // starving the executor. So every event observed here must also be
    // cleared here (official-style single-target write: 0s at the flag bits,
    // 1s elsewhere, IMEN preserved) - wait_event consumers never need to.
    let r = ertc();
    let sts = r.sts().read();
    let mut got = 0u8;
    let mut flags = 0u32;
    if sts.alaf().bit_is_set() {
        got |= EV_ALARM_A;
        flags |= 1 << 8;
    }
    if sts.albf().bit_is_set() {
        got |= EV_ALARM_B;
        flags |= 1 << 9;
    }
    if sts.watf().bit_is_set() {
        got |= EV_WAKEUP;
        flags |= 1 << 10;
    }
    if sts.tsf().bit_is_set() {
        got |= EV_TIMESTAMP;
        flags |= 1 << 11;
    }
    if sts.tp1f().bit_is_set() || sts.tp2f().bit_is_set() {
        got |= EV_TAMPER;
        flags |= (1 << 13) | (1 << 14);
    }
    if got != 0 {
        let imen = sts.imen().bit_is_set() as u32;
        unsafe { r.sts().write(|w| w.bits(!(flags | 0x80) | (imen << 7))) };
        // Then release the EXINT lines (write-1-to-clear pending).
        let ex = unsafe { &*pac::EXINT::PTR };
        for line in [17u8, 21, 22] {
            ex.intsts().write(|w| w.line(line).clear());
        }
        EVENTS.fetch_or(got, Ordering::Relaxed);
        WAKER.wake();
    } else {
        // No ERTC flag (stale EXINT pending, e.g. from before a reset of the
        // enable bits): just release the lines so the vector stops firing.
        let ex = unsafe { &*pac::EXINT::PTR };
        for line in [17u8, 21, 22] {
            ex.intsts().write(|w| w.line(line).clear());
        }
    }
}

// The `bind_interrupts!` companions need Handler impls for the three ERTC
// vectors: ERTCALARM (alarms), TAMPER (tamper/timestamp), ERTC_WKUP (wakeup).
pub struct AlarmHandler;

impl crate::interrupt::typelevel::Handler<crate::interrupt::typelevel::ERTCALARM> for AlarmHandler {
    unsafe fn on_interrupt() {
        on_irq();
    }
}

pub struct InterruptHandler;

impl crate::interrupt::typelevel::Handler<crate::interrupt::typelevel::TAMPER>
    for InterruptHandler
{
    unsafe fn on_interrupt() {
        on_irq();
    }
}

pub struct WakeupHandler;

impl crate::interrupt::typelevel::Handler<crate::interrupt::typelevel::ERTC_WKUP>
    for WakeupHandler
{
    unsafe fn on_interrupt() {
        on_irq();
    }
}

use core::task::Poll;
