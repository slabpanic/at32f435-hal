//! HICK automatic clock calibration (ACC, RM 22): trims the HICK oscillator
//! against the USB SOF signal (1 ms period) to within +/- 0.25%, as required
//! for USB device operation without a crystal-accurate system clock.
//!
//! The module has no external pins; it samples the (internally divided, ~8
//! MHz) HICK clock against USB OTGFS1/2 SOF pulses and steps
//! CRM_CTRL.HICKTRIM (default 32 = 8 MHz +/- 0.25%; each step ~ 20 kHz at
//! 48 MHz HICK) or HICKCAL depending on the calibration mode.
//!
//! ```ignore
//! let mut acc = hal::acc::Acc::new(p.ACC, hal::acc::SofSource::Otgfs1);
//! acc.start(&hal::acc::AccConfig::default());
//! while !acc.calibration_ready() {}
//! ```

use embassy_hal_internal::Peri;

use crate::pac;

/// USB SOF reference source (CTRL1.SOFSEL).
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum SofSource {
    /// OTGFS1 SOF pulse.
    Otgfs1,
    /// OTGFS2 SOF pulse.
    Otgfs2,
}

/// ACC calibration configuration.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct AccConfig {
    /// Which USB core provides the SOF reference.
    pub sof_source: SofSource,
    /// Calibration trim path: false = CRM_HICKCAL (bypass mode, coarse),
    /// true = CRM_HICKTRIM (fine, the usual path).
    pub trim_mode: TrimMode,
    /// Step value written to CTRL1.STEP (the official library uses 4).
    pub step: u8,
}

impl Default for AccConfig {
    fn default() -> Self {
        Self {
            sof_source: SofSource::Otgfs1,
            trim_mode: TrimMode::HickTrim,
            step: 4,
        }
    }
}

/// Calibration trim path (CTRL1.ENTRIM).
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
pub enum TrimMode {
    /// Adjust `CRM_CTRL.HICKCAL[7:0]` (coarse; used when HICK bypasses).
    #[default]
    HickTrim,
    /// Adjust CRM_CTRL.HICKCAL (bypass mode only).
    HickCal,
}

/// ACC handle.
pub struct Acc {
    _peri: Peri<'static, crate::peripherals::ACC>,
}

impl Acc {
    /// Takes the ACC singleton (clock enabled by `hal::init` via the rcc
    /// table; also enable the USB core clocks so SOF can be produced).
    pub fn new(peri: Peri<'static, crate::peripherals::ACC>, cfg: &AccConfig) -> Self {
        <crate::peripherals::ACC as crate::peripheral::ClockControl>::enable_and_reset();
        let r = unsafe { &*pac::ACC::PTR };
        r.ctrl1().modify(|_, w| {
            match cfg.sof_source {
                SofSource::Otgfs1 => w.sofsel().clear_bit(),
                SofSource::Otgfs2 => w.sofsel().set_bit(),
            };
            match cfg.trim_mode {
                TrimMode::HickTrim => w.entrim().set_bit(),
                TrimMode::HickCal => w.entrim().clear_bit(),
            };
            unsafe { w.step().bits(cfg.step) };
            w.calon().clear_bit();
            w
        });
        Self { _peri: peri }
    }

    /// Starts the automatic calibration (CTRL1.CALON = 1).
    pub fn start(&mut self, _cfg: &AccConfig) {
        unsafe { &*pac::ACC::PTR }
            .ctrl1()
            .modify(|_, w| w.calon().set_bit());
    }

    /// Stops the calibration.
    pub fn stop(&mut self) {
        unsafe { &*pac::ACC::PTR }
            .ctrl1()
            .modify(|_, w| w.calon().clear_bit());
    }

    /// Current CRM_HICKTRIM value the ACC adjusted to (CTRL2.HICKTRIM,
    /// read-only mirror; default 32 = 8 MHz +/- 0.25%).
    pub fn hick_trim(&self) -> u8 {
        unsafe { &*pac::ACC::PTR }.ctrl2().read().hicktrim().bits()
    }

    /// Calibration-ready flag (STS.CALRDY: the HICK is within +/- 0.25%).
    pub fn calibration_ready(&self) -> bool {
        unsafe { &*pac::ACC::PTR }
            .sts()
            .read()
            .calrdy()
            .bit_is_set()
    }

    /// Reference-signal-lost error flag (STS.RSLOST: SOF missing).
    pub fn reference_lost(&self) -> bool {
        unsafe { &*pac::ACC::PTR }
            .sts()
            .read()
            .rslost()
            .bit_is_set()
    }

    /// Clears both flags. The PAC marks ACC_STS read-only; the RM 22.6.2
    /// documents the flags as rw0c, so this writes the register directly
    /// through its address.
    pub fn clear_flags(&self) {
        let base = pac::ACC::PTR as usize;
        // STS is at offset 0x00 per the RM register map (22.6.1).
        let sts = base as *mut u32;
        unsafe {
            sts.write_volatile(0);
        }
    }

    /// Enables the CALRDY / RSLOST interrupts and unmasks the NVIC line.
    ///
    /// NOTE: the ACC vector (IRQ 103, `ACC_IRQn`) is missing from the family
    /// SVD/PAC `Interrupt` enum, so `bind_interrupts!` cannot be used for it.
    /// Applications wire it manually with
    /// `#[no_mangle] extern "C" fn ACC() { hal::acc::on_irq() }` (the
    /// cortex-m-rt device vector table references the symbol by name) - or
    /// just poll `calibration_ready()` / `reference_lost()`.
    pub fn listen(&mut self) {
        unsafe { &*pac::ACC::PTR }.ctrl1().modify(|_, w| {
            w.calrdyien().set_bit();
            w.eien().set_bit();
            w
        });
        unsafe {
            cortex_m::peripheral::NVIC::unmask(Irq103);
        }
    }

    /// Disables the ACC interrupts (NVIC mask only; flags stay sticky).
    pub fn unlisten(&mut self) {
        cortex_m::peripheral::NVIC::mask(Irq103);
    }
}

/// Minimal `InterruptNumber` wrapper for the ACC vector (IRQ 103, named
/// `ACC_IRQn` in the CMSIS header but absent from the family SVD/PAC).
#[derive(Copy, Clone)]
struct Irq103;
unsafe impl cortex_m::interrupt::InterruptNumber for Irq103 {
    fn number(self) -> u16 {
        103
    }
}

/// ACC interrupt handler body (flags are sticky; read `calibration_ready()` /
/// `reference_lost()` and call `clear_flags()` when done). Wire manually with
/// `#[no_mangle] extern "C" fn ACC() { hal::acc::on_irq() }`.
pub fn on_irq() {
    // Nothing to do in the HAL itself: the flags are polled by the user.
}
