//! PWC power control (RM 3): sleep / deepsleep / standby entry plus the
//! deepsleep LDO state and voltage-scaling helpers used by the clock tree.
//!
//! Mode entry (RM 3.6):
//! - sleep: WFI/WFE with SLEEPDEEP = 0 (any interrupt wakes).
//! - deepsleep: SLEEPDEEP = 1, PWC_CTRL.LPSEL = 0; HICK/HEXT stop, the LDO
//!   stays on (CTRL.VRSEL selects normal/low-power). Wake = any EXINT line;
//!   the system resumes on HICK before the HAL restores its run clock.
//! - standby: SLEEPDEEP = 1, LPSEL = 1, CTRLSTS.SWEF cleared; the LDO is off
//!   and SRAM content is lost. Wake = WKUP pin rising edge / ERTC alarm,
//!   tamper, timestamp or wakeup event / NRST / WDT reset; the chip reboots
//!   afterwards (check `RSTSSTS` for the reset cause).
//!
//! ```ignore
//! let mut pwc = hal::pwc::Pwc::new(p.PWC);
//! pwc.enter_deepsleep(DeepsleepLdo::Normal, None);      // wake on any EXINT
//! // standby resets the chip on exit:
//! pwc.enter_standby();
//! ```

use embassy_hal_internal::Peri;

use crate::pac;

/// Deep-sleep LDO state (CTRL.VRSEL).
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
pub enum DeepsleepLdo {
    /// LDO stays fully on (fastest wake).
    #[default]
    Normal,
    /// LDO low-power mode (lower current, extra wake delay for LDO settle).
    LowPower,
}

/// Power controller handle.
pub struct Pwc {
    _peri: Peri<'static, crate::peripherals::PWC>,
}

impl Pwc {
    /// Takes the PWC singleton and enables the APB1 clock.
    pub fn new(peri: Peri<'static, crate::peripherals::PWC>) -> Self {
        // The clock initializer may have selected 1.3 V for high-speed
        // operation. The PWC peripheral reset would put the LDO back at 1.2 V.
        crate::rcc::enable_pwc_clock();
        Self { _peri: peri }
    }

    /// Unlocks the battery-domain write protection (needed before ERTC/BPR
    /// register access; see the ertc module).
    pub fn unlock_backup_domain(&self) {
        let pwc = unsafe { &*pac::PWC::PTR };
        critical_section::with(|_| pwc.ctrl().modify(|_, w| w.bpwen().set_bit()));
    }

    /// Enters sleep mode (WFI): the CPU stops, all peripherals keep running;
    /// any enabled interrupt wakes. This call returns after the wake.
    pub fn enter_sleep(&mut self) {
        // SCR.SLEEPDEEP = bit 2 (cortex-m's SCR vcell is a raw u32).
        let scr = 0xE000_ED10 as *mut u32;
        unsafe {
            scr.write_volatile(scr.read_volatile() & !(1 << 2));
        }
        cortex_m::asm::wfi();
    }

    /// Enters deep sleep using WFI and an enabled EXINT wake source.
    ///
    /// The optional callback runs before entering sleep and can arm a wake
    /// source. The driver restores the previous system clock before pending
    /// interrupts are serviced. The Embassy time base stops during sleep.
    pub fn enter_deepsleep(&mut self, ldo: DeepsleepLdo, pre_wake: Option<&mut dyn FnMut()>) {
        if let Some(arm_wake) = pre_wake {
            arm_wake();
        }
        critical_section::with(|_| {
            // A SysTick wrap during clock switching can make WFI return
            // before the external wake event. Pause it as in the vendor's
            // ERTC wake example and resume it at the restored run rate.
            const SYSTICK_ENABLE: u32 = 1;
            const SYSTICK_WRITABLE: u32 = 0b111;
            let systick = unsafe { &*cortex_m::peripheral::SYST::PTR };
            let systick_ctrl = systick.csr.read() & SYSTICK_WRITABLE;
            unsafe { systick.csr.write(systick_ctrl & !SYSTICK_ENABLE) };

            let source = crate::rcc::active_system_clock();
            let pwc = unsafe { &*pac::PWC::PTR };
            let ldo_setting = pwc.ldoov().read().ldoovsel().bits();
            let low_power = matches!(ldo, DeepsleepLdo::LowPower);
            if low_power {
                crate::rcc::switch_to_hick_for_deepsleep();
                pwc.ldoov()
                    .modify(|_, w| unsafe { w.ldoovsel().bits(0b100) });
            }
            pwc.ctrl()
                .modify(|_, w| w.lpsel().clear_bit().vrsel().bit(low_power));
            let scr = 0xE000_ED10 as *mut u32;
            unsafe { scr.write_volatile(scr.read_volatile() | (1 << 2)) };
            cortex_m::asm::dsb();
            cortex_m::asm::isb();
            // A pending enabled IRQ can wake WFI with PRIMASK set. The
            // handler runs after the clock tree is back at the run rate.
            cortex_m::asm::wfi();
            unsafe { scr.write_volatile(scr.read_volatile() & !(1 << 2)) };
            if low_power {
                // Allow at least 120 us at the maximum HICK rate for
                // the post-wake regulator settling interval in the BSP.
                cortex_m::asm::delay(120 * (crate::rcc::HICK_FREQ.0 / 1_000_000));
                pwc.ldoov()
                    .modify(|_, w| unsafe { w.ldoovsel().bits(ldo_setting) });
            }
            crate::rcc::restore_after_deepsleep(source);
            unsafe { systick.csr.write(systick_ctrl) };
        });
    }

    /// Enters standby: SLEEPDEEP = 1, LPSEL = 1, SWEF cleared. The LDO shuts
    /// down and SRAM/registers are lost; the chip RESETS when the wake source
    /// fires (WKUP pin rising edge, ERTC alarm/tamper/timestamp/wakeup, NRST,
    /// WDT) - this call never returns.
    ///
    /// # Safety
    ///
    /// Everything in SRAM (including the Rust runtime state) is lost; the
    /// wake-up path goes through the normal reset vector.
    pub unsafe fn enter_standby(&mut self) {
        let pwc = unsafe { &*pac::PWC::PTR };
        pwc.ctrl().modify(|_, w| w.lpsel().set_bit());
        // Clear the software-standby flag: CTRLSTS.CLSWEF (bit 2) is a
        // write-1 clear strobe for SWEF (official pwc_standby_mode_enter);
        // the PAC exposes no writer field for it, so write raw bits.
        let sts = pwc.ctrlsts().read().bits();
        pwc.ctrlsts().write(|w| unsafe { w.bits(sts | (1 << 2)) });
        let scr = 0xE000_ED10 as *mut u32;
        unsafe {
            scr.write_volatile(scr.read_volatile() | (1 << 2));
        }
        cortex_m::asm::dsb();
        cortex_m::asm::wfi();
        // Not reached: the standby exit is a chip reset. If WFI returns
        // spuriously, sleep again (matches pwc_standby_mode_enter).
        loop {
            cortex_m::asm::wfi();
        }
    }

    /// Enables one of the programmable voltage monitors (CTRLSTS.PVMSEL +
    /// PVMEN); the PVM interrupt goes through the PVM vector.
    pub fn enable_pvm(&mut self, threshold: PvmThreshold) {
        let pwc = unsafe { &*pac::PWC::PTR };
        pwc.ctrl().modify(|_, w| {
            unsafe {
                w.pvmsel().bits(threshold.code());
            }
            w.pvmen().set_bit();
            w
        });
    }

    /// Disables the voltage monitor.
    pub fn disable_pvm(&mut self) {
        let pwc = unsafe { &*pac::PWC::PTR };
        pwc.ctrl().modify(|_, w| w.pvmen().clear_bit());
    }

    /// Reads the PVM output flag (CTRLSTS.PVMOF): true = VDD below threshold.
    pub fn pvm_below_threshold(&self) -> bool {
        unsafe { &*pac::PWC::PTR }
            .ctrlsts()
            .read()
            .pvmof()
            .bit_is_set()
    }
}

/// Programmable voltage monitor thresholds (CTRLSTS.PVMSEL).
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum PvmThreshold {
    V1d9,
    V2d1,
    V2d3,
    V2d5,
    V2d7,
    V2d9,
}

impl PvmThreshold {
    const fn code(self) -> u8 {
        match self {
            PvmThreshold::V1d9 => 0,
            PvmThreshold::V2d1 => 1,
            PvmThreshold::V2d3 => 2,
            PvmThreshold::V2d5 => 3,
            PvmThreshold::V2d7 => 4,
            PvmThreshold::V2d9 => 5,
        }
    }
}
