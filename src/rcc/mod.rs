//! Clock tree (CRM) configuration: `Config` + presets + `init()` + a global
//! [`Clocks`] snapshot, plus the macro tables implementing [`RccPeripheral`].
//!
//! The initialization sequence follows the flow validated on the
//! evb_at32f435cgt7 board (RM section 3.7 "LDO output voltage regulation"):
//! SCLK on HICK/HEXT -> set LDO to 1.3 V (>240 MHz) -> FLASH divider (/2//3//4
//! picked automatically for <=104 MHz, plus NZW_BST above 240 MHz) ->
//! HEXT/PLL ready wait -> AHB/APB dividers -> smooth switching (auto step,
//! >108 MHz) -> switch SCLK to the target source.
//!
//! WARNING: The FLASH `DIVR.FDIV` encoding is unusual (per the Artery firmware
//! library's `flash_clock_divider_type`): `0b00`=/2, `0b01`=/3, `0b10`=/4  -
//! there is **no /1**. At 288 MHz, /3 gives 96 MHz <= 104 MHz.

mod config;
pub use config::{AhbDiv, ApbDiv, Config, Hext, Pll, PllFr, PllSource, Sysclk};

use crate::peripheral::{ClockControl, RccPeripheral};
use crate::time::Hertz;
use embassy_sync::once_lock::OnceLock;

/// Undivided internal high-speed oscillator frequency.
pub const HICK_FREQ: Hertz = Hertz(48_000_000);
const HICK_DIV6: Hertz = Hertz(8_000_000);

/// FLASH interface clock limit (RM): the automatic divider targets this.
const FLASH_CLK_MAX: u32 = 104_000_000;
/// Above this SCLK frequency: LDO 1.3 V + NZW_BST + smooth switching (auto step).
const HI_FREQ_THRESHOLD: u32 = 240_000_000;
/// Threshold for enabling smooth switching (auto step); required for PLL > 108 MHz.
const AUTO_STEP_THRESHOLD: u32 = 108_000_000;
/// System clock limit (AT32F435 = 288 MHz).
const SYSCLK_MAX: u32 = 288_000_000;
/// APB1/APB2 bus limit.
const APB_MAX: u32 = 144_000_000;

/// Global clock snapshot: valid after `init()` succeeds; drivers read it on demand.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct Clocks {
    /// System clock (SCLK).
    pub sysclk: Hertz,
    /// AHB (HCLK).
    pub hclk: Hertz,
    /// APB1 (PCLK1).
    pub pclk1: Hertz,
    /// APB2 (PCLK2).
    pub pclk2: Hertz,
    /// APB1 timer clock: twice PCLK1 when the APB1 divider is > 1.
    pub pclk1_tim: Hertz,
    /// APB2 timer clock: twice PCLK2 when the APB2 divider is > 1.
    pub pclk2_tim: Hertz,
}

static CLOCKS: OnceLock<Clocks> = OnceLock::new();
const RESET_CLOCKS: Clocks = Clocks {
    sysclk: HICK_DIV6,
    hclk: HICK_DIV6,
    pclk1: HICK_DIV6,
    pclk2: HICK_DIV6,
    pclk1_tim: HICK_DIV6,
    pclk2_tim: HICK_DIV6,
};

/// Current clock configuration (reset default 8 MHz before `init()`).
#[inline]
pub fn clocks() -> &'static Clocks {
    CLOCKS.try_get().unwrap_or(&RESET_CLOCKS)
}

/// Ready-flag wait with a timeout panic (context included for easier debugging).
fn wait_with_timeout(cond: impl Fn() -> bool, what: &str) {
    let mut remaining = 5_000_000u32;
    while !cond() {
        remaining -= 1;
        assert!(remaining > 0, "clock init failed: {what}");
    }
}

#[inline]
fn crm() -> &'static crate::pac::crm::RegisterBlock {
    unsafe { &*crate::pac::CRM::PTR }
}

// HICK_TO_SCLK must be set before writing HICKDIV. Restore the fixed
// 8 MHz system clock path if SCLK will be switched to PLL.
fn select_hick_48m(use_as_sysclk: bool) {
    let misc1 = crm().misc1();
    misc1.modify(|_, w| w.hick_to_sclk().set_bit());
    cortex_m::asm::isb();
    misc1.modify(|_, w| w.hickdiv().set_bit());
    cortex_m::asm::isb();
    misc1.modify(|_, w| w.hick_to_sclk().bit(use_as_sysclk));
    cortex_m::asm::isb();
}

/// Enable the clocks of all GPIO ports (AT32 GPIOs hang off AHB:
/// CRM_AHBEN1 bits 0..7 = GPIOA..GPIOH).
/// Called once from `hal::init()`; pin-level drivers no longer toggle port
/// clocks individually.
pub(crate) fn enable_gpio_clocks() {
    crm()
        .ahben1()
        .modify(|r, w| unsafe { w.bits(r.bits() | 0xFF) });
}

/// Enable the SCFG clock (APB2): needed for muxing config such as EXINT port
/// selection (EXINTC).
pub(crate) fn enable_scfg_clock() {
    crm().apb2en().modify(|_, w| w.scfg().set_bit());
}

/// Apply the clock configuration. Must be called once after
/// `Peripherals::take()` and before any task/driver consumes the clocks
/// (`hal::init()` already takes care of this).
pub(crate) fn init(config: Config) {
    let crm = crm();
    let flash = unsafe { &*crate::pac::FLASH::PTR };

    let clocks = config.clocks();
    let sysclk = clocks.sysclk;

    // ---- LDO 1.3 V (required above 240 MHz; done while SCLK is still on
    // HICK/HEXT, which is the state right after reset) ----
    if sysclk.0 > HI_FREQ_THRESHOLD {
        enable_pwc_clock();
        let pwc = unsafe { &*crate::pac::PWC::PTR };
        pwc.ldoov()
            .modify(|_, w| unsafe { w.ldoovsel().bits(0b001) });
    }

    // ---- FLASH clock divider: pick the smallest of {/2, /3, /4} keeping
    // flash_clk <= 104 MHz ----
    // (FDIV encoding: 0b00=/2, 0b01=/3, 0b10=/4 - no /1!)
    let (fdiv_code, flash_hz) = [(0u8, 2u32), (1, 3), (2, 4)]
        .into_iter()
        .find(|(_, div)| sysclk.0 / div <= FLASH_CLK_MAX)
        .map(|(code, div)| (code, sysclk.0 / div))
        .unwrap_or((2, sysclk.0 / 4));
    assert!(
        flash_hz <= FLASH_CLK_MAX,
        "no compliant flash divider found for {} Hz",
        sysclk.0
    );
    flash
        .divr()
        .modify(|_, w| unsafe { w.fdiv().bits(fdiv_code) });
    // Step 3 of the RM "LDO output voltage regulation" flow: NZW_BST goes
    // together with DIVR; omitting it causes sporadic flash access errors and
    // runaway execution at high frequencies (observed on the evb board).
    if sysclk.0 > HI_FREQ_THRESHOLD {
        flash.psr().modify(|_, w| w.nzw_bst().set_bit());
    }

    // ---- HEXT enable + ready wait ----
    let hext_used = match (config.sys, config.pll) {
        (Sysclk::Hext, _) => true,
        (Sysclk::Pll, Some(pll)) => pll.source == PllSource::Hext,
        _ => false,
    };
    if hext_used {
        let hext = config
            .hext
            .expect("config.hext must be set when sys=Hext or PLL(HEXT)");
        crm.ctrl().modify(|_, w| {
            w.hexten().set_bit();
            if hext.bypass {
                w.hextbyps().set_bit();
            }
            w
        });
        wait_with_timeout(
            || crm.ctrl().read().hextstbl().bit_is_set(),
            "HEXT failed to start",
        );
    }

    if config.sys == Sysclk::Hick {
        // The reset SCLK path provides 8 MHz. Select the undivided HICK
        // explicitly to obtain the 48 MHz promised by Config::DEFAULT.
        select_hick_48m(true);
    }

    // ---- PLL configuration + enable + lock wait ----
    if config.sys == Sysclk::Pll {
        let pll = config.pll.expect("PLL configuration was validated");
        if pll.source == PllSource::Hick {
            // The PLL reference on the HICK path is the divided 8 MHz clock;
            // the HICK must be stable before the PLL can lock (the reset
            // default HICKEN=1 starts it, but the ready wait is still
            // required - observed on the evb: without this wait the PLL
            // never locks).
            wait_with_timeout(
                || crm.ctrl().read().hickstbl().bit_is_set(),
                "HICK failed to start",
            );
            // Keep SCLK on the fixed 8 MHz path until switching to the PLL.
            // The PLL's HICK reference is 8 MHz with this sequence, as in
            // the vendor's crm_pll_config and crm_clocks_freq_get.
            select_hick_48m(false);
        }
        crm.pllcfg().write(|w| {
            match pll.source {
                PllSource::Hext => w.pllrcs().set_bit(),
                PllSource::Hick => w.pllrcs().clear_bit(),
            };
            unsafe {
                w.pll_ns().bits(pll.ns);
                w.pll_ms().bits(pll.ms);
                w.pll_fr().bits(pll.fr.bits());
            }
            w
        });
        crm.ctrl().modify(|_, w| w.pllen().set_bit());
        wait_with_timeout(
            || crm.ctrl().read().pllstbl().bit_is_set(),
            "PLL failed to lock",
        );
    }

    // ---- Bus dividers: AHB, APB1, APB2 ----
    crm.cfg().modify(|_, w| {
        match config.ahb_pre {
            AhbDiv::Div1 => w.ahbdiv().sclk(),
            AhbDiv::Div2 => w.ahbdiv().div2(),
            AhbDiv::Div4 => w.ahbdiv().div4(),
            AhbDiv::Div8 => w.ahbdiv().div8(),
            AhbDiv::Div16 => w.ahbdiv().div16(),
            AhbDiv::Div64 => w.ahbdiv().div64(),
            AhbDiv::Div128 => w.ahbdiv().div128(),
            AhbDiv::Div256 => w.ahbdiv().div256(),
            AhbDiv::Div512 => w.ahbdiv().div512(),
        };
        match config.apb1_pre {
            ApbDiv::Div1 => w.apb1div().hclk(),
            ApbDiv::Div2 => w.apb1div().div2(),
            ApbDiv::Div4 => w.apb1div().div4(),
            ApbDiv::Div8 => w.apb1div().div8(),
            ApbDiv::Div16 => w.apb1div().div16(),
        };
        match config.apb2_pre {
            ApbDiv::Div1 => w.apb2div().hclk(),
            ApbDiv::Div2 => w.apb2div().div2(),
            ApbDiv::Div4 => w.apb2div().div4(),
            ApbDiv::Div8 => w.apb2div().div8(),
            ApbDiv::Div16 => w.apb2div().div16(),
        };
        w
    });

    switch_system_clock(config.sys, sysclk.0 > AUTO_STEP_THRESHOLD);
    CLOCKS.init(clocks).expect("clocks initialized twice");
}

pub(crate) fn enable_pwc_clock() {
    critical_section::with(|_| {
        let crm = crm();
        crm.apb1en().modify(|_, w| w.pwc().set_bit());
        let _ = crm.apb1en().read();
    });
}

pub(crate) fn active_system_clock() -> Sysclk {
    let status = crm().cfg().read().sclksts();
    if status.is_pll() {
        Sysclk::Pll
    } else if status.is_hext() {
        Sysclk::Hext
    } else {
        Sysclk::Hick
    }
}

fn switch_system_clock(source: Sysclk, smooth: bool) {
    let crm = crm();
    if smooth {
        crm.misc2()
            .modify(|_, w| unsafe { w.auto_step_en().bits(0b11) });
    }
    match source {
        Sysclk::Hick => crm.cfg().modify(|_, w| w.sclksel().hick()),
        Sysclk::Hext => crm.cfg().modify(|_, w| w.sclksel().hext()),
        Sysclk::Pll => crm.cfg().modify(|_, w| w.sclksel().pll()),
    };
    wait_with_timeout(
        || active_system_clock() == source,
        "SCLK switch did not take effect",
    );
    if smooth {
        crm.misc2()
            .modify(|_, w| unsafe { w.auto_step_en().bits(0b00) });
    }
}

pub(crate) fn switch_to_hick_for_deepsleep() {
    let crm = crm();
    crm.ctrl().modify(|_, w| w.hicken().set_bit());
    wait_with_timeout(
        || crm.ctrl().read().hickstbl().bit_is_set(),
        "HICK failed to start",
    );
    switch_system_clock(Sysclk::Hick, false);
}

pub(crate) fn restore_after_deepsleep(source: Sysclk) {
    if active_system_clock() == source {
        return;
    }
    // Dividers and PLL parameters are retained. The oscillator and PLL
    // need to be restarted and stable before SCLK can switch back (RM 3.6).
    let crm = crm();
    if source == Sysclk::Hext
        || (source == Sysclk::Pll && crm.pllcfg().read().pllrcs().bit_is_set())
    {
        crm.ctrl().modify(|_, w| w.hexten().set_bit());
        wait_with_timeout(
            || crm.ctrl().read().hextstbl().bit_is_set(),
            "HEXT failed to restart",
        );
    }
    if source == Sysclk::Pll {
        crm.ctrl().modify(|_, w| w.pllen().set_bit());
        wait_with_timeout(
            || crm.ctrl().read().pllstbl().bit_is_set(),
            "PLL failed to relock",
        );
    }
    switch_system_clock(source, clocks().sysclk.0 > AUTO_STEP_THRESHOLD);
}

/// `RccPeripheral` macro table: for each named peripheral, (enable register,
/// enable field, reset register, reset field, frequency source).
///
/// All field/register names are verified CRM register fields from
/// at32-pac; timer-class peripherals take their frequency from
/// `pclk1_tim`/`pclk2_tim` (2x the APB clock when the APB divider is > 1).
macro_rules! impl_rcc {
    ($($peri:ident => ($en_reg:ident, $en_field:ident, $rst_reg:ident, $rst_field:ident, $freq:ident)),* $(,)?) => {
        $(
            impl RccPeripheral for crate::peripherals::$peri {
                #[inline]
                fn frequency() -> Hertz {
                    crate::rcc::clocks().$freq
                }

            }

            impl ClockControl for crate::peripherals::$peri {
                fn enable_and_reset() {
                    critical_section::with(|_| {
                        let crm = unsafe { &*crate::pac::CRM::PTR };
                        crm.$en_reg().modify(|_, w| w.$en_field().set_bit());
                        // Reset pulse: set to 1 then clear to 0 (writing 1 resets the peripheral)
                        crm.$rst_reg().modify(|_, w| w.$rst_field().set_bit());
                        crm.$rst_reg().modify(|_, w| w.$rst_field().clear_bit());
                    });
                }

                fn disable() {
                    critical_section::with(|_| {
                        let crm = unsafe { &*crate::pac::CRM::PTR };
                        crm.$en_reg().modify(|_, w| w.$en_field().clear_bit());
                    });
                }
            }
        )*
    };
}

impl_rcc! {
    // ---- AHB1 (CRM_AHBEN1 / AHBRST1), frequency source hclk ----
    CRC      => (ahben1, crc,     ahbrst1, crc,     hclk),
    EDMA     => (ahben1, edma,    ahbrst1, edma,    hclk),
    DMA1     => (ahben1, dma1,    ahbrst1, dma1,    hclk),
    DMA2     => (ahben1, dma2,    ahbrst1, dma2,    hclk),
    // ---- AHB2 (CRM_AHBEN2 / AHBRST2) ----
    DVP      => (ahben2, dvp,     ahbrst2, dvp,     hclk),
    USB_OTG1 => (ahben2, otgfs1,  ahbrst2, otgfs1,  hclk),
    SDIO1    => (ahben2, sdio1,   ahbrst2, sdio1,   hclk),
    // ---- AHB3 (CRM_AHBEN3 / AHBRST3) ----
    XMC      => (ahben3, xmc,     ahbrst3, xmc,     hclk),
    QSPI1    => (ahben3, qspi1,   ahbrst3, qspi1,   hclk),
    QSPI2    => (ahben3, qspi2,   ahbrst3, qspi2,   hclk),
    SDIO2    => (ahben3, sdio2,   ahbrst3, sdio2,   hclk),
    // ---- APB1 (CRM_APB1EN / APB1RST); timers use pclk1_tim, the rest pclk1 ----
    TMR2     => (apb1en, tmr2,    apb1rst, tmr2,    pclk1_tim),
    TMR3     => (apb1en, tmr3,    apb1rst, tmr3,    pclk1_tim),
    TMR4     => (apb1en, tmr4,    apb1rst, tmr4,    pclk1_tim),
    TMR5     => (apb1en, tmr5,    apb1rst, tmr5,    pclk1_tim),
    TMR6     => (apb1en, tmr6,    apb1rst, tmr6,    pclk1_tim),
    TMR7     => (apb1en, tmr7,    apb1rst, tmr7,    pclk1_tim),
    TMR12    => (apb1en, tmr12,   apb1rst, tmr12,   pclk1_tim),
    TMR13    => (apb1en, tmr13,   apb1rst, tmr13,   pclk1_tim),
    TMR14    => (apb1en, tmr14,   apb1rst, tmr14,   pclk1_tim),
    WWDT     => (apb1en, wwdt,    apb1rst, wwdt,    pclk1),
    SPI2     => (apb1en, spi2,    apb1rst, spi2,    pclk1),
    I2S2_EXT => (apb1en, spi2,    apb1rst, spi2,    pclk1),
    SPI3     => (apb1en, spi3,    apb1rst, spi3,    pclk1),
    I2S3_EXT => (apb1en, spi3,    apb1rst, spi3,    pclk1),
    USART2   => (apb1en, usart2,  apb1rst, usart2,  pclk1),
    USART3   => (apb1en, usart3,  apb1rst, usart3,  pclk1),
    UART4    => (apb1en, uart4,   apb1rst, uart4,   pclk1),
    UART5    => (apb1en, uart5,   apb1rst, uart5,   pclk1),
    UART7    => (apb1en, uart7,   apb1rst, uart7,   pclk1),
    UART8    => (apb1en, uart8,   apb1rst, uart8,   pclk1),
    I2C1     => (apb1en, i2c1,    apb1rst, i2c1,    pclk1),
    I2C2     => (apb1en, i2c2,    apb1rst, i2c2,    pclk1),
    I2C3     => (apb1en, i2c3,    apb1rst, i2c3,    pclk1),
    CAN1     => (apb1en, can1,    apb1rst, can1,    pclk1),
    CAN2     => (apb1en, can2,    apb1rst, can2,    pclk1),
    PWC      => (apb1en, pwc,     apb1rst, pwc,     pclk1),
    DAC      => (apb1en, dac,     apb1rst, dac,     pclk1),
    // ---- APB2 (CRM_APB2EN / APB2RST); timers use pclk2_tim, the rest pclk2 ----
    TMR1     => (apb2en, tmr1,    apb2rst, tmr1,    pclk2_tim),
    TMR8     => (apb2en, tmr8,    apb2rst, tmr8,    pclk2_tim),
    TMR9     => (apb2en, tmr9,    apb2rst, tmr9,    pclk2_tim),
    TMR10    => (apb2en, tmr10,   apb2rst, tmr10,   pclk2_tim),
    TMR11    => (apb2en, tmr11,   apb2rst, tmr11,   pclk2_tim),
    TMR20    => (apb2en, tmr20,   apb2rst, tmr20,   pclk2_tim),
    USART1   => (apb2en, usart1,  apb2rst, usart1,  pclk2),
    USART6   => (apb2en, usart6,  apb2rst, usart6,  pclk2),
    // ADC1/2/3 have separate enable bits but share the `adc` reset field
    // (single ADC clock domain)
    ADC1     => (apb2en, adc1,    apb2rst, adc,     pclk2),
    ADC2     => (apb2en, adc2,    apb2rst, adc,     pclk2),
    ADC3     => (apb2en, adc3,    apb2rst, adc,     pclk2),
    SPI1     => (apb2en, spi1,    apb2rst, spi1,    pclk2),
    SPI4     => (apb2en, spi4,    apb2rst, spi4,    pclk2),
    SCFG     => (apb2en, scfg,    apb2rst, scfg,    pclk2),
    ACC      => (apb2en, acc,     apb2rst, acc,     pclk2),
    USB_OTG2 => (ahben1, otgfs2,  ahbrst1, otgfs2,  hclk),
}
