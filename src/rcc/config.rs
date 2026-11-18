use super::{Clocks, APB_MAX, HICK_DIV6, HICK_FREQ, SYSCLK_MAX};
use crate::time::Hertz;

/// HEXT (external high-speed crystal/clock source) configuration.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct Hext {
    /// Crystal/input frequency.
    pub freq: Hertz,
    /// true = bypass mode (external active clock), false = on-board crystal.
    pub bypass: bool,
}

/// System clock source.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Sysclk {
    /// Undivided internal oscillator at a nominal 48 MHz.
    Hick,
    /// HEXT used directly as the system clock.
    Hext,
    /// PLL output (requires the `pll` field).
    Pll,
}

/// PLL reference source (PLLCFG.PLLRCS).
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum PllSource {
    /// HEXT, prescaled by PLL_MS before entering the PLL.
    Hext,
    /// Nominal 8 MHz HICK reference to the PLL. Use HEXT when oscillator
    /// accuracy is important.
    Hick,
}

/// PLL post-divider (RM 4.5.3).
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum PllFr {
    Div4,
    Div8,
    Div16,
    Div32,
}

impl PllFr {
    /// PAC encoding of the FR field.
    pub(super) const fn bits(self) -> u8 {
        match self {
            PllFr::Div4 => 0b10,
            PllFr::Div8 => 0b011,
            PllFr::Div16 => 0b100,
            PllFr::Div32 => 0b101,
        }
    }

    /// Divider factor.
    pub(super) const fn factor(self) -> u32 {
        match self {
            PllFr::Div4 => 4,
            PllFr::Div8 => 8,
            PllFr::Div16 => 16,
            PllFr::Div32 => 32,
        }
    }
}

/// PLL configuration: output = reference / MS * NS / FR.
/// RM constraint: the reference input after /MS must be 2-16 MHz.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct Pll {
    pub source: PllSource,
    /// Multiplier (NS field, 9 bits).
    pub ns: u16,
    /// Prescaler (MS field, 4 bits, >=1).
    pub ms: u8,
    /// Post-divider (FR field).
    pub fr: PllFr,
}

/// AHB divider (AHBDIV field).
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum AhbDiv {
    Div1,
    Div2,
    Div4,
    Div8,
    Div16,
    Div64,
    Div128,
    Div256,
    Div512,
}

impl AhbDiv {
    pub(super) const fn factor(self) -> u32 {
        match self {
            AhbDiv::Div1 => 1,
            AhbDiv::Div2 => 2,
            AhbDiv::Div4 => 4,
            AhbDiv::Div8 => 8,
            AhbDiv::Div16 => 16,
            AhbDiv::Div64 => 64,
            AhbDiv::Div128 => 128,
            AhbDiv::Div256 => 256,
            AhbDiv::Div512 => 512,
        }
    }
}

/// APB1/APB2 divider (limit 144 MHz; when the divider is > 1, timer clocks on
/// that APB are 2x).
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum ApbDiv {
    Div1,
    Div2,
    Div4,
    Div8,
    Div16,
}

impl ApbDiv {
    pub(super) const fn factor(self) -> u32 {
        match self {
            ApbDiv::Div1 => 1,
            ApbDiv::Div2 => 2,
            ApbDiv::Div4 => 4,
            ApbDiv::Div8 => 8,
            ApbDiv::Div16 => 16,
        }
    }
}

/// Clock tree configuration. Start from [`Config::DEFAULT`] (HICK 48 MHz) or
/// one of the presets, then tweak fields as needed.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct Config {
    /// HEXT configuration; required when HEXT/PLL is used.
    pub hext: Option<Hext>,
    /// Target system clock source.
    pub sys: Sysclk,
    /// PLL configuration; required when `sys == Sysclk::Pll`.
    pub pll: Option<Pll>,
    /// AHB divider.
    pub ahb_pre: AhbDiv,
    /// APB1 divider.
    pub apb1_pre: ApbDiv,
    /// APB2 divider.
    pub apb2_pre: ApbDiv,
}

impl Config {
    /// Computes and validates the configured clock tree without touching hardware.
    ///
    /// Panics if the configuration violates the PLL or bus clock limits.
    pub fn clocks(self) -> Clocks {
        let hext_frequency = || {
            let hext = self.hext.expect("missing HEXT configuration");
            assert!(
                (1..=25_000_000).contains(&hext.freq.0),
                "HEXT must be at most 25 MHz"
            );
            assert!(
                hext.bypass || hext.freq.0 >= 4_000_000,
                "a HEXT crystal must be at least 4 MHz"
            );
            hext.freq
        };
        let sysclk = match self.sys {
            Sysclk::Hick => HICK_FREQ,
            Sysclk::Hext => hext_frequency(),
            Sysclk::Pll => {
                let pll = self.pll.expect("missing PLL configuration");
                let input = u64::from(match pll.source {
                    PllSource::Hext => hext_frequency().0,
                    PllSource::Hick => HICK_DIV6.0,
                });
                assert!((1..=15).contains(&pll.ms), "PLL_MS must be 1..=15");
                assert!((31..=500).contains(&pll.ns), "PLL_NS must be 31..=500");
                let pre = u64::from(pll.ms);
                let vco = input * u64::from(pll.ns);
                assert!(
                    (2_000_000 * pre..=16_000_000 * pre).contains(&input),
                    "PLL reference must be 2..=16 MHz"
                );
                assert!(
                    (500_000_000 * pre..=1_200_000_000 * pre).contains(&vco),
                    "PLL VCO must be 500..=1200 MHz"
                );
                Hertz((vco / (pre * u64::from(pll.fr.factor()))) as u32)
            }
        };
        assert!(
            (1..=SYSCLK_MAX).contains(&sysclk.0),
            "invalid system clock frequency"
        );
        let hclk = Hertz(sysclk.0 / self.ahb_pre.factor());
        let apb = |divider: ApbDiv| {
            let divisor = divider.factor();
            let bus = Hertz(hclk.0 / divisor);
            assert!(
                (1..=APB_MAX).contains(&bus.0),
                "invalid APB clock frequency"
            );
            let timer = Hertz(hclk.0 / (divisor / 2).max(1));
            (bus, timer)
        };
        let (pclk1, pclk1_tim) = apb(self.apb1_pre);
        let (pclk2, pclk2_tim) = apb(self.apb2_pre);
        Clocks {
            sysclk,
            hclk,
            pclk1,
            pclk2,
            pclk1_tim,
            pclk2_tim,
        }
    }

    /// Default configuration: undivided HICK at 48 MHz.
    pub const DEFAULT: Config = Config {
        hext: None,
        sys: Sysclk::Hick,
        pll: None,
        ahb_pre: AhbDiv::Div1,
        apb1_pre: ApbDiv::Div1,
        apb2_pre: ApbDiv::Div1,
    };

    /// HICK (8 MHz divided path) -> PLL * 144/(1*4) -> 288 MHz, AHB=288,
    /// APB1=APB2=144. Crystal-free 288 MHz boot. The PLL reference on the
    /// HICK path is the divided 8 MHz clock (NOT the 48 MHz one - confirmed
    /// against the official `sclk_64m_hick_config`: 8/1*64/8 = 64 MHz).
    /// NOTE: the HICK frequency is nominal (see the WDT LICK note in
    /// pitfalls.md); use HEXT or ACC calibration when clock accuracy matters.
    pub const SYSCLK_FREQ_288MHZ_HICK: Config = Config {
        hext: None,
        sys: Sysclk::Pll,
        pll: Some(Pll {
            source: PllSource::Hick,
            ns: 144,
            ms: 1,
            fr: PllFr::Div4,
        }),
        ahb_pre: AhbDiv::Div1,
        apb1_pre: ApbDiv::Div2,
        apb2_pre: ApbDiv::Div2,
    };

    /// HICK (8 MHz) -> PLL * 64/(1*8) -> 64 MHz - official example config,
    /// kept as the low-frequency HICK-PLL reference/validation preset.
    pub const SYSCLK_FREQ_64MHZ_HICK: Config = Config {
        hext: None,
        sys: Sysclk::Pll,
        pll: Some(Pll {
            source: PllSource::Hick,
            ns: 64,
            ms: 1,
            fr: PllFr::Div8,
        }),
        ahb_pre: AhbDiv::Div1,
        apb1_pre: ApbDiv::Div2,
        apb2_pre: ApbDiv::Div2,
    };

    /// HEXT 8 MHz crystal -> PLL * 144/(1*4) -> 288 MHz, AHB=288, APB1=APB2=144.
    /// On-board preset for the evb_at32f435cgt7 (validated on hardware).
    pub const SYSCLK_FREQ_288MHZ_HEXT_8M: Config = Config {
        hext: Some(Hext {
            freq: Hertz(8_000_000),
            bypass: false,
        }),
        sys: Sysclk::Pll,
        pll: Some(Pll {
            source: PllSource::Hext,
            ns: 144,
            ms: 1,
            fr: PllFr::Div4,
        }),
        ahb_pre: AhbDiv::Div1,
        apb1_pre: ApbDiv::Div2,
        apb2_pre: ApbDiv::Div2,
    };
}

impl Default for Config {
    fn default() -> Self {
        Config::DEFAULT
    }
}

#[cfg(test)]
mod tests {
    use super::{Config, Hertz};

    #[test]
    fn board_and_hick_presets_resolve_to_documented_rates() {
        let external = Config::SYSCLK_FREQ_288MHZ_HEXT_8M.clocks();
        let internal = Config::SYSCLK_FREQ_288MHZ_HICK.clocks();
        assert_eq!(external, internal);
        assert_eq!(internal.sysclk, Hertz(288_000_000));
        assert_eq!(internal.pclk1, Hertz(144_000_000));
        assert_eq!(internal.pclk1_tim, internal.hclk);
        assert_eq!(
            Config::SYSCLK_FREQ_64MHZ_HICK.clocks().sysclk,
            Hertz(64_000_000)
        );
    }

    #[test]
    #[should_panic(expected = "PLL VCO")]
    fn out_of_range_vco_is_rejected_before_touching_hardware() {
        let mut config = Config::SYSCLK_FREQ_288MHZ_HICK;
        config.pll.as_mut().unwrap().ns = 36;
        let _ = config.clocks();
    }
}
