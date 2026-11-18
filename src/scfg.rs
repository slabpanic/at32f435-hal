//! SCFG configuration beyond EXINT routing (RM 7): XMC address-mapping swap.
//!
//! `SCFG_CFG1.SWAP_XMC[11:10]` exchanges the chip-select address windows of the
//! XMC memory regions (RM 7.2.1). This only matters with external memory
//! hardware; it exists here because the register lives in SCFG, not XMC.
//!
//! The other SCFG_CFG1 fields are owned by their drivers: IR_POL / IR_SRC_SEL
//! by [`crate::irtmr`], MEM_MAP_SEL by the boot process (leave at reset).

use crate::pac;

/// XMC address-mapping swap mode (RM 7.2.1 SCFG_CFG1.SWAP_XMC).
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum XmcSwapMode {
    /// No swap (reset): NOR/PSRAM/SRAM/NAND2 at 0x6000_0000/0x6400_0000,
    /// SDRAM at 0x6800_0000/0x6C00_0000, NAND3 at 0xB000_0000.
    #[default]
    None = 0,
    /// SDRAM <-> 0x6000_0000/0x7000_0000 and NOR/PSRAM/SRAM/NAND2 <-> their
    /// 0xC000_0000/0xD000_0000 windows are exchanged.
    Mode1 = 1,
    /// QSPI2 <-> 0x8000_0000 and NAND3 <-> 0xB000_0000 are exchanged.
    Mode2 = 2,
    /// Mode1 and Mode2 combined.
    Mode3 = 3,
}

impl XmcSwapMode {
    fn bits(self) -> u8 {
        self as u8
    }

    fn from_bits(bits: u8) -> Self {
        match bits & 0x3 {
            1 => Self::Mode1,
            2 => Self::Mode2,
            3 => Self::Mode3,
            _ => Self::None,
        }
    }
}

/// Set the XMC address-mapping swap mode (same function as the official
/// `scfg_xmc_mapping_swap_set`). Only meaningful with external memory on the
/// XMC bus; harmless otherwise.
pub fn set_xmc_mapping_swap(mode: XmcSwapMode) {
    let scfg = unsafe { &*pac::SCFG::PTR };
    scfg.cfg1()
        .modify(|_, w| unsafe { w.swap_xmc().bits(mode.bits()) });
}

/// Read back the current XMC address-mapping swap mode.
pub fn xmc_mapping_swap() -> XmcSwapMode {
    let scfg = unsafe { &*pac::SCFG::PTR };
    XmcSwapMode::from_bits(scfg.cfg1().read().swap_xmc().bits())
}
