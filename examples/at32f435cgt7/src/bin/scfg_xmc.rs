//! SCFG: the XMC address-mapping swap (SCFG_CFG1.SWAP_XMC, RM 7.2.1).
//!
//! The swap exchanges the chip-select address windows of the XMC memory
//! regions (e.g. SDRAM <-> 0x6000_0000/0x7000_0000). It only matters with
//! external memory hardware, but the register is a plain rw field, so the
//! example proves access with a write/read-back round trip.

#![no_std]
#![no_main]

use at32f435_hal as hal;
use defmt_rtt as _;
use embassy_executor::main;
use embassy_time::{Duration, Timer};
use hal::scfg::{self, XmcSwapMode};
use panic_probe as _;

defmt::timestamp!("{=u64:us}", hal::embassy::now_us());

#[main]
async fn main(_spawner: embassy_executor::Spawner) {
    let _p = hal::init(hal::rcc::Config::SYSCLK_FREQ_288MHZ_HEXT_8M);
    defmt::info!(
        "scfg_xmc up: {} MHz",
        hal::rcc::clocks().sysclk.0 / 1_000_000
    );

    let before = scfg::xmc_mapping_swap();
    scfg::set_xmc_mapping_swap(XmcSwapMode::Mode1);
    let mid = scfg::xmc_mapping_swap();
    scfg::set_xmc_mapping_swap(XmcSwapMode::None);
    let after = scfg::xmc_mapping_swap();

    defmt::info!(
        "swap_xmc: before={:?} mid={:?} after={:?}",
        before,
        mid,
        after
    );
    if mid == XmcSwapMode::Mode1 && after == XmcSwapMode::None {
        defmt::info!("scfg swap_xmc round-trip PASS");
    } else {
        defmt::error!("scfg swap_xmc round-trip FAIL");
    }

    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}
