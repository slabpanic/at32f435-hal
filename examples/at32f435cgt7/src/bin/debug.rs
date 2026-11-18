//! DEBUG block (RM 30): IDCODE, the low-power debug clock bits (so RTT
//! survives sleep/deepsleep/standby) and the debug-halt peripheral freeze
//! flags.
//!
//! All values are written and read back; everything is restored to reset
//! values at the end.

#![no_std]
#![no_main]

use at32f435_hal as hal;
use defmt_rtt as _;
use embassy_executor::main;
use embassy_time::{Duration, Timer};
use hal::debug::{Apb1Pause, Apb2Pause, Debug};
use panic_probe as _;

defmt::timestamp!("{=u64:us}", hal::embassy::now_us());

#[main]
async fn main(_spawner: embassy_executor::Spawner) {
    let p = hal::init(hal::rcc::Config::SYSCLK_FREQ_288MHZ_HEXT_8M);
    defmt::info!("debug up: {} MHz", hal::rcc::clocks().sysclk.0 / 1_000_000);

    let mut dbg = Debug::new(p.DEBUG);

    // ---- IDCODE ----
    defmt::info!(
        "idcode={=u32:#010x} (device={=u16:#06x} revision={=u16:#06x})",
        dbg.idcode(),
        dbg.device_id(),
        dbg.revision_id()
    );

    // ---- low-power debug bits: keep clocks alive while sleeping ----
    dbg.set_low_power_debug(true, true, false);
    if dbg.low_power_debug() == (true, true, false) {
        defmt::info!("low_power_debug set/readback PASS (sleep+deepsleep clocked)");
    } else {
        defmt::error!("low_power_debug FAIL");
    }
    dbg.set_low_power_debug(false, false, false);

    // ---- debug-halt freeze flags ----
    let flags1 = Apb1Pause::new()
        .with_wdt(true)
        .with_wwdt(true)
        .with_can1(true)
        .with_ertc512(true);
    dbg.set_apb1_pause(flags1);
    if dbg.apb1_pause() == flags1 {
        defmt::info!(
            "apb1 pause flags round-trip PASS ({=u32:#010x})",
            flags1.bits()
        );
    } else {
        defmt::error!("apb1 pause flags FAIL");
    }
    dbg.set_apb1_pause(Apb1Pause::new());

    let flags2 = Apb2Pause::new()
        .with_tmr1(true)
        .with_tmr20(true)
        .with_tmr9(true);
    dbg.set_apb2_pause(flags2);
    if dbg.apb2_pause() == flags2 {
        defmt::info!(
            "apb2 pause flags round-trip PASS ({=u32:#010x})",
            flags2.bits()
        );
    } else {
        defmt::error!("apb2 pause flags FAIL");
    }
    dbg.set_apb2_pause(Apb2Pause::new());

    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}
