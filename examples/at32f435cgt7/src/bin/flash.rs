//! FLASH controller: layout query, USD (option bytes) read, and an
//! erase + program + verify round trip on the last sector.
//!
//! Writes go through `unlock()` -> `blocking_write_sector()` -> `lock()`;
//! the USD read shows the factory option bytes (FAP protection state etc.).

#![no_std]
#![no_main]

use at32f435_hal as hal;
use defmt_rtt as _;
use embassy_executor::main;
use embassy_time::{Duration, Timer};
use panic_probe as _;

defmt::timestamp!("{=u64:us}", hal::embassy::now_us());

#[main]
async fn main(_spawner: embassy_executor::Spawner) {
    let p = hal::init(hal::rcc::Config::SYSCLK_FREQ_288MHZ_HEXT_8M);
    defmt::info!("flash up: {} MHz", hal::rcc::clocks().sysclk.0 / 1_000_000);

    let mut flash = hal::flash::Flash::new(p.FLASH);

    // ---- layout + USD read ----
    let layout = flash.layout();
    defmt::info!(
        "flash: {} KB, sector {} B, bank2 @{}",
        layout.total_size / 1024,
        layout.sector_size,
        layout.bank2_start.unwrap_or(0)
    );
    let usd = flash.usd();
    defmt::info!(
        "usd: fap={} ssb={:#x} d0={:#x} d1={:#x} usderr={}",
        usd.fap,
        usd.ssb,
        usd.user_d0,
        usd.user_d1,
        usd.usderr
    );

    // ---- erase + program + verify the last sector ----
    let last = layout.num_sectors() - 1;
    let payload: &[u8] = b"AT32F435 flash example";
    extern "C" {
        static _flash_scratch_start: u8;
    }
    let scratch = core::ptr::addr_of!(_flash_scratch_start) as u32;
    assert_eq!(layout.total_size, 1024 * 1024);
    assert_eq!(layout.sector_size, 2048);
    assert_eq!(layout.sector_addr(last), scratch);

    flash.unlock();
    // SAFETY: memory.x reserves this exact sector outside the firmware.
    let ok = unsafe { flash.blocking_write_sector(last, payload) }.is_ok();
    flash.lock();
    if ok {
        defmt::info!("flash erase+program+verify PASS (last sector)");
    } else {
        defmt::error!("flash erase+program+verify FAIL");
    }

    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}
