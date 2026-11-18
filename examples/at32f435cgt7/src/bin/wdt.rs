//! Independent watchdog (WDT): start with a 5 s nominal timeout and feed it
//! periodically.
//!
//! Note the on-silicon caveat (docs/pitfalls.md): the LICK clock that feeds
//! the WDT deviates far from its 40 kHz nominal value, so `timeout_ms` is a
//! nominal figure - measure the real timeout on the board before relying on
//! it. The example feeds every 500 ms; removing the feed resets the chip.

#![no_std]
#![no_main]

use at32f435_hal as hal;
use defmt_rtt as _;
use embassy_executor::main;
use embassy_time::{Duration, Timer};
use hal::wdg::IndependentWdt;
use panic_probe as _;

defmt::timestamp!("{=u64:us}", hal::embassy::now_us());

#[main]
async fn main(_spawner: embassy_executor::Spawner) {
    let p = hal::init(hal::rcc::Config::SYSCLK_FREQ_288MHZ_HEXT_8M);
    defmt::info!("wdt up: {} MHz", hal::rcc::clocks().sysclk.0 / 1_000_000);

    // Start the watchdog with a nominal 5 s timeout, then feed every 500 ms.
    let mut wdt = IndependentWdt::start(p.WDT, 5_000);
    defmt::info!("wdt running (nominal 5 s), feeding every 500 ms");

    let mut feeds = 0u32;
    loop {
        Timer::after(Duration::from_millis(500)).await;
        wdt.feed();
        feeds += 1;
        if feeds.is_multiple_of(20) {
            defmt::info!("wdt fed {} times without reset", feeds);
        }
    }
}
