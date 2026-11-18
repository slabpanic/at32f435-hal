//! Clock tree configuration: switch between the built-in RCC presets and
//! print the resulting clocks.
//!
//! This example uses the board's 8 MHz HEXT crystal. The 48 MHz direct
//! HICK configuration and the 64/288 MHz HICK PLL presets were also
//! verified to start on the EVB.

#![no_std]
#![no_main]

use at32f435_hal as hal;
use defmt_rtt as _;
use embassy_executor::main;
use embassy_time::{Duration, Timer};
use hal::gpio::{Level, Output, Speed};
use panic_probe as _;

defmt::timestamp!("{=u64:us}", hal::embassy::now_us());

fn log_clocks(tag: &str) {
    let c = hal::rcc::clocks();
    defmt::info!(
        "{}: sysclk={} MHz hclk={} MHz pclk1={} MHz pclk2={} MHz pclk1_tim={} MHz pclk2_tim={} MHz",
        tag,
        c.sysclk.0 / 1_000_000,
        c.hclk.0 / 1_000_000,
        c.pclk1.0 / 1_000_000,
        c.pclk2.0 / 1_000_000,
        c.pclk1_tim.0 / 1_000_000,
        c.pclk2_tim.0 / 1_000_000,
    );
}

#[main]
async fn main(_spawner: embassy_executor::Spawner) {
    // Presets (mutually exclusive, pick one at init):
    // - hal::rcc::Config::SYSCLK_FREQ_288MHZ_HEXT_8M  (evb default, below)
    // - hal::rcc::Config::SYSCLK_FREQ_144MHZ_HEXT_8M
    // - hal::rcc::Config::SYSCLK_FREQ_288MHZ_HICK
    // - hal::rcc::Config::SYSCLK_FREQ_64MHZ_HICK
    let p = hal::init(hal::rcc::Config::SYSCLK_FREQ_288MHZ_HEXT_8M);
    log_clocks("clocks");

    let mut led = Output::new(p.PA0, Level::High, Speed::Low);
    defmt::info!("usb 48 MHz derivation: PLL 288 MHz / USBDIV 6 (CRM_MISC2, half-step code 0xB)",);

    loop {
        led.toggle();
        Timer::after(Duration::from_millis(500)).await;
    }
}
