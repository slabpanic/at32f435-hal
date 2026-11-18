//! GPIO output: heartbeat blink on the evb red LED (D2 = PA0, active low).
//!
//! Demonstrates `Output`, `Level`, `Speed` and the embassy time driver.

#![no_std]
#![no_main]

use at32f435_hal as hal;
use defmt_rtt as _;
use embassy_executor::main;
use embassy_time::{Duration, Timer};
use hal::gpio::{Level, Output, Speed};
use panic_probe as _;

defmt::timestamp!("{=u64:us}", hal::embassy::now_us());

#[main]
async fn main(_spawner: embassy_executor::Spawner) {
    let p = hal::init(hal::rcc::Config::SYSCLK_FREQ_288MHZ_HEXT_8M);
    defmt::info!(
        "blink up: sysclk {} MHz, apb1 {} MHz, apb2 {} MHz",
        hal::rcc::clocks().sysclk.0 / 1_000_000,
        hal::rcc::clocks().pclk1.0 / 1_000_000,
        hal::rcc::clocks().pclk2.0 / 1_000_000,
    );

    // D2 red = PA0, on-board LED wired active low.
    let mut led = Output::new(p.PA0, Level::High, Speed::Low);

    loop {
        led.toggle();
        Timer::after(Duration::from_millis(500)).await;
    }
}
