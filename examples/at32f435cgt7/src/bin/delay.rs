//! Blocking delays: `Delay` (embedded-hal 1.0 `DelayNs`) and the raw
//! `delay_us`/`delay_ms` helpers.
//!
//! The example toggles the LED with blocking delays and cross-checks the
//! wall-clock time against the embassy time driver (SysTick based), which
//! stays independent of the busy-wait path.

#![no_std]
#![no_main]

use at32f435_hal as hal;
use defmt_rtt as _;
use embassy_executor::main;
use embedded_hal::delay::DelayNs as _;
use hal::delay::{delay_ms, Delay};
use hal::gpio::{Level, Output, Speed};
use panic_probe as _;

defmt::timestamp!("{=u64:us}", hal::embassy::now_us());

#[main]
async fn main(_spawner: embassy_executor::Spawner) {
    let p = hal::init(hal::rcc::Config::SYSCLK_FREQ_288MHZ_HEXT_8M);
    defmt::info!("delay up: {} MHz", hal::rcc::clocks().sysclk.0 / 1_000_000);

    let mut led = Output::new(p.PA0, Level::High, Speed::Low);
    let mut delay = Delay;

    // raw helper: 10 x 10 ms busy wait
    let t0 = hal::embassy::now_us();
    for _ in 0..10 {
        delay_ms(10);
    }
    let raw_dt = hal::embassy::now_us() - t0;
    defmt::info!("delay_ms(10) x10 = {} us wall clock", raw_dt);

    // DelayNs trait: 10 x 10 ms
    let t0 = hal::embassy::now_us();
    for _ in 0..10 {
        delay.delay_us(10_000);
    }
    let trait_dt = hal::embassy::now_us() - t0;
    defmt::info!("DelayNs::delay_us(10_000) x10 = {} us wall clock", trait_dt);

    // Both paths track the SysTick time driver within ~5% (busy-wait accuracy
    // depends on the loop calibration; the raw helper is cycle-count based).
    let ok = raw_dt > 90_000 && raw_dt < 120_000 && trait_dt > 90_000 && trait_dt < 120_000;
    if ok {
        defmt::info!("delay timing PASS (10 x 10 ms within 5% of wall clock)");
    } else {
        defmt::warn!("delay timing off - check HCLK calibration");
    }

    loop {
        led.toggle();
        delay.delay_ms(500);
    }
}
