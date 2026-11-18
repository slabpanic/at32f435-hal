//! EXINT: async edge waiting with `ExtiInput`.
//!
//! Wiring (jumper): PA1 -> PB0. PA1 is driven by software in the pattern
//! below and PB0 (EXINT line 0) waits for rising/falling edges asynchronously.
//! Without the jumper the example waits forever on the first edge - the LED
//! keeps blinking through a separate task so liveness is visible either way.
//!
//! The 7 EXINT vector handlers are owned by the HAL's `exti` feature (default
//! on); the application never binds them via `bind_interrupts!`.

#![no_std]
#![no_main]

use at32f435_hal as hal;
use defmt_rtt as _;
use embassy_executor::{main, task, Spawner};
use embassy_time::{Duration, Timer};
use hal::exti::ExtiInput;
use hal::gpio::{Level, Output, Pull, Speed};
use panic_probe as _;

defmt::timestamp!("{=u64:us}", hal::embassy::now_us());

#[main]
async fn main(spawner: Spawner) {
    let p = hal::init(hal::rcc::Config::SYSCLK_FREQ_288MHZ_HEXT_8M);
    defmt::info!("exti up: {} MHz", hal::rcc::clocks().sysclk.0 / 1_000_000);

    spawner.spawn(heartbeat(p.PA0.into()).unwrap());

    // PB0 -> EXINT line 0 (line number == pin number). PA1 drives the wire.
    let mut input = ExtiInput::new(p.PB0, p.EXINT0, Pull::Down);
    let mut drive = Output::new(p.PA1, Level::Low, Speed::High);

    loop {
        for _ in 0..3 {
            drive.set_high();
            input.wait_for_high().await;
            drive.set_low();
            input.wait_for_low().await;
        }
        defmt::info!("exti: 3 rise+fall edges seen on PB0 (driven from PA1)");
        Timer::after(Duration::from_millis(200)).await;
    }
}

#[task]
async fn heartbeat(pin: hal::Peri<'static, hal::gpio::AnyPin>) {
    let mut led = Output::new(pin, Level::High, Speed::Low);
    loop {
        led.toggle();
        Timer::after(Duration::from_millis(500)).await;
    }
}
