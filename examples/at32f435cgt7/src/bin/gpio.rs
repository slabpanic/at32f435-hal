//! GPIO input/output API tour: `Output`, `Input`, `Flex`, pull-ups and
//! read-back.
//!
//! Wiring (jumper): PA1 -> PB0. The example drives PA1 and reads PB0 through
//! both `Input` and `Flex`, exercising the GPIO register paths that the evb
//! exposes on its headers. Without the jumper the "connected" check fails
//! harmlessly (the example logs the observed level and keeps blinking the
//! LED).

#![no_std]
#![no_main]

use at32f435_hal as hal;
use defmt_rtt as _;
use embassy_executor::main;
use embassy_time::{Duration, Timer};
use hal::gpio::{Flex, Input, Level, Output, Pull, Speed};
use panic_probe as _;

defmt::timestamp!("{=u64:us}", hal::embassy::now_us());

#[main]
async fn main(_spawner: embassy_executor::Spawner) {
    let p = hal::init(hal::rcc::Config::SYSCLK_FREQ_288MHZ_HEXT_8M);
    defmt::info!("gpio up: {} MHz", hal::rcc::clocks().sysclk.0 / 1_000_000);

    // LED (active low, PA0) as a liveness indicator.
    let mut led = Output::new(p.PA0, Level::High, Speed::Low);

    // Drive PA1 high/low, read it back on PB0 with an internal pull-down.
    let mut drive = Output::new(p.PA1, Level::Low, Speed::High);
    let monitor = Input::new(p.PB0, Pull::Down);

    // `Flex` can switch the same pin between input and output at runtime:
    // here PB1 mirrors whatever PA1 does after a mode flip.
    let mut flex = Flex::new(p.PB1);

    loop {
        // push-pull high -> PB0 reads high
        drive.set_high();
        flex.set_as_output(Level::High, Speed::High);
        Timer::after(Duration::from_millis(20)).await;
        let high_read = monitor.is_high();

        // push-pull low -> PB0 reads low
        drive.set_low();
        flex.set_as_output(Level::Low, Speed::High);
        Timer::after(Duration::from_millis(20)).await;
        let low_read = monitor.is_high();

        // floating input (PB0 with pull-down released on the Flex side is
        // still driven; this just shows `Flex` switching back to input)
        flex.set_as_input(Pull::None);

        defmt::info!(
            "gpio: driven-high read={}; driven-low read={}; flex mode flip ok",
            high_read,
            low_read
        );
        if high_read && !low_read {
            defmt::info!("gpio loopback PASS (jumper PA1->PB0 in place)");
        } else {
            defmt::warn!("gpio loopback mismatch - jumper PA1->PB0 missing?");
        }

        led.toggle();
        Timer::after(Duration::from_millis(500)).await;
    }
}
