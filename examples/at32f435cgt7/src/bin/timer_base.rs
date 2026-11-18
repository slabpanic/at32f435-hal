//! Basic timer: `TimerBase` on TMR6 with overflow interrupts awaited
//! asynchronously.
//!
//! 1000 overflows at 1 kHz should take ~1 s of wall clock time (measured with
//! the SysTick-backed embassy time driver, which is independent of TMR6).

#![no_std]
#![no_main]

use at32f435_hal as hal;
use defmt_rtt as _;
use embassy_executor::main;
use embassy_time::{Duration, Timer};
use hal::time::Hertz;
use hal::timer::TimerBase;
use panic_probe as _;

defmt::timestamp!("{=u64:us}", hal::embassy::now_us());

hal::bind_interrupts!(struct Irqs {
    TMR6 => hal::timer::BaseInterruptHandler<hal::peripherals::TMR6>;
});

#[main]
async fn main(_spawner: embassy_executor::Spawner) {
    let p = hal::init(hal::rcc::Config::SYSCLK_FREQ_288MHZ_HEXT_8M);
    defmt::info!(
        "timer_base up: {} MHz",
        hal::rcc::clocks().sysclk.0 / 1_000_000
    );

    let mut tmr6 = TimerBase::new(p.TMR6, Hertz::hz(1_000));
    tmr6.listen();

    // Local measurement: 1000 overflows timed against the SysTick driver.
    let t0 = hal::embassy::now_us();
    for _ in 0..1000 {
        tmr6.wait_update().await;
    }
    tmr6.unlisten();
    let wall_ms = (hal::embassy::now_us() - t0) / 1000;
    defmt::info!(
        "tmr6: 1000 overflows in {} ms wall clock (expect ~1000 ms)",
        wall_ms
    );
    if wall_ms > 900 && wall_ms < 1100 {
        defmt::info!("tmr6 time base PASS");
    } else {
        defmt::error!("tmr6 time base FAIL");
    }

    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}
