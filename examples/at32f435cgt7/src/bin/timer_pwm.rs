//! General-purpose timer PWM: `SimplePwm` on TMR2 (32-bit, PMEN), channels
//! CH1/CH2/CH3 on PA0/PA1/PA2 at 1 kHz, with polarity control and duty sweep.
//!
//! The evb wires PA0/PA1/PA2 to the RGB status LEDs, so the sweep is visible.
//! PA0 also carries the heartbeat convention of the other examples; this
//! example owns it exclusively.

#![no_std]
#![no_main]

use at32f435_hal as hal;
use defmt_rtt as _;
use embassy_executor::main;
use embassy_time::{Duration, Timer};
use hal::time::Hertz;
use hal::timer::{Channel, Polarity, PwmPin, SimplePwm};
use panic_probe as _;

defmt::timestamp!("{=u64:us}", hal::embassy::now_us());

#[main]
async fn main(_spawner: embassy_executor::Spawner) {
    let p = hal::init(hal::rcc::Config::SYSCLK_FREQ_288MHZ_HEXT_8M);
    defmt::info!(
        "timer_pwm up: {} MHz",
        hal::rcc::clocks().sysclk.0 / 1_000_000
    );

    let mut pwm = SimplePwm::new(
        p.TMR2,
        Some(PwmPin::new_ch1(p.PA0)), // TMR2_CH1 @ mux 1
        Some(PwmPin::new_ch2(p.PA1)),
        Some(PwmPin::new_ch3(p.PA2)),
        None,
        Hertz::hz(1_000),
    );
    pwm.set_polarity(Channel::Ch1, Polarity::ActiveLow); // LEDs are active low
    pwm.set_polarity(Channel::Ch2, Polarity::ActiveLow);
    pwm.set_polarity(Channel::Ch3, Polarity::ActiveLow);

    let max = pwm.get_max_duty();
    defmt::info!(
        "tmr2 pwm: max duty = {=u32} ticks @ 1 kHz (32-bit TMR2)",
        max
    );

    // Breathing sweep on all three channels (phase-shifted).
    loop {
        for step in 0..=100u32 {
            let duty = max * step / 100;
            pwm.set_duty(Channel::Ch1, max - duty); // red: inverse ramp
            pwm.set_duty(Channel::Ch2, duty);
            let ch3 = if step % 2 == 0 { max / 4 } else { max / 2 };
            pwm.set_duty(Channel::Ch3, ch3);
            Timer::after(Duration::from_millis(10)).await;
        }
    }
}
