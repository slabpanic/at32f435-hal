//! Advanced timer: `ComplementaryPwm` on TMR1 with dead time, software brake
//! and brake pin, plus the master-mode TRGOUT chain to a slave timer capture.
//!
//! PA8 = TMR1_CH1 (upper), PA7 = TMR1_CH1C (complementary), PA6 = brake input
//! (configured with a pull-up so the idle level does not assert the brake).
//! The second half of the example feeds TMR1 overflow into TMR2 via the
//! internal trigger chain and captures it with `InputCapture`.

#![no_std]
#![no_main]

use at32f435_hal as hal;
use defmt_rtt as _;
use embassy_executor::main;
use embassy_time::{Duration, Timer};
use hal::time::Hertz;
use hal::timer::{
    BrakeConfig, CaptureConfig, Channel, ComplementaryChannel, ComplementaryPwm, InputCapture,
};
use panic_probe as _;

defmt::timestamp!("{=u64:us}", hal::embassy::now_us());

hal::bind_interrupts!(struct Irqs {
    TMR2 => hal::timer::GeneralInterruptHandler<hal::peripherals::TMR2>;
});

#[main]
async fn main(_spawner: embassy_executor::Spawner) {
    let p = hal::init(hal::rcc::Config::SYSCLK_FREQ_288MHZ_HEXT_8M);
    defmt::info!(
        "timer_advanced up: {} MHz",
        hal::rcc::clocks().sysclk.0 / 1_000_000
    );

    // ---- TMR1: complementary pair + 2 us dead time ----
    let mut adv = ComplementaryPwm::new(
        p.TMR1,
        Hertz::hz(1_000),
        2_000, // dead time in ns (encoder-coded into DTC ticks)
        BrakeConfig::default(),
    );
    adv.enable_ch1_pair(p.PA8, p.PA7); // CH1 / CH1C on the headers
    adv.set_duty(ComplementaryChannel::Ch1, adv.period() / 2);
    adv.enable_output();
    adv.set_master_output(2); // PTOS=010: TRGOUT = overflow event (for the capture chain)
    defmt::info!(
        "tmr1: period={} ticks, dead_ticks={}, poen={}",
        adv.period(),
        adv.dead_time_ticks(),
        adv.output_enabled()
    );

    // ---- software brake + recover ----
    // The brake input pin PA6 is pulled up: idle level high = brake not asserted.
    adv.enable_brake_pin(p.PA6, hal::gpio::Pull::Up);
    adv.enable_brake_input(BrakeConfig {
        active_high: false,
        idle_output_enable: false,
        disable_output_enable: false,
        auto_recover: false,
    });
    adv.trigger_brake_software();
    Timer::after(Duration::from_millis(1)).await;
    let braked = !adv.output_enabled() && adv.brake_flag();
    adv.clear_brake_flag();
    adv.enable_output();
    let recovered = adv.output_enabled();
    defmt::info!("tmr1 brake: braked={} recovered={}", braked, recovered);
    if braked && recovered {
        defmt::info!("tmr1 software brake PASS");
    } else {
        defmt::error!("tmr1 software brake FAIL");
    }

    // ---- TMR1 -> TMR2 internal trigger chain + STCI capture ----
    // TMR1 overflows at 1 kHz; TMR2 counts at 1 MHz (divider 288 on 288 MHz
    // timer clock) and captures the chained trigger - consecutive capture
    // deltas read ~1000 ticks per TMR1 period.
    let mut cap = InputCapture::new(p.TMR2, CaptureConfig::default());
    cap.set_clock_divider(287); // 288 MHz / 288 = 1 MHz count tick
    let mut found_itrx: Option<u8> = None;
    for itrx in 0u8..4 {
        cap.set_trigger_input(itrx);
        cap.capture_internal_trigger(Channel::Ch1);
        Timer::after(Duration::from_millis(3)).await;
        let mut hits = 0;
        for _ in 0..6 {
            if cap.peek(Channel::Ch1).is_some() {
                hits += 1;
            }
            Timer::after(Duration::from_millis(2)).await;
        }
        defmt::info!("tmr2 IS{}: capture hits in ~12 ms = {}", itrx, hits);
        if hits >= 2 {
            found_itrx = Some(itrx);
            break;
        }
    }
    match found_itrx {
        Some(itrx) => {
            let v1 = cap.capture(Channel::Ch1).await;
            let v2 = cap.capture(Channel::Ch1).await;
            let delta = (v2 as i32 - v1 as i32).abs();
            defmt::info!(
                "tmr2 capture via IS{}: delta {} (expect ~1000)",
                itrx,
                delta
            );
            if delta > 900 && delta < 1100 {
                defmt::info!("tmr1->tmr2 trigger chain PASS");
            } else {
                defmt::error!("tmr1->tmr2 trigger chain FAIL");
            }
        }
        None => defmt::error!("tmr2: no internal trigger carried TMR1 TRGOUT"),
    }

    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}
