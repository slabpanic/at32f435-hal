//! Timer extras: `OnePulse` (single-cycle pulse, TMR4) and `Qei` (quadrature
//! encoder interface, TMR3).
//!
//! Neither needs external wiring for the register-level checks shown here:
//! OnePulse is verified by its TMREN auto-clear timing, QEI by a stable count
//! with no encoder connected (swap in an encoder on PB4/PB5 to see it move).

#![no_std]
#![no_main]

use at32f435_hal as hal;
use defmt_rtt as _;
use embassy_executor::main;
use embassy_time::{Duration, Timer};
use hal::time::Hertz;
use hal::timer::{OnePulse, Qei};
use panic_probe as _;

defmt::timestamp!("{=u64:us}", hal::embassy::now_us());

#[main]
async fn main(_spawner: embassy_executor::Spawner) {
    let p = hal::init(hal::rcc::Config::SYSCLK_FREQ_288MHZ_HEXT_8M);
    defmt::info!(
        "timer_extras up: {} MHz",
        hal::rcc::clocks().sysclk.0 / 1_000_000
    );

    // ---- OnePulse: TMR4 CH1 @ PB6 (mux 2), 5 ms cycle, 50% duty ----
    let mut op = OnePulse::new_ch1(
        p.TMR4,
        p.PB6,
        Hertz::hz(1_000_000),
        5_000, // 5 ms period at a 1 us tick
        2_500, // 50% duty
        None,
    );
    op.start();
    let running_at_start = op.is_running();
    Timer::after(Duration::from_millis(8)).await;
    let stopped_after = !op.is_running();
    defmt::info!(
        "tmr4 one-pulse: running@start={} stopped@8ms={}",
        running_at_start,
        stopped_after
    );
    if running_at_start && stopped_after {
        defmt::info!("tmr4 one pulse PASS (TMREN auto-clears after the cycle)");
    } else {
        defmt::error!("tmr4 one pulse FAIL");
    }

    // ---- QEI: TMR3 CH1/CH2 @ PB4/PB5 (mux 2) ----
    let mut qei = Qei::new(p.TMR3, p.PB4, p.PB5);
    let c0 = qei.count();
    Timer::after(Duration::from_millis(5)).await;
    let c1 = qei.count();
    defmt::info!("tmr3 qei: count {} -> {} (no encoder, stable)", c0, c1);
    if c0 == 0 && c1 == 0 {
        defmt::info!("tmr3 qei config PASS (wire an encoder to PB4/PB5 to count)");
    } else {
        defmt::info!("tmr3 qei counting (encoder connected?)");
    }

    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}
