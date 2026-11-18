//! Window watchdog (WWDT): feed inside the window, prove that feeding early
//! resets the chip.
//!
//! The example uses the two-phase flow of the original evb validation: after
//! a WWDT reset the firmware detects the reset flag (CRM_CTRLSTS.WWDTRSTF)
//! and reports the pass; a fresh boot feeds in-window 20 times and then
//! deliberately feeds early. Under `cargo run -p at32f435cgt7-examples --bin wwdt` probe-rs
//! reattaches after the reset and shows both halves.

#![no_std]
#![no_main]

use at32f435_hal as hal;
use defmt_rtt as _;
use embassy_executor::main;
use embassy_time::{Duration, Timer};
use hal::wwdt::WindowWdt;
use panic_probe as _;

defmt::timestamp!("{=u64:us}", hal::embassy::now_us());

#[main]
async fn main(_spawner: embassy_executor::Spawner) {
    let p = hal::init(hal::rcc::Config::SYSCLK_FREQ_288MHZ_HEXT_8M);

    // Read (and clear) the reset flags: CRM_CTRLSTS.WWDTRSTF is set when the
    // window watchdog caused the last reset.
    let crm = unsafe { &*hal::pac::CRM::PTR };
    let sts = crm.ctrlsts().read();
    let wwdt_reset = sts.wwdtrstf().bit_is_set();
    crm.ctrlsts().modify(|_, w| w.rstfc().set_bit());

    // 13 ms timeout, window opens at 6 ms (PCLK1-derived, auto divider).
    let mut wwdt = WindowWdt::start(p.WWDT, 13, 6).expect("wwdt start");
    defmt::info!(
        "wwdt up: timeout 13 ms, window 6 ms, wwdrstf={}, counter={:#x}",
        wwdt_reset,
        wwdt.counter()
    );

    if wwdt_reset {
        // The previous run fed early on purpose and was reset - complete.
        defmt::info!("wwdt: early-feed reset observed - full sequence PASS");
        loop {
            Timer::after(Duration::from_secs(1)).await;
        }
    }

    // Phase 1: 20 feeds inside the window (>= 6 ms after each start).
    for i in 0..20u32 {
        Timer::after(Duration::from_millis(8)).await;
        wwdt.feed();
        if i % 5 == 4 {
            defmt::info!(
                "wwdt: {} in-window feeds, counter={:#x}, no reset",
                i + 1,
                wwdt.counter()
            );
        }
    }

    // Phase 2: feed immediately - the counter is still below the window, so
    // the hardware must reset; the reboot lands in the reset-flag branch.
    defmt::info!("wwdt: feeding EARLY (2 ms) to prove the window reset");
    Timer::after(Duration::from_millis(2)).await;
    wwdt.feed();
    loop {
        Timer::after(Duration::from_millis(50)).await;
    }
}
