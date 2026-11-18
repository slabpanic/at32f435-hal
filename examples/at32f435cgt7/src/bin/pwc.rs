//! Power control: deepsleep entry with an ERTC periodic-wakeup (EXINT22 /
//! ERTC_WKUP line).
//!
//! The board sleeps for about one second. The ERTC calendar verifies the
//! wake interval and the CRM status register verifies PLL recovery.
//!
//! Note: `enter_standby` is not exercised here (exiting standby resets the
//! chip, which would loop the example).

#![no_std]
#![no_main]

use at32f435_hal as hal;
use defmt_rtt as _;
use embassy_executor::main;
use embassy_time::{Duration, Timer};
use hal::ertc::{ClockSource, Ertc, Event, WakeupClock};
use hal::pwc::{DeepsleepLdo, Pwc};
use panic_probe as _;

defmt::timestamp!("{=u64:us}", hal::embassy::now_us());

hal::bind_interrupts!(struct Irqs {
    ERTC_WKUP => hal::ertc::WakeupHandler;
    ERTCALARM => hal::ertc::AlarmHandler;
    TAMPER => hal::ertc::InterruptHandler;
});

#[main]
async fn main(_spawner: embassy_executor::Spawner) {
    let p = hal::init(hal::rcc::Config::SYSCLK_FREQ_288MHZ_HEXT_8M);
    defmt::info!("pwc up: {} MHz", hal::rcc::clocks().sysclk.0 / 1_000_000);

    let mut ertc = match Ertc::new(p.ERTC, ClockSource::Lext) {
        Ok(e) => e,
        Err(e) => {
            defmt::error!("ertc init failed: {:?} (LEXT missing?)", e);
            loop {
                Timer::after(Duration::from_secs(1)).await;
            }
        }
    };
    let mut pwc = Pwc::new(p.PWC);

    // Wakeup timer: ERTC clock / 16 = 2048 Hz, 2048 counts = 1 s.
    ertc.set_wakeup(WakeupClock::ErtcClkDiv16, 2048)
        .expect("set_wakeup");
    ertc.listen(&[Event::Wakeup]);
    let sec0 = ertc.calendar().second;
    defmt::info!("entering deepsleep, wake in ~1 s (sec0={})", sec0);

    pwc.enter_deepsleep(DeepsleepLdo::Normal, None);

    // Woken by the ERTC: calendar advanced ~1 s and the HAL restored the system clock.
    let cal = ertc.calendar();
    let delta = (i32::from(cal.second) - i32::from(sec0)).rem_euclid(60);
    ertc.clear_events(&[Event::Wakeup]);
    ertc.disable_wakeup().ok();
    defmt::info!(
        "woke up: calendar sec {} -> {} (delta {} s)",
        sec0,
        cal.second,
        delta
    );
    // SAFETY: Read the CRM status register to verify the live clock source.
    let crm = unsafe { &*hal::pac::CRM::PTR };
    let pll_restored = crm.cfg().read().sclksts().is_pll();
    if (1..=3).contains(&delta) && pll_restored {
        defmt::info!("pwc deepsleep + ertc-wake PASS");
    } else {
        defmt::error!("pwc deepsleep + ertc-wake FAIL");
    }

    // Report the restored system clock.
    defmt::info!(
        "clocks after wake: PLL selected={}, configured sysclk={} MHz",
        pll_restored,
        hal::rcc::clocks().sysclk.0 / 1_000_000
    );

    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}
