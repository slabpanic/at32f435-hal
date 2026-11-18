//! ERTC: LEXT-backed calendar, backup registers and the asynchronous alarm-A
//! event (via the ERTCALARM EXINT line).
//!
//! Sets the calendar, watches it tick for ~2 s, writes/reads a BPR slot and
//! waits for alarm A two seconds ahead (only the seconds field compared).

#![no_std]
#![no_main]

use at32f435_hal as hal;
use defmt_rtt as _;
use embassy_executor::main;
use embassy_time::{Duration, Timer};
use hal::ertc::{Alarm, AlarmMask, ClockSource, Ertc, Event, Timestamp, Weekday};
use panic_probe as _;

defmt::timestamp!("{=u64:us}", hal::embassy::now_us());

hal::bind_interrupts!(struct Irqs {
    ERTCALARM => hal::ertc::AlarmHandler;
    TAMPER => hal::ertc::InterruptHandler;
    ERTC_WKUP => hal::ertc::WakeupHandler;
});

#[main]
async fn main(_spawner: embassy_executor::Spawner) {
    let p = hal::init(hal::rcc::Config::SYSCLK_FREQ_288MHZ_HEXT_8M);
    defmt::info!("ertc up: {} MHz", hal::rcc::clocks().sysclk.0 / 1_000_000);

    let mut ertc = match Ertc::new(p.ERTC, ClockSource::Lext) {
        Ok(e) => e,
        Err(e) => {
            defmt::error!("ertc init failed: {:?} (LEXT missing?)", e);
            loop {
                Timer::after(Duration::from_secs(1)).await;
            }
        }
    };

    // ---- calendar set + tick ----
    ertc.set_calendar(Timestamp {
        year: 26,
        month: 11,
        day: 18,
        weekday: Weekday::Tuesday,
        hour: 12,
        minute: 0,
        second: 0,
    })
    .expect("set_calendar");
    let t0 = ertc.calendar();
    Timer::after(Duration::from_millis(2200)).await;
    let t1 = ertc.calendar();
    let delta = i32::from(t1.second) - i32::from(t0.second);
    defmt::info!(
        "ertc: 20{:02}-{:02}-{:02} {:02}:{:02}:{:02} -> delta {} s",
        t1.year,
        t1.month,
        t1.day,
        t1.hour,
        t1.minute,
        t1.second,
        delta
    );
    if (2..=3).contains(&delta) {
        defmt::info!("ertc lext calendar PASS");
    } else {
        defmt::error!("ertc lext calendar FAIL");
    }

    // ---- backup registers ----
    ertc.write_bpr(3, 0x5A_A5);
    if ertc.read_bpr(3) == 0x5A_A5 {
        defmt::info!("ertc bpr write-read PASS");
    } else {
        defmt::error!("ertc bpr write-read FAIL");
    }

    // ---- alarm A, async (mask everything except seconds, fire in 2 s) ----
    let target = (t1.second + 2) % 60;
    ertc.set_alarm(
        Alarm::A,
        AlarmMask {
            day: true,
            hours: true,
            minutes: true,
            seconds: false,
        },
        0,
        0,
        target,
    )
    .expect("set_alarm");
    ertc.clear_events(&[Event::AlarmA]);
    ertc.listen(&[Event::AlarmA]);
    let ev = ertc.wait_event(&[Event::AlarmA]).await;
    ertc.disable_alarm(Alarm::A).ok();
    defmt::info!("ertc alarm-a event: {:?}", ev);
    if ev == Event::AlarmA {
        defmt::info!("ertc alarm-a async PASS");
    } else {
        defmt::error!("ertc alarm-a async FAIL");
    }

    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}
