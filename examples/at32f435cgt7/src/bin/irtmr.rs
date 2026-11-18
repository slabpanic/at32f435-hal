//! IRTMR: infrared transmit envelope via TMR10 CH1 + TMR11 CH1 carrier,
//! OR-ed in hardware under SCFG_CFG1.IR_POL, output on PB9.
//!
//! This example configures a 38 kHz carrier modulated by a 1 kHz envelope
//! (the standard NEC-style IR transmit shape) and exercises the polarity
//! control; to observe the signal, wire PB9 to an IR LED driver or a scope.

#![no_std]
#![no_main]

use at32f435_hal as hal;
use defmt_rtt as _;
use embassy_executor::main;
use embassy_time::{Duration, Timer};
use hal::irtmr::{ChannelPwm, IrTmr};
use hal::time::Hertz;
use panic_probe as _;

defmt::timestamp!("{=u64:us}", hal::embassy::now_us());

#[main]
async fn main(_spawner: embassy_executor::Spawner) {
    let p = hal::init(hal::rcc::Config::SYSCLK_FREQ_288MHZ_HEXT_8M);
    defmt::info!("irtmr up: {} MHz", hal::rcc::clocks().sysclk.0 / 1_000_000);

    // Envelope 1 kHz + carrier 38 kHz (NEC-style transmit shape); PB9 = IR_OUT
    // (TMR10_CH1 envelope + TMR11_CH1 carrier, hardware OR via SCFG IR_POL).
    let ir = IrTmr::new(
        p.TMR10,
        p.TMR11,
        p.PB9,
        ChannelPwm::new(Hertz::hz(1_000), 1, 3),
        ChannelPwm::new(Hertz::hz(38_000), 1, 2),
    );

    // Polarity flip-flop through SCFG_CFG1.IR_POL.
    ir.set_polarity(true);
    let set = ir.polarity_inverted();
    ir.set_polarity(false);
    let cleared = !ir.polarity_inverted();
    defmt::info!("irtmr: polarity set={} clear={}", set, cleared);
    if set && cleared {
        defmt::info!("irtmr polarity control PASS");
    } else {
        defmt::error!("irtmr polarity control FAIL");
    }

    defmt::info!("IR burst running on PB9 (1 kHz envelope / 38 kHz carrier)");
    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}
