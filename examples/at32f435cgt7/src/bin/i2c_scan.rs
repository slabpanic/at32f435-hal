//! I2C: bus probe on I2C1 (SCL=PB6, SDA=PB7 @ 100 kHz).
//!
//! Connect a 7-bit peripheral to PB6/PB7 with external pull-ups. The scan
//! sends an address-only write so it does not read or program device data.

#![no_std]
#![no_main]

use at32f435_hal as hal;
use defmt_rtt as _;
use embassy_executor::main;
use embassy_time::{Duration, Timer};
use hal::i2c::{self, I2c};
use hal::time::Hertz;
use panic_probe as _;

defmt::timestamp!("{=u64:us}", hal::embassy::now_us());

#[main]
async fn main(_spawner: embassy_executor::Spawner) {
    let p = hal::init(hal::rcc::Config::SYSCLK_FREQ_288MHZ_HEXT_8M);
    defmt::info!(
        "i2c_scan up: {} MHz",
        hal::rcc::clocks().sysclk.0 / 1_000_000
    );

    let mut i2c = I2c::new_blocking(
        p.I2C1,
        p.PB6,
        p.PB7,
        i2c::Config {
            freq: Hertz::hz(100_000),
            pull: hal::gpio::Pull::Up,
            ..Default::default()
        },
    );

    // Scan the 7-bit space one page at a time (0x08..0x78, skipping reserved).
    let mut found: [u8; 8] = [0; 8];
    let mut found_n = 0usize;
    let mut timeouts = 0usize;
    for addr in (0x08u8..0x78).step_by(8) {
        for a in addr..(addr + 8).min(0x78) {
            match i2c.blocking_probe(a) {
                Ok(true) => {
                    defmt::info!("i2c device ACK at {:#04x}", a);
                    if found_n < found.len() {
                        found[found_n] = a;
                        found_n += 1;
                    }
                }
                Ok(false) => {}                             // NACK: nothing there
                Err(hal::i2c::Error::ArbitrationLost) => {} // floating SDA
                Err(hal::i2c::Error::NoAcknowledge) => {}   // NACK flavor
                Err(hal::i2c::Error::Timeout) => timeouts += 1,
                Err(hal::i2c::Error::Bus) => {
                    defmt::warn!("i2c probe {:#04x}: bus error", a);
                }
                Err(_) => {
                    defmt::warn!("i2c probe {:#04x}: other error", a);
                }
            }
        }
    }

    if found_n > 0 {
        defmt::info!(
            "i2c scan found {} device(s), first at {:#04x}",
            found_n,
            found[0]
        );
    } else if timeouts == 0 {
        defmt::info!("i2c scan: no devices (open bus handled without hangs) PASS");
    } else {
        defmt::error!("i2c scan: {} timeout(s) - driver/bus fault", timeouts);
    }

    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}
