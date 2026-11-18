//! CRC: known-answer test for the hardware CRC-32 (MPEG-2 reference) with the
//! word/byte feed interfaces and register reset.
//!
//! The hardware data register only takes whole words: `feed_bytes` packs bytes
//! big-endian and zero-pads the tail, so "123456789" (+ one 0x00000000 word)
//! matches the software CRC-32/MPEG-2 value 0xAE24E09D.

#![no_std]
#![no_main]

use at32f435_hal as hal;
use defmt_rtt as _;
use embassy_executor::main;
use embassy_time::{Duration, Timer};
use hal::crc;
use panic_probe as _;

defmt::timestamp!("{=u64:us}", hal::embassy::now_us());

#[main]
async fn main(_spawner: embassy_executor::Spawner) {
    let p = hal::init(hal::rcc::Config::SYSCLK_FREQ_288MHZ_HEXT_8M);
    defmt::info!("crc up: {} MHz", hal::rcc::clocks().sysclk.0 / 1_000_000);

    let mut c = crc::Crc::new(
        p.CRC,
        crc::Config {
            initial_data: 0xFFFF_FFFF,
            ..crc::Config::default()
        },
    );

    // KAT: bytes (big-endian packed, zero-padded tail).
    let v = c.feed_bytes(b"123456789");
    defmt::info!(
        "crc feed_bytes(\"123456789\") = {=u32:#010x} (expect 0xae24e09d)",
        v
    );
    if v == 0xAE24_E09D {
        defmt::info!("crc kat PASS");
    } else {
        defmt::error!("crc kat FAIL");
    }

    // Reset restores the initial data (CTRL.RST); the KAT must repeat.
    c.reset();
    let v2 = c.feed_bytes(b"123456789");
    if v2 == 0xAE24_E09D {
        defmt::info!("crc reset-and-repeat PASS");
    } else {
        defmt::error!("crc reset-and-repeat FAIL ({=u32:#010x})", v2);
    }

    // Whole-word input (no padding): "12345678" as two big-endian words.
    c.reset();
    let v3 = c.feed_words(&[0x3132_3334, 0x3536_3738]);
    defmt::info!("crc feed_words = {=u32:#010x} (expect 0x49e3c2fb)", v3);
    if v3 == 0x49E3_C2FB {
        defmt::info!("crc whole-words PASS");
    } else {
        defmt::error!("crc whole-words FAIL");
    }

    // value() read-back equals the last feed result.
    if c.value() == v3 {
        defmt::info!("crc value read-back PASS");
    } else {
        defmt::error!("crc value read-back FAIL");
    }

    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}
