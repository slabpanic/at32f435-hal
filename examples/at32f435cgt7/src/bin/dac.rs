//! DAC: DAC1 output (PA4) sampled back by ADC12_IN4 on the same pin, plus a
//! dual-channel synchronized write to DAC2 (PA5).
//!
//! PA4/PA5 carry both the DAC outputs and the ADC inputs, so the loopback
//! needs no wiring: the DAC drives the pin and the ADC samples it.

#![no_std]
#![no_main]

use at32f435_hal as hal;
use defmt_rtt as _;
use embassy_executor::main;
use embassy_time::{Duration, Timer};
use hal::dac::{Channel, ChannelConfig, Dac};
use panic_probe as _;

defmt::timestamp!("{=u64:us}", hal::embassy::now_us());

#[main]
async fn main(_spawner: embassy_executor::Spawner) {
    let mut p = hal::init(hal::rcc::Config::SYSCLK_FREQ_288MHZ_HEXT_8M);
    defmt::info!("dac up: {} MHz", hal::rcc::clocks().sysclk.0 / 1_000_000);

    let mut dac = Dac::new(p.DAC);
    dac.enable_channel(Channel::Ch1, &ChannelConfig::buffered());
    dac.enable_channel(Channel::Ch2, &ChannelConfig::buffered());
    let mut adc = hal::adc::Adc::new(p.ADC1, hal::adc::Config::default());

    // ---- DAC1 -> ADC IN4 same-pin loopback (output buffer on) ----
    const POINTS: [u16; 5] = [0x100, 0x400, 0x800, 0xC00, 0xF00];
    let mut prev: i32 = -1000;
    let mut all_ok = true;
    for &target in POINTS.iter() {
        dac.set_value_12bit(Channel::Ch1, target);
        Timer::after(Duration::from_millis(10)).await; // settle
        let v = adc.blocking_read(p.PA4.reborrow(), hal::adc::SampleTime::Cycles24_5);
        let err = (i32::from(v) - i32::from(target)).abs();
        let mono = i32::from(v) > prev;
        prev = i32::from(v);
        defmt::info!("dac1 target={} adc={} err={} mono={}", target, v, err, mono);
        if err > 100 || !mono {
            all_ok = false;
        }
    }
    if all_ok {
        defmt::info!("dac1->adc in4 loopback PASS (buffered, err <= 100 counts)");
    } else {
        defmt::error!("dac1->adc in4 loopback FAIL");
    }

    // ---- dual-channel synchronized write: read back both ODT registers ----
    dac.set_value_dual_12bit(0x200, 0xE00);
    Timer::after(Duration::from_millis(10)).await;
    let v1 = dac.output_value(Channel::Ch1);
    let v2 = dac.output_value(Channel::Ch2);
    defmt::info!("dual odt: ch1={} ch2={}", v1, v2);
    if v1 == 0x200 && v2 == 0xE00 {
        defmt::info!("dac dual sync write PASS");
    } else {
        defmt::error!("dac dual sync write FAIL");
    }

    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}
