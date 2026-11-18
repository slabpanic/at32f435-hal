//! ADC: internal reference channels (VINTRV / VTS), sequence mode and DMA
//! scan on ADC1 - no external wiring needed.
//!
//! VINTRV is the internal 1.2 V reference (used to back-calculate VDDA), VTS
//! the temperature sensor. The DMA scan runs 8 consecutive samples through
//! DMAMUX.

#![no_std]
#![no_main]

use at32f435_hal as hal;
use defmt_rtt as _;
use embassy_executor::main;
use embassy_time::{Duration, Timer};
use panic_probe as _;

defmt::timestamp!("{=u64:us}", hal::embassy::now_us());

hal::bind_interrupts!(struct Irqs {
    DMA2_CHANNEL1 => hal::dma::InterruptHandler<hal::peripherals::DMA2_CH1>;
});

#[main]
async fn main(_spawner: embassy_executor::Spawner) {
    let p = hal::init(hal::rcc::Config::SYSCLK_FREQ_288MHZ_HEXT_8M);
    defmt::info!("adc up: {} MHz", hal::rcc::clocks().sysclk.0 / 1_000_000);

    let mut adc = hal::adc::Adc::new(p.ADC1, hal::adc::Config::default());

    // ---- internal channels ----
    let vintrv = adc.blocking_read_vintrv(hal::adc::SampleTime::Cycles47_5);
    let vts = adc.blocking_read_vts(hal::adc::SampleTime::Cycles47_5);
    let vdda_mv = if vintrv > 0 {
        (1_200_000u64 * 4095 / vintrv as u64 / 1000) as u32
    } else {
        0
    };
    defmt::info!("adc vintrv={} (vdda ~{} mV) vts={}", vintrv, vdda_mv, vts);

    // ---- sequence mode: [VINTRV, VTS] in one software sweep ----
    let mut seq = [0u16; 2];
    let seq_ok = adc
        .blocking_read_sequence(
            &[hal::adc::AdcChannel::Vintrv, hal::adc::AdcChannel::Vts],
            hal::adc::SampleTime::Cycles47_5,
            &mut seq,
        )
        .is_ok()
        && seq[0] > 0
        && seq[1] > 0;
    defmt::info!("adc sequence [vintrv, vts] = [{}, {}]", seq[0], seq[1]);

    // ---- DMA scan: 8 consecutive VINTRV samples ----
    let mut dma = hal::dma::Channel::new(p.DMA2_CH1);
    let mut buf = [0u16; 8];
    // SAFETY: buf is in SRAM and the DMA future is awaited to completion.
    let scan_ok = unsafe {
        adc.scan_dma(
            &[hal::adc::AdcChannel::Vintrv; 8],
            hal::adc::SampleTime::Cycles47_5,
            &mut buf,
            &mut dma,
        )
        .await
        .is_ok()
    } && buf.iter().all(|&v| v > 0);
    defmt::info!(
        "adc dma scan vintrv x8: {} {} {} {} {} {} {} {}",
        buf[0],
        buf[1],
        buf[2],
        buf[3],
        buf[4],
        buf[5],
        buf[6],
        buf[7]
    );

    let ratio_x100 = (vts as u32 * 100) / vintrv.max(1) as u32;
    let internal_ok = vintrv > 800 && vts > 800 && (100..=130).contains(&ratio_x100);
    if internal_ok {
        defmt::info!(
            "adc internal channels PASS (vts/vintrv ratio x100 = {})",
            ratio_x100
        );
    } else {
        defmt::error!("adc internal channels FAIL (ratio x100 = {})", ratio_x100);
    }
    if seq_ok {
        defmt::info!("adc sequence PASS");
    } else {
        defmt::error!("adc sequence FAIL");
    }
    if scan_ok {
        defmt::info!("adc dma scan PASS");
    } else {
        defmt::error!("adc dma scan FAIL");
    }

    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}
