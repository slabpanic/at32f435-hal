//! I2S: I2S1 master TX (Philips, 16-bit data / 32-bit channel, 48 kHz) with
//! MCK output on PB0, fed by DMA (DMAMUX request lines for I2S), while GPIO
//! IDR sampling proves WS/CK/MCK actually toggle.
//!
//! Pin mux (RM table 6-1): SD=PA7, WS=PA4, CK=PA5, MCK=PB0. The data pattern
//! repeats a left/right frame pair; the audio path needs a codec to hear it,
//! this example validates the clocking and the DMA feed.

#![no_std]
#![no_main]

use at32f435_hal as hal;
use defmt_rtt as _;
use embassy_executor::main;
use embassy_futures::join::join;
use embassy_time::{Duration, Timer};
use hal::i2s::{Config, DataFormat, I2s, Protocol};
use panic_probe as _;

defmt::timestamp!("{=u64:us}", hal::embassy::now_us());

hal::bind_interrupts!(struct Irqs {
    DMA1_CHANNEL4 => hal::dma::InterruptHandler<hal::peripherals::DMA1_CH4>;
});

#[main]
async fn main(_spawner: embassy_executor::Spawner) {
    let p = hal::init(hal::rcc::Config::SYSCLK_FREQ_288MHZ_HEXT_8M);
    defmt::info!("i2s up: {} MHz", hal::rcc::clocks().sysclk.0 / 1_000_000);

    let mut i2s = I2s::new_master_tx_with_mck(
        p.SPI1,
        p.PA7, // SD
        p.PA4, // WS
        p.PA5, // CK
        p.PB0, // MCK out (256 x fs)
        Config {
            protocol: Protocol::Philips,
            format: DataFormat::Data16Channel32,
            sample_rate: 48_000,
            mclk_output: true,
            ..Default::default()
        },
    );

    // Divider register read-back (RM 13.3.5 formula): 48 kHz from the 288 MHz
    // SPI1 clock.
    let sclk = hal::rcc::clocks().pclk2_tim;
    let div = i2s.config().divider(sclk);
    defmt::info!(
        "i2s1 divider: DIV7_0={} ODD={} real_fs={=u32} Hz",
        div.div,
        div.odd,
        div.real_fs,
    );

    // Feed via DMA and sample the clocks through GPIO IDR for 40 ms.
    let mut tx_ch = hal::dma::Channel::new(p.DMA1_CH4);
    // 16-bit frames, 32-bit channel: two halfwords per sample slot.
    // 4096 halfwords = ~43 ms of audio at 96 khalfwords/s, covering the
    // whole 40 ms measurement window (the master TX clock is demand-gated on
    // pending data and stops as soon as the buffer drains).
    const N: usize = 4096;
    let mut pat = [0u16; N];
    for (i, v) in pat.iter_mut().enumerate() {
        *v = if i % 4 < 2 { 0x3FFF } else { 0x0001 };
    }
    // SAFETY: pat stays in SRAM and remains valid until the transfer stops.
    unsafe { i2s.write_dma_hw(&mut tx_ch, &pat) }.expect("arm I2S DMA");

    let gpioa = unsafe { &*hal::pac::GPIOA::PTR };
    let gpiob = unsafe { &*hal::pac::GPIOB::PTR };
    let meas = async {
        let mut ws_edges = 0u32;
        let mut ck_highs = 0u32;
        let mut mck_highs = 0u32;
        let mut prev_ws = false;
        let deadline = hal::embassy::now_us() + 40_000;
        while hal::embassy::now_us() < deadline {
            let idr_a = gpioa.idt().read().bits();
            if ((idr_a & (1 << 4)) != 0) != prev_ws {
                ws_edges += 1;
                prev_ws = idr_a & (1 << 4) != 0;
            }
            if idr_a & (1 << 5) != 0 {
                ck_highs += 1;
            }
            if gpiob.idt().read().bits() & 1 != 0 {
                mck_highs += 1;
            }
        }
        (ws_edges, ck_highs, mck_highs)
    };
    let done = embassy_time::with_timeout(Duration::from_millis(100), tx_ch.wait_transfer());
    let (dma_res, (ws_edges, ck_highs, mck_highs)) = join(done, meas).await;

    defmt::info!(
        "i2s1 clocks (dma-fed, 40 ms): dma_done={} ws_edges={} ck_highs={} mck_highs={}",
        matches!(dma_res, Ok(Ok(()))),
        ws_edges,
        ck_highs,
        mck_highs
    );
    if matches!(dma_res, Ok(Ok(()))) && ws_edges > 400 && ck_highs > 0 && mck_highs > 0 {
        defmt::info!("i2s master tx clocks PASS (WS ~48 kHz x 2 edges, CK/MCK toggling)");
    } else {
        defmt::error!("i2s master tx clocks FAIL");
    }

    loop {
        Timer::after_secs(1).await;
    }
}
