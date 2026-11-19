//! SPI1 master transfers with SCK on PA5, MISO on PA6, and MOSI on PA7.
//! Leave the pins unconnected. The example checks transfer completion and
//! discards the samples from MISO.

#![no_std]
#![no_main]

use at32f435_hal as hal;
use defmt_rtt as _;
use embassy_executor::main;
use embassy_time::{with_timeout, Duration, Timer};
use hal::spi::{Config, Spi, SpiDma};
use panic_probe as _;
use static_cell::StaticCell;

hal::bind_interrupts!(struct Irqs {
    DMA1_CHANNEL4 => hal::dma::InterruptHandler<hal::peripherals::DMA1_CH4>;
    DMA1_CHANNEL5 => hal::dma::InterruptHandler<hal::peripherals::DMA1_CH5>;
});

static DMA_TX: StaticCell<[u8; 8]> = StaticCell::new();
const PATTERN: [u8; 8] = [0xA5, 0x5A, 0x00, 0xFF, 0x81, 0x18, 0x33, 0xCC];

#[main]
async fn main(_spawner: embassy_executor::Spawner) {
    let mut p = hal::init(hal::rcc::Config::SYSCLK_FREQ_288MHZ_HEXT_8M);
    let config = Config::default();

    {
        let mut spi = Spi::new_blocking(
            p.SPI1.reborrow(),
            p.PA5.reborrow(),
            p.PA6.reborrow(),
            p.PA7.reborrow(),
            config,
        );
        match spi.blocking_write(&PATTERN) {
            Ok(()) => defmt::info!("SPI blocking transfer complete"),
            Err(error) => defmt::error!(
                "SPI blocking transfer failed: {:?}",
                defmt::Debug2Format(&error)
            ),
        }
    }

    let mut spi = SpiDma::new(p.SPI1, p.PA5, p.PA6, p.PA7, p.DMA1_CH5, p.DMA1_CH4, config);
    let tx = DMA_TX.init(PATTERN);
    // SAFETY: The buffer lives in SRAM and is immutable during the transfer.
    // Dropping the transfer on timeout stops both DMA channels.
    match with_timeout(Duration::from_millis(100), unsafe { spi.write(tx) }).await {
        Ok(Ok(())) => defmt::info!("SPI DMA transfer complete"),
        Ok(Err(error)) => {
            defmt::error!("SPI DMA transfer failed: {:?}", defmt::Debug2Format(&error))
        }
        Err(_) => defmt::error!("SPI DMA transfer timed out"),
    }

    loop {
        Timer::after_secs(1).await;
    }
}
