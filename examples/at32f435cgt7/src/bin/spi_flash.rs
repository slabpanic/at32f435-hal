//! SPI: W25Q64 flash on SPI2 (SCK=PB13, MISO=PB14, MOSI=PB15, CS=PB12),
//! blocking JEDEC ID read plus the async-DMA erase / page program / verify
//! round trip.

#![no_std]
#![no_main]

use at32f435_hal as hal;
use defmt_rtt as _;
use embassy_executor::main;
use embassy_time::{Duration, Timer};
use hal::gpio::{Level, Output, Speed};
use panic_probe as _;

defmt::timestamp!("{=u64:us}", hal::embassy::now_us());

hal::bind_interrupts!(struct Irqs {
    DMA1_CHANNEL4 => hal::dma::InterruptHandler<hal::peripherals::DMA1_CH4>;
    DMA1_CHANNEL5 => hal::dma::InterruptHandler<hal::peripherals::DMA1_CH5>;
});

/// Poll the WIP flag (SR bit 0) while keeping CS low for each command.
async fn wait_wip<T: hal::spi::Instance, TXD: hal::spi::TxDma<T>, RXD: hal::spi::RxDma<T>>(
    spi: &mut hal::spi::SpiDma<'_, T, TXD, RXD>,
    cs: &mut Output<'_>,
) -> bool {
    for _ in 0..2_000 {
        let tx = [0x05, 0xff];
        let mut rx = [0; 2];
        cs.set_low();
        // SAFETY: Both buffers remain in SRAM until the awaited transfer ends.
        let result = unsafe { spi.transfer(&mut rx, &tx).await };
        cs.set_high();
        if result.is_err() {
            return false;
        }
        if rx[1] & 1 == 0 {
            return true;
        }
        Timer::after_millis(1).await;
    }
    false
}

#[main]
async fn main(_spawner: embassy_executor::Spawner) {
    let mut p = hal::init(hal::rcc::Config::SYSCLK_FREQ_288MHZ_HEXT_8M);
    defmt::info!(
        "spi_flash up: {} MHz",
        hal::rcc::clocks().sysclk.0 / 1_000_000
    );

    // ---- blocking: JEDEC ID (0x9F -> EF 40 17 for a W25Q64) ----
    let mut cs = Output::new(p.PB12, Level::High, Speed::High);
    {
        let mut spi = hal::spi::Spi::new_blocking(
            p.SPI2.reborrow(),
            p.PB13.reborrow(),
            p.PB14.reborrow(),
            p.PB15.reborrow(),
            hal::spi::Config {
                sck_freq: hal::time::Hertz::hz(2_000_000),
                ..Default::default()
            },
        );
        cs.set_low();
        let mut jedec = [0u8; 3];
        let jedec_ok = {
            let w = spi.blocking_write(&[0x9F]);
            let r = spi.blocking_read(&mut jedec);
            w.is_ok() && r.is_ok()
        };
        cs.set_high();
        defmt::info!(
            "w25q jedec id: {:#x} {:#x} {:#x}",
            jedec[0],
            jedec[1],
            jedec[2]
        );
        if jedec_ok && jedec == [0xEF, 0x40, 0x17] {
            defmt::info!("spi blocking jedec PASS");
        } else {
            defmt::error!("spi blocking jedec FAIL (flash missing?)");
        }
    } // The blocking driver releases its borrows before creating SpiDma.

    // ---- async DMA: erase last sector -> page program -> DMA read verify ----
    let mut spi = hal::spi::SpiDma::new(
        p.SPI2,
        p.PB13,
        p.PB14,
        p.PB15,
        p.DMA1_CH4,
        p.DMA1_CH5,
        hal::spi::Config {
            sck_freq: hal::time::Hertz::hz(4_000_000),
            ..Default::default()
        },
    );
    const ADDR: [u8; 3] = [0x7F, 0xF0, 0x00];
    let pattern: [u8; 16] = core::array::from_fn(|i| i as u8 * 13 + 7);
    let mut back = [0u8; 16];
    let result = async {
        let write_enable = [0x06];
        let erase_cmd = [0x20, ADDR[0], ADDR[1], ADDR[2]];
        let mut program_cmd = [0u8; 4 + 16];
        program_cmd[..4].copy_from_slice(&[0x02, ADDR[0], ADDR[1], ADDR[2]]);
        program_cmd[4..].copy_from_slice(&pattern);
        let read_cmd = [0x03, ADDR[0], ADDR[1], ADDR[2]];
        for (command, wait_ready) in [
            (&write_enable[..], false),
            (&erase_cmd[..], true),
            (&write_enable[..], false),
            (&program_cmd[..], true),
        ] {
            cs.set_low();
            // SAFETY: Commands are borrowed from SRAM for the entire DMA operation.
            let transfer = unsafe { spi.write(command).await };
            cs.set_high();
            if transfer.is_err() || (wait_ready && !wait_wip(&mut spi, &mut cs).await) {
                return false;
            }
        }
        cs.set_low();
        // SAFETY: Both buffers are in SRAM and each DMA future is awaited.
        let transfer =
            unsafe { spi.write(&read_cmd).await.is_ok() && spi.read(&mut back).await.is_ok() };
        cs.set_high();
        transfer
    }
    .await;
    defmt::info!("w25q dma readback: {=[u8]:x}", back);
    if result && back == pattern {
        defmt::info!("spi dma page program+verify PASS");
    } else {
        defmt::error!("spi dma page program+verify FAIL");
    }

    loop {
        cs.set_high();
        Timer::after(Duration::from_secs(1)).await;
    }
}
