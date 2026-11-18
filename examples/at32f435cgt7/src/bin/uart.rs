//! UART: async echo on USART1 (PA9 = TX, PA10 = RX, 115200 8N1) with the
//! interrupt-driven path, plus a blocking write for the banner.
//!
//! Wire PA9/PA10 to a USB-TTL adapter (or the evb header) and any bytes sent
//! to the port come back echoed; every received packet is logged over RTT.

#![no_std]
#![no_main]

use at32f435_hal as hal;
use defmt_rtt as _;
use embassy_executor::main;
use hal::usart::{Config, Uart};
use panic_probe as _;

defmt::timestamp!("{=u64:us}", hal::embassy::now_us());

hal::bind_interrupts!(struct Irqs {
    USART1 => hal::usart::InterruptHandler<hal::peripherals::USART1>;
});

#[main]
async fn main(_spawner: embassy_executor::Spawner) {
    let p = hal::init(hal::rcc::Config::SYSCLK_FREQ_288MHZ_HEXT_8M);
    defmt::info!("uart up: {} MHz", hal::rcc::clocks().sysclk.0 / 1_000_000);

    let mut uart =
        Uart::new(p.USART1, p.PA9, p.PA10, Irqs, Config::default()).expect("usart config");

    uart.blocking_write(b"uart example ready, type anything\r\n")
        .ok();

    loop {
        // The async read fills the whole buffer before returning.
        let mut buf = [0u8; 64];
        uart.read(&mut buf).await.expect("uart read");
        defmt::info!("uart rx {} bytes", buf.len());
        uart.write(&buf).await.ok(); // echo back
    }
}
