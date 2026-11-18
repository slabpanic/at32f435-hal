//! USB device: CDC-ACM echo on OTGFS1 (D+ = PA12, D- = PA11).
//!
//! Plug the evb USB port into a PC: it enumerates as a virtual COM port and
//! echoes everything back. The red LED (PA0) blinks at 1 Hz as a liveness
//! indicator. AT32 differences handled inside the HAL (48 MHz USB clock,
//! GCCFG.VBUSIG, TRDT, core id from GUID@0x3C) are documented in
//! `src/usb/mod.rs`.

#![no_std]
#![no_main]

use at32f435_hal as hal;
use defmt_rtt as _;
use embassy_executor::{main, task, Spawner};
use embassy_time::{Duration, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State as CdcState};
use embassy_usb::{Builder, Config as UsbConfig};
use hal::peripherals::USB_OTG1;
use hal::usb::{Config as OtgConfig, Driver, InterruptHandler};
use panic_probe as _;
use static_cell::StaticCell;

defmt::timestamp!("{=u64:us}", hal::embassy::now_us());

hal::bind_interrupts!(struct Irqs {
    OTGFS1 => InterruptHandler<USB_OTG1>;
});

type UsbDriver = Driver<'static, USB_OTG1>;

#[main]
async fn main(spawner: Spawner) {
    let p = hal::init(hal::rcc::Config::SYSCLK_FREQ_288MHZ_HEXT_8M);
    defmt::info!(
        "usb_cdc up: {} MHz",
        hal::rcc::clocks().sysclk.0 / 1_000_000
    );

    // Staging buffer for OUT packets: must cover the sum of all OUT endpoint
    // max packet sizes.
    static EP_OUT_BUFFER: StaticCell<[u8; 512]> = StaticCell::new();
    let ep_out_buffer = EP_OUT_BUFFER.init([0u8; 512]);

    let driver = Driver::new(
        p.USB_OTG1,
        p.PA12,
        p.PA11,
        Irqs,
        ep_out_buffer,
        OtgConfig::default(), // no VBUS sensing on the evb (VBUSIG patched by the HAL)
    );

    static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static MSOS_DESC: StaticCell<[u8; 128]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
    let (config_desc, bos_desc, msos_desc, control_buf) = (
        CONFIG_DESC.init([0; 256]),
        BOS_DESC.init([0; 256]),
        MSOS_DESC.init([0; 128]),
        CONTROL_BUF.init([0; 64]),
    );

    let mut usb_config = UsbConfig::new(0xc0de, 0xcafe);
    usb_config.manufacturer = Some("at32f435-hal");
    usb_config.product = Some("USB CDC echo");

    static CDC_STATE: StaticCell<CdcState> = StaticCell::new();
    let cdc_state = CDC_STATE.init(CdcState::new());

    let mut builder = Builder::new(
        driver,
        usb_config,
        config_desc,
        bos_desc,
        msos_desc,
        control_buf,
    );
    let class = CdcAcmClass::new(&mut builder, cdc_state, 64);
    let device = builder.build();

    spawner.spawn(usb_task(device).unwrap());
    let (tx, rx) = class.split();
    spawner.spawn(echo_task(rx, tx).unwrap());
    spawner.spawn(heartbeat(p.PA0.into()).unwrap());
}

#[task]
async fn usb_task(mut dev: embassy_usb::UsbDevice<'static, UsbDriver>) {
    dev.run().await;
}

/// CDC echo: whatever the host sends comes back.
#[task]
async fn echo_task(
    mut rx: embassy_usb::class::cdc_acm::Receiver<'static, UsbDriver>,
    mut tx: embassy_usb::class::cdc_acm::Sender<'static, UsbDriver>,
) {
    defmt::info!("cdc echo task running");
    loop {
        rx.wait_connection().await;
        defmt::info!("usb connected");
        loop {
            let mut buf = [0u8; 64];
            match rx.read_packet(&mut buf).await {
                Ok(n) if n > 0 => {
                    if tx.write_packet(&buf[..n]).await.is_err() {
                        break;
                    }
                }
                _ => break,
            }
        }
    }
}

/// Red 1 Hz heartbeat (D2 = PA0, active low).
#[task]
async fn heartbeat(pin: hal::Peri<'static, hal::gpio::AnyPin>) {
    let mut led = hal::gpio::Output::new(pin, hal::gpio::Level::High, hal::gpio::Speed::Low);
    loop {
        led.toggle();
        Timer::after(Duration::from_millis(500)).await;
    }
}
