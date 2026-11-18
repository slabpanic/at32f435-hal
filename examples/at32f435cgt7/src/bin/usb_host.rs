//! USB host: OTGFS1 host enumerating OTGFS2 as a CDC-ACM device on the same
//! chip (jumper loopback).
//!
//! Wiring (two jumpers): PA12 -> PB15 (D+ to D+) and PA11 -> PB14 (D- to
//! D-). The device half runs a CDC echo on OTGFS2; the host half waits for
//! the connection, resets the port, enumerates (device + string descriptors,
//! SET_CONFIGURATION), finds the CDC bulk endpoints in the config descriptor
//! and loops back 512/1/64/200-byte buffers.
//!
//! Without jumpers the example waits 20 s, dumps the host core state (mode/
//! power/line status) and skips. Removing the jumpers mid-run shows the
//! disconnect event.

#![no_std]
#![no_main]

use at32f435_hal as hal;
use defmt_rtt as _;
use embassy_executor::{main, task, Spawner};
use embassy_futures::select;
use embassy_time::{Duration, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State as CdcState};
use embassy_usb::{Builder, Config as UsbConfig};
use hal::peripherals::{USB_OTG1, USB_OTG2};
use hal::usb::host::{
    enumerate, get_string, pipe, DeviceEvent, Direction, EndpointAddress, EndpointInfo,
    EndpointType, HostInterruptHandler, Speed, UsbHost, UsbHostAllocator, UsbHostController,
    UsbPipe,
};
use hal::usb::{Config as OtgConfig, Driver, InterruptHandler};
use panic_probe as _;
use static_cell::StaticCell;

defmt::timestamp!("{=u64:us}", hal::embassy::now_us());

hal::bind_interrupts!(struct IrqsHost {
    OTGFS1 => HostInterruptHandler<USB_OTG1>;
});

hal::bind_interrupts!(struct IrqsDevice {
    OTGFS2 => InterruptHandler<USB_OTG2>;
});

type DeviceDriver = Driver<'static, USB_OTG2>;

#[main]
async fn main(spawner: Spawner) {
    let p = hal::init(hal::rcc::Config::SYSCLK_FREQ_288MHZ_HEXT_8M);
    defmt::info!(
        "usb_host up: {} MHz",
        hal::rcc::clocks().sysclk.0 / 1_000_000
    );

    // ---- device half (OTGFS2, PB14/PB15): CDC echo ----
    static EP_OUT_BUFFER: StaticCell<[u8; 256]> = StaticCell::new();
    let ep_out_buffer = EP_OUT_BUFFER.init([0u8; 256]);

    let dev_driver = Driver::new(
        p.USB_OTG2,
        p.PB15,
        p.PB14,
        IrqsDevice,
        ep_out_buffer,
        OtgConfig::default(),
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
    usb_config.product = Some("OTG loopback device");

    static CDC_STATE: StaticCell<CdcState> = StaticCell::new();
    let cdc_state = CDC_STATE.init(CdcState::new());

    let mut builder = Builder::new(
        dev_driver,
        usb_config,
        config_desc,
        bos_desc,
        msos_desc,
        control_buf,
    );
    let class = CdcAcmClass::new(&mut builder, cdc_state, 64);
    let device = builder.build();

    spawner.spawn(usb_device_task(device).unwrap());
    let (tx, rx) = class.split();
    spawner.spawn(usb_echo_task(rx, tx).unwrap());
    spawner.spawn(heartbeat(p.PA0.into()).unwrap());

    // ---- host half (OTGFS1, PA11/PA12) ----
    static HOST: StaticCell<UsbHost<'static, USB_OTG1>> = StaticCell::new();
    let host = HOST.init(UsbHost::new(p.USB_OTG1, p.PA12, p.PA11, IrqsHost));
    spawner.spawn(host_task(host).unwrap());
}

#[task]
async fn usb_device_task(mut dev: embassy_usb::UsbDevice<'static, DeviceDriver>) {
    dev.run().await;
}

/// Device-side CDC echo: receive a packet, send it back.
#[task]
async fn usb_echo_task(
    mut rx: embassy_usb::class::cdc_acm::Receiver<'static, DeviceDriver>,
    mut tx: embassy_usb::class::cdc_acm::Sender<'static, DeviceDriver>,
) {
    defmt::info!("[device] cdc echo task running (OTGFS2, PB14/PB15)");
    loop {
        rx.wait_connection().await;
        defmt::info!("[device] cdc connected");
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

/// Host task: wait for connect, enumerate, bulk loopback.
#[task]
async fn host_task(host: &'static mut UsbHost<'static, USB_OTG1>) {
    defmt::info!("[host] waiting for device (jumper PA12->PB15, PA11->PB14)...");
    loop {
        match select::select(host.wait_for_device_event(), Timer::after_millis(20_000)).await {
            select::Either::Second(_) => {
                dump_host_core().await;
                defmt::info!("[host] no device within 20 s (jumper absent?) -> SKIPPED");
                return;
            }
            select::Either::First(DeviceEvent::Disconnected) => {
                defmt::info!("[host] device disconnected, re-waiting");
                continue;
            }
            select::Either::First(DeviceEvent::Overcurrent) => {
                defmt::error!("[host] overcurrent!");
                continue;
            }
            select::Either::First(DeviceEvent::Connected(speed)) => {
                defmt::info!("[host] connected, speed = {:?}", speed);
                if run_loopback(host, speed).await {
                    defmt::info!("[host] OTG host loopback PASS");
                } else {
                    defmt::error!("[host] OTG host loopback FAIL");
                }
                defmt::info!("[host] remove jumpers to restart, or plug in again");
            }
            select::Either::First(_) => continue,
        }
    }
}

/// Host core state dump - the bring-up evidence when no device is attached.
async fn dump_host_core() {
    let g = unsafe { &*hal::pac::USB_OTG1_GLOBAL::PTR };
    let h = unsafe { &*hal::pac::USB_OTG1_HOST::PTR };
    let curmod = g.gintsts().read().curmod().bit_is_set() as u8;
    let hprt = h.hprt().read();
    let (pwr, plsts, spd) = (
        hprt.prtpwr().bit_is_set() as u8,
        hprt.prtlnsts().bits(),
        hprt.prtspd().bits(),
    );
    let clksel = h.hcfg().read().fslspclksel().bits();
    defmt::info!(
        "[host] core: curmod={} pwr={} plsts={} speed={} hcfg_clksel={}",
        curmod,
        pwr,
        plsts,
        spd,
        clksel
    );
}

/// Enumerate + bulk loopback; true = everything passed.
async fn run_loopback(host: &mut UsbHost<'static, USB_OTG1>, speed: Speed) -> bool {
    let alloc = host.allocator();

    static CONFIG_BUF: StaticCell<[u8; 256]> = StaticCell::new();
    let config_buf = CONFIG_BUF.init([0u8; 256]);

    let info = match enumerate(&alloc, speed, Some(&mut config_buf[..])).await {
        Ok(i) => i,
        Err(e) => {
            defmt::error!("[host] enumeration failed: {:?}", e);
            return false;
        }
    };
    defmt::info!(
        "[host] enumerated: addr={} vid={=u16:#06x} pid={=u16:#06x} bcdUSB={=u16:#04x} mps0={} class={=u8:#04x}",
        info.address,
        info.vid,
        info.pid,
        info.bcd_usb,
        info.max_packet_size0,
        info.device_class
    );

    static STR_BUF: StaticCell<[u8; 64]> = StaticCell::new();
    let str_buf = STR_BUF.init([0u8; 64]);
    match get_string(
        &alloc,
        info.address,
        info.max_packet_size0,
        1,
        &mut str_buf[..],
    )
    .await
    {
        Ok(s) => defmt::info!("[host] iManufacturer = {}", s),
        Err(e) => defmt::debug!("[host] no manufacturer string: {:?}", e),
    }
    match get_string(
        &alloc,
        info.address,
        info.max_packet_size0,
        2,
        &mut str_buf[..],
    )
    .await
    {
        Ok(s) => defmt::info!("[host] iProduct = {}", s),
        Err(e) => defmt::debug!("[host] no product string: {:?}", e),
    }

    // Parse the config descriptor for the CDC data interface (class 0x0A)
    // bulk pair; fall back to the first bulk pair of any interface.
    let (out_ep, in_ep, mps) = match find_cdc_bulk_endpoints(&config_buf[..info.config_len]) {
        Some(x) => x,
        None => {
            defmt::error!("[host] no bulk endpoints in config descriptor");
            return false;
        }
    };
    defmt::info!(
        "[host] CDC data endpoints: OUT ep{} / IN ep{} (mps {})",
        out_ep,
        in_ep,
        mps
    );

    let mut bulk_out = alloc
        .alloc_pipe::<pipe::Bulk, pipe::Out>(
            info.address,
            &EndpointInfo {
                addr: EndpointAddress::from_parts(out_ep as usize, Direction::Out),
                ep_type: EndpointType::Bulk,
                max_packet_size: mps,
                interval_ms: 0,
            },
            None,
        )
        .expect("alloc bulk out pipe");
    let mut bulk_in = alloc
        .alloc_pipe::<pipe::Bulk, pipe::In>(
            info.address,
            &EndpointInfo {
                addr: EndpointAddress::from_parts(in_ep as usize, Direction::In),
                ep_type: EndpointType::Bulk,
                max_packet_size: mps,
                interval_ms: 0,
            },
            None,
        )
        .expect("alloc bulk in pipe");

    // Loopback rounds: 512 B (8 packets), 1 B, 64 B, 200 B (odd cross-packet).
    let sizes: [usize; 4] = [512, 1, 64, 200];
    let mut seed = 0xA5u8;
    for (round, &size) in sizes.iter().enumerate() {
        let mut tx_buf = [0u8; 512];
        let mut rx_buf = [0u8; 512];
        for (i, b) in tx_buf[..size].iter_mut().enumerate() {
            *b = seed.wrapping_add(i as u8).rotate_left((i as u32) % 8);
        }
        seed = seed.wrapping_add(0x11);

        if bulk_out.request_out(&tx_buf[..size], false).await.is_err() {
            defmt::error!("[host] round {} ({} B): bulk out failed", round, size);
            return false;
        }
        let n = match bulk_in.request_in(&mut rx_buf[..size]).await {
            Ok(n) => n,
            Err(e) => {
                defmt::error!(
                    "[host] round {} ({} B): bulk in failed: {:?}",
                    round,
                    size,
                    e
                );
                return false;
            }
        };
        if n != size || rx_buf[..size] != tx_buf[..size] {
            defmt::error!(
                "[host] round {} ({} B): got {} B, data mismatch",
                round,
                size,
                n
            );
            return false;
        }
        defmt::info!("[host] round {} ({} B) loopback OK", round, size);
    }
    true
}

/// Find the CDC data interface (bInterfaceClass = 0x0A) bulk OUT/IN pair in a
/// config descriptor; fall back to the first bulk pair of any interface.
fn find_cdc_bulk_endpoints(cfg: &[u8]) -> Option<(u8, u8, u16)> {
    let mut fallback: Option<(u8, u8, u16)> = None;
    let mut cur_out: Option<(u8, u16)> = None;
    let mut cur_in: Option<(u8, u16)> = None;
    let mut data_iface = false;
    let mut i = 0usize;
    while i + 2 <= cfg.len() {
        let len = cfg[i] as usize;
        if len < 2 || i + len > cfg.len() {
            break;
        }
        let typ = cfg[i + 1];
        match typ {
            0x04 => {
                if data_iface {
                    if let (Some((o, om)), Some((n, im))) = (cur_out, cur_in) {
                        return Some((o, n, om.min(im)));
                    }
                }
                data_iface = cfg[i + 5] == 0x0A;
                cur_out = None;
                cur_in = None;
            }
            0x05 => {
                let attr = cfg[i + 3];
                let addr = cfg[i + 2];
                let mps = u16::from_le_bytes([cfg[i + 4], cfg[i + 5]]);
                if attr & 0x03 == 0x02 {
                    if addr & 0x80 != 0 {
                        cur_in = Some((addr & 0x0F, mps));
                    } else {
                        cur_out = Some((addr & 0x0F, mps));
                    }
                    if !data_iface && fallback.is_none() {
                        if let (Some((o, om)), Some((n, im))) = (cur_out, cur_in) {
                            fallback = Some((o, n, om.min(im)));
                        }
                    }
                }
            }
            _ => {}
        }
        i += len;
    }
    if data_iface {
        if let (Some((o, om)), Some((n, im))) = (cur_out, cur_in) {
            return Some((o, n, om.min(im)));
        }
    }
    fallback
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
