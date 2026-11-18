//! CAN: async loopback on CAN1 (TX=PA12, RX=PA11 @ 500 kbit/s) using
//! `CanAsync` - concurrent transmit/receive futures driven by the TX/RX0
//! interrupt vectors.
//!
//! Loopback mode needs no transceiver: frames transmitted by the core are
//! received back through its own receive filters. On a real bus, swap
//! `Mode::Loopback` for `Mode::Normal` and wire a transceiver.

#![no_std]
#![no_main]

use at32f435_hal as hal;
use defmt_rtt as _;
use embassy_executor::main;
use embassy_futures::join::join;
use embassy_time::{Duration, Timer};
use hal::can::{self, Fifo};
use panic_probe as _;

defmt::timestamp!("{=u64:us}", hal::embassy::now_us());

hal::bind_interrupts!(struct Irqs {
    CAN1_TX => hal::can::TxInterruptHandler<hal::peripherals::CAN1>;
    CAN1_RX0 => hal::can::Rx0InterruptHandler<hal::peripherals::CAN1>;
    CAN_RX1 => hal::can::Rx1InterruptHandler<hal::peripherals::CAN1>;
});

#[main]
async fn main(_spawner: embassy_executor::Spawner) {
    let p = hal::init(hal::rcc::Config::SYSCLK_FREQ_288MHZ_HEXT_8M);
    defmt::info!("can up: {} MHz", hal::rcc::clocks().sysclk.0 / 1_000_000);

    // Bit timing search around the 87.5% sample point.
    let cfg = match can::Config::baud(hal::rcc::clocks().pclk1, 500_000) {
        Ok(c) => c,
        Err(e) => {
            defmt::error!("can timing calc failed: {:?}", e);
            loop {
                Timer::after(Duration::from_secs(1)).await;
            }
        }
    };
    defmt::info!(
        "can timing: presc={} seg1={} seg2={} baud={}",
        cfg.timing.prescaler,
        cfg.timing.seg1,
        cfg.timing.seg2,
        cfg.timing.baud(hal::rcc::clocks().pclk1)
    );
    let mut mode = cfg;
    mode.mode = can::Mode::Loopback;

    let mut can_dev = can::CanAsync::new(p.CAN1, p.PA12, p.PA11, mode, Irqs, Irqs, Irqs);
    can_dev
        .blocking()
        .add_filter(&can::FilterEntry::accept_all_std(0, Fifo::Fifo0));

    // 8 frames sent concurrently while the receive future drains FIFO0.
    let make_frame = |i: u8| {
        can::Frame::new(
            can::Id::Standard(0x100 + i as u16),
            &[(i << 4) | 0xA, 0x5A, i, 0x00],
        )
    };

    let tx_task = async {
        let mut ok = true;
        for i in 0u8..8 {
            ok &= can_dev.transmit(&make_frame(i)).await.is_ok();
        }
        ok
    };
    let rx_task = async {
        let mut frames = [make_frame(0); 8];
        for slot in frames.iter_mut() {
            *slot = can_dev.receive(Fifo::Fifo0).await.unwrap();
        }
        frames
    };

    let (tx_ok, rx_frames) = join(tx_task, rx_task).await;
    let mut ids_match = true;
    for (i, f) in rx_frames.iter().enumerate() {
        if f.id != can::Id::Standard(0x100 + i as u16) || f.data[0] != ((i as u8) << 4) | 0xA {
            ids_match = false;
        }
    }
    defmt::info!(
        "can loopback: tx_ok={} rx_count={} ids_match={}",
        tx_ok,
        rx_frames.len(),
        ids_match
    );
    if tx_ok && rx_frames.len() == 8 && ids_match {
        defmt::info!("can async loopback PASS (8 frames, FIFO0, TEC/REC clean)");
    } else {
        defmt::error!("can async loopback FAIL");
    }

    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}
