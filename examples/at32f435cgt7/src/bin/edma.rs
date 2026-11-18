//! EDMA (RM 29): the 8-stream controller doing memory-to-memory copies -
//! blocking word, async halfword and byte transfers on different streams.
//!
//! Byte-width mem2mem never latches FDTF on this silicon (docs/pitfalls.md):
//! the driver accepts DTCNT==0 as a completion criterion for that case.

#![no_std]
#![no_main]

use at32f435_hal as hal;
use defmt_rtt as _;
use embassy_executor::main;
use embassy_time::{Duration, Timer};
use hal::edma;
use panic_probe as _;

defmt::timestamp!("{=u64:us}", hal::embassy::now_us());

hal::bind_interrupts!(struct Irqs {
    EDMA_STREAM2 => hal::edma::InterruptHandler<hal::peripherals::EDMA_STREAM2>;
});

#[main]
async fn main(_spawner: embassy_executor::Spawner) {
    let p = hal::init(hal::rcc::Config::SYSCLK_FREQ_288MHZ_HEXT_8M);
    defmt::info!("edma up: {} MHz", hal::rcc::clocks().sysclk.0 / 1_000_000);

    // ---- 1. blocking mem2mem, 128 words on stream 1 ----
    {
        let mut s1 = edma::Stream::new(p.EDMA_STREAM1);
        static mut SRC: [u32; 128] = [0; 128];
        static mut DST: [u32; 128] = [0; 128];
        let src = unsafe { &mut *core::ptr::addr_of_mut!(SRC) };
        let dst = unsafe { &mut *core::ptr::addr_of_mut!(DST) };
        for (i, v) in src.iter_mut().enumerate() {
            *v = 0x1000_0000 + i as u32;
        }
        dst.fill(0);
        let ok = unsafe { s1.mem_to_mem(dst.as_mut_ptr(), src.as_ptr(), 128) }.is_ok()
            && dst.iter().zip(src.iter()).all(|(d, s)| d == s);
        if ok {
            defmt::info!("edma mem2mem word x128 PASS");
        } else {
            defmt::error!("edma mem2mem word x128 FAIL");
        }
    }

    // ---- 2. async mem2mem, 64 halfwords on stream 2 ----
    {
        let mut s2 = edma::Stream::new(p.EDMA_STREAM2);
        static mut SRC2: [u16; 64] = [0; 64];
        static mut DST2: [u16; 64] = [0; 64];
        let src = unsafe { &mut *core::ptr::addr_of_mut!(SRC2) };
        let dst = unsafe { &mut *core::ptr::addr_of_mut!(DST2) };
        for (i, v) in src.iter_mut().enumerate() {
            *v = 0xB000 + i as u16;
        }
        dst.fill(0);
        s2.listen();
        let ok = unsafe { s2.mem_to_mem_async(dst.as_mut_ptr(), src.as_ptr(), 64) }
            .await
            .is_ok()
            && dst.iter().zip(src.iter()).all(|(d, s)| d == s);
        s2.unlisten();
        if ok {
            defmt::info!("edma mem2mem async halfword x64 PASS");
        } else {
            defmt::error!("edma mem2mem async halfword x64 FAIL");
        }
    }

    // ---- 3. blocking mem2mem, 96 bytes on stream 3 (FDTF quirk path) ----
    {
        let mut s3 = edma::Stream::new(p.EDMA_STREAM3);
        static mut SRC3: [u8; 96] = [0; 96];
        static mut DST3: [u8; 96] = [0; 96];
        let src = unsafe { &mut *core::ptr::addr_of_mut!(SRC3) };
        let dst = unsafe { &mut *core::ptr::addr_of_mut!(DST3) };
        for (i, v) in src.iter_mut().enumerate() {
            *v = (i as u8).wrapping_mul(7).wrapping_add(1);
        }
        dst.fill(0);
        let ok = unsafe { s3.mem_to_mem(dst.as_mut_ptr(), src.as_ptr(), 96) }.is_ok()
            && dst.iter().zip(src.iter()).all(|(d, s)| d == s);
        if ok {
            defmt::info!("edma mem2mem byte x96 PASS (DTCNT completion)");
        } else {
            defmt::error!("edma mem2mem byte x96 FAIL");
        }
    }

    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}
