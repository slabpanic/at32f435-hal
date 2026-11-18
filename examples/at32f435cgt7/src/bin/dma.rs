//! DMA1: memory-to-memory copy (blocking), the async path with
//! `bind_interrupts!`, and DMAMUX peripheral requests are what the other
//! examples use under the hood.

#![no_std]
#![no_main]

use at32f435_hal as hal;
use defmt_rtt as _;
use embassy_executor::main;
use embassy_time::{Duration, Timer};
use hal::dma::Channel;
use panic_probe as _;

defmt::timestamp!("{=u64:us}", hal::embassy::now_us());

hal::bind_interrupts!(struct Irqs {
    DMA1_CHANNEL1 => hal::dma::InterruptHandler<hal::peripherals::DMA1_CH1>;
    DMA1_CHANNEL2 => hal::dma::InterruptHandler<hal::peripherals::DMA1_CH2>;
});

#[main]
async fn main(_spawner: embassy_executor::Spawner) {
    let p = hal::init(hal::rcc::Config::SYSCLK_FREQ_288MHZ_HEXT_8M);
    defmt::info!("dma up: {} MHz", hal::rcc::clocks().sysclk.0 / 1_000_000);

    // Channel initialization must preserve sibling channel registers.
    let mut word_channel = Channel::new(p.DMA1_CH1);
    word_channel.set_count(128);
    let mut halfword_channel = Channel::new(p.DMA1_CH2);
    // SAFETY: DMA1 points to its MMIO block. This is a read of the disabled channel.
    let count = unsafe { &*hal::pac::DMA1::PTR }
        .channel(0)
        .dtcnt()
        .read()
        .bits();
    if count == 128 {
        defmt::info!("DMA sibling configuration PASS");
    } else {
        defmt::error!("DMA sibling configuration FAIL");
    }

    // Blocking memory copy on the first channel.
    {
        static mut SRC: [u32; 128] = [0; 128];
        static mut DST: [u32; 128] = [0; 128];
        let src = unsafe { &mut *core::ptr::addr_of_mut!(SRC) };
        let dst = unsafe { &mut *core::ptr::addr_of_mut!(DST) };
        for (i, v) in src.iter_mut().enumerate() {
            *v = 0x1000_0000 + i as u32;
        }
        dst.fill(0);
        let ok = unsafe { word_channel.mem_to_mem(dst.as_mut_ptr(), src.as_ptr(), src.len()) }
            .is_ok()
            && dst == src;
        if ok {
            defmt::info!("dma mem2mem 128 words PASS");
        } else {
            defmt::error!("dma mem2mem 128 words FAIL");
        }
    }

    // ---- 2. async mem-to-memory (interrupt + wait), 64 halfwords ----
    {
        static mut SRC2: [u16; 64] = [0; 64];
        static mut DST2: [u16; 64] = [0; 64];
        let src = unsafe { &mut *core::ptr::addr_of_mut!(SRC2) };
        let dst = unsafe { &mut *core::ptr::addr_of_mut!(DST2) };
        for (i, v) in src.iter_mut().enumerate() {
            *v = 0xB000 + i as u16;
        }
        dst.fill(0);
        let ok = unsafe {
            halfword_channel
                .mem_to_mem_async(dst.as_mut_ptr(), src.as_ptr(), src.len())
                .await
        }
        .is_ok()
            && dst.iter().zip(src.iter()).all(|(d, s)| d == s);
        if ok {
            defmt::info!("dma mem2mem async 64 halfwords PASS");
        } else {
            defmt::error!("dma mem2mem async 64 halfwords FAIL");
        }
    }

    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}
