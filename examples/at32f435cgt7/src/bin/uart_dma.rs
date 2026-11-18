//! UART DMA: `write_dma` on USART1 through DMAMUX, the RX DMA arming path,
//! and the RS485 driver-enable pin behavior (`enable_rs485`, PA12 = DE).
//!
//! The RX path can only be armed (no peer transmitter on the evb - see
//! docs/pitfalls.md for why the board cannot loop its own RX); a bounded
//! check proves no completion is faked when the bus is idle. The DE pin is
//! sampled through GPIO IDR during a DMA transmission.

#![no_std]
#![no_main]

use at32f435_hal as hal;
use defmt_rtt as _;
use embassy_executor::main;
use embassy_futures::join::join;
use embassy_time::{Duration, Timer};
use hal::dma::{Channel, Config as DmaConfig, InterruptHandler as DmaIrq, Request, Width};
use hal::usart::{Config, DePolarity, Uart};
use panic_probe as _;

defmt::timestamp!("{=u64:us}", hal::embassy::now_us());

hal::bind_interrupts!(struct Irqs {
    USART1 => hal::usart::InterruptHandler<hal::peripherals::USART1>;
    DMA1_CHANNEL4 => DmaIrq<hal::peripherals::DMA1_CH4>;
    DMA1_CHANNEL5 => DmaIrq<hal::peripherals::DMA1_CH5>;
});

fn usart_dt_addr() -> u32 {
    hal::pac::USART1::PTR as u32 + 0x04 // DT (TDR/udr) register
}

#[main]
async fn main(_spawner: embassy_executor::Spawner) {
    let p = hal::init(hal::rcc::Config::SYSCLK_FREQ_288MHZ_HEXT_8M);
    defmt::info!(
        "uart_dma up: {} MHz",
        hal::rcc::clocks().sysclk.0 / 1_000_000
    );

    let mut uart =
        Uart::new(p.USART1, p.PA9, p.PA10, Irqs, Config::default()).expect("usart config");
    let mut tx_dma = Channel::new(p.DMA1_CH4);
    let mut rx_dma = Channel::new(p.DMA1_CH5);

    // ---- 1. TX DMA: 64-byte burst, then wait for the last shift out (TDC) ----
    let pat: [u8; 64] = core::array::from_fn(|i| (i * 7 + 11) as u8);
    // SAFETY: pat is in SRAM and the future is awaited to completion.
    let ok = unsafe { uart.write_dma(&pat, &mut tx_dma).await }.is_ok();
    let tdc = {
        let u = unsafe { &*hal::pac::USART1::PTR };
        u.sts().read().tdc().bit_is_set()
    };
    defmt::info!("tx dma: sent={} tdc={}", ok, tdc);
    if ok && tdc {
        defmt::info!("tx dma PASS (64 B via DMAMUX, TDC after)");
    } else {
        defmt::error!("tx dma FAIL");
    }

    // ---- 2. RX DMA armed path: idle bus -> no fake completion, clean stop ----
    {
        const N: usize = 8;
        let mut rbuf = [0xEEu8; N];
        // SAFETY: rbuf is a DMA-accessible SRAM buffer that lives beyond stop().
        unsafe {
            rx_dma
                .configure_peripheral_to_memory(
                    Request::Usart1Rx,
                    usart_dt_addr(),
                    rbuf.as_mut_ptr(),
                    N as u16,
                    DmaConfig::peripheral_to_memory(Width::Bits8),
                )
                .expect("configure USART RX DMA");
        }
        rx_dma.listen();
        let u = unsafe { &*hal::pac::USART1::PTR };
        u.sts().modify(|_, w| w.rdbf().clear_bit());
        u.ctrl3().modify(|_, w| w.dmaren().enable());
        // SAFETY: The configured rbuf remains exclusively available until stop().
        unsafe { rx_dma.start() };
        // Bounded spin: with no peer the transfer must stay pending
        // (NDT not decremented, FDTF not set).
        let mut left = 200_000u32;
        let mut still_pending = true;
        while left > 0 {
            if rx_dma.full_transfer_flag() || rx_dma.error_flag() {
                still_pending = false;
                break;
            }
            core::hint::spin_loop();
            left -= 1;
        }
        let ndt = unsafe {
            let b = &*hal::pac::DMA1::PTR;
            b.channel(4).dtcnt().read().bits()
        };
        rx_dma.stop();
        u.ctrl3().modify(|_, w| w.dmaren().disable());
        defmt::info!("rx dma armed: pending={} ndt={}", still_pending, ndt);
        if still_pending && ndt == N as u16 && !rx_dma.is_enabled() {
            defmt::info!("rx dma arming PASS (idle bus, clean stop)");
        } else {
            defmt::error!("rx dma arming FAIL");
        }
    }

    // ---- 3. RS485 DE pin electrical check (PA12, active high) ----
    uart.enable_rs485(p.PA12, DePolarity::ActiveHigh, 8, 8);
    let gpioa = unsafe { &*hal::pac::GPIOA::PTR };
    let de_high = || gpioa.idt().read().bits() & (1 << 12) != 0;
    let idle = de_high();

    // 64 B @ 115200 ~ 5.6 ms; sample DE 2 ms in (inside the window). The
    // sampler must be truly async (join arm order!), see docs/pitfalls.md.
    let pat2: [u8; 64] = core::array::from_fn(|i| i as u8);
    let fut = async {
        // SAFETY: pat2 remains accessible in SRAM while join awaits this future.
        unsafe { uart.write_dma(&pat2, &mut tx_dma).await }
    };
    let sampler = async {
        Timer::after_millis(2).await;
        de_high()
    };
    let (dma_result, de_mid) = join(fut, sampler).await;
    let after = de_high();
    defmt::info!("rs485 de: idle={} mid={} after={}", idle, de_mid, after);
    if dma_result.is_ok() && !idle && de_mid && !after {
        defmt::info!("rs485 DE PASS (idle low / high during TX / low after TDC)");
    } else {
        defmt::error!("rs485 DE FAIL");
    }

    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}
