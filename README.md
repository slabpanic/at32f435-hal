# at32f435-hal

An embassy-style, `no_std` hardware abstraction layer for the Artery **AT32F435**
(Cortex-M4F, `thumbv7em-none-eabihf`).

> **Note**
> The project is under development: the drivers listed as `done` below are validated
> on hardware, the rest is compile-checked or not started. See the coverage checklist
> for the exact state of every peripheral.

Keypoints:

- Embassy framework driver built on `embassy-hal-internal`, structured after
  [ch32-hal](https://github.com/ch32-rs/ch32-hal)
- Register access through the [`at32f4xx-pac`](https://crates.io/crates/at32f4xx-pac)
  0.4 (svd2rust) PAC; the HAL never touches the PAC's `Peripherals::take()/steal()`
- AT32-vs-STM32 register differences are patched inside the HAL (clock tree, USB
  OTGFS, timer 32-bit mode, GPIO IOMUX), so applications stay idiomatic embassy-style

## Peripheral support

| Peripheral | Status | Notes |
| ---------- | ------ | ----- |
| RCC / clock tree | partial | 288 MHz HEXT->PLL preset, hardware-validated |
| GPIO (+ IOMUX, e-h 1.0) | done | hardware-validated |
| EXINT (async input) | done | hardware-validated |
| Timer PWM (TMR2/3/4/5) | partial | 4ch PWM, hardware-validated on TMR2 |
| USART/UART (async + blocking) | partial | compile-validated |
| USB OTGFS device (embassy-usb) | done | CDC-ACM loopback, hardware-validated |
| WDT (independent watchdog) | partial | compile-validated |
| Delay / embassy time drivers | partial | SysTick driver hardware-validated |
| DMA / EDMA, SPI / I2S, I2C, ADC, DAC, CAN, ERTC, WWDT, CRC, PWC, FLASH, ACC, advanced/basic timers, IRTMR, XMC, QSPI, SDIO, DVP | todo | see the checklist |
| EMAC | n/a | F437 only |

Legend: `done` = implemented and validated on hardware, `partial` = implemented
(feature subset and/or compile-validated only), `todo` = not started, `n/a` = not
present on AT32F435.

## Usage

```rust
#![no_std]
#![no_main]

use at32f435_hal as hal;
use hal::gpio::{AnyPin, Level, Output, Speed};
use hal::Peri;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = hal::init(hal::rcc::Config::SYSCLK_FREQ_288MHZ_HEXT_8M);
    spawner.spawn(blink(p.PA0.into()).unwrap());
}

#[embassy_executor::task]
async fn blink(pin: Peri<'static, AnyPin>) {
    let mut led = Output::new(pin, Level::High, Speed::Low);
    loop {
        led.toggle();
        Timer::after(Duration::from_millis(500)).await;
    }
}
```

The application brings its own `embassy-executor`/`embassy-time` (pick a
`generic-queue-N`), `memory.x` + `build.rs`, and rustflags `-Tlink.x`. Per-driver
usage (PWM, serial, EXTI, USB) is documented in the module-level rustdoc
(`cargo doc --open`).

## Cargo features

| feature | default | description |
| ------- | ------ | ------ |
| `embassy` | yes | time driver + queue infrastructure (embassy-time-driver/queue-utils) |
| `rt` | yes | cortex-m-rt (`#[interrupt]`/`#[exception]`, vector table) |
| `exti` | yes | built-in EXINT driver; the 7 EXINT vector handlers are owned by the HAL; disable to bind them yourself |
| `time-driver-systick` | yes | SysTick 1 ms interrupt + CVR interpolation; **the SysTick exception belongs to the HAL** |
| `time-driver-tim2` | - | TMR2 32-bit time driver; **the TMR2 interrupt belongs to the HAL** |
| `usb` | - | OTGFS1/OTGFS2 device driver (pairs with embassy-usb-driver 0.2 / embassy-usb 0.6) |
| `defmt` | - | forwards Format/logging from embassy-hal-internal + embassy-usb-synopsys-otg |

`systick`/`tim2` time drivers are mutually exclusive (enabling both is a
compile_error).

## Documentation

Per-driver usage is documented in the module-level rustdoc (`cargo doc --open`).
Artery's vendor documents (datasheet, reference manual) are not distributed in this
repository (copyright); download them from
<https://www.arterytek.com/cn/product/AT32F435.jsp> or
<https://www.arterychip.com/en/product/AT32F435.jsp>.

## Minimum supported Rust version (MSRV)

Rust 1.82 (`rust-version` in Cargo.toml); developed with stable on
`thumbv7em-none-eabihf`.

## License

MIT, see [LICENSE](LICENSE).
