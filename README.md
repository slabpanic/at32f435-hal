# at32f435-hal

A `no_std` Rust HAL for Artery AT32F435 microcontrollers (Arm Cortex-M4F).
It integrates with [Embassy](https://github.com/embassy-rs/embassy) and follows
the driver conventions of [ch32-hal](https://github.com/ch32-rs/ch32-hal).

The project is under active development.

## Quick start

Install Rust 1.87 or newer and [probe-rs](https://probe.rs/) to run the board examples.
From the repository root:

```sh
rustup target add thumbv7em-none-eabihf
cargo run --locked --release -p at32f435cgt7-examples --bin blink
```

The runner flashes the board and streams RTT logs. The examples assume an 8 MHz
crystal and 384 KiB of SRAM. The basic SPI and I2C examples run with their
bus pins unconnected. See the board guide for optional loopback wiring.

- [Blink source](examples/at32f435cgt7/src/bin/blink.rs)
- [Board setup and examples](examples/at32f435cgt7/README.md)
- [Application manifest](examples/at32f435cgt7/Cargo.toml)

To use the HAL in your own firmware, add it to `Cargo.toml`:

```toml
[dependencies]
at32f435-hal = { git = "https://github.com/slabpanic/at32f435-hal" }
```

Use a linker script and clock configuration that match your board. To browse
the API and its usage examples, run `cargo doc --no-deps --features usb,defmt`.

## Peripheral support

The board results below apply to the listed paths on the AT32F435CGT7 EVB.

| Peripheral | Implemented | Board coverage |
| ---------- | ----------- | -------------- |
| Clocks and time | HICK, HEXT, PLL, SysTick and TMR2 Embassy time drivers | Clock presets and both time drivers exercised |
| GPIO and EXTI | Digital I/O, alternate functions, async edge waits | GPIO blink and mode switching exercised |
| DMA and EDMA | DMAMUX, memory copies, peripheral transfers | Memory copies, peripheral DMA and UART DMA cancellation exercised |
| USART and CAN | Blocking and interrupt-driven I/O, USART DMA and RS-485, CAN filters | USART DMA TX and RS-485 DE, CAN internal loopback |
| SPI | Blocking and DMA master, hardware chip select and CRC | Prior board checks exercised blocking and DMA transfers, hardware chip select and CRC |
| I2S | Blocking master and slave, full duplex, DMA setup | Master clocks exercised |
| I2C | Blocking 7-/10-bit master with reload, SMBus PEC, 7-bit DMA | Empty-bus scan. Target checks exercised DMA writes, blocking reload and repeated START, PEC generation and mismatch detection, and timeout recovery |
| Timers and analog | PWM, complementary PWM, capture, one-pulse, QEI, ADC and DAC | Timer capture and brake, ADC scan/DMA, DAC-to-ADC loopback |
| USB OTG FS | Embassy device and host support | CDC echo and host startup exercised |
| Flash, watchdogs and RTC | Main flash, option-byte access, WDT, WWDT, ERTC | Main-flash scratch sector, watchdogs, RTC calendar and alarm |
| Power and utilities | Sleep modes, CRC, ACC, debug and XMC mapping | Normal-LDO deep-sleep wakeup, CRC and debug checks |

## Cargo features

| Feature | Default | Purpose |
| ------- | ------- | ------- |
| `embassy` | Yes | Embassy time integration |
| `rt` | Yes | Cortex-M runtime and interrupt bindings |
| `exti` | Yes | Async GPIO edge waits using HAL-owned EXINT vectors |
| `time-driver-systick` | Yes | Embassy time on SysTick |
| `time-driver-tim2` | No | Embassy time on TMR2, reserving that timer |
| `usb` | No | USB OTG FS device and host |
| `defmt` | No | Embedded logging support |

With `embassy` enabled, select exactly one time driver. To use TMR2, set
`default-features = false` and enable `embassy`, `exti`, and
`time-driver-tim2`. The time-driver features and `exti` enable `rt`.

## TODO

- Validate I2C DMA receive, successful 10-bit transfers and receipt of valid SMBus PEC with suitable targets.
- Verify USB host enumeration and bulk transfers, USART DMA reception and flow control, and I2S full-duplex audio with external hardware.
- Test deep-sleep wakeup in low-power LDO mode and validate ACC calibration against USB SOF.
- Add drivers for SDIO, QSPI, XMC and DVP.

## Contributing

Driver fixes, board examples and hardware test reports are welcome. Include
the chip and board, wiring and Cargo features when reporting hardware behavior.
See the [CI workflow](.github/workflows/ci.yml) for build and test commands.

## License

[MIT](LICENSE).
