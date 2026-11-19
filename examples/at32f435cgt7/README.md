# AT32F435CGT7 board examples

These firmware binaries target the AT32F435CGT7 EVB with an 8 MHz crystal.
The blink example uses the PA0 LED. See the module docs in `src/bin` for pin
mappings and optional test connections. Keep tests for specific external
devices in the corresponding board application.

From the repository root, build or run the firmware with these commands.

```sh
cargo build --locked --release -p at32f435cgt7-examples --bins
cargo run --locked --release -p at32f435cgt7-examples --bin blink
```

The configured runner flashes the firmware using probe-rs and shows defmt RTT
logs. Run the basic bus examples with their pins unconnected:

```sh
cargo run --locked --release -p at32f435cgt7-examples --bin spi
cargo run --locked --release -p at32f435cgt7-examples --bin i2c_scan
```

- `spi` checks that blocking and DMA transfers complete on SPI1 (PA5 SCK,
  PA6 MISO, PA7 MOSI). Data sampled from MISO is discarded.
- `i2c_scan` probes 7-bit addresses on I2C1 (PB6 SCL, PB7 SDA) with internal
  pull-ups. No device acknowledgments are expected on an empty bus.

The `flash` example erases the last 2 KiB of the MCU's internal flash, which
`memory.x` reserves for the example. Use a chip whose last sector is available.

To build the heartbeat with TMR2 as the Embassy time base, run the following.

```sh
cargo build --locked --release -p at32f435cgt7-examples \
  --no-default-features --features time-driver-tim2 --bin blink
```
