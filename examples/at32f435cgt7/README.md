# AT32F435CGT7 board examples

These firmware binaries target the AT32F435CGT7 EVB with an 8 MHz crystal.
The blink example uses the onboard LED on PA0. See the module docs in
src/bin for pin mappings and any external connections.

From the repository root, build or run the firmware with these commands.

```sh
cargo build --locked --release -p at32f435cgt7-examples --bins
cargo run --locked --release -p at32f435cgt7-examples --bin blink
```

The configured runner flashes the firmware using probe-rs and shows defmt RTT
logs. The flash example erases the last 2 KiB of internal flash reserved by
memory.x. The spi_flash example erases the last sector of the attached W25Q64.
Use a chip whose last sector is available for the test.


To build the heartbeat with TMR2 as the Embassy time base, run the following.

```sh
cargo build --locked --release -p at32f435cgt7-examples \
  --no-default-features --features time-driver-tim2 --bin blink
```
