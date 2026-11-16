//! # at32f435-hal
//!
//! Embassy-style HAL for the Artery AT32F435 (Cortex-M4F), modeled after ch32-hal:
//! a unified `init(Config)` entry point, `Peri<'d, T>` peripheral singletons,
//! `RccPeripheral` clock management, gpio drivers (`Output`/`Input`/`Flex`/`AnyPin`),
//! an embassy time driver (SysTick by default, TMR2 optional), and `bind_interrupts!`
//! interrupt-binding infrastructure.
//!
//! Typical usage (the RGB breathing LED from evb_at32f435cgt7):
//! ```ignore
//! #![no_std]
//! #![no_main]
//!
//! use at32f435_hal as hal;
//! use hal::gpio::{AnyPin, Level, Output, Speed};
//! use hal::Peri;
//! use embassy_executor::Spawner;
//! use embassy_time::{Duration, Timer};
//!
//! #[embassy_executor::main]
//! async fn main(spawner: Spawner) {
//!     let p = hal::init(hal::rcc::Config::SYSCLK_FREQ_288MHZ_HEXT_8M);
//!     spawner.spawn(blink(p.PA0.into()).unwrap());
//! }
//!
//! #[embassy_executor::task]
//! async fn blink(pin: Peri<'static, AnyPin>) {
//!     let mut led = Output::new(pin, Level::High, Speed::Low);
//!     loop {
//!         led.toggle();
//!         Timer::after(Duration::from_millis(500)).await;
//!     }
//! }
//! ```
//!
//! The application also needs: `embassy-executor`, `embassy-time` (pick one
//! `generic-queue-N`), cortex-m's `critical-section-single-core` (force-enabled by the HAL),
//! cortex-m-rt (already included via the HAL's `rt` feature), and linker scripts
//! (the application project supplies its own memory.x + link.x).
//!
//! ## Key differences from going HAL-less
//! - `at32f4xx_pac::Peripherals::take()/steal()` is never touched: HAL peripheral
//!   singletons are zero-sized marker types and registers are accessed via the
//!   `Periph::PTR` pointer, avoiding the PAC pitfall where mixing take/steal makes
//!   `take()` stop working.
//! - SysTick (the default time driver) or TMR2 (`time-driver-tim2`) is owned by the HAL;
//!   the application must no longer define its own exception/interrupt handler for it.
//! - GPIO mode registers are always accessed read-modify-write (`modify`), preserving the
//!   SWD alternate-function bits in GPIOA's reset value.

#![no_std]
#![allow(non_snake_case)]

#[cfg(all(
    feature = "embassy",
    feature = "time-driver-systick",
    feature = "time-driver-tim2"
))]
compile_error!("time-driver-systick and time-driver-tim2 are mutually exclusive; pick one");

#[cfg(all(
    feature = "embassy",
    not(any(feature = "time-driver-systick", feature = "time-driver-tim2"))
))]
compile_error!("the embassy feature requires a time driver: enable time-driver-systick or time-driver-tim2");

#[cfg(all(feature = "embassy", feature = "time-driver-systick", not(feature = "rt")))]
compile_error!("time-driver-systick depends on rt (the SysTick exception must be registered via cortex-m-rt)");

#[cfg(all(feature = "embassy", feature = "time-driver-tim2", not(feature = "rt")))]
compile_error!("time-driver-tim2 depends on rt (the TMR2 interrupt must be registered via cortex-m-rt)");

pub use at32f4xx_pac::at32f435 as pac;
pub use embassy_hal_internal;
pub use embassy_hal_internal::{impl_peripheral, Peri, PeripheralType};

/// cortex-m-rt's interrupt/exception registration macros (required by `bind_interrupts!`), shared by the HAL and applications.
#[cfg(feature = "rt")]
pub use cortex_m_rt::{entry, interrupt};

// Compile the macros module first (foreach_pin! is expanded by gpio and other modules).
#[macro_use]
mod macros;

pub mod delay;
pub mod gpio;
pub mod interrupt;
pub mod rcc;
pub mod time;
pub mod timer;
pub mod usart;
pub mod wdg;

#[cfg(feature = "usb")]
pub mod usb;

pub mod exti;

mod peripheral;
pub use peripheral::RccPeripheral;

#[cfg(feature = "embassy")]
pub mod embassy;

embassy_hal_internal::peripherals!(
    // GPIO pin singletons (port*16+pin encoded into AnyPin)
    PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PA8, PA9, PA10, PA11, PA12, PA13, PA14, PA15, //
    PB0, PB1, PB2, PB3, PB4, PB5, PB6, PB7, PB8, PB9, PB10, PB11, PB12, PB13, PB14, PB15, //
    PC0, PC1, PC2, PC3, PC4, PC5, PC6, PC7, PC8, PC9, PC10, PC11, PC12, PC13, PC14, PC15, //
    PD0, PD1, PD2, PD3, PD4, PD5, PD6, PD7, PD8, PD9, PD10, PD11, PD12, PD13, PD14, PD15, //
    PE0, PE1, PE2, PE3, PE4, PE5, PE6, PE7, PE8, PE9, PE10, PE11, PE12, PE13, PE14, PE15, //
    PF0, PF1, PF2, PF3, PF4, PF5, PF6, PF7, PF8, PF9, PF10, PF11, PF12, PF13, PF14, PF15, //
    PG0, PG1, PG2, PG3, PG4, PG5, PG6, PG7, PG8, PG9, PG10, PG11, PG12, PG13, PG14, PG15, //
    PH0, PH1, PH2, PH3, PH4, PH5, PH6, PH7, PH8, PH9, PH10, PH11, PH12, PH13, PH14, PH15, //
    // Named peripheral singletons
    CRM, PWC, FLASH, SCFG, EXINT, EDMA, DMA1, DMA2, SDIO1, SDIO2, ERTC, WDT, WWDT, //
    // EXINT channel singletons (line number = GPIO pin number; lines 16-22 are internal wakeup sources and have no singletons)
    EXINT0, EXINT1, EXINT2, EXINT3, EXINT4, EXINT5, EXINT6, EXINT7, //
    EXINT8, EXINT9, EXINT10, EXINT11, EXINT12, EXINT13, EXINT14, EXINT15, //
    TMR1, TMR2, TMR3, TMR4, TMR5, TMR6, TMR7, TMR8, TMR9, TMR10, TMR11, TMR12, TMR13, TMR14, TMR20, //
    ACC, I2C1, I2C2, I2C3, SPI1, SPI2, SPI3, SPI4, //
    USART1, USART2, USART3, USART6, UART4, UART5, UART7, UART8, //
    ADC1, ADC2, ADC3, ADCCOM, CAN1, CAN2, DAC, CRC, DVP, //
    USB_OTG1, USB_OTG2, QSPI1, QSPI2, XMC
);

/// Global initialization: takes the peripheral singletons (once), configures the clock tree,
/// enables the GPIO port clocks, and initializes the embassy time driver. Call it as the
/// first line of the `#[embassy_executor::main]` function body.
pub fn init(config: impl Into<Config>) -> Peripherals {
    let config: Config = config.into();
    let p = Peripherals::take();

    rcc::init(config.rcc);
    rcc::enable_gpio_clocks();
    rcc::enable_scfg_clock();
    #[cfg(feature = "exti")]
    exti::init();
    #[cfg(feature = "embassy")]
    embassy::init();

    p
}

/// Global configuration (currently only the clock tree).
#[derive(Copy, Clone, Debug, Default)]
pub struct Config {
    pub rcc: rcc::Config,
}

impl From<rcc::Config> for Config {
    fn from(rcc: rcc::Config) -> Self {
        Self { rcc }
    }
}

/// Interrupt binding (same scheme as embassy-stm32 / ch32-hal):
///
/// ```ignore
/// use at32f435_hal as hal;
///
/// bind_interrupts!(struct Irqs {
///     USART1 => hal::usart::InterruptHandler<hal::pac::USART1>;
/// });
/// ```
///
/// Generates a `#[interrupt] fn <IRQ>()` vector entry (calling each handler's
/// `on_interrupt()` in turn) plus a `Binding<IRQ, H>` proof of ownership.
///   Holding this struct gives a driver exclusive ownership of the IRQ (the async serial
/// constructor receives it as its `_irq` parameter).
///
/// Note: the EXINT vectors are owned by the HAL's exti module by default (feature `exti`).
#[macro_export]
macro_rules! bind_interrupts {
    ($vis:vis struct $name:ident { $($irq:ident => $($handler:ty),*;)* }) => {
        #[derive(Copy, Clone)]
        $vis struct $name;

        $(
            #[allow(non_snake_case)]
            #[$crate::interrupt]
            unsafe fn $irq() {
                // Lets cortex-m-rt's `interrupt::<IRQ>` existence check resolve the path
                use $crate::interrupt;
                $(
                    <$handler as $crate::interrupt::typelevel::Handler<
                        $crate::interrupt::typelevel::$irq,
                    >>::on_interrupt();
                )*
            }

            $(
                unsafe impl $crate::interrupt::typelevel::Binding<
                    $crate::interrupt::typelevel::$irq,
                    $handler,
                > for $name {}
            )*
        )*
    };
}
