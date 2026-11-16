//! Pin-table expansion macros and peripheral signal trait infrastructure (same
//! structure as ch32-hal's `macros.rs`).
//!
//! - [`foreach_pin!`]: feeds all AT32F435 GPIO pins (PA0..PH15), together with their
//!   EXINT channel singleton names, to the caller's callback macro. Callback arguments:
//!   `$pin` pin singleton type name, `$port` port number (A=0..H=7), `$num` pin number
//!   (0..15), `$exti` EXINT channel singleton type name.
//! - [`pin_trait!`]: declares a trait stating "which pins a peripheral signal can be
//!   routed to" (the AT32 has no remap; each pin carries a 4-bit alternate-function
//!   number, and the trait method `af_num()` returns the value to write to MUXL/MUXH).
//! - [`pin_trait_impl!`]: implements a signal trait for a concrete (peripheral instance, pin) pair.
//! - [`new_pin!`]: configures the pin's alternate function and consumes it into an `Option<AnyPin>`.

#[macro_export]
macro_rules! foreach_pin {
    ($callback:ident) => {
        $crate::foreach_pin!(@port $callback, 0, [
            PA0 = 0, EXINT0, PA1 = 1, EXINT1, PA2 = 2, EXINT2, PA3 = 3, EXINT3,
            PA4 = 4, EXINT4, PA5 = 5, EXINT5, PA6 = 6, EXINT6, PA7 = 7, EXINT7,
            PA8 = 8, EXINT8, PA9 = 9, EXINT9, PA10 = 10, EXINT10, PA11 = 11, EXINT11,
            PA12 = 12, EXINT12, PA13 = 13, EXINT13, PA14 = 14, EXINT14, PA15 = 15, EXINT15
        ]);
        $crate::foreach_pin!(@port $callback, 1, [
            PB0 = 0, EXINT0, PB1 = 1, EXINT1, PB2 = 2, EXINT2, PB3 = 3, EXINT3,
            PB4 = 4, EXINT4, PB5 = 5, EXINT5, PB6 = 6, EXINT6, PB7 = 7, EXINT7,
            PB8 = 8, EXINT8, PB9 = 9, EXINT9, PB10 = 10, EXINT10, PB11 = 11, EXINT11,
            PB12 = 12, EXINT12, PB13 = 13, EXINT13, PB14 = 14, EXINT14, PB15 = 15, EXINT15
        ]);
        $crate::foreach_pin!(@port $callback, 2, [
            PC0 = 0, EXINT0, PC1 = 1, EXINT1, PC2 = 2, EXINT2, PC3 = 3, EXINT3,
            PC4 = 4, EXINT4, PC5 = 5, EXINT5, PC6 = 6, EXINT6, PC7 = 7, EXINT7,
            PC8 = 8, EXINT8, PC9 = 9, EXINT9, PC10 = 10, EXINT10, PC11 = 11, EXINT11,
            PC12 = 12, EXINT12, PC13 = 13, EXINT13, PC14 = 14, EXINT14, PC15 = 15, EXINT15
        ]);
        $crate::foreach_pin!(@port $callback, 3, [
            PD0 = 0, EXINT0, PD1 = 1, EXINT1, PD2 = 2, EXINT2, PD3 = 3, EXINT3,
            PD4 = 4, EXINT4, PD5 = 5, EXINT5, PD6 = 6, EXINT6, PD7 = 7, EXINT7,
            PD8 = 8, EXINT8, PD9 = 9, EXINT9, PD10 = 10, EXINT10, PD11 = 11, EXINT11,
            PD12 = 12, EXINT12, PD13 = 13, EXINT13, PD14 = 14, EXINT14, PD15 = 15, EXINT15
        ]);
        $crate::foreach_pin!(@port $callback, 4, [
            PE0 = 0, EXINT0, PE1 = 1, EXINT1, PE2 = 2, EXINT2, PE3 = 3, EXINT3,
            PE4 = 4, EXINT4, PE5 = 5, EXINT5, PE6 = 6, EXINT6, PE7 = 7, EXINT7,
            PE8 = 8, EXINT8, PE9 = 9, EXINT9, PE10 = 10, EXINT10, PE11 = 11, EXINT11,
            PE12 = 12, EXINT12, PE13 = 13, EXINT13, PE14 = 14, EXINT14, PE15 = 15, EXINT15
        ]);
        $crate::foreach_pin!(@port $callback, 5, [
            PF0 = 0, EXINT0, PF1 = 1, EXINT1, PF2 = 2, EXINT2, PF3 = 3, EXINT3,
            PF4 = 4, EXINT4, PF5 = 5, EXINT5, PF6 = 6, EXINT6, PF7 = 7, EXINT7,
            PF8 = 8, EXINT8, PF9 = 9, EXINT9, PF10 = 10, EXINT10, PF11 = 11, EXINT11,
            PF12 = 12, EXINT12, PF13 = 13, EXINT13, PF14 = 14, EXINT14, PF15 = 15, EXINT15
        ]);
        $crate::foreach_pin!(@port $callback, 6, [
            PG0 = 0, EXINT0, PG1 = 1, EXINT1, PG2 = 2, EXINT2, PG3 = 3, EXINT3,
            PG4 = 4, EXINT4, PG5 = 5, EXINT5, PG6 = 6, EXINT6, PG7 = 7, EXINT7,
            PG8 = 8, EXINT8, PG9 = 9, EXINT9, PG10 = 10, EXINT10, PG11 = 11, EXINT11,
            PG12 = 12, EXINT12, PG13 = 13, EXINT13, PG14 = 14, EXINT14, PG15 = 15, EXINT15
        ]);
        $crate::foreach_pin!(@port $callback, 7, [
            PH0 = 0, EXINT0, PH1 = 1, EXINT1, PH2 = 2, EXINT2, PH3 = 3, EXINT3,
            PH4 = 4, EXINT4, PH5 = 5, EXINT5, PH6 = 6, EXINT6, PH7 = 7, EXINT7,
            PH8 = 8, EXINT8, PH9 = 9, EXINT9, PH10 = 10, EXINT10, PH11 = 11, EXINT11,
            PH12 = 12, EXINT12, PH13 = 13, EXINT13, PH14 = 14, EXINT14, PH15 = 15, EXINT15
        ]);
    };
    (@port $callback:ident, $port:expr, [$($pin:ident = $num:expr, $exti:ident),* $(,)?]) => {
        $( $callback!($pin, $port, $num, $exti); )*
    };
}

/// Declares a peripheral signal trait: `trait TxPin<T: Instance>: Pin { fn af_num(&self) -> u8 }`.
///
/// The AT32F435 uses per-pin 4-bit alternate-function numbers (GPIOx_MUXL/MUXH) and has no
/// STM32F1-style chip-wide remap, so only this one form exists (corresponding to ch32-hal's
/// `afio_h4` form).
#[macro_export]
macro_rules! pin_trait {
    ($signal:ident, $instance:path) => {
        pub trait $signal<T: $instance>: $crate::gpio::Pin {
            /// The alternate-function number when this pin carries this signal (the 4-bit value written to GPIOx_MUXL/MUXH).
            fn af_num(&self) -> u8;
        }
    };
}

/// Implements a signal trait for a (peripheral instance, pin) pair: `af_num()` returns the alternate-function number.
#[macro_export]
#[allow(clippy::crate_in_macro_def)]
macro_rules! pin_trait_impl {
    (crate::$mod:ident::$trait:ident, $instance:ident, $pin:ident, $n:expr) => {
        impl $crate::$mod::$trait<$crate::peripherals::$instance> for $crate::peripherals::$pin {
            #[inline]
            fn af_num(&self) -> u8 {
                $n
            }
        }
    };
}

/// Configures the pin's alternate function (via the trait's `af_num()`) and consumes it into an `Option<AnyPin>`.
#[macro_export]
macro_rules! new_pin {
    ($name:ident, $af_type:expr) => {{
        let pin = $name;
        pin.set_as_af(pin.af_num(), $af_type);
        Some(pin.into())
    }};
}
