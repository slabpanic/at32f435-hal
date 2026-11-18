use super::*;

// ---------------------------------------------------------------------------
// Pin trait impls (channel numbers from DS table 9 "additional function" column)
// ---------------------------------------------------------------------------

macro_rules! impl_adc_pin {
    ($peri:ident, $pin:ident, $ch:expr) => {
        impl AdcPin<crate::peripherals::$peri> for crate::peripherals::$pin {
            #[inline]
            fn channel(&self) -> AdcChannel {
                $ch
            }
        }
    };
}

// ADC123 shared pins (IN0-3 = PA0-3, IN10-13 = PC0-3)
impl_adc_pin!(ADC1, PA0, AdcChannel::Ch0);
impl_adc_pin!(ADC1, PA1, AdcChannel::Ch1);
impl_adc_pin!(ADC1, PA2, AdcChannel::Ch2);
impl_adc_pin!(ADC1, PA3, AdcChannel::Ch3);
impl_adc_pin!(ADC1, PC0, AdcChannel::Ch10);
impl_adc_pin!(ADC1, PC1, AdcChannel::Ch11);
impl_adc_pin!(ADC1, PC2, AdcChannel::Ch12);
impl_adc_pin!(ADC1, PC3, AdcChannel::Ch13);
impl_adc_pin!(ADC2, PA0, AdcChannel::Ch0);
impl_adc_pin!(ADC2, PA1, AdcChannel::Ch1);
impl_adc_pin!(ADC2, PA2, AdcChannel::Ch2);
impl_adc_pin!(ADC2, PA3, AdcChannel::Ch3);
impl_adc_pin!(ADC2, PC0, AdcChannel::Ch10);
impl_adc_pin!(ADC2, PC1, AdcChannel::Ch11);
impl_adc_pin!(ADC2, PC2, AdcChannel::Ch12);
impl_adc_pin!(ADC2, PC3, AdcChannel::Ch13);
impl_adc_pin!(ADC3, PA0, AdcChannel::Ch0);
impl_adc_pin!(ADC3, PA1, AdcChannel::Ch1);
impl_adc_pin!(ADC3, PA2, AdcChannel::Ch2);
impl_adc_pin!(ADC3, PA3, AdcChannel::Ch3);
impl_adc_pin!(ADC3, PC0, AdcChannel::Ch10);
impl_adc_pin!(ADC3, PC1, AdcChannel::Ch11);
impl_adc_pin!(ADC3, PC2, AdcChannel::Ch12);
impl_adc_pin!(ADC3, PC3, AdcChannel::Ch13);

// ADC12 pins (PA4-7 = IN4-7 + DAC out, PB0/1 = IN8/9, PC4/5 = IN14/15)
impl_adc_pin!(ADC1, PA4, AdcChannel::Ch4);
impl_adc_pin!(ADC1, PA5, AdcChannel::Ch5);
impl_adc_pin!(ADC1, PA6, AdcChannel::Ch6);
impl_adc_pin!(ADC1, PA7, AdcChannel::Ch7);
impl_adc_pin!(ADC1, PB0, AdcChannel::Ch8);
impl_adc_pin!(ADC1, PB1, AdcChannel::Ch9);
impl_adc_pin!(ADC1, PC4, AdcChannel::Ch14);
impl_adc_pin!(ADC1, PC5, AdcChannel::Ch15);
impl_adc_pin!(ADC2, PA4, AdcChannel::Ch4);
impl_adc_pin!(ADC2, PA5, AdcChannel::Ch5);
impl_adc_pin!(ADC2, PA6, AdcChannel::Ch6);
impl_adc_pin!(ADC2, PA7, AdcChannel::Ch7);
impl_adc_pin!(ADC2, PB0, AdcChannel::Ch8);
impl_adc_pin!(ADC2, PB1, AdcChannel::Ch9);
impl_adc_pin!(ADC2, PC4, AdcChannel::Ch14);
impl_adc_pin!(ADC2, PC5, AdcChannel::Ch15);

// ADC3-only extra pins (PF3-PF10)
impl_adc_pin!(ADC3, PF3, AdcChannel::Ch9);
impl_adc_pin!(ADC3, PF4, AdcChannel::Ch14);
impl_adc_pin!(ADC3, PF5, AdcChannel::Ch15);
impl_adc_pin!(ADC3, PF6, AdcChannel::Ch4);
impl_adc_pin!(ADC3, PF7, AdcChannel::Ch5);
impl_adc_pin!(ADC3, PF8, AdcChannel::Ch6);
impl_adc_pin!(ADC3, PF9, AdcChannel::Ch7);
impl_adc_pin!(ADC3, PF10, AdcChannel::Ch8);

// DMA routing impls (any channel can serve any ADC request via the DMAMUX)
macro_rules! impl_adc_dma_routes {
    ($($ch:ident),*) => {
        $(
            impl RxDma<crate::peripherals::ADC1> for crate::peripherals::$ch {
                #[inline] fn request() -> crate::dma::Request { crate::dma::Request::Adc1 }
            }
            impl RxDma<crate::peripherals::ADC2> for crate::peripherals::$ch {
                #[inline] fn request() -> crate::dma::Request { crate::dma::Request::Adc2 }
            }
            impl RxDma<crate::peripherals::ADC3> for crate::peripherals::$ch {
                #[inline] fn request() -> crate::dma::Request { crate::dma::Request::Adc3 }
            }
        )*
    };
}

impl_adc_dma_routes!(
    DMA1_CH1, DMA1_CH2, DMA1_CH3, DMA1_CH4, DMA1_CH5, DMA1_CH6, DMA1_CH7, //
    DMA2_CH1, DMA2_CH2, DMA2_CH3, DMA2_CH4, DMA2_CH5, DMA2_CH6, DMA2_CH7
);
