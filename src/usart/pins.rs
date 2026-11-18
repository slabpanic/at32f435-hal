// AF numbers verified row-by-row against the RM V2.09 IOMUX tables 6-1..6-8
// (GPIOA..GPIOH mux tables, pages 114-127; all entries below come straight
// from those tables). mux 7 = USART family, mux 8 = the secondary USART/UART
// column. Entries confirmed on hardware additionally carry a note.
crate::pin_trait_impl!(crate::usart::TxPin, USART1, PA9, 7); // [V] official BSP + evb header
crate::pin_trait_impl!(crate::usart::TxPin, USART1, PA15, 7); // [V] table 6-1
crate::pin_trait_impl!(crate::usart::TxPin, USART1, PB6, 7); // [V] table 6-2
crate::pin_trait_impl!(crate::usart::RxPin, USART1, PA10, 7); // [V] official BSP + evb header
crate::pin_trait_impl!(crate::usart::RxPin, USART1, PB3, 7); // [V] table 6-2
crate::pin_trait_impl!(crate::usart::RxPin, USART1, PB7, 7); // [V] table 6-2

crate::pin_trait_impl!(crate::usart::TxPin, USART2, PA2, 7); // [V] official example
crate::pin_trait_impl!(crate::usart::TxPin, USART2, PD5, 7); // [V] official example
crate::pin_trait_impl!(crate::usart::TxPin, USART2, PA8, 8); // [V] table 6-1/6-2
crate::pin_trait_impl!(crate::usart::TxPin, USART2, PA14, 8); // [V] table 6-1
crate::pin_trait_impl!(crate::usart::RxPin, USART2, PA3, 7); // [V] official example
crate::pin_trait_impl!(crate::usart::RxPin, USART2, PD6, 7); // [V] official example
crate::pin_trait_impl!(crate::usart::RxPin, USART2, PA15, 8); // [V] table 6-1

crate::pin_trait_impl!(crate::usart::TxPin, USART3, PB10, 7); // [V] official example
crate::pin_trait_impl!(crate::usart::TxPin, USART3, PC4, 7); // [V] table 6-3
crate::pin_trait_impl!(crate::usart::TxPin, USART3, PC10, 7); // [V] table 6-3
crate::pin_trait_impl!(crate::usart::TxPin, USART3, PD8, 7); // [V] table 6-4
crate::pin_trait_impl!(crate::usart::RxPin, USART3, PB11, 7); // [V] official example
crate::pin_trait_impl!(crate::usart::RxPin, USART3, PC5, 7); // [V] table 6-3
crate::pin_trait_impl!(crate::usart::RxPin, USART3, PC11, 7); // [V] table 6-3
crate::pin_trait_impl!(crate::usart::RxPin, USART3, PD9, 7); // [V] table 6-4

crate::pin_trait_impl!(crate::usart::TxPin, USART6, PA4, 8); // [V] table 6-1
crate::pin_trait_impl!(crate::usart::TxPin, USART6, PA11, 8); // [V] table 6-1
crate::pin_trait_impl!(crate::usart::TxPin, USART6, PC6, 8); // [V] table 6-3
crate::pin_trait_impl!(crate::usart::TxPin, USART6, PG14, 8); // [V] table 6-7
crate::pin_trait_impl!(crate::usart::RxPin, USART6, PA5, 8); // [V] table 6-1
crate::pin_trait_impl!(crate::usart::RxPin, USART6, PA12, 8); // [V] table 6-1
crate::pin_trait_impl!(crate::usart::RxPin, USART6, PC7, 8); // [V] table 6-3
crate::pin_trait_impl!(crate::usart::RxPin, USART6, PG9, 8); // [V] table 6-7

crate::pin_trait_impl!(crate::usart::TxPin, UART4, PA0, 8); // [V] table 6-1
crate::pin_trait_impl!(crate::usart::TxPin, UART4, PC10, 8); // [V] table 6-3
crate::pin_trait_impl!(crate::usart::TxPin, UART4, PE8, 7); // [V] table 6-5
crate::pin_trait_impl!(crate::usart::TxPin, UART4, PH3, 8); // [V] table 6-8
crate::pin_trait_impl!(crate::usart::RxPin, UART4, PA1, 8); // [V] table 6-1
crate::pin_trait_impl!(crate::usart::RxPin, UART4, PC11, 8); // [V] table 6-3
crate::pin_trait_impl!(crate::usart::RxPin, UART4, PE9, 7); // [V] table 6-5
crate::pin_trait_impl!(crate::usart::RxPin, UART4, PH2, 8); // [V] table 6-8

crate::pin_trait_impl!(crate::usart::TxPin, UART5, PC12, 8); // [V] table 6-3
crate::pin_trait_impl!(crate::usart::TxPin, UART5, PB6, 8); // [V] table 6-2
crate::pin_trait_impl!(crate::usart::TxPin, UART5, PB9, 8); // [V] table 6-2
crate::pin_trait_impl!(crate::usart::TxPin, UART5, PE10, 8); // [V] table 6-5
crate::pin_trait_impl!(crate::usart::RxPin, UART5, PD2, 8); // [V] table 6-4
crate::pin_trait_impl!(crate::usart::RxPin, UART5, PB5, 8); // [V] table 6-2
crate::pin_trait_impl!(crate::usart::RxPin, UART5, PB8, 8); // [V] table 6-2
crate::pin_trait_impl!(crate::usart::RxPin, UART5, PE11, 8); // [V] table 6-5

crate::pin_trait_impl!(crate::usart::TxPin, UART7, PB4, 8); // [V] table 6-2
crate::pin_trait_impl!(crate::usart::TxPin, UART7, PE8, 8); // [V] table 6-5
crate::pin_trait_impl!(crate::usart::TxPin, UART7, PF7, 8); // [V] table 6-6
crate::pin_trait_impl!(crate::usart::RxPin, UART7, PB3, 8); // [V] table 6-2
crate::pin_trait_impl!(crate::usart::RxPin, UART7, PE7, 8); // [V] table 6-5
crate::pin_trait_impl!(crate::usart::RxPin, UART7, PF6, 8); // [V] table 6-6

crate::pin_trait_impl!(crate::usart::TxPin, UART8, PC8, 7); // [V] table 6-3
crate::pin_trait_impl!(crate::usart::TxPin, UART8, PC2, 8); // [V] table 6-3
crate::pin_trait_impl!(crate::usart::TxPin, UART8, PD13, 8); // [V] table 6-4
crate::pin_trait_impl!(crate::usart::TxPin, UART8, PE1, 8); // [V] table 6-5
crate::pin_trait_impl!(crate::usart::RxPin, UART8, PC9, 7); // [V] table 6-3
crate::pin_trait_impl!(crate::usart::RxPin, UART8, PC3, 8); // [V] table 6-3
crate::pin_trait_impl!(crate::usart::RxPin, UART8, PD14, 8); // [V] table 6-4
crate::pin_trait_impl!(crate::usart::RxPin, UART8, PE0, 8); // [V] table 6-5

// CTS/RTS pins (USART1/2/3/6 only - UART4/5/7/8 have no flow-control pins in
// RM tables 6-1..6-8). Entries from af_table_notes.md (page-image verified).
crate::pin_trait_impl!(crate::usart::CtsPin, USART1, PA11, 7);
crate::pin_trait_impl!(crate::usart::RtsPin, USART1, PA12, 7);
crate::pin_trait_impl!(crate::usart::CtsPin, USART2, PA0, 7);
crate::pin_trait_impl!(crate::usart::CtsPin, USART2, PD3, 7);
crate::pin_trait_impl!(crate::usart::RtsPin, USART2, PA1, 7);
crate::pin_trait_impl!(crate::usart::RtsPin, USART2, PD4, 7);
crate::pin_trait_impl!(crate::usart::CtsPin, USART3, PA6, 7);
crate::pin_trait_impl!(crate::usart::CtsPin, USART3, PD11, 7);
crate::pin_trait_impl!(crate::usart::CtsPin, USART3, PB13, 8);
crate::pin_trait_impl!(crate::usart::RtsPin, USART3, PB14, 7);
crate::pin_trait_impl!(crate::usart::RtsPin, USART3, PD2, 7);
crate::pin_trait_impl!(crate::usart::RtsPin, USART3, PD12, 7);
crate::pin_trait_impl!(crate::usart::RtsPin, USART3, PB1, 8);
crate::pin_trait_impl!(crate::usart::CtsPin, USART6, PG13, 8);
crate::pin_trait_impl!(crate::usart::CtsPin, USART6, PG15, 8);
crate::pin_trait_impl!(crate::usart::RtsPin, USART6, PG8, 8);
crate::pin_trait_impl!(crate::usart::RtsPin, USART6, PG12, 8);
