//! BLE Task — Embedded stub
//!
//! This is a compile-time stub for UART4 -> nRF52840 BLE module.

use embassy_stm32::usart::Uart;
use embassy_stm32::peripherals::UART4;

pub struct BleTask<'a> {
    _uart: Uart<'a, UART4>,
}

impl<'a> BleTask<'a> {
    pub fn new(_uart: Uart<'a, UART4>) -> Self {
        Self { _uart }
    }

    pub async fn run(&mut self) -> ! {
        defmt::info!("BLE task started");
        loop {
            embassy_time::Timer::after_secs(1).await;
        }
    }
}
