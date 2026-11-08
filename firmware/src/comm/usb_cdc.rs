//! USB CDC Task — Embedded stub
//!
//! This is a compile-time stub for the STM32 USB_OTG_FS peripheral.
//! In production, this drives the USB CDC serial interface.

use embassy_stm32::usb::Driver;
use embassy_stm32::Peripherals;

pub struct UsbCdcTask<'a> {
    _driver: Driver<'a, embassy_stm32::peripherals::USB_OTG_FS>,
}

impl<'a> UsbCdcTask<'a> {
    pub fn new(_driver: Driver<'a, embassy_stm32::peripherals::USB_OTG_FS>) -> Self {
        Self { _driver }
    }

    pub async fn run(&mut self) -> ! {
        defmt::info!("USB CDC task started");
        loop {
            embassy_time::Timer::after_secs(1).await;
        }
    }
}
