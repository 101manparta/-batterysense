//! BatterySense Firmware — STM32G474RE + ADS1262 × 16-ch + CAN FD + Dual-bank OTA
//!
//! Architecture: embassy async runtime, zero-heap allocation, safety-critical tasks isolated

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]
#![feature(async_fn_in_trait)]
#![feature(generic_const_exprs)]
#![deny(unsafe_code)]
#![deny(missing_docs)]

use embassy_executor::Spawner;
use embassy_stm32::{
    bind_interrupts, can, peripherals, spi, usb, Config,
    can::fdcan::{Fdcan, RxFifo, TxFifo},
    gpio::{Level, Output, Speed},
    time::Hertz,
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

mod adc;
mod can;
mod safety;
mod ota;
mod comm;

use crate::{
    adc::{Ads1262Driver, AdcConfig, AdcSample},
    can::{CanTask, CanFrame, J1939Encoder},
    safety::{WatchdogTask, PlausibilityChecker, SafetyEvent},
    ota::{Bootloader, OtaManifest, OtaState},
    comm::{UsbCdcTask, BleTask, CliParser},
};

/// Global channels for inter-task communication
static ADC_CHANNEL: Channel<CriticalSectionRawMutex, AdcSample, 32> = Channel::new();
static CAN_TX_CHANNEL: Channel<CriticalSectionRawMutex, CanFrame, 16> = Channel::new();
static CAN_RX_CHANNEL: Channel<CriticalSectionRawMutex, CanFrame, 16> = Channel::new();
static SAFETY_CHANNEL: Channel<CriticalSectionRawMutex, SafetyEvent, 8> = Channel::new();
static OTA_CHANNEL: Channel<CriticalSectionRawMutex, OtaManifest, 1> = Channel::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Initialize config: 170 MHz, HSE 25 MHz, PLL
    let mut config = Config::default();
    config.rcc.hse = Some(Hertz(25_000_000));
    config.rcc.pll1 = Some(embassy_stm32::rcc::Pll {
        source: embassy_stm32::rcc::PllSource::HSE,
        prediv: embassy_stm32::rcc::PllPreDiv::DIV5,
        mul: embassy_stm32::rcc::PllMul::MUL68,
        divp: Some(embassy_stm32::rcc::PllDiv::DIV2),
        divq: Some(embassy_stm32::rcc::PllDiv::DIV2),
        divr: Some(embassy_stm32::rcc::PllDiv::DIV2),
    });
    config.rcc.sys = embassy_stm32::rcc::Sysclk::PLL1_P;
    config.rcc.hclk = Some(Hertz(170_000_000));
    config.rcc.pclk1 = Some(Hertz(170_000_000));
    config.rcc.pclk2 = Some(Hertz(170_000_000));

    let p = embassy_stm32::init(config);

    // Initialize safety-critical watchdog first
    let watchdog = safety::WatchdogTask::new(p.IWDG, p.WWDG);
    spawner.must_spawn(watchdog_task(watchdog));

    // Initialize ADC (ADS1262 via SPI3)
    let spi_config = spi::Config::default();
    let spi = spi::Spi::new(
        p.SPI3,
        p.PB3,  // SCK
        p.PB4,  // MISO
        p.PB5,  // MOSI
        p.DMA1_CH1,  // RX DMA
        p.DMA1_CH2,  // TX DMA
        spi_config,
    );
    let adc_cs = Output::new(p.PB6, Level::High, Speed::VeryHigh);
    let adc_drdy = p.PB7;
    let adc_reset = Output::new(p.PB8, Level::High, Speed::VeryHigh);

    let adc_driver = Ads1262Driver::new(spi, adc_cs, adc_drdy, adc_reset, AdcConfig::default());
    spawner.must_spawn(adc_task(adc_driver));

    // Initialize CAN FD (FDCAN1)
    bind_interrupts!(struct Irqs {
        FDCAN1_IT0 => can::fdcan::InterruptHandler<peripherals::FDCAN1>;
        FDCAN1_IT1 => can::fdcan::InterruptHandler<peripherals::FDCAN1>;
    });
    let can = Fdcan::new(
        p.FDCAN1,
        p.PA11,  // RX
        p.PA12,  // TX
        Irqs,
        p.DMA1_CH3,  // RX FIFO0
        p.DMA1_CH4,  // TX FIFO
        can::fdcan::Config::default()
            .bit_timing(can::fdcan::BitTiming::new(
                Hertz(170_000_000),
                Hertz(2_000_000),  // 2 Mbps nominal
                Hertz(5_000_000),  // 5 Mbps data
            ))
            .rx_fifo0_size(16)
            .tx_fifo_size(8),
    );
    spawner.must_spawn(can_task(can));

    // Initialize USB CDC (USB_OTG_FS)
    let usb_driver = usb::Driver::new(p.USB_OTG_FS, p.PA12, p.PA11, Irqs);
    spawner.must_spawn(usb_cdc_task(usb_driver));

    // Initialize BLE (UART4 → nRF52840)
    let ble_uart = embassy_stm32::usart::Uart::new(
        p.UART4,
        p.PA0,  // TX
        p.PA1,  // RX
        Irqs,
        p.DMA1_CH5,
        p.DMA1_CH6,
        Default::default(),
    );
    spawner.must_spawn(ble_task(ble_uart));

    // Initialize plausibility checker
    let checker = PlausibilityChecker::new();
    spawner.must_spawn(plausibility_task(checker));

    // Initialize OTA bootloader
    let bootloader = Bootloader::new();
    spawner.must_spawn(ota_task(bootloader));

    // Main supervision loop
    spawner.must_spawn(supervision_task());

    defmt::info!("BatterySense firmware started — v{}", env!("CARGO_PKG_VERSION"));
}

#[embassy_executor::task]
async fn adc_task(mut driver: Ads1262Driver) {
    driver.init().await.expect("ADS1262 init failed");
    defmt::info!("ADC task started");

    loop {
        let sample = driver.read_all_channels().await;
        if ADC_CHANNEL.try_send(sample).is_err() {
            defmt::warn!("ADC channel full, dropping sample");
        }
        Timer::after_millis(1).await;  // 1 kHz sample rate
    }
}

#[embassy_executor::task]
async fn can_task(mut can: Fdcan<'static, peripherals::FDCAN1>) {
    can.init().await.expect("CAN init failed");
    defmt::info!("CAN task started");

    let mut encoder = J1939Encoder::new();

    loop {
        // TX path
        if let Ok(frame) = CAN_TX_CHANNEL.try_receive() {
            can.transmit(&frame).await.ok();
        }

        // RX path
        if let Ok(frame) = can.receive().await {
            CAN_RX_CHANNEL.try_send(frame).ok();
        }

        // Periodic J1939 broadcast (100 ms)
        Timer::after_millis(100).await;
        let bat_voltage = ADC_CHANNEL.try_receive().map(|s| s.pack_voltage_mv);
        if let Some(v) = bat_voltage {
            let frame = encoder.encode_battery_voltage(v);
            CAN_TX_CHANNEL.try_send(frame).ok();
        }
    }
}

#[embassy_executor::task]
async fn usb_cdc_task(driver: usb::Driver<'static, peripherals::USB_OTG_FS>) {
    let mut task = UsbCdcTask::new(driver);
    task.run().await;
}

#[embassy_executor::task]
async fn ble_task(uart: embassy_stm32::usart::Uart<'static, peripherals::UART4>) {
    let mut task = BleTask::new(uart);
    task.run().await;
}

#[embassy_executor::task]
async fn watchdog_task(mut wd: WatchdogTask) {
    wd.init().await;
    defmt::info!("Watchdog task started");

    loop {
        wd.kick().await;
        Timer::after_millis(50).await;  // Kick every 50ms (window: 100ms)
    }
}

#[embassy_executor::task]
async fn plausibility_task(mut checker: PlausibilityChecker) {
    defmt::info!("Plausibility task started");

    loop {
        if let Ok(sample) = ADC_CHANNEL.try_receive() {
            if let Err(event) = checker.check(&sample) {
                SAFETY_CHANNEL.try_send(event).ok();
            }
        }
        Timer::after_millis(10).await;
    }
}

#[embassy_executor::task]
async fn ota_task(mut bootloader: Bootloader) {
    defmt::info!("OTA task started");

    loop {
        if let Ok(manifest) = OTA_CHANNEL.try_receive() {
            defmt::info!("OTA manifest received, verifying...");
            if bootloader.verify_and_stage(&manifest).await {
                defmt::info!("OTA staged successfully, requesting reboot");
                cortex_m::peripheral::SCB::sys_reset();
            }
        }
        Timer::after_secs(1).await;
    }
}

#[embassy_executor::task]
async fn supervision_task() {
    defmt::info!("Supervision task started");

    let mut last_heap = 0;
    loop {
        // Log system health every 30 seconds
        Timer::after_secs(30).await;

        let heap_used = embassy_sync::heapless::heap_used();
        if heap_used != last_heap {
            defmt::info!("Heap: {} bytes used", heap_used);
            last_heap = heap_used;
        }

        // Check task stack watermarks (if enabled)
        #[cfg(feature = "stack-watermark")]
        {
            defmt::info!("Stack watermarks: ADC={}, CAN={}, USB={}",
                embassy_executor::stack_watermark(adc_task),
                embassy_executor::stack_watermark(can_task),
                embassy_executor::stack_watermark(usb_cdc_task));
        }
    }
}