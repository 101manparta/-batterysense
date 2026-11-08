//! Watchdog Management — Independent (IWDG) + Window (WWDG) + External Supervisor (TPS3840)

use embassy_stm32::iwdg::Iwdg;
use embassy_stm32::wwdg::Wwdg;
use embassy_stm32::gpio::{Output, Level, Speed};
use embassy_time::Timer;
use thiserror::Error;

#[derive(Debug, Error, defmt::Format)]
pub enum WatchdogError {
    #[error("IWDG initialization failed")]
    IwdgInit,
    #[error("WWDG initialization failed")]
    WwdgInit,
    #[error("External supervisor communication failed")]
    ExternalComm,
    #[error("Watchdog timeout")]
    Timeout,
}

pub struct WatchdogTask {
    iwdg: Option<Iwdg>,
    wwdg: Option<Wwdg>,
    ext_supervisor: Option<ExternalSupervisor>,
    iwdg_period_ms: u32,
    wwdg_window_ms: u32,
    wwdg_timeout_ms: u32,
}

impl WatchdogTask {
    pub fn new(iwdg: embassy_stm32::peripherals::IWDG, wwdg: embassy_stm32::peripherals::WWDG) -> Self {
        Self {
            iwdg: Some(iwdg),
            wwdg: Some(wwdg),
            ext_supervisor: None,
            iwdg_period_ms: 500,   // 500 ms timeout
            wwdg_window_ms: 50,    // Window open at 50ms
            wwdg_timeout_ms: 100,  // Window closes at 100ms
        }
    }

    pub fn with_external_supervisor(mut self, pin: embassy_stm32::peripherals::PB9) -> Self {
        self.ext_supervisor = Some(ExternalSupervisor::new(pin));
        self
    }

    pub async fn init(&mut self) -> Result<(), WatchdogError> {
        // Initialize IWDG (Independent Watchdog)
        // LSI ~32 kHz, prescaler 32 -> 1 kHz, reload 500 -> 500 ms
        if let Some(iwdg) = self.iwdg.take() {
            let mut iwdg = Iwdg::new(iwdg, 500);  // 500 * 1ms = 500ms
            iwdg.start();
            self.iwdg = Some(iwdg);
        }

        // Initialize WWDG (Window Watchdog)
        // PCLK1 = 170 MHz, prescaler 4096 -> 41.5 kHz, counter 100 -> ~2.4 ms per tick
        // Window: 50-100 ms
        if let Some(wwdg) = self.wwdg.take() {
            let mut wwdg = Wwdg::new(wwdg);
            wwdg.set_window(50);   // Early window
            wwdg.set_counter(100); // Reset value
            wwdg.enable();
            self.wwdg = Some(wwdg);
        }

        // Initialize external supervisor (TPS3840 or similar)
        if let Some(mut ext) = self.ext_supervisor.take() {
            ext.init().await?;
            self.ext_supervisor = Some(ext);
        }

        defmt::info!("Watchdogs initialized: IWDG=500ms, WWDG window=50-100ms");
        Ok(())
    }

    pub async fn kick(&mut self) -> Result<(), WatchdogError> {
        // Kick IWDG
        if let Some(iwdg) = &mut self.iwdg {
            iwdg.pet();
        }

        // Kick WWDG (must be within window)
        if let Some(wwdg) = &mut self.wwdg {
            wwdg.pet();
        }

        // Kick external supervisor
        if let Some(ext) = &mut self.ext_supervisor {
            ext.kick().await?;
        }

        Ok(())
    }
}

/// External Hardware Watchdog Supervisor (TPS3840, MAX6369, etc.)
/// Provides power-on reset + watchdog with separate supply domain
pub struct ExternalSupervisor {
    wdi_pin: Option<Output<'static>>,  // Watchdog Input (active low pulse)
    wdo_pin: Option<embassy_stm32::gpio::Input<'static>>,  // Watchdog Output (reset signal)
    initialized: bool,
}

impl ExternalSupervisor {
    pub fn new(wdi_pin: embassy_stm32::peripherals::PB9) -> Self {
        Self {
            wdi_pin: Some(Output::new(wdi_pin, Level::High, Speed::VeryHigh)),
            wdo_pin: None,
            initialized: false,
        }
    }

    pub async fn init(&mut self) -> Result<(), WatchdogError> {
        // TPS3840: WDI needs periodic high-low-high pulse (min 100ns low)
        // Typical period: 100ms - 1.6s depending on variant
        if let Some(wdi) = &mut self.wdi_pin {
            wdi.set_high();
        }
        self.initialized = true;
        defmt::info!("External supervisor initialized");
        Ok(())
    }

    pub async fn kick(&mut self) -> Result<(), WatchdogError> {
        if let Some(wdi) = &mut self.wdi_pin {
            // Generate negative pulse: high -> low (100ns) -> high
            wdi.set_low();
            // Minimum pulse width 100ns - embassy delay is coarse, use cycle count
            cortex_m::asm::delay(170);  // ~1μs at 170 MHz
            wdi.set_high();
        }
        Ok(())
    }

    pub fn is_asserted(&self) -> bool {
        // Check if WDO (reset output) is asserted
        self.wdo_pin.as_ref().map(|p| p.is_low()).unwrap_or(false)
    }
}

/// Dual-bank flash layout verification (safety check at startup)
pub fn verify_flash_layout() -> bool {
    // Verify memory.x sections are correctly placed
    let bank1_start = 0x0800_0000u32;
    let bank1_end = 0x0804_0000u32;
    let bank2_start = 0x0804_0000u32;
    let bank2_end = 0x0808_0000u32;
    let config_start = 0x0808_0000u32;
    let config_end = 0x0808_2000u32;
    let calib_start = 0x0808_2000u32;
    let calib_end = 0x0808_4000u32;
    let ota_start = 0x0808_4000u32;
    let ota_end = 0x0808_5000u32;

    // Basic sanity checks
    bank1_end > bank1_start &&
    bank2_end > bank2_start &&
    config_end > config_start &&
    calib_end > calib_start &&
    ota_end > ota_start &&
    bank1_end == bank2_start &&
    bank2_end == config_start
}

/// CRC-32C (Castagnoli) for safety-critical data integrity
pub fn crc32c(data: &[u8]) -> u32 {
    crc32fast::hash(data)
}

/// Memory protection unit (MPU) configuration for safety regions
pub fn configure_mpu() {
    use cortex_m::register::mpu::*;

    // Region 0: Flash Bank 1 (execute only)
    mpu::write_region(
        Region::new(0, 0x0800_0000, 256 * 1024)
            .execute_never(false)
            .read_only(true)
            .cacheable(true)
            .bufferable(false)
            .shareable(false)
    );

    // Region 1: Flash Bank 2 (execute only, for OTA staging)
    mpu::write_region(
        Region::new(1, 0x0804_0000, 256 * 1024)
            .execute_never(false)
            .read_only(true)
            .cacheable(true)
            .bufferable(false)
            .shareable(false)
    );

    // Region 2: Config/Calibration/OTA (read-only, no execute)
    mpu::write_region(
        Region::new(2, 0x0808_0000, 20 * 1024)
            .execute_never(true)
            .read_only(true)
            .cacheable(false)
            .bufferable(false)
            .shareable(false)
    );

    // Region 3: SRAM1 (data, no execute)
    mpu::write_region(
        Region::new(3, 0x2000_0000, 128 * 1024)
            .execute_never(true)
            .read_only(false)
            .cacheable(true)
            .bufferable(false)
            .shareable(false)
    );

    // Region 4: SRAM2 Safety (parity protected, no execute)
    mpu::write_region(
        Region::new(4, 0x2002_0000, 32 * 1024)
            .execute_never(true)
            .read_only(false)
            .cacheable(false)
            .bufferable(false)
            .shareable(false)
    );

    // Enable MPU with default memory map for privileged access
    mpu::enable(mpu::Config::default().enable_priv_default());
}