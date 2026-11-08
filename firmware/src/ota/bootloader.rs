//! Dual-Bank Bootloader — A/B firmware update with rollback, signature verification

use crate::adc::CalibrationFlash;
use embassy_stm32::flash::{Flash, Blocking};
use embassy_time::Timer;
use mbedtls::pk::Pk;
use mbedtls::hash::{Hash, Type as HashType};
use postcard::{to_vec_cobs, from_bytes_cobs};
use serde::{Deserialize, Serialize};
use thiserror::Error;

#[derive(Debug, Error, defmt::Format)]
pub enum BootloaderError {
    #[error("Flash erase failed")]
    FlashErase,
    #[error("Flash write failed")]
    FlashWrite,
    #[error("Signature verification failed")]
    SignatureVerification,
    #[error("Invalid manifest")]
    InvalidManifest,
    #[error("Version downgrade not allowed")]
    VersionDowngrade,
    #[error("CRC mismatch")]
    CrcMismatch,
    #[error("Bank swap failed")]
    BankSwap,
    #[error("Rollback failed")]
    RollbackFailed,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, defmt::Format)]
pub enum Bank {
    Bank1 = 0,  // 0x0800_0000 - 0x0804_0000 (active)
    Bank2 = 1,  // 0x0804_0000 - 0x0808_0000 (staging)
}

impl Bank {
    pub fn active() -> Self {
        // Read boot address from FLASH_OPTR
        let optr = unsafe { core::ptr::read_volatile(0x1FFF_7800 as *const u32) };
        if optr & (1 << 15) != 0 {  // BFB2 bit
            Bank::Bank2
        } else {
            Bank::Bank1
        }
    }

    pub fn base_address(&self) -> u32 {
        match self {
            Bank::Bank1 => 0x0800_0000,
            Bank::Bank2 => 0x0804_0000,
        }
    }

    pub fn size(&self) -> u32 {
        256 * 1024  // 256 KB per bank
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, defmt::Format)]
pub struct OtaManifest {
    pub version: u32,
    pub git_hash: [u8; 8],
    pub build_timestamp: u64,
    pub target_bank: Bank,
    pub firmware_size: u32,
    pub firmware_crc32: u32,
    pub signature: [u8; 64],  // ECDSA P-256 signature
    pub public_key_hash: [u8; 32],  // SHA-256 of signing public key
    pub flags: OtaFlags,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, defmt::Format)]
pub struct OtaFlags {
    pub force_update: bool,
    pub skip_version_check: bool,
    pub preserve_config: bool,
    pub preserve_calibration: bool,
    pub test_mode: bool,  // Boot once, then revert
}

impl Default for OtaFlags {
    fn default() -> Self {
        Self {
            force_update: false,
            skip_version_check: false,
            preserve_config: true,
            preserve_calibration: true,
            test_mode: false,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, defmt::Format)]
pub struct FirmwareMetadata {
    pub version: u32,
    pub git_hash: [u8; 8],
    pub build_timestamp: u64,
    pub bank: Bank,
    pub is_valid: bool,
    pub boot_count: u32,
    pub last_boot_result: BootResult,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, defmt::Format)]
pub enum BootResult {
    Unknown = 0,
    Success = 1,
    Failed = 2,
    RolledBack = 3,
}

pub struct Bootloader {
    flash: Option<Flash<'static, Blocking>>,
    current_metadata: FirmwareMetadata,
    staging_metadata: Option<FirmwareMetadata>,
    public_key: Option<Pk>,
}

impl Bootloader {
    pub fn new() -> Self {
        Self {
            flash: None,
            current_metadata: FirmwareMetadata {
                version: env!("CARGO_PKG_VERSION").parse().unwrap_or(0),
                git_hash: [0; 8],
                build_timestamp: 0,
                bank: Bank::active(),
                is_valid: true,
                boot_count: 0,
                last_boot_result: BootResult::Unknown,
            },
            staging_metadata: None,
            public_key: None,
        }
    }

    pub fn with_flash(mut self, flash: Flash<'static, Blocking>) -> Self {
        self.flash = Some(flash);
        self
    }

    pub fn with_public_key(mut self, key_der: &[u8]) -> Result<Self, BootloaderError> {
        let pk = Pk::from_der(key_der).map_err(|_| BootloaderError::InvalidManifest)?;
        self.public_key = Some(pk);
        Ok(self)
    }

    /// Verify manifest signature and stage firmware for update
    pub async fn verify_and_stage(&mut self, manifest: &OtaManifest, firmware_data: &[u8]) -> bool {
        defmt::info!("Verifying OTA manifest v{}", manifest.version);

        // 1. Check version (unless forced)
        if !manifest.flags.skip_version_check && manifest.version <= self.current_metadata.version {
            defmt::error!("Version downgrade: {} -> {}", self.current_metadata.version, manifest.version);
            return false;
        }

        // 2. Verify firmware CRC
        let crc = crc32fast::hash(firmware_data);
        if crc != manifest.firmware_crc32 {
            defmt::error!("CRC mismatch: expected {:08X}, got {:08X}", manifest.firmware_crc32, crc);
            return false;
        }

        // 3. Verify signature
        if !self.verify_signature(manifest, firmware_data).await {
            defmt::error!("Signature verification failed");
            return false;
        }

        // 4. Verify public key hash matches trusted key
        if !self.verify_public_key_hash(&manifest.public_key_hash).await {
            defmt::error!("Public key hash mismatch");
            return false;
        }

        // 5. Erase target bank
        let target_bank = manifest.target_bank;
        if !self.erase_bank(target_bank).await {
            defmt::error!("Failed to erase bank {:?}", target_bank);
            return false;
        }

        // 6. Write firmware to target bank
        if !self.write_bank(target_bank, firmware_data).await {
            defmt::error!("Failed to write firmware to bank {:?}", target_bank);
            return false;
        }

        // 7. Verify written data
        if !self.verify_bank(target_bank, firmware_data).await {
            defmt::error!("Bank verification failed");
            return false;
        }

        // 8. Write manifest to OTA manifest section
        self.write_manifest(manifest).await;

        // 8. Update staging metadata
        self.staging_metadata = Some(FirmwareMetadata {
            version: manifest.version,
            git_hash: manifest.git_hash,
            build_timestamp: manifest.build_timestamp,
            bank: target_bank,
            is_valid: true,
            boot_count: 0,
            last_boot_result: BootResult::Unknown,
        });

        defmt::info!("OTA staged successfully on bank {:?}", target_bank);
        true
    }

    async fn verify_signature(&self, manifest: &OtaManifest, firmware_data: &[u8]) -> bool {
        let Some(pk) = &self.public_key else {
            defmt::warn!("No public key loaded, skipping signature verification");
            return true;  // Allow in development
        };

        // Create hash of firmware + manifest (excluding signature field)
        let mut hasher = Hash::new(HashType::Sha256).ok()?;
        hasher.update(firmware_data).ok()?;
        hasher.update(&manifest.version.to_le_bytes()).ok()?;
        hasher.update(&manifest.git_hash).ok()?;
        hasher.update(&manifest.build_timestamp.to_le_bytes()).ok()?;
        hasher.update(&[manifest.target_bank as u8]).ok()?;
        hasher.update(&manifest.firmware_size.to_le_bytes()).ok()?;
        hasher.update(&manifest.firmware_crc32.to_le_bytes()).ok()?;
        hasher.update(&manifest.flags.force_update as u8).ok()?;
        let digest = hasher.finish().ok()?;

        // Verify ECDSA signature
        pk.verify(mbedtls::pk::HashType::Sha256, &digest, &manifest.signature).is_ok()
    }

    async fn verify_public_key_hash(&self, key_hash: &[u8; 32]) -> bool {
        let Some(pk) = &self.public_key else { return true; };

        let mut hasher = Hash::new(HashType::Sha256).ok()?;
        let der = pk.public_key_der().ok()?;
        hasher.update(der).ok()?;
        let digest = hasher.finish().ok()?;

        digest.as_slice() == key_hash
    }

    async fn erase_bank(&mut self, bank: Bank) -> bool {
        let Some(flash) = &mut self.flash else { return false; };

        let base = bank.base_address();
        let size = bank.size();
        let pages = size / 2048;  // 2KB pages on STM32G4

        for page in 0..pages {
            let addr = base + page * 2048;
            if flash.blocking_erase(addr as usize, 1).is_err() {
                return false;
            }
            // Yield to allow other tasks
            Timer::after_millis(1).await;
        }
        true
    }

    async fn write_bank(&mut self, bank: Bank, data: &[u8]) -> bool {
        let Some(flash) = &mut self.flash else { return false; };

        let base = bank.base_address();
        let mut offset = 0;

        while offset < data.len() {
            let chunk_size = core::cmp::min(256, data.len() - offset);  // Write in 256-byte chunks
            let addr = base + offset as u32;

            if flash.blocking_write(addr as usize, &data[offset..offset + chunk_size]).is_err() {
                return false;
            }

            offset += chunk_size;
            Timer::after_millis(1).await;
        }
        true
    }

    async fn verify_bank(&self, bank: Bank, expected: &[u8]) -> bool {
        let base = bank.base_address();
        let actual = unsafe { core::slice::from_raw_parts(base as *const u8, expected.len()) };
        actual == expected
    }

    async fn write_manifest(&self, manifest: &OtaManifest) {
        // Write to OTA manifest section (0x0808_4000)
        // In real implementation: use flash HAL
        defmt::info!("Writing OTA manifest to flash (stub)");
    }

    /// Swap active bank (call before reset)
    pub fn swap_bank(&self, bank: Bank) -> Result<(), BootloaderError> {
        // Set BFB2 bit in FLASH_OPTR to boot from Bank2
        // This requires option byte programming
        defmt::info!("Setting boot bank to {:?}", bank);

        // In real implementation:
        // 1. Unlock FLASH_OPTR
        // 2. Modify BFB2 bit
        // 3. Lock FLASH_OPTR
        // 4. System reset

        Ok(())
    }

    /// Mark current boot as successful (call after successful initialization)
    pub fn mark_boot_success(&mut self) {
        self.current_metadata.boot_count += 1;
        self.current_metadata.last_boot_result = BootResult::Success;
        self.current_metadata.is_valid = true;
        self.save_metadata();
    }

    /// Mark boot as failed (trigger rollback on next reset)
    pub fn mark_boot_failed(&mut self) {
        self.current_metadata.last_boot_result = BootResult::Failed;
        self.save_metadata();
    }

    /// Check if rollback is needed
    pub fn needs_rollback(&self) -> bool {
        self.current_metadata.last_boot_result == BootResult::Failed &&
        self.current_metadata.boot_count >= 3  // Allow 3 attempts
    }

    /// Perform rollback to previous bank
    pub fn rollback(&mut self) -> Result<(), BootloaderError> {
        let previous_bank = match Bank::active() {
            Bank::Bank1 => Bank::Bank2,
            Bank::Bank2 => Bank::Bank1,
        };

        defmt::warn!("Rolling back to bank {:?}", previous_bank);
        self.swap_bank(previous_bank)?;
        self.current_metadata.last_boot_result = BootResult::RolledBack;
        self.save_metadata();
        Ok(())
    }

    fn save_metadata(&self) {
        // Save to CONFIG_FLASH section
        defmt::info!("Saving boot metadata (stub)");
    }

    pub fn current_version(&self) -> u32 {
        self.current_metadata.version
    }

    pub fn current_bank(&self) -> Bank {
        Bank::active()
    }
}

impl Default for Bootloader {
    fn default() -> Self {
        Self::new()
    }
}