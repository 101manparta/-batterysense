//! OTA Manifest — Firmware update package metadata and verification structures
//!
//! Defines the manifest format used for dual-bank A/B OTA updates.
//! The manifest is signed with ECDSA P-256 and verified before flashing.

use heapless::Vec;
use serde::{Deserialize, Serialize};
use thiserror::Error;

/// OTA Manifest — metadata accompanying a firmware image
#[derive(Debug, Clone, Serialize, Deserialize, defmt::Format)]
pub struct OtaManifest {
    /// Firmware version (semver major.minor.patch encoded as u32)
    pub version: u32,
    /// Short git commit hash (8 bytes)
    pub git_hash: [u8; 8],
    /// Unix timestamp of build
    pub build_timestamp: u64,
    /// Target flash bank (Bank1 or Bank2)
    pub target_bank: crate::ota::Bank,
    /// Size of firmware image in bytes
    pub firmware_size: u32,
    /// CRC-32C of firmware image
    pub firmware_crc32: u32,
    /// ECDSA P-256 signature (64 bytes, DER-encoded r || s)
    pub signature: [u8; 64],
    /// SHA-256 of the trusted signing public key (32 bytes)
    pub public_key_hash: [u8; 32],
    /// Update behavior flags
    pub flags: OtaFlags,
}

/// Flags controlling OTA update behavior
#[derive(Debug, Clone, Copy, Serialize, Deserialize, defmt::Format)]
pub struct OtaFlags {
    /// Force install even if version is lower or equal
    pub force_update: bool,
    /// Skip version comparison check
    pub skip_version_check: bool,
    /// Preserve configuration sector during update
    pub preserve_config: bool,
    /// Preserve calibration data during update
    pub preserve_calibration: bool,
    /// Boot once into new firmware, then revert on failure
    pub test_mode: bool,
}

/// Metadata about currently installed firmware in a bank
#[derive(Debug, Clone, Serialize, Deserialize, defmt::Format)]
pub struct FirmwareMetadata {
    pub version: u32,
    pub git_hash: [u8; 8],
    pub build_timestamp: u64,
    pub bank: crate::ota::Bank,
    pub is_valid: bool,
    pub boot_count: u32,
    pub last_boot_result: crate::ota::BootResult,
}

/// Errors that can occur during manifest processing
#[derive(Debug, Error, defmt::Format)]
pub enum ManifestError {
    #[error("Manifest serialization failed")]
    Serialization,
    #[error("Manifest deserialization failed")]
    Deserialization,
    #[error("Invalid manifest format")]
    InvalidFormat,
    #[error("Version too old")]
    VersionTooOld,
    #[error("Signature invalid")]
    InvalidSignature,
}

impl OtaManifest {
    /// Compute the canonical byte representation for signing/verification
    /// (excludes the signature field itself)
    pub fn signable_bytes(&self) -> Vec<u8, 256> {
        let mut buf = Vec::new();
        buf.extend_from_slice(&self.version.to_le_bytes()).ok();
        buf.extend_from_slice(&self.git_hash).ok();
        buf.extend_from_slice(&self.build_timestamp.to_le_bytes()).ok();
        buf.extend_from_slice(&[self.target_bank as u8]).ok();
        buf.extend_from_slice(&self.firmware_size.to_le_bytes()).ok();
        buf.extend_from_slice(&self.firmware_crc32.to_le_bytes()).ok();
        buf.extend_from_slice(&[
            self.flags.force_update as u8,
            self.flags.skip_version_check as u8,
            self.flags.preserve_config as u8,
            self.flags.preserve_calibration as u8,
            self.flags.test_mode as u8,
        ])
        .ok();
        buf.extend_from_slice(&self.public_key_hash).ok();
        buf
    }

    /// Validate internal consistency of the manifest
    pub fn validate(&self) -> Result<(), ManifestError> {
        if self.firmware_size == 0 {
            return Err(ManifestError::InvalidFormat);
        }
        if self.public_key_hash == [0u8; 32] {
            return Err(ManifestError::InvalidFormat);
        }
        Ok(())
    }
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_signable_bytes_excludes_signature() {
        let manifest = OtaManifest {
            version: 1,
            git_hash: [0; 8],
            build_timestamp: 0,
            target_bank: crate::ota::Bank::Bank1,
            firmware_size: 1024,
            firmware_crc32: 0xDEADBEEF,
            signature: [0xAA; 64],
            public_key_hash: [0xBB; 32],
            flags: OtaFlags::default(),
        };
        let bytes = manifest.signable_bytes();
        // Signature (0xAA bytes) should NOT be in signable bytes
        assert!(!bytes.iter().any(|&b| b == 0xAA));
    }

    #[test]
    fn test_validate_rejects_zero_size() {
        let manifest = OtaManifest {
            version: 1,
            git_hash: [0; 8],
            build_timestamp: 0,
            target_bank: crate::ota::Bank::Bank1,
            firmware_size: 0,
            firmware_crc32: 0,
            signature: [0; 64],
            public_key_hash: [1; 32],
            flags: OtaFlags::default(),
        };
        assert!(manifest.validate().is_err());
    }

    #[test]
    fn test_validate_rejects_zero_key_hash() {
        let manifest = OtaManifest {
            version: 1,
            git_hash: [0; 8],
            build_timestamp: 0,
            target_bank: crate::ota::Bank::Bank1,
            firmware_size: 1024,
            firmware_crc32: 0,
            signature: [0; 64],
            public_key_hash: [0; 32],
            flags: OtaFlags::default(),
        };
        assert!(manifest.validate().is_err());
    }

    #[test]
    fn test_validate_accepts_valid() {
        let manifest = OtaManifest {
            version: 1,
            git_hash: [0; 8],
            build_timestamp: 0,
            target_bank: crate::ota::Bank::Bank1,
            firmware_size: 1024,
            firmware_crc32: 0,
            signature: [0; 64],
            public_key_hash: [1; 32],
            flags: OtaFlags::default(),
        };
        assert!(manifest.validate().is_ok());
    }
}
