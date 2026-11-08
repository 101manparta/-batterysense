//! OTA Module — Dual-bank A/B firmware update with signature verification

pub mod bootloader;
pub mod manifest;

pub use bootloader::{Bootloader, BootloaderError, Bank};
pub use manifest::{OtaManifest, ManifestError, FirmwareMetadata};