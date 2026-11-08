//! Communication Module — USB CDC, BLE, and CLI parser
//!
//! This module handles host communication interfaces.
//! On embedded targets, these are hardware-bound stubs.
//! The host simulator crate provides its own mock communication layer.

#![cfg_attr(not(feature = "simulator"), no_std)]

#[cfg(not(feature = "simulator"))]
pub mod usb_cdc;
#[cfg(not(feature = "simulator"))]
pub mod ble;

pub mod cli;

#[cfg(not(feature = "simulator"))]
pub use usb_cdc::UsbCdcTask;
#[cfg(not(feature = "simulator"))]
pub use ble::BleTask;

pub use cli::CliParser;
