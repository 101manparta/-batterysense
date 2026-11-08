//! ADC Module — ADS1262 32-bit ΔΣ ADC driver with calibration & temperature compensation

pub mod ads1262;
pub mod calibration;

pub use ads1262::{Ads1262Driver, AdcConfig, AdcSample, AdcError};
pub use calibration::{CalibrationTable, PolynomialCoeffs, TemperatureCompensation};