//! Calibration Module — Production calibration with uncertainty budget & temperature compensation

use crate::adc::{PolynomialCoeffs, CalibrationTable};
use heapless::Vec;
use micromath::F32Ext;
use serde::{Deserialize, Serialize};

/// Calibration coefficients stored in flash (per device)
#[derive(Debug, Clone, Copy, Serialize, Deserialize, defmt::Format)]
pub struct CalibrationData {
    pub device_id: u64,
    pub firmware_version: u32,
    pub calibration_date: u64,  // Unix timestamp
    pub temperature_c: f32,     // Calibration temperature
    pub coeffs: [PolynomialCoeffs; 16],
    pub uncertainty: UncertaintyBudget,
    pub crc32: u32,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, defmt::Format)]
pub struct UncertaintyBudget {
    pub reference_std_mv: f32,      // Reference standard uncertainty
    pub resolution_mv: f32,         // ADC resolution uncertainty
    pub linearity_mv: f32,          // Integral non-linearity
    pub temp_coeff_mv_per_c: f32,   // Temperature coefficient
    pub drift_mv_per_year: f32,     // Long-term drift
    pub combined_mv: f32,           // Combined standard uncertainty (k=1)
    pub expanded_mv: f32,           // Expanded uncertainty (k=2, 95%)
}

impl Default for UncertaintyBudget {
    fn default() -> Self {
        Self {
            reference_std_mv: 0.5,
            resolution_mv: 0.1,
            linearity_mv: 1.0,
            temp_coeff_mv_per_c: 0.05,
            drift_mv_per_year: 2.0,
            combined_mv: 1.14,
            expanded_mv: 2.28,
        }
    }
}

impl UncertaintyBudget {
    pub fn calculate(&mut self, temp_delta_c: f32, age_years: f32) {
        let temp_unc = self.temp_coeff_mv_per_c * temp_delta_c.abs();
        let drift_unc = self.drift_mv_per_year * age_years;

        self.combined_mv = libm::sqrtf(
            self.reference_std_mv * self.reference_std_mv +
            self.resolution_mv * self.resolution_mv +
            self.linearity_mv * self.linearity_mv +
            temp_unc * temp_unc +
            drift_unc * drift_unc
        );
        self.expanded_mv = self.combined_mv * 2.0;  // k=2
    }
}

/// Temperature compensation model for cell voltage
#[derive(Debug, Clone, Copy, Serialize, Deserialize, defmt::Format)]
pub struct TemperatureCompensation {
    pub ref_temp_c: f32,
    pub alpha: [f32; 16],   // V/°C per channel (typically -0.3 to -0.5 mV/°C for Li-ion)
    pub beta: [f32; 16],    // V/°C² per channel (quadratic term)
}

impl Default for TemperatureCompensation {
    fn default() -> Self {
        Self {
            ref_temp_c: 25.0,
            alpha: [-0.4; 16],
            beta: [0.001; 16],
        }
    }
}

impl TemperatureCompensation {
    pub fn compensate(&self, channel: u8, voltage_mv: f32, temp_c: f32) -> f32 {
        let dt = temp_c - self.ref_temp_c;
        let ch = channel as usize;
        voltage_mv + self.alpha[ch] * dt + self.beta[ch] * dt * dt
    }
}

/// Calibration wizard state machine (used by PC GUI)
#[derive(Debug, Clone, Copy, PartialEq, defmt::Format)]
pub enum CalibrationStep {
    Idle,
    ApplyKnownVoltage { channel: u8, expected_mv: f32 },
    Measure { channel: u8, expected_mv: f32 },
    ComputeCoeffs,
    Verify,
    SaveToFlash,
    Complete,
    Error,
}

/// Calibration point for polynomial fitting
#[derive(Debug, Clone, Copy, Serialize, Deserialize, defmt::Format)]
pub struct CalibrationPoint {
    pub channel: u8,
    pub applied_mv: f32,
    pub measured_raw: f32,
}

/// In-memory calibration session
pub struct CalibrationSession {
    pub points: Vec<CalibrationPoint, 64>,
    pub current_step: CalibrationStep,
    pub target_channel: u8,
    pub target_voltage: f32,
}

impl CalibrationSession {
    pub fn new() -> Self {
        Self {
            points: Vec::new(),
            current_step: CalibrationStep::Idle,
            target_channel: 0,
            target_voltage: 0.0,
        }
    }

    pub fn add_point(&mut self, channel: u8, applied_mv: f32, measured_raw: f32) -> Result<(), &'static str> {
        self.points.push(CalibrationPoint { channel, applied_mv, measured_raw })
            .map_err(|_| "Too many calibration points")?;
        Ok(())
    }

    pub fn compute_polynomial(&self, channel: u8) -> Option<PolynomialCoeffs> {
        let channel_points: Vec<_, 4> = self.points.iter()
            .filter(|p| p.channel == channel)
            .take(4)
            .cloned()
            .collect();

        if channel_points.len() < 3 {
            return None;
        }

        // Simple 3rd order polynomial fit using Lagrange interpolation
        // In production: use proper least squares with more points
        let x = [channel_points[0].measured_raw, channel_points[1].measured_raw, channel_points[2].measured_raw];
        let y = [channel_points[0].applied_mv, channel_points[1].applied_mv, channel_points[2].applied_mv];

        // Solve for cubic: y = c0 + c1*x + c2*x² + c3*x³
        // Using 3 points, assume c3 = 0 (quadratic fit)
        let c2 = (y[2] - y[0] - (y[1] - y[0]) * (x[2] - x[0]) / (x[1] - x[0]))
            / (x[2] * x[2] - x[0] * x[0] - (x[1] * x[1] - x[0] * x[0]) * (x[2] - x[0]) / (x[1] - x[0]));
        let c1 = (y[1] - y[0] - c2 * (x[1] * x[1] - x[0] * x[0])) / (x[1] - x[0]);
        let c0 = y[0] - c1 * x[0] - c2 * x[0] * x[0];

        Some(PolynomialCoeffs { c0, c1, c2, c3: 0.0 })
    }
}

/// Flash storage for calibration data (maps to .calibration section in memory.x)
pub struct CalibrationFlash;

impl CalibrationFlash {
    const BASE_ADDR: *mut u8 = 0x08082000 as *mut u8;
    const SIZE: usize = 8192;

    pub fn read() -> Option<CalibrationData> {
        unsafe {
            let ptr = Self::BASE_ADDR as *const CalibrationData;
            let data = core::ptr::read_volatile(ptr);
            if data.crc32 == Self::calculate_crc(&data) {
                Some(data)
            } else {
                None
            }
        }
    }

    pub fn write(data: &CalibrationData) -> Result<(), &'static str> {
        // In real implementation: use STM32 flash HAL with unlock/erase/write/lock
        // This is a placeholder
        defmt::info!("Writing calibration to flash (stub)");
        Ok(())
    }

    fn calculate_crc(data: &CalibrationData) -> u32 {
        let bytes = unsafe {
            core::slice::from_raw_parts(
                data as *const _ as *const u8,
                core::mem::size_of::<CalibrationData>() - 4
            )
        };
        crc32fast::hash(bytes)
    }
}