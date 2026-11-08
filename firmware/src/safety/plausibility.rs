//! Plausibility Checks — Cross-channel validation, dV/dt limits, stuck-at detection

use crate::adc::AdcSample;
use heapless::Vec;
use thiserror::Error;

#[derive(Debug, Error, Clone, Copy, defmt::Format)]
pub enum SafetyEvent {
    #[error("Cell overvoltage: ch={0}, v={1}mV > {2}mV")]
    CellOvervoltage(u8, f32, f32),
    #[error("Cell undervoltage: ch={0}, v={1}mV < {2}mV")]
    CellUndervoltage(u8, f32, f32),
    #[error("Cell imbalance: max_delta={0}mV > {1}mV")]
    CellImbalance(f32, f32),
    #[error("dV/dt exceeded: ch={0}, dvdt={1}V/s > {2}V/s")]
    DVdtExceeded(u8, f32, f32),
    #[error("Stuck-at detection: ch={0}, same value for {1} samples")]
    StuckAt(u8, u32),
    #[error("Open wire detection: ch={0}")]
    OpenWire(u8),
    #[error("ADC communication fault")]
    AdcCommFault,
    #[error("Temperature out of range: ch={0}, T={1}°C")]
    TemperatureOutOfRange(u8, f32),
    #[error("Pack overvoltage: {0}mV > {1}mV")]
    PackOvervoltage(f32, f32),
    #[error("Pack undervoltage: {0}mV < {1}mV")]
    PackUndervoltage(f32, f32),
    #[error("Cross-channel mismatch: ch={0}, diff={1}mV")]
    CrossChannelMismatch(u8, f32),
}

#[derive(Debug, Clone, Copy, defmt::Format)]
pub struct PlausibilityConfig {
    pub cell_ov_threshold_mv: f32,
    pub cell_uv_threshold_mv: f32,
    pub max_cell_delta_mv: f32,
    pub max_dvdt_v_per_s: f32,
    pub stuck_at_samples: u32,
    pub open_wire_threshold_mv: f32,
    pub temp_min_c: f32,
    pub temp_max_c: f32,
    pub pack_ov_threshold_mv: f32,
    pub pack_uv_threshold_mv: f32,
    pub cross_channel_tolerance_mv: f32,
}

impl Default for PlausibilityConfig {
    fn default() -> Self {
        Self {
            cell_ov_threshold_mv: 3650.0,   // Li-ion 3.65V
            cell_uv_threshold_mv: 2500.0,   // Li-ion 2.5V
            max_cell_delta_mv: 50.0,        // 50mV max imbalance
            max_dvdt_v_per_s: 10.0,         // 10V/s max slew rate
            stuck_at_samples: 1000,         // 1000 samples at 1kHz = 1s
            open_wire_threshold_mv: 100.0,  // < 100mV = open wire
            temp_min_c: -40.0,
            temp_max_c: 85.0,
            pack_ov_threshold_mv: 58400.0,  // 16S * 3.65V
            pack_uv_threshold_mv: 40000.0,  // 16S * 2.5V
            cross_channel_tolerance_mv: 20.0,
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct ChannelHistory {
    last_voltage: f32,
    last_timestamp: u64,
    stuck_count: u32,
    last_values: [f32; 4],  // Ring buffer for noise detection
    ring_idx: usize,
}

impl Default for ChannelHistory {
    fn default() -> Self {
        Self {
            last_voltage: 0.0,
            last_timestamp: 0,
            stuck_count: 0,
            last_values: [0.0; 4],
            ring_idx: 0,
        }
    }
}

pub struct PlausibilityChecker {
    config: PlausibilityConfig,
    history: [ChannelHistory; 16],
    pack_history: ChannelHistory,
    sample_count: u64,
}

impl PlausibilityChecker {
    pub fn new() -> Self {
        Self {
            config: PlausibilityConfig::default(),
            history: [ChannelHistory::default(); 16],
            pack_history: ChannelHistory::default(),
            sample_count: 0,
        }
    }

    pub fn with_config(mut self, config: PlausibilityConfig) -> Self {
        self.config = config;
        self
    }

    pub fn check(&mut self, sample: &AdcSample) -> Result<(), SafetyEvent> {
        self.sample_count += 1;

        // 1. Cell over/under voltage
        if sample.voltage_mv > self.config.cell_ov_threshold_mv {
            return Err(SafetyEvent::CellOvervoltage(
                sample.channel, sample.voltage_mv, self.config.cell_ov_threshold_mv));
        }
        if sample.voltage_mv < self.config.cell_uv_threshold_mv {
            return Err(SafetyEvent::CellUndervoltage(
                sample.channel, sample.voltage_mv, self.config.cell_uv_threshold_mv));
        }

        // 2. Open wire detection
        if sample.voltage_mv < self.config.open_wire_threshold_mv {
            return Err(SafetyEvent::OpenWire(sample.channel));
        }

        // 3. Temperature range
        if sample.channel == 0 {  // Temperature read on channel 0
            if sample.temperature_c < self.config.temp_min_c || sample.temperature_c > self.config.temp_max_c {
                return Err(SafetyEvent::TemperatureOutOfRange(sample.channel, sample.temperature_c));
            }
        }

        // 4. dV/dt check (per channel)
        let hist = &mut self.history[sample.channel as usize];
        if hist.last_timestamp > 0 {
            let dt_s = (sample.timestamp_us - hist.last_timestamp) as f32 / 1_000_000.0;
            if dt_s > 0.0 {
                let dvdt = (sample.voltage_mv - hist.last_voltage).abs() / dt_s / 1000.0;  // V/s
                if dvdt > self.config.max_dvdt_v_per_s {
                    return Err(SafetyEvent::DVdtExceeded(
                        sample.channel, dvdt, self.config.max_dvdt_v_per_s));
                }
            }
        }

        // 5. Stuck-at detection
        if (sample.voltage_mv - hist.last_voltage).abs() < 0.1 {  // < 0.1mV change
            hist.stuck_count += 1;
            if hist.stuck_count >= self.config.stuck_at_samples {
                return Err(SafetyEvent::StuckAt(sample.channel, hist.stuck_count));
            }
        } else {
            hist.stuck_count = 0;
        }

        // Update ring buffer for noise detection
        hist.last_values[hist.ring_idx] = sample.voltage_mv;
        hist.ring_idx = (hist.ring_idx + 1) % 4;

        // Check for excessive noise (all 4 recent values identical = suspicious)
        if hist.last_values.iter().all(|&v| (v - sample.voltage_mv).abs() < 0.01) && self.sample_count > 10 {
            // Could be stuck ADC - flag but don't trip immediately
            defmt::warn!("Possible ADC stuck: ch={}, val={}", sample.channel, sample.voltage_mv);
        }

        hist.last_voltage = sample.voltage_mv;
        hist.last_timestamp = sample.timestamp_us;

        // 6. Pack-level checks (only when we have full pack data)
        if sample.channel == 15 {  // Last channel in 16S pack
            self.check_pack_level(sample.pack_voltage_mv)?;
        }

        Ok(())
    }

    fn check_pack_level(&mut self, pack_voltage_mv: f32) -> Result<(), SafetyEvent> {
        // Pack over/under voltage
        if pack_voltage_mv > self.config.pack_ov_threshold_mv {
            return Err(SafetyEvent::PackOvervoltage(pack_voltage_mv, self.config.pack_ov_threshold_mv));
        }
        if pack_voltage_mv < self.config.pack_uv_threshold_mv {
            return Err(SafetyEvent::PackUndervoltage(pack_voltage_mv, self.config.pack_uv_threshold_mv));
        }

        // Cell imbalance (need all channels - simplified using min/max from history)
        let mut min_v = f32::MAX;
        let mut max_v = f32::MIN;
        let mut valid_count = 0;

        for hist in &self.history {
            if hist.last_timestamp > 0 && hist.last_voltage > self.config.open_wire_threshold_mv {
                min_v = min_v.min(hist.last_voltage);
                max_v = max_v.max(hist.last_voltage);
                valid_count += 1;
            }
        }

        if valid_count >= 2 {
            let delta = max_v - min_v;
            if delta > self.config.max_cell_delta_mv {
                return Err(SafetyEvent::CellImbalance(delta, self.config.max_cell_delta_mv));
            }
        }

        // dV/dt pack level
        if self.pack_history.last_timestamp > 0 {
            // Note: we don't have pack timestamp in AdcSample, approximate
            // In real impl: track pack timestamp separately
        }

        self.pack_history.last_voltage = pack_voltage_mv;

        Ok(())
    }

    /// Cross-channel plausibility (redundant measurement comparison)
    /// For systems with dual ADC or redundant channels
    pub fn check_cross_channel(&self, ch1: u8, ch2: u8, v1: f32, v2: f32) -> Result<(), SafetyEvent> {
        let diff = (v1 - v2).abs();
        if diff > self.config.cross_channel_tolerance_mv {
            return Err(SafetyEvent::CrossChannelMismatch(ch1, diff));
        }
        Ok(())
    }

    /// Reset history for a channel (e.g., after calibration)
    pub fn reset_channel(&mut self, channel: u8) {
        if (channel as usize) < self.history.len() {
            self.history[channel as usize] = ChannelHistory::default();
        }
    }

    /// Get current cell imbalance
    pub fn get_imbalance_mv(&self) -> f32 {
        let mut min_v = f32::MAX;
        let mut max_v = f32::MIN;
        for hist in &self.history {
            if hist.last_timestamp > 0 && hist.last_voltage > self.config.open_wire_threshold_mv {
                min_v = min_v.min(hist.last_voltage);
                max_v = max_v.max(hist.last_voltage);
            }
        }
        if min_v < f32::MAX {
            max_v - min_v
        } else {
            0.0
        }
    }
}

impl Default for PlausibilityChecker {
    fn default() -> Self {
        Self::new()
    }
}