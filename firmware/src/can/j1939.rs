//! J1939 Encoder/Decoder — PGN/SPN mapping for battery management
//!
//! Reference: SAE J1939-71, J1939-73, ISO 11992

use heapless::Vec;
use postcard::{to_vec_cobs, from_bytes_cobs};
use serde::{Deserialize, Serialize};

/// J1939 Parameter Group Number
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, defmt::Format)]
pub struct Pgn(pub u32);

impl Pgn {
    // Battery-related PGNs (SAE J1939-71)
    pub const BATTERY_STATUS: Pgn = Pgn(0xFEF7);      // 65271 - Battery status
    pub const BATTERY_VOLTAGE: Pgn = Pgn(0xFEF8);     // 65272 - Battery voltage
    pub const BATTERY_CURRENT: Pgn = Pgn(0xFEF9);     // 65273 - Battery current
    pub const BATTERY_TEMP: Pgn = Pgn(0xFEFA);        // 65274 - Battery temperature
    pub const CELL_VOLTAGES: Pgn = Pgn(0xFEFB);       // 65275 - Individual cell voltages
    pub const CELL_TEMPS: Pgn = Pgn(0xFEFC);          // 65276 - Individual cell temperatures
    pub const BATTERY_CONFIG: Pgn = Pgn(0xFEFD);      // 65277 - Battery configuration
    pub const BATTERY_ALARMS: Pgn = Pgn(0xFEFE);      // 65278 - Battery alarms/warnings
    pub const BATTERY_SOH: Pgn = Pgn(0xFEFF);         // 65279 - State of Health
    pub const BATTERY_SOC: Pgn = Pgn(0xFF00);         // 65280 - State of Charge

    // Proprietary PGNs (manufacturer specific)
    pub const PROP_CALIBRATION: Pgn = Pgn(0xFF01);    // 65281 - Calibration data
    pub const PROP_OTA: Pgn = Pgn(0xFF02);            // 65282 - OTA firmware update
    pub const PROP_DIAG: Pgn = Pgn(0xFF03);           // 65283 - Diagnostics

    pub fn priority(&self) -> u8 {
        ((self.0 >> 26) & 0x07) as u8
    }

    pub fn source_address(&self) -> u8 {
        (self.0 & 0xFF) as u8
    }

    pub fn set_source(&mut self, addr: u8) {
        self.0 = (self.0 & !0xFF) | (addr as u32);
    }
}

/// J1939 Suspect Parameter Number
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, defmt::Format)]
pub struct Spn(pub u32);

impl Spn {
    // Battery SPNs
    pub const BATTERY_VOLTAGE: Spn = Spn(1632);
    pub const BATTERY_CURRENT: Spn = Spn(1633);
    pub const BATTERY_TEMP: Spn = Spn(1634);
    pub const CELL_VOLTAGE_MIN: Spn = Spn(1635);
    pub const CELL_VOLTAGE_MAX: Spn = Spn(1636);
    pub const CELL_VOLTAGE_AVG: Spn = Spn(1637);
    pub const CELL_VOLTAGE_DELTA: Spn = Spn(1638);
    pub const SOC: Spn = Spn(1639);
    pub const SOH: Spn = Spn(1640);
    pub const CYCLE_COUNT: Spn = Spn(1641);
    pub const ALARM_FLAGS: Spn = Spn(1642);
    pub const WARNING_FLAGS: Spn = Spn(1643);
}

/// CAN Frame with J1939 metadata
#[derive(Debug, Clone, Serialize, Deserialize, defmt::Format)]
pub struct CanFrame {
    pub id: u32,           // 29-bit CAN ID (priority + PGN + SA)
    pub dlc: u8,           // Data length code (0-64 for CAN FD)
    pub data: Vec<u8, 64>, // Up to 64 bytes for CAN FD
    pub timestamp_us: u64,
    pub is_fd: bool,
    pub bitrate_switch: bool,
}

impl CanFrame {
    pub fn new_j1939(pgn: Pgn, sa: u8, data: &[u8], priority: u8) -> Result<Self, &'static str> {
        let mut frame = Self {
            id: ((priority as u32) << 26) | (pgn.0 & 0x3FFFF) | (sa as u32),
            dlc: data.len() as u8,
            data: Vec::from_slice(data).map_err(|_| "Data too long")?,
            timestamp_us: embassy_time::Instant::now().as_micros(),
            is_fd: data.len() > 8,
            bitrate_switch: data.len() > 8,
        };
        Ok(frame)
    }

    pub fn pgn(&self) -> Pgn {
        Pgn(self.id & 0x3FFFF)
    }

    pub fn source_address(&self) -> u8 {
        (self.id & 0xFF) as u8
    }

    pub fn priority(&self) -> u8 {
        ((self.id >> 26) & 0x07) as u8
    }
}

/// J1939 Encoder for battery data
pub struct J1939Encoder {
    source_address: u8,
    sequence: u8,
}

impl J1939Encoder {
    pub fn new(source_address: u8) -> Self {
        Self { source_address, sequence: 0 }
    }

    pub fn encode_battery_voltage(&mut self, pack_voltage_mv: f32) -> CanFrame {
        // SPN 1632: Battery Voltage, 0.1 V/bit, offset 0
        let value = (pack_voltage_mv / 100.0) as u16;  // Convert mV to 0.1V units
        let data = [
            (value & 0xFF) as u8,
            (value >> 8) as u8,
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,  // Reserved
        ];
        CanFrame::new_j1939(Pgn::BATTERY_VOLTAGE, self.source_address, &data, 6).unwrap()
    }

    pub fn encode_cell_voltages(&mut self, voltages: &[f32]) -> Vec<CanFrame, 8> {
        // SPN 1635-1638: Min/Max/Avg/Delta cell voltage
        // PGN 65275 (Cell Voltages) - transport protocol for >8 bytes
        let mut frames = Vec::new();

        // Pack cell voltages into 2-byte each (0.1 mV resolution)
        let mut data = [0u8; 64];
        for (i, &v) in voltages.iter().enumerate().take(32) {
            let val = (v * 10.0) as u16;  // 0.1 mV per bit
            data[i * 2] = (val & 0xFF) as u8;
            data[i * 2 + 1] = (val >> 8) as u8;
        }

        // Split into 8-byte frames (classic CAN) or single 64-byte FD frame
        if voltages.len() * 2 <= 8 {
            frames.push(CanFrame::new_j1939(
                Pgn::CELL_VOLTAGES, self.source_address, &data[..voltages.len() * 2], 6).unwrap());
        } else {
            // Use CAN FD single frame
            let frame = CanFrame {
                id: ((6 << 26) | (Pgn::CELL_VOLTAGES.0 & 0x3FFFF) | (self.source_address as u32)),
                dlc: (voltages.len() * 2) as u8,
                data: Vec::from_slice(&data[..voltages.len() * 2]).unwrap(),
                timestamp_us: embassy_time::Instant::now().as_micros(),
                is_fd: true,
                bitrate_switch: true,
            };
            frames.push(frame);
        }

        frames
    }

    pub fn encode_battery_status(&mut self, soc: f32, soh: f32, cycle_count: u32, alarms: u16, warnings: u16) -> CanFrame {
        // PGN 65271: Battery Status
        let data = [
            (soc * 2.0) as u8,          // 0.5%/bit
            (soh * 2.0) as u8,          // 0.5%/bit
            (cycle_count & 0xFF) as u8,
            (cycle_count >> 8) as u8,
            (cycle_count >> 16) as u8,
            (cycle_count >> 24) as u8,
            (alarms & 0xFF) as u8,
            (alarms >> 8) as u8,
        ];
        CanFrame::new_j1939(Pgn::BATTERY_STATUS, self.source_address, &data, 6).unwrap()
    }

    pub fn encode_alarms(&mut self, cell_ov: u32, cell_uv: u32, pack_ov: bool, pack_uv: bool,
                         overtemp: bool, undertemp: bool, overcurrent: bool, comm_fault: bool) -> CanFrame {
        // PGN 65278: Battery Alarms
        let mut alarms = 0u32;
        alarms |= cell_ov & 0xFFFF;      // Cell overvoltage bitmask (16 cells)
        alarms |= (cell_uv & 0xFFFF) << 16;  // Cell undervoltage bitmask

        let mut warnings = 0u16;
        warnings |= (pack_ov as u16) << 0;
        warnings |= (pack_uv as u16) << 1;
        warnings |= (overtemp as u16) << 2;
        warnings |= (undertemp as u16) << 3;
        warnings |= (overcurrent as u16) << 4;
        warnings |= (comm_fault as u16) << 5;

        let data = [
            (alarms & 0xFF) as u8,
            ((alarms >> 8) & 0xFF) as u8,
            ((alarms >> 16) & 0xFF) as u8,
            ((alarms >> 24) & 0xFF) as u8,
            (warnings & 0xFF) as u8,
            (warnings >> 8) as u8,
            0xFF, 0xFF,
        ];
        CanFrame::new_j1939(Pgn::BATTERY_ALARMS, self.source_address, &data, 3).unwrap()
    }

    pub fn encode_ota_chunk(&mut self, chunk_index: u32, data: &[u8], total_chunks: u32, crc32: u32) -> CanFrame {
        // Proprietary PGN for OTA: 8-byte header + up to 56 bytes data
        let mut frame_data = [0u8; 64];
        frame_data[0] = (chunk_index & 0xFF) as u8;
        frame_data[1] = ((chunk_index >> 8) & 0xFF) as u8;
        frame_data[2] = ((chunk_index >> 16) & 0xFF) as u8;
        frame_data[3] = ((chunk_index >> 24) & 0xFF) as u8;
        frame_data[4] = (total_chunks & 0xFF) as u8;
        frame_data[5] = ((total_chunks >> 8) & 0xFF) as u8;
        frame_data[6] = (crc32 & 0xFF) as u8;
        frame_data[7] = ((crc32 >> 8) & 0xFF) as u8;
        frame_data[8..8+data.len()].copy_from_slice(data);

        CanFrame {
            id: ((7 << 26) | (Pgn::PROP_OTA.0 & 0x3FFFF) | (self.source_address as u32)),
            dlc: (8 + data.len()) as u8,
            data: Vec::from_slice(&frame_data[..8+data.len()]).unwrap(),
            timestamp_us: embassy_time::Instant::now().as_micros(),
            is_fd: true,
            bitrate_switch: true,
        }
    }
}

/// J1939 Decoder for incoming frames
pub struct J1939Decoder;

impl J1939Decoder {
    pub fn decode_battery_voltage(frame: &CanFrame) -> Option<f32> {
        if frame.pgn() != Pgn::BATTERY_VOLTAGE || frame.data.len() < 2 {
            return None;
        }
        let value = u16::from_le_bytes([frame.data[0], frame.data[1]]);
        Some(value as f32 * 100.0)  // 0.1V -> mV
    }

    pub fn decode_cell_voltages(frame: &CanFrame) -> Option<Vec<f32, 32>> {
        if frame.pgn() != Pgn::CELL_VOLTAGES || frame.data.len() < 2 {
            return None;
        }
        let mut voltages = Vec::new();
        for chunk in frame.data.chunks_exact(2) {
            let val = u16::from_le_bytes([chunk[0], chunk[1]]);
            voltages.push(val as f32 * 0.1).ok()?;  // 0.1 mV -> mV
        }
        Some(voltages)
    }

    pub fn decode_battery_status(frame: &CanFrame) -> Option<(f32, f32, u32, u16, u16)> {
        if frame.pgn() != Pgn::BATTERY_STATUS || frame.data.len() < 8 {
            return None;
        }
        let soc = frame.data[0] as f32 * 0.5;
        let soh = frame.data[1] as f32 * 0.5;
        let cycle_count = u32::from_le_bytes([frame.data[2], frame.data[3], frame.data[4], frame.data[5]]);
        let alarms = u16::from_le_bytes([frame.data[6], frame.data[7]]);
        let warnings = 0;  // Not in this frame
        Some((soc, soh, cycle_count, alarms, warnings))
    }

    pub fn decode_alarms(frame: &CanFrame) -> Option<(u32, u32, u16)> {
        if frame.pgn() != Pgn::BATTERY_ALARMS || frame.data.len() < 6 {
            return None;
        }
        let cell_ov = u16::from_le_bytes([frame.data[0], frame.data[1]]) as u32;
        let cell_uv = u16::from_le_bytes([frame.data[2], frame.data[3]]) as u32;
        let warnings = u16::from_le_bytes([frame.data[4], frame.data[5]]);
        Some((cell_ov, cell_uv, warnings))
    }
}