//! CLI Parser — Simple command-line interface over USB CDC / UART
//!
//! Supported commands:
//!   - `help`        : List available commands
//!   - `version`     : Show firmware version
//!   - `voltage`     : Show latest pack voltage
//!   - `status`      : Show system status (banks, watchdog, safety)
//!   - `calib`       : Enter calibration mode
//!   - `ota`         : Trigger OTA update check

use core::fmt::Write;
use heapless::String;
use heapless::Vec;
use thiserror::Error;

#[derive(Debug, Error, defmt::Format)]
pub enum CliError {
    #[error("Command not found")]
    NotFound,
    #[error("Invalid arguments")]
    InvalidArgs,
    #[error("Buffer overflow")]
    BufferOverflow,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CliCommand {
    Help,
    Version,
    Voltage,
    Status,
    Calib,
    Ota,
    Unknown,
}

pub struct CliParser {
    buffer: Vec<u8, 128>,
}

impl Default for CliParser {
    fn default() -> Self {
        Self::new()
    }
}

impl CliParser {
    pub fn new() -> Self {
        Self {
            buffer: Vec::new(),
        }
    }

    /// Feed a byte into the parser. Returns `Some(command)` when a full line is received.
    pub fn feed(&mut self, byte: u8) -> Option<CliCommand> {
        if byte == b'\n' || byte == b'\r' {
            let line = core::str::from_utf8(self.buffer.as_slice()).unwrap_or("");
            self.buffer.clear();
            if !line.is_empty() {
                return Some(self.parse(line));
            }
            None
        } else if byte >= 0x20 {
            self.buffer.push(byte).ok();
            None
        } else {
            None
        }
    }

    fn parse(&self, line: &str) -> CliCommand {
        match line.trim().to_lowercase().as_str() {
            "help" => CliCommand::Help,
            "version" | "ver" => CliCommand::Version,
            "voltage" | "v" => CliCommand::Voltage,
            "status" | "s" => CliCommand::Status,
            "calib" | "cal" => CliCommand::Calib,
            "ota" => CliCommand::Ota,
            _ => CliCommand::Unknown,
        }
    }

    pub fn format_help() -> &'static str {
        "BatterySense CLI Commands:\n\
         help      - Show this help\n\
         version   - Show firmware version\n\
         voltage   - Show pack voltage (mV)\n\
         status    - Show system status\n\
         calib     - Calibration mode\n\
         ota       - Check for OTA update\n"
    }

    pub fn format_version() -> String<64> {
        let mut s = String::new();
        let _ = core::write!(s, "BatterySense Firmware v{}", env!("CARGO_PKG_VERSION"));
        s
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_help() {
        let parser = CliParser::new();
        assert_eq!(parser.parse("help"), CliCommand::Help);
        assert_eq!(parser.parse("HELP"), CliCommand::Help);
    }

    #[test]
    fn test_parse_version() {
        let parser = CliParser::new();
        assert_eq!(parser.parse("version"), CliCommand::Version);
        assert_eq!(parser.parse("ver"), CliCommand::Version);
    }

    #[test]
    fn test_parse_voltage() {
        let parser = CliParser::new();
        assert_eq!(parser.parse("voltage"), CliCommand::Voltage);
        assert_eq!(parser.parse("v"), CliCommand::Voltage);
    }

    #[test]
    fn test_parse_unknown() {
        let parser = CliParser::new();
        assert_eq!(parser.parse("foobar"), CliCommand::Unknown);
    }

    #[test]
    fn test_feed_newline() {
        let mut parser = CliParser::new();
        for b in b"help\n" {
            let result = parser.feed(*b);
            assert!(result.is_none());
        }
        // The last \n should trigger parsing
        let mut parser2 = CliParser::new();
        for b in b"h" {
            parser2.feed(*b);
        }
        for b in b"elp\n" {
            let result = parser2.feed(*b);
            assert!(matches!(result, Some(CliCommand::Help)));
        }
    }
}
