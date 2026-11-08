//! BatterySense Host Simulator
//!
//! Runs the BatterySense firmware logic on a host PC without STM32 hardware.
//! Useful for demonstrating functionality, CI testing, and debugging logic
//! before flashing to real hardware.
//!
//! Usage:
//!   cargo run -p batterysense-simulator
//!
//! The simulator reads a simulated battery pack (16 cells), runs ADC sampling,
//! plausibility checks, CAN FD/J1939 encoding, and OTA state machine — all on
//! the host using the same library code as the embedded firmware.

use batterysense_firmware::{
    can::J1939Encoder,
    safety::PlausibilityChecker,
};
use clap::Parser;
use rand::Rng;
use std::time::Duration;
use tokio::time::sleep;

/// CLI arguments for the simulator
#[derive(Parser, Debug)]
#[command(author, version, about = "BatterySense Host Simulator")]
struct Args {
    /// Number of simulated cells (default: 16)
    #[arg(short, long, default_value_t = 16)]
    cells: u8,

    /// Sample rate in Hz (default: 1000)
    #[arg(short, long, default_value_t = 1000)]
    rate: u32,

    /// Run for N seconds then exit (0 = run forever)
    #[arg(short, long, default_value_t = 0)]
    duration: u64,

    /// Inject a fault on a specific channel (0-15, or 255 for none)
    #[arg(long, default_value_t = 255)]
    fault_channel: u8,

    /// Fault type: overvoltage, undervoltage, openwire, stuck
    #[arg(long, default_value = "none")]
    fault_type: String,
}

/// Simulated ADC driver for host testing
struct SimulatedAdc {
    channel_count: u8,
    base_voltages: [f32; 16],
    rng: rand::rngs::ThreadRng,
    fault_channel: u8,
    fault_type: String,
    sample_count: u64,
}

impl SimulatedAdc {
    fn new(channel_count: u8, fault_channel: u8, fault_type: String) -> Self {
        let mut base_voltages = [3700.0f32; 16]; // Nominal 3.7V per cell
        // Add slight variation between cells
        for (i, v) in base_voltages.iter_mut().enumerate().take(16) {
            *v = 3700.0 + (i as f32 * 5.0) - 40.0; // 3.66V to 3.74V
        }

        Self {
            channel_count,
            base_voltages,
            rng: rand::thread_rng(),
            fault_channel,
            fault_type,
            sample_count: 0,
        }
    }

    fn read_all_channels(&mut self) -> batterysense_firmware::adc::AdcSample {
        self.sample_count += 1;
        let mut pack_voltage = 0.0f32;
        let mut last_sample = batterysense_firmware::adc::AdcSample {
            channel: 0,
            raw: 0,
            voltage_mv: 0.0,
            temperature_c: 25.0,
            timestamp_us: (self.sample_count * 1000) as u64,
            pack_voltage_mv: 0.0,
        };

        for ch in 0..self.channel_count {
            let mut voltage = self.base_voltages[ch as usize];

            // Add noise (±5 mV)
            voltage += self.rng.gen_range(-5.0..5.0);

            // Inject fault if configured
            if ch == self.fault_channel {
                match self.fault_type.as_str() {
                    "overvoltage" => voltage = 4200.0, // Above 3.65V threshold
                    "undervoltage" => voltage = 2000.0, // Below 2.5V threshold
                    "openwire" => voltage = 50.0,      // Below 100 mV threshold
                    "stuck" => voltage = self.base_voltages[ch as usize], // No change
                    _ => {}
                }
            }

            pack_voltage += voltage;
            if ch == self.channel_count - 1 {
                last_sample = batterysense_firmware::adc::AdcSample {
                    channel: ch,
                    raw: (voltage * 10.0) as i32,
                    voltage_mv: voltage,
                    temperature_c: 25.0 + (self.sample_count as f32 * 0.01),
                    timestamp_us: (self.sample_count * 1000) as u64,
                    pack_voltage_mv: 0.0,
                };
            }
        }

        batterysense_firmware::adc::AdcSample {
            pack_voltage_mv: pack_voltage,
            ..last_sample
        }
    }
}

#[tokio::main]
async fn main() {
    tracing_subscriber::fmt::init();

    let args = Args::parse();

    println!("=== BatterySense Host Simulator ===");
    println!("Cells: {}", args.cells);
    println!("Sample rate: {} Hz", args.rate);
    if args.fault_channel < 16 {
        println!("Fault injected: channel {} -> {}", args.fault_channel, args.fault_type);
    }
    println!("====================================\n");

    let mut adc = SimulatedAdc::new(args.cells, args.fault_channel, args.fault_type);
    let mut encoder = J1939Encoder::new(0x20); // Source address 0x20
    let mut checker = PlausibilityChecker::new();

    let interval = Duration::from_millis(1000 / args.rate as u64);
    let mut start = std::time::Instant::now();

    loop {
        // Read simulated ADC
        let sample = adc.read_all_channels();

        // Run plausibility check
        if let Err(event) = checker.check(&sample) {
            tracing::warn!("SAFETY EVENT: {}", event);
            println!("[SAFETY] {}", event);
        }

        // Encode and log CAN FD J1939 frame
        let frame = encoder.encode_battery_voltage(sample.pack_voltage_mv);
        tracing::debug!(
            "CAN TX: PGN=0x{:05X} data={:?}",
            frame.pgn().0,
            frame.data.as_slice()
        );

        // Print pack voltage every 100 samples
        if adc.sample_count % 100 == 0 {
            println!(
                "[{:>6} samples] Pack voltage: {:.2} mV | Temp: {:.1}°C | Imbalance: {:.2} mV",
                adc.sample_count,
                sample.pack_voltage_mv,
                sample.temperature_c,
                checker.get_imbalance_mv()
            );
        }

        // Check duration limit
        if args.duration > 0 && start.elapsed() >= Duration::from_secs(args.duration) {
            println!("\n=== Simulator finished after {} seconds ===", args.duration);
            break;
        }

        sleep(interval).await;
    }
}
