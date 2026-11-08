# BatterySense

Industrial-grade battery voltage acquisition system — open, safe, auditable.

A complete Battery Management System (BMS) firmware stack for STM32G474RE with
ADS1262 32-bit ΔΣ ADC, CAN FD (J1939), dual-bank OTA, hardware watchdogs, and
plausibility checking. Includes host simulator, MQTT gateway, and Tauri GUI.

## Repository Structure

```
batterysense/
├── Cargo.toml              # Rust workspace root
├── firmware/               # Embedded firmware (STM32G474RE, Embassy RTOS)
│   ├── Cargo.toml
│   ├── memory.x            # Dual-bank flash linker script
│   ├── .cargo/config.toml  # probe-rs target config
│   └── src/
│       ├── main.rs         # Task supervisor (Embassy async)
│       ├── adc/            # ADS1262 driver + calibration + temp compensation
│       ├── can/            # CAN FD + J1939 encoder/decoder
│       ├── ota/            # Dual-bank A/B bootloader + signed manifest
│       ├── safety/         # Watchdog (IWDG/WWDG/external) + plausibility
│       └── comm/           # USB CDC + BLE + CLI parser
├── simulator/              # Host simulator (runs on PC, no hardware needed)
│   ├── Cargo.toml
│   └── src/main.rs
├── gateway/                # CAN-to-MQTT/HTTP cloud gateway
│   ├── Cargo.toml
│   └── src/main.rs
├── gui/                    # Tauri + Leptos desktop dashboard
│   ├── Cargo.toml
│   └── src/main.rs
├── .github/workflows/ci.yml # CI: build, test, clippy, fmt
├── README.md
├── LICENSE                 # Apache-2.0 OR MIT
└── .gitignore
```

## Features

| Module | Capability |
|--------|------------|
| **ADC (ADS1262)** | 16-channel, 32-bit delta-sigma, 1 kHz sampling, SPI+DMA, polynomial calibration per channel, temperature compensation |
| **CAN FD** | 2 Mbps nominal / 5 Mbps data, J1939 PGN/SPN encoding for battery voltage, current, temperature, cell voltages, alarms, OTA |
| **Safety** | Independent watchdog (IWDG), window watchdog (WWDG), external supervisor, over/undervoltage, dV/dt, stuck-at, open-wire, cell imbalance detection |
| **OTA** | Dual-bank A/B firmware update, ECDSA P-256 signature verification, CRC-32C, rollback on boot failure, manifest-based |
| **Simulator** | Host PC simulator — run full firmware logic without STM32 hardware, inject faults, verify safety checks |
| **Gateway** | MQTT/HTTP bridge — forward CAN FD battery data to cloud platforms |
| **GUI** | Tauri desktop dashboard for real-time BMS monitoring |

## Quick Start

### Prerequisites

- Rust toolchain (stable)
- `thumbv7em-none-eabihf` target for embedded build
- `probe-rs` for flashing to STM32G474RE

```bash
# Add embedded target
rustup target add thumbv7em-none-eabihf

# Install probe-rs
cargo install probe-rs-cli --features cli
```

### Build Firmware (Embedded)

```bash
cd batterysense/firmware
cargo build --release --features defmt-rtt
```

### Flash to STM32G474RE

```bash
cargo flash --chip STM32G474RE
```

### Run Host Simulator (No Hardware Needed)

```bash
cargo run -p batterysense-simulator

# Run with fault injection (simulate overvoltage on cell 3)
cargo run -p batterysense-simulator -- --fault-channel 3 --fault-type overvoltage --duration 10

# Run for 30 seconds then exit
cargo run -p batterysense-simulator -- --duration 30
```

### Run Unit Tests

```bash
# Firmware tests (host, no hardware)
cargo test -p batterysense-firmware

# All workspace tests
cargo test --workspace
```

### Clippy & Formatting

```bash
cargo clippy --workspace --all-targets --all-features -- -D warnings
cargo fmt --all -- --check
```

## CI

GitHub Actions runs on every push and PR:
- `cargo fmt --check`
- `cargo clippy --all-targets --all-features`
- `cargo test --workspace`
- `cargo build --release` (embedded target)

## License

Apache-2.0 OR MIT. See [LICENSE](LICENSE).

## Portofolio Notes

This project demonstrates:
- Embedded Rust with Embassy async runtime
- No-std development with `deny(unsafe_code)`
- Safety-critical design (watchdogs, plausibility, dual-bank OTA)
- Host simulator for CI/CD without hardware
- Multi-crate workspace (firmware, simulator, gateway, gui)
- Professional documentation and CI pipeline
