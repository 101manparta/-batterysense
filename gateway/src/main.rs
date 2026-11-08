//! BatterySense Gateway
//!
//! Bridges CAN FD battery data from the STM32 firmware to cloud platforms
//! via MQTT and HTTP. Reads from SocketCAN or serial, serializes as JSON,
//! and publishes to an MQTT broker.
//!
//! Usage:
//!   cargo run -p batterysense-gateway -- --interface can0 --broker mqtt://localhost:1883

use clap::Parser;
use tracing_subscriber::{fmt, EnvFilter};

/// CLI arguments for the gateway
#[derive(Parser, Debug)]
#[command(author, version, about = "BatterySense Gateway — CAN to MQTT/HTTP bridge")]
struct Args {
    /// CAN interface (e.g., can0) or serial device (e.g., /dev/ttyACM0)
    #[arg(short, long, default_value = "can0")]
    interface: String,

    /// MQTT broker URL (e.g., mqtt://localhost:1883)
    #[arg(long, default_value = "mqtt://localhost:1883")]
    broker: String,

    /// MQTT topic prefix for battery data
    #[arg(long, default_value = "batterysense")]
    topic_prefix: String,

    /// Enable HTTP metrics endpoint on port 9090
    #[arg(long)]
    metrics: bool,
}

/// A battery reading received from CAN FD
#[derive(Debug, serde::Serialize)]
struct BatteryReading {
    timestamp: u64,
    pack_voltage_mv: f32,
    cells_mv: Vec<f32>,
    temperature_c: f32,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();

    tracing_subscriber::fmt()
        .with_env_filter(EnvFilter::from_default_env())
        .init();

    tracing::info!("Starting BatterySense Gateway");
    tracing::info!("Interface: {}", args.interface);
    tracing::info!("MQTT broker: {}", args.broker);
    tracing::info!("Topic prefix: {}", args.topic_prefix);

    // Connect to MQTT broker
    let (mut mqtt_client, mut connection) = rumqttc::AsyncClient::new(
        args.broker.parse()?,
        10,
    );

    // Spawn connection task
    tokio::spawn(async move {
        if let Err(e) = connection.await {
            tracing::error!("MQTT connection error: {}", e);
        }
    });

    // Subscribe to command topic
    let cmd_topic = format!("{}/cmd", args.topic_prefix);
    mqtt_client.subscribe(&cmd_topic, rumqttc::QoS::AtLeastOnce).await?;

    tracing::info!("Gateway running. Press Ctrl+C to stop.");

    // Main loop: read CAN frames, publish to MQTT
    loop {
        // In production: read from SocketCAN or serial port
        // For now: publish a heartbeat every 5 seconds
        let reading = BatteryReading {
            timestamp: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_millis() as u64,
            pack_voltage_mv: 0.0,
            cells_mv: vec![],
            temperature_c: 0.0,
        };

        let payload = serde_json::to_string(&reading)?;
        let topic = format!("{}/data", args.topic_prefix);

        mqtt_client.publish(&topic, rumqttc::QoS::AtLeastOnce, false, payload).await?;

        tracing::debug!("Published to {}: {}", topic, payload);

        tokio::time::sleep(tokio::time::Duration::from_secs(5)).await;
    }
}
