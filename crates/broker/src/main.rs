//! RelayStar MQTT broker.
//!
//! Thin wrapper around [`rumqttd`] that loads a TOML config file and starts the
//! broker. The config path can be overridden with `RELAYSTAR_BROKER_CONFIG`
//! (defaults to `rumqttd.toml` in the working directory, which is what the
//! Docker image ships).
//!
//! ## Topics
//!
//! The broker itself is transport-agnostic and does not decode
//! [`relaystar_proto::Message`]s. Firmware nodes speak two topic families over
//! this broker:
//!
//! - **Legacy** (used by `cardputer-fw`'s bespoke MQTT client):
//!   [`LEGACY_UPLINK`] and [`LEGACY_DOWNLINK`].
//! - **New relay-native** (used by anything built on
//!   [`relaystar_relay::ports::mqtt::MqttPort`]):
//!   [`relaystar_relay::ports::mqtt::Topic::BROADCAST`] and per-node
//!   `relaystar/u/{hex-addr}` topics.
//!
//! If you deploy a mesh-aware gateway on the host side, subscribe to
//! `relaystar/#` and use [`relaystar_relay::Relay`] to decode, dedup, and
//! reassemble arriving frames.

use anyhow::Context;
use relaystar_relay::ports::mqtt::Topic as RelayMqttTopic;
use rumqttd::{Broker, Config};

/// Legacy topic (published to by `cardputer-fw/src/mqtt.rs`) that carries
/// mesh downlink text.
const LEGACY_DOWNLINK: &str = "relaystar/downlink";
/// Legacy topic that a client can publish to in order to inject a text
/// message into the mesh.
const LEGACY_UPLINK: &str = "relaystar/uplink";

fn main() -> anyhow::Result<()> {
  tracing_subscriber::fmt()
    .with_env_filter(
      tracing_subscriber::EnvFilter::try_from_default_env()
        .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
    )
    .with_line_number(false)
    .with_file(false)
    .init();

  let config_path =
    std::env::var("RELAYSTAR_BROKER_CONFIG").unwrap_or_else(|_| "rumqttd.toml".to_string());

  tracing::info!(config = %config_path, "loading RelayStar broker config");

  let config = config::Config::builder()
    .add_source(config::File::with_name(&config_path))
    .build()
    .with_context(|| format!("failed to load config file: {config_path}"))?;

  let config: Config = config
    .try_deserialize()
    .context("failed to deserialize rumqttd config")?;

  let mut broker = Broker::new(config);

  tracing::info!(
    max_payload_bytes = relaystar_proto::MAX_PAYLOAD,
    legacy_uplink = %LEGACY_UPLINK,
    legacy_downlink = %LEGACY_DOWNLINK,
    relay_broadcast = %RelayMqttTopic::BROADCAST,
    relay_unicast_prefix = %RelayMqttTopic::UNICAST_PREFIX,
    "RelayStar broker starting",
  );

  // Blocking: returns only when every configured server stops.
  broker.start().context("broker terminated with error")?;

  Ok(())
}
