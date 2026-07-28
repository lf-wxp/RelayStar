//! RelayStar MQTT broker.
//!
//! Thin wrapper around [`rumqttd`] that loads a TOML config file and starts the
//! broker. The config path can be overridden with `RELAYSTAR_BROKER_CONFIG`
//! (defaults to `rumqttd.toml` in the working directory, which is what the
//! Docker image ships).

use anyhow::Context;
use rumqttd::{Broker, Config};

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
    "RelayStar broker starting; shared protocol wire version carries {} byte max payload",
    relaystar_proto::MAX_PAYLOAD
  );

  // Blocking: returns only when every configured server stops.
  broker.start().context("broker terminated with error")?;

  Ok(())
}
