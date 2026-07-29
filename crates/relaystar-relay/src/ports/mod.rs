//! Transport-port adapters.
//!
//! Each submodule provides a ready-to-use [`TransportPort`] implementation
//! parameterised over a *tiny* hardware trait. You bring the radio / socket
//! driver; the adapter translates RelayStar frames into transport-native
//! calls (including LoRa unicast headers, ESP-NOW peer registration, and
//! MQTT topic naming for unicast/broadcast).
//!
//! Common pattern:
//!
//! ```ignore
//! // 1. Implement the hardware trait for your specific driver.
//! impl LoraRadio for MyLoraDriver { /* async tx/rx */ }
//!
//! // 2. Wrap it.
//! let mut port = LoraPort::new(driver);
//!
//! // 3. Hand it to the relay.
//! relay.send(&mut port, MsgKind::Text, b"hi", Destination::Broadcast).await?;
//! ```
//!
//! [`TransportPort`]: crate::TransportPort

pub mod espnow;
pub mod lora;
pub mod mqtt;
