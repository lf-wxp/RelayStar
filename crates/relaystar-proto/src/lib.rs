#![cfg_attr(not(feature = "std"), no_std)]

//! RelayStar shared wire protocol.
//!
//! This crate is `no_std` by default so it can be shared between the ESP32
//! firmwares and the host-side broker. It defines the [`Message`] type that
//! travels across every transport (LoRa, MQTT, ESP-NOW), a compact
//! [`postcard`]-based framing, and the loop-prevention primitives used by the
//! Cardputer bridge.

use heapless::Vec;
use serde::{Deserialize, Serialize};

/// Maximum application payload carried in a single [`Message`].
///
/// Sized to comfortably fit inside a LoRa frame (SX126x max ~255 bytes) after
/// postcard header/framing overhead.
pub const MAX_PAYLOAD: usize = 200;

/// Upper bound for an encoded frame. Encoding into a buffer of this size will
/// never fail for a well-formed [`Message`].
pub const MAX_FRAME: usize = 255;

/// Default hop budget assigned to freshly-created messages.
pub const DEFAULT_TTL: u8 = 4;

/// Special address value that means "broadcast to every reachable peer".
///
/// Used both as [`Message::to`] and as the physical destination in transport
/// adapters (e.g. ESP-NOW's `FF:FF:FF:FF:FF:FF` MAC).
pub const BROADCAST_ADDR: [u8; 6] = [0xFF; 6];

/// Radio parameters shared by every LoRa node so they can hear each other.
///
/// The concrete `lora-phy` enum values (spreading factor, bandwidth, coding
/// rate) are derived from these in each firmware; keep them in sync across the
/// mesh. Defaults target the EU868 band.
pub mod radio {
  /// LoRa centre frequency in Hz (EU868). Change for your region (e.g.
  /// 915_000_000 for US915).
  pub const LORA_FREQUENCY_HZ: u32 = 868_000_000;
  /// Spreading factor (maps to `lora_phy::mod_params::SpreadingFactor::_10`).
  pub const LORA_SPREADING_FACTOR: u8 = 10;
  /// Bandwidth in kHz (maps to `Bandwidth::_250KHz`).
  pub const LORA_BANDWIDTH_KHZ: u16 = 250;
  /// Coding rate denominator, i.e. 4/8 (maps to `CodingRate::_4_8`).
  pub const LORA_CODING_RATE_DENOM: u8 = 8;
  /// Preamble length in symbols.
  pub const LORA_PREAMBLE_LEN: u16 = 4;
  /// TX output power in dBm.
  pub const LORA_TX_POWER_DBM: i32 = 20;
}

/// The physical transport a message was seen on / should be sent over.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum Transport {
  Lora,
  Mqtt,
  EspNow,
}

impl Transport {
  /// All transports, useful for fan-out iteration.
  pub const ALL: [Transport; 3] = [Transport::Lora, Transport::Mqtt, Transport::EspNow];

  pub const fn as_str(self) -> &'static str {
    match self {
      Transport::Lora => "lora",
      Transport::Mqtt => "mqtt",
      Transport::EspNow => "espnow",
    }
  }

  /// Maximum **application payload** (post-fragmentation) that fits into a
  /// single frame of this transport.
  ///
  /// These values are intentionally conservative and account for the postcard
  /// header + [`Message`] envelope overhead. LoRa is the tightest bound;
  /// ESP-NOW v1 caps at 250 bytes; MQTT is only bounded by broker limits.
  pub const fn max_payload(self) -> usize {
    match self {
      // LoRa SX126x max ~255 bytes; leave headroom for postcard envelope.
      Transport::Lora => 180,
      // ESP-NOW v1 payload cap is 250 bytes; leave headroom.
      Transport::EspNow => 200,
      // MQTT is effectively unbounded for our use; keep a sane cap that fits
      // a Cardputer stack buffer.
      Transport::Mqtt => 4096,
    }
  }
}

/// Semantic type of a message payload.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[non_exhaustive]
pub enum MsgKind {
  /// UTF-8 chat/text payload.
  Text,
  /// Liveness probe.
  Ping,
  /// Reply to a [`MsgKind::Ping`].
  Pong,
  /// Opaque binary telemetry.
  Telemetry,
}

/// Fragmentation header attached to messages that were split to fit a
/// transport's MTU.
///
/// A `Message` with `frag = None` is a **single, complete** payload. A
/// `Message` with `frag = Some(_)` is one slice of a larger logical payload
/// identified by `group_id`; the receiver reassembles by collecting all
/// `total` slices ordered by `seq`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct Fragment {
  /// Identifier shared by every slice of the same logical payload.
  pub group_id: u32,
  /// Zero-based index of this slice, in `[0, total)`.
  pub seq: u16,
  /// Total number of slices that make up the logical payload.
  pub total: u16,
}

/// A transport-agnostic RelayStar message.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Message {
  /// Globally-unique-ish id used for de-duplication across the mesh.
  pub id: u32,
  /// Remaining hop budget. Decremented on each relay; dropped at 0.
  pub ttl: u8,
  /// Transport the message originally entered the mesh on.
  pub origin: Transport,
  /// Source node address (MAC for ESP-NOW/Wi-Fi, synthetic id otherwise).
  pub from: [u8; 6],
  /// Destination address ([`BROADCAST_ADDR`] means broadcast).
  pub to: [u8; 6],
  /// Payload interpretation hint.
  pub kind: MsgKind,
  /// Application payload bytes (for this **fragment**; see [`Fragment`]).
  pub payload: Vec<u8, MAX_PAYLOAD>,
  /// Fragmentation header (`None` for single-frame messages).
  pub frag: Option<Fragment>,
}

/// Errors produced while (de)serializing a [`Message`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum ProtoError {
  /// The payload exceeded [`MAX_PAYLOAD`].
  PayloadTooLarge,
  /// The provided buffer was too small to hold the encoded frame.
  BufferTooSmall,
  /// The bytes could not be decoded into a [`Message`].
  Decode,
}

impl core::fmt::Display for ProtoError {
  fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
    let s = match self {
      ProtoError::PayloadTooLarge => "payload too large",
      ProtoError::BufferTooSmall => "buffer too small",
      ProtoError::Decode => "decode error",
    };
    f.write_str(s)
  }
}

#[cfg(feature = "std")]
impl std::error::Error for ProtoError {}

impl Message {
  /// Build a new **broadcast** message with [`DEFAULT_TTL`] from a byte
  /// payload.
  ///
  /// Prefer [`Message::unicast`] when you already know the recipient.
  ///
  /// # Errors
  /// Returns [`ProtoError::PayloadTooLarge`] if `payload.len() > MAX_PAYLOAD`.
  pub fn new(
    id: u32,
    origin: Transport,
    from: [u8; 6],
    kind: MsgKind,
    payload: &[u8],
  ) -> Result<Self, ProtoError> {
    Self::unicast(id, origin, from, BROADCAST_ADDR, kind, payload)
  }

  /// Build a unicast message targeted at `to`.
  ///
  /// # Errors
  /// Returns [`ProtoError::PayloadTooLarge`] if `payload.len() > MAX_PAYLOAD`.
  pub fn unicast(
    id: u32,
    origin: Transport,
    from: [u8; 6],
    to: [u8; 6],
    kind: MsgKind,
    payload: &[u8],
  ) -> Result<Self, ProtoError> {
    let payload = Vec::from_slice(payload).map_err(|_| ProtoError::PayloadTooLarge)?;
    Ok(Message {
      id,
      ttl: DEFAULT_TTL,
      origin,
      from,
      to,
      kind,
      payload,
      frag: None,
    })
  }

  /// Convenience constructor for a broadcast UTF-8 text message.
  ///
  /// # Errors
  /// Returns [`ProtoError::PayloadTooLarge`] if the text exceeds [`MAX_PAYLOAD`].
  pub fn text(id: u32, origin: Transport, from: [u8; 6], text: &str) -> Result<Self, ProtoError> {
    Self::new(id, origin, from, MsgKind::Text, text.as_bytes())
  }

  /// Returns `true` if [`Self::to`] equals [`BROADCAST_ADDR`].
  pub fn is_broadcast(&self) -> bool {
    self.to == BROADCAST_ADDR
  }

  /// Returns `true` if this message is a fragment of a larger payload.
  pub fn is_fragmented(&self) -> bool {
    self.frag.is_some()
  }

  /// Interpret the payload as UTF-8 text, if valid.
  pub fn as_text(&self) -> Option<&str> {
    core::str::from_utf8(&self.payload).ok()
  }

  /// Encode into `buf`, returning the populated slice.
  ///
  /// # Errors
  /// Returns [`ProtoError::BufferTooSmall`] if `buf` cannot hold the encoded
  /// frame; use [`MAX_FRAME`]-sized buffers to guarantee success.
  pub fn encode<'a>(&self, buf: &'a mut [u8]) -> Result<&'a mut [u8], ProtoError> {
    postcard::to_slice(self, buf).map_err(|_| ProtoError::BufferTooSmall)
  }

  /// Decode a message from its wire bytes.
  ///
  /// # Errors
  /// Returns [`ProtoError::Decode`] if the bytes are not a valid postcard
  /// encoding of a [`Message`].
  pub fn decode(bytes: &[u8]) -> Result<Self, ProtoError> {
    postcard::from_bytes(bytes).map_err(|_| ProtoError::Decode)
  }

  /// Produce the version of this message to forward onto another transport,
  /// decrementing the hop budget. Returns `None` once the TTL is exhausted.
  ///
  /// This is a *low-level* helper that only touches TTL. For MTU-aware
  /// fragmentation and multi-transport fan-out, use `relaystar-relay`.
  pub fn prepared_for_relay(&self) -> Option<Message> {
    if self.ttl == 0 {
      return None;
    }
    let mut relayed = self.clone();
    relayed.ttl -= 1;
    Some(relayed)
  }
}

/// Fixed-capacity ring cache of recently-seen message ids for loop prevention.
///
/// `N` must be a power of two is *not* required; this is a simple overwriting
/// ring. It is intentionally allocation-free for use in firmware.
pub struct SeenCache<const N: usize> {
  ids: [u32; N],
  /// `true` for slots that hold a valid id.
  valid: [bool; N],
  next: usize,
}

impl<const N: usize> Default for SeenCache<N> {
  fn default() -> Self {
    Self::new()
  }
}

impl<const N: usize> SeenCache<N> {
  pub const fn new() -> Self {
    SeenCache {
      ids: [0; N],
      valid: [false; N],
      next: 0,
    }
  }

  /// Returns `true` if `id` is already present in the cache.
  pub fn contains(&self, id: u32) -> bool {
    for i in 0..N {
      if self.valid[i] && self.ids[i] == id {
        return true;
      }
    }
    false
  }

  /// Record `id`, evicting the oldest entry if the cache is full.
  pub fn insert(&mut self, id: u32) {
    self.ids[self.next] = id;
    self.valid[self.next] = true;
    self.next = (self.next + 1) % N;
  }

  /// Combined check + record. Returns `true` if the id is new (not seen
  /// before), `false` if it was a duplicate. New ids are recorded.
  pub fn check_and_insert(&mut self, id: u32) -> bool {
    if self.contains(id) {
      false
    } else {
      self.insert(id);
      true
    }
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn round_trip_text() {
    let msg = Message::text(42, Transport::Lora, [1, 2, 3, 4, 5, 6], "hello relay").unwrap();
    let mut buf = [0u8; MAX_FRAME];
    let encoded = msg.encode(&mut buf).unwrap();
    let decoded = Message::decode(encoded).unwrap();
    assert_eq!(decoded, msg);
    assert_eq!(decoded.as_text(), Some("hello relay"));
    assert!(decoded.is_broadcast());
    assert!(!decoded.is_fragmented());
  }

  #[test]
  fn round_trip_unicast_fragment() {
    let mut msg = Message::unicast(
      7,
      Transport::EspNow,
      [1; 6],
      [2; 6],
      MsgKind::Telemetry,
      &[0xAA; 32],
    )
    .unwrap();
    msg.frag = Some(Fragment {
      group_id: 999,
      seq: 1,
      total: 3,
    });
    let mut buf = [0u8; MAX_FRAME];
    let encoded = msg.encode(&mut buf).unwrap();
    let decoded = Message::decode(encoded).unwrap();
    assert_eq!(decoded, msg);
    assert!(!decoded.is_broadcast());
    assert!(decoded.is_fragmented());
    assert_eq!(decoded.to, [2; 6]);
  }

  #[test]
  fn ttl_relay() {
    let msg = Message::text(1, Transport::Mqtt, [0; 6], "hi").unwrap();
    let relayed = msg.prepared_for_relay().unwrap();
    assert_eq!(relayed.ttl, DEFAULT_TTL - 1);

    let mut m = msg.clone();
    m.ttl = 0;
    assert!(m.prepared_for_relay().is_none());
  }

  #[test]
  fn dedup_cache() {
    let mut cache: SeenCache<4> = SeenCache::new();
    assert!(cache.check_and_insert(10));
    assert!(!cache.check_and_insert(10));
    assert!(cache.check_and_insert(11));
    // Overflow eviction: fill past capacity.
    cache.insert(12);
    cache.insert(13);
    cache.insert(14); // evicts id 10
    assert!(cache.check_and_insert(10)); // 10 was evicted -> treated as new
  }

  #[test]
  fn payload_too_large() {
    let big = [0u8; MAX_PAYLOAD + 1];
    assert_eq!(
      Message::new(1, Transport::Lora, [0; 6], MsgKind::Telemetry, &big),
      Err(ProtoError::PayloadTooLarge)
    );
  }

  #[test]
  fn transport_max_payload_ordering() {
    // LoRa is the tightest, MQTT the loosest.
    assert!(Transport::Lora.max_payload() < Transport::EspNow.max_payload());
    assert!(Transport::EspNow.max_payload() < Transport::Mqtt.max_payload());
  }
}
