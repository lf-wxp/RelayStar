//! Transport-port abstraction.
//!
//! Every physical transport (LoRa, ESP-NOW, MQTT, ...) implements
//! [`TransportPort`]. The [`crate::Relay`] engine calls `send_frame` when it
//! wants a specific frame emitted on a specific transport, and callers push
//! received bytes back through [`crate::Relay::ingest`].
//!
//! Uses native `async fn in trait` (Rust 1.75+). Implementations must be
//! `Send + Sync` when used from `Relay` running under a multi-executor.

use crate::error::PortError;
use relaystar_proto::Transport;

/// Frame-level addressing hint passed to a [`TransportPort`].
///
/// This is **not** the same as [`crate::Destination`]:
/// - `Destination` is an *application-level* intent (broadcast / unicast to a
///   node address); the [`crate::Relay`] resolves it against the receiver
///   table.
/// - `FrameAddr` is the *transport-native* addressing for a single frame
///   (e.g. ESP-NOW unicast MAC vs. `FF:FF:FF:FF:FF:FF`; MQTT unicast topic vs.
///   broadcast topic).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameAddr {
  /// Send this frame to a specific peer address, using the transport's own
  /// unicast facility.
  Unicast([u8; 6]),
  /// Send this frame using the transport's broadcast facility.
  Broadcast,
}

impl FrameAddr {
  /// Convert to a raw 6-byte address, mapping `Broadcast` to
  /// [`relaystar_proto::BROADCAST_ADDR`].
  pub const fn as_bytes(self) -> [u8; 6] {
    match self {
      FrameAddr::Unicast(a) => a,
      FrameAddr::Broadcast => relaystar_proto::BROADCAST_ADDR,
    }
  }

  /// Build a [`FrameAddr`] from a raw 6-byte address; `FF:FF:FF:FF:FF:FF` maps
  /// to [`FrameAddr::Broadcast`].
  pub const fn from_bytes(addr: [u8; 6]) -> Self {
    if matches_broadcast(addr) {
      FrameAddr::Broadcast
    } else {
      FrameAddr::Unicast(addr)
    }
  }
}

const fn matches_broadcast(addr: [u8; 6]) -> bool {
  let bc = relaystar_proto::BROADCAST_ADDR;
  addr[0] == bc[0]
    && addr[1] == bc[1]
    && addr[2] == bc[2]
    && addr[3] == bc[3]
    && addr[4] == bc[4]
    && addr[5] == bc[5]
}

/// A single physical transport that can send and identify itself.
///
/// # Contract for implementors
/// - `transport()` must return a *stable* value for the lifetime of the port.
/// - `send_frame` **should not** perform application-level fragmentation; the
///   caller ([`crate::Relay`]) has already sized `frame` to fit
///   [`Transport::max_payload`]-shaped bounds.
/// - Implementations must be cancel-safe: if the future is dropped mid-await,
///   the port must remain usable for the next call.
///
/// [`Transport::max_payload`]: relaystar_proto::Transport::max_payload
pub trait TransportPort {
  /// Which transport this port represents.
  fn transport(&self) -> Transport;

  /// Emit a single already-encoded [`relaystar_proto::Message`] frame.
  ///
  /// # Errors
  /// Returns [`PortError::Io`] on hardware failure, [`PortError::NotReady`] if
  /// the transport is offline, and [`PortError::FrameTooLarge`] if the frame
  /// exceeds a *hardware* limit (see [`PortError::FrameTooLarge`] docs).
  fn send_frame(
    &mut self,
    addr: FrameAddr,
    frame: &[u8],
  ) -> impl core::future::Future<Output = Result<(), PortError>>;
}
