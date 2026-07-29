//! Error types for the RelayStar relay engine.
//!
//! Follows the "many small errors" pattern from `rust-skills` (err-typed-lib):
//! transport-level failures use [`PortError`], while higher-level engine
//! failures use [`RelayError`]. Both are `#[non_exhaustive]` so new variants
//! can be added without a breaking release.

/// Error emitted by a [`crate::TransportPort`] implementation.
///
/// Adapters translate their own hardware errors into these coarse variants.
/// Keeping the surface small lets the [`crate::Relay`] engine reason about
/// them uniformly.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum PortError {
  /// The underlying radio / socket / peripheral rejected the send.
  Io,
  /// The transport is currently offline (no link, not connected, not paired).
  NotReady,
  /// The frame exceeds a limit the hardware refused to accept.
  ///
  /// This is a *hardware* limit; MTU-based application-level fragmentation is
  /// already handled inside [`crate::Relay`], so seeing this variant usually
  /// means the transport's own [`relaystar_proto::Transport::max_payload`]
  /// value is set too high for this specific device.
  FrameTooLarge,
  /// The transport does not support the requested addressing mode
  /// (e.g. unicast to an unregistered ESP-NOW peer).
  UnsupportedAddr,
}

impl core::fmt::Display for PortError {
  fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
    let s = match self {
      PortError::Io => "transport i/o error",
      PortError::NotReady => "transport not ready",
      PortError::FrameTooLarge => "frame too large for transport",
      PortError::UnsupportedAddr => "unsupported addressing mode",
    };
    f.write_str(s)
  }
}

#[cfg(feature = "std")]
impl std::error::Error for PortError {}

/// Errors returned by the [`crate::Relay`] engine.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum RelayError {
  /// A [`crate::Destination::Unicast`] was requested but the recipient is not
  /// in the [`crate::ReceiverTable`]. Register the receiver first, or use
  /// [`crate::Destination::Broadcast`].
  UnknownReceiver,
  /// No [`crate::TransportPort`] has been registered for the target transport.
  NoSuchPort,
  /// No transports are currently registered at all.
  NoPortsRegistered,
  /// Payload cannot be split into at most `u16::MAX` fragments given the
  /// target transport's MTU. In practice this only fires for pathologically
  /// large payloads or a misconfigured MTU of 0.
  PayloadUnfragmentable,
  /// A [`relaystar_proto::ProtoError`] happened while (de)serialising.
  Protocol(relaystar_proto::ProtoError),
  /// The underlying transport port returned an error.
  Port(PortError),
  /// The `ReceiverTable` is full and cannot admit a new peer.
  ReceiverTableFull,
  /// The `Reassembler` dropped the message (timeout / overflow / mismatched
  /// header). Included so callers can log or count these.
  ReassemblyFailed,
}

impl core::fmt::Display for RelayError {
  fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
    match self {
      RelayError::UnknownReceiver => f.write_str("unknown unicast receiver"),
      RelayError::NoSuchPort => f.write_str("no port registered for target transport"),
      RelayError::NoPortsRegistered => f.write_str("no transport ports registered"),
      RelayError::PayloadUnfragmentable => f.write_str("payload cannot be fragmented"),
      RelayError::Protocol(e) => write!(f, "protocol error: {}", e),
      RelayError::Port(e) => write!(f, "port error: {}", e),
      RelayError::ReceiverTableFull => f.write_str("receiver table full"),
      RelayError::ReassemblyFailed => f.write_str("fragment reassembly failed"),
    }
  }
}

impl From<relaystar_proto::ProtoError> for RelayError {
  fn from(e: relaystar_proto::ProtoError) -> Self {
    RelayError::Protocol(e)
  }
}

impl From<PortError> for RelayError {
  fn from(e: PortError) -> Self {
    RelayError::Port(e)
  }
}

#[cfg(feature = "std")]
impl std::error::Error for RelayError {}
