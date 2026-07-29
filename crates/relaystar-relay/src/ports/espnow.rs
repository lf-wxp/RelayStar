//! ESP-NOW transport adapter.
//!
//! ESP-NOW is a link-layer unicast/broadcast protocol running on the WiFi
//! radio of ESP32 chips. Unlike LoRa, ESP-NOW distinguishes unicast and
//! broadcast at the MAC layer:
//!
//! - **Unicast**: `esp_now_send(peer_mac, ...)`; the peer must be pre-registered.
//! - **Broadcast**: `esp_now_send(FF:FF:FF:FF:FF:FF, ...)`; the broadcast MAC
//!   is a pseudo-peer that must also be registered exactly once.
//!
//! ## What you provide
//!
//! Implement [`EspNowRadio`] over your ESP-NOW driver. The adapter takes care
//! of translating a [`FrameAddr`] into a raw MAC (using
//! [`relaystar_proto::BROADCAST_ADDR`] for broadcast) and calling
//! [`EspNowRadio::send_to`].
//!
//! The adapter also exposes [`EspNowPort::ensure_peer`] so higher layers can
//! auto-register peers as they are learned by the receiver table.

use crate::error::PortError;
use crate::port::{FrameAddr, TransportPort};
use relaystar_proto::{BROADCAST_ADDR, Transport};

/// Hardware-facing trait an ESP-NOW driver must implement.
pub trait EspNowRadio {
  /// Driver-specific error. Mapped to [`PortError::Io`] by default; the
  /// adapter can distinguish `UnsupportedAddr` via [`EspNowRadio::add_peer`]
  /// failing.
  type Error: core::fmt::Debug;

  /// Transmit `frame` to `mac`. For a broadcast send, `mac` equals
  /// [`BROADCAST_ADDR`].
  fn send_to(
    &mut self,
    mac: [u8; 6],
    frame: &[u8],
  ) -> impl core::future::Future<Output = Result<(), Self::Error>>;

  /// Register `mac` as a peer so subsequent `send_to` calls succeed.
  ///
  /// Idempotent; drivers that don't need pre-registration (or already know
  /// the peer) can return `Ok(())` unconditionally.
  fn add_peer(
    &mut self,
    mac: [u8; 6],
  ) -> impl core::future::Future<Output = Result<(), Self::Error>>;

  /// Returns `true` when `mac` has been added as a peer.
  fn peer_exists(&self, mac: [u8; 6]) -> bool;
}

/// [`TransportPort`] implementation over any [`EspNowRadio`].
pub struct EspNowPort<R: EspNowRadio> {
  radio: R,
}

impl<R: EspNowRadio> EspNowPort<R> {
  /// Wrap an ESP-NOW driver.
  pub const fn new(radio: R) -> Self {
    EspNowPort { radio }
  }

  /// Ensure `mac` is registered as a peer (idempotent).
  ///
  /// Firmware bridges typically call this in response to
  /// [`crate::Relay::learn_receiver`] events so that later unicast sends can
  /// succeed. Broadcasting works without any peers registered *except* the
  /// broadcast MAC itself; call `ensure_peer(BROADCAST_ADDR)` once at
  /// startup.
  ///
  /// # Errors
  /// Returns [`PortError::UnsupportedAddr`] if the driver rejects the peer.
  pub async fn ensure_peer(&mut self, mac: [u8; 6]) -> Result<(), PortError> {
    if self.radio.peer_exists(mac) {
      return Ok(());
    }
    self
      .radio
      .add_peer(mac)
      .await
      .map_err(|_| PortError::UnsupportedAddr)
  }

  /// Access the inner radio (e.g. to drive the RX side from another task).
  pub fn inner(&mut self) -> &mut R {
    &mut self.radio
  }

  /// Consume the port and return the inner radio driver.
  pub fn into_inner(self) -> R {
    self.radio
  }
}

impl<R: EspNowRadio> TransportPort for EspNowPort<R> {
  fn transport(&self) -> Transport {
    Transport::EspNow
  }

  async fn send_frame(&mut self, addr: FrameAddr, frame: &[u8]) -> Result<(), PortError> {
    let mac = match addr {
      FrameAddr::Unicast(a) => a,
      FrameAddr::Broadcast => BROADCAST_ADDR,
    };
    // Auto-register peers on demand so that unicast to a newly-learned peer
    // doesn't require the caller to first stage `ensure_peer`.
    if !self.radio.peer_exists(mac) {
      self
        .radio
        .add_peer(mac)
        .await
        .map_err(|_| PortError::UnsupportedAddr)?;
    }
    self
      .radio
      .send_to(mac, frame)
      .await
      .map_err(|_| PortError::Io)
  }
}
