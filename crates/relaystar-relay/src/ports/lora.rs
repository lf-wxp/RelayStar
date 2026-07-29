//! LoRa transport adapter.
//!
//! LoRa is a shared-medium radio: every node in range hears every frame.
//! "Unicast" and "broadcast" are therefore *logical* labels applied at the
//! link layer — the wire itself carries the same bits. The relay honours the
//! distinction by writing the target address into [`relaystar_proto::Message::to`]
//! before encoding; downstream receivers filter (or don't) accordingly.
//!
//! ## Two integration paths
//!
//! ### 1. Bring your own driver — implement [`LoraRadio`]
//!
//! Wrap any radio you like (a custom SPI protocol, a mocked test double, a
//! different driver crate) by implementing [`LoraRadio`], then hand it to
//! [`LoraPort::new`]. This is the most flexible option and has no extra
//! dependency footprint on `relaystar-relay`.
//!
//! ### 2. Use the bundled [`SxLoraPort`] (feature = `"lora"`)
//!
//! When enabled, the crate ships a ready-made port on top of the
//! [`lora-phy`](https://docs.rs/lora-phy) driver. You still supply the
//! SPI + GPIO handles (those are board-specific), but everything from
//! `prepare_for_tx` → `tx` → `sleep` is handled for you.
//!
//! ## Example (BYO driver)
//!
//! ```ignore
//! use relaystar_relay::ports::lora::{LoraPort, LoraRadio};
//!
//! struct MyLora { /* lora-phy handle, packet params, ... */ }
//! impl LoraRadio for MyLora {
//!   type Error = MyErr;
//!   async fn transmit(&mut self, frame: &[u8]) -> Result<(), Self::Error> {
//!     // drive prepare_for_tx + tx() on your LoRa handle
//!     Ok(())
//!   }
//! }
//!
//! let mut port = LoraPort::new(MyLora { /* ... */ });
//! ```

use crate::error::PortError;
use crate::port::{FrameAddr, TransportPort};
use relaystar_proto::Transport;

/// Hardware-facing trait that a LoRa driver must implement.
///
/// The relay hands the driver a single already-encoded frame (a serialised
/// [`relaystar_proto::Message`]); the driver is responsible for whatever
/// timing / prepare / actually-transmit / go-back-to-rx dance the underlying
/// chip requires.
pub trait LoraRadio {
  /// Driver-specific error. Mapped to [`PortError::Io`].
  type Error: core::fmt::Debug;

  /// Transmit one frame. Blocks until the airtime is complete.
  fn transmit(
    &mut self,
    frame: &[u8],
  ) -> impl core::future::Future<Output = Result<(), Self::Error>>;
}

/// [`TransportPort`] implementation over any [`LoraRadio`].
///
/// `LoraPort` is a thin wrapper: no fragmentation, no dedup, no MTU logic —
/// those all live in [`crate::Relay`]. It exists so that
/// [`Transport::Lora`](relaystar_proto::Transport::Lora) has a concrete port
/// type instead of forcing every firmware to write its own.
pub struct LoraPort<R: LoraRadio> {
  radio: R,
}

impl<R: LoraRadio> LoraPort<R> {
  /// Wrap a radio driver.
  pub const fn new(radio: R) -> Self {
    LoraPort { radio }
  }

  /// Access the inner radio (e.g. to feed it into a separate RX task).
  pub fn inner(&mut self) -> &mut R {
    &mut self.radio
  }

  /// Consume the port and return the inner radio driver.
  pub fn into_inner(self) -> R {
    self.radio
  }
}

impl<R: LoraRadio> TransportPort for LoraPort<R> {
  fn transport(&self) -> Transport {
    Transport::Lora
  }

  async fn send_frame(&mut self, _addr: FrameAddr, frame: &[u8]) -> Result<(), PortError> {
    // LoRa is a broadcast medium; the destination MAC is already baked into
    // the encoded frame (`Message::to`). Whether a receiver acts on it is
    // decided at ingest time.
    self
      .radio
      .transmit(frame)
      .await
      .map_err(|_| PortError::Io)?;
    Ok(())
  }
}

// ──────────────────────────────────────────────────────────────────────────
// Bundled `lora-phy` implementation (feature = "lora").
// ──────────────────────────────────────────────────────────────────────────

#[cfg(feature = "lora")]
pub use sx126x_impl::{LoraModulation, SxLoraPort, SxRxOutcome};

/// LoRa modulation parameters expressed in wire-friendly units. Used by
/// [`SxLoraPort::new`] so callers don't have to reach into `lora-phy` enums
/// themselves.
///
/// Sensible defaults for EU868 are exposed on
/// [`relaystar_proto::radio`](relaystar_proto::radio).
#[cfg(feature = "lora")]
#[cfg_attr(docsrs, doc(cfg(feature = "lora")))]
#[derive(Debug, Clone, Copy)]
pub struct LoraModulationParams {
  /// Centre frequency in Hz (e.g. `868_000_000`).
  pub frequency_hz: u32,
  /// Spreading factor, `5..=12`.
  pub spreading_factor: u8,
  /// Bandwidth in kHz; supported values are `125`, `250`, `500`.
  pub bandwidth_khz: u16,
  /// Coding rate denominator (4/5, 4/6, 4/7, 4/8 → 5, 6, 7, 8).
  pub coding_rate_denom: u8,
  /// Preamble length in symbols (typical: 4-12).
  pub preamble_symbols: u16,
  /// TX output power in dBm.
  pub tx_power_dbm: i32,
}

#[cfg(feature = "lora")]
mod sx126x_impl {
  //! Concrete `TransportPort` built on top of `lora-phy`'s `LoRa<RK, DLY>`.
  //!
  //! The board-specific bits (SPI + GPIO + antenna switch + TCXO voltage)
  //! must be handled by the caller *before* constructing the `LoRa<..>`
  //! handle; this port takes over from that point.

  use super::LoraModulationParams;
  use crate::error::PortError;
  use crate::port::{FrameAddr, TransportPort};
  use lora_phy::mod_params::{
    Bandwidth, CodingRate, ModulationParams, PacketParams, SpreadingFactor,
  };
  use lora_phy::mod_traits::RadioKind;
  use lora_phy::{LoRa, RxMode};
  use relaystar_proto::Transport;

  /// Re-export of `LoraModulationParams` for use inside the module.
  #[doc(hidden)]
  pub type LoraModulation = LoraModulationParams;

  /// A completed receive on a [`SxLoraPort`].
  #[derive(Debug, Clone, Copy)]
  #[cfg_attr(docsrs, doc(cfg(feature = "lora")))]
  pub struct SxRxOutcome {
    /// Number of bytes written to the caller's buffer.
    pub len: u8,
    /// RSSI reported by the radio (dBm, negative).
    pub rssi: i16,
    /// SNR reported by the radio (dB).
    pub snr: i16,
  }

  /// Bundled [`TransportPort`] backed by a `lora-phy` [`LoRa`] handle.
  ///
  /// Owns the handle plus pre-computed modulation and packet parameters, so
  /// each call to [`Self::send_frame`] or [`Self::rx_frame`] is a single
  /// `prepare_* + do_op + sleep` sequence.
  #[cfg_attr(docsrs, doc(cfg(feature = "lora")))]
  pub struct SxLoraPort<RK, DLY>
  where
    RK: RadioKind,
    DLY: embedded_hal_async::delay::DelayNs,
  {
    lora: LoRa<RK, DLY>,
    modulation: ModulationParams,
    rx_params: PacketParams,
    tx_params: PacketParams,
    tx_power_dbm: i32,
  }

  impl<RK, DLY> SxLoraPort<RK, DLY>
  where
    RK: RadioKind,
    DLY: embedded_hal_async::delay::DelayNs,
  {
    /// Build a port from a *fully-initialised* `LoRa<RK, DLY>` handle plus
    /// the wire parameters. The `max_frame` argument is the largest single
    /// frame the caller intends to receive (usually
    /// [`relaystar_proto::MAX_FRAME`] cast to `u8`).
    ///
    /// # Errors
    /// Returns [`PortError::NotReady`] if `lora-phy` rejects the modulation
    /// or packet parameters (usually a spreading-factor / bandwidth /
    /// coding-rate mismatch).
    pub async fn new(
      mut lora: LoRa<RK, DLY>,
      params: LoraModulationParams,
      max_frame: u8,
    ) -> Result<Self, PortError> {
      let modulation = lora
        .create_modulation_params(
          map_spreading_factor(params.spreading_factor),
          map_bandwidth(params.bandwidth_khz),
          map_coding_rate(params.coding_rate_denom),
          params.frequency_hz,
        )
        .map_err(|_| PortError::NotReady)?;

      let rx_params = lora
        .create_rx_packet_params(
          params.preamble_symbols,
          false,
          max_frame,
          true,
          false,
          &modulation,
        )
        .map_err(|_| PortError::NotReady)?;

      let tx_params = lora
        .create_tx_packet_params(params.preamble_symbols, false, true, false, &modulation)
        .map_err(|_| PortError::NotReady)?;

      Ok(SxLoraPort {
        lora,
        modulation,
        rx_params,
        tx_params,
        tx_power_dbm: params.tx_power_dbm,
      })
    }

    /// Enter continuous receive mode and wait for a single frame. Fills
    /// `buf` with the payload and returns metadata.
    ///
    /// # Errors
    /// Returns [`PortError::Io`] on any driver-level failure.
    pub async fn rx_frame(&mut self, buf: &mut [u8]) -> Result<SxRxOutcome, PortError> {
      self
        .lora
        .prepare_for_rx(RxMode::Continuous, &self.modulation, &self.rx_params)
        .await
        .map_err(|_| PortError::Io)?;

      let (len, status) = self
        .lora
        .rx(&self.rx_params, buf)
        .await
        .map_err(|_| PortError::Io)?;

      Ok(SxRxOutcome {
        len,
        rssi: status.rssi,
        snr: status.snr,
      })
    }

    /// Access the inner `lora-phy` handle. Useful when the caller needs to
    /// e.g. run `lora.sleep()` between long idle periods.
    pub fn inner(&mut self) -> &mut LoRa<RK, DLY> {
      &mut self.lora
    }

    /// Consume the port and return the inner handle.
    pub fn into_inner(self) -> LoRa<RK, DLY> {
      self.lora
    }
  }

  impl<RK, DLY> TransportPort for SxLoraPort<RK, DLY>
  where
    RK: RadioKind,
    DLY: embedded_hal_async::delay::DelayNs,
  {
    fn transport(&self) -> Transport {
      Transport::Lora
    }

    async fn send_frame(&mut self, _addr: FrameAddr, frame: &[u8]) -> Result<(), PortError> {
      self
        .lora
        .prepare_for_tx(
          &self.modulation,
          &mut self.tx_params,
          self.tx_power_dbm,
          frame,
        )
        .await
        .map_err(|_| PortError::Io)?;

      self.lora.tx().await.map_err(|_| PortError::Io)?;
      let _ = self.lora.sleep(false).await;
      Ok(())
    }
  }

  fn map_spreading_factor(sf: u8) -> SpreadingFactor {
    match sf {
      5 => SpreadingFactor::_5,
      6 => SpreadingFactor::_6,
      7 => SpreadingFactor::_7,
      8 => SpreadingFactor::_8,
      9 => SpreadingFactor::_9,
      10 => SpreadingFactor::_10,
      11 => SpreadingFactor::_11,
      _ => SpreadingFactor::_12,
    }
  }

  fn map_bandwidth(khz: u16) -> Bandwidth {
    match khz {
      125 => Bandwidth::_125KHz,
      250 => Bandwidth::_250KHz,
      _ => Bandwidth::_500KHz,
    }
  }

  fn map_coding_rate(denom: u8) -> CodingRate {
    match denom {
      5 => CodingRate::_4_5,
      6 => CodingRate::_4_6,
      7 => CodingRate::_4_7,
      _ => CodingRate::_4_8,
    }
  }
}
