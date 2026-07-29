#![cfg_attr(not(feature = "std"), no_std)]
#![doc = include_str!("../README.md")]

//! # RelayStar relay engine
//!
//! See [`README.md`](https://github.com/franciscowu/RelayStar/blob/main/crates/relaystar-relay/README.md)
//! for a walkthrough. Below is the shortest possible tour.
//!
//! ## What this crate does
//!
//! - **Fragments** application payloads that exceed a transport's MTU into a
//!   sequence of on-wire frames.
//! - **Reassembles** fragments on the receive side.
//! - **Deduplicates** by message id so the same frame cannot loop back through
//!   the mesh.
//! - **Tracks receivers**: which peers are reachable on which transports.
//! - **Fans out** unicast to all transports where the peer is reachable, and
//!   broadcast to all registered transports (each using its own native
//!   broadcast facility).
//! - **Provides transport adapters** ([`ports::lora::LoraPort`],
//!   [`ports::espnow::EspNowPort`], [`ports::mqtt::MqttPort`]) that translate
//!   `RelayStar` frames + [`FrameAddr`] into transport-native calls. You only
//!   have to implement a tiny hardware-facing trait (e.g.
//!   [`ports::lora::LoraRadio`]).
//!
//! ## Two usage flavours
//!
//! ### 1. Direct API (`Relay::send` / `Relay::forward`)
//!
//! Call the async methods and pass in `&mut` references to your transport
//! ports. The relay drives fragmentation, encoding, and sending for you.
//!
//! ```ignore
//! use relaystar_relay::{Relay, Destination, ports::lora::LoraPort};
//! use relaystar_proto::MsgKind;
//!
//! let mut relay: Relay<16, 4, 32> = Relay::new([0x02, 0, 0, 0, 0, 1], 0x0100_0000);
//! let _ = relay.register_port(relaystar_proto::Transport::Lora);
//! let mut lora = LoraPort::new(my_lora_driver);
//!
//! relay
//!   .send(&mut lora, MsgKind::Text, b"hello world",
//!         Destination::Unicast([0x02, 0, 0, 0, 0, 2]))
//!   .await?;
//! ```
//!
//! Because every call takes `&mut P` where `P: TransportPort`, the
//! generic-over-port design keeps async futures inline (no heap, no
//! `dyn Future`) and remains fully `no_std`.
//!
//! ### 2. Planner API (`Relay::plan_send` / `Relay::plan_forward`)
//!
//! When your firmware already routes outbound frames through embassy
//! `Channel`s (as the Cardputer bridge does), skip the direct API and let the
//! relay compute a list of `(Transport, FrameAddr, Message)` triples that you
//! push onto whichever channel matches. This is what
//! [`crates/cardputer-fw/src/bridge.rs`] should be migrated to.
//!
//! Both APIs share the same fragmentation, deduplication, and receiver-table
//! logic.

pub mod error;
pub mod fragment;
pub mod port;
pub mod ports;
pub mod receivers;

use core::sync::atomic::{AtomicU32, Ordering};

use heapless::Vec as HVec;
use relaystar_proto::{
  BROADCAST_ADDR, DEFAULT_TTL, MAX_FRAME, Message, MsgKind, SeenCache, Transport,
};

pub use error::{PortError, RelayError};
pub use fragment::{Fragmenter, MAX_FRAGMENTS, ReassembleOutcome, Reassembler, RejectReason};
pub use port::{FrameAddr, TransportPort};
pub use receivers::{Receiver, ReceiverTable};

/// Re-export the underlying protocol crate so downstream users don't need a
/// separate dependency line.
pub use relaystar_proto as proto;

/// Application-level destination for [`Relay::send`].
///
/// Distinct from [`FrameAddr`], which is the per-frame transport-native
/// addressing hint chosen by the relay engine after consulting the
/// [`ReceiverTable`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Destination {
  /// Send to a specific peer address. The relay looks the address up in the
  /// [`ReceiverTable`] to decide which transports to use.
  Unicast([u8; 6]),
  /// Broadcast on every registered transport, using each transport's own
  /// broadcast facility.
  Broadcast,
}

/// Result of ingesting an inbound frame.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub enum IngestOutcome {
  /// The frame was a duplicate (id already seen recently). Dropped.
  Duplicate,
  /// A fragment was buffered; more slices are pending before the payload is
  /// complete.
  Buffered,
  /// A complete [`Message`] is ready for the application. Whether to forward
  /// it further is up to the caller (they can call [`Relay::forward`]).
  Complete(Message),
  /// The frame was decoded but its `to` field targets neither this node nor
  /// broadcast. The full message is returned so the caller can decide to
  /// relay it (typical bridge behaviour).
  NotForMe(Message),
  /// A fragment was dropped by the reassembler.
  Dropped(RejectReason),
}

/// A single frame planned for emission by [`Relay::plan_send`] or
/// [`Relay::plan_forward`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PlannedFrame {
  /// Which transport should carry this frame.
  pub transport: Transport,
  /// How the transport should address this frame.
  pub addr: FrameAddr,
  /// The `Message` to encode and send.
  pub message: Message,
}

/// The relay engine.
///
/// Generic parameters:
/// - `NR`: max tracked peers in the [`ReceiverTable`] (`>= 1`).
/// - `NA`: number of concurrent reassembly slots (`>= 1`).
/// - `NF`: max fragments *per group* (`>= 1`, `<= 64`).
pub struct Relay<const NR: usize, const NA: usize, const NF: usize> {
  self_addr: [u8; 6],
  id_ctr: AtomicU32,
  id_base: u32,
  receivers: ReceiverTable<NR>,
  reassembler: Reassembler<NA, NF>,
  seen: SeenCache<64>,
  /// Which transports have been registered, in [`Transport::ALL`] order.
  registered: [bool; 3],
}

impl<const NR: usize, const NA: usize, const NF: usize> Relay<NR, NA, NF> {
  /// Create a fresh relay owned by node `self_addr`. `id_base` is XOR'd into
  /// every allocated message id to keep ids from colliding with other nodes'
  /// counters.
  pub const fn new(self_addr: [u8; 6], id_base: u32) -> Self {
    Relay {
      self_addr,
      id_ctr: AtomicU32::new(0),
      id_base,
      receivers: ReceiverTable::new(),
      reassembler: Reassembler::new(),
      seen: SeenCache::new(),
      registered: [false; 3],
    }
  }

  /// Address of the local node.
  pub const fn self_addr(&self) -> [u8; 6] {
    self.self_addr
  }

  /// Allocate a fresh, monotonically-increasing message id.
  pub fn next_id(&self) -> u32 {
    self
      .id_base
      .wrapping_add(self.id_ctr.fetch_add(1, Ordering::Relaxed))
  }

  // ──────────────────────────────────────────────────────────────────────
  // Port registration
  // ──────────────────────────────────────────────────────────────────────

  /// Declare that a transport is available. This does not take ownership of
  /// any hardware; it merely enables broadcast fan-out on that transport.
  ///
  /// # Errors
  /// Never returns an error today; kept for API stability.
  pub fn register_port(&mut self, t: Transport) -> Result<(), RelayError> {
    self.registered[Self::transport_idx(t)] = true;
    Ok(())
  }

  /// Undeclare a transport.
  pub fn unregister_port(&mut self, t: Transport) {
    self.registered[Self::transport_idx(t)] = false;
  }

  /// Returns `true` if any transport has been registered.
  pub fn has_ports(&self) -> bool {
    self.registered.iter().any(|b| *b)
  }

  /// Returns `true` if `t` was registered via [`Self::register_port`].
  pub fn is_port_registered(&self, t: Transport) -> bool {
    self.registered[Self::transport_idx(t)]
  }

  const fn transport_idx(t: Transport) -> usize {
    match t {
      Transport::Lora => 0,
      Transport::Mqtt => 1,
      Transport::EspNow => 2,
    }
  }

  // ──────────────────────────────────────────────────────────────────────
  // Receiver-table management (delegated conveniences)
  // ──────────────────────────────────────────────────────────────────────

  /// Immutable access to the receiver table.
  pub fn receivers(&self) -> &ReceiverTable<NR> {
    &self.receivers
  }

  /// Explicitly register that `addr` is reachable on `t`.
  ///
  /// # Errors
  /// See [`ReceiverTable::add`].
  pub fn add_receiver(&mut self, addr: [u8; 6], t: Transport) -> Result<bool, RelayError> {
    self.receivers.add(addr, t)
  }

  /// Remove `addr` entirely from the receiver table.
  pub fn remove_receiver(&mut self, addr: [u8; 6]) -> bool {
    self.receivers.remove(addr)
  }

  /// Auto-learn `addr` from an inbound frame arriving via `t`.
  pub fn learn_receiver(&mut self, addr: [u8; 6], t: Transport) -> bool {
    self.receivers.learn(addr, t)
  }

  // ──────────────────────────────────────────────────────────────────────
  // Planner API (no I/O): plan_send / plan_forward
  // ──────────────────────────────────────────────────────────────────────

  /// Compute the frames that should be emitted to send `payload` with `kind`
  /// to `dest`.
  ///
  /// The relay allocates a message id, applies MTU-aware fragmentation for
  /// every selected target transport, and returns a list of
  /// [`PlannedFrame`]s. Callers push each frame onto their transport-specific
  /// channel (for example a fw-side embassy `Channel`).
  ///
  /// # Errors
  /// See [`Self::send`].
  pub fn plan_send(
    &self,
    kind: MsgKind,
    payload: &[u8],
    dest: Destination,
  ) -> Result<HVec<PlannedFrame, { MAX_FRAGMENTS * 3 }>, RelayError> {
    let targets = self.resolve_targets(dest)?;
    if targets.is_empty() {
      return Err(RelayError::NoPortsRegistered);
    }
    let id = self.next_id();
    let to = match dest {
      Destination::Unicast(a) => a,
      Destination::Broadcast => BROADCAST_ADDR,
    };
    let frame_addr = match dest {
      Destination::Broadcast => FrameAddr::Broadcast,
      Destination::Unicast(a) => FrameAddr::Unicast(a),
    };

    let mut out: HVec<PlannedFrame, { MAX_FRAGMENTS * 3 }> = HVec::new();
    for t in targets {
      let msg = Fragmenter::prepare(id, DEFAULT_TTL, t, self.self_addr, to, kind, payload)?;
      let group = self.next_id();
      let frames = Fragmenter::split(msg, t, group)?;
      for frame in frames {
        out
          .push(PlannedFrame {
            transport: t,
            addr: frame_addr,
            message: frame,
          })
          .map_err(|_| RelayError::PayloadUnfragmentable)?;
      }
    }
    Ok(out)
  }

  /// Compute the frames needed to forward `msg` onto `target`. Decrements TTL
  /// and fragments as needed. Returns an empty list when TTL is exhausted.
  ///
  /// # Errors
  /// See [`Self::forward`].
  pub fn plan_forward(
    &self,
    msg: Message,
    target: Transport,
  ) -> Result<HVec<PlannedFrame, MAX_FRAGMENTS>, RelayError> {
    let mut out: HVec<PlannedFrame, MAX_FRAGMENTS> = HVec::new();
    let Some(relayed) = msg.prepared_for_relay() else {
      return Ok(out);
    };
    let frame_addr = if relayed.is_broadcast() {
      FrameAddr::Broadcast
    } else {
      FrameAddr::Unicast(relayed.to)
    };
    let group = self.next_id();
    let frames = Fragmenter::split(relayed, target, group)?;
    for frame in frames {
      out
        .push(PlannedFrame {
          transport: target,
          addr: frame_addr,
          message: frame,
        })
        .map_err(|_| RelayError::PayloadUnfragmentable)?;
    }
    Ok(out)
  }

  // ──────────────────────────────────────────────────────────────────────
  // Direct-I/O API: send / forward (generic over TransportPort)
  // ──────────────────────────────────────────────────────────────────────

  /// Send a payload directly through the supplied port(s).
  ///
  /// This variant takes a single [`TransportPort`] and only fans out on that
  /// transport (i.e. it is the natural entry point when your firmware owns a
  /// single radio). For multi-transport nodes, either call `send` once per
  /// port, or use [`Self::plan_send`] and dispatch yourself.
  ///
  /// # Errors
  /// - [`RelayError::UnknownReceiver`] when `dest = Unicast` and the address
  ///   is not in the [`ReceiverTable`].
  /// - [`RelayError::NoSuchPort`] when the port doesn't match a resolved
  ///   target transport.
  /// - [`RelayError::Protocol`] / [`RelayError::Port`] propagated from the
  ///   fragmentation / encoding / send path.
  pub async fn send<P: TransportPort>(
    &mut self,
    port: &mut P,
    kind: MsgKind,
    payload: &[u8],
    dest: Destination,
  ) -> Result<usize, RelayError> {
    let targets = self.resolve_targets(dest)?;
    if !targets.iter().any(|t| *t == port.transport()) {
      return Err(RelayError::NoSuchPort);
    }
    let id = self.next_id();
    let to = match dest {
      Destination::Unicast(a) => a,
      Destination::Broadcast => BROADCAST_ADDR,
    };
    let msg = Fragmenter::prepare(
      id,
      DEFAULT_TTL,
      port.transport(),
      self.self_addr,
      to,
      kind,
      payload,
    )?;
    self.emit(port, msg, dest).await
  }

  /// Forward an existing [`Message`] onto `port`.
  ///
  /// # Errors
  /// See [`Self::send`].
  pub async fn forward<P: TransportPort>(
    &mut self,
    port: &mut P,
    msg: Message,
  ) -> Result<usize, RelayError> {
    let Some(relayed) = msg.prepared_for_relay() else {
      return Ok(0);
    };
    let dest = if relayed.is_broadcast() {
      Destination::Broadcast
    } else {
      Destination::Unicast(relayed.to)
    };
    self.emit(port, relayed, dest).await
  }

  /// Handle bytes arriving from `source`. The byte slice must contain a
  /// single encoded [`Message`] frame.
  ///
  /// # Errors
  /// Returns [`RelayError::Protocol`] if the bytes cannot be decoded.
  pub fn ingest(&mut self, source: Transport, frame: &[u8]) -> Result<IngestOutcome, RelayError> {
    let mut msg = Message::decode(frame)?;
    msg.origin = source;

    if msg.from != BROADCAST_ADDR && msg.from != self.self_addr {
      let _ = self.receivers.add(msg.from, source);
    }

    // Fragments share `id`; key dedup on (id, seq) so we still drop true
    // duplicates without collapsing distinct slices.
    let dedup_key = match msg.frag {
      Some(f) => msg.id ^ ((f.seq as u32) << 16),
      None => msg.id,
    };
    if !self.seen.check_and_insert(dedup_key) {
      return Ok(IngestOutcome::Duplicate);
    }

    let outcome = self.reassembler.ingest(msg);
    let result = match outcome {
      ReassembleOutcome::Passthrough(m) => self.direct(m),
      ReassembleOutcome::Complete(m) => self.direct(m),
      ReassembleOutcome::Buffered => IngestOutcome::Buffered,
      ReassembleOutcome::Dropped(r) => IngestOutcome::Dropped(r),
    };
    Ok(result)
  }

  fn direct(&self, msg: Message) -> IngestOutcome {
    if msg.to == self.self_addr || msg.is_broadcast() {
      IngestOutcome::Complete(msg)
    } else {
      IngestOutcome::NotForMe(msg)
    }
  }

  // ──────────────────────────────────────────────────────────────────────
  // Internals
  // ──────────────────────────────────────────────────────────────────────

  fn resolve_targets(&self, dest: Destination) -> Result<HVec<Transport, 3>, RelayError> {
    if !self.has_ports() {
      return Err(RelayError::NoPortsRegistered);
    }
    let mut out: HVec<Transport, 3> = HVec::new();
    match dest {
      Destination::Broadcast => {
        for t in Transport::ALL {
          if self.registered[Self::transport_idx(t)] {
            let _ = out.push(t);
          }
        }
      }
      Destination::Unicast(addr) => {
        let row = self
          .receivers
          .lookup(addr)
          .ok_or(RelayError::UnknownReceiver)?;
        for t in row.transports() {
          if self.registered[Self::transport_idx(t)] {
            let _ = out.push(t);
          }
        }
        if out.is_empty() {
          return Err(RelayError::UnknownReceiver);
        }
      }
    }
    Ok(out)
  }

  async fn emit<P: TransportPort>(
    &self,
    port: &mut P,
    msg: Message,
    dest: Destination,
  ) -> Result<usize, RelayError> {
    let target = port.transport();
    let frame_addr = match dest {
      Destination::Broadcast => FrameAddr::Broadcast,
      Destination::Unicast(a) => FrameAddr::Unicast(a),
    };

    let group = self.next_id();
    let frames = Fragmenter::split(msg, target, group)?;
    let mut count = 0usize;
    let mut buf = [0u8; MAX_FRAME];
    for frame in frames {
      let encoded = frame.encode(&mut buf)?;
      port.send_frame(frame_addr, encoded).await?;
      count += 1;
    }
    Ok(count)
  }
}

#[cfg(test)]
mod tests;
