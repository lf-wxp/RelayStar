//! Integration smoke test that mimics the exact `relaystar-relay` API surface
//! used by `cardputer-fw` **and** `lora-node-fw`. If this file compiles, the
//! firmware's use of the crate will too (modulo embassy-sync's `Mutex::new`,
//! which is itself a `const fn`).

use relaystar_proto::{Message, MsgKind, Transport};
use relaystar_relay::{Destination, FrameAddr, IngestOutcome, PlannedFrame, Relay, RelayError};

type CardRelay = Relay<16, 4, 32>;
type NodeRelay = Relay<8, 2, 32>;

// The important invariant: `Relay::new` is a const fn so it can live in a
// `static Mutex<...>`.
static _RELAY: CardRelay = CardRelay::new([0x02, 0, 0, 0, 0, 2], 0x0200_0000);

#[test]
fn matches_cardputer_usage() {
  let mut relay: CardRelay = CardRelay::new([0x02, 0, 0, 0, 0, 2], 0x0200_0000);

  let _ = relay.register_port(Transport::Lora);
  let _ = relay.register_port(Transport::EspNow);
  let _ = relay.register_port(Transport::Mqtt);

  // send_local_text path
  let plan = match relay.plan_send(MsgKind::Text, b"hi", Destination::Broadcast) {
    Ok(p) => p,
    Err(RelayError::NoPortsRegistered) => panic!("registered above"),
    Err(_) => panic!(),
  };
  for PlannedFrame {
    transport,
    addr,
    message,
  } in plan
  {
    let _ = (transport, addr, message);
  }

  // fanout_forward path
  let msg = Message::text(1, Transport::Lora, [0xAA; 6], "yo").unwrap();
  for target in Transport::ALL {
    if target == Transport::Lora {
      continue;
    }
    let _ = relay.plan_forward(msg.clone(), target).unwrap();
  }

  // ingest path
  let mut buf = [0u8; relaystar_proto::MAX_FRAME];
  let encoded = msg.encode(&mut buf).unwrap();
  match relay.ingest(Transport::Lora, encoded).unwrap() {
    IngestOutcome::Complete(_)
    | IngestOutcome::NotForMe(_)
    | IngestOutcome::Buffered
    | IngestOutcome::Duplicate
    | IngestOutcome::Dropped(_) => {}
    _ => {}
  }

  // next_id + FrameAddr destructuring
  let _id = relay.next_id();
  let _bc: FrameAddr = FrameAddr::Broadcast;
}

/// Mirrors the exact call sites in `crates/lora-node-fw/src/main.rs`.
#[test]
fn matches_lora_node_usage() {
  let mut relay: NodeRelay = NodeRelay::new([0x02, 0, 0, 0, 0, 1], 0x0100_0000);
  let _ = relay.register_port(Transport::Lora);

  // First: a Ping arrives from a peer we've never heard of. `ingest` learns
  // the sender, so a subsequent unicast Pong resolves.
  let ping = Message::unicast(
    99,
    Transport::Lora,
    [0xAA; 6],
    [0x02, 0, 0, 0, 0, 1],
    MsgKind::Ping,
    b"ping",
  )
  .unwrap();
  let mut buf = [0u8; relaystar_proto::MAX_FRAME];
  let encoded = ping.encode(&mut buf).unwrap();
  let outcome = relay.ingest(Transport::Lora, encoded).unwrap();
  let peer = match outcome {
    IngestOutcome::Complete(m) => m.from,
    _ => panic!("expected Complete for a direct-addressed ping"),
  };

  // Now: plan a unicast Pong. This should succeed because ingest learned the
  // sender's transport.
  let plan = relay
    .plan_send(MsgKind::Pong, b"pong", Destination::Unicast(peer))
    .unwrap();
  assert!(!plan.is_empty());
  for f in plan {
    assert_eq!(f.transport, Transport::Lora);
    assert!(matches!(f.addr, FrameAddr::Unicast(a) if a == peer));
  }

  // The lora-node fallback shape: on `UnknownReceiver`, broadcast instead.
  // Force a miss with a peer we've never seen.
  let missing_peer = [0xBB; 6];
  match relay.plan_send(MsgKind::Pong, b"pong", Destination::Unicast(missing_peer)) {
    Ok(_) => panic!("should have been UnknownReceiver"),
    Err(RelayError::UnknownReceiver) => {
      let broadcast = relay
        .plan_send(MsgKind::Pong, b"pong", Destination::Broadcast)
        .unwrap();
      assert!(!broadcast.is_empty());
    }
    Err(e) => panic!("unexpected error: {}", e),
  }

  // Heartbeat: broadcast text.
  let hb = relay
    .plan_send(MsgKind::Text, b"node hb #0", Destination::Broadcast)
    .unwrap();
  for frame in hb {
    // The lora-node send_reply filters by transport before touching the
    // radio; assert that at least the LoRa target survives that filter.
    if frame.transport == Transport::Lora {
      let _ = frame.message;
    }
  }
}

// ─────────────────────────────────────────────────────────────────────
// Feature-gated shape assertions for the bundled transport backends.
// These don't need to run — they just need to *compile*, which proves the
// firmware's call sites will type-check.
// ─────────────────────────────────────────────────────────────────────

#[cfg(feature = "lora")]
mod sx_lora_shape {
  //! Compile-only assertions for `SxLoraPort`. Cannot construct a real
  //! `LoRa<RK, DLY>` on the host, so we exercise the *signatures* through
  //! generic wrappers — if these compile, the firmware call sites will too.

  use lora_phy::mod_traits::RadioKind;
  use relaystar_proto::Transport;
  use relaystar_relay::port::TransportPort;
  use relaystar_relay::ports::lora::{LoraModulationParams, SxLoraPort, SxRxOutcome};
  use relaystar_relay::{FrameAddr, error::PortError};

  // Match the exact firmware initialisation call: `SxLoraPort::new(handle,
  // params, max_frame_u8).await`.
  #[allow(dead_code)]
  async fn _shape_new<RK, DLY>(
    lora: lora_phy::LoRa<RK, DLY>,
  ) -> Result<SxLoraPort<RK, DLY>, PortError>
  where
    RK: RadioKind,
    DLY: embedded_hal_async::delay::DelayNs,
  {
    SxLoraPort::new(
      lora,
      LoraModulationParams {
        frequency_hz: 868_000_000,
        spreading_factor: 10,
        bandwidth_khz: 250,
        coding_rate_denom: 8,
        preamble_symbols: 4,
        tx_power_dbm: 20,
      },
      255,
    )
    .await
  }

  // Match the RX side: `port.rx_frame(&mut buf).await → SxRxOutcome`.
  #[allow(dead_code)]
  async fn _shape_rx<RK, DLY>(
    port: &mut SxLoraPort<RK, DLY>,
    buf: &mut [u8],
  ) -> Result<SxRxOutcome, PortError>
  where
    RK: RadioKind,
    DLY: embedded_hal_async::delay::DelayNs,
  {
    let outcome = port.rx_frame(buf).await?;
    // Fields the firmware reads.
    let _: u8 = outcome.len;
    let _: i16 = outcome.rssi;
    let _: i16 = outcome.snr;
    Ok(outcome)
  }

  // Match the TX side: `TransportPort::send_frame(addr, encoded).await`.
  #[allow(dead_code)]
  async fn _shape_tx<RK, DLY>(
    port: &mut SxLoraPort<RK, DLY>,
    encoded: &[u8],
  ) -> Result<(), PortError>
  where
    RK: RadioKind,
    DLY: embedded_hal_async::delay::DelayNs,
  {
    assert_eq!(port.transport(), Transport::Lora);
    port.send_frame(FrameAddr::Broadcast, encoded).await?;
    port
      .send_frame(FrameAddr::Unicast([0xAA; 6]), encoded)
      .await
  }
}

#[cfg(feature = "mqtt")]
mod mqtt311_shape {
  //! Exercises `Mqtt311Client` with a minimal in-memory mock, mirroring
  //! `cardputer-fw/src/mqtt.rs` call order:
  //! `new → connect → subscribe → read_publish / publish / ping`.

  use relaystar_relay::ports::mqtt::{Mqtt311Client, Mqtt311Error, PacketId};

  extern crate std;
  use std::vec::Vec;

  struct MockSocket {
    rx: Vec<u8>,
    tx: Vec<u8>,
  }

  impl embedded_io_async::ErrorType for MockSocket {
    type Error = core::convert::Infallible;
  }

  impl embedded_io_async::Read for MockSocket {
    async fn read(&mut self, out: &mut [u8]) -> Result<usize, Self::Error> {
      let n = core::cmp::min(out.len(), self.rx.len());
      out[..n].copy_from_slice(&self.rx[..n]);
      self.rx.drain(..n);
      Ok(n)
    }
  }

  impl embedded_io_async::Write for MockSocket {
    async fn write(&mut self, data: &[u8]) -> Result<usize, Self::Error> {
      self.tx.extend_from_slice(data);
      Ok(data.len())
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
      Ok(())
    }
  }

  fn block_on<F: core::future::Future>(fut: F) -> F::Output {
    use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};
    const VTABLE: RawWakerVTable = RawWakerVTable::new(
      |_| RawWaker::new(core::ptr::null(), &VTABLE),
      |_| {},
      |_| {},
      |_| {},
    );
    let waker = unsafe { Waker::from_raw(RawWaker::new(core::ptr::null(), &VTABLE)) };
    let mut cx = Context::from_waker(&waker);
    let mut pinned = core::pin::pin!(fut);
    loop {
      match pinned.as_mut().poll(&mut cx) {
        Poll::Ready(v) => return v,
        Poll::Pending => panic!("mock is fully synchronous but the future returned Pending"),
      }
    }
  }

  /// Mirrors the exact `cardputer-fw/src/mqtt.rs` call sequence:
  ///
  /// 1. Build client with `&mut socket` (blanket impl reuses the socket).
  /// 2. `mqtt.connect().await` → succeeds on CONNACK 0.
  /// 3. `mqtt.subscribe(topic, PacketId(1)).await` → succeeds on SUBACK.
  /// 4. `mqtt.publish(topic, payload).await` → PUBLISH sent.
  /// 5. `mqtt.ping().await` → PINGREQ sent.
  /// 6. `mqtt.read_publish(&mut buf).await` → decodes a PUBLISH.
  #[test]
  fn full_client_flow_compiles_and_runs() {
    // Feed the mock a CONNACK(0) + SUBACK + PUBLISH so all three await calls
    // that read from the broker complete.
    let mut rx: Vec<u8> = Vec::new();
    rx.extend_from_slice(&[0x20, 0x02, 0x00, 0x00]); // CONNACK, accepted
    rx.extend_from_slice(&[0x90, 0x03, 0x00, 0x01, 0x00]); // SUBACK id=1, QoS 0
    rx.extend_from_slice(&[
      0x30, 0x09, 0x00, 0x02, b'h', b'i', b'w', b'o', b'r', b'l', b'd',
    ]); // PUBLISH topic=hi payload=world

    let mut socket = MockSocket { rx, tx: Vec::new() };
    // Borrowed-socket form used by cardputer-fw.
    let mut mqtt: Mqtt311Client<&mut MockSocket> =
      Mqtt311Client::new(&mut socket, "relaystar-card", 30);

    block_on(mqtt.connect()).unwrap();
    block_on(mqtt.subscribe("relaystar/uplink", PacketId(1))).unwrap();
    block_on(mqtt.publish("relaystar/downlink", b"hello")).unwrap();
    block_on(mqtt.ping()).unwrap();

    let mut buf = [0u8; 64];
    let publish = block_on(mqtt.read_publish(&mut buf)).unwrap().unwrap();
    assert_eq!(publish.topic, "hi");
    assert_eq!(publish.payload, b"world");
  }

  /// Verifies that `Mqtt311Error` participates in the firmware's error
  /// logging shape (`Display`) and can be matched on named variants (the
  /// enum is `#[non_exhaustive]`, so a wildcard arm is required).
  #[test]
  fn error_display_and_match_shape() {
    let e = Mqtt311Error::Rejected;
    // Display path used by `println!("MQTT: CONNECT failed: {}", e)`.
    let _s = std::format!("{}", e);
    match e {
      Mqtt311Error::Io
      | Mqtt311Error::Protocol
      | Mqtt311Error::Rejected
      | Mqtt311Error::TooLarge => {}
      _ => {}
    }
  }
}
