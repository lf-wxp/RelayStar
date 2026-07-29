//! Unit tests for the relay engine.
//!
//! Tests run on the host under `cargo test`; the `#[cfg(test)]` gate ensures
//! they don't touch firmware builds.

extern crate std;

use std::vec;
use std::vec::Vec;

use super::*;
use relaystar_proto::{BROADCAST_ADDR, MAX_PAYLOAD, Message, MsgKind, Transport};

// ─── Fragmenter ────────────────────────────────────────────────────────

#[test]
fn small_payload_is_single_frame() {
  let msg = Fragmenter::prepare(
    1,
    4,
    Transport::Lora,
    [1; 6],
    BROADCAST_ADDR,
    MsgKind::Text,
    b"hi",
  )
  .unwrap();
  let frames = Fragmenter::split(msg, Transport::Lora, 42).unwrap();
  assert_eq!(frames.len(), 1);
  assert!(frames[0].frag.is_none());
}

#[test]
fn large_payload_fragments_for_lora() {
  // Use a payload bigger than LoRa's MTU but within MAX_PAYLOAD.
  let n = MAX_PAYLOAD; // 200 bytes
  let payload: Vec<u8> = (0..n).map(|i| i as u8).collect();
  let msg = Fragmenter::prepare(
    2,
    4,
    Transport::Lora,
    [1; 6],
    BROADCAST_ADDR,
    MsgKind::Telemetry,
    &payload,
  )
  .unwrap();
  let frames = Fragmenter::split(msg, Transport::Lora, 100).unwrap();
  let lora_mtu = Transport::Lora.max_payload();
  let expected = n.div_ceil(lora_mtu);
  assert_eq!(frames.len(), expected);
  for (i, f) in frames.iter().enumerate() {
    let frag = f.frag.as_ref().expect("frame is a fragment");
    assert_eq!(frag.group_id, 100);
    assert_eq!(frag.seq as usize, i);
    assert_eq!(frag.total as usize, expected);
    assert!(f.payload.len() <= lora_mtu);
  }
  // Concatenation restores the original payload.
  let reassembled: Vec<u8> = frames
    .iter()
    .flat_map(|f| f.payload.iter().copied())
    .collect();
  assert_eq!(reassembled, payload);
}

#[test]
fn mqtt_needs_no_fragmentation_for_small() {
  let msg = Fragmenter::prepare(
    3,
    4,
    Transport::Mqtt,
    [1; 6],
    BROADCAST_ADDR,
    MsgKind::Text,
    b"still small",
  )
  .unwrap();
  let frames = Fragmenter::split(msg, Transport::Mqtt, 1).unwrap();
  assert_eq!(frames.len(), 1);
  assert!(frames[0].frag.is_none());
}

// ─── Reassembler ────────────────────────────────────────────────────────

#[test]
fn reassembler_completes_out_of_order() {
  let payload: Vec<u8> = (0..MAX_PAYLOAD).map(|i| i as u8).collect();
  let msg = Fragmenter::prepare(
    9,
    4,
    Transport::Lora,
    [1; 6],
    [2; 6],
    MsgKind::Telemetry,
    &payload,
  )
  .unwrap();
  let frames = Fragmenter::split(msg, Transport::Lora, 77).unwrap();
  assert!(frames.len() > 1);

  let mut reasm: Reassembler<2, 32> = Reassembler::new();

  // Feed in reverse order to prove index-based reassembly works.
  let n = frames.len();
  let mut collected: Option<Message> = None;
  let mut reversed: Vec<Message> = frames.into_iter().collect();
  reversed.reverse();
  for (i, frame) in reversed.into_iter().enumerate() {
    let is_last = i + 1 == n;
    match reasm.ingest(frame) {
      ReassembleOutcome::Buffered => assert!(!is_last),
      ReassembleOutcome::Complete(m) => {
        assert!(is_last);
        collected = Some(m);
      }
      other => panic!("unexpected outcome: {:?}", other),
    }
  }

  let m = collected.expect("assembly completed");
  assert_eq!(m.payload.as_slice(), payload.as_slice());
  assert!(m.frag.is_none());
  assert_eq!(m.to, [2; 6]);
  assert_eq!(reasm.in_flight(), 0);
}

#[test]
fn reassembler_rejects_duplicate_slice() {
  let payload: Vec<u8> = vec![0xAB; 260];
  let msg = Fragmenter::prepare(
    11,
    4,
    Transport::Lora,
    [1; 6],
    BROADCAST_ADDR,
    MsgKind::Telemetry,
    // shorten to MAX_PAYLOAD so `Message::payload` fits pre-fragmentation
    &payload[..MAX_PAYLOAD],
  )
  .unwrap();
  let frames = Fragmenter::split(msg, Transport::Lora, 55).unwrap();
  let first = frames[0].clone();
  let mut reasm: Reassembler<2, 32> = Reassembler::new();
  assert!(matches!(
    reasm.ingest(first.clone()),
    ReassembleOutcome::Buffered
  ));
  assert!(matches!(
    reasm.ingest(first),
    ReassembleOutcome::Dropped(RejectReason::DuplicateSlice)
  ));
}

// ─── ReceiverTable ─────────────────────────────────────────────────────

#[test]
fn receivers_add_and_lookup() {
  let mut t: ReceiverTable<4> = ReceiverTable::new();
  assert!(t.add([1; 6], Transport::Lora).unwrap());
  assert!(t.add([1; 6], Transport::EspNow).unwrap());
  assert!(!t.add([1; 6], Transport::Lora).unwrap()); // already there
  let row = t.lookup([1; 6]).unwrap();
  let set: Vec<Transport> = row.transports().collect();
  assert!(set.contains(&Transport::Lora));
  assert!(set.contains(&Transport::EspNow));
  assert!(!set.contains(&Transport::Mqtt));
  assert!(t.remove_transport([1; 6], Transport::Lora));
  assert_eq!(
    t.lookup([1; 6]).unwrap().transports().collect::<Vec<_>>(),
    vec![Transport::EspNow]
  );
}

#[test]
fn receivers_table_full() {
  let mut t: ReceiverTable<2> = ReceiverTable::new();
  assert!(t.add([1; 6], Transport::Lora).is_ok());
  assert!(t.add([2; 6], Transport::Lora).is_ok());
  assert!(matches!(
    t.add([3; 6], Transport::Lora),
    Err(RelayError::ReceiverTableFull)
  ));
}

// ─── Relay: planner API ─────────────────────────────────────────────────

fn make_relay() -> Relay<8, 2, 32> {
  let mut r: Relay<8, 2, 32> = Relay::new([0x02, 0, 0, 0, 0, 1], 0x0100_0000);
  r.register_port(Transport::Lora).unwrap();
  r.register_port(Transport::EspNow).unwrap();
  r
}

#[test]
fn plan_send_broadcast_fans_out_all_registered() {
  let relay = make_relay();
  let plan = relay
    .plan_send(MsgKind::Text, b"hi", Destination::Broadcast)
    .unwrap();
  let ports: Vec<Transport> = plan.iter().map(|p| p.transport).collect();
  assert!(ports.contains(&Transport::Lora));
  assert!(ports.contains(&Transport::EspNow));
  assert!(!ports.contains(&Transport::Mqtt));
  for p in &plan {
    assert!(matches!(p.addr, FrameAddr::Broadcast));
    assert_eq!(p.message.to, BROADCAST_ADDR);
  }
}

#[test]
fn plan_send_unicast_uses_only_receiver_transports() {
  let mut relay = make_relay();
  relay.add_receiver([2; 6], Transport::EspNow).unwrap();
  let plan = relay
    .plan_send(MsgKind::Ping, &[], Destination::Unicast([2; 6]))
    .unwrap();
  assert!(plan.iter().all(|p| p.transport == Transport::EspNow));
  assert!(
    plan
      .iter()
      .all(|p| matches!(p.addr, FrameAddr::Unicast(a) if a == [2; 6]))
  );
  assert!(plan.iter().all(|p| p.message.to == [2; 6]));
}

#[test]
fn plan_send_unicast_unknown_receiver_errors() {
  let relay = make_relay();
  let err = relay
    .plan_send(MsgKind::Ping, &[], Destination::Unicast([9; 6]))
    .unwrap_err();
  assert_eq!(err, RelayError::UnknownReceiver);
}

#[test]
fn plan_forward_zero_ttl_returns_empty() {
  let relay = make_relay();
  let mut msg = Message::new(1, Transport::Lora, [1; 6], MsgKind::Text, b"x").unwrap();
  msg.ttl = 0;
  let plan = relay.plan_forward(msg, Transport::EspNow).unwrap();
  assert!(plan.is_empty());
}

#[test]
fn plan_forward_fragments_when_needed() {
  let relay = make_relay();
  let payload: Vec<u8> = (0..MAX_PAYLOAD).map(|i| i as u8).collect();
  let msg = Message::unicast(
    7,
    Transport::Mqtt,
    [1; 6],
    [2; 6],
    MsgKind::Telemetry,
    &payload,
  )
  .unwrap();
  let plan = relay.plan_forward(msg, Transport::Lora).unwrap();
  assert!(plan.len() > 1);
  assert!(plan.iter().all(|p| p.transport == Transport::Lora));
  assert!(
    plan
      .iter()
      .all(|p| matches!(p.addr, FrameAddr::Unicast(a) if a == [2; 6]))
  );
}

// ─── Relay: ingest ──────────────────────────────────────────────────────

#[test]
fn ingest_deduplicates() {
  let mut relay = make_relay();
  let msg = Message::text(1, Transport::Lora, [9; 6], "yo").unwrap();
  let mut buf = [0u8; relaystar_proto::MAX_FRAME];
  let encoded = msg.encode(&mut buf).unwrap();

  let out = relay.ingest(Transport::Lora, encoded).unwrap();
  assert!(matches!(out, IngestOutcome::Complete(_)));
  let out = relay.ingest(Transport::Lora, encoded).unwrap();
  assert_eq!(out, IngestOutcome::Duplicate);
}

#[test]
fn ingest_learns_receivers() {
  let mut relay = make_relay();
  let msg = Message::unicast(
    2,
    Transport::EspNow,
    [0xAA; 6],
    [0x02, 0, 0, 0, 0, 1],
    MsgKind::Text,
    b"hi",
  )
  .unwrap();
  let mut buf = [0u8; relaystar_proto::MAX_FRAME];
  let encoded = msg.encode(&mut buf).unwrap();

  let _ = relay.ingest(Transport::EspNow, encoded).unwrap();
  let row = relay.receivers().lookup([0xAA; 6]).expect("learned");
  assert!(row.reachable_via(Transport::EspNow));
}

#[test]
fn ingest_not_for_me() {
  let mut relay = make_relay();
  let msg = Message::unicast(
    3,
    Transport::Lora,
    [0xAA; 6],
    [0xBB; 6],
    MsgKind::Text,
    b"not for me",
  )
  .unwrap();
  let mut buf = [0u8; relaystar_proto::MAX_FRAME];
  let encoded = msg.encode(&mut buf).unwrap();
  let out = relay.ingest(Transport::Lora, encoded).unwrap();
  assert!(matches!(out, IngestOutcome::NotForMe(_)));
}

#[test]
fn ingest_reassembles_fragments() {
  let mut relay = make_relay();
  let payload: Vec<u8> = (0..MAX_PAYLOAD).map(|i| (i * 3) as u8).collect();
  let base = Message::unicast(
    42,
    Transport::Lora,
    [0xAA; 6],
    [0x02, 0, 0, 0, 0, 1],
    MsgKind::Telemetry,
    &payload,
  )
  .unwrap();
  let frames = Fragmenter::split(base, Transport::Lora, 42).unwrap();
  assert!(frames.len() > 1);

  let mut assembled: Option<Message> = None;
  for (i, f) in frames.iter().enumerate() {
    let mut buf = [0u8; relaystar_proto::MAX_FRAME];
    let encoded = f.encode(&mut buf).unwrap();
    let out = relay.ingest(Transport::Lora, encoded).unwrap();
    let is_last = i + 1 == frames.len();
    match out {
      IngestOutcome::Buffered => assert!(!is_last),
      IngestOutcome::Complete(m) => {
        assert!(is_last);
        assembled = Some(m);
      }
      other => panic!("unexpected: {:?}", other),
    }
  }
  assert_eq!(assembled.unwrap().payload.as_slice(), payload.as_slice());
}
