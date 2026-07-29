//! Central message-routing fabric for the Cardputer terminal.
//!
//! Built on top of [`relaystar_relay::Relay`]: fragmentation, reassembly,
//! deduplication, receiver learning, and unicast/broadcast fan-out are all
//! handled by the engine. This module just wires it into embassy channels.
//!
//! ## Data flow
//!
//! ```text
//!   ┌──── LoRa RX ────┐   ┌──── ESP-NOW RX ────┐   ┌──── MQTT RX ────┐
//!   │  raw bytes      │   │  raw bytes         │   │  raw bytes      │
//!   └───────┬─────────┘   └─────────┬──────────┘   └─────────┬───────┘
//!           │                       │                        │
//!           ▼                       ▼                        ▼
//!         RELAY.ingest(source, bytes)   ← locked briefly, sync
//!           │
//!           ├─ Complete(msg)  → UI_IN + (if broadcast) relay forward to other transports
//!           ├─ NotForMe(msg)  → relay forward to every other transport
//!           ├─ Buffered / Duplicate / Dropped → drop
//!           │
//!    forward → RELAY.plan_forward(msg, target) → PlannedFrame
//!                                                       │
//!                                                       ▼
//!                                       LORA_OUT / ESPNOW_OUT / MQTT_OUT
//!                                                       │
//!                                                       ▼
//!                                       transport TX loop encodes + sends
//! ```
//!
//! Locally-composed keyboard input goes through `send_local`, which is a thin
//! wrapper around [`Relay::plan_send`].

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use esp_println::println;
use relaystar_proto::{Message, MsgKind, Transport};
use relaystar_relay::{Destination, FrameAddr, IngestOutcome, PlannedFrame, Relay, RelayError};

/// This terminal's synthetic 6-byte address (not a real MAC).
pub const CARD_ADDR: [u8; 6] = [0x02, 0x00, 0x00, 0x00, 0x00, 0x02];
/// Message-id base so the Cardputer's ids don't collide with the LoRa node's.
pub const CARD_ID_BASE: u32 = 0x0200_0000;

// Relay dimensions (compile-time, no heap):
//   NR = 16 tracked peers
//   NA = 4  concurrent reassembly groups
//   NF = 32 fragments per group
type CardRelay = Relay<16, 4, 32>;

/// The one relay engine instance. Guarded by an embassy `Mutex` so that any
/// task can call the *sync* API (`ingest`, `plan_send`, `plan_forward`,
/// `add_receiver`, ...). We never hold the lock across an `.await`.
pub static RELAY: Mutex<CriticalSectionRawMutex, CardRelay> =
  Mutex::new(CardRelay::new(CARD_ADDR, CARD_ID_BASE));

/// Allocate a fresh, monotonically-increasing message id.
///
/// Kept for compatibility with the older API; internally delegates to the
/// relay's own id allocator.
pub async fn next_id() -> u32 {
  RELAY.lock().await.next_id()
}

/// A line to render on the display log.
#[derive(Clone)]
pub struct DisplayLine {
  pub origin: Transport,
  pub text: heapless::String<48>,
}

impl DisplayLine {
  pub fn from_message(msg: &Message) -> Self {
    let mut text: heapless::String<48> = heapless::String::new();
    let body = msg.as_text().unwrap_or("<binary>");
    for c in body.chars().take(47) {
      let _ = text.push(c);
    }
    DisplayLine {
      origin: msg.origin,
      text,
    }
  }
}

/// One frame planned for emission on a specific transport.
///
/// The three per-transport TX channels carry these instead of raw `Message`s
/// so that the sending task always knows *how* to address the frame
/// (broadcast vs. unicast MAC), even for fragmented sends.
pub type OutFrame = (FrameAddr, Message);

type OutChannel<const N: usize> = Channel<CriticalSectionRawMutex, OutFrame, N>;
type RawChannel<const N: usize> = Channel<CriticalSectionRawMutex, RawFrame, N>;

/// A newly-received frame from a transport, still in raw wire form. Fixed
/// capacity so we don't allocate.
#[derive(Clone)]
pub struct RawFrame {
  pub source: Transport,
  pub bytes: heapless::Vec<u8, { relaystar_proto::MAX_FRAME }>,
}

/// Raw inbound frames from every transport (source = arrival transport).
pub static INBOUND_RAW: RawChannel<8> = Channel::new();
/// Outbound queue for the LoRa transmitter.
pub static LORA_OUT: OutChannel<8> = Channel::new();
/// Outbound queue for the MQTT publisher.
pub static MQTT_OUT: OutChannel<8> = Channel::new();
/// Outbound queue for the ESP-NOW sender.
pub static ESPNOW_OUT: OutChannel<8> = Channel::new();
/// Feed of lines to render on the display.
pub static UI_IN: Channel<CriticalSectionRawMutex, DisplayLine, 8> = Channel::new();

fn enqueue(plan: PlannedFrame) {
  let PlannedFrame {
    transport,
    addr,
    message,
  } = plan;
  let result = match transport {
    Transport::Lora => LORA_OUT.try_send((addr, message)),
    Transport::Mqtt => MQTT_OUT.try_send((addr, message)),
    Transport::EspNow => ESPNOW_OUT.try_send((addr, message)),
  };
  if result.is_err() {
    println!(
      "bridge: {} TX queue full, dropping frame",
      transport.as_str()
    );
  }
}

/// Push a locally-composed broadcast text message. Called from the UI task.
pub async fn send_local_text(text: &str) {
  let plan = {
    let relay = RELAY.lock().await;
    relay.plan_send(MsgKind::Text, text.as_bytes(), Destination::Broadcast)
  };
  let plan = match plan {
    Ok(p) => p,
    Err(RelayError::NoPortsRegistered) => {
      println!("bridge: no ports registered, cannot send");
      return;
    }
    Err(e) => {
      println!("bridge: plan_send failed: {}", e);
      return;
    }
  };

  // Also surface locally so the user sees their own text.
  if let Some(first) = plan.first() {
    let _ = UI_IN.try_send(DisplayLine::from_message(&first.message));
  }
  for frame in plan {
    enqueue(frame);
  }
}

/// The relay engine driver. Runs forever, translating raw inbound frames into
/// display updates plus forward plans.
pub async fn bridge_loop() {
  println!("bridge: relay engine started");

  // Advertise the three transports so `plan_send(Destination::Broadcast)`
  // fans out over all of them.
  {
    let mut relay = RELAY.lock().await;
    let _ = relay.register_port(Transport::Lora);
    let _ = relay.register_port(Transport::EspNow);
    let _ = relay.register_port(Transport::Mqtt);
  }

  loop {
    let raw = INBOUND_RAW.receive().await;

    // Ingest under the lock; unlock before doing any UI/channel work that
    // might contend.
    let outcome = {
      let mut relay = RELAY.lock().await;
      match relay.ingest(raw.source, &raw.bytes) {
        Ok(o) => o,
        Err(e) => {
          println!("bridge: ingest error from {}: {}", raw.source.as_str(), e);
          continue;
        }
      }
    };

    match outcome {
      IngestOutcome::Complete(msg) => {
        // Deliver locally.
        let _ = UI_IN.try_send(DisplayLine::from_message(&msg));
        // Broadcasts should also be relayed to the *other* transports so a
        // LoRa broadcast reaches MQTT / ESP-NOW peers.
        if msg.is_broadcast() {
          fanout_forward(msg, raw.source).await;
        }
      }
      IngestOutcome::NotForMe(msg) => {
        // Pure relay path: forward to every transport except the source.
        fanout_forward(msg, raw.source).await;
      }
      IngestOutcome::Buffered | IngestOutcome::Duplicate => {}
      IngestOutcome::Dropped(reason) => {
        println!("bridge: reassembly dropped: {:?}", reason);
      }
      // `IngestOutcome` is #[non_exhaustive]; ignore future variants.
      _ => {}
    }
  }
}

/// Forward `msg` to every transport except `skip`. Held-lock windows are
/// short and never span an `.await`.
async fn fanout_forward(msg: Message, skip: Transport) {
  for target in Transport::ALL {
    if target == skip {
      continue;
    }
    let plan = {
      let relay = RELAY.lock().await;
      relay.plan_forward(msg.clone(), target)
    };
    match plan {
      Ok(frames) => {
        for f in frames {
          enqueue(f);
        }
      }
      Err(e) => println!("bridge: plan_forward to {} failed: {}", target.as_str(), e),
    }
  }
}
