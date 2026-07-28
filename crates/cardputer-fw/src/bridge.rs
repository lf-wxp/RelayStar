//! Central message-routing fabric for the Cardputer terminal.
//!
//! Every transport RX path funnels decoded [`Message`]s into [`INBOUND`] (after
//! stamping the *arrival* transport into `msg.origin`). Locally-composed
//! messages (from the keyboard) go into [`LOCAL_OUT`]. The [`bridge_loop`]
//! de-duplicates by message id and fans messages out to the per-transport TX
//! channels ([`LORA_OUT`], [`MQTT_OUT`], [`ESPNOW_OUT`]) while decrementing TTL,
//! never echoing a message back onto the transport it arrived on.

use core::sync::atomic::{AtomicU32, Ordering};

use embassy_futures::select::{select, Either};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use esp_println::println;
use relaystar_proto::{Message, SeenCache, Transport};

/// This terminal's synthetic 6-byte address (not a real MAC).
pub const CARD_ADDR: [u8; 6] = [0x02, 0x00, 0x00, 0x00, 0x00, 0x02];
/// Message-id base so the Cardputer's ids don't collide with the LoRa node's.
pub const CARD_ID_BASE: u32 = 0x0200_0000;

static ID_CTR: AtomicU32 = AtomicU32::new(0);

/// Allocate a fresh, monotonically-increasing message id for this terminal.
pub fn next_id() -> u32 {
  CARD_ID_BASE.wrapping_add(ID_CTR.fetch_add(1, Ordering::Relaxed))
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
    // Truncate to fit.
    for c in body.chars().take(47) {
      let _ = text.push(c);
    }
    DisplayLine {
      origin: msg.origin,
      text,
    }
  }
}

type MsgChannel<const N: usize> = Channel<CriticalSectionRawMutex, Message, N>;

/// Decoded inbound messages from every transport (origin = arrival transport).
pub static INBOUND: MsgChannel<8> = Channel::new();
/// Locally-composed messages destined for *all* transports.
pub static LOCAL_OUT: MsgChannel<4> = Channel::new();
/// Outbound queue for the LoRa transmitter.
pub static LORA_OUT: MsgChannel<8> = Channel::new();
/// Outbound queue for the MQTT publisher.
pub static MQTT_OUT: MsgChannel<8> = Channel::new();
/// Outbound queue for the ESP-NOW sender.
pub static ESPNOW_OUT: MsgChannel<8> = Channel::new();
/// Feed of lines to render on the display.
pub static UI_IN: Channel<CriticalSectionRawMutex, DisplayLine, 8> = Channel::new();

fn enqueue(target: Transport, msg: Message) {
  let result = match target {
    Transport::Lora => LORA_OUT.try_send(msg),
    Transport::Mqtt => MQTT_OUT.try_send(msg),
    Transport::EspNow => ESPNOW_OUT.try_send(msg),
  };
  if result.is_err() {
    println!(
      "bridge: {} TX queue full, dropping message",
      target.as_str()
    );
  }
}

/// The relay engine. Runs forever.
pub async fn bridge_loop() {
  let mut seen: SeenCache<64> = SeenCache::new();
  println!("bridge: relay engine started");

  loop {
    match select(INBOUND.receive(), LOCAL_OUT.receive()).await {
      // Message arrived from a transport (origin already stamped).
      Either::First(msg) => {
        if !seen.check_and_insert(msg.id) {
          // Already seen this id -> loop prevention, drop.
          continue;
        }

        // Surface it on the display.
        let _ = UI_IN.try_send(DisplayLine::from_message(&msg));

        // Relay to every *other* transport, decrementing TTL.
        if let Some(relayed) = msg.prepared_for_relay() {
          for target in Transport::ALL {
            if target == msg.origin {
              continue;
            }
            enqueue(target, relayed.clone());
          }
        }
      }

      // Locally-composed message -> send everywhere.
      Either::Second(msg) => {
        seen.check_and_insert(msg.id);
        let _ = UI_IN.try_send(DisplayLine::from_message(&msg));
        for target in Transport::ALL {
          enqueue(target, msg.clone());
        }
      }
    }
  }
}
