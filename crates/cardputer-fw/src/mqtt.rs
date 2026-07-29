//! MQTT bridge for the Cardputer terminal.
//!
//! This module owns the TCP socket to the broker and hands it to the crate-
//! provided [`Mqtt311Client`] for all protocol work. Local responsibilities
//! shrink to:
//!
//! 1. Reading `BROKER_IP` / `BROKER_PORT` from build-time env.
//! 2. Reconnect logic (TCP + MQTT CONNECT + SUBSCRIBE).
//! 3. Bridging between the broker and the mesh:
//!    - Inbound MQTT PUBLISH → wrap as [`Message`] → push into
//!      [`INBOUND_RAW`](crate::bridge::INBOUND_RAW).
//!    - Outbound [`Message`] from [`MQTT_OUT`](crate::bridge::MQTT_OUT) →
//!      publish as text on [`TOPIC_DOWNLINK`].
//!
//! Topic convention (avoids self-echo loops through the broker):
//! - The terminal SUBSCRIBES to [`TOPIC_UPLINK`] (MQTT world → mesh).
//! - The terminal PUBLISHES to [`TOPIC_DOWNLINK`] (mesh → MQTT world).

use embassy_futures::select::{Either3, select3};
use embassy_net::tcp::TcpSocket;
use embassy_net::{IpAddress, IpEndpoint, Ipv4Address, Stack};
use embassy_time::{Duration, Ticker, Timer};
use esp_println::println;

use crate::bridge::{self, INBOUND_RAW, MQTT_OUT, RawFrame};
use relaystar_proto::{MAX_FRAME, Message, Transport};
use relaystar_relay::ports::mqtt::{Mqtt311Client, PacketId};

/// Injected into the mesh from the MQTT world.
pub const TOPIC_UPLINK: &str = "relaystar/uplink";
/// Emitted from the mesh out to the MQTT world.
pub const TOPIC_DOWNLINK: &str = "relaystar/downlink";

/// MQTT client identifier for this terminal.
const CLIENT_ID: &str = "relaystar-card";
/// Keepalive interval sent in CONNECT and honoured by [`Mqtt311Client::ping`].
const KEEPALIVE_SECS: u16 = 30;

fn parse_ipv4(s: &str) -> Option<Ipv4Address> {
  let mut parts = [0u8; 4];
  let mut i = 0;
  for octet in s.split('.') {
    if i >= 4 {
      return None;
    }
    parts[i] = octet.parse::<u8>().ok()?;
    i += 1;
  }
  if i == 4 {
    Some(Ipv4Address::new(parts[0], parts[1], parts[2], parts[3]))
  } else {
    None
  }
}

/// Handle a broker-supplied uplink publish: wrap the text in a broadcast
/// [`Message`], encode it, and push it through the same [`INBOUND_RAW`]
/// pipeline the other transports use so the relay engine sees uniform
/// inputs (dedup, learning, forwarding).
async fn handle_uplink_text(text: &str) {
  let id = bridge::next_id().await;
  let msg = match Message::text(id, Transport::Mqtt, bridge::CARD_ADDR, text) {
    Ok(m) => m,
    Err(e) => {
      println!("MQTT uplink message build failed: {:?}", e);
      return;
    }
  };
  let mut buf = [0u8; MAX_FRAME];
  let encoded = match msg.encode(&mut buf) {
    Ok(e) => e,
    Err(e) => {
      println!("MQTT uplink encode failed: {:?}", e);
      return;
    }
  };
  let mut bytes: heapless::Vec<u8, MAX_FRAME> = heapless::Vec::new();
  if bytes.extend_from_slice(encoded).is_err() {
    println!("MQTT uplink frame exceeds MAX_FRAME");
    return;
  }
  INBOUND_RAW
    .send(RawFrame {
      source: Transport::Mqtt,
      bytes,
    })
    .await;
  println!("MQTT uplink -> mesh: \"{}\"", text);
}

/// Owns the MQTT link: connects, (re)subscribes, and shuttles messages
/// between the broker and the bridge. Runs forever, reconnecting on
/// failure. Protocol details are delegated to
/// [`Mqtt311Client`](relaystar_relay::ports::mqtt::Mqtt311Client).
pub async fn mqtt_loop(stack: Stack<'_>) {
  let ip = match parse_ipv4(env!("BROKER_IP")) {
    Some(ip) => ip,
    None => {
      println!("MQTT: invalid BROKER_IP env, MQTT disabled");
      return;
    }
  };
  let port: u16 = env!("BROKER_PORT").parse().unwrap_or(1883);
  let endpoint = IpEndpoint::new(IpAddress::Ipv4(ip), port);

  loop {
    stack.wait_config_up().await;

    let mut rx_buffer = [0u8; 1024];
    let mut tx_buffer = [0u8; 1024];
    let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
    socket.set_timeout(Some(Duration::from_secs(60)));

    println!("MQTT: connecting to {}:{}", env!("BROKER_IP"), port);
    if socket.connect(endpoint).await.is_err() {
      println!("MQTT: TCP connect failed, retrying");
      Timer::after(Duration::from_secs(5)).await;
      continue;
    }

    // Whether the *session* below aborted before entering the steady-state
    // event loop. When true we sleep 5s to avoid tight reconnect loops.
    // (The steady-state event loop, in contrast, breaks out immediately
    // and reconnects without extra backoff since the socket itself is
    // likely already stalled.)
    let mut needs_backoff = false;

    // Scope the borrow of `socket` into the Mqtt311Client. Everything the
    // MQTT protocol touches lives in this block; when it ends, `mqtt` is
    // dropped and `socket` becomes free again — no explicit `drop(mqtt)`
    // needed (which would only trigger `clippy::drop_non_drop` because
    // the client does not own any Drop resources of its own).
    {
      let mut mqtt = Mqtt311Client::new(&mut socket, CLIENT_ID, KEEPALIVE_SECS);

      // Handshake. Any failure aborts the session and requests a backoff
      // via the flag above; the borrow ends at the block's `}`.
      'session: {
        if let Err(e) = mqtt.connect().await {
          println!("MQTT: CONNECT failed: {}", e);
          needs_backoff = true;
          break 'session;
        }
        if let Err(e) = mqtt.subscribe(TOPIC_UPLINK, PacketId(1)).await {
          println!("MQTT: SUBSCRIBE failed: {}", e);
          needs_backoff = true;
          break 'session;
        }
        println!("MQTT: connected and subscribed to {}", TOPIC_UPLINK);

        let mut pkt_buf = [0u8; 512];
        let mut ping_ticker = Ticker::every(Duration::from_secs(KEEPALIVE_SECS as u64));

        // Steady-state event loop: any error breaks out and triggers an
        // immediate reconnect (no backoff — the failure itself is the
        // signal that something's already stalled).
        loop {
          match select3(
            mqtt.read_publish(&mut pkt_buf),
            MQTT_OUT.receive(),
            ping_ticker.next(),
          )
          .await
          {
            // Inbound packet from broker.
            Either3::First(res) => match res {
              Ok(Some(pubmsg)) => {
                if pubmsg.topic == TOPIC_UPLINK {
                  let text = core::str::from_utf8(pubmsg.payload).unwrap_or("<binary>");
                  handle_uplink_text(text).await;
                }
              }
              Ok(None) => {
                // Non-PUBLISH packet (e.g. PINGRESP) — ignore.
              }
              Err(e) => {
                println!("MQTT: read error {}, reconnecting", e);
                break;
              }
            },

            // Outbound frame from the bridge → publish downlink.
            //
            // Body: publish the raw text if it decodes as UTF-8; otherwise
            // emit "<binary>". `FrameAddr` is unused because MQTT's topic
            // scheme is fixed for the legacy uplink/downlink convention.
            // If you need per-recipient topics, register the relay-native
            // `MqttPort` (from `relaystar-relay`), which encodes the
            // address into the topic.
            Either3::Second((_addr, msg)) => {
              let text = msg.as_text().unwrap_or("<binary>");
              if let Err(e) = mqtt.publish(TOPIC_DOWNLINK, text.as_bytes()).await {
                println!("MQTT: publish failed {}, reconnecting", e);
                break;
              }
            }

            // Keepalive.
            Either3::Third(_) => {
              if let Err(e) = mqtt.ping().await {
                println!("MQTT: ping failed {}, reconnecting", e);
                break;
              }
            }
          }
        }
      }
      // `mqtt` (and thus its `&mut socket` borrow) is released here.
    }

    if needs_backoff {
      Timer::after(Duration::from_secs(5)).await;
    }
  }
}
