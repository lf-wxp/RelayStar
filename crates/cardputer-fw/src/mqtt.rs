//! Minimal MQTT 3.1.1 client (QoS 0) over an embassy-net `TcpSocket`.
//!
//! Rather than depend on a large MQTT crate (whose API churns and whose
//! `embedded-io-async` version must be matched exactly), RelayStar speaks just
//! enough of the protocol to CONNECT, SUBSCRIBE, PUBLISH and parse inbound
//! PUBLISH packets. This is intentionally small and easy to audit.
//!
//! Topic convention (avoids self-echo loops through the broker):
//! - The terminal SUBSCRIBES to [`TOPIC_UPLINK`] (MQTT world -> mesh).
//! - The terminal PUBLISHES to [`TOPIC_DOWNLINK`] (mesh -> MQTT world).

use embassy_futures::select::{select3, Either3};
use embassy_net::tcp::TcpSocket;
use embassy_net::{IpAddress, IpEndpoint, Ipv4Address, Stack};
use embassy_time::{Duration, Ticker, Timer};
use embedded_io_async::{Read, Write};
use esp_println::println;

use crate::bridge::{self, MQTT_OUT};
use relaystar_proto::{Message, Transport};

/// Injected into the mesh from the MQTT world.
pub const TOPIC_UPLINK: &str = "relaystar/uplink";
/// Emitted from the mesh out to the MQTT world.
pub const TOPIC_DOWNLINK: &str = "relaystar/downlink";

const CLIENT_ID: &str = "relaystar-card";
const KEEPALIVE_SECS: u16 = 30;

#[derive(Debug)]
pub enum MqttError {
  Io,
  Protocol,
  Rejected,
  TooLarge,
}

fn write_remaining_len<const N: usize>(v: &mut heapless::Vec<u8, N>, mut len: usize) {
  loop {
    let mut byte = (len % 128) as u8;
    len /= 128;
    if len > 0 {
      byte |= 0x80;
    }
    let _ = v.push(byte);
    if len == 0 {
      break;
    }
  }
}

async fn send_frame<S: Write>(
  socket: &mut S,
  first_byte: u8,
  variable: &[u8],
) -> Result<(), MqttError> {
  let mut frame: heapless::Vec<u8, 320> = heapless::Vec::new();
  if frame.push(first_byte).is_err() {
    return Err(MqttError::TooLarge);
  }
  write_remaining_len(&mut frame, variable.len());
  if frame.extend_from_slice(variable).is_err() {
    return Err(MqttError::TooLarge);
  }
  socket.write_all(&frame).await.map_err(|_| MqttError::Io)
}

/// Read one full MQTT packet into `buf`, returning `(first_byte, body)`.
async fn read_packet<'a, S: Read>(
  socket: &mut S,
  buf: &'a mut [u8],
) -> Result<(u8, &'a [u8]), MqttError> {
  let mut header = [0u8; 1];
  socket
    .read_exact(&mut header)
    .await
    .map_err(|_| MqttError::Io)?;

  let mut multiplier = 1usize;
  let mut value = 0usize;
  loop {
    let mut b = [0u8; 1];
    socket.read_exact(&mut b).await.map_err(|_| MqttError::Io)?;
    value += (b[0] & 0x7f) as usize * multiplier;
    if b[0] & 0x80 == 0 {
      break;
    }
    multiplier *= 128;
    if multiplier > 128 * 128 * 128 {
      return Err(MqttError::Protocol);
    }
  }

  if value > buf.len() {
    return Err(MqttError::TooLarge);
  }
  socket
    .read_exact(&mut buf[..value])
    .await
    .map_err(|_| MqttError::Io)?;
  Ok((header[0], &buf[..value]))
}

async fn connect<S: Read + Write>(socket: &mut S) -> Result<(), MqttError> {
  let mut vh: heapless::Vec<u8, 64> = heapless::Vec::new();
  // Protocol name + level 4 (3.1.1) + clean-session flag.
  let _ = vh.extend_from_slice(&[0, 4, b'M', b'Q', b'T', b'T', 0x04, 0x02]);
  let _ = vh.extend_from_slice(&KEEPALIVE_SECS.to_be_bytes());
  let cid = CLIENT_ID.as_bytes();
  let _ = vh.extend_from_slice(&(cid.len() as u16).to_be_bytes());
  let _ = vh.extend_from_slice(cid);

  send_frame(socket, 0x10, &vh).await?;

  let mut buf = [0u8; 8];
  let (first, body) = read_packet(socket, &mut buf).await?;
  if first >> 4 != 0x02 || body.len() < 2 {
    return Err(MqttError::Protocol);
  }
  if body[1] != 0x00 {
    return Err(MqttError::Rejected);
  }
  Ok(())
}

async fn subscribe<S: Read + Write>(
  socket: &mut S,
  topic: &str,
  packet_id: u16,
) -> Result<(), MqttError> {
  let mut vh: heapless::Vec<u8, 128> = heapless::Vec::new();
  let _ = vh.extend_from_slice(&packet_id.to_be_bytes());
  let _ = vh.extend_from_slice(&(topic.len() as u16).to_be_bytes());
  let _ = vh.extend_from_slice(topic.as_bytes());
  let _ = vh.push(0x00); // requested QoS 0
  send_frame(socket, 0x82, &vh).await?;

  let mut buf = [0u8; 16];
  let (first, _body) = read_packet(socket, &mut buf).await?;
  if first >> 4 != 0x09 {
    return Err(MqttError::Protocol);
  }
  Ok(())
}

async fn publish<S: Write>(socket: &mut S, topic: &str, payload: &[u8]) -> Result<(), MqttError> {
  let mut vh: heapless::Vec<u8, 300> = heapless::Vec::new();
  let _ = vh.extend_from_slice(&(topic.len() as u16).to_be_bytes());
  let _ = vh.extend_from_slice(topic.as_bytes());
  if vh.extend_from_slice(payload).is_err() {
    return Err(MqttError::TooLarge);
  }
  send_frame(socket, 0x30, &vh).await
}

async fn ping<S: Write>(socket: &mut S) -> Result<(), MqttError> {
  socket
    .write_all(&[0xC0, 0x00])
    .await
    .map_err(|_| MqttError::Io)
}

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

/// Handle an inbound PUBLISH body: extract topic + payload and, if it is on the
/// uplink topic, inject it into the mesh.
fn handle_publish(body: &[u8]) {
  if body.len() < 2 {
    return;
  }
  let topic_len = u16::from_be_bytes([body[0], body[1]]) as usize;
  if body.len() < 2 + topic_len {
    return;
  }
  let topic = core::str::from_utf8(&body[2..2 + topic_len]).unwrap_or("");
  let payload = &body[2 + topic_len..];

  if topic == TOPIC_UPLINK {
    let text = core::str::from_utf8(payload).unwrap_or("<binary>");
    match Message::text(bridge::next_id(), Transport::Mqtt, bridge::CARD_ADDR, text) {
      Ok(msg) => {
        let _ = bridge::INBOUND.try_send(msg);
        println!("MQTT uplink -> mesh: \"{}\"", text);
      }
      Err(e) => println!("MQTT uplink message build failed: {:?}", e),
    }
  }
}

/// Owns the MQTT link: connects, (re)subscribes, and shuttles messages between
/// the broker and the bridge. Runs forever, reconnecting on failure.
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
    if let Err(e) = connect(&mut socket).await {
      println!("MQTT: CONNECT failed: {:?}", e);
      Timer::after(Duration::from_secs(5)).await;
      continue;
    }
    if let Err(e) = subscribe(&mut socket, TOPIC_UPLINK, 1).await {
      println!("MQTT: SUBSCRIBE failed: {:?}", e);
      Timer::after(Duration::from_secs(5)).await;
      continue;
    }
    println!("MQTT: connected and subscribed to {}", TOPIC_UPLINK);

    let mut pkt_buf = [0u8; 512];
    let mut ping_ticker = Ticker::every(Duration::from_secs(KEEPALIVE_SECS as u64));

    loop {
      match select3(
        read_packet(&mut socket, &mut pkt_buf),
        MQTT_OUT.receive(),
        ping_ticker.next(),
      )
      .await
      {
        // Inbound packet from broker.
        Either3::First(res) => match res {
          Ok((first, body)) => {
            let ptype = first >> 4;
            if ptype == 0x03 {
              handle_publish(body);
            }
            // 0x0D = PINGRESP, others ignored.
          }
          Err(e) => {
            println!("MQTT: read error {:?}, reconnecting", e);
            break;
          }
        },

        // Outbound message from the bridge -> publish downlink.
        Either3::Second(msg) => {
          let text = msg.as_text().unwrap_or("<binary>");
          if let Err(e) = publish(&mut socket, TOPIC_DOWNLINK, text.as_bytes()).await {
            println!("MQTT: publish failed {:?}, reconnecting", e);
            break;
          }
        }

        // Keepalive.
        Either3::Third(_) => {
          if ping(&mut socket).await.is_err() {
            println!("MQTT: ping failed, reconnecting");
            break;
          }
        }
      }
    }
  }
}
