//! MQTT transport adapter.
//!
//! MQTT has no built-in "unicast MAC" concept, so the adapter encodes the
//! addressing via topic naming:
//!
//! - **Broadcast**: publish to [`Topic::BROADCAST`] (`"relaystar/b"`).
//! - **Unicast to `AA:BB:CC:DD:EE:FF`**: publish to
//!   [`Topic::unicast`](Topic::unicast_for), which yields
//!   `"relaystar/u/aabbccddeeff"`.
//!
//! Every node subscribes to `relaystar/b/#` and to
//! `relaystar/u/{own-hex-addr}` so it receives exactly what is addressed to
//! it or the broadcast group.
//!
//! ## Two integration paths
//!
//! ### 1. Bring your own client — implement [`MqttClient`]
//!
//! Wrap any MQTT client (`rumqttc` on a host, `esp-mqtt-client`, a custom
//! embedded implementation) by implementing [`MqttClient`], then hand it to
//! [`MqttPort::new`].
//!
//! ### 2. Use the bundled [`Mqtt311Client`] (feature = `"mqtt"`)
//!
//! When enabled, the crate ships a tiny hand-rolled MQTT 3.1.1 QoS-0 client
//! on top of [`embedded_io_async::Read`] + [`embedded_io_async::Write`]. It
//! is intentionally minimal (CONNECT / SUBSCRIBE / PUBLISH / PINGREQ /
//! incoming PUBLISH) and works with the `rumqttd` v4.1 listener the broker
//! crate configures on port 1883.

use crate::error::PortError;
use crate::port::{FrameAddr, TransportPort};
use relaystar_proto::Transport;

/// Topic-naming helper.
///
/// The topic scheme is stable and forms part of the wire protocol: don't
/// change it without updating the broker's subscription rules in lockstep.
pub struct Topic;

impl Topic {
  /// Prefix used for broadcast publications and subscriptions.
  pub const BROADCAST: &'static str = "relaystar/b";
  /// Prefix used for unicast publications; append `/{hex(addr)}` to complete.
  pub const UNICAST_PREFIX: &'static str = "relaystar/u/";

  /// Build the unicast topic for `addr` into `buf`, returning the populated
  /// slice.
  ///
  /// The topic is `relaystar/u/aabbccddeeff` (12 hex chars, lowercase).
  /// `buf.len()` must be at least [`Self::UNICAST_TOPIC_LEN`].
  ///
  /// # Errors
  /// Returns [`PortError::FrameTooLarge`] when the buffer is too small.
  pub fn unicast_for(addr: [u8; 6], buf: &mut [u8]) -> Result<&str, PortError> {
    if buf.len() < Self::UNICAST_TOPIC_LEN {
      return Err(PortError::FrameTooLarge);
    }
    let prefix = Self::UNICAST_PREFIX.as_bytes();
    buf[..prefix.len()].copy_from_slice(prefix);
    let mut i = prefix.len();
    for byte in addr {
      buf[i] = hex_nibble(byte >> 4);
      buf[i + 1] = hex_nibble(byte & 0x0F);
      i += 2;
    }
    // SAFETY: we only wrote ASCII bytes above.
    Ok(unsafe { core::str::from_utf8_unchecked(&buf[..i]) })
  }

  /// Length of a unicast topic string: `"relaystar/u/"` + 12 hex chars.
  pub const UNICAST_TOPIC_LEN: usize = 12 + 12;
}

const fn hex_nibble(n: u8) -> u8 {
  match n {
    0..=9 => b'0' + n,
    _ => b'a' + (n - 10),
  }
}

/// Hardware-facing trait an MQTT client must implement.
///
/// Kept intentionally minimal: publish only. Subscribing / receiving is
/// driven by whatever client task owns the connection and pushes decoded
/// payloads back through [`crate::Relay::ingest`].
pub trait MqttClient {
  /// Client-specific error. Mapped to [`PortError::Io`].
  type Error: core::fmt::Debug;

  /// Publish `payload` at QoS 0 to `topic`.
  fn publish(
    &mut self,
    topic: &str,
    payload: &[u8],
  ) -> impl core::future::Future<Output = Result<(), Self::Error>>;
}

/// [`TransportPort`] implementation over any [`MqttClient`].
pub struct MqttPort<C: MqttClient> {
  client: C,
}

impl<C: MqttClient> MqttPort<C> {
  /// Wrap an MQTT client.
  pub const fn new(client: C) -> Self {
    MqttPort { client }
  }

  /// Access the inner client (e.g. to send administrative messages).
  pub fn inner(&mut self) -> &mut C {
    &mut self.client
  }

  /// Consume the port and return the inner client.
  pub fn into_inner(self) -> C {
    self.client
  }
}

impl<C: MqttClient> TransportPort for MqttPort<C> {
  fn transport(&self) -> Transport {
    Transport::Mqtt
  }

  async fn send_frame(&mut self, addr: FrameAddr, frame: &[u8]) -> Result<(), PortError> {
    let mut topic_buf = [0u8; Topic::UNICAST_TOPIC_LEN];
    let topic: &str = match addr {
      FrameAddr::Broadcast => Topic::BROADCAST,
      FrameAddr::Unicast(a) => Topic::unicast_for(a, &mut topic_buf)?,
    };
    self
      .client
      .publish(topic, frame)
      .await
      .map_err(|_| PortError::Io)
  }
}

// ──────────────────────────────────────────────────────────────────────────
// Bundled MQTT 3.1.1 QoS-0 client (feature = "mqtt").
// ──────────────────────────────────────────────────────────────────────────

#[cfg(feature = "mqtt")]
pub use mqtt311_impl::{IncomingPublish, Mqtt311Client, Mqtt311Error, PacketId};

#[cfg(feature = "mqtt")]
mod mqtt311_impl {
  //! Minimal MQTT 3.1.1 QoS-0 client over `embedded_io_async::{Read, Write}`.
  //!
  //! Intentionally small so it stays easy to audit; supports exactly:
  //! `CONNECT`, `SUBSCRIBE`, `PUBLISH` (out + in), `PINGREQ`.
  //!
  //! Compatible with the `rumqttd` v4.1 listener on port 1883 (as configured
  //! in `docker/rumqttd.toml`).

  use super::MqttClient;
  use core::fmt;
  use embedded_io_async::{Read, Write};
  use heapless::Vec as HVec;

  /// MQTT packet identifier newtype.
  #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
  #[cfg_attr(docsrs, doc(cfg(feature = "mqtt")))]
  pub struct PacketId(pub u16);

  /// Errors produced by [`Mqtt311Client`].
  #[derive(Debug, Clone, Copy, PartialEq, Eq)]
  #[non_exhaustive]
  #[cfg_attr(docsrs, doc(cfg(feature = "mqtt")))]
  pub enum Mqtt311Error {
    /// Underlying transport I/O failure.
    Io,
    /// Broker returned an unexpected packet or malformed data.
    Protocol,
    /// Broker returned a non-zero CONNACK code.
    Rejected,
    /// Outgoing packet exceeded the client's internal buffer.
    TooLarge,
  }

  impl fmt::Display for Mqtt311Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
      let s = match self {
        Mqtt311Error::Io => "mqtt i/o error",
        Mqtt311Error::Protocol => "mqtt protocol error",
        Mqtt311Error::Rejected => "mqtt connection rejected",
        Mqtt311Error::TooLarge => "mqtt packet too large",
      };
      f.write_str(s)
    }
  }

  #[cfg(feature = "std")]
  impl std::error::Error for Mqtt311Error {}

  /// A decoded inbound `PUBLISH` packet borrowed from the caller's read
  /// buffer.
  #[derive(Debug)]
  #[cfg_attr(docsrs, doc(cfg(feature = "mqtt")))]
  pub struct IncomingPublish<'a> {
    /// Topic the message was published to.
    pub topic: &'a str,
    /// Raw payload bytes.
    pub payload: &'a [u8],
  }

  /// Maximum size of a single outgoing MQTT packet held on the stack.
  ///
  /// Larger publishes will fail with [`Mqtt311Error::TooLarge`]; sized to
  /// comfortably fit the [`super::Topic`] scheme plus a
  /// [`relaystar_proto::MAX_FRAME`]-sized payload.
  const OUT_BUF: usize = 320;

  /// Minimal MQTT 3.1.1 QoS-0 client.
  ///
  /// Owns a bidirectional byte stream (typically an
  /// [`embassy_net::tcp::TcpSocket`](https://docs.embassy.dev/embassy-net/git/default/tcp/struct.TcpSocket.html))
  /// and speaks just enough MQTT to be useful in a relay node.
  ///
  /// # Example
  ///
  /// ```ignore
  /// use relaystar_relay::ports::mqtt::{Mqtt311Client, MqttPort, Topic, PacketId};
  ///
  /// let mut mqtt = Mqtt311Client::new(socket, "relaystar-node", 30);
  /// mqtt.connect().await?;
  /// mqtt.subscribe(Topic::BROADCAST, PacketId(1)).await?;
  ///
  /// // Feed to the relay via MqttPort — it publishes on Topic::BROADCAST or
  /// // Topic::unicast_for(addr) automatically.
  /// let mut port = MqttPort::new(mqtt);
  /// ```
  #[cfg_attr(docsrs, doc(cfg(feature = "mqtt")))]
  pub struct Mqtt311Client<S: Read + Write> {
    socket: S,
    client_id: &'static str,
    keepalive_secs: u16,
  }

  impl<S: Read + Write> Mqtt311Client<S> {
    /// Wrap an already-open bidirectional byte stream. The stream must
    /// already be connected at the TCP layer.
    pub const fn new(socket: S, client_id: &'static str, keepalive_secs: u16) -> Self {
      Mqtt311Client {
        socket,
        client_id,
        keepalive_secs,
      }
    }

    /// Configured keepalive in seconds. Callers should send [`Self::ping`]
    /// at most every `keepalive_secs()` seconds.
    pub const fn keepalive_secs(&self) -> u16 {
      self.keepalive_secs
    }

    /// Borrow the inner transport.
    pub fn inner_mut(&mut self) -> &mut S {
      &mut self.socket
    }

    /// Consume the client and yield the inner transport.
    pub fn into_inner(self) -> S {
      self.socket
    }

    /// Perform the MQTT CONNECT handshake. Sends CONNECT and waits for a
    /// CONNACK with return code 0.
    ///
    /// # Errors
    /// - [`Mqtt311Error::Io`] on socket failure.
    /// - [`Mqtt311Error::Protocol`] if the reply is malformed / wrong type.
    /// - [`Mqtt311Error::Rejected`] if the broker responds with a non-zero
    ///   CONNACK code.
    pub async fn connect(&mut self) -> Result<(), Mqtt311Error> {
      let mut vh: HVec<u8, 64> = HVec::new();
      // Protocol name + level 4 (3.1.1) + clean-session flag.
      vh.extend_from_slice(&[0, 4, b'M', b'Q', b'T', b'T', 0x04, 0x02])
        .map_err(|_| Mqtt311Error::TooLarge)?;
      vh.extend_from_slice(&self.keepalive_secs.to_be_bytes())
        .map_err(|_| Mqtt311Error::TooLarge)?;
      let cid = self.client_id.as_bytes();
      vh.extend_from_slice(&(cid.len() as u16).to_be_bytes())
        .map_err(|_| Mqtt311Error::TooLarge)?;
      vh.extend_from_slice(cid)
        .map_err(|_| Mqtt311Error::TooLarge)?;

      send_frame(&mut self.socket, 0x10, &vh).await?;

      let mut buf = [0u8; 8];
      let (first, body) = read_packet(&mut self.socket, &mut buf).await?;
      if first >> 4 != 0x02 || body.len() < 2 {
        return Err(Mqtt311Error::Protocol);
      }
      if body[1] != 0x00 {
        return Err(Mqtt311Error::Rejected);
      }
      Ok(())
    }

    /// Send a SUBSCRIBE (QoS 0) and wait for the matching SUBACK.
    ///
    /// # Errors
    /// See [`Self::connect`].
    pub async fn subscribe(
      &mut self,
      topic: &str,
      packet_id: PacketId,
    ) -> Result<(), Mqtt311Error> {
      let mut vh: HVec<u8, 128> = HVec::new();
      vh.extend_from_slice(&packet_id.0.to_be_bytes())
        .map_err(|_| Mqtt311Error::TooLarge)?;
      vh.extend_from_slice(&(topic.len() as u16).to_be_bytes())
        .map_err(|_| Mqtt311Error::TooLarge)?;
      vh.extend_from_slice(topic.as_bytes())
        .map_err(|_| Mqtt311Error::TooLarge)?;
      vh.push(0x00).map_err(|_| Mqtt311Error::TooLarge)?; // requested QoS 0
      send_frame(&mut self.socket, 0x82, &vh).await?;

      let mut buf = [0u8; 16];
      let (first, _body) = read_packet(&mut self.socket, &mut buf).await?;
      if first >> 4 != 0x09 {
        return Err(Mqtt311Error::Protocol);
      }
      Ok(())
    }

    /// Publish `payload` at QoS 0 to `topic`.
    ///
    /// # Errors
    /// - [`Mqtt311Error::TooLarge`] if the encoded packet exceeds
    ///   `OUT_BUF` (320 B) — usually only hit for MQTT payloads much larger
    ///   than [`relaystar_proto::MAX_FRAME`].
    /// - [`Mqtt311Error::Io`] on socket failure.
    pub async fn publish(&mut self, topic: &str, payload: &[u8]) -> Result<(), Mqtt311Error> {
      let mut vh: HVec<u8, OUT_BUF> = HVec::new();
      vh.extend_from_slice(&(topic.len() as u16).to_be_bytes())
        .map_err(|_| Mqtt311Error::TooLarge)?;
      vh.extend_from_slice(topic.as_bytes())
        .map_err(|_| Mqtt311Error::TooLarge)?;
      vh.extend_from_slice(payload)
        .map_err(|_| Mqtt311Error::TooLarge)?;
      send_frame(&mut self.socket, 0x30, &vh).await
    }

    /// Send a PINGREQ. Callers should invoke this on their own timer
    /// (`keepalive_secs()` intervals) to keep the broker connection alive.
    ///
    /// # Errors
    /// [`Mqtt311Error::Io`] on socket failure.
    pub async fn ping(&mut self) -> Result<(), Mqtt311Error> {
      self
        .socket
        .write_all(&[0xC0, 0x00])
        .await
        .map_err(|_| Mqtt311Error::Io)
    }

    /// Read one MQTT packet and, if it is a PUBLISH, decode the topic +
    /// payload into a borrow of `buf`. Non-PUBLISH packets (e.g. PINGRESP)
    /// are consumed and reported as `Ok(None)`.
    ///
    /// # Errors
    /// - [`Mqtt311Error::Io`] on socket failure.
    /// - [`Mqtt311Error::Protocol`] on a malformed variable header.
    /// - [`Mqtt311Error::TooLarge`] if the packet exceeds `buf.len()`.
    pub async fn read_publish<'a>(
      &mut self,
      buf: &'a mut [u8],
    ) -> Result<Option<IncomingPublish<'a>>, Mqtt311Error> {
      let (first, body) = read_packet(&mut self.socket, buf).await?;
      // 0x03 << 4 == PUBLISH
      if first >> 4 != 0x03 {
        return Ok(None);
      }
      if body.len() < 2 {
        return Err(Mqtt311Error::Protocol);
      }
      let topic_len = u16::from_be_bytes([body[0], body[1]]) as usize;
      if body.len() < 2 + topic_len {
        return Err(Mqtt311Error::Protocol);
      }
      let topic =
        core::str::from_utf8(&body[2..2 + topic_len]).map_err(|_| Mqtt311Error::Protocol)?;
      let payload = &body[2 + topic_len..];
      Ok(Some(IncomingPublish { topic, payload }))
    }
  }

  impl<S: Read + Write> MqttClient for Mqtt311Client<S> {
    type Error = Mqtt311Error;

    async fn publish(&mut self, topic: &str, payload: &[u8]) -> Result<(), Self::Error> {
      // Delegate to the inherent method (which is generic over S).
      Mqtt311Client::publish(self, topic, payload).await
    }
  }

  // ── Helpers ─────────────────────────────────────────────────────────

  fn write_remaining_len<const N: usize>(
    v: &mut HVec<u8, N>,
    mut len: usize,
  ) -> Result<(), Mqtt311Error> {
    loop {
      let mut byte = (len % 128) as u8;
      len /= 128;
      if len > 0 {
        byte |= 0x80;
      }
      v.push(byte).map_err(|_| Mqtt311Error::TooLarge)?;
      if len == 0 {
        return Ok(());
      }
    }
  }

  async fn send_frame<S: Write>(
    socket: &mut S,
    first_byte: u8,
    variable: &[u8],
  ) -> Result<(), Mqtt311Error> {
    let mut frame: HVec<u8, OUT_BUF> = HVec::new();
    frame.push(first_byte).map_err(|_| Mqtt311Error::TooLarge)?;
    write_remaining_len(&mut frame, variable.len())?;
    frame
      .extend_from_slice(variable)
      .map_err(|_| Mqtt311Error::TooLarge)?;
    socket.write_all(&frame).await.map_err(|_| Mqtt311Error::Io)
  }

  async fn read_packet<'a, S: Read>(
    socket: &mut S,
    buf: &'a mut [u8],
  ) -> Result<(u8, &'a [u8]), Mqtt311Error> {
    let mut header = [0u8; 1];
    socket
      .read_exact(&mut header)
      .await
      .map_err(|_| Mqtt311Error::Io)?;

    let mut multiplier = 1usize;
    let mut value = 0usize;
    loop {
      let mut b = [0u8; 1];
      socket
        .read_exact(&mut b)
        .await
        .map_err(|_| Mqtt311Error::Io)?;
      value += (b[0] & 0x7f) as usize * multiplier;
      if b[0] & 0x80 == 0 {
        break;
      }
      multiplier *= 128;
      if multiplier > 128 * 128 * 128 {
        return Err(Mqtt311Error::Protocol);
      }
    }

    if value > buf.len() {
      return Err(Mqtt311Error::TooLarge);
    }
    socket
      .read_exact(&mut buf[..value])
      .await
      .map_err(|_| Mqtt311Error::Io)?;
    Ok((header[0], &buf[..value]))
  }

  // ── Tests (require std for the mock socket) ─────────────────────────

  #[cfg(test)]
  mod tests {
    use super::*;
    extern crate std;
    use std::vec::Vec;

    /// A trivial in-memory bidirectional socket for offline packet
    /// verification.
    struct MockSocket {
      rx: Vec<u8>,
      tx: Vec<u8>,
    }

    impl MockSocket {
      fn new(rx: Vec<u8>) -> Self {
        MockSocket { rx, tx: Vec::new() }
      }
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
      // A brutally simple executor for offline tests.
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
          Poll::Pending => panic!("test future was Pending (mock is fully synchronous)"),
        }
      }
    }

    #[test]
    fn publish_encodes_expected_bytes() {
      let sock = MockSocket::new(Vec::new());
      let mut client = Mqtt311Client::new(sock, "test-client", 30);
      block_on(client.publish("hi", b"world")).unwrap();

      let expected: Vec<u8> = std::vec![
        0x30, // PUBLISH, DUP=0, QoS=0, RETAIN=0
        0x09, // remaining length: 2 (topic len) + 2 (topic) + 5 (payload)
        0x00, 0x02, // topic length
        b'h', b'i', b'w', b'o', b'r', b'l', b'd',
      ];
      let got = client.into_inner().tx;
      assert_eq!(got, expected);
    }

    #[test]
    fn connect_accepts_zero_connack() {
      let connack: Vec<u8> = std::vec![0x20, 0x02, 0x00, 0x00];
      let sock = MockSocket::new(connack);
      let mut client = Mqtt311Client::new(sock, "cid", 30);
      block_on(client.connect()).unwrap();
      // First byte in tx should be CONNECT type.
      assert_eq!(client.into_inner().tx[0], 0x10);
    }

    #[test]
    fn connect_rejects_non_zero_connack() {
      let connack: Vec<u8> = std::vec![0x20, 0x02, 0x00, 0x05];
      let sock = MockSocket::new(connack);
      let mut client = Mqtt311Client::new(sock, "cid", 30);
      let err = block_on(client.connect()).unwrap_err();
      assert_eq!(err, Mqtt311Error::Rejected);
    }

    #[test]
    fn read_publish_decodes_topic_and_payload() {
      // Full PUBLISH: type=0x30, remaining_len=9 (varint 0x09), topic_len=2,
      // topic='hi', payload='world'
      let publish: Vec<u8> = std::vec![
        0x30, 0x09, 0x00, 0x02, b'h', b'i', b'w', b'o', b'r', b'l', b'd',
      ];
      let sock = MockSocket::new(publish);
      let mut client = Mqtt311Client::new(sock, "cid", 30);
      let mut buf = [0u8; 32];
      let got = block_on(client.read_publish(&mut buf)).unwrap().unwrap();
      assert_eq!(got.topic, "hi");
      assert_eq!(got.payload, b"world");
    }

    #[test]
    fn read_publish_returns_none_for_pingresp() {
      // PINGRESP: 0xD0 0x00
      let pingresp: Vec<u8> = std::vec![0xD0, 0x00];
      let sock = MockSocket::new(pingresp);
      let mut client = Mqtt311Client::new(sock, "cid", 30);
      let mut buf = [0u8; 8];
      let got = block_on(client.read_publish(&mut buf)).unwrap();
      assert!(got.is_none());
    }
  }
}
