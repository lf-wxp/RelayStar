//! ESP-NOW transport loop for the Cardputer terminal.
//!
//! Broadcasts outbound frames or unicasts them to a specific MAC based on the
//! `FrameAddr` supplied by the relay engine, and forwards received frames as
//! raw bytes into [`INBOUND_RAW`](crate::bridge::INBOUND_RAW). Unknown senders
//! are auto-registered as peers.

use embassy_futures::select::{Either, select};
use esp_println::println;
use esp_radio::esp_now::{
  BROADCAST_ADDRESS, EspNowManager, EspNowReceiver, EspNowSender, EspNowWifiInterface, PeerInfo,
};
use relaystar_proto::{MAX_FRAME, Transport};
use relaystar_relay::FrameAddr;

use crate::bridge::{ESPNOW_OUT, INBOUND_RAW, RawFrame};

/// Run the ESP-NOW transport forever.
pub async fn espnow_loop(
  manager: EspNowManager<'_>,
  mut sender: EspNowSender<'_>,
  mut receiver: EspNowReceiver<'_>,
) {
  // Ensure the broadcast address is a known peer so `send` works.
  let _ = manager.add_peer(PeerInfo {
    interface: EspNowWifiInterface::Station,
    peer_address: BROADCAST_ADDRESS,
    lmk: None,
    channel: None,
    encrypt: false,
  });
  println!("espnow: transport started");

  loop {
    match select(receiver.receive_async(), ESPNOW_OUT.receive()).await {
      // Inbound ESP-NOW frame: forward raw bytes to the relay engine.
      Either::First(recv) => {
        let src = recv.info.src_address;
        let data = recv.data();
        println!("espnow: RX {} bytes from {:02x?}", data.len(), src);

        // Auto-register unicast peers we haven't seen so we can unicast back.
        if src != BROADCAST_ADDRESS && !manager.peer_exists(&src) {
          let _ = manager.add_peer(PeerInfo {
            interface: EspNowWifiInterface::Station,
            peer_address: src,
            lmk: None,
            channel: None,
            encrypt: false,
          });
        }

        let mut bytes: heapless::Vec<u8, MAX_FRAME> = heapless::Vec::new();
        if bytes.extend_from_slice(data).is_err() {
          println!("espnow: rx frame exceeds MAX_FRAME, dropped");
          continue;
        }
        INBOUND_RAW
          .send(RawFrame {
            source: Transport::EspNow,
            bytes,
          })
          .await;
      }

      // Outbound frame with an explicit addressing hint.
      Either::Second((addr, msg)) => {
        let target_mac = match addr {
          FrameAddr::Unicast(a) => a,
          FrameAddr::Broadcast => BROADCAST_ADDRESS,
        };
        // Auto-register on-the-fly for unicast destinations.
        if target_mac != BROADCAST_ADDRESS && !manager.peer_exists(&target_mac) {
          let _ = manager.add_peer(PeerInfo {
            interface: EspNowWifiInterface::Station,
            peer_address: target_mac,
            lmk: None,
            channel: None,
            encrypt: false,
          });
        }

        let mut buf = [0u8; MAX_FRAME];
        match msg.encode(&mut buf) {
          Ok(encoded) => {
            let _ = sender.send_async(&target_mac, encoded).await;
            println!(
              "espnow: TX id={} ({} bytes) to {:02x?}",
              msg.id,
              encoded.len(),
              target_mac
            );
          }
          Err(e) => println!("espnow: encode error {:?}", e),
        }
      }
    }
  }
}
