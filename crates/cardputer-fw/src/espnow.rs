//! ESP-NOW transport loop for the Cardputer terminal.
//!
//! Broadcasts outbound messages and receives inbound ones. Received frames are
//! decoded, stamped with `origin = EspNow`, and pushed to the bridge. Unknown
//! senders are auto-registered as peers.

use embassy_futures::select::{select, Either};
use esp_println::println;
use esp_radio::esp_now::{
  EspNowManager, EspNowReceiver, EspNowSender, EspNowWifiInterface, PeerInfo, BROADCAST_ADDRESS,
};
use relaystar_proto::{Message, Transport, MAX_FRAME};

use crate::bridge::{ESPNOW_OUT, INBOUND};

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
      // Inbound ESP-NOW frame.
      Either::First(recv) => {
        let src = recv.info.src_address;
        match Message::decode(recv.data()) {
          Ok(mut msg) => {
            msg.origin = Transport::EspNow;
            println!("espnow: RX id={} from {:02x?}", msg.id, src);
            // Auto-register unicast peers we haven't seen.
            if src != BROADCAST_ADDRESS && !manager.peer_exists(&src) {
              let _ = manager.add_peer(PeerInfo {
                interface: EspNowWifiInterface::Station,
                peer_address: src,
                lmk: None,
                channel: None,
                encrypt: false,
              });
            }
            INBOUND.send(msg).await;
          }
          Err(e) => println!("espnow: decode error {:?}", e),
        }
      }

      // Outbound message -> broadcast.
      Either::Second(msg) => {
        let mut buf = [0u8; MAX_FRAME];
        match msg.encode(&mut buf) {
          Ok(encoded) => {
            let _ = sender.send_async(&BROADCAST_ADDRESS, encoded).await;
            println!("espnow: TX id={} ({} bytes)", msg.id, encoded.len());
          }
          Err(e) => println!("espnow: encode error {:?}", e),
        }
      }
    }
  }
}
