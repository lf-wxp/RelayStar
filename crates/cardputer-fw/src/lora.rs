//! LoRa (SX1262) transport loop for the Cardputer terminal.
//!
//! Owns the radio and interleaves continuous RX with servicing the
//! [`LORA_OUT`](crate::bridge::LORA_OUT) transmit queue. Inbound frames are
//! forwarded as raw bytes into
//! [`INBOUND_RAW`](crate::bridge::INBOUND_RAW); the relay engine (see
//! [`crate::bridge`]) decodes, deduplicates, and reassembles them.

use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Timer};
use esp_println::println;
use lora_phy::{
  LoRa, RxMode,
  mod_params::{Bandwidth, CodingRate, ModulationParams, PacketParams, SpreadingFactor},
};
use relaystar_proto::{MAX_FRAME, Transport, radio as rparams};

use crate::bridge::{INBOUND_RAW, LORA_OUT, RawFrame};

pub fn map_spreading_factor(sf: u8) -> SpreadingFactor {
  match sf {
    5 => SpreadingFactor::_5,
    6 => SpreadingFactor::_6,
    7 => SpreadingFactor::_7,
    8 => SpreadingFactor::_8,
    9 => SpreadingFactor::_9,
    10 => SpreadingFactor::_10,
    11 => SpreadingFactor::_11,
    _ => SpreadingFactor::_12,
  }
}

pub fn map_bandwidth(khz: u16) -> Bandwidth {
  match khz {
    125 => Bandwidth::_125KHz,
    250 => Bandwidth::_250KHz,
    _ => Bandwidth::_500KHz,
  }
}

pub fn map_coding_rate(denom: u8) -> CodingRate {
  match denom {
    5 => CodingRate::_4_5,
    6 => CodingRate::_4_6,
    7 => CodingRate::_4_7,
    _ => CodingRate::_4_8,
  }
}

/// Run the LoRa transport forever.
pub async fn lora_loop<RK, DLY>(
  mut lora: LoRa<RK, DLY>,
  mdltn: ModulationParams,
  rx_pp: PacketParams,
  mut tx_pp: PacketParams,
) where
  RK: lora_phy::mod_traits::RadioKind,
  DLY: embedded_hal_async::delay::DelayNs,
{
  println!("lora: transport started");
  let mut rx_buf = [0u8; MAX_FRAME];

  loop {
    if let Err(e) = lora
      .prepare_for_rx(RxMode::Continuous, &mdltn, &rx_pp)
      .await
    {
      println!("lora: prepare_for_rx error {:?}", e);
      Timer::after(Duration::from_millis(500)).await;
      continue;
    }

    match select(lora.rx(&rx_pp, &mut rx_buf), LORA_OUT.receive()).await {
      // Inbound frame: hand raw bytes to the relay engine via the bridge.
      Either::First(res) => match res {
        Ok((len, status)) => {
          println!("lora: RX {} bytes rssi={}", len, status.rssi);
          let mut bytes: heapless::Vec<u8, MAX_FRAME> = heapless::Vec::new();
          if bytes.extend_from_slice(&rx_buf[..len as usize]).is_err() {
            println!("lora: rx frame exceeds MAX_FRAME, dropped");
            continue;
          }
          INBOUND_RAW
            .send(RawFrame {
              source: Transport::Lora,
              bytes,
            })
            .await;
        }
        Err(e) => println!("lora: rx error {:?}", e),
      },

      // Outbound frame from the bridge. `FrameAddr` is informational for
      // LoRa (link-layer is shared-medium); the destination is already
      // encoded inside the `Message.to` field.
      Either::Second((_addr, msg)) => {
        let mut buf = [0u8; MAX_FRAME];
        match msg.encode(&mut buf) {
          Ok(encoded) => {
            let encoded_len = encoded.len();
            if let Err(e) = lora
              .prepare_for_tx(&mdltn, &mut tx_pp, rparams::LORA_TX_POWER_DBM, encoded)
              .await
            {
              println!("lora: prepare_for_tx error {:?}", e);
              continue;
            }
            match lora.tx().await {
              Ok(()) => println!("lora: TX id={} ({} bytes)", msg.id, encoded_len),
              Err(e) => println!("lora: tx error {:?}", e),
            }
            let _ = lora.sleep(false).await;
          }
          Err(e) => println!("lora: encode error {:?}", e),
        }
      }
    }
  }
}
