#![no_std]
#![no_main]

//! RelayStar simple LoRa node for the LilyGO T3-S3 (ESP32-S3 + SX1262).
//!
//! Behaviour:
//! - Uses [`relaystar_relay::Relay`] to decode inbound LoRa frames (with
//!   automatic reassembly + deduplication) and to plan outbound frames (with
//!   automatic MTU-aware fragmentation).
//! - Every few seconds, transmits a text heartbeat as a **broadcast**.
//! - When it receives a [`MsgKind::Ping`], it replies with a **unicast**
//!   `Pong` back to the sender. The receiver table is auto-populated on
//!   `ingest`, so the unicast target is resolvable without any manual setup.
//! - Renders the last received text and counters on the onboard SSD1306 OLED.
//!
//! The LoRa hardware is driven entirely through
//! [`relaystar_relay::ports::lora::SxLoraPort`], which owns the `lora-phy`
//! handle and handles `prepare_for_tx → tx → sleep` (and `prepare_for_rx → rx`)
//! internally. This firmware only has to deal with board-specific concerns
//! (SPI / GPIO wiring and the OLED).

use core::fmt::Write as _;

use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_time::{Delay, Duration, Ticker, Timer};
use esp_backtrace as _;
use esp_hal::{
  clock::CpuClock,
  gpio::{Input, InputConfig, Level, Output, OutputConfig},
  i2c::master::{Config as I2cConfig, I2c},
  interrupt::software::SoftwareInterruptControl,
  spi::{
    Mode,
    master::{Config as SpiConfig, Spi},
  },
  time::Rate,
  timer::timg::TimerGroup,
};
use esp_println::println;

use embedded_graphics::{
  mono_font::{MonoTextStyleBuilder, ascii::FONT_6X10},
  pixelcolor::BinaryColor,
  prelude::*,
  text::{Baseline, Text},
};
use embedded_hal_bus::spi::ExclusiveDevice;
use lora_phy::{
  LoRa,
  iv::GenericSx126xInterfaceVariant,
  sx126x::{self, Sx126x, Sx1262, TcxoCtrlVoltage},
};
use relaystar_proto::{MAX_FRAME, MsgKind, Transport, radio as rparams};
use relaystar_relay::port::TransportPort;
use relaystar_relay::ports::lora::{LoraModulationParams, SxLoraPort};
use relaystar_relay::{Destination, IngestOutcome, PlannedFrame, Relay, RelayError};
use ssd1306::{I2CDisplayInterface, Ssd1306, prelude::*};

esp_bootloader_esp_idf::esp_app_desc!();

/// This node's synthetic 6-byte address (not a real MAC).
const NODE_ADDR: [u8; 6] = [0x02, 0x00, 0x00, 0x00, 0x00, 0x01];
/// Message id base so ids from this node don't collide with the Cardputer's.
const NODE_ID_BASE: u32 = 0x0100_0000;

/// Relay engine dimensions for this node:
/// - `NR = 8` tracked peers (a LoRa node typically sees a handful).
/// - `NA = 2` concurrent reassembly slots.
/// - `NF = 32` max fragments per group.
type NodeRelay = Relay<8, 2, 32>;

#[esp_rtos::main]
async fn main(_spawner: Spawner) {
  esp_println::logger::init_logger_from_env();

  let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
  esp_alloc::heap_allocator!(size: 64 * 1024);

  let timg0 = TimerGroup::new(peripherals.TIMG0);
  let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
  esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

  println!("RelayStar LoRa node booting (LilyGO T3-S3, SX1262)");

  // --- OLED (SSD1306 128x64 over I2C: SDA=18, SCL=17) ---
  let i2c = I2c::new(peripherals.I2C0, I2cConfig::default())
    .unwrap()
    .with_sda(peripherals.GPIO18)
    .with_scl(peripherals.GPIO17);
  let iface = I2CDisplayInterface::new(i2c);
  let mut display =
    Ssd1306::new(iface, DisplaySize128x64, DisplayRotation::Rotate0).into_buffered_graphics_mode();
  let _ = display.init();

  // --- SX1262 LoRa over SPI2 (SCK=5, MISO=3, MOSI=6, NSS=7, RST=8, DIO1=33, BUSY=34) ---
  let nss = Output::new(peripherals.GPIO7, Level::High, OutputConfig::default());
  let reset = Output::new(peripherals.GPIO8, Level::High, OutputConfig::default());
  let busy = Input::new(peripherals.GPIO34, InputConfig::default());
  let dio1 = Input::new(peripherals.GPIO33, InputConfig::default());

  let spi = Spi::new(
    peripherals.SPI2,
    SpiConfig::default()
      .with_frequency(Rate::from_mhz(4))
      .with_mode(Mode::_0),
  )
  .unwrap()
  .with_sck(peripherals.GPIO5)
  .with_mosi(peripherals.GPIO6)
  .with_miso(peripherals.GPIO3)
  .into_async();

  let spi_device = ExclusiveDevice::new(spi, nss, Delay).unwrap();

  let lora_config = sx126x::Config {
    chip: Sx1262,
    tcxo_ctrl: Some(TcxoCtrlVoltage::Ctrl1V7),
    use_dcdc: true,
    rx_boost: false,
  };
  let iv = GenericSx126xInterfaceVariant::new(reset, dio1, busy, None, None).unwrap();
  let lora = match LoRa::new(Sx126x::new(spi_device, iv, lora_config), false, Delay).await {
    Ok(l) => l,
    Err(e) => {
      println!("LoRa init failed: {:?}", e);
      draw_status(&mut display, "LoRa init FAILED", "", 0, 0);
      loop {
        Timer::after(Duration::from_secs(5)).await;
      }
    }
  };

  // Wrap the raw lora-phy handle in a SxLoraPort so all subsequent TX/RX
  // goes through relaystar-relay's bundled adapter — no more hand-written
  // `prepare_for_tx → tx → sleep` sequences here.
  let mut lora_port = match SxLoraPort::new(
    lora,
    LoraModulationParams {
      frequency_hz: rparams::LORA_FREQUENCY_HZ,
      spreading_factor: rparams::LORA_SPREADING_FACTOR,
      bandwidth_khz: rparams::LORA_BANDWIDTH_KHZ,
      coding_rate_denom: rparams::LORA_CODING_RATE_DENOM,
      preamble_symbols: rparams::LORA_PREAMBLE_LEN,
      tx_power_dbm: rparams::LORA_TX_POWER_DBM,
    },
    MAX_FRAME as u8,
  )
  .await
  {
    Ok(p) => p,
    Err(e) => {
      println!("SxLoraPort init failed: {}", e);
      draw_status(&mut display, "LoRa params FAILED", "", 0, 0);
      loop {
        Timer::after(Duration::from_secs(5)).await;
      }
    }
  };

  println!("LoRa ready @ {} Hz", rparams::LORA_FREQUENCY_HZ);

  // --- Relay engine setup ---
  //
  // Only LoRa is registered because this node has a single transport. All
  // fan-out logic, dedup, reassembly, and receiver learning still apply.
  let mut relay: NodeRelay = NodeRelay::new(NODE_ADDR, NODE_ID_BASE);
  if let Err(e) = relay.register_port(Transport::Lora) {
    println!("relay register_port failed: {}", e);
  }

  let mut rx_buf = [0u8; MAX_FRAME];
  let mut tx_counter: u32 = 0;
  let mut rx_counter: u32 = 0;
  let mut last_rx: heapless::String<48> = heapless::String::new();
  let mut heartbeat = Ticker::every(Duration::from_secs(8));

  draw_status(&mut display, "LoRa node ready", "", tx_counter, rx_counter);

  loop {
    match select(lora_port.rx_frame(&mut rx_buf), heartbeat.next()).await {
      // --- Inbound LoRa frame ---
      Either::First(rx_result) => match rx_result {
        Ok(outcome) => {
          let len = outcome.len as usize;
          let bytes = &rx_buf[..len];
          match relay.ingest(Transport::Lora, bytes) {
            Ok(IngestOutcome::Complete(msg)) => {
              rx_counter = rx_counter.wrapping_add(1);
              let text = msg.as_text().unwrap_or("<binary>");
              println!(
                "RX id={} kind={:?} rssi={} snr={} \"{}\"",
                msg.id, msg.kind, outcome.rssi, outcome.snr, text
              );
              last_rx.clear();
              let _ = write!(last_rx, "{}", text);
              draw_status(
                &mut display,
                "RX <-",
                last_rx.as_str(),
                tx_counter,
                rx_counter,
              );

              // Reply to pings so the terminal can measure the link. The
              // relay's auto-learned receiver table lets us reply as a proper
              // unicast; if for some reason the sender isn't in the table
              // yet (edge cases with lost fragments) we fall back to a
              // broadcast.
              if msg.kind == MsgKind::Ping {
                let sent = match send_via_port(
                  &mut lora_port,
                  &relay,
                  MsgKind::Pong,
                  b"pong",
                  Destination::Unicast(msg.from),
                )
                .await
                {
                  Ok(n) => n,
                  Err(RelayError::UnknownReceiver) => send_via_port(
                    &mut lora_port,
                    &relay,
                    MsgKind::Pong,
                    b"pong",
                    Destination::Broadcast,
                  )
                  .await
                  .unwrap_or_else(|e| {
                    println!("plan_send Pong (broadcast fallback) failed: {}", e);
                    0
                  }),
                  Err(e) => {
                    println!("plan_send Pong failed: {}", e);
                    0
                  }
                };
                tx_counter = tx_counter.wrapping_add(sent as u32);
              }
            }
            Ok(IngestOutcome::NotForMe(_)) => {
              // Single-transport node: no other transport to forward onto.
              // Silently drop.
            }
            Ok(IngestOutcome::Buffered) => {
              // Waiting for more fragments; nothing to display yet.
            }
            Ok(IngestOutcome::Duplicate) => {
              // Loop prevention already handled by the relay.
            }
            Ok(IngestOutcome::Dropped(reason)) => {
              println!("relay: reassembly dropped: {:?}", reason);
            }
            // `IngestOutcome` is #[non_exhaustive]; ignore future variants.
            Ok(_) => {}
            Err(e) => println!("relay: ingest error: {}", e),
          }
        }
        Err(e) => println!("rx error: {}", e),
      },

      // --- Heartbeat: transmit a broadcast text message ---
      Either::Second(_) => {
        let mut text: heapless::String<32> = heapless::String::new();
        let _ = write!(text, "node hb #{}", tx_counter);
        match send_via_port(
          &mut lora_port,
          &relay,
          MsgKind::Text,
          text.as_bytes(),
          Destination::Broadcast,
        )
        .await
        {
          Ok(n) => {
            tx_counter = tx_counter.wrapping_add(n as u32);
            draw_status(&mut display, "TX ->", text.as_str(), tx_counter, rx_counter);
          }
          Err(e) => println!("heartbeat plan_send failed: {}", e),
        }
      }
    }
  }
}

/// Plan a send with the relay engine, then emit every resulting LoRa frame
/// through [`SxLoraPort`]. Returns the number of frames actually transmitted.
///
/// This is the *only* place in this firmware that touches the radio TX path,
/// so callers get the relay's MTU-aware fragmentation for free.
async fn send_via_port<RK, DLY>(
  port: &mut SxLoraPort<RK, DLY>,
  relay: &NodeRelay,
  kind: MsgKind,
  payload: &[u8],
  dest: Destination,
) -> Result<usize, RelayError>
where
  RK: lora_phy::mod_traits::RadioKind,
  DLY: embedded_hal_async::delay::DelayNs,
{
  let plan = relay.plan_send(kind, payload, dest)?;
  let mut sent = 0usize;
  for frame in plan {
    if frame.transport != Transport::Lora {
      continue;
    }
    if transmit_planned(port, &frame).await {
      sent += 1;
    }
  }
  Ok(sent)
}

/// Encode a [`PlannedFrame`] and hand it to the port. Returns `true` on
/// successful TX.
async fn transmit_planned<RK, DLY>(port: &mut SxLoraPort<RK, DLY>, frame: &PlannedFrame) -> bool
where
  RK: lora_phy::mod_traits::RadioKind,
  DLY: embedded_hal_async::delay::DelayNs,
{
  let mut buf = [0u8; MAX_FRAME];
  let encoded = match frame.message.encode(&mut buf) {
    Ok(b) => b,
    Err(e) => {
      println!("encode failed: {:?}", e);
      return false;
    }
  };
  match port.send_frame(frame.addr, encoded).await {
    Ok(()) => {
      println!("TX id={} ({} bytes)", frame.message.id, encoded.len());
      true
    }
    Err(e) => {
      println!("send_frame failed: {}", e);
      false
    }
  }
}

/// Render a two-line status plus counters onto the OLED.
fn draw_status<DI>(
  display: &mut Ssd1306<
    DI,
    DisplaySize128x64,
    ssd1306::mode::BufferedGraphicsMode<DisplaySize128x64>,
  >,
  line1: &str,
  line2: &str,
  tx: u32,
  rx: u32,
) where
  DI: ssd1306::prelude::WriteOnlyDataCommand,
{
  let style = MonoTextStyleBuilder::new()
    .font(&FONT_6X10)
    .text_color(BinaryColor::On)
    .build();

  display.clear(BinaryColor::Off).ok();
  Text::with_baseline("RelayStar node", Point::new(0, 0), style, Baseline::Top)
    .draw(display)
    .ok();
  Text::with_baseline(line1, Point::new(0, 16), style, Baseline::Top)
    .draw(display)
    .ok();
  Text::with_baseline(line2, Point::new(0, 28), style, Baseline::Top)
    .draw(display)
    .ok();

  let mut counters: heapless::String<32> = heapless::String::new();
  let _ = write!(counters, "TX:{}  RX:{}", tx, rx);
  Text::with_baseline(counters.as_str(), Point::new(0, 52), style, Baseline::Top)
    .draw(display)
    .ok();

  display.flush().ok();
}
