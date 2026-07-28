#![no_std]
#![no_main]

//! RelayStar simple LoRa node for the LilyGO T3-S3 (ESP32-S3 + SX1262).
//!
//! Behaviour:
//! - Listens continuously for RelayStar [`Message`]s over LoRa.
//! - Every few seconds, transmits a text heartbeat so the Cardputer terminal
//!   can see the node is alive and relay it onwards.
//! - Renders the last received text and counters on the onboard SSD1306 OLED.

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
  LoRa, RxMode,
  iv::GenericSx126xInterfaceVariant,
  mod_params::{Bandwidth, CodingRate, SpreadingFactor},
  sx126x::{self, Sx126x, Sx1262, TcxoCtrlVoltage},
};
use relaystar_proto::{MAX_FRAME, Message, MsgKind, Transport, radio as rparams};
use ssd1306::{I2CDisplayInterface, Ssd1306, prelude::*};

esp_bootloader_esp_idf::esp_app_desc!();

/// This node's synthetic 6-byte address (not a real MAC).
const NODE_ADDR: [u8; 6] = [0x02, 0x00, 0x00, 0x00, 0x00, 0x01];
/// Message id base so ids from this node don't collide with the Cardputer's.
const NODE_ID_BASE: u32 = 0x0100_0000;

fn map_spreading_factor(sf: u8) -> SpreadingFactor {
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

fn map_bandwidth(khz: u16) -> Bandwidth {
  match khz {
    125 => Bandwidth::_125KHz,
    250 => Bandwidth::_250KHz,
    _ => Bandwidth::_500KHz,
  }
}

fn map_coding_rate(denom: u8) -> CodingRate {
  match denom {
    5 => CodingRate::_4_5,
    6 => CodingRate::_4_6,
    7 => CodingRate::_4_7,
    _ => CodingRate::_4_8,
  }
}

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
  let mut lora = match LoRa::new(Sx126x::new(spi_device, iv, lora_config), false, Delay).await {
    Ok(l) => l,
    Err(e) => {
      println!("LoRa init failed: {:?}", e);
      draw_status(&mut display, "LoRa init FAILED", "", 0, 0);
      loop {
        Timer::after(Duration::from_secs(5)).await;
      }
    }
  };

  let mdltn = lora
    .create_modulation_params(
      map_spreading_factor(rparams::LORA_SPREADING_FACTOR),
      map_bandwidth(rparams::LORA_BANDWIDTH_KHZ),
      map_coding_rate(rparams::LORA_CODING_RATE_DENOM),
      rparams::LORA_FREQUENCY_HZ,
    )
    .unwrap();
  let rx_pp = lora
    .create_rx_packet_params(
      rparams::LORA_PREAMBLE_LEN,
      false,
      MAX_FRAME as u8,
      true,
      false,
      &mdltn,
    )
    .unwrap();
  let mut tx_pp = lora
    .create_tx_packet_params(rparams::LORA_PREAMBLE_LEN, false, true, false, &mdltn)
    .unwrap();

  println!("LoRa ready @ {} Hz", rparams::LORA_FREQUENCY_HZ);

  let mut rx_buf = [0u8; MAX_FRAME];
  let mut tx_counter: u32 = 0;
  let mut rx_counter: u32 = 0;
  let mut last_rx: heapless::String<48> = heapless::String::new();
  let mut heartbeat = Ticker::every(Duration::from_secs(8));

  draw_status(&mut display, "LoRa node ready", "", tx_counter, rx_counter);

  loop {
    if let Err(e) = lora
      .prepare_for_rx(RxMode::Continuous, &mdltn, &rx_pp)
      .await
    {
      println!("prepare_for_rx error: {:?}", e);
      Timer::after(Duration::from_millis(500)).await;
      continue;
    }

    match select(lora.rx(&rx_pp, &mut rx_buf), heartbeat.next()).await {
      // --- Inbound LoRa frame ---
      Either::First(rx_result) => match rx_result {
        Ok((len, status)) => {
          let bytes = &rx_buf[..len as usize];
          match Message::decode(bytes) {
            Ok(msg) => {
              rx_counter = rx_counter.wrapping_add(1);
              let text = msg.as_text().unwrap_or("<binary>");
              println!(
                "RX id={} kind={:?} rssi={} \"{}\"",
                msg.id, msg.kind, status.rssi, text
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

              // Reply to pings so the terminal can measure the link.
              if msg.kind == MsgKind::Ping {
                send_message(
                  &mut lora,
                  &mdltn,
                  &mut tx_pp,
                  MsgKind::Pong,
                  b"pong",
                  node_msg_id(tx_counter),
                )
                .await;
                tx_counter = tx_counter.wrapping_add(1);
              }
            }
            Err(e) => println!("decode error ({} bytes): {:?}", len, e),
          }
        }
        Err(e) => println!("rx error: {:?}", e),
      },

      // --- Heartbeat: transmit a text message ---
      Either::Second(_) => {
        let mut text: heapless::String<32> = heapless::String::new();
        let _ = write!(text, "node hb #{}", tx_counter);
        send_message(
          &mut lora,
          &mdltn,
          &mut tx_pp,
          MsgKind::Text,
          text.as_bytes(),
          node_msg_id(tx_counter),
        )
        .await;
        tx_counter = tx_counter.wrapping_add(1);
        draw_status(&mut display, "TX ->", text.as_str(), tx_counter, rx_counter);
      }
    }
  }
}

fn node_msg_id(counter: u32) -> u32 {
  NODE_ID_BASE.wrapping_add(counter)
}

/// Build, encode, and transmit a RelayStar message over LoRa.
async fn send_message<RK, DLY>(
  lora: &mut LoRa<RK, DLY>,
  mdltn: &lora_phy::mod_params::ModulationParams,
  tx_pp: &mut lora_phy::mod_params::PacketParams,
  kind: MsgKind,
  payload: &[u8],
  id: u32,
) where
  RK: lora_phy::mod_traits::RadioKind,
  DLY: embedded_hal_async::delay::DelayNs,
{
  let msg = match Message::new(id, Transport::Lora, NODE_ADDR, kind, payload) {
    Ok(m) => m,
    Err(e) => {
      println!("build message failed: {:?}", e);
      return;
    }
  };
  let mut buf = [0u8; MAX_FRAME];
  let encoded = match msg.encode(&mut buf) {
    Ok(b) => b,
    Err(e) => {
      println!("encode failed: {:?}", e);
      return;
    }
  };
  if let Err(e) = lora
    .prepare_for_tx(mdltn, tx_pp, rparams::LORA_TX_POWER_DBM, encoded)
    .await
  {
    println!("prepare_for_tx failed: {:?}", e);
    return;
  }
  match lora.tx().await {
    Ok(()) => println!("TX id={} ({} bytes)", id, encoded.len()),
    Err(e) => println!("tx failed: {:?}", e),
  }
  let _ = lora.sleep(false).await;
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
