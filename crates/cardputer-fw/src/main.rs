#![no_std]
#![no_main]

//! RelayStar terminal firmware for the M5Stack Cardputer-Adv (ESP32-S3).
//!
//! Bridges three transports:
//! - LoRa (Cap LoRa-1262 / SX1262 on the EXT bus)
//! - MQTT (over Wi-Fi STA, to the rumqttd broker)
//! - ESP-NOW (2.4 GHz)
//!
//! A central [`bridge`] task de-duplicates and relays messages between them, and
//! a UI task renders traffic on the ST7789V2 LCD and lets you compose messages
//! on the TCA8418 keyboard.

extern crate alloc;

mod bridge;
mod display;
mod espnow;
mod keyboard;
mod lora;
mod mqtt;

use embassy_futures::join::{join, join3, join4};
use embassy_futures::select::{select, Either};
use embassy_time::{Delay, Duration, Ticker};
use esp_backtrace as _;
use esp_hal::{
  clock::CpuClock,
  delay::Delay as HalDelay,
  gpio::{Input, InputConfig, Level, Output, OutputConfig},
  i2c::master::{Config as I2cConfig, I2c},
  interrupt::software::SoftwareInterruptControl,
  rng::Rng,
  spi::{
    master::{Config as SpiConfig, Spi},
    Mode,
  },
  time::Rate,
  timer::timg::TimerGroup,
};
use esp_println::println;

use embedded_graphics::{pixelcolor::Rgb565, prelude::*};
use embedded_hal_bus::spi::ExclusiveDevice;
use lora_phy::{
  iv::GenericSx126xInterfaceVariant,
  sx126x::{self, Sx1262, Sx126x, TcxoCtrlVoltage},
  LoRa,
};
use mipidsi::{
  interface::SpiInterface,
  models::ST7789,
  options::{ColorInversion, Orientation, Rotation},
  Builder,
};

use crate::bridge::{LOCAL_OUT, UI_IN};
use crate::display::Ui;
use crate::keyboard::{decode_key, KeyAction, Keyboard};
use relaystar_proto::{radio as rparams, Message, Transport, MAX_FRAME};

esp_bootloader_esp_idf::esp_app_desc!();

/// Promote a value to a `'static` reference via a `StaticCell`.
macro_rules! mk_static {
  ($t:ty, $val:expr) => {{
    static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
    STATIC_CELL.uninit().write($val)
  }};
}

// PI4IOE5V6408 I2C port expander on the Cap LoRa-1262 (controls the SX1262 RF
// antenna switch via P0). Without enabling this, LoRa TX/RX will appear dead.
const IOEXP_ADDR: u8 = 0x43;

#[esp_rtos::main]
async fn main(_spawner: embassy_executor::Spawner) {
  esp_println::logger::init_logger_from_env();

  let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
  esp_alloc::heap_allocator!(size: 110 * 1024);

  let timg0 = TimerGroup::new(peripherals.TIMG0);
  let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
  esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

  println!("RelayStar terminal booting (Cardputer-Adv, ESP32-S3)");

  // -------------------------------------------------------------------------
  // Internal I2C bus (SDA=G8, SCL=G9): TCA8418 keyboard (0x34) + Cap LoRa
  // antenna switch expander (0x43).
  // -------------------------------------------------------------------------
  let mut i2c = I2c::new(peripherals.I2C0, I2cConfig::default())
    .unwrap()
    .with_sda(peripherals.GPIO8)
    .with_scl(peripherals.GPIO9);

  // Enable the SX1262 antenna switch (drive P0 high). Registers per
  // PI4IOE5V6408: 0x07 output high-impedance, 0x03 direction, 0x05 output.
  let _ = i2c.write(IOEXP_ADDR, &[0x07, 0xFE]); // P0 not high-impedance
  let _ = i2c.write(IOEXP_ADDR, &[0x03, 0x01]); // P0 = output
  let _ = i2c.write(IOEXP_ADDR, &[0x05, 0x01]); // P0 = high (RF path on)

  let mut keyboard = Keyboard::new(i2c);

  // -------------------------------------------------------------------------
  // LoRa (SX1262) on SPI2. Cardputer-Adv + Cap LoRa-1262 pin map:
  // NSS=G5, SCK=G40, MOSI=G14, MISO=G39, IRQ/DIO1=G4, BUSY=G6, RST=G3.
  // -------------------------------------------------------------------------
  let lora_nss = Output::new(peripherals.GPIO5, Level::High, OutputConfig::default());
  let lora_reset = Output::new(peripherals.GPIO3, Level::High, OutputConfig::default());
  let lora_busy = Input::new(peripherals.GPIO6, InputConfig::default());
  let lora_dio1 = Input::new(peripherals.GPIO4, InputConfig::default());

  let lora_spi = Spi::new(
    peripherals.SPI2,
    SpiConfig::default()
      .with_frequency(Rate::from_mhz(4))
      .with_mode(Mode::_0),
  )
  .unwrap()
  .with_sck(peripherals.GPIO40)
  .with_mosi(peripherals.GPIO14)
  .with_miso(peripherals.GPIO39)
  .into_async();

  let lora_spi_dev = ExclusiveDevice::new(lora_spi, lora_nss, Delay).unwrap();

  let lora_config = sx126x::Config {
    chip: Sx1262,
    tcxo_ctrl: Some(TcxoCtrlVoltage::Ctrl1V7),
    use_dcdc: true,
    rx_boost: false,
  };
  let iv =
    GenericSx126xInterfaceVariant::new(lora_reset, lora_dio1, lora_busy, None, None).unwrap();
  let mut lora = LoRa::new(Sx126x::new(lora_spi_dev, iv, lora_config), false, Delay)
    .await
    .expect("LoRa init failed");

  let mdltn = lora
    .create_modulation_params(
      lora::map_spreading_factor(rparams::LORA_SPREADING_FACTOR),
      lora::map_bandwidth(rparams::LORA_BANDWIDTH_KHZ),
      lora::map_coding_rate(rparams::LORA_CODING_RATE_DENOM),
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
  let tx_pp = lora
    .create_tx_packet_params(rparams::LORA_PREAMBLE_LEN, false, true, false, &mdltn)
    .unwrap();

  // -------------------------------------------------------------------------
  // Display (ST7789V2, 240x135) on SPI3.
  // PINS TO VERIFY against the Cardputer-Adv schematic:
  // SCK=G36, MOSI=G35, CS=G37, DC=G34, RST=G33, BL=G38.
  // -------------------------------------------------------------------------
  let disp_cs = Output::new(peripherals.GPIO37, Level::High, OutputConfig::default());
  let disp_dc = Output::new(peripherals.GPIO34, Level::Low, OutputConfig::default());
  let disp_rst = Output::new(peripherals.GPIO33, Level::High, OutputConfig::default());
  let _disp_bl = Output::new(peripherals.GPIO38, Level::High, OutputConfig::default());

  let disp_spi = Spi::new(
    peripherals.SPI3,
    SpiConfig::default()
      .with_frequency(Rate::from_mhz(40))
      .with_mode(Mode::_0),
  )
  .unwrap()
  .with_sck(peripherals.GPIO36)
  .with_mosi(peripherals.GPIO35);

  let disp_spi_dev = ExclusiveDevice::new(disp_spi, disp_cs, HalDelay::new()).unwrap();
  let disp_buffer = mk_static!([u8; 512], [0u8; 512]);
  let di = SpiInterface::new(disp_spi_dev, disp_dc, disp_buffer);
  let mut display = Builder::new(ST7789, di)
    .reset_pin(disp_rst)
    .display_size(240, 135)
    .orientation(Orientation::new().rotate(Rotation::Deg270))
    .invert_colors(ColorInversion::Inverted)
    .init(&mut HalDelay::new())
    .unwrap();
  let _ = display.clear(Rgb565::BLACK);

  // -------------------------------------------------------------------------
  // Wi-Fi STA + ESP-NOW (coexisting on one radio).
  // -------------------------------------------------------------------------
  let rng = Rng::new();
  let seed = ((rng.random() as u64) << 32) | (rng.random() as u64);

  let station_config = esp_radio::wifi::Config::Station(
    esp_radio::wifi::sta::StationConfig::default()
      .with_ssid(env!("SSID"))
      .with_password(alloc::string::String::from(env!("PASSWORD"))),
  );
  let (mut controller, interfaces) = esp_radio::wifi::new(
    peripherals.WIFI,
    esp_radio::wifi::ControllerConfig::default().with_initial_config(station_config),
  )
  .expect("wifi init failed");

  let wifi_interface = interfaces.station;
  let esp_now = interfaces.esp_now;

  let net_config = embassy_net::Config::dhcpv4(Default::default());
  let (stack, mut runner) = embassy_net::new(
    wifi_interface,
    net_config,
    mk_static!(
      embassy_net::StackResources<4>,
      embassy_net::StackResources::<4>::new()
    ),
    seed,
  );

  let (manager, sender, receiver) = esp_now.split();

  // -------------------------------------------------------------------------
  // Compose all concurrent work into one task via joins.
  // -------------------------------------------------------------------------
  let wifi_conn = async move {
    loop {
      match controller.connect_async().await {
        Ok(_) => {
          println!("wifi: connected");
          controller.wait_for_disconnect_async().await.ok();
          println!("wifi: disconnected");
        }
        Err(e) => println!("wifi: connect failed {:?}", e),
      }
      embassy_time::Timer::after(Duration::from_secs(5)).await;
    }
  };

  let net_fut = join4(
    runner.run(),
    wifi_conn,
    mqtt::mqtt_loop(stack),
    bridge::bridge_loop(),
  );
  let app_fut = join3(
    lora::lora_loop(lora, mdltn, rx_pp, tx_pp),
    espnow::espnow_loop(manager, sender, receiver),
    ui_loop(&mut display, &mut keyboard),
  );

  join(net_fut, app_fut).await;
}

/// UI task: render inbound traffic and handle keyboard composition.
async fn ui_loop<D, I>(display: &mut D, keyboard: &mut Keyboard<I>)
where
  D: DrawTarget<Color = Rgb565>,
  I: embedded_hal::i2c::I2c,
{
  let mut ui = Ui::new();
  ui.set_status("ready");
  ui.render(display);

  let mut scan = Ticker::every(Duration::from_millis(30));

  loop {
    match select(UI_IN.receive(), scan.next()).await {
      // A message to display.
      Either::First(line) => {
        ui.push_log(line);
        ui.render(display);
      }
      // Poll the keyboard.
      Either::Second(_) => {
        let mut dirty = false;
        while let Some(ev) = keyboard.poll() {
          if !ev.pressed {
            continue;
          }
          match decode_key(ev.key) {
            Some(KeyAction::Char(c)) => {
              let _ = ui.input.push(c);
              dirty = true;
            }
            Some(KeyAction::Backspace) => {
              ui.input.pop();
              dirty = true;
            }
            Some(KeyAction::Enter) => {
              if !ui.input.is_empty() {
                // The bridge echoes LOCAL_OUT back to UI_IN for display.
                send_local(&ui.input);
                ui.input.clear();
              }
              dirty = true;
            }
            None => {}
          }
        }
        if dirty {
          ui.render(display);
        }
      }
    }
  }
}

/// Push a locally-composed message onto the bridge for fan-out to all transports.
fn send_local(text: &str) {
  match Message::text(bridge::next_id(), Transport::Mqtt, bridge::CARD_ADDR, text) {
    Ok(msg) => {
      let _ = LOCAL_OUT.try_send(msg);
      println!("ui: sent \"{}\"", text);
    }
    Err(e) => println!("ui: compose failed {:?}", e),
  }
}
