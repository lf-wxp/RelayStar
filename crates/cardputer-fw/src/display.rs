//! Terminal UI state and rendering for the Cardputer ST7789V2 (240x135).
//!
//! Keeps a small scrolling log of recent messages plus the current input line,
//! and renders them with embedded-graphics onto any `DrawTarget<Color=Rgb565>`.

use embedded_graphics::{
  mono_font::{ascii::FONT_6X10, MonoTextStyle},
  pixelcolor::Rgb565,
  prelude::*,
  text::{Baseline, Text},
};

use crate::bridge::DisplayLine;
use relaystar_proto::Transport;

const LOG_CAPACITY: usize = 8;

/// Screen state: message log + input buffer.
pub struct Ui {
  log: heapless::Deque<DisplayLine, LOG_CAPACITY>,
  pub input: heapless::String<64>,
  status: heapless::String<32>,
}

impl Default for Ui {
  fn default() -> Self {
    Self::new()
  }
}

impl Ui {
  pub fn new() -> Self {
    let mut status: heapless::String<32> = heapless::String::new();
    let _ = status.push_str("booting...");
    Ui {
      log: heapless::Deque::new(),
      input: heapless::String::new(),
      status,
    }
  }

  pub fn set_status(&mut self, s: &str) {
    self.status.clear();
    for c in s.chars().take(31) {
      let _ = self.status.push(c);
    }
  }

  /// Append a line to the scrolling log (evicting the oldest if full).
  pub fn push_log(&mut self, line: DisplayLine) {
    if self.log.is_full() {
      let _ = self.log.pop_front();
    }
    let _ = self.log.push_back(line);
  }

  fn color_for(origin: Transport) -> Rgb565 {
    match origin {
      Transport::Lora => Rgb565::CSS_ORANGE,
      Transport::Mqtt => Rgb565::CSS_CYAN,
      Transport::EspNow => Rgb565::CSS_LIGHT_GREEN,
    }
  }

  /// Redraw the whole screen.
  pub fn render<D>(&self, display: &mut D)
  where
    D: DrawTarget<Color = Rgb565>,
  {
    let _ = display.clear(Rgb565::BLACK);

    let title_style = MonoTextStyle::new(&FONT_6X10, Rgb565::WHITE);
    let _ =
      Text::with_baseline("RelayStar", Point::new(2, 0), title_style, Baseline::Top).draw(display);
    let _ = Text::with_baseline(
      self.status.as_str(),
      Point::new(80, 0),
      MonoTextStyle::new(&FONT_6X10, Rgb565::CSS_GRAY),
      Baseline::Top,
    )
    .draw(display);

    // Log lines.
    let mut y = 14;
    for line in self.log.iter() {
      let style = MonoTextStyle::new(&FONT_6X10, Self::color_for(line.origin));
      let mut buf: heapless::String<56> = heapless::String::new();
      let _ = buf.push('[');
      for c in line.origin.as_str().chars() {
        let _ = buf.push(c);
      }
      let _ = buf.push(']');
      let _ = buf.push(' ');
      for c in line.text.chars() {
        let _ = buf.push(c);
      }
      let _ =
        Text::with_baseline(buf.as_str(), Point::new(2, y), style, Baseline::Top).draw(display);
      y += 12;
    }

    // Input line at the bottom.
    let input_style = MonoTextStyle::new(&FONT_6X10, Rgb565::YELLOW);
    let mut input_buf: heapless::String<66> = heapless::String::new();
    let _ = input_buf.push('>');
    let _ = input_buf.push(' ');
    for c in self.input.chars() {
      let _ = input_buf.push(c);
    }
    let _ = Text::with_baseline(
      input_buf.as_str(),
      Point::new(2, 124),
      input_style,
      Baseline::Top,
    )
    .draw(display);
  }
}
