//! Minimal TCA8418 keypad-scanner driver for the Cardputer-Adv keyboard.
//!
//! The Cardputer-Adv reads its 56-key matrix through a TCA8418 I2C keypad
//! controller (7-bit address 0x34) on the internal I2C bus (SDA=G8, SCL=G9).
//! This driver initialises the matrix and reads key events from the FIFO.
//!
//! NOTE: the raw key-code -> character mapping in [`decode_key`] depends on the
//! exact physical row/column wiring of the Cardputer-Adv, which must be
//! confirmed against the schematic. The scanning/FIFO logic below is generic
//! and correct; only the keymap table needs calibration for your unit.

use embedded_hal::i2c::I2c;

/// TCA8418 7-bit I2C address.
pub const TCA8418_ADDR: u8 = 0x34;

// Register map.
const REG_CFG: u8 = 0x01;
const REG_INT_STAT: u8 = 0x02;
const REG_KEY_LCK_EC: u8 = 0x03;
const REG_KEY_EVENT_A: u8 = 0x04;
const REG_KP_GPIO1: u8 = 0x1D;
const REG_KP_GPIO2: u8 = 0x1E;
const REG_KP_GPIO3: u8 = 0x1F;

/// A decoded keyboard action.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum KeyAction {
  Char(char),
  Backspace,
  Enter,
}

/// A raw key event from the FIFO.
#[derive(Debug, Clone, Copy)]
pub struct KeyEvent {
  /// Matrix key code (1..=80).
  pub key: u8,
  /// `true` = pressed, `false` = released.
  pub pressed: bool,
}

pub struct Keyboard<I> {
  i2c: I,
}

impl<I> Keyboard<I>
where
  I: I2c,
{
  /// Initialise the TCA8418 for full-matrix keypad scanning.
  pub fn new(mut i2c: I) -> Self {
    // Route all rows (R0-R7) and columns (C0-C9) into the keypad matrix.
    let _ = i2c.write(TCA8418_ADDR, &[REG_KP_GPIO1, 0xFF]); // rows R0-R7
    let _ = i2c.write(TCA8418_ADDR, &[REG_KP_GPIO2, 0xFF]); // cols C0-C7
    let _ = i2c.write(TCA8418_ADDR, &[REG_KP_GPIO3, 0x03]); // cols C8-C9
                                                            // Enable key-event interrupt bookkeeping.
    let _ = i2c.write(TCA8418_ADDR, &[REG_CFG, 0x01]);
    // Clear any pending interrupt status.
    let _ = i2c.write(TCA8418_ADDR, &[REG_INT_STAT, 0x0F]);
    Keyboard { i2c }
  }

  fn read_reg(&mut self, reg: u8) -> u8 {
    let mut buf = [0u8; 1];
    let _ = self.i2c.write_read(TCA8418_ADDR, &[reg], &mut buf);
    buf[0]
  }

  /// Poll for a single pending key event, if any.
  pub fn poll(&mut self) -> Option<KeyEvent> {
    let count = self.read_reg(REG_KEY_LCK_EC) & 0x0F;
    if count == 0 {
      return None;
    }
    let ev = self.read_reg(REG_KEY_EVENT_A);
    if ev == 0 {
      return None;
    }
    Some(KeyEvent {
      key: ev & 0x7F,
      pressed: (ev & 0x80) != 0,
    })
  }
}

/// Best-effort mapping of a TCA8418 matrix key code to an action.
///
/// CALIBRATE: this table is a placeholder. Determine the real codes for your
/// Cardputer-Adv by logging [`KeyEvent::key`] while pressing each physical key,
/// then fill in the mapping. Codes are `(row * 10) + col + 1`.
pub fn decode_key(key: u8) -> Option<KeyAction> {
  match key {
    // A few well-known control keys (placeholder positions).
    1 => Some(KeyAction::Enter),
    2 => Some(KeyAction::Backspace),
    3 => Some(KeyAction::Char(' ')),
    // Map a contiguous run of codes to a QWERTY-ish sequence as a stand-in.
    10..=35 => {
      let idx = (key - 10) as usize;
      const LETTERS: &[u8; 26] = b"abcdefghijklmnopqrstuvwxyz";
      Some(KeyAction::Char(LETTERS[idx] as char))
    }
    40..=49 => {
      let idx = (key - 40) as usize;
      const DIGITS: &[u8; 10] = b"0123456789";
      Some(KeyAction::Char(DIGITS[idx] as char))
    }
    _ => None,
  }
}
