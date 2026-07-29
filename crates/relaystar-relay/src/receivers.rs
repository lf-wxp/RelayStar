//! Receiver tracking: which peers are reachable on which transports.
//!
//! When the [`crate::Relay`] wants to send a unicast to node `A`, it consults
//! the [`ReceiverTable`] to decide which transports to fan out on. Entries can
//! be added explicitly ([`ReceiverTable::add`]) or learned automatically from
//! the arrival transport of incoming frames ([`ReceiverTable::learn`]).

use heapless::Vec as HVec;
use relaystar_proto::Transport;

/// Maximum number of transports we track per receiver.
///
/// Equal to `Transport::ALL.len()`; sized as `u8` bitmap for simplicity.
const MAX_TRANSPORTS: usize = 3;

/// A single row of the [`ReceiverTable`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Receiver {
  /// 6-byte node address (MAC-like).
  pub addr: [u8; 6],
  /// Bitmap of transports on which this peer is reachable.
  transports: TransportBitmap,
}

impl Receiver {
  /// Iterate over the transports this receiver is reachable on.
  pub fn transports(&self) -> impl Iterator<Item = Transport> + '_ {
    self.transports.iter()
  }

  /// Returns `true` if this receiver has been observed on `t`.
  pub fn reachable_via(&self, t: Transport) -> bool {
    self.transports.contains(t)
  }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
struct TransportBitmap(u8);

impl TransportBitmap {
  const fn bit(t: Transport) -> u8 {
    match t {
      Transport::Lora => 1 << 0,
      Transport::Mqtt => 1 << 1,
      Transport::EspNow => 1 << 2,
    }
  }

  fn insert(&mut self, t: Transport) -> bool {
    let b = Self::bit(t);
    let was = self.0 & b != 0;
    self.0 |= b;
    !was
  }

  fn remove(&mut self, t: Transport) -> bool {
    let b = Self::bit(t);
    let was = self.0 & b != 0;
    self.0 &= !b;
    was
  }

  const fn contains(self, t: Transport) -> bool {
    self.0 & Self::bit(t) != 0
  }

  const fn is_empty(self) -> bool {
    self.0 == 0
  }

  fn iter(self) -> impl Iterator<Item = Transport> {
    Transport::ALL
      .into_iter()
      .filter(move |t| self.contains(*t))
  }
}

/// Fixed-capacity table mapping node addresses to the set of transports they
/// have been observed on.
///
/// `N` is the maximum number of tracked peers.
pub struct ReceiverTable<const N: usize> {
  entries: HVec<Receiver, N>,
}

impl<const N: usize> Default for ReceiverTable<N> {
  fn default() -> Self {
    Self::new()
  }
}

impl<const N: usize> ReceiverTable<N> {
  /// Create an empty table.
  pub const fn new() -> Self {
    ReceiverTable {
      entries: HVec::new(),
    }
  }

  /// Number of tracked peers.
  pub fn len(&self) -> usize {
    self.entries.len()
  }

  /// Returns `true` when no peers are tracked.
  pub fn is_empty(&self) -> bool {
    self.entries.is_empty()
  }

  /// Iterate over all tracked receivers.
  pub fn iter(&self) -> impl Iterator<Item = &Receiver> {
    self.entries.iter()
  }

  /// Look up the transports reachable for `addr`.
  pub fn lookup(&self, addr: [u8; 6]) -> Option<&Receiver> {
    self.entries.iter().find(|r| r.addr == addr)
  }

  /// Assert that `addr` is reachable via `t`, creating a new row if needed.
  ///
  /// Returns:
  /// - `Ok(true)` if `t` was newly added (for either an existing or a new row).
  /// - `Ok(false)` if `t` was already known for this addr.
  ///
  /// # Errors
  /// Returns [`ReceiverTableFull`](crate::RelayError::ReceiverTableFull)
  /// if the addr is new and the table is at capacity.
  pub fn add(&mut self, addr: [u8; 6], t: Transport) -> Result<bool, crate::RelayError> {
    if let Some(row) = self.entries.iter_mut().find(|r| r.addr == addr) {
      return Ok(row.transports.insert(t));
    }
    let mut bitmap = TransportBitmap::default();
    bitmap.insert(t);
    let row = Receiver {
      addr,
      transports: bitmap,
    };
    self
      .entries
      .push(row)
      .map_err(|_| crate::RelayError::ReceiverTableFull)?;
    Ok(true)
  }

  /// Auto-learn from an inbound frame. Equivalent to [`Self::add`] but
  /// swallows a full-table error to keep the RX path infallible; the caller
  /// can inspect the return value if desired.
  pub fn learn(&mut self, addr: [u8; 6], t: Transport) -> bool {
    self.add(addr, t).unwrap_or(false)
  }

  /// Remove `t` from the row for `addr`. If the row becomes empty, it is
  /// dropped entirely. Returns `true` if a change was made.
  pub fn remove_transport(&mut self, addr: [u8; 6], t: Transport) -> bool {
    let idx = self.entries.iter().position(|r| r.addr == addr);
    let Some(i) = idx else {
      return false;
    };
    let changed = self.entries[i].transports.remove(t);
    if self.entries[i].transports.is_empty() {
      let _ = self.entries.swap_remove(i);
    }
    changed
  }

  /// Remove every trace of `addr` from the table. Returns `true` if the entry
  /// existed.
  pub fn remove(&mut self, addr: [u8; 6]) -> bool {
    if let Some(i) = self.entries.iter().position(|r| r.addr == addr) {
      let _ = self.entries.swap_remove(i);
      return true;
    }
    false
  }

  /// Clear the whole table.
  pub fn clear(&mut self) {
    self.entries.clear();
  }

  /// Materialise the set of transports for `addr` into a small stack buffer.
  ///
  /// Returns `None` if `addr` is unknown.
  pub fn transports_for(&self, addr: [u8; 6]) -> Option<HVec<Transport, MAX_TRANSPORTS>> {
    let row = self.lookup(addr)?;
    let mut out: HVec<Transport, MAX_TRANSPORTS> = HVec::new();
    for t in row.transports() {
      let _ = out.push(t);
    }
    Some(out)
  }
}
