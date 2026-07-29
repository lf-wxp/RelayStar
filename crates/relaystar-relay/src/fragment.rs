//! Automatic fragmentation and reassembly.
//!
//! The [`Fragmenter`] slices a large logical payload into a sequence of
//! [`Message`]s whose per-frame payload fits the target transport's
//! [`Transport::max_payload`]. The [`Reassembler`] does the inverse on the
//! receive side: it collects fragments into complete messages, with a bounded
//! number of concurrently-tracked groups and a per-group fragment cap.
//!
//! Both are pure, allocation-free logic — they perform no I/O.

use heapless::Vec as HVec;
use relaystar_proto::{Fragment, MAX_PAYLOAD, Message, MsgKind, Transport};

use crate::error::RelayError;

/// Absolute upper bound on fragments per logical message.
///
/// Chosen so that a single big payload (e.g. 4 KB) can be split across LoRa
/// (~180 B/frame) with headroom, and so that per-group state stays tiny in
/// firmware.
pub const MAX_FRAGMENTS: usize = 32;

/// Splits a logical [`Message`] into one or more transport-sized frames.
///
/// The returned iterator yields at least one item. When the payload fits in
/// a single frame the fragmenter is *zero-copy in spirit*: it emits the input
/// message with `frag = None`.
pub struct Fragmenter;

impl Fragmenter {
  /// Split `msg` into frames that each fit inside `target.max_payload()`.
  ///
  /// The `group_id` is used to correlate fragments on the receiver. Callers
  /// usually pass `msg.id` here, which keeps the id stable across the mesh.
  ///
  /// # Errors
  /// Returns [`RelayError::PayloadUnfragmentable`] when the payload cannot be
  /// split within [`MAX_FRAGMENTS`] slices given the target MTU, or when the
  /// target MTU is zero.
  pub fn split(
    msg: Message,
    target: Transport,
    group_id: u32,
  ) -> Result<HVec<Message, MAX_FRAGMENTS>, RelayError> {
    // The effective per-frame slice size is bounded by *both* the transport
    // MTU and MAX_PAYLOAD (the on-wire message can never carry more than
    // MAX_PAYLOAD bytes anyway).
    let mtu = core::cmp::min(target.max_payload(), MAX_PAYLOAD);
    if mtu == 0 {
      return Err(RelayError::PayloadUnfragmentable);
    }

    let mut out: HVec<Message, MAX_FRAGMENTS> = HVec::new();

    // Fast path: message already fits.
    if msg.payload.len() <= mtu && msg.frag.is_none() {
      // heapless::Vec::push takes ownership on error; safe because len == 0.
      out
        .push(msg)
        .map_err(|_| RelayError::PayloadUnfragmentable)?;
      return Ok(out);
    }

    // If it's already fragmented we don't re-fragment; treat as single frame.
    if msg.frag.is_some() {
      out
        .push(msg)
        .map_err(|_| RelayError::PayloadUnfragmentable)?;
      return Ok(out);
    }

    let total = msg.payload.len().div_ceil(mtu);
    if total > MAX_FRAGMENTS || total > u16::MAX as usize {
      return Err(RelayError::PayloadUnfragmentable);
    }

    for seq in 0..total {
      let start = seq * mtu;
      let end = core::cmp::min(start + mtu, msg.payload.len());
      let slice = &msg.payload[start..end];

      // Build a per-slice payload. `slice.len() <= mtu <= MAX_PAYLOAD` so
      // this cannot fail as long as MAX_PAYLOAD >= largest transport MTU.
      let mut payload: HVec<u8, MAX_PAYLOAD> = HVec::new();
      payload
        .extend_from_slice(slice)
        .map_err(|_| RelayError::PayloadUnfragmentable)?;

      let frame = Message {
        id: msg.id,
        ttl: msg.ttl,
        origin: msg.origin,
        from: msg.from,
        to: msg.to,
        kind: msg.kind,
        payload,
        frag: Some(Fragment {
          group_id,
          seq: seq as u16,
          total: total as u16,
        }),
      };
      out
        .push(frame)
        .map_err(|_| RelayError::PayloadUnfragmentable)?;
    }
    Ok(out)
  }
}

/// The outcome of ingesting a single (possibly fragmented) message.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ReassembleOutcome {
  /// The message had `frag = None`; return it as-is to the caller.
  Passthrough(Message),
  /// A fragment was buffered; more slices are still needed.
  Buffered,
  /// All slices have been received; the reassembled message is returned.
  Complete(Message),
  /// The fragment was rejected (see [`RejectReason`] for why).
  Dropped(RejectReason),
}

/// Why a fragment was dropped by [`Reassembler::ingest`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum RejectReason {
  /// Not enough free assembly slots to track a new group.
  NoSlot,
  /// `seq >= total` or `total == 0`, header is inconsistent.
  BadHeader,
  /// A duplicate fragment for a slot that already has this `seq`.
  DuplicateSlice,
  /// Concatenated payload would exceed [`MAX_PAYLOAD`].
  PayloadTooLarge,
}

/// Fixed-capacity reassembly buffer.
///
/// `SLOTS` = number of concurrent groups that can be tracked; `PER_SLOT` =
/// fragments per group upper bound (recommend >= [`MAX_FRAGMENTS`]).
pub struct Reassembler<const SLOTS: usize, const PER_SLOT: usize> {
  slots: [Slot<PER_SLOT>; SLOTS],
}

struct Slot<const PER_SLOT: usize> {
  in_use: bool,
  group_id: u32,
  total: u16,
  received: u16,
  seen_mask: u64,
  fragments: [HVec<u8, MAX_PAYLOAD>; PER_SLOT],
  template: Option<Message>,
}

impl<const PER_SLOT: usize> Slot<PER_SLOT> {
  const EMPTY_FRAGMENT: HVec<u8, MAX_PAYLOAD> = HVec::new();

  const fn new() -> Self {
    Slot {
      in_use: false,
      group_id: 0,
      total: 0,
      received: 0,
      seen_mask: 0,
      fragments: [Self::EMPTY_FRAGMENT; PER_SLOT],
      template: None,
    }
  }

  fn reset(&mut self) {
    self.in_use = false;
    self.group_id = 0;
    self.total = 0;
    self.received = 0;
    self.seen_mask = 0;
    for f in &mut self.fragments {
      f.clear();
    }
    self.template = None;
  }
}

impl<const SLOTS: usize, const PER_SLOT: usize> Default for Reassembler<SLOTS, PER_SLOT> {
  fn default() -> Self {
    Self::new()
  }
}

impl<const SLOTS: usize, const PER_SLOT: usize> Reassembler<SLOTS, PER_SLOT> {
  /// Create an empty reassembler.
  pub const fn new() -> Self {
    // Manual construction; can't use array-init macros here because Slot
    // owns HVecs and is not Copy.
    let slots: [Slot<PER_SLOT>; SLOTS] = {
      // SAFETY: MaybeUninit init pattern — but const fn can't do that.
      // Instead we rely on Slot::new() being `const` and cheap.
      [const { Slot::<PER_SLOT>::new() }; SLOTS]
    };
    Reassembler { slots }
  }

  /// Ingest a message. If it is not fragmented, returns
  /// [`ReassembleOutcome::Passthrough`]. Otherwise buffers and, when the
  /// group is complete, returns [`ReassembleOutcome::Complete`] with the
  /// reassembled message.
  pub fn ingest(&mut self, msg: Message) -> ReassembleOutcome {
    let Some(frag) = msg.frag else {
      return ReassembleOutcome::Passthrough(msg);
    };

    // Validate header up front (also guards seen_mask which is 64 bits wide).
    if frag.total == 0
      || frag.seq >= frag.total
      || frag.total as usize > PER_SLOT
      || frag.total > 64
    {
      return ReassembleOutcome::Dropped(RejectReason::BadHeader);
    }

    // Find existing slot for this group, else claim a free one.
    let idx = match self.find_or_claim(frag.group_id, frag.total, &msg) {
      Some(i) => i,
      None => return ReassembleOutcome::Dropped(RejectReason::NoSlot),
    };
    let slot = &mut self.slots[idx];

    let bit = 1u64 << frag.seq;
    if slot.seen_mask & bit != 0 {
      return ReassembleOutcome::Dropped(RejectReason::DuplicateSlice);
    }

    // Copy this slice into its indexed bucket.
    let bucket = &mut slot.fragments[frag.seq as usize];
    bucket.clear();
    if bucket.extend_from_slice(&msg.payload).is_err() {
      slot.reset();
      return ReassembleOutcome::Dropped(RejectReason::PayloadTooLarge);
    }
    slot.seen_mask |= bit;
    slot.received += 1;

    if slot.received < slot.total {
      return ReassembleOutcome::Buffered;
    }

    // All slices in: concatenate.
    let mut assembled: HVec<u8, MAX_PAYLOAD> = HVec::new();
    for seq in 0..slot.total as usize {
      if assembled.extend_from_slice(&slot.fragments[seq]).is_err() {
        slot.reset();
        return ReassembleOutcome::Dropped(RejectReason::PayloadTooLarge);
      }
    }

    let template = slot.template.take().unwrap_or_else(|| Message {
      id: msg.id,
      ttl: msg.ttl,
      origin: msg.origin,
      from: msg.from,
      to: msg.to,
      kind: msg.kind,
      payload: HVec::new(),
      frag: None,
    });
    slot.reset();

    ReassembleOutcome::Complete(Message {
      payload: assembled,
      frag: None,
      ..template
    })
  }

  fn find_or_claim(&mut self, group_id: u32, total: u16, msg: &Message) -> Option<usize> {
    // Existing.
    for (i, slot) in self.slots.iter().enumerate() {
      if slot.in_use && slot.group_id == group_id && slot.total == total {
        return Some(i);
      }
    }
    // Claim free.
    for (i, slot) in self.slots.iter_mut().enumerate() {
      if !slot.in_use {
        slot.in_use = true;
        slot.group_id = group_id;
        slot.total = total;
        slot.received = 0;
        slot.seen_mask = 0;
        slot.template = Some(Message {
          id: msg.id,
          ttl: msg.ttl,
          origin: msg.origin,
          from: msg.from,
          to: msg.to,
          kind: msg.kind,
          payload: HVec::new(),
          frag: None,
        });
        return Some(i);
      }
    }
    None
  }

  /// Explicitly drop the group tracked for `group_id`, if any. Useful for
  /// timeout handling driven by an external timer.
  pub fn evict(&mut self, group_id: u32) -> bool {
    for slot in self.slots.iter_mut() {
      if slot.in_use && slot.group_id == group_id {
        slot.reset();
        return true;
      }
    }
    false
  }

  /// Number of assembly slots currently in use.
  pub fn in_flight(&self) -> usize {
    self.slots.iter().filter(|s| s.in_use).count()
  }
}

// Convenience: make it easy to build a big text/telemetry message and let the
// caller feed it to `Fragmenter`.
impl Fragmenter {
  /// Build a not-yet-fragmented [`Message`] whose payload is potentially
  /// larger than [`MAX_PAYLOAD`]. Callers that already hold a
  /// [`heapless::Vec`] should construct the message directly.
  ///
  /// # Errors
  /// Returns [`RelayError::Protocol`] with
  /// [`relaystar_proto::ProtoError::PayloadTooLarge`] if the payload exceeds
  /// what a single [`Message::payload`] can hold *for the pre-fragmentation
  /// message*.
  pub fn prepare(
    id: u32,
    ttl: u8,
    origin: Transport,
    from: [u8; 6],
    to: [u8; 6],
    kind: MsgKind,
    payload: &[u8],
  ) -> Result<Message, RelayError> {
    let mut buf: HVec<u8, MAX_PAYLOAD> = HVec::new();
    buf
      .extend_from_slice(payload)
      .map_err(|_| RelayError::Protocol(relaystar_proto::ProtoError::PayloadTooLarge))?;
    Ok(Message {
      id,
      ttl,
      origin,
      from,
      to,
      kind,
      payload: buf,
      frag: None,
    })
  }
}
