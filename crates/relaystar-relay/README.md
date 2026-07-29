# relaystar-relay

Transport-agnostic mesh relay engine for **RelayStar**. Handles automatic
fragmentation & reassembly, deduplication, receiver tracking, and unicast /
broadcast fan-out across LoRa, ESP-NOW and MQTT — all in a single `no_std`
crate that both firmware and the host broker can depend on.

- **Wire protocol**: [`relaystar-proto`](../relaystar-proto)
- **This crate**: engine + transport adapters
- **Consumers**: `lora-node-fw`, `cardputer-fw`, `broker`

---

## Why this crate exists

Three transports means three payload MTUs:

| Transport | Practical per-frame limit |
| --------- | ------------------------- |
| LoRa (SX126x, SF10/BW250) | ~180 B (post postcard overhead) |
| ESP-NOW v1 | ~200 B                     |
| MQTT       | multi-KB (broker-bound)    |

Without a common layer, the mesh would either (a) cap everything to the LoRa
MTU, wasting the other transports' capacity, or (b) silently drop oversize
messages when they hit a LoRa hop. `relaystar-relay` solves both by
**fragmenting on the fly per target transport** and **reassembling on
receive** — with no application-level changes required.

Additional guarantees:

- **Deduplication**: a rolling seen-cache prevents mesh loops even with
  aggressive fan-out.
- **Receiver awareness**: for `Unicast`, the engine only fans out on
  transports where the peer has actually been observed (auto-learned from
  inbound frames or explicitly registered).
- **Transport-native addressing**: each frame is emitted using the target
  transport's own unicast/broadcast facility (ESP-NOW MAC, MQTT topic,
  LoRa link-layer address).

---

## Getting started

### 1. Add the dependency

```toml
[dependencies]
relaystar-relay = { path = "../relaystar-relay", default-features = false }
```

Enable the `std` feature only when building for a host (e.g. `broker`).

### 2. Implement the hardware trait for each transport you use

There are two paths, pick per transport:

**Path A — use a bundled backend (recommended).** Enable a feature and hand
the bundled adapter a fully-initialised driver handle.

```toml
[dependencies]
relaystar-relay = { path = "...", default-features = false, features = ["lora", "mqtt"] }
```

```rust,ignore
use relaystar_relay::ports::lora::{SxLoraPort, LoraModulationParams};
use relaystar_relay::ports::mqtt::{Mqtt311Client, MqttPort, PacketId, Topic};

// LoRa — you still init SPI/GPIO/lora-phy; the port owns everything after.
let lora_port = SxLoraPort::new(
    lora_handle,
    LoraModulationParams {
        frequency_hz: 868_000_000,
        spreading_factor: 10,
        bandwidth_khz: 250,
        coding_rate_denom: 8,
        preamble_symbols: 4,
        tx_power_dbm: 20,
    },
    relaystar_proto::MAX_FRAME as u8,
).await?;

// MQTT — connect the socket yourself, hand it over.
let mut mqtt = Mqtt311Client::new(socket, "relaystar-node", 30);
mqtt.connect().await?;
mqtt.subscribe(Topic::BROADCAST, PacketId(1)).await?;
let mqtt_port = MqttPort::new(mqtt);
```

**Path B — bring your own driver.** Implement the (tiny) hardware trait
directly. This is the only supported route for ESP-NOW, and it's also
useful for custom LoRa chips or MQTT libraries.

```rust,ignore
use relaystar_relay::ports::lora::{LoraPort, LoraRadio};

struct MyLora { /* lora-phy handle + packet params */ }

impl LoraRadio for MyLora {
    type Error = MyErr;
    async fn transmit(&mut self, frame: &[u8]) -> Result<(), MyErr> {
        // prepare_for_tx + tx() on your lora-phy handle
        Ok(())
    }
}

let mut lora_port = LoraPort::new(MyLora { /* ... */ });
```

See:

- [`ports/lora.rs`](src/ports/lora.rs) — `LoraRadio` trait (1 method) or bundled `SxLoraPort` (`feature = "lora"`)
- [`ports/espnow.rs`](src/ports/espnow.rs) — `EspNowRadio` trait (3 methods; no bundled impl — `esp-radio` is platform-locked)
- [`ports/mqtt.rs`](src/ports/mqtt.rs) — `MqttClient` trait (1 method) or bundled `Mqtt311Client` (`feature = "mqtt"`)

### 3. Create a `Relay` and register your transports

```rust,ignore
use relaystar_relay::{Relay, Destination};
use relaystar_proto::{MsgKind, Transport};

const SELF_ADDR: [u8; 6] = [0x02, 0, 0, 0, 0, 1];
const ID_BASE:   u32     = 0x0100_0000;

// NR=16 tracked peers, NA=4 concurrent reassembly slots, NF=32 frags per group.
let mut relay: Relay<16, 4, 32> = Relay::new(SELF_ADDR, ID_BASE);

relay.register_port(Transport::Lora)?;
relay.register_port(Transport::EspNow)?;

// Wrap your drivers.
let mut lora   = LoraPort::new(my_lora_driver);
let mut espnow = EspNowPort::new(my_espnow_driver);
```

### 4. Send a message — automatic fragmentation & fan-out

There are two APIs; pick whichever fits your executor model.

#### 4a. Direct async API — "one call and it's sent"

```rust,ignore
// Send a 2 KB blob unicast to a known peer. Fragmentation happens
// automatically per transport; each transport's frames use its native
// unicast facility.
relay
  .send(&mut espnow, MsgKind::Telemetry, &big_blob,
        Destination::Unicast([0x02, 0, 0, 0, 0, 2]))
  .await?;

// Broadcast a text ping on every registered transport.
relay
  .send(&mut lora, MsgKind::Text, b"hello mesh", Destination::Broadcast)
  .await?;
```

For multi-transport fan-out call `send` once per port, or use the planner API
below.

#### 4b. Planner API — "give me the frames, I'll dispatch"

Ideal for firmware that already routes outbound frames through embassy
`Channel`s (as `cardputer-fw/src/bridge.rs` does):

```rust,ignore
let frames = relay.plan_send(MsgKind::Text, b"hi",
                             Destination::Unicast(peer))?;
for f in frames {
    match f.transport {
        Transport::Lora   => LORA_OUT.send((f.addr, f.message)).await,
        Transport::EspNow => ESPNOW_OUT.send((f.addr, f.message)).await,
        Transport::Mqtt   => MQTT_OUT.send((f.addr, f.message)).await,
    }
}
```

### 5. Handle received frames

```rust,ignore
match relay.ingest(Transport::Lora, &raw_bytes)? {
    IngestOutcome::Complete(msg)  => app_channel.send(msg).await,
    IngestOutcome::NotForMe(msg)  => { relay.forward(&mut espnow, msg).await?; }
    IngestOutcome::Buffered       => {}   // waiting for more fragments
    IngestOutcome::Duplicate      => {}   // already saw this frame id
    IngestOutcome::Dropped(_)     => {}   // logging opportunity
}
```

The receiver table is updated automatically from the `from` field on every
non-broadcast frame. Peers you haven't yet heard from can be added
explicitly:

```rust,ignore
relay.add_receiver([0x02, 0, 0, 0, 0, 2], Transport::EspNow)?;
```

---

## Design notes

- **`no_std` first**. All buffers are `heapless::Vec` / fixed-size arrays.
  The three const generics on `Relay<NR, NA, NF>` are your only knobs; pick
  them once and forget.
- **Object-safety avoidance**. Adapters expose `TransportPort` (an
  `async fn in trait`) rather than a boxed trait object. This keeps futures
  inline and executor-agnostic. If you need `dyn`-style dispatch, use
  `plan_send` / `plan_forward` and push onto typed channels — no dispatch
  overhead, and each transport's send future can live in its own task.
- **Deduplication key**. For fragments we dedup on `id ^ (seq << 16)` so
  distinct slices of the same message aren't collapsed into one, while true
  retransmissions still get suppressed.
- **Non-exhaustive enums** ([`RelayError`](src/error.rs),
  [`Transport`](../relaystar-proto/src/lib.rs), etc.) leave room for future
  variants without breaking downstream matches.

---

## Migrating an existing firmware bridge

Concretely, `crates/cardputer-fw/src/bridge.rs` currently does:

```rust,ignore
if let Some(relayed) = msg.prepared_for_relay() {
    for target in Transport::ALL {
        if target == msg.origin { continue; }
        enqueue(target, relayed.clone());
    }
}
```

With this crate that becomes:

```rust,ignore
for t in Transport::ALL {
    if t == msg.origin { continue; }
    let frames = relay.plan_forward(msg.clone(), t)?;
    for f in frames { enqueue(t, f.message); }
}
```

The `enqueue` (channel send) side is untouched; you get MTU-aware
fragmentation for free.

---

## Feature flags

The crate is `no_std` by default and pulls **zero** platform-specific
dependencies. Bundled transport implementations opt-in via features so
downstream crates only pay for what they use.

| Feature | Adds | Extra deps | Typical user |
|---|---|---|---|
| `default` | Nothing extra — you get the mesh engine + trait-only adapters (`LoraPort<R>`, `EspNowPort<R>`, `MqttPort<C>`). | none | Broker / any host tool that reuses the wire types. |
| `lora` | Concrete [`SxLoraPort<RK, DLY>`](src/ports/lora.rs) built on `lora-phy`; drives `prepare_for_tx → tx → sleep` and the RX side for you. | `lora-phy`, `embedded-hal-async` (both `no_std`) | LoRa firmware nodes. |
| `mqtt` | Concrete [`Mqtt311Client<S>`](src/ports/mqtt.rs) — a QoS-0 MQTT 3.1.1 client over `embedded-io-async::{Read, Write}`. Speaks the same wire protocol as the `rumqttd` v4.1 listener on port 1883. | `embedded-io-async` (`no_std`) | Firmware that talks MQTT (e.g. `cardputer-fw`). |
| `std` | `std::error::Error` impls on every error type. | none | Host-side callers (e.g. the broker). |

### Enabling from Cargo

```toml
# LoRa-only single-transport node:
relaystar-relay = { path = "...", default-features = false, features = ["lora"] }

# Cardputer terminal: LoRa + MQTT bundled; ESP-NOW stays trait-only (users
# supply their `EspNowRadio` impl themselves since `esp-radio` is
# platform-locked).
relaystar-relay = { path = "...", default-features = false, features = ["lora", "mqtt"] }

# Broker or any host crate that only needs the wire types:
relaystar-relay = { path = "...", features = ["std"] }
```

### Why ESP-NOW is *not* bundled

`esp-radio` is chip-family-specific (each of `esp32s3`, `esp32c3`, `esp32c6`
requires a distinct feature) and evolves faster than this crate is likely to
track. Users implement [`EspNowRadio`](src/ports/espnow.rs) (three methods)
in ~30 lines against whichever `esp-radio` version their firmware pins. See
[`crates/cardputer-fw/src/espnow.rs`](../cardputer-fw/src/espnow.rs) for a
worked example.

### Why `rumqttc` is not the MQTT backend

The upstream `rumqttc` crate hard-depends on `tokio` and `std::net`, so it
cannot compile for `xtensa-esp32s3-none-elf` or any bare-metal target.
[`Mqtt311Client`](src/ports/mqtt.rs) is the `no_std` alternative, wire-
compatible with `rumqttd` on the broker side.

---

## License

MIT — see the workspace `LICENSE` file.
