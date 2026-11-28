# Wireless: 2.4 GHz dongle (primary) and Bluetooth LE (optional)

nRF52840 keyboards can connect without a cable in two ways:

| Backend | Link | Report rate to the host | Needs |
|---|---|---|---|
| **2.4 GHz (default)** | Nordic Enhanced ShockBurst (ESB) to a USB receiver | ≈1 kHz (the receiver's USB full-speed polling); a radio packet with ACK takes ≈0.3 ms (estimate) | nRF52840 Dongle (PCA10059) running `libhmk-nrf52840-dongle` |
| Bluetooth LE | HOGP, TrouBLE + Nordic SoftDevice Controller | ≤133 Hz (7.5 ms minimum connection interval) | nothing (host Bluetooth) |

Only one radio stack runs at a time, because both need the RADIO and TIMER0 peripherals. The
keyboard picks the backend at boot from its stored link settings. Switching backends saves the
new choice and reboots (≈1 s, including the 500 ms matrix calibration).

## Keys

| Keycode | Value | Action |
|---|---|---|
| `BT_24G` | `0xD9` | Use the 2.4 GHz dongle link (wireless output; reboots if the backend changes) |
| `BT_BLE` | `0xD7` | Use Bluetooth LE (wireless output; reboots if the backend changes) |
| `BT_TOGG` | `0xD5` | Cycle the output: Auto → wireless → USB → Auto |
| `BT_USB` | `0xD6` | Force USB output |
| `BT_PAIR` | `0xD8` | 2.4 GHz: open the 30 s pairing window. BLE: drop the connection and advertise |

- **`BT_24G` is a libhmk Rust extension.** The C firmware and `hmkconf` don't know it yet, so set
  it in `keyboard.json` (as on the nRF52840-DK layer 1) until `hmkconf` learns the keycode.
- **Persistence:** the output transport (Auto/USB/wireless), the backend and the paired dongle
  address are stored in a dedicated flash page just below the wear-leveling area. They're kept
  outside `eeconfig`, so the configurator's memory layout is unchanged.

## Pairing a keyboard with a dongle

1. Plug in the dongle and press **SW1** (the small button next to the USB connector). The blue
   LED blinks while the 30 s pairing window is open.
2. On the keyboard, select 2.4 GHz (`BT_24G`) if it isn't already, then press `BT_PAIR`.
3. The keyboard receives the dongle's address, stores it and reboots. The dongle's green LED
   lights up as soon as the keyboard is linked.

Each dongle has a unique address derived from its chip ID. The keyboard talks only to the
dongle it paired with, until it is paired again.

## Building and flashing the dongle

The dongle firmware is built for a keyboard: it takes the USB identity (VID/PID, manufacturer)
and the RF channel from the same `keyboard.json`. That keyboard must have a `wireless.esb`
section.

```bash
LIBHMK_KEYBOARD=nrf52840-dk-he16 cargo build -p libhmk-nrf52840-dongle --target thumbv7em-none-eabihf --release
```

The memory layout keeps the PCA10059's stock Nordic MBR and open USB DFU bootloader:
- **Application:** 0x1000–0xE0000.
- **RAM:** from 0x20000008.

No debug probe is needed. Flash it as a DFU package with
[`nrfutil`](https://www.nordicsemi.com/Products/Development-tools/nRF-Util) (CI also uploads
`firmware.hex` for each dongle build):

```bash
llvm-objcopy -O ihex target/thumbv7em-none-eabihf/release/libhmk-nrf52840-dongle dongle.hex
```

```bash
nrfutil pkg generate --hw-version 52 --sd-req 0x00 --application dongle.hex --application-version 1 dongle.zip
```

Press the dongle's sideways **RESET** button; the red LED pulses in bootloader mode. Then:

```bash
nrfutil dfu usb-serial -pkg dongle.zip -p /dev/ttyACM0
```

[nRF Connect Programmer](https://www.nordicsemi.com/Products/Development-tools/nRF-Connect-for-Desktop)
can flash `dongle.hex` directly, too.

## keyboard.json

```json
"bluetooth": { "enabled": true, "device_name": "My Keyboard" },
"wireless": {
  "default": "esb",
  "esb": { "channel": 40, "tx_power": 0 }
}
```

| Field | Default | Notes |
|---|---|---|
| `wireless.default` | `"esb"` | Backend used before anything is stored (`"esb"` or `"ble"`) |
| `wireless.esb.enabled` | `true` | |
| `wireless.esb.channel` | `40` | 2400 MHz + channel (0–100). Keyboard and dongle must match: build both from the same file |
| `wireless.esb.tx_power` | `0` | dBm: −40, −30, −20, −16, −12, −8, −4, 0, 2–8 |

- **BLE stays optional:** leave out `bluetooth` (or set `"enabled": false`) for a 2.4 GHz-only
  keyboard.
- **Battery settings:** the battery pin and the idle/sleep timeouts under `bluetooth` apply to
  both backends.

## How the 2.4 GHz link works

The keyboard is the ESB primary transmitter (PTX), the dongle the primary receiver (PRX). Every
packet is acknowledged, and the dongle can only send data inside ACK payloads.

```text
keyboard (PTX)                                  dongle (PRX)                     host
  HID state  ──────── ESB packet ─────────────▶  forward unchanged ── USB HID ─▶
  heartbeat  ──────── ESB packet ─────────────▶  (keeps failsafe armed)
             ◀─────── ACK payload: raw HID ────  ◀── raw HID OUT (configurator)
  raw resp.  ──────── ESB packet ─────────────▶  ─── raw HID IN ──────────────▶
```

- **State, not events:** each HID message carries the complete current report. A lost packet is
  repaired by sending the latest state again: failed frames are re-queued beneath any newer
  state (`libhmk_link::PtxScheduler`). ESB retransmits up to 8 times before the firmware sees a
  failure.
- **Heartbeats:**
  - every 50 ms while keys are held, 100 ms otherwise (1 s in battery idle/sleep);
  - every 2 ms for 2 s after configurator traffic, so `hmkconf` stays responsive.
  - Heartbeats also carry the battery level.
- **Stuck-key failsafe:** the dongle releases all keys if the keyboard goes silent for 250 ms
  while something is held (out of range, battery died mid-press).
- **Back-off:** while the dongle doesn't answer, the keyboard waits 50 ms between attempts,
  instead of retrying continuously and draining the battery.

### Code map

| Crate / module | Role |
|---|---|
| `libhmk-link` | Hardware-independent protocol: message framing, pairing addresses, link settings record, PTX scheduling, failsafe (unit tested on the host) |
| `libhmk-nrf-esb` | Interrupt-driven ESB PTX/PRX driver on the embassy-nrf PAC (2 Mbit/s, dynamic payloads, auto-ACK with payloads, fast ramp-up) |
| `libhmk-nrf52840/src/rf.rs` | Keyboard link task |
| `libhmk-nrf52840/src/wireless.rs` | Transport/backend selection, power states, settings persistence |
| `libhmk-nrf52840-dongle` | Receiver firmware |

## Status and known limitations

None of this has run on hardware yet. Check these when hardware is available:

- [ ] Link comes up after pairing; green LED; keys reach the host through the dongle.
- [ ] Raw HID through the dongle: `hmkconf` connects to the dongle and can read and write the
      keyboard configuration.
- [ ] Failsafe: switch the keyboard off while holding a key; the host sees a release within
      ~250 ms.
- [ ] Backend switching (`BT_24G` ↔ `BT_BLE`) reboots into the other stack; BLE still pairs
      with a host.
- [ ] Range and packet loss in a busy 2.4 GHz environment (Wi-Fi channel overlap).
- [ ] Battery life on 2.4 GHz vs BLE.

Known limitations:

- **Fixed RF channel:** no channel hopping. Heavy Wi-Fi on the same frequencies degrades the
  link (ESB retransmits help). Change the channel in `keyboard.json` and rebuild both
  firmwares; hopping is a possible follow-up.
- **No encryption:** packets aren't encrypted or authenticated (as with typical ESB keyboards).
- **Nordic ESB compatibility:** register setup and on-air format follow Nordic's `nrf_esb` but
  haven't been checked against Nordic's implementation. Both ends here run this driver.
- **One keyboard per dongle:** the receiver serves a single keyboard at a time.
- **Unknown keycode:** `hmkconf` doesn't know `BT_24G` (0xD9) yet.
