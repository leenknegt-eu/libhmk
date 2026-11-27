# nRF52840-DK HE16

Reference 16-key Hall-effect macropad definition for the Nordic Semiconductor **nRF52840 DK** (`nrf52840xx` driver) built with the Rust **Embassy + TrouBLE BLE + `nrf-sdc` + `embassy-usb`** stack.

## Features

- **Single-Binary BLE**: Links Nordic's SoftDevice Controller (`nrf-sdc`) + pure-Rust `trouble-host` GATT server directly into the firmware ELF—no separate S140 `.hex` required.
- **BLE HID over GATT (HOGP)**: Boot + NKRO Keyboard, System Control, Consumer Control, Mouse, and 64-byte Raw HID (`hmkconf` over BLE).
- **Dual-Mode Transport**: Automatically uses USB when mounted and falls back to BLE on battery, with manual override keycodes on Layer 1 (`BT_TOGG`, `BT_USB`, `BT_BLE`, `BT_PAIR`).
- **Battery Service (`0x180F`) & Power Management**: Measures VDD (or an optional voltage-divider pin) via SAADC and throttles matrix scans in Idle/Sleep states.

## Pinout

| Function | nRF52840 Pin |
| --- | --- |
| Mux 0 Analog Output (Keys 1–8) | `P0.02` (`AIN0`) |
| Mux 1 Analog Output (Keys 9–16) | `P0.03` (`AIN1`) |
| Mux Select S0 | `P0.11` |
| Mux Select S1 | `P0.12` |
| Mux Select S2 | `P0.13` |
| Battery Sensing (Optional) | Internal `VDD` (or set `bluetooth.battery_pin` in `keyboard.json`) |

## Building & Flashing

```bash
LIBHMK_KEYBOARD=nrf52840-dk-he16 cargo build -p libhmk-nrf52840 --target thumbv7em-none-eabihf --release
probe-rs run --chip nRF52840_xxAA target/thumbv7em-none-eabihf/release/libhmk-nrf52840
```
