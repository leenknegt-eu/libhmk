# libhmk (Rust Rewrite)

A pure-Rust `#![no_std]` Hall-effect keyboard firmware workspace built on **Embassy** and **TrouBLE BLE**, preserving full wire compatibility with [`hmkconf`](https://github.com/peppapighs/hmkconf) and existing `keyboards/*/keyboard.json` definitions.

## Features

- **Analog Input & Auto-Calibration**: EMA filtering, 1024-entry non-linear Hall-effect distance curve, and automatic rest/bottom-out calibration.
- **Rapid Trigger & Continuous Rapid Trigger**: Dynamic press/release actuation with optional asymmetric `rt_down`/`rt_up` sensitivity.
- **Advanced Keys**: Null Bind (SOCD + Rappy Snappy + Bottom-Out), Dynamic Keystroke (4-stage), Tap-Hold (with Hold-on-Other-Key-Press), Toggle, and linked-list Macros.
- **6KRO + NKRO HID**: Composite Boot Keyboard + NKRO, System Control, Consumer Control, Mouse, and 64-byte Raw HID (`hmkconf`) interfaces.
- **XInput Gamepad**: 4-point analog curve, circular/square joystick scaling, and snappy opposite-axis handling.
- **Wear-Leveled Flash Storage**: Virtual EEPROM with write-log replay and CRC32 consolidation, binary-compatible with `eeconfig` v0x0105.
- **Bluetooth Low Energy (nRF52840)**: Pure-Rust [`trouble-host`](https://github.com/embassy-rs/trouble) + Nordic SoftDevice Controller (`nrf-sdc`) linked directly into a single binary — no separate S140 hex blob required! Includes HOGP (Keyboard/System/Consumer/Mouse), Raw HID over BLE (`hmkconf`), Battery Service (`0x180F`), Device Information (`0x180A`), dual-mode USB/BLE switching (`BT_TOGG`, `BT_USB`, `BT_BLE`, `BT_PAIR`), and Active/Idle/Sleep scan throttling.

## Workspace Layout

| Crate | Target | Role |
| --- | --- | --- |
| [`crates/libhmk-core`](crates/libhmk-core) | `#![no_std]` (host-testable) | Platform-agnostic firmware engine: matrix, Rapid Trigger, advanced keys, layout, wear leveling, `eeconfig`, HID, XInput, and `hmkconf` Raw HID command protocol |
| [`crates/libhmk-config`](crates/libhmk-config) | Host (`build.rs`) | Validates `keyboards/*/keyboard.json`, compresses `hmkconf` metadata, and generates `keyboard_gen.rs` + `memory.x` |
| [`crates/libhmk-nrf52840`](crates/libhmk-nrf52840) | `thumbv7em-none-eabihf` | Nordic nRF52840 firmware (`embassy-nrf`, `embassy-usb`, `trouble-host`, `nrf-sdc`) |
| [`crates/libhmk-stm32f446`](crates/libhmk-stm32f446) | `thumbv7em-none-eabihf` | STM32F446xx firmware (`embassy-stm32`, `embassy-usb`) |
| [`crates/libhmk-at32f405`](crates/libhmk-at32f405) | `thumbv7em-none-eabihf` | Artery AT32F405xx bare-metal firmware (`at32f4xx-pac`, `cortex-m-rt`) |

## Prerequisites

- Rust toolchain (`rustup`) with Cortex-M4F target:
  ```bash
  rustup target add thumbv7em-none-eabihf
  ```
- [`probe-rs`](https://probe.rs/) (recommended for flashing & RTT `defmt` logging)

## Building & Testing

### Run host unit tests (`libhmk-core` & `libhmk-config`)
```bash
cargo test
```

### Build firmware for a specific keyboard
Select any keyboard under `keyboards/` via `LIBHMK_KEYBOARD`:

```bash
# Nordic nRF52840 DK (BLE + USB)
LIBHMK_KEYBOARD=nrf52840-dk-he16 cargo build -p libhmk-nrf52840 --target thumbv7em-none-eabihf --release

# STM32F446xx (e.g. he60)
LIBHMK_KEYBOARD=he60 cargo build -p libhmk-stm32f446 --target thumbv7em-none-eabihf --release

# Artery AT32F405xx (e.g. he16)
LIBHMK_KEYBOARD=he16 cargo build -p libhmk-at32f405 --target thumbv7em-none-eabihf --release
```

### Flash to nRF52840 DK via `probe-rs`
Because `nrf-sdc` links Nordic's SoftDevice Controller directly into the ELF binary, no separate SoftDevice `.hex` flashing step is needed:

```bash
LIBHMK_KEYBOARD=nrf52840-dk-he16 probe-rs run --chip nRF52840_xxAA \
  target/thumbv7em-none-eabihf/release/libhmk-nrf52840
```

## License

GPL-3.0-or-later — see [LICENSE](LICENSE).
