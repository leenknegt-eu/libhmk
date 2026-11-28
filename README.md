# libhmk (Rust Rewrite)

A pure-Rust `#![no_std]` Hall-effect keyboard firmware workspace built on **Embassy** and **TrouBLE BLE**, preserving full wire compatibility with [`hmkconf`](https://github.com/peppapighs/hmkconf) and existing `keyboards/*/keyboard.json` definitions.

## Features

- **Analog Input & Auto-Calibration**: EMA filtering, 1024-entry non-linear Hall-effect distance curve, and automatic rest/bottom-out calibration.
- **Rapid Trigger & Continuous Rapid Trigger**: Dynamic press/release actuation with optional asymmetric `rt_down`/`rt_up` sensitivity.
- **Advanced Keys**: Null Bind (SOCD + Rappy Snappy + Bottom-Out), Dynamic Keystroke (4-stage), Tap-Hold (with Hold-on-Other-Key-Press), Toggle, and linked-list Macros.
- **6KRO + NKRO HID**: Composite Boot Keyboard + NKRO, System Control, Consumer Control, Mouse, and 64-byte Raw HID (`hmkconf`) interfaces.
- **XInput Gamepad**: 4-point analog curve, circular/square joystick scaling, and snappy opposite-axis handling.
- **Wear-Leveled Flash Storage**: Virtual EEPROM with write-log replay and CRC32 consolidation, binary-compatible with `eeconfig` v0x0105.
- **2.4 GHz wireless (nRF52840, primary)**: Nordic Enhanced ShockBurst link to an nRF52840 USB Dongle (PCA10059) running `libhmk-nrf52840-dongle`, with ≈1 kHz reports instead of BLE's ≤133 Hz. Runtime pairing, raw HID (`hmkconf`) relayed through the dongle, and a stuck-key failsafe. See [`docs/wireless.md`](docs/wireless.md).
- **Bluetooth Low Energy (nRF52840, optional)**: Pure-Rust [`trouble-host`](https://github.com/embassy-rs/trouble) + Nordic SoftDevice Controller (`nrf-sdc`) linked directly into a single binary — no separate S140 hex blob required! Includes HOGP (Keyboard/System/Consumer/Mouse), Raw HID over BLE (`hmkconf`), Battery Service (`0x180F`), Device Information (`0x180A`), dual-mode USB/BLE switching (`BT_TOGG`, `BT_USB`, `BT_BLE`, `BT_PAIR`), and Active/Idle/Sleep scan throttling.

## Workspace Layout

| Crate | Target | Role |
| --- | --- | --- |
| [`crates/libhmk-core`](crates/libhmk-core) | `#![no_std]` (host-testable) | Platform-agnostic firmware engine: matrix, Rapid Trigger, advanced keys, layout, wear leveling, `eeconfig`, HID, XInput, and `hmkconf` Raw HID command protocol |
| [`crates/libhmk-config`](crates/libhmk-config) | Host (`build.rs`) | Validates `keyboards/*/keyboard.json`, compresses `hmkconf` metadata, and generates `keyboard_gen.rs` + `memory.x` |
| [`crates/libhmk-nrf52840`](crates/libhmk-nrf52840) | `thumbv7em-none-eabihf` | Nordic nRF52840 firmware: USB, 2.4 GHz dongle link, optional BLE (`embassy-nrf`, `embassy-usb`, `trouble-host`, `nrf-sdc`) |
| [`crates/libhmk-nrf52840-dongle`](crates/libhmk-nrf52840-dongle) | `thumbv7em-none-eabihf` | 2.4 GHz USB receiver for the nRF52840 Dongle (PCA10059) |
| [`crates/libhmk-link`](crates/libhmk-link) | `#![no_std]` (host-testable) | Keyboard ↔ dongle protocol: framing, pairing, link settings, scheduling |
| [`crates/libhmk-nrf-esb`](crates/libhmk-nrf-esb) | `thumbv7em-none-eabihf` | Interrupt-driven Enhanced ShockBurst PTX/PRX radio driver |
| [`crates/libhmk-stm32f446`](crates/libhmk-stm32f446) | `thumbv7em-none-eabihf` | STM32F446xx firmware (`embassy-stm32`, `embassy-usb`) |
| [`crates/libhmk-at32f405`](crates/libhmk-at32f405) | `thumbv7em-none-eabihf` | Artery AT32F405xx firmware (`at32f4xx-pac`, Embassy executor + TMR2 time driver, `embassy-usb-synopsys-otg` on OTGFS1/OTGHS) |

## Scan Pipeline

ADC sampling never blocks the keyboard loop. Each firmware samples the matrix in the background and publishes the latest value per key through lock-free atomics. The main loop runs matrix scan, layout and XInput back to back, the same way the C superloop does:

| MCU | Sampling | Main loop |
| --- | --- | --- |
| STM32F446 | Scanner task on a high-priority `InterruptExecutor`: mux settle via `Timer` (yields), then one ADC scan sequence over all inputs read by DMA2 (ADC stays powered) | free-running, `yield_now()` between iterations |
| AT32F405 | Interrupt driven, as in the C firmware: ADC sequence → DMA1 full-transfer IRQ (store, advance mux) → TMR6 settle delay → next conversion | free-running, `yield_now()` between iterations |
| nRF52840 | Async SAADC task (3 µs acquisition), mux settle timed by TIMER1 at 1 MHz (the RTC time base only has 30.5 µs ticks); battery measured every 10 s with its own channel config | free-running on USB power; on battery sleeps until the next scan pass (Active/Idle/Sleep throttling) |

ADC inputs and mux select pins come from `keyboards/*/keyboard.json`. Each firmware's `build.rs` generates the peripheral bindings (`pins_gen.rs`).

Performance against the C original, and the planned hardware measurements, are in [`docs/performance.md`](docs/performance.md). For on-board timing, build with `--features scan-probe` and set `LIBHMK_PROBE_SCAN_PIN` (optional: `LIBHMK_PROBE_LOOP_PIN`) to put the mux-step and main-loop timing on spare GPIOs.

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

The core tests include a model check of `Matrix::scan` against a port of the C `matrix_scan()` (Rapid Trigger, continuous RT, asymmetric `rt_up`) over 20k randomized ticks.

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

### Build the 2.4 GHz dongle for a keyboard
```bash
LIBHMK_KEYBOARD=nrf52840-dk-he16 cargo build -p libhmk-nrf52840-dongle --target thumbv7em-none-eabihf --release
```
See [`docs/wireless.md`](docs/wireless.md) for pairing and flashing the dongle over its USB bootloader.

### Flash to nRF52840 DK via `probe-rs`
Because `nrf-sdc` links Nordic's SoftDevice Controller directly into the ELF binary, no separate SoftDevice `.hex` flashing step is needed:

```bash
LIBHMK_KEYBOARD=nrf52840-dk-he16 probe-rs run --chip nRF52840_xxAA \
  target/thumbv7em-none-eabihf/release/libhmk-nrf52840
```

## License

GPL-3.0-or-later — see [LICENSE](LICENSE).
