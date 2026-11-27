# nRF52840-DK HE16

Reference 16-key Hall-effect macropad definition for the Nordic Semiconductor **nRF52840 DK** (`nrf52840xx` driver) with simultaneous USB Full-Speed (TinyUSB) and Bluetooth Low Energy (SoftDevice S140 v7.3.0) support.

## Features

- **BLE HID over GATT (HOGP)**: NKRO / 6KRO Boot Keyboard, Consumer Control, System Control, and Mouse reports.
- **Dual-Mode Transport**: Automatically uses USB when mounted and falls back to BLE on battery, with manual override keycodes on Layer 1 (`BT_TOGG`, `BT_USB`, `BT_BLE`, `BT_PAIR`).
- **Raw HID over BLE**: Supports the `hmkconf` web configurator protocol over both USB Raw HID and BLE HID Report ID 5 (`0xFFAB` / `0xAB`, 64-byte packets).
- **Battery Service (BAS) & Power Management**: Reports battery percentage over BLE (`0x180F`) and throttles SAADC scan rate while using `sd_app_evt_wait()` in idle/sleep states.

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

1. Generate `platformio.ini`:
   ```bash
   python setup.py -k nrf52840-dk-he16
   ```
2. Build the firmware (PlatformIO automatically fetches CMSIS, `nrfx`, and SoftDevice S140 v7.3.0 headers on first run):
   ```bash
   pio run
   ```
3. Flash **SoftDevice S140 v7.3.0** once (required before running BLE firmware on a blank nRF52840 DK):
   ```bash
   nrfjprog -f nrf52 --program s140_nrf52_7.3.0_softdevice.hex --chiperase --verify
   ```
4. Flash the `libhmk` application firmware (starts at `0x27000`):
   ```bash
   nrfjprog -f nrf52 --program .pio/build/nrf52840-dk-he16/firmware.hex --sectorerase --verify --reset
   ```
   Or upload directly via PlatformIO J-Link (`pio run --target upload`).
