/*
 * This program is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include "common.h"

//--------------------------------------------------------------------+
// Bluetooth Types
//--------------------------------------------------------------------+

typedef enum {
  // Prefer USB when mounted, automatically fall back to BLE otherwise
  BLE_TRANSPORT_AUTO = 0,
  // Force USB transport
  BLE_TRANSPORT_USB,
  // Force BLE transport (even while USB VBUS is connected)
  BLE_TRANSPORT_BLE,
} ble_transport_mode_t;

typedef enum {
  BLE_STATE_DISABLED = 0,
  BLE_STATE_IDLE,
  BLE_STATE_ADVERTISING,
  BLE_STATE_CONNECTED,
} ble_state_t;

typedef enum {
  BLE_POWER_ACTIVE = 0,
  BLE_POWER_IDLE,
  BLE_POWER_SLEEP,
} ble_power_state_t;

//--------------------------------------------------------------------+
// Bluetooth Configuration Defaults
//--------------------------------------------------------------------+

#if defined(BLUETOOTH_ENABLED)

#if !defined(BLE_DEVICE_NAME)
#define BLE_DEVICE_NAME USB_PRODUCT_NAME
#endif

#if !defined(BLE_TX_POWER)
#define BLE_TX_POWER 0
#endif

#if !defined(BLE_IDLE_TIMEOUT_MS)
#define BLE_IDLE_TIMEOUT_MS 30000
#endif

#if !defined(BLE_SLEEP_TIMEOUT_MS)
#define BLE_SLEEP_TIMEOUT_MS 300000
#endif

#if !defined(BLE_BATTERY_DIVIDER_MILLIS)
#define BLE_BATTERY_DIVIDER_MILLIS 1000
#endif

//--------------------------------------------------------------------+
// Bluetooth API
//--------------------------------------------------------------------+

/**
 * @brief Initialize the Bluetooth stack, GATT services (HIDS, DIS, BAS,
 * Raw HID), and start advertising
 *
 * @return None
 */
void ble_init(void);

/**
 * @brief Process pending Bluetooth events and periodic tasks (e.g. battery
 * level updates)
 *
 * @return None
 */
void ble_task(void);

/**
 * @brief Get the current BLE connection state
 *
 * @return Current BLE state
 */
ble_state_t ble_get_state(void);

/**
 * @brief Check whether a BLE host is currently connected
 *
 * @return true if connected over BLE, false otherwise
 */
bool ble_is_connected(void);

/**
 * @brief Check whether USB should currently be used as the active HID transport
 *
 * @return true if USB transport is selected and active, false otherwise
 */
bool ble_is_usb_active(void);

/**
 * @brief Get the configured output transport mode
 *
 * @return Current transport mode
 */
ble_transport_mode_t ble_get_transport_mode(void);

/**
 * @brief Set the output transport mode (AUTO, USB, or BLE)
 *
 * @param mode Desired transport mode
 *
 * @return None
 */
void ble_set_transport_mode(ble_transport_mode_t mode);

/**
 * @brief Cycle between USB/BLE transport modes
 *
 * @return None
 */
void ble_toggle_transport(void);

/**
 * @brief Start BLE advertising (disconnecting the current peer if requested)
 *
 * @param disconnect_current If true, disconnect any active peer first to allow
 * pairing with a new host
 *
 * @return None
 */
void ble_start_advertising(bool disconnect_current);

/**
 * @brief Check whether the BLE HID notification queue can accept a report
 *
 * @return true if ready to send a HID report over BLE
 */
bool ble_hid_ready(void);

/**
 * @brief Send a keyboard HID report over BLE (automatically formats Boot or
 * NKRO report according to the host's Protocol Mode)
 *
 * @param report Pointer to `hid_nkro_kb_report_t`
 * @param len Length of the report in bytes
 *
 * @return true if queued for transmission
 */
bool ble_hid_send_keyboard(const void *report, uint16_t len);

/**
 * @brief Send a System Control HID report over BLE
 *
 * @param report System control usage value
 *
 * @return true if queued for transmission
 */
bool ble_hid_send_system(uint16_t report);

/**
 * @brief Send a Consumer Control HID report over BLE
 *
 * @param report Consumer control usage value
 *
 * @return true if queued for transmission
 */
bool ble_hid_send_consumer(uint16_t report);

/**
 * @brief Send a Mouse HID report over BLE
 *
 * @param report Pointer to `hid_mouse_report_t`
 * @param len Length of the report in bytes
 *
 * @return true if queued for transmission
 */
bool ble_hid_send_mouse(const void *report, uint16_t len);

/**
 * @brief Check whether the BLE Raw HID notification characteristic is ready
 *
 * @return true if ready to send a Raw HID response over BLE
 */
bool ble_raw_hid_ready(void);

/**
 * @brief Send a Raw HID response packet over BLE
 *
 * @param buf Response buffer (RAW_HID_EP_SIZE bytes)
 * @param len Length in bytes
 *
 * @return true if queued for transmission
 */
bool ble_raw_hid_send(const uint8_t *buf, uint16_t len);

/**
 * @brief Get the last measured battery level percentage (0-100)
 *
 * @return Battery percentage
 */
uint8_t ble_battery_get_level(void);

/**
 * @brief Measure battery voltage and update the Battery Service characteristic
 *
 * @return None
 */
void ble_battery_update(void);

/**
 * @brief Notify the power manager of user or host activity to reset idle/sleep
 * timers and return to full-speed scanning
 *
 * @return None
 */
void ble_notify_activity(void);

/**
 * @brief Update idle/sleep power states and enter low-power wait when on
 * battery power
 *
 * @return None
 */
void ble_power_manage(void);

#else

// No-op inline stubs when Bluetooth is not enabled
static inline void ble_init(void) {}
static inline void ble_task(void) {}
static inline ble_state_t ble_get_state(void) { return BLE_STATE_DISABLED; }
static inline bool ble_is_connected(void) { return false; }
static inline bool ble_is_usb_active(void) { return true; }
static inline ble_transport_mode_t ble_get_transport_mode(void) {
  return BLE_TRANSPORT_USB;
}
static inline void ble_set_transport_mode(ble_transport_mode_t mode) {
  (void)mode;
}
static inline void ble_toggle_transport(void) {}
static inline void ble_start_advertising(bool disconnect_current) {
  (void)disconnect_current;
}
static inline bool ble_hid_ready(void) { return false; }
static inline bool ble_hid_send_keyboard(const void *report, uint16_t len) {
  (void)report;
  (void)len;
  return false;
}
static inline bool ble_hid_send_system(uint16_t report) {
  (void)report;
  return false;
}
static inline bool ble_hid_send_consumer(uint16_t report) {
  (void)report;
  return false;
}
static inline bool ble_hid_send_mouse(const void *report, uint16_t len) {
  (void)report;
  (void)len;
  return false;
}
static inline bool ble_raw_hid_ready(void) { return false; }
static inline bool ble_raw_hid_send(const uint8_t *buf, uint16_t len) {
  (void)buf;
  (void)len;
  return false;
}
static inline uint8_t ble_battery_get_level(void) { return 100; }
static inline void ble_battery_update(void) {}
static inline void ble_notify_activity(void) {}
static inline void ble_power_manage(void) {}

#endif
