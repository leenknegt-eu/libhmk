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

#include "hardware/hardware.h"

#if defined(BLUETOOTH_ENABLED)

#include "commands.h"
#include "tusb.h"
#include "usb_descriptors.h"

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wcast-qual"
#pragma GCC diagnostic ignored "-Wstrict-prototypes"
#endif

#include "ble.h"
#include "ble_err.h"
#include "ble_gap.h"
#include "ble_gatt.h"
#include "ble_gatts.h"
#include "ble_hci.h"
#include "ble_types.h"
#include "nrf.h"
#include "nrf_error.h"
#include "nrf_sdm.h"
#include "nrf_soc.h"

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

//--------------------------------------------------------------------+
// BLE Configuration Constants
//--------------------------------------------------------------------+

#define BLE_CONN_CFG_TAG 1U
// ATT MTU large enough for 64-byte Raw HID reports + 3-byte ATT header
#define BLE_ATT_MTU_SIZE 128U
#define BLE_HVN_TX_QUEUE_SIZE 8U
#define BLE_GATTS_ATTR_TAB_SIZE 2400UL

// Preferred connection parameters (units: interval = 1.25 ms, timeout = 10 ms)
// 7.5 ms .. 15 ms for responsive Hall-effect keyboard input
#define BLE_MIN_CONN_INTERVAL 6U
#define BLE_MAX_CONN_INTERVAL 12U
#define BLE_SLAVE_LATENCY 4U
#define BLE_CONN_SUP_TIMEOUT 400U

// Advertising intervals (units of 0.625 ms)
#define BLE_ADV_INTERVAL_FAST 64U   // 40 ms
#define BLE_ADV_INTERVAL_SLOW 320U  // 200 ms

// Battery polling period (30 seconds)
#define BLE_BATTERY_UPDATE_INTERVAL_MS 30000UL

// SAADC scan throttling intervals in low-power modes (microseconds)
#define BLE_IDLE_SCAN_INTERVAL_US 5000UL
#define BLE_SLEEP_SCAN_INTERVAL_US 20000UL

// Standard Bluetooth SIG 16-bit Service & Characteristic UUIDs
#define UUID_SVC_DEVICE_INFO 0x180AU
#define UUID_SVC_BATTERY 0x180FU
#define UUID_SVC_HID 0x1812U

#define UUID_CHAR_MODEL_NUM 0x2A24U
#define UUID_CHAR_SERIAL_NUM 0x2A25U
#define UUID_CHAR_FW_REV 0x2A26U
#define UUID_CHAR_MANUFACTURER 0x2A29U
#define UUID_CHAR_PNP_ID 0x2A50U

#define UUID_CHAR_BATTERY_LEVEL 0x2A19U

#define UUID_CHAR_BOOT_KB_IN 0x2A22U
#define UUID_CHAR_BOOT_KB_OUT 0x2A32U
#define UUID_CHAR_HID_INFO 0x2A4AU
#define UUID_CHAR_REPORT_MAP 0x2A4BU
#define UUID_CHAR_HID_CTRL_PT 0x2A4CU
#define UUID_CHAR_HID_REPORT 0x2A4DU
#define UUID_CHAR_PROTOCOL_MODE 0x2A4EU
#define UUID_DESC_REPORT_REF 0x2908U

// HID Protocol Modes
#define HID_PROTOCOL_MODE_BOOT 0x00U
#define HID_PROTOCOL_MODE_REPORT 0x01U

// BLE HID Report IDs inside the unified Report Map
enum {
  BLE_REPORT_ID_KEYBOARD = 1,
  BLE_REPORT_ID_SYSTEM = 2,
  BLE_REPORT_ID_CONSUMER = 3,
  BLE_REPORT_ID_MOUSE = 4,
  BLE_REPORT_ID_RAW_HID = 5,
};

// Linker-provided application RAM start symbol
extern uint32_t __ram_start__[];

//--------------------------------------------------------------------+
// Unified BLE HID Report Map (Keyboard NKRO, System, Consumer, Mouse, Raw HID)
//--------------------------------------------------------------------+

static const uint8_t ble_hid_report_map[] = {
    // --- Report ID 1: Keyboard (NKRO + 6KRO compatible) ---
    HID_USAGE_PAGE(HID_USAGE_PAGE_DESKTOP),
    HID_USAGE(HID_USAGE_DESKTOP_KEYBOARD),
    HID_COLLECTION(HID_COLLECTION_APPLICATION),
    HID_REPORT_ID(BLE_REPORT_ID_KEYBOARD)

    // 8 bits for modifiers
    HID_USAGE_PAGE(HID_USAGE_PAGE_KEYBOARD), HID_USAGE_MIN(224),
    HID_USAGE_MAX(231), HID_LOGICAL_MIN(0), HID_LOGICAL_MAX(1),
    HID_REPORT_COUNT(8), HID_REPORT_SIZE(1),
    HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE),

    // 8 bits reserved
    HID_REPORT_COUNT(1), HID_REPORT_SIZE(8), HID_INPUT(HID_CONSTANT),

    // 5-bit LED indicator output + 3-bit padding
    HID_USAGE_PAGE(HID_USAGE_PAGE_LED), HID_USAGE_MIN(1), HID_USAGE_MAX(5),
    HID_REPORT_COUNT(5), HID_REPORT_SIZE(1),
    HID_OUTPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE),
    HID_REPORT_COUNT(1), HID_REPORT_SIZE(3), HID_OUTPUT(HID_CONSTANT),

    // 6-byte padding matching hid_nkro_kb_report_t layout
    HID_REPORT_COUNT(48), HID_REPORT_SIZE(1), HID_INPUT(HID_CONSTANT),

    // NKRO bitmap (20 bytes = 160 keys)
    HID_USAGE_PAGE(HID_USAGE_PAGE_KEYBOARD), HID_USAGE_MIN(0),
    HID_USAGE_MAX(NUM_NKRO_BYTES * 8 - 1), HID_LOGICAL_MIN(0),
    HID_LOGICAL_MAX(1), HID_REPORT_COUNT(NUM_NKRO_BYTES * 8),
    HID_REPORT_SIZE(1), HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE),
    HID_COLLECTION_END,

    // --- Report ID 2: System Control ---
    TUD_HID_REPORT_DESC_SYSTEM_CONTROL(HID_REPORT_ID(BLE_REPORT_ID_SYSTEM)),

    // --- Report ID 3: Consumer Control ---
    TUD_HID_REPORT_DESC_CONSUMER(HID_REPORT_ID(BLE_REPORT_ID_CONSUMER)),

    // --- Report ID 4: Mouse ---
    TUD_HID_REPORT_DESC_MOUSE(HID_REPORT_ID(BLE_REPORT_ID_MOUSE)),

    // --- Report ID 5: Raw HID (Web Configurator / hmkconf) ---
    HID_USAGE_PAGE_N(RAW_HID_USAGE_PAGE, 2), HID_USAGE(RAW_HID_USAGE),
    HID_COLLECTION(HID_COLLECTION_APPLICATION),
    HID_REPORT_ID(BLE_REPORT_ID_RAW_HID)

    // Data to host (64 bytes)
    HID_USAGE(RAW_HID_USAGE + 1), HID_LOGICAL_MIN(0), HID_LOGICAL_MAX_N(255, 2),
    HID_REPORT_COUNT(RAW_HID_EP_SIZE), HID_REPORT_SIZE(8),
    HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE),

    // Data from host (64 bytes)
    HID_USAGE(RAW_HID_USAGE + 2), HID_LOGICAL_MIN(0), HID_LOGICAL_MAX_N(255, 2),
    HID_REPORT_COUNT(RAW_HID_EP_SIZE), HID_REPORT_SIZE(8),
    HID_OUTPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE | HID_NON_VOLATILE),
    HID_COLLECTION_END,
};

//--------------------------------------------------------------------+
// Module State
//--------------------------------------------------------------------+

static volatile ble_state_t ble_state = BLE_STATE_DISABLED;
static volatile ble_transport_mode_t transport_mode = BLE_TRANSPORT_AUTO;
static volatile ble_power_state_t power_state = BLE_POWER_ACTIVE;

static volatile uint16_t conn_handle = BLE_CONN_HANDLE_INVALID;
static uint8_t adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;
static volatile uint8_t protocol_mode = HID_PROTOCOL_MODE_REPORT;
static volatile uint8_t hvn_tx_pending = 0;

// GATT characteristic handles
static ble_gatts_char_handles_t hids_protocol_mode_handles;
static ble_gatts_char_handles_t hids_ctrl_point_handles;
static ble_gatts_char_handles_t hids_boot_kb_in_handles;
static ble_gatts_char_handles_t hids_boot_kb_out_handles;
static ble_gatts_char_handles_t hids_kb_in_handles;
static ble_gatts_char_handles_t hids_kb_out_handles;
static ble_gatts_char_handles_t hids_sys_in_handles;
static ble_gatts_char_handles_t hids_consumer_in_handles;
static ble_gatts_char_handles_t hids_mouse_in_handles;
static ble_gatts_char_handles_t hids_raw_in_handles;
static ble_gatts_char_handles_t hids_raw_out_handles;
static ble_gatts_char_handles_t bas_level_handles;

// CCCD notification enable flags
static volatile bool cccd_boot_kb_in;
static volatile bool cccd_kb_in;
static volatile bool cccd_sys_in;
static volatile bool cccd_consumer_in;
static volatile bool cccd_mouse_in;
static volatile bool cccd_raw_in;
static volatile bool cccd_battery;

// Bonding key storage (in-session Just Works LTK storage for re-encryption)
static ble_gap_enc_key_t bond_own_enc_key;
static ble_gap_id_key_t bond_peer_id_key;
static bool bond_valid = false;

// Advertising buffers (must persist while advertising is active in S140)
static uint8_t adv_data_buf[BLE_GAP_ADV_SET_DATA_SIZE_MAX];
static uint8_t scan_rsp_buf[BLE_GAP_ADV_SET_DATA_SIZE_MAX];

// Battery & power management state
static uint8_t battery_level = 100;
static uint32_t last_battery_update_ms = 0;
static volatile uint32_t last_activity_ms = 0;

// SoftDevice BLE event receive buffer (4-byte aligned)
#define BLE_EVT_BUF_SIZE (sizeof(ble_evt_t) + BLE_ATT_MTU_SIZE + 32U)
__attribute__((aligned(4))) static uint8_t ble_evt_buf[BLE_EVT_BUF_SIZE];

//--------------------------------------------------------------------+
// Advertising Helpers
//--------------------------------------------------------------------+

static void ble_configure_advertising(uint16_t interval) {
  uint8_t idx = 0;

  // Flags: LE General Discoverable Mode | BR/EDR Not Supported
  adv_data_buf[idx++] = 2U;
  adv_data_buf[idx++] = BLE_GAP_AD_TYPE_FLAGS;
  adv_data_buf[idx++] = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

  // Appearance: Keyboard (0x03C1)
  adv_data_buf[idx++] = 3U;
  adv_data_buf[idx++] = BLE_GAP_AD_TYPE_APPEARANCE;
  adv_data_buf[idx++] = (uint8_t)(BLE_APPEARANCE_HID_KEYBOARD & 0xFFU);
  adv_data_buf[idx++] = (uint8_t)((BLE_APPEARANCE_HID_KEYBOARD >> 8) & 0xFFU);

  // Complete List of 16-bit Service Class UUIDs (HIDS + BAS)
  adv_data_buf[idx++] = 5U;
  adv_data_buf[idx++] = BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE;
  adv_data_buf[idx++] = (uint8_t)(UUID_SVC_HID & 0xFFU);
  adv_data_buf[idx++] = (uint8_t)((UUID_SVC_HID >> 8) & 0xFFU);
  adv_data_buf[idx++] = (uint8_t)(UUID_SVC_BATTERY & 0xFFU);
  adv_data_buf[idx++] = (uint8_t)((UUID_SVC_BATTERY >> 8) & 0xFFU);

  // Scan Response: Complete Local Name
  const char *name = BLE_DEVICE_NAME;
  uint8_t name_len = (uint8_t)strlen(name);
  if (name_len > BLE_GAP_ADV_SET_DATA_SIZE_MAX - 2U)
    name_len = BLE_GAP_ADV_SET_DATA_SIZE_MAX - 2U;

  uint8_t sr_idx = 0;
  scan_rsp_buf[sr_idx++] = (uint8_t)(name_len + 1U);
  scan_rsp_buf[sr_idx++] = BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME;
  memcpy(&scan_rsp_buf[sr_idx], name, name_len);
  sr_idx = (uint8_t)(sr_idx + name_len);

  ble_gap_adv_data_t adv_data = {
      .adv_data = {.p_data = adv_data_buf, .len = idx},
      .scan_rsp_data = {.p_data = scan_rsp_buf, .len = sr_idx},
  };

  ble_gap_adv_params_t adv_params = {0};
  adv_params.properties.type =
      BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
  adv_params.interval = interval;
  adv_params.duration = BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED;
  adv_params.filter_policy = BLE_GAP_ADV_FP_ANY;
  adv_params.primary_phy = BLE_GAP_PHY_1MBPS;

  (void)sd_ble_gap_adv_stop(adv_handle);
  if (sd_ble_gap_adv_set_configure(&adv_handle, &adv_data, &adv_params) !=
      NRF_SUCCESS)
    return;

  (void)sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, adv_handle,
                                (int8_t)BLE_TX_POWER);

  if (sd_ble_gap_adv_start(adv_handle, BLE_CONN_CFG_TAG) == NRF_SUCCESS)
    ble_state = BLE_STATE_ADVERTISING;
}

//--------------------------------------------------------------------+
// GATT Service Registration Helpers
//--------------------------------------------------------------------+

static void ble_add_readonly_char(uint16_t service_handle, uint16_t uuid16,
                                  const void *data, uint16_t len) {
  ble_uuid_t uuid = {.uuid = uuid16, .type = BLE_UUID_TYPE_BLE};

  ble_gatts_char_md_t char_md = {0};
  char_md.char_props.read = 1;

  ble_gatts_attr_md_t attr_md = {0};
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
  attr_md.vloc = BLE_GATTS_VLOC_STACK;

  ble_gatts_attr_t attr = {
      .p_uuid = &uuid,
      .p_attr_md = &attr_md,
      .init_len = len,
      .init_offs = 0,
      .max_len = len,
      .p_value = (uint8_t *)(uintptr_t)data,
  };

  ble_gatts_char_handles_t handles = {0};
  if (sd_ble_gatts_characteristic_add(service_handle, &char_md, &attr,
                                      &handles) != NRF_SUCCESS)
    board_error_handler();
}

static void ble_add_hid_report_char(uint16_t service_handle, uint8_t report_id,
                                    uint8_t report_type, uint16_t max_len,
                                    bool is_input,
                                    ble_gatts_char_handles_t *out_handles) {
  ble_uuid_t uuid = {.uuid = UUID_CHAR_HID_REPORT, .type = BLE_UUID_TYPE_BLE};

  ble_gatts_attr_md_t cccd_md = {0};
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
  cccd_md.vloc = BLE_GATTS_VLOC_STACK;

  ble_gatts_char_md_t char_md = {0};
  char_md.char_props.read = 1;
  if (is_input) {
    char_md.char_props.notify = 1;
    char_md.p_cccd_md = &cccd_md;
  } else {
    char_md.char_props.write = 1;
    char_md.char_props.write_wo_resp = 1;
  }

  ble_gatts_attr_md_t attr_md = {0};
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
  if (is_input)
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
  else
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
  attr_md.vloc = BLE_GATTS_VLOC_STACK;
  attr_md.vlen = 1;

  uint8_t init_zero[RAW_HID_EP_SIZE] = {0};
  ble_gatts_attr_t attr = {
      .p_uuid = &uuid,
      .p_attr_md = &attr_md,
      .init_len = max_len,
      .init_offs = 0,
      .max_len = max_len,
      .p_value = init_zero,
  };

  if (sd_ble_gatts_characteristic_add(service_handle, &char_md, &attr,
                                      out_handles) != NRF_SUCCESS)
    board_error_handler();

  // Add Report Reference Descriptor (UUID 0x2908: [report_id, report_type])
  uint8_t report_ref[2] = {report_id, report_type};
  ble_uuid_t desc_uuid = {.uuid = UUID_DESC_REPORT_REF,
                          .type = BLE_UUID_TYPE_BLE};
  ble_gatts_attr_md_t desc_md = {0};
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&desc_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&desc_md.write_perm);
  desc_md.vloc = BLE_GATTS_VLOC_STACK;

  ble_gatts_attr_t desc_attr = {
      .p_uuid = &desc_uuid,
      .p_attr_md = &desc_md,
      .init_len = sizeof(report_ref),
      .init_offs = 0,
      .max_len = sizeof(report_ref),
      .p_value = report_ref,
  };

  uint16_t desc_handle = BLE_GATT_HANDLE_INVALID;
  if (sd_ble_gatts_descriptor_add(out_handles->value_handle, &desc_attr,
                                  &desc_handle) != NRF_SUCCESS)
    board_error_handler();
}

static void ble_dis_init(void) {
  ble_uuid_t svc_uuid = {.uuid = UUID_SVC_DEVICE_INFO,
                         .type = BLE_UUID_TYPE_BLE};
  uint16_t svc_handle = BLE_GATT_HANDLE_INVALID;
  if (sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &svc_uuid,
                               &svc_handle) != NRF_SUCCESS)
    board_error_handler();

  const char *mfg = USB_MANUFACTURER_NAME;
  ble_add_readonly_char(svc_handle, UUID_CHAR_MANUFACTURER, mfg,
                        (uint16_t)strlen(mfg));

  const char *model = USB_PRODUCT_NAME;
  ble_add_readonly_char(svc_handle, UUID_CHAR_MODEL_NUM, model,
                        (uint16_t)strlen(model));

  char serial[32];
  const uint32_t serial_len = board_serial(serial);
  ble_add_readonly_char(svc_handle, UUID_CHAR_SERIAL_NUM, serial,
                        (uint16_t)serial_len);

  // Format FIRMWARE_VERSION (0xMMmm -> "M.m")
  char fw_rev[8];
  const uint8_t maj = (uint8_t)(FIRMWARE_VERSION >> 8);
  const uint8_t min = (uint8_t)(FIRMWARE_VERSION & 0xFFU);
  fw_rev[0] = (char)('0' + (maj / 10U));
  fw_rev[1] = (char)('0' + (maj % 10U));
  fw_rev[2] = '.';
  fw_rev[3] = (char)('0' + (min / 10U));
  fw_rev[4] = (char)('0' + (min % 10U));
  ble_add_readonly_char(svc_handle, UUID_CHAR_FW_REV, fw_rev, 5U);

  // PnP ID (required by HOGP): Vendor ID Source = 0x02 (USB-IF)
  const uint8_t pnp_id[7] = {
      0x02U,
      (uint8_t)(USB_VENDOR_ID & 0xFFU),
      (uint8_t)((USB_VENDOR_ID >> 8) & 0xFFU),
      (uint8_t)(USB_PRODUCT_ID & 0xFFU),
      (uint8_t)((USB_PRODUCT_ID >> 8) & 0xFFU),
      (uint8_t)(FIRMWARE_VERSION & 0xFFU),
      (uint8_t)((FIRMWARE_VERSION >> 8) & 0xFFU),
  };
  ble_add_readonly_char(svc_handle, UUID_CHAR_PNP_ID, pnp_id, sizeof(pnp_id));
}

static void ble_bas_init(void) {
  ble_uuid_t svc_uuid = {.uuid = UUID_SVC_BATTERY, .type = BLE_UUID_TYPE_BLE};
  uint16_t svc_handle = BLE_GATT_HANDLE_INVALID;
  if (sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &svc_uuid,
                               &svc_handle) != NRF_SUCCESS)
    board_error_handler();

  ble_gatts_attr_md_t cccd_md = {0};
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
  cccd_md.vloc = BLE_GATTS_VLOC_STACK;

  ble_gatts_char_md_t char_md = {0};
  char_md.char_props.read = 1;
  char_md.char_props.notify = 1;
  char_md.p_cccd_md = &cccd_md;

  ble_gatts_attr_md_t attr_md = {0};
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
  attr_md.vloc = BLE_GATTS_VLOC_STACK;

  ble_uuid_t uuid = {.uuid = UUID_CHAR_BATTERY_LEVEL,
                     .type = BLE_UUID_TYPE_BLE};
  ble_gatts_attr_t attr = {
      .p_uuid = &uuid,
      .p_attr_md = &attr_md,
      .init_len = sizeof(battery_level),
      .init_offs = 0,
      .max_len = sizeof(battery_level),
      .p_value = &battery_level,
  };

  if (sd_ble_gatts_characteristic_add(svc_handle, &char_md, &attr,
                                      &bas_level_handles) != NRF_SUCCESS)
    board_error_handler();
}

static void ble_hids_init(void) {
  ble_uuid_t svc_uuid = {.uuid = UUID_SVC_HID, .type = BLE_UUID_TYPE_BLE};
  uint16_t svc_handle = BLE_GATT_HANDLE_INVALID;
  if (sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &svc_uuid,
                               &svc_handle) != NRF_SUCCESS)
    board_error_handler();

  // HID Information (bcdHID = 0x0111, country = 0x00, flags = RemoteWake |
  // NormallyConnectable)
  const uint8_t hid_info[4] = {0x11U, 0x01U, 0x00U, 0x03U};
  ble_add_readonly_char(svc_handle, UUID_CHAR_HID_INFO, hid_info,
                        sizeof(hid_info));

  // Report Map
  ble_add_readonly_char(svc_handle, UUID_CHAR_REPORT_MAP, ble_hid_report_map,
                        sizeof(ble_hid_report_map));

  // Protocol Mode (Read + WriteWithoutResponse, default Report Protocol)
  {
    ble_uuid_t uuid = {.uuid = UUID_CHAR_PROTOCOL_MODE,
                       .type = BLE_UUID_TYPE_BLE};
    ble_gatts_char_md_t char_md = {0};
    char_md.char_props.read = 1;
    char_md.char_props.write_wo_resp = 1;

    ble_gatts_attr_md_t attr_md = {0};
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc = BLE_GATTS_VLOC_STACK;

    uint8_t init_mode = HID_PROTOCOL_MODE_REPORT;
    ble_gatts_attr_t attr = {
        .p_uuid = &uuid,
        .p_attr_md = &attr_md,
        .init_len = 1U,
        .init_offs = 0,
        .max_len = 1U,
        .p_value = &init_mode,
    };
    if (sd_ble_gatts_characteristic_add(svc_handle, &char_md, &attr,
                                        &hids_protocol_mode_handles) !=
        NRF_SUCCESS)
      board_error_handler();
  }

  // HID Control Point (WriteWithoutResponse)
  {
    ble_uuid_t uuid = {.uuid = UUID_CHAR_HID_CTRL_PT,
                       .type = BLE_UUID_TYPE_BLE};
    ble_gatts_char_md_t char_md = {0};
    char_md.char_props.write_wo_resp = 1;

    ble_gatts_attr_md_t attr_md = {0};
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc = BLE_GATTS_VLOC_STACK;

    uint8_t init_cp = 0;
    ble_gatts_attr_t attr = {
        .p_uuid = &uuid,
        .p_attr_md = &attr_md,
        .init_len = 1U,
        .init_offs = 0,
        .max_len = 1U,
        .p_value = &init_cp,
    };
    if (sd_ble_gatts_characteristic_add(svc_handle, &char_md, &attr,
                                        &hids_ctrl_point_handles) !=
        NRF_SUCCESS)
      board_error_handler();
  }

  // Boot Keyboard Input Report (8 bytes)
  {
    ble_uuid_t uuid = {.uuid = UUID_CHAR_BOOT_KB_IN, .type = BLE_UUID_TYPE_BLE};
    ble_gatts_attr_md_t cccd_md = {0};
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    ble_gatts_char_md_t char_md = {0};
    char_md.char_props.read = 1;
    char_md.char_props.notify = 1;
    char_md.p_cccd_md = &cccd_md;

    ble_gatts_attr_md_t attr_md = {0};
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    attr_md.vloc = BLE_GATTS_VLOC_STACK;

    uint8_t init_boot[8] = {0};
    ble_gatts_attr_t attr = {
        .p_uuid = &uuid,
        .p_attr_md = &attr_md,
        .init_len = sizeof(init_boot),
        .init_offs = 0,
        .max_len = sizeof(init_boot),
        .p_value = init_boot,
    };
    if (sd_ble_gatts_characteristic_add(svc_handle, &char_md, &attr,
                                        &hids_boot_kb_in_handles) !=
        NRF_SUCCESS)
      board_error_handler();
  }

  // Boot Keyboard Output Report (1 byte LEDs)
  {
    ble_uuid_t uuid = {.uuid = UUID_CHAR_BOOT_KB_OUT,
                       .type = BLE_UUID_TYPE_BLE};
    ble_gatts_char_md_t char_md = {0};
    char_md.char_props.read = 1;
    char_md.char_props.write = 1;
    char_md.char_props.write_wo_resp = 1;

    ble_gatts_attr_md_t attr_md = {0};
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc = BLE_GATTS_VLOC_STACK;

    uint8_t init_led = 0;
    ble_gatts_attr_t attr = {
        .p_uuid = &uuid,
        .p_attr_md = &attr_md,
        .init_len = 1U,
        .init_offs = 0,
        .max_len = 1U,
        .p_value = &init_led,
    };
    if (sd_ble_gatts_characteristic_add(svc_handle, &char_md, &attr,
                                        &hids_boot_kb_out_handles) !=
        NRF_SUCCESS)
      board_error_handler();
  }

  // Report Characteristics (type: 0x01 = Input, 0x02 = Output)
  ble_add_hid_report_char(svc_handle, BLE_REPORT_ID_KEYBOARD, 0x01U,
                          sizeof(hid_nkro_kb_report_t), true,
                          &hids_kb_in_handles);
  ble_add_hid_report_char(svc_handle, BLE_REPORT_ID_KEYBOARD, 0x02U, 1U, false,
                          &hids_kb_out_handles);
  ble_add_hid_report_char(svc_handle, BLE_REPORT_ID_SYSTEM, 0x01U,
                          sizeof(uint16_t), true, &hids_sys_in_handles);
  ble_add_hid_report_char(svc_handle, BLE_REPORT_ID_CONSUMER, 0x01U,
                          sizeof(uint16_t), true, &hids_consumer_in_handles);
  ble_add_hid_report_char(svc_handle, BLE_REPORT_ID_MOUSE, 0x01U,
                          sizeof(hid_mouse_report_t), true,
                          &hids_mouse_in_handles);
  ble_add_hid_report_char(svc_handle, BLE_REPORT_ID_RAW_HID, 0x01U,
                          RAW_HID_EP_SIZE, true, &hids_raw_in_handles);
  ble_add_hid_report_char(svc_handle, BLE_REPORT_ID_RAW_HID, 0x02U,
                          RAW_HID_EP_SIZE, false, &hids_raw_out_handles);
}

//--------------------------------------------------------------------+
// Notification & Event Handling
//--------------------------------------------------------------------+

static bool ble_send_hvn(uint16_t value_handle, const void *data,
                         uint16_t len) {
  if (conn_handle == BLE_CONN_HANDLE_INVALID ||
      hvn_tx_pending >= BLE_HVN_TX_QUEUE_SIZE)
    return false;

  uint16_t hvx_len = len;
  ble_gatts_hvx_params_t hvx = {
      .handle = value_handle,
      .type = BLE_GATT_HVX_NOTIFICATION,
      .offset = 0,
      .p_len = &hvx_len,
      .p_data = (const uint8_t *)data,
  };

  const uint32_t err = sd_ble_gatts_hvx(conn_handle, &hvx);
  if (err == NRF_SUCCESS) {
    hvn_tx_pending++;
    return true;
  }

  return false;
}

static void ble_handle_gatts_write(const ble_gatts_evt_write_t *w) {
  if (w->handle == hids_protocol_mode_handles.value_handle && w->len >= 1U) {
    protocol_mode = w->data[0];
    return;
  }

  if (w->handle == hids_raw_out_handles.value_handle) {
    if (w->len == RAW_HID_EP_SIZE) {
      (void)command_enqueue_from(w->data, w->len, COMMAND_TRANSPORT_BLE);
    } else if (w->len > 0U && w->len < RAW_HID_EP_SIZE) {
      // Pad shorter BLE GATT writes to RAW_HID_EP_SIZE for compatibility
      uint8_t padded[RAW_HID_EP_SIZE] = {0};
      memcpy(padded, w->data, w->len);
      (void)command_enqueue_from(padded, RAW_HID_EP_SIZE,
                                 COMMAND_TRANSPORT_BLE);
    }
    return;
  }

  if (w->len >= 2U) {
    const bool notify_en = ((w->data[0] & 0x01U) != 0U);
    if (w->handle == hids_boot_kb_in_handles.cccd_handle)
      cccd_boot_kb_in = notify_en;
    else if (w->handle == hids_kb_in_handles.cccd_handle)
      cccd_kb_in = notify_en;
    else if (w->handle == hids_sys_in_handles.cccd_handle)
      cccd_sys_in = notify_en;
    else if (w->handle == hids_consumer_in_handles.cccd_handle)
      cccd_consumer_in = notify_en;
    else if (w->handle == hids_mouse_in_handles.cccd_handle)
      cccd_mouse_in = notify_en;
    else if (w->handle == hids_raw_in_handles.cccd_handle)
      cccd_raw_in = notify_en;
    else if (w->handle == bas_level_handles.cccd_handle)
      cccd_battery = notify_en;
  }
}

static void ble_dispatch_evt(const ble_evt_t *evt) {
  switch (evt->header.evt_id) {
  case BLE_GAP_EVT_CONNECTED: {
    conn_handle = evt->evt.gap_evt.conn_handle;
    ble_state = BLE_STATE_CONNECTED;
    hvn_tx_pending = 0;
    protocol_mode = HID_PROTOCOL_MODE_REPORT;
    ble_notify_activity();

    // Request preferred low-latency connection parameters
    ble_gap_conn_params_t ppcp = {
        .min_conn_interval = BLE_MIN_CONN_INTERVAL,
        .max_conn_interval = BLE_MAX_CONN_INTERVAL,
        .slave_latency = BLE_SLAVE_LATENCY,
        .conn_sup_timeout = BLE_CONN_SUP_TIMEOUT,
    };
    (void)sd_ble_gap_conn_param_update(conn_handle, &ppcp);
    break;
  }

  case BLE_GAP_EVT_DISCONNECTED: {
    conn_handle = BLE_CONN_HANDLE_INVALID;
    ble_state = BLE_STATE_IDLE;
    hvn_tx_pending = 0;
    cccd_boot_kb_in = false;
    cccd_kb_in = false;
    cccd_sys_in = false;
    cccd_consumer_in = false;
    cccd_mouse_in = false;
    cccd_raw_in = false;
    cccd_battery = false;

    if (transport_mode != BLE_TRANSPORT_USB)
      ble_configure_advertising(BLE_ADV_INTERVAL_FAST);
    break;
  }

  case BLE_GAP_EVT_SEC_PARAMS_REQUEST: {
    ble_gap_sec_params_t sec_params = {0};
    sec_params.bond = 1;
    sec_params.mitm = 0;
    sec_params.lesc = 0;
    sec_params.keypress = 0;
    sec_params.io_caps = BLE_GAP_IO_CAPS_NONE;
    sec_params.oob = 0;
    sec_params.min_key_size = 7U;
    sec_params.max_key_size = 16U;
    sec_params.kdist_own.enc = 1;
    sec_params.kdist_own.id = 1;
    sec_params.kdist_peer.enc = 1;
    sec_params.kdist_peer.id = 1;

    ble_gap_sec_keyset_t keyset = {0};
    keyset.keys_own.p_enc_key = &bond_own_enc_key;
    keyset.keys_peer.p_id_key = &bond_peer_id_key;

    (void)sd_ble_gap_sec_params_reply(
        conn_handle, BLE_GAP_SEC_STATUS_SUCCESS, &sec_params, &keyset);
    break;
  }

  case BLE_GAP_EVT_AUTH_STATUS: {
    if (evt->evt.gap_evt.params.auth_status.auth_status ==
        BLE_GAP_SEC_STATUS_SUCCESS)
      bond_valid = true;
    break;
  }

  case BLE_GAP_EVT_SEC_INFO_REQUEST: {
    if (bond_valid &&
        evt->evt.gap_evt.params.sec_info_request.enc_info != 0U) {
      (void)sd_ble_gap_sec_info_reply(conn_handle, &bond_own_enc_key.enc_info,
                                      NULL, NULL);
    } else {
      (void)sd_ble_gap_sec_info_reply(conn_handle, NULL, NULL, NULL);
    }
    break;
  }

  case BLE_GAP_EVT_PHY_UPDATE_REQUEST: {
    const ble_gap_phys_t phys = {
        .rx_phys = BLE_GAP_PHY_AUTO,
        .tx_phys = BLE_GAP_PHY_AUTO,
    };
    (void)sd_ble_gap_phy_update(conn_handle, &phys);
    break;
  }

  case BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST: {
    (void)sd_ble_gap_data_length_update(conn_handle, NULL, NULL);
    break;
  }

  case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST: {
    (void)sd_ble_gap_conn_param_update(
        conn_handle,
        &evt->evt.gap_evt.params.conn_param_update_request.conn_params);
    break;
  }

  case BLE_GATTS_EVT_SYS_ATTR_MISSING: {
    (void)sd_ble_gatts_sys_attr_set(conn_handle, NULL, 0, 0);
    break;
  }

  case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST: {
    (void)sd_ble_gatts_exchange_mtu_reply(conn_handle, BLE_ATT_MTU_SIZE);
    break;
  }

  case BLE_GATTS_EVT_WRITE: {
    ble_handle_gatts_write(&evt->evt.gatts_evt.params.write);
    break;
  }

  case BLE_GATTS_EVT_HVN_TX_COMPLETE: {
    const uint8_t count = evt->evt.gatts_evt.params.hvn_tx_complete.count;
    hvn_tx_pending = (count >= hvn_tx_pending) ? 0U
                                               : (uint8_t)(hvn_tx_pending - count);
    break;
  }

  default:
    break;
  }
}

void nrf52_ble_sd_evt_poll(void) {
  if (!nrf52_is_sd_enabled())
    return;

  while (1) {
    uint16_t evt_len = (uint16_t)sizeof(ble_evt_buf);
    const uint32_t err = sd_ble_evt_get(ble_evt_buf, &evt_len);
    if (err != NRF_SUCCESS)
      break;
    ble_dispatch_evt((const ble_evt_t *)(const void *)ble_evt_buf);
  }
}

//--------------------------------------------------------------------+
// Battery Conversion Helper
//--------------------------------------------------------------------+

static uint8_t battery_mv_to_percent(uint16_t mv) {
#if defined(BLE_BATTERY_ADC_CHANNEL)
  // Standard single-cell LiPo discharge curve (4200 mV .. 3300 mV)
  if (mv >= 4200U)
    return 100U;
  if (mv <= 3300U)
    return 0U;
  if (mv >= 3950U)
    return (uint8_t)(75U + ((uint32_t)(mv - 3950U) * 25U) / 250U);
  if (mv >= 3700U)
    return (uint8_t)(25U + ((uint32_t)(mv - 3700U) * 50U) / 250U);
  return (uint8_t)(((uint32_t)(mv - 3300U) * 25U) / 400U);
#else
  // Direct VDD measurement (e.g. 3.0V regulated / CR2032 / 2xAA: 3000..2000 mV)
  if (mv >= 3000U)
    return 100U;
  if (mv <= 2000U)
    return 0U;
  return (uint8_t)(((uint32_t)(mv - 2000U) * 100U) / 1000U);
#endif
}

//--------------------------------------------------------------------+
// Public Bluetooth API
//--------------------------------------------------------------------+

void ble_init(void) {
  if (!nrf52_is_sd_enabled())
    return;

  uint32_t ram_start = (uint32_t)(uintptr_t)__ram_start__;
  ble_cfg_t cfg;

  // Configure GAP role count: 1 peripheral, 0 central
  memset(&cfg, 0, sizeof(cfg));
  cfg.gap_cfg.role_count_cfg.periph_role_count = 1U;
  cfg.gap_cfg.role_count_cfg.central_role_count = 0U;
  cfg.gap_cfg.role_count_cfg.central_sec_count = 0U;
  if (sd_ble_cfg_set(BLE_GAP_CFG_ROLE_COUNT, &cfg, ram_start) != NRF_SUCCESS)
    board_error_handler();

  // Configure GAP connection count & event length
  memset(&cfg, 0, sizeof(cfg));
  cfg.conn_cfg.conn_cfg_tag = BLE_CONN_CFG_TAG;
  cfg.conn_cfg.params.gap_conn_cfg.conn_count = 1U;
  cfg.conn_cfg.params.gap_conn_cfg.event_length = BLE_GAP_EVENT_LENGTH_DEFAULT;
  if (sd_ble_cfg_set(BLE_CONN_CFG_GAP, &cfg, ram_start) != NRF_SUCCESS)
    board_error_handler();

  // Configure ATT MTU size (128 bytes for single-packet 64-byte Raw HID)
  memset(&cfg, 0, sizeof(cfg));
  cfg.conn_cfg.conn_cfg_tag = BLE_CONN_CFG_TAG;
  cfg.conn_cfg.params.gatt_conn_cfg.att_mtu = BLE_ATT_MTU_SIZE;
  if (sd_ble_cfg_set(BLE_CONN_CFG_GATT, &cfg, ram_start) != NRF_SUCCESS)
    board_error_handler();

  // Configure GATTS HVN notification queue depth
  memset(&cfg, 0, sizeof(cfg));
  cfg.conn_cfg.conn_cfg_tag = BLE_CONN_CFG_TAG;
  cfg.conn_cfg.params.gatts_conn_cfg.hvn_tx_queue_size = BLE_HVN_TX_QUEUE_SIZE;
  if (sd_ble_cfg_set(BLE_CONN_CFG_GATTS, &cfg, ram_start) != NRF_SUCCESS)
    board_error_handler();

  // Configure GATTS attribute table size
  memset(&cfg, 0, sizeof(cfg));
  cfg.gatts_cfg.attr_tab_size.attr_tab_size = BLE_GATTS_ATTR_TAB_SIZE;
  if (sd_ble_cfg_set(BLE_GATTS_CFG_ATTR_TAB_SIZE, &cfg, ram_start) !=
      NRF_SUCCESS)
    board_error_handler();

  // Enable SoftDevice BLE stack
  if (sd_ble_enable(&ram_start) != NRF_SUCCESS)
    board_error_handler();

  // Configure GAP device name, appearance, and preferred connection parameters
  ble_gap_conn_sec_mode_t sec_mode;
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
  const char *name = BLE_DEVICE_NAME;
  (void)sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *)name,
                                   (uint16_t)strlen(name));
  (void)sd_ble_gap_appearance_set(BLE_APPEARANCE_HID_KEYBOARD);

  ble_gap_conn_params_t ppcp = {
      .min_conn_interval = BLE_MIN_CONN_INTERVAL,
      .max_conn_interval = BLE_MAX_CONN_INTERVAL,
      .slave_latency = BLE_SLAVE_LATENCY,
      .conn_sup_timeout = BLE_CONN_SUP_TIMEOUT,
  };
  (void)sd_ble_gap_ppcp_set(&ppcp);

  // Register GATT services
  ble_dis_init();
  ble_bas_init();
  ble_hids_init();

  ble_state = BLE_STATE_IDLE;
  last_activity_ms = timer_read();
  ble_battery_update();

  // Begin advertising immediately
  ble_configure_advertising(BLE_ADV_INTERVAL_FAST);
}

void ble_task(void) {
  nrf52_soc_evt_poll();
  nrf52_ble_sd_evt_poll();

  if (timer_elapsed(last_battery_update_ms) >= BLE_BATTERY_UPDATE_INTERVAL_MS)
    ble_battery_update();
}

ble_state_t ble_get_state(void) { return ble_state; }

bool ble_is_connected(void) { return conn_handle != BLE_CONN_HANDLE_INVALID; }

bool ble_is_usb_active(void) {
  switch (transport_mode) {
  case BLE_TRANSPORT_USB:
    return true;
  case BLE_TRANSPORT_BLE:
    return false;
  case BLE_TRANSPORT_AUTO:
  default:
    return nrf52_is_usb_vbus_present() && tud_mounted();
  }
}

ble_transport_mode_t ble_get_transport_mode(void) { return transport_mode; }

void ble_set_transport_mode(ble_transport_mode_t mode) {
  transport_mode = mode;
  ble_notify_activity();

  if (mode == BLE_TRANSPORT_USB) {
    if (ble_state == BLE_STATE_ADVERTISING) {
      (void)sd_ble_gap_adv_stop(adv_handle);
      ble_state = BLE_STATE_IDLE;
    }
  } else if (conn_handle == BLE_CONN_HANDLE_INVALID &&
             ble_state != BLE_STATE_ADVERTISING) {
    ble_configure_advertising(BLE_ADV_INTERVAL_FAST);
  }
}

void ble_toggle_transport(void) {
  switch (transport_mode) {
  case BLE_TRANSPORT_AUTO:
    ble_set_transport_mode(ble_is_usb_active() ? BLE_TRANSPORT_BLE
                                               : BLE_TRANSPORT_USB);
    break;
  case BLE_TRANSPORT_USB:
    ble_set_transport_mode(BLE_TRANSPORT_BLE);
    break;
  case BLE_TRANSPORT_BLE:
  default:
    ble_set_transport_mode(BLE_TRANSPORT_AUTO);
    break;
  }
}

void ble_start_advertising(bool disconnect_current) {
  ble_notify_activity();

  if (disconnect_current && conn_handle != BLE_CONN_HANDLE_INVALID) {
    bond_valid = false;
    (void)sd_ble_gap_disconnect(conn_handle,
                                BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    return;
  }

  if (conn_handle == BLE_CONN_HANDLE_INVALID)
    ble_configure_advertising(BLE_ADV_INTERVAL_FAST);
}

bool ble_hid_ready(void) {
  return conn_handle != BLE_CONN_HANDLE_INVALID &&
         hvn_tx_pending < BLE_HVN_TX_QUEUE_SIZE;
}

bool ble_hid_send_keyboard(const void *report, uint16_t len) {
  if (conn_handle == BLE_CONN_HANDLE_INVALID)
    return false;

  if (protocol_mode == HID_PROTOCOL_MODE_BOOT) {
    if (!cccd_boot_kb_in || len < 8U)
      return false;
    // First 8 bytes of hid_nkro_kb_report_t are standard 6KRO boot report
    return ble_send_hvn(hids_boot_kb_in_handles.value_handle, report, 8U);
  }

  if (!cccd_kb_in)
    return false;
  return ble_send_hvn(hids_kb_in_handles.value_handle, report, len);
}

bool ble_hid_send_system(uint16_t report) {
  if (!cccd_sys_in)
    return false;
  return ble_send_hvn(hids_sys_in_handles.value_handle, &report,
                      sizeof(report));
}

bool ble_hid_send_consumer(uint16_t report) {
  if (!cccd_consumer_in)
    return false;
  return ble_send_hvn(hids_consumer_in_handles.value_handle, &report,
                      sizeof(report));
}

bool ble_hid_send_mouse(const void *report, uint16_t len) {
  if (!cccd_mouse_in)
    return false;
  return ble_send_hvn(hids_mouse_in_handles.value_handle, report, len);
}

bool ble_raw_hid_ready(void) { return ble_hid_ready() && cccd_raw_in; }

bool ble_raw_hid_send(const uint8_t *buf, uint16_t len) {
  if (!cccd_raw_in)
    return false;
  return ble_send_hvn(hids_raw_in_handles.value_handle, buf, len);
}

uint8_t ble_battery_get_level(void) { return battery_level; }

void ble_battery_update(void) {
  last_battery_update_ms = timer_read();
  const uint16_t mv = analog_read_battery_mv();
  const uint8_t new_level = battery_mv_to_percent(mv);

  const bool changed = (new_level != battery_level);
  battery_level = new_level;

  ble_gatts_value_t val = {
      .len = sizeof(battery_level),
      .offset = 0,
      .p_value = &battery_level,
  };
  (void)sd_ble_gatts_value_set(BLE_CONN_HANDLE_INVALID,
                               bas_level_handles.value_handle, &val);

  if (changed && cccd_battery && conn_handle != BLE_CONN_HANDLE_INVALID)
    (void)ble_send_hvn(bas_level_handles.value_handle, &battery_level,
                       sizeof(battery_level));
}

void ble_notify_activity(void) {
  last_activity_ms = timer_read();

  if (power_state != BLE_POWER_ACTIVE) {
    power_state = BLE_POWER_ACTIVE;
    analog_set_scan_interval(0UL);

    if (conn_handle == BLE_CONN_HANDLE_INVALID &&
        ble_state == BLE_STATE_ADVERTISING)
      ble_configure_advertising(BLE_ADV_INTERVAL_FAST);
  }
}

void ble_power_manage(void) {
  if (ble_is_usb_active()) {
    // Full performance on USB power
    if (power_state != BLE_POWER_ACTIVE) {
      power_state = BLE_POWER_ACTIVE;
      analog_set_scan_interval(0UL);
    }
    return;
  }

  const uint32_t inactive_ms = timer_elapsed(last_activity_ms);

  if (BLE_SLEEP_TIMEOUT_MS > 0U && inactive_ms >= BLE_SLEEP_TIMEOUT_MS) {
    if (power_state != BLE_POWER_SLEEP) {
      power_state = BLE_POWER_SLEEP;
      analog_set_scan_interval(BLE_SLEEP_SCAN_INTERVAL_US);

      if (conn_handle == BLE_CONN_HANDLE_INVALID &&
          ble_state == BLE_STATE_ADVERTISING)
        ble_configure_advertising(BLE_ADV_INTERVAL_SLOW);
    }
  } else if (BLE_IDLE_TIMEOUT_MS > 0U && inactive_ms >= BLE_IDLE_TIMEOUT_MS) {
    if (power_state != BLE_POWER_IDLE) {
      power_state = BLE_POWER_IDLE;
      analog_set_scan_interval(BLE_IDLE_SCAN_INTERVAL_US);
    }
  }

  // Enter System ON low-power sleep until next RTC1/SAADC/BLE/USB event
  if (nrf52_is_sd_enabled())
    (void)sd_app_evt_wait();
  else
    __WFE();
}

#endif
