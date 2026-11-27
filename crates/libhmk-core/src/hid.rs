//! USB & BLE HID report state, 6KRO+NKRO encoding, and standard HID Report Descriptors.

use crate::keycodes::*;

pub const NUM_NKRO_BYTES: usize = 20;
pub const RAW_HID_EP_SIZE: usize = 64;
pub const RAW_HID_USAGE_PAGE: u16 = 0xFFAB;
pub const RAW_HID_USAGE: u8 = 0xAB;

pub const REPORT_ID_SYSTEM_CONTROL: u8 = 1;
pub const REPORT_ID_CONSUMER_CONTROL: u8 = 2;
pub const REPORT_ID_MOUSE: u8 = 3;

// Unified BLE HID Report IDs (single HOGP Report Map)
pub const BLE_REPORT_ID_KEYBOARD: u8 = 1;
pub const BLE_REPORT_ID_SYSTEM: u8 = 2;
pub const BLE_REPORT_ID_CONSUMER: u8 = 3;
pub const BLE_REPORT_ID_MOUSE: u8 = 4;
pub const BLE_REPORT_ID_RAW_HID: u8 = 5;

/// 28-byte hybrid 6KRO + NKRO keyboard report matching `hid_nkro_kb_report_t`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct KeyboardReport {
    pub modifiers: u8,
    pub reserved: u8,
    pub keycodes: [u8; 6],
    pub bitmap: [u8; NUM_NKRO_BYTES],
}

impl KeyboardReport {
    pub const SIZE: usize = 28;

    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        let mut out = [0u8; Self::SIZE];
        out[0] = self.modifiers;
        out[1] = self.reserved;
        out[2..8].copy_from_slice(&self.keycodes);
        out[8..28].copy_from_slice(&self.bitmap);
        out
    }

    pub fn to_boot_bytes(&self) -> [u8; 8] {
        let mut out = [0u8; 8];
        out[0] = self.modifiers;
        out[1] = self.reserved;
        out[2..8].copy_from_slice(&self.keycodes);
        out
    }
}

/// 5-byte HID mouse report matching TinyUSB `hid_mouse_report_t`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct MouseReport {
    pub buttons: u8,
    pub x: i8,
    pub y: i8,
    pub wheel: i8,
    pub pan: i8,
}

impl MouseReport {
    pub const SIZE: usize = 5;

    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        [
            self.buttons,
            self.x as u8,
            self.y as u8,
            self.wheel as u8,
            self.pan as u8,
        ]
    }
}

/// Snapshot of changed HID reports ready to be transmitted over USB or BLE.
#[derive(Debug, Clone, Copy, Default)]
pub struct PendingHidReports {
    pub keyboard: Option<KeyboardReport>,
    pub system: Option<u16>,
    pub consumer: Option<u16>,
    pub mouse: Option<MouseReport>,
}

impl PendingHidReports {
    pub fn is_empty(&self) -> bool {
        self.keyboard.is_none()
            && self.system.is_none()
            && self.consumer.is_none()
            && self.mouse.is_none()
    }
}

/// Manages active HID reports and delta tracking.
#[derive(Debug, Default)]
pub struct HidState {
    num_6kro_keys: usize,
    kb_report: KeyboardReport,
    system_report: u16,
    consumer_report: u16,
    mouse_report: MouseReport,

    prev_kb_report: KeyboardReport,
    prev_system_report: u16,
    prev_consumer_report: u16,
    prev_mouse_report: MouseReport,
}

impl HidState {
    pub const fn new() -> Self {
        Self {
            num_6kro_keys: 0,
            kb_report: KeyboardReport {
                modifiers: 0,
                reserved: 0,
                keycodes: [0; 6],
                bitmap: [0; NUM_NKRO_BYTES],
            },
            system_report: 0,
            consumer_report: 0,
            mouse_report: MouseReport {
                buttons: 0,
                x: 0,
                y: 0,
                wheel: 0,
                pan: 0,
            },
            prev_kb_report: KeyboardReport {
                modifiers: 0,
                reserved: 0,
                keycodes: [0; 6],
                bitmap: [0; NUM_NKRO_BYTES],
            },
            prev_system_report: 0,
            prev_consumer_report: 0,
            prev_mouse_report: MouseReport {
                buttons: 0,
                x: 0,
                y: 0,
                wheel: 0,
                pan: 0,
            },
        }
    }

    pub fn keycode_add(&mut self, keycode: u8) {
        let hid_code = keycode_to_hid(keycode);
        if hid_code == 0 {
            return;
        }
        if is_keyboard_keycode(keycode) {
            let code = hid_code as u8;
            let found = self.kb_report.keycodes[..self.num_6kro_keys].contains(&code);
            if !found {
                if self.num_6kro_keys == 6 {
                    self.kb_report.keycodes.copy_within(1..6, 0);
                    self.num_6kro_keys = 5;
                }
                self.kb_report.keycodes[self.num_6kro_keys] = code;
                self.num_6kro_keys += 1;
            }
            let byte_idx = (code / 8) as usize;
            if byte_idx < NUM_NKRO_BYTES {
                self.kb_report.bitmap[byte_idx] |= 1u8 << (code & 7);
            }
        } else if is_modifier_keycode(keycode) {
            self.kb_report.modifiers |= hid_code as u8;
        } else if is_system_keycode(keycode) {
            self.system_report = hid_code;
        } else if is_consumer_keycode(keycode) {
            self.consumer_report = hid_code;
        } else if is_mouse_keycode(keycode) {
            self.mouse_report.buttons |= hid_code as u8;
        }
    }

    pub fn keycode_remove(&mut self, keycode: u8) {
        let hid_code = keycode_to_hid(keycode);
        if hid_code == 0 {
            return;
        }
        if is_keyboard_keycode(keycode) {
            let code = hid_code as u8;
            if let Some(pos) = self.kb_report.keycodes[..self.num_6kro_keys]
                .iter()
                .position(|&c| c == code)
            {
                self.kb_report.keycodes.copy_within(pos + 1..self.num_6kro_keys, pos);
                self.num_6kro_keys -= 1;
                self.kb_report.keycodes[self.num_6kro_keys] = 0;
            }
            let byte_idx = (code / 8) as usize;
            if byte_idx < NUM_NKRO_BYTES {
                self.kb_report.bitmap[byte_idx] &= !(1u8 << (code & 7));
            }
        } else if is_modifier_keycode(keycode) {
            self.kb_report.modifiers &= !(hid_code as u8);
        } else if is_system_keycode(keycode) {
            if self.system_report == hid_code {
                self.system_report = 0;
            }
        } else if is_consumer_keycode(keycode) {
            if self.consumer_report == hid_code {
                self.consumer_report = 0;
            }
        } else if is_mouse_keycode(keycode) {
            self.mouse_report.buttons &= !(hid_code as u8);
        }
    }

    /// Take all changed reports and update the previous-sent snapshots.
    pub fn take_pending(&mut self) -> PendingHidReports {
        let mut out = PendingHidReports::default();
        if self.kb_report != self.prev_kb_report {
            self.prev_kb_report = self.kb_report;
            out.keyboard = Some(self.kb_report);
        }
        if self.system_report != self.prev_system_report {
            self.prev_system_report = self.system_report;
            out.system = Some(self.system_report);
        }
        if self.consumer_report != self.prev_consumer_report {
            self.prev_consumer_report = self.consumer_report;
            out.consumer = Some(self.consumer_report);
        }
        if self.mouse_report != self.prev_mouse_report {
            self.prev_mouse_report = self.mouse_report;
            out.mouse = Some(self.mouse_report);
        }
        out
    }

    pub fn keyboard_report(&self) -> &KeyboardReport {
        &self.kb_report
    }
}

//--------------------------------------------------------------------+
// USB & BLE HID Report Descriptors (byte-exact with `src/usb_descriptors.c`)
//--------------------------------------------------------------------+

/// USB Keyboard interface HID report descriptor (NKRO + 6KRO boot compatible).
pub const DESC_KEYBOARD_REPORT: &[u8] = &[
    0x05, 0x01, // Usage Page (Generic Desktop)
    0x09, 0x06, // Usage (Keyboard)
    0xA1, 0x01, // Collection (Application)
    // 8 bits modifiers
    0x05, 0x07, //   Usage Page (Keyboard/Keypad)
    0x19, 0xE0, //   Usage Minimum (224)
    0x29, 0xE7, //   Usage Maximum (231)
    0x15, 0x00, //   Logical Minimum (0)
    0x25, 0x01, //   Logical Maximum (1)
    0x95, 0x08, //   Report Count (8)
    0x75, 0x01, //   Report Size (1)
    0x81, 0x02, //   Input (Data, Var, Abs)
    // 8 bits reserved
    0x95, 0x01, 0x75, 0x08, 0x81, 0x01, // Input (Const)
    // 5-bit LEDs + 3-bit padding
    0x05, 0x08, 0x19, 0x01, 0x29, 0x05, 0x95, 0x05, 0x75, 0x01, 0x91, 0x02,
    0x95, 0x01, 0x75, 0x03, 0x91, 0x01,
    // 6-byte padding for 6KRO boot layout compatibility
    0x95, 0x30, 0x75, 0x01, 0x81, 0x01,
    // 160-bit NKRO bitmap
    0x05, 0x07, 0x19, 0x00, 0x29, 0x9F, 0x15, 0x00, 0x25, 0x01, 0x95, 0xA0, 0x75, 0x01, 0x81, 0x02,
    0xC0, // End Collection
];

/// USB Generic HID interface report descriptor (System ID 1, Consumer ID 2, Mouse ID 3).
pub const DESC_HID_REPORT: &[u8] = &[
    // System Control (Report ID 1)
    0x05, 0x01, 0x09, 0x80, 0xA1, 0x01,
    0x85, REPORT_ID_SYSTEM_CONTROL,
    0x15, 0x00, 0x26, 0xFF, 0x00, 0x19, 0x00, 0x2A, 0xFF, 0x00,
    0x75, 0x10, 0x95, 0x01, 0x81, 0x00,
    0xC0,
    // Consumer Control (Report ID 2)
    0x05, 0x0C, 0x09, 0x01, 0xA1, 0x01,
    0x85, REPORT_ID_CONSUMER_CONTROL,
    0x15, 0x00, 0x26, 0xFF, 0x03, 0x19, 0x00, 0x2A, 0xFF, 0x03,
    0x75, 0x10, 0x95, 0x01, 0x81, 0x00,
    0xC0,
    // Mouse (Report ID 3)
    0x05, 0x01, 0x09, 0x02, 0xA1, 0x01,
    0x85, REPORT_ID_MOUSE,
    0x09, 0x01, 0xA1, 0x00,
    0x05, 0x09, 0x19, 0x01, 0x29, 0x05, 0x15, 0x00, 0x25, 0x01, 0x95, 0x05, 0x75, 0x01, 0x81, 0x02,
    0x95, 0x01, 0x75, 0x03, 0x81, 0x01,
    0x05, 0x01, 0x09, 0x30, 0x09, 0x31, 0x09, 0x38, 0x15, 0x81, 0x25, 0x7F, 0x75, 0x08, 0x95, 0x03, 0x81, 0x06,
    0x05, 0x0C, 0x0A, 0x38, 0x02, 0x15, 0x81, 0x25, 0x7F, 0x75, 0x08, 0x95, 0x01, 0x81, 0x06,
    0xC0, 0xC0,
];

/// USB Raw HID interface report descriptor (`0xFFAB` / `0xAB`, 64-byte IN + OUT for `hmkconf`).
pub const DESC_RAW_HID_REPORT: &[u8] = &[
    0x06, 0xAB, 0xFF, // Usage Page (0xFFAB)
    0x09, 0xAB,       // Usage (0xAB)
    0xA1, 0x01,       // Collection (Application)
    // Data to host (64 bytes)
    0x09, 0xAC, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x95, 0x40, 0x75, 0x08, 0x81, 0x02,
    // Data from host (64 bytes)
    0x09, 0xAD, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x95, 0x40, 0x75, 0x08, 0x91, 0x82,
    0xC0,
];

/// Unified BLE HOGP Report Map (ID 1 Keyboard, ID 2 System, ID 3 Consumer, ID 4 Mouse, ID 5 Raw HID).
pub const BLE_HID_REPORT_MAP: &[u8] = &[
    // Report ID 1: Keyboard (NKRO + 6KRO layout)
    0x05, 0x01, 0x09, 0x06, 0xA1, 0x01,
    0x85, BLE_REPORT_ID_KEYBOARD,
    0x05, 0x07, 0x19, 0xE0, 0x29, 0xE7, 0x15, 0x00, 0x25, 0x01, 0x95, 0x08, 0x75, 0x01, 0x81, 0x02,
    0x95, 0x01, 0x75, 0x08, 0x81, 0x01,
    0x05, 0x08, 0x19, 0x01, 0x29, 0x05, 0x95, 0x05, 0x75, 0x01, 0x91, 0x02,
    0x95, 0x01, 0x75, 0x03, 0x91, 0x01,
    0x95, 0x30, 0x75, 0x01, 0x81, 0x01,
    0x05, 0x07, 0x19, 0x00, 0x29, 0x9F, 0x15, 0x00, 0x25, 0x01, 0x95, 0xA0, 0x75, 0x01, 0x81, 0x02,
    0xC0,
    // Report ID 2: System Control
    0x05, 0x01, 0x09, 0x80, 0xA1, 0x01,
    0x85, BLE_REPORT_ID_SYSTEM,
    0x15, 0x00, 0x26, 0xFF, 0x00, 0x19, 0x00, 0x2A, 0xFF, 0x00, 0x75, 0x10, 0x95, 0x01, 0x81, 0x00,
    0xC0,
    // Report ID 3: Consumer Control
    0x05, 0x0C, 0x09, 0x01, 0xA1, 0x01,
    0x85, BLE_REPORT_ID_CONSUMER,
    0x15, 0x00, 0x26, 0xFF, 0x03, 0x19, 0x00, 0x2A, 0xFF, 0x03, 0x75, 0x10, 0x95, 0x01, 0x81, 0x00,
    0xC0,
    // Report ID 4: Mouse
    0x05, 0x01, 0x09, 0x02, 0xA1, 0x01,
    0x85, BLE_REPORT_ID_MOUSE,
    0x09, 0x01, 0xA1, 0x00,
    0x05, 0x09, 0x19, 0x01, 0x29, 0x05, 0x15, 0x00, 0x25, 0x01, 0x95, 0x05, 0x75, 0x01, 0x81, 0x02,
    0x95, 0x01, 0x75, 0x03, 0x81, 0x01,
    0x05, 0x01, 0x09, 0x30, 0x09, 0x31, 0x09, 0x38, 0x15, 0x81, 0x25, 0x7F, 0x75, 0x08, 0x95, 0x03, 0x81, 0x06,
    0x05, 0x0C, 0x0A, 0x38, 0x02, 0x15, 0x81, 0x25, 0x7F, 0x75, 0x08, 0x95, 0x01, 0x81, 0x06,
    0xC0, 0xC0,
    // Report ID 5: Raw HID (hmkconf over BLE)
    0x06, 0xAB, 0xFF, 0x09, 0xAB, 0xA1, 0x01,
    0x85, BLE_REPORT_ID_RAW_HID,
    0x09, 0xAC, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x95, 0x40, 0x75, 0x08, 0x81, 0x02,
    0x09, 0xAD, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x95, 0x40, 0x75, 0x08, 0x91, 0x82,
    0xC0,
];
