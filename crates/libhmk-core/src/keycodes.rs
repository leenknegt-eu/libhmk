//! Keycode definitions and HID usage lookup table, matching `include/keycodes.h` & `src/keycodes.c`.

pub const KC_NO: u8 = 0x00;
pub const KC_TRANSPARENT: u8 = 0x01;

// Full keycode list (mirrors `include/keycodes.h`)
pub const KC_B: u8 = 0x03;
pub const KC_C: u8 = 0x04;
pub const KC_D: u8 = 0x05;
pub const KC_E: u8 = 0x06;
pub const KC_F: u8 = 0x07;
pub const KC_G: u8 = 0x08;
pub const KC_H: u8 = 0x09;
pub const KC_I: u8 = 0x0A;
pub const KC_J: u8 = 0x0B;
pub const KC_K: u8 = 0x0C;
pub const KC_L: u8 = 0x0D;
pub const KC_M: u8 = 0x0E;
pub const KC_N: u8 = 0x0F;
pub const KC_O: u8 = 0x10;
pub const KC_P: u8 = 0x11;
pub const KC_Q: u8 = 0x12;
pub const KC_R: u8 = 0x13;
pub const KC_S: u8 = 0x14;
pub const KC_T: u8 = 0x15;
pub const KC_U: u8 = 0x16;
pub const KC_V: u8 = 0x17;
pub const KC_W: u8 = 0x18;
pub const KC_X: u8 = 0x19;
pub const KC_Y: u8 = 0x1A;
pub const KC_Z: u8 = 0x1B;
pub const KC_1: u8 = 0x1C;
pub const KC_2: u8 = 0x1D;
pub const KC_3: u8 = 0x1E;
pub const KC_4: u8 = 0x1F;
pub const KC_5: u8 = 0x20;
pub const KC_6: u8 = 0x21;
pub const KC_7: u8 = 0x22;
pub const KC_8: u8 = 0x23;
pub const KC_9: u8 = 0x24;
pub const KC_0: u8 = 0x25;
pub const KC_ENTER: u8 = 0x26;
pub const KC_ESCAPE: u8 = 0x27;
pub const KC_BACKSPACE: u8 = 0x28;
pub const KC_TAB: u8 = 0x29;
pub const KC_SPACE: u8 = 0x2A;
pub const KC_MINUS: u8 = 0x2B;
pub const KC_EQUAL: u8 = 0x2C;
pub const KC_LEFT_BRACKET: u8 = 0x2D;
pub const KC_RIGHT_BRACKET: u8 = 0x2E;
pub const KC_BACKSLASH: u8 = 0x2F;
pub const KC_NONUS_HASH: u8 = 0x30;
pub const KC_SEMICOLON: u8 = 0x31;
pub const KC_QUOTE: u8 = 0x32;
pub const KC_GRAVE: u8 = 0x33;
pub const KC_COMMA: u8 = 0x34;
pub const KC_DOT: u8 = 0x35;
pub const KC_SLASH: u8 = 0x36;
pub const KC_CAPS_LOCK: u8 = 0x37;
pub const KC_F1: u8 = 0x38;
pub const KC_F2: u8 = 0x39;
pub const KC_F3: u8 = 0x3A;
pub const KC_F4: u8 = 0x3B;
pub const KC_F5: u8 = 0x3C;
pub const KC_F6: u8 = 0x3D;
pub const KC_F7: u8 = 0x3E;
pub const KC_F8: u8 = 0x3F;
pub const KC_F9: u8 = 0x40;
pub const KC_F10: u8 = 0x41;
pub const KC_F11: u8 = 0x42;
pub const KC_F12: u8 = 0x43;
pub const KC_PRINT_SCREEN: u8 = 0x44;
pub const KC_SCROLL_LOCK: u8 = 0x45;
pub const KC_PAUSE: u8 = 0x46;
pub const KC_INSERT: u8 = 0x47;
pub const KC_HOME: u8 = 0x48;
pub const KC_PAGE_UP: u8 = 0x49;
pub const KC_DELETE: u8 = 0x4A;
pub const KC_END: u8 = 0x4B;
pub const KC_PAGE_DOWN: u8 = 0x4C;
pub const KC_RIGHT: u8 = 0x4D;
pub const KC_LEFT: u8 = 0x4E;
pub const KC_DOWN: u8 = 0x4F;
pub const KC_UP: u8 = 0x50;
pub const KC_NUM_LOCK: u8 = 0x51;
pub const KC_KP_SLASH: u8 = 0x52;
pub const KC_KP_ASTERISK: u8 = 0x53;
pub const KC_KP_MINUS: u8 = 0x54;
pub const KC_KP_PLUS: u8 = 0x55;
pub const KC_KP_ENTER: u8 = 0x56;
pub const KC_KP_1: u8 = 0x57;
pub const KC_KP_2: u8 = 0x58;
pub const KC_KP_3: u8 = 0x59;
pub const KC_KP_4: u8 = 0x5A;
pub const KC_KP_5: u8 = 0x5B;
pub const KC_KP_6: u8 = 0x5C;
pub const KC_KP_7: u8 = 0x5D;
pub const KC_KP_8: u8 = 0x5E;
pub const KC_KP_9: u8 = 0x5F;
pub const KC_KP_0: u8 = 0x60;
pub const KC_KP_DOT: u8 = 0x61;
pub const KC_NONUS_BACKSLASH: u8 = 0x62;
pub const KC_APPLICATION: u8 = 0x63;
pub const KC_F13: u8 = 0x64;
pub const KC_F14: u8 = 0x65;
pub const KC_F15: u8 = 0x66;
pub const KC_F16: u8 = 0x67;
pub const KC_F17: u8 = 0x68;
pub const KC_F18: u8 = 0x69;
pub const KC_F19: u8 = 0x6A;
pub const KC_F20: u8 = 0x6B;
pub const KC_F21: u8 = 0x6C;
pub const KC_F22: u8 = 0x6D;
pub const KC_F23: u8 = 0x6E;
pub const KC_F24: u8 = 0x6F;
pub const KC_INTERNATIONAL_1: u8 = 0x70;
pub const KC_INTERNATIONAL_2: u8 = 0x71;
pub const KC_INTERNATIONAL_3: u8 = 0x72;
pub const KC_INTERNATIONAL_4: u8 = 0x73;
pub const KC_INTERNATIONAL_5: u8 = 0x74;
pub const KC_INTERNATIONAL_6: u8 = 0x75;
pub const KC_LANGUAGE_1: u8 = 0x76;
pub const KC_LANGUAGE_2: u8 = 0x77;
pub const KC_LANGUAGE_3: u8 = 0x78;
pub const KC_LANGUAGE_4: u8 = 0x79;
pub const KC_LEFT_SHIFT: u8 = 0x7C;
pub const KC_LEFT_ALT: u8 = 0x7D;
pub const KC_LEFT_GUI: u8 = 0x7E;
pub const KC_RIGHT_CTRL: u8 = 0x7F;
pub const KC_RIGHT_SHIFT: u8 = 0x80;
pub const KC_RIGHT_ALT: u8 = 0x81;
pub const KC_AUDIO_VOL_UP: u8 = 0x87;
pub const KC_AUDIO_VOL_DOWN: u8 = 0x88;
pub const KC_MEDIA_NEXT_TRACK: u8 = 0x89;
pub const KC_MEDIA_PREV_TRACK: u8 = 0x8A;
pub const KC_MEDIA_STOP: u8 = 0x8B;
pub const KC_MEDIA_PLAY_PAUSE: u8 = 0x8C;
pub const KC_MEDIA_SELECT: u8 = 0x8D;
pub const KC_MAIL: u8 = 0x8E;
pub const KC_CALCULATOR: u8 = 0x8F;
pub const KC_MY_COMPUTER: u8 = 0x90;
pub const KC_WWW_SEARCH: u8 = 0x91;
pub const KC_WWW_HOME: u8 = 0x92;
pub const KC_WWW_BACK: u8 = 0x93;
pub const KC_WWW_FORWARD: u8 = 0x94;
pub const KC_WWW_REFRESH: u8 = 0x95;
pub const KC_WWW_FAVORITES: u8 = 0x96;
pub const KC_BRIGHTNESS_UP: u8 = 0x97;
pub const KC_BRIGHTNESS_DOWN: u8 = 0x98;
pub const KC_MISSION_CONTROL: u8 = 0x99;
pub const SP_MOUSE_BUTTON_2: u8 = 0x9C;
pub const SP_MOUSE_BUTTON_3: u8 = 0x9D;
pub const SP_MOUSE_BUTTON_4: u8 = 0x9E;

// Keyboard keycodes (0x02..=0x7A)
pub const KC_A: u8 = 0x02;
pub const KC_LANGUAGE_5: u8 = 0x7A;

// Modifier keycodes (0x7B..=0x82)
pub const KC_LEFT_CTRL: u8 = 0x7B;
pub const KC_RIGHT_GUI: u8 = 0x82;

// System keycodes (0x83..=0x85)
pub const KC_SYSTEM_POWER: u8 = 0x83;
pub const KC_SYSTEM_SLEEP: u8 = 0x84;
pub const KC_SYSTEM_WAKE: u8 = 0x85;

// Consumer keycodes (0x86..=0x9A)
pub const KC_AUDIO_MUTE: u8 = 0x86;
pub const KC_LAUNCHPAD: u8 = 0x9A;

// Mouse keycodes (0x9B..=0x9F)
pub const SP_MOUSE_BUTTON_1: u8 = 0x9B;
pub const SP_MOUSE_BUTTON_5: u8 = 0x9F;

// Momentary layer keycodes (0xC0..=0xC7)
pub const SP_MO_MIN: u8 = 0xC0;
pub const SP_MO_MAX: u8 = 0xC7;

// Profile keycodes (0xC8..=0xCF)
pub const SP_PF_MIN: u8 = 0xC8;
pub const SP_PF_MAX: u8 = 0xCF;

// Special keycodes
pub const SP_KEY_LOCK: u8 = 0xD0;
pub const SP_LAYER_LOCK: u8 = 0xD1;
pub const SP_PROFILE_SWAP: u8 = 0xD2;
pub const SP_PROFILE_NEXT: u8 = 0xD3;
pub const SP_BOOT: u8 = 0xD4;
pub const SP_BT_TOGGLE: u8 = 0xD5;
pub const SP_BT_USB: u8 = 0xD6;
pub const SP_BT_BLE: u8 = 0xD7;
pub const SP_BT_PAIR: u8 = 0xD8;
/// Select the 2.4 GHz dongle link (libhmk Rust extension; not in the C keycode table).
pub const SP_BT_24G: u8 = 0xD9;

#[inline]
pub const fn mo(layer: u8) -> u8 {
    SP_MO_MIN | (layer & 0x07)
}

#[inline]
pub const fn mo_get_layer(kc: u8) -> u8 {
    kc & 0x07
}

#[inline]
pub const fn pf(profile: u8) -> u8 {
    SP_PF_MIN | (profile & 0x07)
}

#[inline]
pub const fn pf_get_profile(kc: u8) -> u8 {
    kc & 0x07
}

#[inline]
pub const fn is_keyboard_keycode(kc: u8) -> bool {
    kc >= KC_A && kc <= KC_LANGUAGE_5
}

#[inline]
pub const fn is_modifier_keycode(kc: u8) -> bool {
    kc >= KC_LEFT_CTRL && kc <= KC_RIGHT_GUI
}

#[inline]
pub const fn is_system_keycode(kc: u8) -> bool {
    kc >= KC_SYSTEM_POWER && kc <= KC_SYSTEM_WAKE
}

#[inline]
pub const fn is_consumer_keycode(kc: u8) -> bool {
    kc >= KC_AUDIO_MUTE && kc <= KC_LAUNCHPAD
}

#[inline]
pub const fn is_mouse_keycode(kc: u8) -> bool {
    kc >= SP_MOUSE_BUTTON_1 && kc <= SP_MOUSE_BUTTON_5
}

#[inline]
pub const fn is_hid_keycode(kc: u8) -> bool {
    kc >= KC_A && kc <= SP_MOUSE_BUTTON_5
}

#[inline]
pub const fn is_momentary_layer(kc: u8) -> bool {
    kc >= SP_MO_MIN && kc <= SP_MO_MAX
}

#[inline]
pub const fn is_profile_keycode(kc: u8) -> bool {
    kc >= SP_PF_MIN && kc <= SP_PF_MAX
}

/// Map a `libhmk` keycode byte to its USB/BLE HID usage or modifier/mouse bitmask.
pub const fn keycode_to_hid(kc: u8) -> u16 {
    match kc {
        // Keyboard A..=Z (0x04..=0x1D), 1..=0 (0x1E..=0x27), Enter..=F12 (0x28..=0x45),
        // PrintScreen..=Application (0x46..=0x65), F13..=F24 (0x68..=0x73),
        // International 1..=6 (0x87..=0x8C), Language 1..=5 (0x90..=0x94)
        0x02..=0x63 => (kc as u16) + 0x02,
        0x64..=0x6F => (kc as u16) + 0x04, // F13 (0x68) .. F24 (0x73)
        0x70..=0x75 => (kc as u16) + 0x17, // INT1 (0x87) .. INT6 (0x8C)
        0x76..=0x7A => (kc as u16) + 0x1A, // LNG1 (0x90) .. LNG5 (0x94)

        // Modifiers (bitmask 0x01..=0x80)
        0x7B..=0x82 => 1u16 << (kc - 0x7B),

        // System Control
        0x83 => 0x0081, // System Power Down
        0x84 => 0x0082, // System Sleep
        0x85 => 0x0083, // System Wake Up

        // Consumer Control
        0x86 => 0x00E2, // Mute
        0x87 => 0x00E9, // Volume Up
        0x88 => 0x00EA, // Volume Down
        0x89 => 0x00B5, // Next Track
        0x8A => 0x00B6, // Prev Track
        0x8B => 0x00B7, // Stop
        0x8C => 0x00CD, // Play/Pause
        0x8D => 0x0183, // Media Select
        0x8E => 0x018A, // Mail
        0x8F => 0x0192, // Calculator
        0x90 => 0x0194, // My Computer
        0x91 => 0x0221, // WWW Search
        0x92 => 0x0223, // WWW Home
        0x93 => 0x0224, // WWW Back
        0x94 => 0x0225, // WWW Forward
        0x95 => 0x0227, // WWW Refresh
        0x96 => 0x022A, // WWW Favorites
        0x97 => 0x006F, // Brightness Up
        0x98 => 0x0070, // Brightness Down
        0x99 => 0x029F, // Mission Control
        0x9A => 0x02A0, // Launchpad

        // Mouse buttons (bitmask 0x01..=0x10)
        0x9B..=0x9F => 1u16 << (kc - 0x9B),

        _ => 0,
    }
}
