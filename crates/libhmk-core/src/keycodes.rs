//! Keycode definitions and HID usage lookup table, matching `include/keycodes.h` & `src/keycodes.c`.

pub const KC_NO: u8 = 0x00;
pub const KC_TRANSPARENT: u8 = 0x01;

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
