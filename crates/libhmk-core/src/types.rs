//! Binary-compatible configuration & protocol data types (matching C `include/common.h` and `eeconfig.h`).

pub const FIRMWARE_VERSION: u16 = 0x0108;
pub const MACRO_NODE_NONE: u8 = u8::MAX;

/// Per-key Hall-effect actuation and Rapid Trigger settings (4 bytes packed).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Actuation {
    pub actuation_point: u8,
    pub rt_down: u8,
    pub rt_up: u8,
    pub continuous: bool,
}

impl Actuation {
    pub const SIZE: usize = 4;

    pub const fn default_with_point(actuation_point: u8) -> Self {
        Self {
            actuation_point,
            rt_down: 0,
            rt_up: 0,
            continuous: false,
        }
    }

    pub fn from_bytes(b: &[u8]) -> Self {
        Self {
            actuation_point: b[0],
            rt_down: b[1],
            rt_up: b[2],
            continuous: b[3] != 0,
        }
    }

    pub fn to_bytes(self) -> [u8; Self::SIZE] {
        [
            self.actuation_point,
            self.rt_down,
            self.rt_up,
            u8::from(self.continuous),
        ]
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum AdvancedKeyType {
    None = 0,
    NullBind = 1,
    DynamicKeystroke = 2,
    TapHold = 3,
    Toggle = 4,
    Macro = 5,
}

impl From<u8> for AdvancedKeyType {
    fn from(v: u8) -> Self {
        match v {
            1 => Self::NullBind,
            2 => Self::DynamicKeystroke,
            3 => Self::TapHold,
            4 => Self::Toggle,
            5 => Self::Macro,
            _ => Self::None,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum NullBindBehavior {
    Last = 0,
    Primary = 1,
    Secondary = 2,
    Neutral = 3,
    Distance = 4,
}

impl From<u8> for NullBindBehavior {
    fn from(v: u8) -> Self {
        match v {
            1 => Self::Primary,
            2 => Self::Secondary,
            3 => Self::Neutral,
            4 => Self::Distance,
            _ => Self::Last,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct NullBind {
    pub secondary_key: u8,
    pub behavior: u8,
    pub bottom_out_point: u8,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum DksAction {
    Hold = 0,
    Press = 1,
    Release = 2,
    Tap = 3,
}

impl From<u8> for DksAction {
    fn from(v: u8) -> Self {
        match v & 0x03 {
            1 => Self::Press,
            2 => Self::Release,
            3 => Self::Tap,
            _ => Self::Hold,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DynamicKeystroke<const MAX_BINDINGS: usize> {
    pub keycodes: [u8; MAX_BINDINGS],
    pub bitmap: [u8; MAX_BINDINGS],
    pub bottom_out_point: u8,
}

impl<const MAX_BINDINGS: usize> Default for DynamicKeystroke<MAX_BINDINGS> {
    fn default() -> Self {
        Self {
            keycodes: [0; MAX_BINDINGS],
            bitmap: [0; MAX_BINDINGS],
            bottom_out_point: 0,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct TapHold {
    pub tap_keycode: u8,
    pub hold_keycode: u8,
    pub tapping_term: u16,
    pub hold_on_other_key_press: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Toggle {
    pub keycode: u8,
    pub tapping_term: u16,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum MacroAction {
    None = 0,
    Press = 1,
    Tap = 2,
    Release = 3,
}

impl From<u8> for MacroAction {
    fn from(v: u8) -> Self {
        match v & 0x07 {
            1 => Self::Press,
            2 => Self::Tap,
            3 => Self::Release,
            _ => Self::None,
        }
    }
}

/// Packed 4-byte macro linked-list node matching `macro_node_t`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MacroNode {
    pub keycode: u8,
    pub action: u8,
    pub delay: u16,
    pub next: u8,
}

impl MacroNode {
    pub const SIZE: usize = 4;

    pub const fn empty() -> Self {
        Self {
            keycode: 0,
            action: MacroAction::None as u8,
            delay: 0,
            next: MACRO_NODE_NONE,
        }
    }

    pub fn from_bytes(b: &[u8]) -> Self {
        let packed = u16::from_le_bytes([b[1], b[2]]);
        Self {
            keycode: b[0],
            action: (packed & 0x07) as u8,
            delay: (packed >> 3) & 0x1FFF,
            next: b[3],
        }
    }

    pub fn to_bytes(self) -> [u8; Self::SIZE] {
        let packed = ((self.action as u16) & 0x07) | ((self.delay & 0x1FFF) << 3);
        let [lo, hi] = packed.to_le_bytes();
        [self.keycode, lo, hi, self.next]
    }
}

impl Default for MacroNode {
    fn default() -> Self {
        Self::empty()
    }
}

/// Advanced key configuration record matching C `advanced_key_t` wire layout:
/// `[layer: u8, key: u8, type: u8, payload: [u8; 2 * MAX_BINDINGS + 1]]`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AdvancedKey<const MAX_BINDINGS: usize> {
    pub layer: u8,
    pub key: u8,
    pub kind: u8,
    pub payload: [u8; 129], // supports MAX_BINDINGS up to 64 (2*64 + 1 = 129)
}

impl<const MAX_BINDINGS: usize> AdvancedKey<MAX_BINDINGS> {
    pub const PAYLOAD_SIZE: usize = 2 * MAX_BINDINGS + 1;
    pub const WIRE_SIZE: usize = 3 + Self::PAYLOAD_SIZE;

    pub const fn empty() -> Self {
        Self {
            layer: 0,
            key: 0,
            kind: AdvancedKeyType::None as u8,
            payload: [0; 129],
        }
    }

    pub fn from_bytes(b: &[u8]) -> Self {
        let mut ak = Self::empty();
        ak.layer = b[0];
        ak.key = b[1];
        ak.kind = b[2];
        ak.payload[..Self::PAYLOAD_SIZE].copy_from_slice(&b[3..3 + Self::PAYLOAD_SIZE]);
        ak
    }

    pub fn write_bytes(&self, dst: &mut [u8]) {
        dst[0] = self.layer;
        dst[1] = self.key;
        dst[2] = self.kind;
        dst[3..3 + Self::PAYLOAD_SIZE].copy_from_slice(&self.payload[..Self::PAYLOAD_SIZE]);
    }

    pub fn kind(&self) -> AdvancedKeyType {
        AdvancedKeyType::from(self.kind)
    }

    pub fn null_bind(&self) -> NullBind {
        NullBind {
            secondary_key: self.payload[0],
            behavior: self.payload[1],
            bottom_out_point: self.payload[2],
        }
    }

    pub fn dynamic_keystroke(&self) -> DynamicKeystroke<MAX_BINDINGS> {
        let mut dks = DynamicKeystroke::default();
        dks.keycodes.copy_from_slice(&self.payload[..MAX_BINDINGS]);
        dks.bitmap
            .copy_from_slice(&self.payload[MAX_BINDINGS..2 * MAX_BINDINGS]);
        dks.bottom_out_point = self.payload[2 * MAX_BINDINGS];
        dks
    }

    pub fn tap_hold(&self) -> TapHold {
        TapHold {
            tap_keycode: self.payload[0],
            hold_keycode: self.payload[1],
            tapping_term: u16::from_le_bytes([self.payload[2], self.payload[3]]),
            hold_on_other_key_press: self.payload[4] != 0,
        }
    }

    pub fn toggle(&self) -> Toggle {
        Toggle {
            keycode: self.payload[0],
            tapping_term: u16::from_le_bytes([self.payload[1], self.payload[2]]),
        }
    }

    pub fn macro_head(&self) -> u8 {
        self.payload[0]
    }
}

/// Gamepad button codes matching `gamepad_button_t`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum GamepadButton {
    None = 0,
    A = 1,
    B = 2,
    X = 3,
    Y = 4,
    Up = 5,
    Down = 6,
    Left = 7,
    Right = 8,
    Start = 9,
    Back = 10,
    Home = 11,
    Ls = 12,
    Rs = 13,
    Lb = 14,
    Rb = 15,
    LsUp = 16,
    LsDown = 17,
    LsLeft = 18,
    LsRight = 19,
    RsUp = 20,
    RsDown = 21,
    RsLeft = 22,
    RsRight = 23,
    Lt = 24,
    Rt = 25,
}

/// Packed 9-byte gamepad options matching `gamepad_options_t`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GamepadOptions {
    pub analog_curve: [[u8; 2]; 4],
    pub options: u8,
}

impl GamepadOptions {
    pub const SIZE: usize = 9;

    pub const fn default_options() -> Self {
        Self {
            analog_curve: [[4, 20], [85, 95], [165, 170], [255, 255]],
            // keyboard_enabled (bit 0) | snappy_joystick (bit 3)
            options: 0b0000_1001,
        }
    }

    pub fn keyboard_enabled(self) -> bool {
        (self.options & (1 << 0)) != 0
    }

    pub fn gamepad_override(self) -> bool {
        (self.options & (1 << 1)) != 0
    }

    pub fn square_joystick(self) -> bool {
        (self.options & (1 << 2)) != 0
    }

    pub fn snappy_joystick(self) -> bool {
        (self.options & (1 << 3)) != 0
    }

    pub fn from_bytes(b: &[u8]) -> Self {
        Self {
            analog_curve: [[b[0], b[1]], [b[2], b[3]], [b[4], b[5]], [b[6], b[7]]],
            options: b[8],
        }
    }

    pub fn to_bytes(self) -> [u8; Self::SIZE] {
        [
            self.analog_curve[0][0],
            self.analog_curve[0][1],
            self.analog_curve[1][0],
            self.analog_curve[1][1],
            self.analog_curve[2][0],
            self.analog_curve[2][1],
            self.analog_curve[3][0],
            self.analog_curve[3][1],
            self.options,
        ]
    }
}

/// Packed 4-byte calibration config matching `eeconfig_calibration_t`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Calibration {
    pub initial_rest_value: u16,
    pub initial_bottom_out_threshold: u16,
}

impl Calibration {
    pub const SIZE: usize = 4;

    pub fn from_bytes(b: &[u8]) -> Self {
        Self {
            initial_rest_value: u16::from_le_bytes([b[0], b[1]]),
            initial_bottom_out_threshold: u16::from_le_bytes([b[2], b[3]]),
        }
    }

    pub fn to_bytes(self) -> [u8; Self::SIZE] {
        let r = self.initial_rest_value.to_le_bytes();
        let t = self.initial_bottom_out_threshold.to_le_bytes();
        [r[0], r[1], t[0], t[1]]
    }
}

/// Packed 2-byte global options bitfield matching `eeconfig_options_t`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EeconfigOptions {
    pub raw: u16,
}

impl EeconfigOptions {
    pub const SIZE: usize = 2;

    pub const fn default_options() -> Self {
        // high_polling_rate_enabled = true (bit 2)
        Self { raw: 1 << 2 }
    }

    pub const fn xinput_enabled(self) -> bool {
        (self.raw & (1 << 0)) != 0
    }

    pub const fn high_polling_rate_enabled(self) -> bool {
        (self.raw & (1 << 2)) != 0
    }

    pub fn from_bytes(b: &[u8]) -> Self {
        Self {
            raw: u16::from_le_bytes([b[0], b[1]]),
        }
    }

    pub fn to_bytes(self) -> [u8; Self::SIZE] {
        self.raw.to_le_bytes()
    }
}
