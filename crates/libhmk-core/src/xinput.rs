//! XInput gamepad report processing, analog curves, and circular joystick scaling (`src/xinput.c`).

use crate::matrix::KeyState;
use crate::types::{GamepadButton, GamepadOptions};
use crate::usqrt::{usqrt16, usqrt32};

pub const XINPUT_REPORT_SIZE: usize = 20;

const XINPUT_BUTTON_MASKS: [u16; 16] = [
    0,
    1 << 12, // A
    1 << 13, // B
    1 << 14, // X
    1 << 15, // Y
    1 << 0,  // Up
    1 << 1,  // Down
    1 << 2,  // Left
    1 << 3,  // Right
    1 << 4,  // Start
    1 << 5,  // Back
    1 << 10, // Home
    1 << 6,  // LS
    1 << 7,  // RS
    1 << 8,  // LB
    1 << 9,  // RB
];

const JOYSTICK_AXES: [[usize; 2]; 4] = [
    [GamepadButton::LsLeft as usize, GamepadButton::LsRight as usize],
    [GamepadButton::LsDown as usize, GamepadButton::LsUp as usize],
    [GamepadButton::RsLeft as usize, GamepadButton::RsRight as usize],
    [GamepadButton::RsDown as usize, GamepadButton::RsUp as usize],
];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct XInputReport {
    pub report_id: u8,
    pub report_size: u8,
    pub buttons: u16,
    pub lz: u8,
    pub rz: u8,
    pub joysticks: [i16; 4],
}

impl Default for XInputReport {
    fn default() -> Self {
        Self {
            report_id: 0,
            report_size: XINPUT_REPORT_SIZE as u8,
            buttons: 0,
            lz: 0,
            rz: 0,
            joysticks: [0; 4],
        }
    }
}

impl XInputReport {
    pub fn to_bytes(&self) -> [u8; XINPUT_REPORT_SIZE] {
        let mut out = [0u8; XINPUT_REPORT_SIZE];
        out[0] = self.report_id;
        out[1] = self.report_size;
        out[2..4].copy_from_slice(&self.buttons.to_le_bytes());
        out[4] = self.lz;
        out[5] = self.rz;
        for i in 0..4 {
            out[6 + i * 2..8 + i * 2].copy_from_slice(&self.joysticks[i].to_le_bytes());
        }
        out
    }
}

#[inline]
fn square_to_circular(x: u8, y: u8) -> u8 {
    let y2_half = ((y as u16) * (y as u16)) >> 1;
    let factor = usqrt16(255u16 * 255u16 - y2_half) as u32;
    (((x as u32) * factor) / 255) as u8
}

fn apply_analog_curve(curve: &[[u8; 2]; 4], value: u8) -> (u8, bool) {
    if value > curve[3][0] {
        return (255, true);
    }
    if value <= curve[0][0] {
        return (0, false);
    }
    let mut seg = 0usize;
    while seg < 3 {
        if curve[seg + 1][0] >= value {
            break;
        }
        seg += 1;
    }
    let (x1, y1) = (curve[seg][0] as i32, curve[seg][1] as i32);
    let (x2, y2) = (curve[seg + 1][0] as i32, curve[seg + 1][1] as i32);
    let out = if x2 == x1 {
        y1
    } else {
        y1 + (y2 - y1) * ((value as i32) - x1) / (x2 - x1)
    };
    (out.clamp(0, 255) as u8, false)
}

pub struct XInputState<const NUM_KEYS: usize> {
    key_press_states: [bool; NUM_KEYS],
    analog_states: [u16; 10],
    report: XInputReport,
    last_report: XInputReport,
}

impl<const NUM_KEYS: usize> XInputState<NUM_KEYS> {
    pub const fn new() -> Self {
        Self {
            key_press_states: [false; NUM_KEYS],
            analog_states: [0; 10],
            report: XInputReport {
                report_id: 0,
                report_size: XINPUT_REPORT_SIZE as u8,
                buttons: 0,
                lz: 0,
                rz: 0,
                joysticks: [0; 4],
            },
            last_report: XInputReport {
                report_id: 0,
                report_size: XINPUT_REPORT_SIZE as u8,
                buttons: 0,
                lz: 0,
                rz: 0,
                joysticks: [0; 4],
            },
        }
    }

    pub fn process_key(&mut self, key: usize, state: &KeyState, button: u8) {
        if button == 0 || key >= NUM_KEYS {
            return;
        }
        if (GamepadButton::A as u8..=GamepadButton::Rb as u8).contains(&button) {
            let last = self.key_press_states[key];
            let mask = XINPUT_BUTTON_MASKS[button as usize];
            if state.is_pressed && !last {
                self.report.buttons |= mask;
            } else if !state.is_pressed && last {
                self.report.buttons &= !mask;
            }
            self.key_press_states[key] = state.is_pressed;
        } else if (GamepadButton::LsUp as u8..=GamepadButton::Rt as u8).contains(&button) {
            let idx = (button - GamepadButton::LsUp as u8) as usize;
            self.analog_states[idx] = self.analog_states[idx].max(state.distance as u16);
        }
    }

    #[inline]
    fn analog_axis(&self, button: usize) -> u16 {
        self.analog_states[button - GamepadButton::LsUp as usize]
    }

    /// Compute the next XInput report and return `Some(report)` if changed.
    pub fn task(&mut self, options: GamepadOptions) -> Option<XInputReport> {
        let curve = &options.analog_curve;
        self.report.lz = apply_analog_curve(curve, self.analog_axis(GamepadButton::Lt as usize) as u8).0;
        self.report.rz = apply_analog_curve(curve, self.analog_axis(GamepadButton::Rt as usize) as u8).0;

        let mut joy_states = [0u16; 4];
        for i in 0..4 {
            let neg = self.analog_axis(JOYSTICK_AXES[i][0]);
            let pos = self.analog_axis(JOYSTICK_AXES[i][1]);
            joy_states[i] = if options.snappy_joystick() {
                neg.max(pos)
            } else {
                pos.abs_diff(neg)
            };
        }

        for stick in 0..2 {
            let mut x = joy_states[stick * 2] as u32;
            let mut y = joy_states[stick * 2 + 1] as u32;
            let mag = usqrt32(x * x + y * y);
            if mag == 0 {
                continue;
            }
            let max_x = if x > y { 255 } else { x * 255 / y };
            let max_y = if y > x { 255 } else { y * 255 / x };
            let max_mag = usqrt32(max_x * max_x + max_y * max_y).max(1);
            let scaled_mag = ((mag * 255) / max_mag).min(255) as u8;
            let (new_mag, end_deadzone) = apply_analog_curve(curve, scaled_mag);
            if end_deadzone {
                x = if x == 0 { 0 } else { 255 };
                y = if y == 0 { 0 } else { 255 };
            } else {
                x = max_x * (new_mag as u32) / 255;
                y = max_y * (new_mag as u32) / 255;
            }
            if !options.square_joystick() {
                joy_states[stick * 2] = square_to_circular(x as u8, y as u8) as u16;
                joy_states[stick * 2 + 1] = square_to_circular(y as u8, x as u8) as u16;
            } else {
                joy_states[stick * 2] = x as u16;
                joy_states[stick * 2 + 1] = y as u16;
            }
        }

        for i in 0..4 {
            let neg = self.analog_axis(JOYSTICK_AXES[i][0]);
            let pos = self.analog_axis(JOYSTICK_AXES[i][1]);
            let val = (joy_states[i] << 7) as i16;
            self.report.joysticks[i] = if pos > neg { val } else { -val };
        }

        self.analog_states = [0; 10];

        if self.report != self.last_report {
            self.last_report = self.report;
            Some(self.report)
        } else {
            None
        }
    }
}
