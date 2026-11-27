//! `hmkconf` Raw HID command protocol handler (wire-compatible with `src/commands.c`),
//! supporting both USB Raw HID and BLE Raw HID transports.

use crate::eeconfig::{Eeconfig, EeconfigDefaults};
use crate::hid::{HidState, RAW_HID_EP_SIZE};
use crate::layout::LayoutEngine;
use crate::matrix::Matrix;
use crate::traits::{BleControl, BoardControl, CommandTransport, FlashStorage};
use crate::types::{
    Actuation, Calibration, EeconfigOptions, GamepadOptions, MacroNode, FIRMWARE_VERSION,
};

pub const CMD_FIRMWARE_VERSION: u8 = 0;
pub const CMD_REBOOT: u8 = 1;
pub const CMD_BOOTLOADER: u8 = 2;
pub const CMD_FACTORY_RESET: u8 = 3;
pub const CMD_RECALIBRATE: u8 = 4;
pub const CMD_ANALOG_INFO: u8 = 5;
pub const CMD_GET_CALIBRATION: u8 = 6;
pub const CMD_SET_CALIBRATION: u8 = 7;
pub const CMD_GET_PROFILE: u8 = 8;
pub const CMD_GET_OPTIONS: u8 = 9;
pub const CMD_SET_OPTIONS: u8 = 10;
pub const CMD_RESET_PROFILE: u8 = 11;
pub const CMD_DUPLICATE_PROFILE: u8 = 12;
pub const CMD_GET_METADATA: u8 = 13;
pub const CMD_GET_SERIAL: u8 = 14;
pub const CMD_SAVE_CALIBRATION_THRESHOLD: u8 = 15;

pub const CMD_GET_KEYMAP: u8 = 128;
pub const CMD_SET_KEYMAP: u8 = 129;
pub const CMD_GET_ACTUATION_MAP: u8 = 130;
pub const CMD_SET_ACTUATION_MAP: u8 = 131;
pub const CMD_GET_ADVANCED_KEYS: u8 = 132;
pub const CMD_SET_ADVANCED_KEYS: u8 = 133;
pub const CMD_GET_TICK_RATE: u8 = 134;
pub const CMD_SET_TICK_RATE: u8 = 135;
pub const CMD_GET_GAMEPAD_BUTTONS: u8 = 136;
pub const CMD_SET_GAMEPAD_BUTTONS: u8 = 137;
pub const CMD_GET_GAMEPAD_OPTIONS: u8 = 138;
pub const CMD_SET_GAMEPAD_OPTIONS: u8 = 139;
pub const CMD_GET_MACROS: u8 = 140;
pub const CMD_SET_MACROS: u8 = 141;
pub const CMD_UNKNOWN: u8 = 255;

const STAGED_SET_BYTES_PER_PACKET: usize = 59;
const STAGED_GET_BYTES_PER_PACKET: usize = 62;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
enum StagedKind {
    #[default]
    None,
    AdvancedKeys,
    Macros,
}

pub struct CommandProcessor {
    staged_kind: StagedKind,
    staged_profile: u8,
    staged_offset: usize,
    staged_buf: [u8; 136],
    pub recalibrate_requested: bool,
}

impl CommandProcessor {
    pub const fn new() -> Self {
        Self {
            staged_kind: StagedKind::None,
            staged_profile: 0,
            staged_offset: 0,
            staged_buf: [0; 136],
            recalibrate_requested: false,
        }
    }

    fn reset_staged(&mut self) {
        self.staged_kind = StagedKind::None;
        self.staged_profile = 0;
        self.staged_offset = 0;
    }

    /// Process a 64-byte `hmkconf` request packet and return the 64-byte response packet.
    #[allow(clippy::too_many_arguments)]
    pub fn process<
        F: FlashStorage,
        B: BoardControl,
        BL: BleControl,
        const NUM_KEYS: usize,
        const NUM_LAYERS: usize,
        const NUM_AK: usize,
        const MAX_BINDINGS: usize,
        const VS: usize,
        const WL: usize,
    >(
        &mut self,
        req: &[u8; RAW_HID_EP_SIZE],
        _transport: CommandTransport,
        metadata: &[u8],
        defaults: &EeconfigDefaults<'_>,
        ee: &mut Eeconfig<VS, WL>,
        flash: &mut F,
        matrix: &Matrix<NUM_KEYS>,
        layout: &mut LayoutEngine<NUM_KEYS, NUM_LAYERS, NUM_AK, MAX_BINDINGS>,
        hid: &mut HidState,
        board: &mut B,
        ble: &mut BL,
    ) -> [u8; RAW_HID_EP_SIZE] {
        ble.notify_activity();
        let mut out = [0u8; RAW_HID_EP_SIZE];
        let cmd_id = req[0];
        let mut ok = true;

        match cmd_id {
            CMD_FIRMWARE_VERSION => {
                out[1..3].copy_from_slice(&FIRMWARE_VERSION.to_le_bytes());
            }
            CMD_REBOOT => {
                board.reset();
            }
            CMD_BOOTLOADER => {
                board.enter_bootloader();
            }
            CMD_FACTORY_RESET => {
                layout.clear_advanced_keys(ee, flash, hid, board, ble);
                ok = ee.reset(flash, defaults);
                layout.load_advanced_keys(ee);
            }
            CMD_RECALIBRATE => {
                self.recalibrate_requested = true;
            }
            CMD_ANALOG_INFO => {
                let offset = req[1] as usize;
                if offset >= NUM_KEYS {
                    ok = false;
                } else {
                    for i in 0..21 {
                        if offset + i >= NUM_KEYS {
                            break;
                        }
                        let k = &matrix.keys[offset + i];
                        let base = 1 + i * 3;
                        out[base..base + 2].copy_from_slice(&k.adc_filtered.to_le_bytes());
                        out[base + 2] = k.distance;
                    }
                }
            }
            CMD_GET_CALIBRATION => {
                out[1..5].copy_from_slice(&ee.calibration().to_bytes());
            }
            CMD_SET_CALIBRATION => {
                ok = ee.set_calibration(flash, Calibration::from_bytes(&req[1..5]));
            }
            CMD_GET_PROFILE => {
                out[1] = ee.current_profile();
            }
            CMD_GET_OPTIONS => {
                out[1..3].copy_from_slice(&ee.options().to_bytes());
            }
            CMD_SET_OPTIONS => {
                ok = ee.set_options(flash, EeconfigOptions::from_bytes(&req[1..3]));
            }
            CMD_RESET_PROFILE => {
                let profile = req[1] as usize;
                if profile >= ee.layout.num_profiles {
                    ok = false;
                } else {
                    if profile == ee.current_profile() as usize {
                        layout.clear_advanced_keys(ee, flash, hid, board, ble);
                    }
                    ok = ee.reset_profile(flash, profile, defaults);
                    if profile == ee.current_profile() as usize {
                        layout.load_advanced_keys(ee);
                    }
                }
            }
            CMD_DUPLICATE_PROFILE => {
                let dst = req[1] as usize;
                let src = req[2] as usize;
                if dst >= ee.layout.num_profiles || src >= ee.layout.num_profiles {
                    ok = false;
                } else {
                    if dst == ee.current_profile() as usize {
                        layout.clear_advanced_keys(ee, flash, hid, board, ble);
                    }
                    ok = ee.duplicate_profile(flash, dst, src);
                    if dst == ee.current_profile() as usize {
                        layout.load_advanced_keys(ee);
                    }
                }
            }
            CMD_GET_METADATA => {
                let offset = u32::from_le_bytes([req[1], req[2], req[3], req[4]]) as usize;
                if offset >= metadata.len() {
                    ok = false;
                } else {
                    let rem = metadata.len() - offset;
                    out[1..5].copy_from_slice(&(rem as u32).to_le_bytes());
                    let n = rem.min(59);
                    out[5..5 + n].copy_from_slice(&metadata[offset..offset + n]);
                }
            }
            CMD_GET_SERIAL => {
                let mut serial = [0u8; 32];
                let n = board.serial(&mut serial).min(32);
                out[1..1 + n].copy_from_slice(&serial[..n]);
            }
            CMD_SAVE_CALIBRATION_THRESHOLD => {
                for i in 0..NUM_KEYS {
                    let k = &matrix.keys[i];
                    let thresh = k.adc_bottom_out_value.saturating_sub(k.adc_rest_value);
                    ok &= ee.set_bottom_out_threshold(flash, i, thresh);
                }
            }
            CMD_GET_KEYMAP => {
                let (profile, layer, offset) = (req[1] as usize, req[2] as usize, req[3] as usize);
                if profile >= ee.layout.num_profiles || layer >= NUM_LAYERS || offset >= NUM_KEYS {
                    ok = false;
                } else {
                    let n = (NUM_KEYS - offset).min(63);
                    let src = ee.layout.profile_base(profile)
                        + ee.layout.profile_offset_keymap()
                        + layer * NUM_KEYS
                        + offset;
                    out[1..1 + n].copy_from_slice(&ee.wl.cache[src..src + n]);
                }
            }
            CMD_SET_KEYMAP => {
                let (profile, layer, offset, len) = (
                    req[1] as usize,
                    req[2] as usize,
                    req[3] as usize,
                    req[4] as usize,
                );
                if profile >= ee.layout.num_profiles
                    || layer >= NUM_LAYERS
                    || offset >= NUM_KEYS
                    || len > 59
                    || offset + len > NUM_KEYS
                {
                    ok = false;
                } else {
                    let dst = ee.layout.profile_base(profile)
                        + ee.layout.profile_offset_keymap()
                        + layer * NUM_KEYS
                        + offset;
                    ok = ee.wl.write(flash, dst, &req[5..5 + len]);
                }
            }
            CMD_GET_ACTUATION_MAP => {
                let (profile, offset) = (req[1] as usize, req[2] as usize);
                if profile >= ee.layout.num_profiles || offset >= NUM_KEYS {
                    ok = false;
                } else {
                    let n = (NUM_KEYS - offset).min(15) * Actuation::SIZE;
                    let src = ee.layout.profile_base(profile)
                        + ee.layout.profile_offset_actuation_map()
                        + offset * Actuation::SIZE;
                    out[1..1 + n].copy_from_slice(&ee.wl.cache[src..src + n]);
                }
            }
            CMD_SET_ACTUATION_MAP => {
                let (profile, offset, len) = (req[1] as usize, req[2] as usize, req[3] as usize);
                if profile >= ee.layout.num_profiles
                    || offset >= NUM_KEYS
                    || len > 15
                    || offset + len > NUM_KEYS
                {
                    ok = false;
                } else {
                    let dst = ee.layout.profile_base(profile)
                        + ee.layout.profile_offset_actuation_map()
                        + offset * Actuation::SIZE;
                    let bytes = len * Actuation::SIZE;
                    ok = ee.wl.write(flash, dst, &req[4..4 + bytes]);
                }
            }
            CMD_GET_ADVANCED_KEYS => {
                let profile = req[1] as usize;
                let offset = u16::from_le_bytes([req[2], req[3]]) as usize;
                let field_size = ee.layout.num_advanced_keys * ee.layout.advanced_key_size();
                if profile >= ee.layout.num_profiles || offset >= field_size {
                    ok = false;
                } else {
                    let n = (field_size - offset).min(STAGED_GET_BYTES_PER_PACKET);
                    out[1] = n as u8;
                    let src = ee.layout.profile_base(profile)
                        + ee.layout.profile_offset_advanced_keys()
                        + offset;
                    out[2..2 + n].copy_from_slice(&ee.wl.cache[src..src + n]);
                }
            }
            CMD_SET_ADVANCED_KEYS => {
                let profile = req[1];
                let offset = u16::from_le_bytes([req[2], req[3]]) as usize;
                let len = req[4] as usize;
                let item_size = ee.layout.advanced_key_size();
                let field_size = ee.layout.num_advanced_keys * item_size;
                ok = self.stage_write(
                    StagedKind::AdvancedKeys,
                    profile,
                    offset,
                    len,
                    &req[5..],
                    field_size,
                    item_size,
                    ee,
                    flash,
                    layout,
                    hid,
                    board,
                    ble,
                );
            }
            CMD_GET_MACROS => {
                let profile = req[1] as usize;
                let offset = u16::from_le_bytes([req[2], req[3]]) as usize;
                let field_size = ee.layout.num_macro_nodes * MacroNode::SIZE;
                if profile >= ee.layout.num_profiles || offset >= field_size {
                    ok = false;
                } else {
                    let n = (field_size - offset).min(STAGED_GET_BYTES_PER_PACKET);
                    out[1] = n as u8;
                    let src = ee.layout.profile_base(profile)
                        + ee.layout.profile_offset_macros()
                        + offset;
                    out[2..2 + n].copy_from_slice(&ee.wl.cache[src..src + n]);
                }
            }
            CMD_SET_MACROS => {
                let profile = req[1];
                let offset = u16::from_le_bytes([req[2], req[3]]) as usize;
                let len = req[4] as usize;
                let item_size = MacroNode::SIZE;
                let field_size = ee.layout.num_macro_nodes * item_size;
                ok = self.stage_write(
                    StagedKind::Macros,
                    profile,
                    offset,
                    len,
                    &req[5..],
                    field_size,
                    item_size,
                    ee,
                    flash,
                    layout,
                    hid,
                    board,
                    ble,
                );
            }
            CMD_GET_TICK_RATE => {
                let profile = req[1] as usize;
                if profile >= ee.layout.num_profiles {
                    ok = false;
                } else {
                    out[1] = ee.tick_rate(profile);
                }
            }
            CMD_SET_TICK_RATE => {
                let profile = req[1] as usize;
                if profile >= ee.layout.num_profiles {
                    ok = false;
                } else {
                    let dst = ee.layout.profile_base(profile) + ee.layout.profile_offset_tick_rate();
                    ok = ee.wl.write(flash, dst, &[req[2]]);
                }
            }
            CMD_GET_GAMEPAD_BUTTONS => {
                let (profile, offset) = (req[1] as usize, req[2] as usize);
                if profile >= ee.layout.num_profiles || offset >= NUM_KEYS {
                    ok = false;
                } else {
                    let n = (NUM_KEYS - offset).min(63);
                    let src = ee.layout.profile_base(profile)
                        + ee.layout.profile_offset_gamepad_buttons()
                        + offset;
                    out[1..1 + n].copy_from_slice(&ee.wl.cache[src..src + n]);
                }
            }
            CMD_SET_GAMEPAD_BUTTONS => {
                let (profile, offset, len) = (req[1] as usize, req[2] as usize, req[3] as usize);
                if profile >= ee.layout.num_profiles
                    || offset >= NUM_KEYS
                    || len > 60
                    || offset + len > NUM_KEYS
                {
                    ok = false;
                } else {
                    let dst = ee.layout.profile_base(profile)
                        + ee.layout.profile_offset_gamepad_buttons()
                        + offset;
                    ok = ee.wl.write(flash, dst, &req[4..4 + len]);
                }
            }
            CMD_GET_GAMEPAD_OPTIONS => {
                let profile = req[1] as usize;
                if profile >= ee.layout.num_profiles {
                    ok = false;
                } else {
                    out[1..1 + GamepadOptions::SIZE]
                        .copy_from_slice(&ee.gamepad_options(profile).to_bytes());
                }
            }
            CMD_SET_GAMEPAD_OPTIONS => {
                let profile = req[1] as usize;
                if profile >= ee.layout.num_profiles {
                    ok = false;
                } else {
                    let dst = ee.layout.profile_base(profile)
                        + ee.layout.profile_offset_gamepad_options();
                    ok = ee.wl.write(flash, dst, &req[2..2 + GamepadOptions::SIZE]);
                }
            }
            _ => {
                ok = false;
            }
        }

        out[0] = if ok { cmd_id } else { CMD_UNKNOWN };
        out
    }

    #[allow(clippy::too_many_arguments)]
    fn stage_write<
        F: FlashStorage,
        B: BoardControl,
        BL: BleControl,
        const NUM_KEYS: usize,
        const NUM_LAYERS: usize,
        const NUM_AK: usize,
        const MAX_BINDINGS: usize,
        const VS: usize,
        const WL: usize,
    >(
        &mut self,
        kind: StagedKind,
        profile: u8,
        offset: usize,
        len: usize,
        data: &[u8],
        field_size: usize,
        item_size: usize,
        ee: &mut Eeconfig<VS, WL>,
        flash: &mut F,
        layout: &mut LayoutEngine<NUM_KEYS, NUM_LAYERS, NUM_AK, MAX_BINDINGS>,
        hid: &mut HidState,
        board: &mut B,
        ble: &mut BL,
    ) -> bool {
        if (profile as usize) >= ee.layout.num_profiles
            || offset + len > field_size
            || len > STAGED_SET_BYTES_PER_PACKET
            || len == 0
            || item_size > self.staged_buf.len()
        {
            self.reset_staged();
            return false;
        }

        if offset % item_size == 0 {
            self.staged_kind = kind;
            self.staged_profile = profile;
            self.staged_offset = offset;
        }

        if self.staged_kind != kind
            || self.staged_offset != offset
            || self.staged_profile != profile
        {
            self.reset_staged();
            return false;
        }

        let mut i = 0usize;
        while i < len {
            let cur_off = self.staged_offset % item_size;
            let write_len = (len - i).min(item_size - cur_off);
            self.staged_buf[cur_off..cur_off + write_len]
                .copy_from_slice(&data[i..i + write_len]);

            if len - i >= item_size - cur_off {
                let p = self.staged_profile as usize;
                let item_idx = self.staged_offset / item_size;
                let dst = match kind {
                    StagedKind::AdvancedKeys => {
                        ee.layout.profile_base(p)
                            + ee.layout.profile_offset_advanced_keys()
                            + item_idx * item_size
                    }
                    StagedKind::Macros => {
                        ee.layout.profile_base(p)
                            + ee.layout.profile_offset_macros()
                            + item_idx * item_size
                    }
                    StagedKind::None => {
                        self.reset_staged();
                        return false;
                    }
                };
                if p == ee.current_profile() as usize {
                    layout.clear_advanced_keys(ee, flash, hid, board, ble);
                }
                let ok = ee.wl.write(flash, dst, &self.staged_buf[..item_size]);
                if p == ee.current_profile() as usize {
                    layout.load_advanced_keys(ee);
                }
                if !ok {
                    self.reset_staged();
                    return false;
                }
            }

            self.staged_offset += write_len;
            i += write_len;
        }

        true
    }
}
