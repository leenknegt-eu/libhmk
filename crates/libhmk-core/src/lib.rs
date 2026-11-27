//! Platform-agnostic `#![no_std]` Hall-effect keyboard firmware core for `libhmk`.
//!
//! Combines Hall-effect matrix scanning (EMA filter, auto-calibration, Rapid Trigger,
//! Continuous Rapid Trigger), advanced keys (Null Bind / SOCD, Dynamic Keystroke,
//! Tap-Hold, Toggle, Macros), wear-leveled flash `eeconfig`, 6KRO+NKRO HID reporting,
//! XInput analog gamepad emulation, and the `hmkconf` Raw HID configurator protocol.

#![cfg_attr(not(test), no_std)]

pub mod advanced_keys;
pub mod commands;
pub mod crc32;
pub mod deferred_actions;
pub mod distance;
pub mod eeconfig;
pub mod hid;
pub mod keycodes;
pub mod keymask;
pub mod layout;
pub mod matrix;
pub mod traits;
pub mod types;
pub mod usqrt;
pub mod wear_leveling;
pub mod xinput;

use commands::CommandProcessor;
use eeconfig::{Eeconfig, EeconfigDefaults, EeconfigLayout};
use hid::{HidState, PendingHidReports, RAW_HID_EP_SIZE};
use layout::LayoutEngine;
use matrix::Matrix;
use traits::{AnalogInput, BleControl, BoardControl, CommandTransport, FlashStorage};
use xinput::{XInputReport, XInputState};

/// Top-level Hall-effect keyboard firmware engine.
pub struct Firmware<
    const NUM_KEYS: usize,
    const NUM_LAYERS: usize,
    const NUM_AK: usize,
    const MAX_BINDINGS: usize,
    const WL_VIRTUAL_SIZE: usize,
    const WL_WRITE_LOG_SIZE: usize,
> {
    pub eeconfig: Eeconfig<WL_VIRTUAL_SIZE, WL_WRITE_LOG_SIZE>,
    pub matrix: Matrix<NUM_KEYS>,
    pub hid: HidState,
    pub xinput: XInputState<NUM_KEYS>,
    pub layout: LayoutEngine<NUM_KEYS, NUM_LAYERS, NUM_AK, MAX_BINDINGS>,
    pub commands: CommandProcessor,
}

/// Output produced by a single [`Firmware::step`] iteration.
#[derive(Debug, Clone, Copy, Default)]
pub struct StepOutput {
    pub hid: PendingHidReports,
    pub xinput: Option<XInputReport>,
}

impl<
        const NUM_KEYS: usize,
        const NUM_LAYERS: usize,
        const NUM_AK: usize,
        const MAX_BINDINGS: usize,
        const WL_VIRTUAL_SIZE: usize,
        const WL_WRITE_LOG_SIZE: usize,
    > Firmware<NUM_KEYS, NUM_LAYERS, NUM_AK, MAX_BINDINGS, WL_VIRTUAL_SIZE, WL_WRITE_LOG_SIZE>
{
    pub const fn new(layout_cfg: EeconfigLayout, adc_resolution: u8, invert_adc: bool) -> Self {
        Self {
            eeconfig: Eeconfig::new(layout_cfg),
            matrix: Matrix::new(adc_resolution, invert_adc),
            hid: HidState::new(),
            xinput: XInputState::new(),
            layout: LayoutEngine::new(),
            commands: CommandProcessor::new(),
        }
    }

    /// Initialize persistent configuration and load the active profile's advanced keys.
    pub fn init<F: FlashStorage>(
        &mut self,
        flash: &mut F,
        defaults: &EeconfigDefaults<'_>,
    ) -> bool {
        if !self.eeconfig.init(flash, defaults) {
            return false;
        }
        self.layout.load_advanced_keys(&self.eeconfig);
        true
    }

    /// Execute one scan + layout + XInput tick.
    pub fn step<A: AnalogInput, F: FlashStorage, B: BoardControl, BL: BleControl>(
        &mut self,
        analog: &A,
        flash: &mut F,
        board: &mut B,
        ble: &mut BL,
    ) -> StepOutput {
        self.matrix.scan(analog, &self.eeconfig);
        let hid = self.layout.task(
            &mut self.eeconfig,
            flash,
            &mut self.matrix,
            &mut self.hid,
            &mut self.xinput,
            board,
            ble,
        );
        let profile = self.eeconfig.current_profile() as usize;
        let xinput = if self.eeconfig.options().xinput_enabled() {
            self.xinput.task(self.eeconfig.gamepad_options(profile))
        } else {
            None
        };
        StepOutput { hid, xinput }
    }

    /// Process a 64-byte `hmkconf` Raw HID command received from USB or BLE.
    pub fn handle_command<F: FlashStorage, B: BoardControl, BL: BleControl>(
        &mut self,
        req: &[u8; RAW_HID_EP_SIZE],
        transport: CommandTransport,
        metadata: &[u8],
        defaults: &EeconfigDefaults<'_>,
        flash: &mut F,
        board: &mut B,
        ble: &mut BL,
    ) -> [u8; RAW_HID_EP_SIZE] {
        self.commands.process(
            req,
            transport,
            metadata,
            defaults,
            &mut self.eeconfig,
            flash,
            &self.matrix,
            &mut self.layout,
            &mut self.hid,
            board,
            ble,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::keycodes::*;
    use crate::traits::NoopBleControl;
    use crate::types::{Actuation, Calibration, FIRMWARE_VERSION};

    struct MemFlash {
        buf: [u8; 65536],
    }

    impl MemFlash {
        fn new() -> Self {
            Self { buf: [0xFF; 65536] }
        }
    }

    impl FlashStorage for MemFlash {
        fn flash_size(&self) -> u32 {
            self.buf.len() as u32
        }
        fn num_sectors(&self) -> u32 {
            16
        }
        fn sector_size(&self, sector: u32) -> u32 {
            if sector < 16 { 4096 } else { 0 }
        }
        fn erase_sector(&mut self, sector: u32) -> bool {
            if sector >= 16 {
                return false;
            }
            let start = (sector as usize) * 4096;
            self.buf[start..start + 4096].fill(0xFF);
            true
        }
        fn read(&self, addr: u32, buf: &mut [u8]) -> bool {
            let start = addr as usize;
            if start + buf.len() > self.buf.len() {
                return false;
            }
            buf.copy_from_slice(&self.buf[start..start + buf.len()]);
            true
        }
        fn write(&mut self, addr: u32, data: &[u8]) -> bool {
            let start = addr as usize;
            if start + data.len() > self.buf.len() {
                return false;
            }
            for (dst, &src) in self.buf[start..start + data.len()].iter_mut().zip(data) {
                *dst &= src;
            }
            true
        }
    }

    struct MockBoard {
        now: u32,
    }

    impl BoardControl for MockBoard {
        fn reset(&mut self) {}
        fn enter_bootloader(&mut self) {}
        fn serial(&self, buf: &mut [u8; 32]) -> usize {
            let s = b"NRF52840TEST0001";
            buf[..s.len()].copy_from_slice(s);
            s.len()
        }
        fn now_ms(&self) -> u32 {
            self.now
        }
    }

    struct MockAnalog {
        samples: [u16; 4],
    }

    impl AnalogInput for MockAnalog {
        fn read_raw(&self, key: usize) -> u16 {
            self.samples[key]
        }
    }

    #[test]
    fn crc32_and_usqrt_and_distance() {
        assert_eq!(usqrt::usqrt16(65025), 255);
        assert_eq!(usqrt::usqrt32(1_000_000), 1000);
        assert_eq!(distance::adc_to_distance(1000, 1000, 2000), 0);
        assert_eq!(distance::adc_to_distance(2000, 1000, 2000), 255);
        assert!(distance::adc_to_distance(1500, 1000, 2000) > 120);
        let c1 = crc32::compute(b"libhmk", 0);
        let c2 = crc32::compute(b"libhmk", 0);
        assert_eq!(c1, c2);
        assert_ne!(c1, 0);
    }

    #[test]
    fn wear_leveling_eeconfig_matrix_and_commands() {
        let mut flash = MemFlash::new();
        let mut board = MockBoard { now: 0 };
        let mut ble = NoopBleControl;
        let layout_cfg = EeconfigLayout {
            num_profiles: 2,
            num_layers: 2,
            num_keys: 4,
            num_advanced_keys: 4,
            num_dks_max_bindings: 4,
            num_macro_nodes: 16,
        };
        let keymaps = [
            KC_A, KC_B, KC_C, KC_D, KC_NO, KC_NO, KC_NO, KC_NO, // profile 0
            KC_A, KC_B, KC_C, KC_D, KC_NO, KC_NO, KC_NO, KC_NO, // profile 1
        ];
        let defaults = EeconfigDefaults {
            calibration: Calibration {
                initial_rest_value: 2000,
                initial_bottom_out_threshold: 800,
            },
            actuation_point: 128,
            default_keymaps: &keymaps,
        };

        let mut fw: Firmware<4, 2, 4, 4, 1024, 4096> = Firmware::new(layout_cfg, 12, false);
        assert!(fw.init(&mut flash, &defaults));

        // Verify Raw HID FIRMWARE_VERSION command
        let mut req = [0u8; RAW_HID_EP_SIZE];
        req[0] = commands::CMD_FIRMWARE_VERSION;
        let resp = fw.handle_command(
            &req,
            CommandTransport::Ble,
            b"meta",
            &defaults,
            &mut flash,
            &mut board,
            &mut ble,
        );
        assert_eq!(resp[0], commands::CMD_FIRMWARE_VERSION);
        assert_eq!(u16::from_le_bytes([resp[1], resp[2]]), FIRMWARE_VERSION);

        // Calibrate at rest = 2000, then press key 0 to 2900 -> KC_A should register
        let mut analog = MockAnalog {
            samples: [2000, 2000, 2000, 2000],
        };
        fw.matrix.begin_recalibrate(&mut fw.eeconfig, &mut flash, false);
        for _ in 0..32 {
            fw.matrix.calibrate_step(&analog, &fw.eeconfig);
        }

        analog.samples[0] = 2900;
        let mut saw_a = false;
        for _ in 0..64 {
            board.now += 1;
            let out = fw.step(&analog, &mut flash, &mut board, &mut ble);
            if let Some(kb) = out.hid.keyboard {
                if kb.keycodes[0] == keycode_to_hid(KC_A) as u8 {
                    saw_a = true;
                    break;
                }
            }
        }
        assert!(saw_a);

        // Verify actuation roundtrip via wear leveling persistence
        let act = Actuation {
            actuation_point: 64,
            rt_down: 15,
            rt_up: 10,
            continuous: true,
        };
        assert_eq!(Actuation::from_bytes(&act.to_bytes()), act);
    }

    // ---- Regression tests for the optimized hot paths ----

    static KEYMAPS_ABCD: [u8; 16] = [
        KC_A, KC_B, KC_C, KC_D, KC_NO, KC_NO, KC_NO, KC_NO, // profile 0
        KC_A, KC_B, KC_C, KC_D, KC_NO, KC_NO, KC_NO, KC_NO, // profile 1
    ];

    fn test_defaults() -> EeconfigDefaults<'static> {
        EeconfigDefaults {
            calibration: Calibration {
                initial_rest_value: 2000,
                initial_bottom_out_threshold: 800,
            },
            actuation_point: 128,
            default_keymaps: &KEYMAPS_ABCD,
        }
    }

    fn test_firmware(flash: &mut MemFlash) -> Firmware<4, 2, 4, 4, 1024, 4096> {
        let layout_cfg = EeconfigLayout {
            num_profiles: 2,
            num_layers: 2,
            num_keys: 4,
            num_advanced_keys: 4,
            num_dks_max_bindings: 4,
            num_macro_nodes: 16,
        };
        let mut fw = Firmware::new(layout_cfg, 12, false);
        assert!(fw.init(flash, &test_defaults()));
        fw
    }

    /// Straight port of C `matrix_scan()` (state per key) used as the reference model.
    #[derive(Clone, Copy, Default, Debug, PartialEq)]
    struct RefKey {
        filtered: u16,
        rest: u16,
        bottom: u16,
        distance: u8,
        extremum: u8,
        dir: u8, // 0 inactive, 1 down, 2 up
        pressed: bool,
    }

    fn ref_scan(k: &mut RefKey, raw: u16, act: Actuation) {
        let filtered = ((raw as u32 + k.filtered as u32 * 15) >> 4) as u16;
        k.filtered = filtered;
        if filtered as u32 >= k.bottom as u32 + 5 {
            k.bottom = filtered;
        }
        k.distance = distance::adc_to_distance(filtered, k.rest, k.bottom);
        let d = k.distance as u32;
        let e = k.extremum as u32;
        if act.rt_down == 0 {
            k.dir = 0;
            k.pressed = k.distance >= act.actuation_point;
            return;
        }
        let reset = if act.continuous { 0 } else { act.actuation_point } as u32;
        let rt_up = if act.rt_up == 0 { act.rt_down } else { act.rt_up } as u32;
        match k.dir {
            0 => {
                if d > act.actuation_point as u32 {
                    k.extremum = k.distance;
                    k.dir = 1;
                    k.pressed = true;
                }
            }
            1 => {
                if d <= reset {
                    k.extremum = k.distance;
                    k.dir = 0;
                    k.pressed = false;
                } else if d + rt_up < e {
                    k.extremum = k.distance;
                    k.dir = 2;
                    k.pressed = false;
                } else if d > e {
                    k.extremum = k.distance;
                }
            }
            _ => {
                if d <= reset {
                    k.extremum = k.distance;
                    k.dir = 0;
                    k.pressed = false;
                } else if e + (act.rt_down as u32) < d {
                    k.extremum = k.distance;
                    k.dir = 1;
                    k.pressed = true;
                } else if d < e {
                    k.extremum = k.distance;
                }
            }
        }
    }

    #[test]
    fn matrix_scan_matches_c_reference() {
        let mut flash = MemFlash::new();
        let mut fw = test_firmware(&mut flash);
        let acts = [
            Actuation { actuation_point: 128, rt_down: 0, rt_up: 0, continuous: false },
            Actuation { actuation_point: 60, rt_down: 20, rt_up: 0, continuous: false },
            Actuation { actuation_point: 40, rt_down: 10, rt_up: 30, continuous: false },
            Actuation { actuation_point: 90, rt_down: 5, rt_up: 5, continuous: true },
        ];
        for (k, &a) in acts.iter().enumerate() {
            assert!(fw.eeconfig.set_actuation(&mut flash, 0, k, a));
        }

        let mut analog = MockAnalog { samples: [2000; 4] };
        fw.matrix.begin_recalibrate(&mut fw.eeconfig, &mut flash, false);
        for _ in 0..64 {
            fw.matrix.calibrate_step(&analog, &fw.eeconfig);
        }
        let mut reference: [RefKey; 4] = core::array::from_fn(|k| {
            let s = fw.matrix.keys[k];
            RefKey {
                filtered: s.adc_filtered,
                rest: s.adc_rest_value,
                bottom: s.adc_bottom_out_value,
                ..Default::default()
            }
        });

        // Deterministic pseudo-random travel (xorshift), including over-travel past bottom-out
        let mut seed = 0x1234_5678u32;
        let mut pos = [0i32; 4];
        let mut presses = 0;
        for _ in 0..20_000 {
            for k in 0..4 {
                seed ^= seed << 13;
                seed ^= seed >> 17;
                seed ^= seed << 5;
                pos[k] = (pos[k] + (seed % 161) as i32 - 80).clamp(-50, 1000);
                analog.samples[k] = (2000 + pos[k]) as u16;
            }
            fw.matrix.scan(&analog, &fw.eeconfig);
            for k in 0..4 {
                ref_scan(&mut reference[k], analog.samples[k], acts[k]);
                let s = fw.matrix.keys[k];
                let got = RefKey {
                    filtered: s.adc_filtered,
                    rest: s.adc_rest_value,
                    bottom: s.adc_bottom_out_value,
                    distance: s.distance,
                    extremum: s.extremum,
                    dir: s.key_dir as u8,
                    pressed: s.is_pressed,
                };
                assert_eq!(got, reference[k], "key {k} diverged from the C reference");
                presses += s.is_pressed as u32;
            }
        }
        assert!(presses > 1000, "test input never actuated keys");
    }

    #[test]
    fn advanced_key_view_matches_owned_decode() {
        let mut bytes = [0u8; types::AdvancedKey::<4>::WIRE_SIZE];
        for (i, b) in bytes.iter_mut().enumerate() {
            *b = (i as u8).wrapping_mul(37).wrapping_add(11);
        }
        bytes[2] = types::AdvancedKeyType::TapHold as u8;
        let owned = types::AdvancedKey::<4>::from_bytes(&bytes);
        let view = types::AdvancedKeyRef::<4>::new(&bytes);
        assert_eq!(view.layer(), owned.layer);
        assert_eq!(view.key(), owned.key);
        assert_eq!(view.kind(), owned.kind());
        assert_eq!(view.null_bind(), owned.null_bind());
        assert_eq!(view.tap_hold(), owned.tap_hold());
        assert_eq!(view.toggle(), owned.toggle());
        assert_eq!(view.dynamic_keystroke(), owned.dynamic_keystroke());
        assert_eq!(view.macro_head(), owned.macro_head());
        assert_eq!(view.to_owned(), owned);
    }

    #[test]
    fn layout_reports_simultaneous_presses_and_releases() {
        let mut flash = MemFlash::new();
        let mut fw = test_firmware(&mut flash);
        let mut board = MockBoard { now: 0 };
        let mut ble = NoopBleControl;
        let mut analog = MockAnalog { samples: [2000; 4] };
        fw.matrix.begin_recalibrate(&mut fw.eeconfig, &mut flash, false);
        for _ in 0..64 {
            fw.matrix.calibrate_step(&analog, &fw.eeconfig);
        }

        let expected: [u8; 4] = [KC_A, KC_B, KC_C, KC_D].map(|kc| keycode_to_hid(kc) as u8);
        let mut last_kb = None;
        analog.samples = [2900; 4];
        for _ in 0..64 {
            board.now += 1;
            if let Some(kb) = fw.step(&analog, &mut flash, &mut board, &mut ble).hid.keyboard {
                last_kb = Some(kb);
            }
        }
        let kb = last_kb.expect("no keyboard report for presses");
        let mut got: [u8; 4] = [kb.keycodes[0], kb.keycodes[1], kb.keycodes[2], kb.keycodes[3]];
        got.sort_unstable();
        let mut want = expected;
        want.sort_unstable();
        assert_eq!(got, want);

        analog.samples = [2000; 4];
        let mut released = false;
        for _ in 0..64 {
            board.now += 1;
            if let Some(kb) = fw.step(&analog, &mut flash, &mut board, &mut ble).hid.keyboard {
                released = kb.keycodes.iter().all(|&k| k == 0);
            }
        }
        assert!(released, "keys were not released");
    }
}
