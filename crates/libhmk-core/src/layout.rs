//! Layer management, keycode dispatch, profile switching, and BT keycodes (`src/layout.c`).

use crate::advanced_keys::{ActionEmitter, AdvancedKeyEngine, AkEvent, AkEventType, KeyAction};
use crate::deferred_actions::{
    DeferredAction, DeferredActionQueue, DeferredActionType, MAX_DEFERRED_ACTIONS,
};
use crate::eeconfig::Eeconfig;
use crate::hid::{HidState, PendingHidReports};
use crate::keycodes::*;
use crate::keymask::KeyMask;
use crate::matrix::Matrix;
use crate::traits::{BleControl, BleTransportMode, BoardControl, FlashStorage, WirelessBackend};
use crate::types::{AdvancedKeyType, GamepadButton};
use crate::xinput::XInputState;

pub struct LayoutEngine<
    const NUM_KEYS: usize,
    const NUM_LAYERS: usize,
    const NUM_AK: usize,
    const MAX_BINDINGS: usize,
> {
    layer_mask: u16,
    default_layer: u8,
    should_send_reports: bool,
    last_ak_tick: u32,
    key_disabled: KeyMask,
    key_press_states: KeyMask,
    /// Keys with an active advanced key (`active_advanced_keys[i] != 0`).
    ak_active: KeyMask,
    active_keycodes: [u8; NUM_KEYS],
    advanced_key_indices: [[u8; NUM_KEYS]; NUM_LAYERS],
    active_advanced_keys: [u8; NUM_KEYS],
    pub advanced_keys: AdvancedKeyEngine<NUM_AK, MAX_BINDINGS>,
    pub deferred: DeferredActionQueue,
}

impl<
        const NUM_KEYS: usize,
        const NUM_LAYERS: usize,
        const NUM_AK: usize,
        const MAX_BINDINGS: usize,
    > LayoutEngine<NUM_KEYS, NUM_LAYERS, NUM_AK, MAX_BINDINGS>
{
    pub const fn new() -> Self {
        Self {
            layer_mask: 0,
            default_layer: 0,
            should_send_reports: false,
            last_ak_tick: 0,
            key_disabled: KeyMask::new(),
            key_press_states: KeyMask::new(),
            ak_active: KeyMask::new(),
            active_keycodes: [0; NUM_KEYS],
            advanced_key_indices: [[0; NUM_KEYS]; NUM_LAYERS],
            active_advanced_keys: [0; NUM_KEYS],
            advanced_keys: AdvancedKeyEngine::new(),
            deferred: DeferredActionQueue::new(),
        }
    }

    pub fn load_advanced_keys<const VS: usize, const WL: usize>(&mut self, ee: &Eeconfig<VS, WL>) {
        self.advanced_keys.reset_states(ee);
        self.advanced_key_indices = [[0; NUM_KEYS]; NUM_LAYERS];
        let profile = ee.current_profile() as usize;
        for i in 0..NUM_AK {
            let ak = ee.advanced_key::<MAX_BINDINGS>(profile, i);
            if ak.kind() == AdvancedKeyType::None
                || (ak.layer() as usize) >= NUM_LAYERS
                || (ak.key() as usize) >= NUM_KEYS
            {
                continue;
            }
            self.advanced_key_indices[ak.layer() as usize][ak.key() as usize] = (i + 1) as u8;
            if ak.kind() == AdvancedKeyType::NullBind {
                let sec = ak.null_bind().secondary_key as usize;
                if sec < NUM_KEYS {
                    self.advanced_key_indices[ak.layer() as usize][sec] = (i + 1) as u8;
                }
            }
        }
    }

    pub fn clear_advanced_keys<const VS: usize, const WL: usize>(
        &mut self,
        ee: &mut Eeconfig<VS, WL>,
        flash: &mut impl FlashStorage,
        hid: &mut HidState,
        board: &mut impl BoardControl,
        ble: &mut impl BleControl,
    ) {
        let mut emit = ActionEmitter::new();
        self.advanced_keys.clear(ee, &mut emit);
        self.apply_actions(&emit, ee, flash, hid, board, ble);
    }

    #[inline]
    fn current_layer(&self) -> u8 {
        if self.layer_mask != 0 {
            (15 - self.layer_mask.leading_zeros()) as u8
        } else {
            self.default_layer
        }
    }

    fn resolve_keycode<const VS: usize, const WL: usize>(
        &self,
        ee: &Eeconfig<VS, WL>,
        current_layer: u8,
        key: usize,
    ) -> u8 {
        let profile = ee.current_profile() as usize;
        for layer in (0..=current_layer as usize).rev() {
            if ((self.layer_mask >> layer) & 1) == 0 {
                continue;
            }
            let kc = ee.keymap_keycode(profile, layer, key);
            if kc != KC_TRANSPARENT {
                return kc;
            }
        }
        ee.keymap_keycode(profile, self.default_layer as usize, key)
    }

    /// Press / release / hold handling for one key (kept out of the per-key scan loop so
    /// the common no-change path stays small). Returns whether a non-tap-hold key was pressed.
    #[inline(never)]
    #[allow(clippy::too_many_arguments)]
    fn key_event<
        F: FlashStorage,
        B: BoardControl,
        BL: BleControl,
        const VS: usize,
        const WL: usize,
    >(
        &mut self,
        i: usize,
        is_pressed: bool,
        last_pressed: bool,
        now_ms: u32,
        current_layer: u8,
        ee: &mut Eeconfig<VS, WL>,
        flash: &mut F,
        matrix: &mut Matrix<NUM_KEYS>,
        hid: &mut HidState,
        board: &mut B,
        ble: &mut BL,
    ) -> bool {
        let mut has_non_tap_hold_press = false;
        if is_pressed && !last_pressed {
            ble.notify_activity();
            let keycode = self.resolve_keycode(ee, current_layer, i);
            let ak_index = self.advanced_key_indices[current_layer as usize][i];
            if ak_index != 0 {
                self.active_advanced_keys[i] = ak_index;
                self.ak_active.set(i, true);
                let idx = (ak_index - 1) as usize;
                let mut emit = ActionEmitter::new();
                self.advanced_keys.process(
                    AkEvent {
                        kind: AkEventType::Press,
                        key: i as u8,
                        keycode,
                        ak_index: idx,
                    },
                    now_ms,
                    ee,
                    matrix,
                    &mut self.deferred,
                    &mut emit,
                );
                self.apply_actions(&emit, ee, flash, hid, board, ble);
                let ak_kind = ee
                    .advanced_key::<MAX_BINDINGS>(ee.current_profile() as usize, idx)
                    .kind();
                if ak_kind != AdvancedKeyType::TapHold {
                    has_non_tap_hold_press = true;
                }
            } else {
                self.active_keycodes[i] = keycode;
                self.register(i as u8, keycode, ee, flash, hid, board, ble);
                if keycode != KC_NO {
                    has_non_tap_hold_press = true;
                }
            }
        } else if !is_pressed && last_pressed {
            ble.notify_activity();
            let keycode = self.active_keycodes[i];
            let ak_index = self.active_advanced_keys[i];
            if ak_index != 0 {
                self.active_advanced_keys[i] = 0;
                self.ak_active.set(i, false);
                let mut emit = ActionEmitter::new();
                self.advanced_keys.process(
                    AkEvent {
                        kind: AkEventType::Release,
                        key: i as u8,
                        keycode,
                        ak_index: (ak_index - 1) as usize,
                    },
                    now_ms,
                    ee,
                    matrix,
                    &mut self.deferred,
                    &mut emit,
                );
                self.apply_actions(&emit, ee, flash, hid, board, ble);
            } else {
                self.active_keycodes[i] = KC_NO;
                self.unregister(keycode, hid);
            }
        } else if is_pressed {
            let ak_index = self.active_advanced_keys[i];
            if ak_index != 0 {
                let mut emit = ActionEmitter::new();
                self.advanced_keys.process(
                    AkEvent {
                        kind: AkEventType::Hold,
                        key: i as u8,
                        keycode: self.active_keycodes[i],
                        ak_index: (ak_index - 1) as usize,
                    },
                    now_ms,
                    ee,
                    matrix,
                    &mut self.deferred,
                    &mut emit,
                );
                self.apply_actions(&emit, ee, flash, hid, board, ble);
            }
        }
        has_non_tap_hold_press
    }

    /// Run one layout scan iteration and return any changed HID reports to transmit.
    #[allow(clippy::too_many_arguments)]
    pub fn task<
        F: FlashStorage,
        B: BoardControl,
        BL: BleControl,
        const VS: usize,
        const WL: usize,
    >(
        &mut self,
        ee: &mut Eeconfig<VS, WL>,
        flash: &mut F,
        matrix: &mut Matrix<NUM_KEYS>,
        hid: &mut HidState,
        xinput: &mut XInputState<NUM_KEYS>,
        board: &mut B,
        ble: &mut BL,
    ) -> PendingHidReports {
        let now_ms = board.now_ms();
        let current_layer = self.current_layer();
        let profile = ee.current_profile() as usize;
        let xinput_enabled = ee.options().xinput_enabled();
        let gp_opts = ee.gamepad_options(profile);
        let mut has_non_tap_hold_press = false;

        // Pass 1 (hot, call-free except XInput): find keys with a press/release edge or an
        // active advanced key that needs hold events.
        let gamepad_buttons = if current_layer == 0 && xinput_enabled {
            Some(ee.gamepad_buttons(profile))
        } else {
            None
        };
        // Keys excluded from keyboard processing this scan (XInput override / keyboard off).
        let mut skip = KeyMask::new();
        if let Some(buttons) = gamepad_buttons {
            for (i, k) in matrix.keys.iter().enumerate() {
                let gp_btn = buttons.get(i).copied().unwrap_or(GamepadButton::None as u8);
                if gp_btn != GamepadButton::None as u8 {
                    xinput.process_key(i, k, gp_btn);
                    if gp_opts.gamepad_override() {
                        skip.set(i, true);
                    }
                }
                if !gp_opts.keyboard_enabled() {
                    skip.set(i, true);
                }
            }
        }

        // Pass 1: press/release edges and held advanced keys, a few word ops per 32 keys.
        let num_words = KeyMask::words_for(NUM_KEYS);
        let pressed_mask = matrix.pressed_mask();
        let mut events = KeyMask::new();
        let mut any_event = 0u32;
        for w in 0..num_words {
            let pressed = pressed_mask.words[w];
            let mut e = (pressed ^ self.key_press_states.words[w])
                | (pressed & self.ak_active.words[w]);
            e &= !skip.words[w];
            if current_layer == 0 {
                e &= !self.key_disabled.words[w];
            }
            events.words[w] = e;
            any_event |= e;
        }

        // Pass 2: dispatch events in key order (rare; kept out of the scan loop).
        if any_event != 0 {
            for w in 0..num_words {
                let mut e = events.words[w];
                while e != 0 {
                    let i = w * 32 + e.trailing_zeros() as usize;
                    e &= e - 1;
                    // A key-lock toggled by an earlier key in this pass still applies.
                    if current_layer == 0 && self.key_disabled.get(i) {
                        continue;
                    }
                    let is_pressed = matrix.keys[i].is_pressed;
                    let last_pressed = self.key_press_states.get(i);
                    has_non_tap_hold_press |= self.key_event(
                        i,
                        is_pressed,
                        last_pressed,
                        now_ms,
                        current_layer,
                        ee,
                        flash,
                        matrix,
                        hid,
                        board,
                        ble,
                    );
                    self.key_press_states.set(i, is_pressed);
                }
            }
        }

        if has_non_tap_hold_press || now_ms.wrapping_sub(self.last_ak_tick) > 0 {
            let mut emit = ActionEmitter::new();
            self.advanced_keys
                .tick(now_ms, has_non_tap_hold_press, ee, &mut emit);
            self.apply_actions(&emit, ee, flash, hid, board, ble);
            self.last_ak_tick = now_ms;
        }

        let pending = if self.should_send_reports {
            self.should_send_reports = false;
            hid.take_pending()
        } else {
            PendingHidReports::default()
        };

        // Process deferred actions for the next matrix scan
        let tick_rate = ee.tick_rate(ee.current_profile() as usize);
        let mut ready = [DeferredAction::default(); MAX_DEFERRED_ACTIONS];
        let count = self.deferred.tick_and_drain(tick_rate, &mut ready);
        for action in &ready[..count] {
            match action.kind {
                DeferredActionType::Press => {
                    self.register(action.key, action.keycode, ee, flash, hid, board, ble);
                }
                DeferredActionType::Release => {
                    self.unregister(action.keycode, hid);
                }
                DeferredActionType::Tap => {
                    let pushed = self.deferred.push(
                        DeferredAction {
                            kind: DeferredActionType::Release,
                            key: action.key,
                            keycode: action.keycode,
                            ticks: 0,
                        },
                        tick_rate,
                    );
                    if pushed {
                        self.register(action.key, action.keycode, ee, flash, hid, board, ble);
                    }
                }
                DeferredActionType::None => {}
            }
        }

        pending
    }

    fn apply_actions<const VS: usize, const WL: usize>(
        &mut self,
        emit: &ActionEmitter,
        ee: &mut Eeconfig<VS, WL>,
        flash: &mut impl FlashStorage,
        hid: &mut HidState,
        board: &mut impl BoardControl,
        ble: &mut impl BleControl,
    ) {
        for act in emit.iter() {
            match act {
                KeyAction::Register { key, keycode } => {
                    self.register(key, keycode, ee, flash, hid, board, ble);
                }
                KeyAction::Unregister { keycode, .. } => {
                    self.unregister(keycode, hid);
                }
            }
        }
    }

    fn set_profile<const VS: usize, const WL: usize>(
        &mut self,
        profile: u8,
        ee: &mut Eeconfig<VS, WL>,
        flash: &mut impl FlashStorage,
        hid: &mut HidState,
        board: &mut impl BoardControl,
        ble: &mut impl BleControl,
    ) {
        if (profile as usize) >= ee.layout.num_profiles {
            return;
        }
        self.clear_advanced_keys(ee, flash, hid, board, ble);
        ee.set_current_profile(flash, profile);
        self.load_advanced_keys(ee);
    }

    #[allow(clippy::too_many_arguments)]
    fn register<const VS: usize, const WL: usize>(
        &mut self,
        key: u8,
        keycode: u8,
        ee: &mut Eeconfig<VS, WL>,
        flash: &mut impl FlashStorage,
        hid: &mut HidState,
        board: &mut impl BoardControl,
        ble: &mut impl BleControl,
    ) {
        if keycode == KC_NO {
            return;
        }
        if is_hid_keycode(keycode) {
            hid.keycode_add(keycode);
            self.should_send_reports = true;
        } else if is_momentary_layer(keycode) {
            self.layer_mask |= 1u16 << mo_get_layer(keycode);
        } else if is_profile_keycode(keycode) {
            self.set_profile(pf_get_profile(keycode), ee, flash, hid, board, ble);
        } else {
            match keycode {
                SP_KEY_LOCK => {
                    if (key as usize) < NUM_KEYS {
                        if (key as usize) < NUM_KEYS {
                            self.key_disabled.toggle(key as usize);
                        }
                    }
                }
                SP_LAYER_LOCK => {
                    let cur = self.current_layer();
                    self.default_layer = if cur == self.default_layer { 0 } else { cur };
                }
                SP_PROFILE_SWAP => {
                    let target = if ee.current_profile() != 0 {
                        0
                    } else {
                        ee.last_non_default_profile()
                    };
                    self.set_profile(target, ee, flash, hid, board, ble);
                }
                SP_PROFILE_NEXT => {
                    let next = ((ee.current_profile() as usize + 1) % ee.layout.num_profiles) as u8;
                    self.set_profile(next, ee, flash, hid, board, ble);
                }
                SP_BOOT => board.enter_bootloader(),
                SP_BT_TOGGLE => ble.toggle_transport(),
                SP_BT_USB => ble.set_transport_mode(BleTransportMode::Usb),
                SP_BT_BLE => {
                    ble.set_transport_mode(BleTransportMode::Wireless);
                    ble.select_wireless_backend(WirelessBackend::Ble);
                }
                SP_BT_24G => {
                    ble.set_transport_mode(BleTransportMode::Wireless);
                    ble.select_wireless_backend(WirelessBackend::Esb);
                }
                SP_BT_PAIR => ble.start_advertising(true),
                _ => {}
            }
        }
    }

    fn unregister(&mut self, keycode: u8, hid: &mut HidState) {
        if keycode == KC_NO {
            return;
        }
        if is_hid_keycode(keycode) {
            hid.keycode_remove(keycode);
            self.should_send_reports = true;
        } else if is_momentary_layer(keycode) {
            self.layer_mask &= !(1u16 << mo_get_layer(keycode));
        }
    }
}
