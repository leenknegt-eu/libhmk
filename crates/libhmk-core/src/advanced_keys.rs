//! Advanced key processors: Null Bind (SOCD/Rappy Snappy), Dynamic Keystroke,
//! Tap-Hold, Toggle, and Macro linked lists (`src/advanced_keys.c`).

use crate::deferred_actions::{DeferredAction, DeferredActionQueue, DeferredActionType};
use crate::eeconfig::Eeconfig;
use crate::keycodes::KC_NO;
use crate::matrix::Matrix;
use crate::types::{
    AdvancedKeyType, DksAction, MacroAction, NullBindBehavior, MACRO_NODE_NONE,
};

pub const MAX_MACRO_ACTIVE_KEYCODES: usize = 16;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum AkEventType {
    Hold = 0,
    Press = 1,
    BottomOut = 2,
    ReleaseFromBottomOut = 3,
    Release = 4,
}

#[derive(Debug, Clone, Copy)]
pub struct AkEvent {
    pub kind: AkEventType,
    pub key: u8,
    pub keycode: u8,
    pub ak_index: usize,
}

/// Output actions emitted by advanced-key processing to be applied by the Layout engine.
#[derive(Debug, Clone, Copy)]
pub enum KeyAction {
    Register { key: u8, keycode: u8 },
    Unregister { key: u8, keycode: u8 },
}

/// Bounded queue of actions emitted during a single advanced-key call.
pub struct ActionEmitter<const CAP: usize = 32> {
    items: [Option<KeyAction>; CAP],
    len: usize,
}

impl<const CAP: usize> ActionEmitter<CAP> {
    pub const fn new() -> Self {
        Self {
            items: [None; CAP],
            len: 0,
        }
    }

    pub fn register(&mut self, key: u8, keycode: u8) {
        if self.len < CAP {
            self.items[self.len] = Some(KeyAction::Register { key, keycode });
            self.len += 1;
        }
    }

    pub fn unregister(&mut self, key: u8, keycode: u8) {
        if self.len < CAP {
            self.items[self.len] = Some(KeyAction::Unregister { key, keycode });
            self.len += 1;
        }
    }

    pub fn iter(&self) -> impl Iterator<Item = KeyAction> + '_ {
        self.items[..self.len].iter().filter_map(|x| *x)
    }
}

#[derive(Debug, Clone, Copy, Default)]
struct NullBindState {
    is_pressed: [bool; 2],
    keycodes: [u8; 2],
}

#[derive(Debug, Clone, Copy)]
struct DksState<const MAX_BINDINGS: usize> {
    is_pressed: [bool; MAX_BINDINGS],
    is_bottomed_out: bool,
}

impl<const MAX_BINDINGS: usize> DksState<MAX_BINDINGS> {
    const fn new() -> Self {
        Self {
            is_pressed: [false; MAX_BINDINGS],
            is_bottomed_out: false,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
enum TapHoldStage {
    #[default]
    None,
    Tap,
    Hold,
}

#[derive(Debug, Clone, Copy, Default)]
struct TapHoldState {
    since: u32,
    stage: TapHoldStage,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
enum ToggleStage {
    #[default]
    None,
    Toggle,
    Normal,
}

#[derive(Debug, Clone, Copy, Default)]
struct ToggleState {
    since: u32,
    stage: ToggleStage,
    is_toggled: bool,
}

#[derive(Debug, Clone, Copy)]
struct MacroState {
    since: u32,
    deferred_tap_ticks: u16,
    current_node: u8,
    num_active: usize,
    active_keycodes: [u8; MAX_MACRO_ACTIVE_KEYCODES],
}

impl MacroState {
    const fn new() -> Self {
        Self {
            since: 0,
            deferred_tap_ticks: 0,
            current_node: MACRO_NODE_NONE,
            num_active: 0,
            active_keycodes: [0; MAX_MACRO_ACTIVE_KEYCODES],
        }
    }

    fn reset(&mut self) {
        self.since = 0;
        self.deferred_tap_ticks = 0;
        self.current_node = MACRO_NODE_NONE;
        self.num_active = 0;
    }

    fn add_active(&mut self, kc: u8) {
        if kc == KC_NO || self.num_active >= MAX_MACRO_ACTIVE_KEYCODES {
            return;
        }
        if self.active_keycodes[..self.num_active].contains(&kc) {
            return;
        }
        self.active_keycodes[self.num_active] = kc;
        self.num_active += 1;
    }

    fn remove_active(&mut self, kc: u8) {
        if let Some(pos) = self.active_keycodes[..self.num_active]
            .iter()
            .position(|&c| c == kc)
        {
            self.active_keycodes.copy_within(pos + 1..self.num_active, pos);
            self.num_active -= 1;
        }
    }
}

#[derive(Debug, Clone, Copy)]
enum AkState<const MAX_BINDINGS: usize> {
    None,
    NullBind(NullBindState),
    Dks(DksState<MAX_BINDINGS>),
    TapHold(TapHoldState),
    Toggle(ToggleState),
    Macro(MacroState),
}

pub struct AdvancedKeyEngine<const NUM_AK: usize, const MAX_BINDINGS: usize> {
    states: [AkState<MAX_BINDINGS>; NUM_AK],
}

impl<const NUM_AK: usize, const MAX_BINDINGS: usize> AdvancedKeyEngine<NUM_AK, MAX_BINDINGS> {
    pub const fn new() -> Self {
        Self {
            states: [AkState::None; NUM_AK],
        }
    }

    pub fn reset_states<const VS: usize, const WL: usize>(&mut self, ee: &Eeconfig<VS, WL>) {
        let profile = ee.current_profile() as usize;
        for i in 0..NUM_AK {
            let ak = ee.advanced_key::<MAX_BINDINGS>(profile, i);
            self.states[i] = match ak.kind() {
                AdvancedKeyType::None => AkState::None,
                AdvancedKeyType::NullBind => AkState::NullBind(NullBindState::default()),
                AdvancedKeyType::DynamicKeystroke => AkState::Dks(DksState::new()),
                AdvancedKeyType::TapHold => AkState::TapHold(TapHoldState::default()),
                AdvancedKeyType::Toggle => AkState::Toggle(ToggleState::default()),
                AdvancedKeyType::Macro => AkState::Macro(MacroState::new()),
            };
        }
    }

    pub fn clear<const VS: usize, const WL: usize>(
        &mut self,
        ee: &Eeconfig<VS, WL>,
        emit: &mut ActionEmitter,
    ) {
        let profile = ee.current_profile() as usize;
        for i in 0..NUM_AK {
            let ak = ee.advanced_key::<MAX_BINDINGS>(profile, i);
            match (ak.kind(), &mut self.states[i]) {
                (AdvancedKeyType::TapHold, AkState::TapHold(st)) => {
                    if st.stage == TapHoldStage::Hold {
                        emit.unregister(ak.key(), ak.tap_hold().hold_keycode);
                    }
                }
                (AdvancedKeyType::Toggle, AkState::Toggle(st)) => {
                    if st.stage != ToggleStage::None || st.is_toggled {
                        emit.unregister(ak.key(), ak.toggle().keycode);
                    }
                }
                (AdvancedKeyType::Macro, AkState::Macro(st)) => {
                    for j in 0..st.num_active {
                        emit.unregister(ak.key(), st.active_keycodes[j]);
                    }
                    st.reset();
                }
                _ => {}
            }
        }
        self.reset_states(ee);
    }

    pub fn process<const NUM_KEYS: usize, const VS: usize, const WL: usize>(
        &mut self,
        event: AkEvent,
        now_ms: u32,
        ee: &Eeconfig<VS, WL>,
        matrix: &mut Matrix<NUM_KEYS>,
        deferred: &mut DeferredActionQueue,
        emit: &mut ActionEmitter,
    ) {
        if event.ak_index >= NUM_AK {
            return;
        }
        let profile = ee.current_profile() as usize;
        let ak = ee.advanced_key::<MAX_BINDINGS>(profile, event.ak_index);
        let tick_rate = ee.tick_rate(profile);

        match (ak.kind(), &mut self.states[event.ak_index]) {
            (AdvancedKeyType::NullBind, AkState::NullBind(st)) => {
                let nb = ak.null_bind();
                let keys = [ak.key(), nb.secondary_key];
                let idx = if event.key == keys[0] { 0 } else { 1 };

                match event.kind {
                    AkEventType::Press => st.keycodes[idx] = event.keycode,
                    AkEventType::Release => {
                        if st.is_pressed[idx] {
                            emit.unregister(keys[idx], st.keycodes[idx]);
                            st.is_pressed[idx] = false;
                        }
                        st.keycodes[idx] = KC_NO;
                    }
                    _ => {}
                }

                let mut pressed = [st.keycodes[0] != KC_NO, st.keycodes[1] != KC_NO];
                if pressed[0] && pressed[1] {
                    let k0 = keys[0] as usize;
                    let k1 = keys[1] as usize;
                    let d0 = if k0 < NUM_KEYS { matrix.keys[k0].distance } else { 0 };
                    let d1 = if k1 < NUM_KEYS { matrix.keys[k1].distance } else { 0 };
                    let behavior = NullBindBehavior::from(nb.behavior);

                    if nb.bottom_out_point > 0
                        && d0 >= nb.bottom_out_point
                        && d1 >= nb.bottom_out_point
                    {
                        pressed = [true, true];
                    } else if behavior == NullBindBehavior::Distance {
                        let d_idx = if idx == 0 { d0 } else { d1 };
                        let d_other = if idx == 0 { d1 } else { d0 };
                        pressed[idx] = d_idx >= d_other;
                        pressed[idx ^ 1] = !pressed[idx];
                    } else if event.kind == AkEventType::Press {
                        pressed[idx] = behavior != NullBindBehavior::Neutral
                            && (behavior == NullBindBehavior::Last
                                || (behavior == NullBindBehavior::Primary && idx == 0)
                                || (behavior == NullBindBehavior::Secondary && idx == 1));
                        pressed[idx ^ 1] = behavior != NullBindBehavior::Neutral && !pressed[idx];
                    } else {
                        pressed = st.is_pressed;
                    }
                }

                for i in 0..2 {
                    if pressed[i] && !st.is_pressed[i] {
                        emit.register(keys[i], st.keycodes[i]);
                        st.is_pressed[i] = true;
                    } else if !pressed[i] && st.is_pressed[i] {
                        emit.unregister(keys[i], st.keycodes[i]);
                        st.is_pressed[i] = false;
                    }
                }
            }

            (AdvancedKeyType::DynamicKeystroke, AkState::Dks(st)) => {
                let dks = ak.dynamic_keystroke();
                let dist = matrix.keys[event.key as usize].distance;
                let is_bottomed_out = dist >= dks.bottom_out_point;
                let mut ev_type = event.kind;

                if is_bottomed_out && !st.is_bottomed_out {
                    ev_type = AkEventType::BottomOut;
                } else if ev_type != AkEventType::Release && !is_bottomed_out && st.is_bottomed_out
                {
                    ev_type = AkEventType::ReleaseFromBottomOut;
                }
                st.is_bottomed_out = is_bottomed_out;

                if ev_type == AkEventType::Hold {
                    return;
                }

                matrix.set_rapid_trigger_disabled(
                    event.key as usize,
                    ev_type != AkEventType::Release,
                );
                let shift = ((ev_type as u8) - (AkEventType::Press as u8)) * 2;
                for i in 0..MAX_BINDINGS {
                    let kc = dks.keycodes[i];
                    let action = DksAction::from(dks.bitmap[i] >> shift);
                    if kc == KC_NO || action == DksAction::Hold {
                        continue;
                    }
                    if st.is_pressed[i] {
                        emit.unregister(event.key, kc);
                        st.is_pressed[i] = false;
                    }
                    if action == DksAction::Press || action == DksAction::Tap {
                        let pushed = deferred.push(
                            DeferredAction {
                                kind: if action == DksAction::Press {
                                    DeferredActionType::Press
                                } else {
                                    DeferredActionType::Tap
                                },
                                key: event.key,
                                keycode: kc,
                                ticks: 0,
                            },
                            tick_rate,
                        );
                        st.is_pressed[i] = pushed && action == DksAction::Press;
                    }
                }
            }

            (AdvancedKeyType::TapHold, AkState::TapHold(st)) => {
                let th = ak.tap_hold();
                match event.kind {
                    AkEventType::Press => {
                        st.since = now_ms;
                        st.stage = TapHoldStage::Tap;
                    }
                    AkEventType::Release => {
                        if st.stage == TapHoldStage::Tap {
                            let pushed = deferred.push(
                                DeferredAction {
                                    kind: DeferredActionType::Release,
                                    key: event.key,
                                    keycode: th.tap_keycode,
                                    ticks: 0,
                                },
                                tick_rate,
                            );
                            if pushed {
                                emit.register(event.key, th.tap_keycode);
                            }
                        } else if st.stage == TapHoldStage::Hold {
                            emit.unregister(event.key, th.hold_keycode);
                        }
                        st.stage = TapHoldStage::None;
                    }
                    _ => {}
                }
            }

            (AdvancedKeyType::Toggle, AkState::Toggle(st)) => {
                let tog = ak.toggle();
                match event.kind {
                    AkEventType::Press => {
                        emit.register(event.key, tog.keycode);
                        st.is_toggled = !st.is_toggled;
                        if st.is_toggled {
                            st.since = now_ms;
                            st.stage = ToggleStage::Toggle;
                        } else {
                            st.stage = ToggleStage::Normal;
                        }
                    }
                    AkEventType::Release => {
                        if !st.is_toggled {
                            emit.unregister(event.key, tog.keycode);
                        }
                        st.stage = ToggleStage::None;
                    }
                    _ => {}
                }
            }

            (AdvancedKeyType::Macro, AkState::Macro(st)) => match event.kind {
                AkEventType::Press => {
                    for j in 0..st.num_active {
                        emit.unregister(event.key, st.active_keycodes[j]);
                    }
                    st.reset();
                    Self::macro_run_step(st, event.key, ak.macro_head(), now_ms, ee, emit);
                }
                AkEventType::Release => {
                    for j in 0..st.num_active {
                        emit.unregister(event.key, st.active_keycodes[j]);
                    }
                    st.reset();
                }
                _ => {}
            },

            _ => {}
        }
    }

    fn macro_run_step<const VS: usize, const WL: usize>(
        st: &mut MacroState,
        key: u8,
        node_id: u8,
        now_ms: u32,
        ee: &Eeconfig<VS, WL>,
        emit: &mut ActionEmitter,
    ) {
        if node_id == MACRO_NODE_NONE || (node_id as usize) >= ee.layout.num_macro_nodes {
            st.current_node = MACRO_NODE_NONE;
            return;
        }
        let profile = ee.current_profile() as usize;
        let node = ee.macro_node(profile, node_id as usize);
        st.current_node = node_id;
        st.since = now_ms;

        match MacroAction::from(node.action) {
            MacroAction::Press => {
                emit.register(key, node.keycode);
                st.add_active(node.keycode);
            }
            MacroAction::Tap => {
                emit.register(key, node.keycode);
                st.add_active(node.keycode);
                st.deferred_tap_ticks = ee.tick_rate(profile) as u16;
            }
            MacroAction::Release => {
                emit.unregister(key, node.keycode);
                st.remove_active(node.keycode);
            }
            MacroAction::None => {}
        }
    }

    pub fn tick<const VS: usize, const WL: usize>(
        &mut self,
        now_ms: u32,
        has_non_tap_hold_press: bool,
        ee: &Eeconfig<VS, WL>,
        emit: &mut ActionEmitter,
    ) {
        let profile = ee.current_profile() as usize;
        for i in 0..NUM_AK {
            // Only tap-hold, toggle and running macros have time-based behavior.
            match &self.states[i] {
                AkState::TapHold(_) | AkState::Toggle(_) => {}
                AkState::Macro(st) if st.current_node != MACRO_NODE_NONE => {}
                _ => continue,
            }
            let ak = ee.advanced_key::<MAX_BINDINGS>(profile, i);
            match (ak.kind(), &mut self.states[i]) {
                (AdvancedKeyType::TapHold, AkState::TapHold(st)) => {
                    let th = ak.tap_hold();
                    if st.stage == TapHoldStage::Tap
                        && ((has_non_tap_hold_press && th.hold_on_other_key_press)
                            || now_ms.wrapping_sub(st.since) >= th.tapping_term as u32)
                    {
                        emit.register(ak.key(), th.hold_keycode);
                        st.stage = TapHoldStage::Hold;
                    }
                }
                (AdvancedKeyType::Toggle, AkState::Toggle(st)) => {
                    let tog = ak.toggle();
                    if st.stage == ToggleStage::Toggle
                        && now_ms.wrapping_sub(st.since) >= tog.tapping_term as u32
                    {
                        st.stage = ToggleStage::Normal;
                        st.is_toggled = false;
                    }
                }
                (AdvancedKeyType::Macro, AkState::Macro(st)) => {
                    if st.current_node != MACRO_NODE_NONE
                        && (st.current_node as usize) < ee.layout.num_macro_nodes
                    {
                        let node = ee.macro_node(profile, st.current_node as usize);
                        let elapsed = now_ms.wrapping_sub(st.since) >= node.delay as u32;
                        match MacroAction::from(node.action) {
                            MacroAction::Tap => {
                                if st.deferred_tap_ticks > 0 {
                                    st.deferred_tap_ticks -= 1;
                                    if st.deferred_tap_ticks == 0 {
                                        emit.unregister(ak.key(), node.keycode);
                                        st.remove_active(node.keycode);
                                    }
                                } else if elapsed {
                                    Self::macro_run_step(st, ak.key(), node.next, now_ms, ee, emit);
                                }
                            }
                            MacroAction::Press | MacroAction::Release => {
                                if elapsed {
                                    Self::macro_run_step(st, ak.key(), node.next, now_ms, ee, emit);
                                }
                            }
                            MacroAction::None => {}
                        }
                    }
                }
                _ => {}
            }
        }
    }
}
