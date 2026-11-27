//! Persistent EEPROM configuration store (v0x0105), binary-compatible with `src/eeconfig.c`.

use crate::traits::FlashStorage;
use crate::types::{
    Actuation, AdvancedKey, Calibration, EeconfigOptions, GamepadOptions, MacroNode,
};
use crate::wear_leveling::WearLeveling;

pub const EECONFIG_MAGIC_START: u32 = 0x0A42_494C;
pub const EECONFIG_MAGIC_END: u32 = 0x0A4B_4D48;
pub const EECONFIG_VERSION: u16 = 0x0105;
pub const DEFAULT_TICK_RATE: u8 = 30;

/// Compile-time geometry and byte offsets for the packed `eeconfig_t` v0x0105 layout.
#[derive(Debug, Clone, Copy)]
pub struct EeconfigLayout {
    pub num_profiles: usize,
    pub num_layers: usize,
    pub num_keys: usize,
    pub num_advanced_keys: usize,
    pub num_dks_max_bindings: usize,
    pub num_macro_nodes: usize,
}

impl EeconfigLayout {
    pub const OFFSET_MAGIC_START: usize = 0;
    pub const OFFSET_VERSION: usize = 4;
    pub const OFFSET_CALIBRATION: usize = 6;
    pub const OFFSET_BOTTOM_OUT_THRESHOLD: usize = 10;

    pub const fn offset_options(&self) -> usize {
        Self::OFFSET_BOTTOM_OUT_THRESHOLD + self.num_keys * 2
    }

    pub const fn offset_current_profile(&self) -> usize {
        self.offset_options() + 2
    }

    pub const fn offset_last_non_default_profile(&self) -> usize {
        self.offset_current_profile() + 1
    }

    pub const fn global_size(&self) -> usize {
        self.offset_last_non_default_profile() + 1
    }

    pub const fn advanced_key_size(&self) -> usize {
        3 + 2 * self.num_dks_max_bindings + 1
    }

    pub const fn profile_offset_keymap(&self) -> usize {
        0
    }

    pub const fn profile_offset_actuation_map(&self) -> usize {
        self.num_layers * self.num_keys
    }

    pub const fn profile_offset_advanced_keys(&self) -> usize {
        self.profile_offset_actuation_map() + self.num_keys * Actuation::SIZE
    }

    pub const fn profile_offset_macros(&self) -> usize {
        self.profile_offset_advanced_keys() + self.num_advanced_keys * self.advanced_key_size()
    }

    pub const fn profile_offset_gamepad_buttons(&self) -> usize {
        self.profile_offset_macros() + self.num_macro_nodes * MacroNode::SIZE
    }

    pub const fn profile_offset_gamepad_options(&self) -> usize {
        self.profile_offset_gamepad_buttons() + self.num_keys
    }

    pub const fn profile_offset_tick_rate(&self) -> usize {
        self.profile_offset_gamepad_options() + GamepadOptions::SIZE
    }

    pub const fn profile_size(&self) -> usize {
        self.profile_offset_tick_rate() + 1
    }

    pub const fn profile_base(&self, profile: usize) -> usize {
        self.global_size() + profile * self.profile_size()
    }

    pub const fn offset_magic_end(&self) -> usize {
        self.global_size() + self.num_profiles * self.profile_size()
    }

    pub const fn total_size(&self) -> usize {
        self.offset_magic_end() + 4
    }
}

/// Default configuration values provided by `keyboard_gen.rs`.
pub struct EeconfigDefaults<'a> {
    pub calibration: Calibration,
    pub actuation_point: u8,
    pub default_keymaps: &'a [u8], // flattened [num_profiles][num_layers][num_keys]
}

/// Persistent configuration store backed by `WearLeveling`.
pub struct Eeconfig<const VIRTUAL_SIZE: usize, const WRITE_LOG_SIZE: usize> {
    pub wl: WearLeveling<VIRTUAL_SIZE, WRITE_LOG_SIZE>,
    pub layout: EeconfigLayout,
}

impl<const VIRTUAL_SIZE: usize, const WRITE_LOG_SIZE: usize>
    Eeconfig<VIRTUAL_SIZE, WRITE_LOG_SIZE>
{
    pub const fn new(layout: EeconfigLayout) -> Self {
        Self {
            wl: WearLeveling::new(),
            layout,
        }
    }

    pub fn init<F: FlashStorage>(&mut self, flash: &mut F, defaults: &EeconfigDefaults<'_>) -> bool {
        if !self.wl.init(flash) {
            return false;
        }
        if !self.is_latest_version() {
            return self.reset(flash, defaults);
        }
        true
    }

    pub fn is_latest_version(&self) -> bool {
        let c = &self.wl.cache;
        let magic_start = u32::from_le_bytes([c[0], c[1], c[2], c[3]]);
        let version = u16::from_le_bytes([c[4], c[5]]);
        let end_off = self.layout.offset_magic_end();
        let magic_end =
            u32::from_le_bytes([c[end_off], c[end_off + 1], c[end_off + 2], c[end_off + 3]]);
        magic_start == EECONFIG_MAGIC_START
            && version == EECONFIG_VERSION
            && magic_end == EECONFIG_MAGIC_END
    }

    pub fn reset<F: FlashStorage>(&mut self, flash: &mut F, defaults: &EeconfigDefaults<'_>) -> bool {
        let mut ok = true;
        ok &= self.wl.write(
            flash,
            EeconfigLayout::OFFSET_MAGIC_START,
            &EECONFIG_MAGIC_START.to_le_bytes(),
        );
        ok &= self.wl.write(
            flash,
            EeconfigLayout::OFFSET_VERSION,
            &EECONFIG_VERSION.to_le_bytes(),
        );
        ok &= self.wl.write(
            flash,
            EeconfigLayout::OFFSET_CALIBRATION,
            &defaults.calibration.to_bytes(),
        );
        for k in 0..self.layout.num_keys {
            ok &= self.wl.write(
                flash,
                EeconfigLayout::OFFSET_BOTTOM_OUT_THRESHOLD + k * 2,
                &0u16.to_le_bytes(),
            );
        }
        ok &= self.wl.write(
            flash,
            self.layout.offset_options(),
            &EeconfigOptions::default_options().to_bytes(),
        );
        ok &= self.wl.write(flash, self.layout.offset_current_profile(), &[0]);
        let last_non_default = 1usize.min(self.layout.num_profiles.saturating_sub(1)) as u8;
        ok &= self.wl.write(
            flash,
            self.layout.offset_last_non_default_profile(),
            &[last_non_default],
        );

        for p in 0..self.layout.num_profiles {
            ok &= self.reset_profile(flash, p, defaults);
        }

        ok &= self.wl.write(
            flash,
            self.layout.offset_magic_end(),
            &EECONFIG_MAGIC_END.to_le_bytes(),
        );
        ok
    }

    pub fn reset_profile<F: FlashStorage>(
        &mut self,
        flash: &mut F,
        profile: usize,
        defaults: &EeconfigDefaults<'_>,
    ) -> bool {
        if profile >= self.layout.num_profiles {
            return false;
        }
        let base = self.layout.profile_base(profile);
        let keymap_len = self.layout.num_layers * self.layout.num_keys;
        let km_start = profile * keymap_len;
        let mut ok = self.wl.write(
            flash,
            base + self.layout.profile_offset_keymap(),
            &defaults.default_keymaps[km_start..km_start + keymap_len],
        );

        let default_act = Actuation::default_with_point(defaults.actuation_point).to_bytes();
        let act_base = base + self.layout.profile_offset_actuation_map();
        for k in 0..self.layout.num_keys {
            ok &= self.wl.write(flash, act_base + k * Actuation::SIZE, &default_act);
        }

        // Zero advanced keys
        let ak_base = base + self.layout.profile_offset_advanced_keys();
        let ak_total = self.layout.num_advanced_keys * self.layout.advanced_key_size();
        let zeros = [0u8; 64];
        let mut rem = ak_total;
        let mut off = 0;
        while rem > 0 {
            let n = rem.min(zeros.len());
            ok &= self.wl.write(flash, ak_base + off, &zeros[..n]);
            off += n;
            rem -= n;
        }

        // Empty macro nodes
        let macro_node = MacroNode::empty().to_bytes();
        let m_base = base + self.layout.profile_offset_macros();
        for n in 0..self.layout.num_macro_nodes {
            ok &= self.wl.write(flash, m_base + n * MacroNode::SIZE, &macro_node);
        }

        // Zero gamepad buttons
        let gp_btn_base = base + self.layout.profile_offset_gamepad_buttons();
        let mut rem = self.layout.num_keys;
        let mut off = 0;
        while rem > 0 {
            let n = rem.min(zeros.len());
            ok &= self.wl.write(flash, gp_btn_base + off, &zeros[..n]);
            off += n;
            rem -= n;
        }

        ok &= self.wl.write(
            flash,
            base + self.layout.profile_offset_gamepad_options(),
            &GamepadOptions::default_options().to_bytes(),
        );
        ok &= self.wl.write(
            flash,
            base + self.layout.profile_offset_tick_rate(),
            &[DEFAULT_TICK_RATE],
        );
        ok
    }

    pub fn duplicate_profile<F: FlashStorage>(
        &mut self,
        flash: &mut F,
        dst_profile: usize,
        src_profile: usize,
    ) -> bool {
        if dst_profile >= self.layout.num_profiles || src_profile >= self.layout.num_profiles {
            return false;
        }
        if dst_profile == src_profile {
            return true;
        }
        let psize = self.layout.profile_size();
        let src_base = self.layout.profile_base(src_profile);
        let dst_base = self.layout.profile_base(dst_profile);
        let mut buf = [0u8; 64];
        let mut off = 0;
        while off < psize {
            let n = (psize - off).min(buf.len());
            buf[..n].copy_from_slice(&self.wl.cache[src_base + off..src_base + off + n]);
            if !self.wl.write(flash, dst_base + off, &buf[..n]) {
                return false;
            }
            off += n;
        }
        true
    }

    // ---- Typed Readers ----

    pub fn calibration(&self) -> Calibration {
        let off = EeconfigLayout::OFFSET_CALIBRATION;
        Calibration::from_bytes(&self.wl.cache[off..off + Calibration::SIZE])
    }

    pub fn bottom_out_threshold(&self, key: usize) -> u16 {
        let off = EeconfigLayout::OFFSET_BOTTOM_OUT_THRESHOLD + key * 2;
        u16::from_le_bytes([self.wl.cache[off], self.wl.cache[off + 1]])
    }

    pub fn options(&self) -> EeconfigOptions {
        let off = self.layout.offset_options();
        EeconfigOptions::from_bytes(&self.wl.cache[off..off + EeconfigOptions::SIZE])
    }

    pub fn current_profile(&self) -> u8 {
        self.wl.cache[self.layout.offset_current_profile()]
    }

    pub fn last_non_default_profile(&self) -> u8 {
        self.wl.cache[self.layout.offset_last_non_default_profile()]
    }

    pub fn keymap_keycode(&self, profile: usize, layer: usize, key: usize) -> u8 {
        let off = self.layout.profile_base(profile)
            + self.layout.profile_offset_keymap()
            + layer * self.layout.num_keys
            + key;
        self.wl.cache[off]
    }

    pub fn actuation(&self, profile: usize, key: usize) -> Actuation {
        let off = self.layout.profile_base(profile)
            + self.layout.profile_offset_actuation_map()
            + key * Actuation::SIZE;
        Actuation::from_bytes(&self.wl.cache[off..off + Actuation::SIZE])
    }

    pub fn advanced_key<const MAX_BINDINGS: usize>(
        &self,
        profile: usize,
        index: usize,
    ) -> AdvancedKey<MAX_BINDINGS> {
        let size = self.layout.advanced_key_size();
        let off = self.layout.profile_base(profile)
            + self.layout.profile_offset_advanced_keys()
            + index * size;
        AdvancedKey::from_bytes(&self.wl.cache[off..off + size])
    }

    pub fn macro_node(&self, profile: usize, node_id: usize) -> MacroNode {
        let off = self.layout.profile_base(profile)
            + self.layout.profile_offset_macros()
            + node_id * MacroNode::SIZE;
        MacroNode::from_bytes(&self.wl.cache[off..off + MacroNode::SIZE])
    }

    pub fn gamepad_button(&self, profile: usize, key: usize) -> u8 {
        let off = self.layout.profile_base(profile)
            + self.layout.profile_offset_gamepad_buttons()
            + key;
        self.wl.cache[off]
    }

    pub fn gamepad_options(&self, profile: usize) -> GamepadOptions {
        let off =
            self.layout.profile_base(profile) + self.layout.profile_offset_gamepad_options();
        GamepadOptions::from_bytes(&self.wl.cache[off..off + GamepadOptions::SIZE])
    }

    pub fn tick_rate(&self, profile: usize) -> u8 {
        let off = self.layout.profile_base(profile) + self.layout.profile_offset_tick_rate();
        self.wl.cache[off]
    }

    // ---- Typed Writers ----

    pub fn set_calibration<F: FlashStorage>(&mut self, flash: &mut F, cal: Calibration) -> bool {
        self.wl
            .write(flash, EeconfigLayout::OFFSET_CALIBRATION, &cal.to_bytes())
    }

    pub fn set_options<F: FlashStorage>(&mut self, flash: &mut F, opts: EeconfigOptions) -> bool {
        self.wl
            .write(flash, self.layout.offset_options(), &opts.to_bytes())
    }

    pub fn set_current_profile<F: FlashStorage>(&mut self, flash: &mut F, profile: u8) -> bool {
        if (profile as usize) >= self.layout.num_profiles {
            return false;
        }
        let mut ok = self
            .wl
            .write(flash, self.layout.offset_current_profile(), &[profile]);
        if ok && profile != 0 {
            ok = self.wl.write(
                flash,
                self.layout.offset_last_non_default_profile(),
                &[profile],
            );
        }
        ok
    }

    pub fn set_bottom_out_threshold<F: FlashStorage>(
        &mut self,
        flash: &mut F,
        key: usize,
        val: u16,
    ) -> bool {
        let off = EeconfigLayout::OFFSET_BOTTOM_OUT_THRESHOLD + key * 2;
        self.wl.write(flash, off, &val.to_le_bytes())
    }
}
