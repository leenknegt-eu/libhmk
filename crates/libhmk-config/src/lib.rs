//! Build-time keyboard.json schema loader, validator, and Rust code generator.

pub mod drivers;

use std::fmt::Write as _;
use std::fs;
use std::io::Write as _;
use std::path::Path;

use flate2::write::GzEncoder;
use flate2::Compression;
use serde::{Deserialize, Serialize};

use crate::drivers::{driver_by_name, DriverMetadata, FlashSectors};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum KeyboardUsbPort {
    #[serde(rename = "fs")]
    FullSpeed,
    #[serde(rename = "hs")]
    HighSpeed,
}

#[derive(Debug, Clone, Deserialize)]
pub struct KeyboardUsb {
    pub vid: String,
    pub pid: String,
    pub port: KeyboardUsbPort,
}

#[derive(Debug, Clone, Deserialize)]
pub struct KeyboardSection {
    pub num_profiles: usize,
    pub num_layers: usize,
    pub num_keys: usize,
    pub num_advanced_keys: usize,
    #[serde(default = "default_dks_max_bindings")]
    pub num_dynamic_keystroke_max_bindings: usize,
    #[serde(default = "default_macro_nodes")]
    pub num_macro_nodes: usize,
}

fn default_dks_max_bindings() -> usize {
    4
}

fn default_macro_nodes() -> usize {
    128
}

#[derive(Debug, Clone, Deserialize)]
pub struct KeyboardHardware {
    pub hse_value: u32,
    pub driver: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum BluetoothLfclkSource {
    Xtal,
    Rc,
    Synth,
}

impl Default for BluetoothLfclkSource {
    fn default() -> Self {
        Self::Xtal
    }
}

#[derive(Debug, Clone, Deserialize)]
#[serde(untagged)]
pub enum AdcInputRef {
    Channel(u8),
    Pin(String),
}

#[derive(Debug, Clone, Deserialize)]
pub struct KeyboardBluetooth {
    #[serde(default = "default_true")]
    pub enabled: bool,
    pub device_name: Option<String>,
    #[serde(default)]
    pub lfclk_source: BluetoothLfclkSource,
    #[serde(default)]
    pub tx_power: i8,
    pub battery_pin: Option<AdcInputRef>,
    #[serde(default = "default_divider_ratio")]
    pub battery_divider_ratio: f64,
    #[serde(default = "default_idle_timeout_ms")]
    pub idle_timeout_ms: u32,
    #[serde(default = "default_sleep_timeout_ms")]
    pub sleep_timeout_ms: u32,
}

fn default_true() -> bool {
    true
}
fn default_divider_ratio() -> f64 {
    1.0
}
fn default_idle_timeout_ms() -> u32 {
    30_000
}
fn default_sleep_timeout_ms() -> u32 {
    300_000
}

#[derive(Debug, Clone, Deserialize)]
pub struct KeyboardAnalogRaw {
    pub input: Vec<AdcInputRef>,
    pub vector: Vec<u16>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct KeyboardAnalogMux {
    pub select: Vec<String>,
    pub input: Vec<AdcInputRef>,
    pub matrix: Vec<Vec<u16>>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct KeyboardAnalog {
    pub adc_resolution: Option<u8>,
    #[serde(default)]
    pub invert_adc: bool,
    pub delay: Option<u32>,
    pub raw: Option<KeyboardAnalogRaw>,
    pub mux: Option<KeyboardAnalogMux>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct KeyboardCalibration {
    pub initial_rest_value: u16,
    pub initial_bottom_out_threshold: u16,
}

#[derive(Debug, Clone, Deserialize)]
pub struct KeyboardWearLeveling {
    pub virtual_size: usize,
    pub write_log_size: usize,
}

#[derive(Debug, Clone, Deserialize)]
pub struct KeyboardActuation {
    pub actuation_point: u8,
}

#[derive(Debug, Clone, Deserialize)]
pub struct KeyboardConfig {
    pub name: String,
    pub manufacturer: String,
    pub maintainer: String,
    pub usb: KeyboardUsb,
    pub keyboard: KeyboardSection,
    pub hardware: KeyboardHardware,
    pub analog: KeyboardAnalog,
    pub calibration: KeyboardCalibration,
    pub wear_leveling: Option<KeyboardWearLeveling>,
    pub layout: serde_json::Value,
    pub keymap: Option<Vec<Vec<String>>>,
    pub keymaps: Option<Vec<Vec<Vec<String>>>>,
    pub actuation: Option<KeyboardActuation>,
    pub bluetooth: Option<KeyboardBluetooth>,
}

impl KeyboardConfig {
    pub fn from_file(path: &Path) -> Result<Self, String> {
        let raw = fs::read_to_string(path)
            .map_err(|e| format!("Failed to read {}: {e}", path.display()))?;
        let cfg: Self = serde_json::from_str(&raw)
            .map_err(|e| format!("Invalid JSON in {}: {e}", path.display()))?;
        cfg.validate()?;
        Ok(cfg)
    }

    pub fn driver(&self) -> Result<&'static DriverMetadata, String> {
        driver_by_name(&self.hardware.driver)
    }

    pub fn usb_vid(&self) -> Result<u16, String> {
        parse_hex_u16(&self.usb.vid)
    }

    pub fn usb_pid(&self) -> Result<u16, String> {
        parse_hex_u16(&self.usb.pid)
    }

    pub fn adc_resolution(&self) -> Result<u8, String> {
        Ok(self
            .analog
            .adc_resolution
            .unwrap_or(self.driver()?.adc.max_resolution))
    }

    pub fn resolve_default_keymaps(&self) -> Result<Vec<Vec<Vec<String>>>, String> {
        if let Some(ref kmaps) = self.keymaps {
            Ok(kmaps.clone())
        } else if let Some(ref kmap) = self.keymap {
            Ok(vec![kmap.clone(); self.keyboard.num_profiles])
        } else {
            Err("Either 'keymap' or 'keymaps' must be specified".to_string())
        }
    }

    pub fn wl_virtual_size(&self) -> usize {
        self.wear_leveling
            .as_ref()
            .map(|w| w.virtual_size)
            .unwrap_or(8192)
    }

    pub fn wl_write_log_size(&self) -> usize {
        self.wear_leveling
            .as_ref()
            .map(|w| w.write_log_size)
            .unwrap_or(65536)
    }

    pub fn wl_base_address(&self) -> Result<u32, String> {
        let driver = self.driver()?;
        let backing = (self.wl_virtual_size() + self.wl_write_log_size()) as u32;
        Ok(driver.flash.total_size() - driver.flash.round_up_to_sectors(backing))
    }

    pub fn validate(&self) -> Result<(), String> {
        let _ = self.driver()?;
        let _ = self.usb_vid()?;
        let _ = self.usb_pid()?;

        let kb = &self.keyboard;
        if !(1..=8).contains(&kb.num_profiles) {
            return Err("num_profiles must be in 1..=8".into());
        }
        if !(1..=8).contains(&kb.num_layers) {
            return Err("num_layers must be in 1..=8".into());
        }
        if !(1..=256).contains(&kb.num_keys) {
            return Err("num_keys must be in 1..=256".into());
        }
        if !(1..=64).contains(&kb.num_advanced_keys) {
            return Err("num_advanced_keys must be in 1..=64".into());
        }
        if !(4..=64).contains(&kb.num_dynamic_keystroke_max_bindings) {
            return Err("num_dynamic_keystroke_max_bindings must be in 4..=64".into());
        }
        if !(1..=255).contains(&kb.num_macro_nodes) {
            return Err("num_macro_nodes must be in 1..=255".into());
        }

        let keymaps = self.resolve_default_keymaps()?;
        if keymaps.len() != kb.num_profiles {
            return Err(format!(
                "Expected {} profiles in keymaps, got {}",
                kb.num_profiles,
                keymaps.len()
            ));
        }
        for (pi, prof) in keymaps.iter().enumerate() {
            if prof.len() != kb.num_layers {
                return Err(format!(
                    "Profile {pi}: expected {} layers, got {}",
                    kb.num_layers,
                    prof.len()
                ));
            }
            for (li, layer) in prof.iter().enumerate() {
                if layer.len() != kb.num_keys {
                    return Err(format!(
                        "Profile {pi} layer {li}: expected {} keys, got {}",
                        kb.num_keys,
                        layer.len()
                    ));
                }
            }
        }
        Ok(())
    }

    pub fn resolve_adc_channel(&self, input: &AdcInputRef) -> Result<u8, String> {
        let driver = self.driver()?;
        match input {
            AdcInputRef::Channel(ch) => {
                if (*ch as usize) < driver.adc.input_pins.len() {
                    Ok(*ch)
                } else {
                    Err(format!("ADC channel {ch} out of range for {}", driver.name))
                }
            }
            AdcInputRef::Pin(pin) => driver.adc.pin_to_channel(pin),
        }
    }

    /// Build gzip-compressed JSON metadata compatible with `hmkconf`.
    pub fn compressed_metadata(&self) -> Result<Vec<u8>, String> {
        let default_keymaps = self.resolve_default_keymaps()?;
        let meta = serde_json::json!({
            "name": self.name,
            "vendorId": self.usb.vid,
            "productId": self.usb.pid,
            "usbHighSpeed": self.usb.port == KeyboardUsbPort::HighSpeed,
            "adcResolution": self.adc_resolution()?,
            "numProfiles": self.keyboard.num_profiles,
            "numLayers": self.keyboard.num_layers,
            "numKeys": self.keyboard.num_keys,
            "numAdvancedKeys": self.keyboard.num_advanced_keys,
            "numDynamicKeystrokeMaxBindings": self.keyboard.num_dynamic_keystroke_max_bindings,
            "numMacroNodes": self.keyboard.num_macro_nodes,
            "layout": self.layout,
            "defaultKeymaps": default_keymaps,
        });
        let json_bytes =
            serde_json::to_vec(&meta).map_err(|e| format!("Metadata serialization failed: {e}"))?;
        let mut enc = GzEncoder::new(Vec::new(), Compression::best());
        enc.write_all(&json_bytes)
            .map_err(|e| format!("Gzip compression failed: {e}"))?;
        enc.finish()
            .map_err(|e| format!("Gzip finish failed: {e}"))
    }

    /// Generate `keyboard_gen.rs` to be `include!`-ed by a firmware crate.
    pub fn generate_rust(&self) -> Result<String, String> {
        let driver = self.driver()?;
        let vid = self.usb_vid()?;
        let pid = self.usb_pid()?;
        let adc_res = self.adc_resolution()?;
        let wl_virtual = self.wl_virtual_size();
        let wl_log = self.wl_write_log_size();
        let wl_base = self.wl_base_address()?;
        let metadata = self.compressed_metadata()?;
        let keymaps = self.resolve_default_keymaps()?;

        let mut out = String::new();
        writeln!(out, "// AUTO-GENERATED by libhmk-config from keyboard.json — DO NOT EDIT.").unwrap();
        writeln!(out, "pub const USB_MANUFACTURER_NAME: &str = {:?};", self.manufacturer).unwrap();
        writeln!(out, "pub const USB_PRODUCT_NAME: &str = {:?};", self.name).unwrap();
        writeln!(out, "pub const USB_VENDOR_ID: u16 = 0x{vid:04X};").unwrap();
        writeln!(out, "pub const USB_PRODUCT_ID: u16 = 0x{pid:04X};").unwrap();
        writeln!(
            out,
            "pub const USB_HIGH_SPEED: bool = {};",
            self.usb.port == KeyboardUsbPort::HighSpeed
        )
        .unwrap();
        writeln!(out, "pub const BOARD_HSE_VALUE: u32 = {};", self.hardware.hse_value).unwrap();
        writeln!(out, "pub const BOOTLOADER_ADDR: u32 = 0x{:08X};", driver.bootloader.address).unwrap();
        writeln!(out, "pub const BOOTLOADER_MAGIC: u32 = 0x{:08X};", driver.bootloader.magic).unwrap();

        writeln!(out, "pub const FLASH_BASE: u32 = 0x{:08X};", driver.flash.base_addr).unwrap();
        writeln!(out, "pub const FLASH_SIZE: u32 = {};", driver.flash.total_size()).unwrap();
        writeln!(out, "pub const FLASH_NUM_SECTORS: u32 = {};", driver.flash.num_sectors()).unwrap();
        writeln!(out, "pub const FLASH_EMPTY_VAL: u32 = 0x{:08X};", driver.flash.empty_value).unwrap();
        match driver.flash.sectors {
            FlashSectors::Uniform { size, .. } => {
                writeln!(out, "pub const FLASH_SECTOR_SIZE: u32 = {size};").unwrap();
            }
            FlashSectors::NonUniform { sizes } => {
                writeln!(out, "pub const FLASH_SECTOR_SIZES: &[u32] = &{sizes:?};").unwrap();
            }
        }

        writeln!(out, "pub const WL_VIRTUAL_SIZE: usize = {wl_virtual};").unwrap();
        writeln!(out, "pub const WL_WRITE_LOG_SIZE: usize = {wl_log};").unwrap();
        writeln!(out, "pub const WL_BASE_ADDRESS: u32 = 0x{wl_base:08X};").unwrap();

        let kb = &self.keyboard;
        writeln!(out, "pub const NUM_PROFILES: usize = {};", kb.num_profiles).unwrap();
        writeln!(out, "pub const NUM_LAYERS: usize = {};", kb.num_layers).unwrap();
        writeln!(out, "pub const NUM_KEYS: usize = {};", kb.num_keys).unwrap();
        writeln!(out, "pub const NUM_ADVANCED_KEYS: usize = {};", kb.num_advanced_keys).unwrap();
        writeln!(
            out,
            "pub const NUM_DKS_MAX_BINDINGS: usize = {};",
            kb.num_dynamic_keystroke_max_bindings
        )
        .unwrap();
        writeln!(out, "pub const NUM_MACRO_NODES: usize = {};", kb.num_macro_nodes).unwrap();

        writeln!(out, "pub const ADC_RESOLUTION: u8 = {adc_res};").unwrap();
        writeln!(out, "pub const MATRIX_INVERT_ADC_VALUES: bool = {};", self.analog.invert_adc).unwrap();
        writeln!(
            out,
            "pub const ADC_SAMPLE_DELAY_US: u32 = {};",
            self.analog.delay.unwrap_or(20)
        )
        .unwrap();
        writeln!(
            out,
            "pub const DEFAULT_INITIAL_REST_VALUE: u16 = {};",
            self.calibration.initial_rest_value
        )
        .unwrap();
        writeln!(
            out,
            "pub const DEFAULT_INITIAL_BOTTOM_OUT_THRESHOLD: u16 = {};",
            self.calibration.initial_bottom_out_threshold
        )
        .unwrap();
        writeln!(
            out,
            "pub const DEFAULT_ACTUATION_POINT: u8 = {};",
            self.actuation
                .as_ref()
                .map(|a| a.actuation_point)
                .unwrap_or(128)
        )
        .unwrap();

        // Raw ADC inputs
        if let Some(ref raw) = self.analog.raw {
            let channels: Vec<u8> = raw
                .input
                .iter()
                .map(|x| self.resolve_adc_channel(x))
                .collect::<Result<_, _>>()?;
            writeln!(out, "pub const ADC_RAW_INPUT_CHANNELS: &[u8] = &{channels:?};").unwrap();
            writeln!(out, "pub const ADC_RAW_INPUT_VECTOR: &[u16] = &{:?};", raw.vector).unwrap();
        } else {
            writeln!(out, "pub const ADC_RAW_INPUT_CHANNELS: &[u8] = &[];").unwrap();
            writeln!(out, "pub const ADC_RAW_INPUT_VECTOR: &[u16] = &[];").unwrap();
        }

        // Mux ADC inputs (transposed matrix: [mux_channel][adc_input], matching C scripts/make.py)
        if let Some(ref mux) = self.analog.mux {
            let channels: Vec<u8> = mux
                .input
                .iter()
                .map(|x| self.resolve_adc_channel(x))
                .collect::<Result<_, _>>()?;
            let num_mux_ch = 1usize << mux.select.len();
            let mut transposed: Vec<Vec<u16>> = vec![vec![0; mux.input.len()]; num_mux_ch];
            for (in_idx, row) in mux.matrix.iter().enumerate() {
                for (ch_idx, &key) in row.iter().enumerate() {
                    if ch_idx < num_mux_ch && in_idx < mux.input.len() {
                        transposed[ch_idx][in_idx] = key;
                    }
                }
            }
            writeln!(out, "pub const ADC_MUX_SELECT_PINS: &[&str] = &{:?};", mux.select).unwrap();
            writeln!(out, "pub const ADC_MUX_INPUT_CHANNELS: &[u8] = &{channels:?};").unwrap();
            writeln!(out, "pub const ADC_MUX_NUM_CHANNELS: usize = {num_mux_ch};").unwrap();
            writeln!(out, "pub const ADC_MUX_INPUT_MATRIX: &[&[u16]] = &[").unwrap();
            for row in &transposed {
                writeln!(out, "    &{row:?},").unwrap();
            }
            writeln!(out, "];").unwrap();
        } else {
            writeln!(out, "pub const ADC_MUX_SELECT_PINS: &[&str] = &[];").unwrap();
            writeln!(out, "pub const ADC_MUX_INPUT_CHANNELS: &[u8] = &[];").unwrap();
            writeln!(out, "pub const ADC_MUX_NUM_CHANNELS: usize = 0;").unwrap();
            writeln!(out, "pub const ADC_MUX_INPUT_MATRIX: &[&[u16]] = &[];").unwrap();
        }

        // Bluetooth settings
        if let Some(ref bt) = self.bluetooth {
            let dev_name = bt.device_name.as_deref().unwrap_or(&self.name);
            let batt_ch: Option<u8> = match &bt.battery_pin {
                Some(pin) => Some(self.resolve_adc_channel(pin)?),
                None => None,
            };
            writeln!(out, "pub const BLUETOOTH_ENABLED: bool = {};", bt.enabled).unwrap();
            writeln!(out, "pub const BLE_DEVICE_NAME: &str = {dev_name:?};").unwrap();
            writeln!(out, "pub const BLE_TX_POWER: i8 = {};", bt.tx_power).unwrap();
            writeln!(out, "pub const BLE_IDLE_TIMEOUT_MS: u32 = {};", bt.idle_timeout_ms).unwrap();
            writeln!(out, "pub const BLE_SLEEP_TIMEOUT_MS: u32 = {};", bt.sleep_timeout_ms).unwrap();
            writeln!(out, "pub const BLE_BATTERY_ADC_CHANNEL: Option<u8> = {batt_ch:?};").unwrap();
            writeln!(
                out,
                "pub const BLE_BATTERY_DIVIDER_MILLIS: u32 = {};",
                (bt.battery_divider_ratio * 1000.0).round() as u32
            )
            .unwrap();
        } else {
            writeln!(out, "pub const BLUETOOTH_ENABLED: bool = false;").unwrap();
            writeln!(out, "pub const BLE_DEVICE_NAME: &str = {:?};", self.name).unwrap();
            writeln!(out, "pub const BLE_TX_POWER: i8 = 0;").unwrap();
            writeln!(out, "pub const BLE_IDLE_TIMEOUT_MS: u32 = 30000;").unwrap();
            writeln!(out, "pub const BLE_SLEEP_TIMEOUT_MS: u32 = 300000;").unwrap();
            writeln!(out, "pub const BLE_BATTERY_ADC_CHANNEL: Option<u8> = None;").unwrap();
            writeln!(out, "pub const BLE_BATTERY_DIVIDER_MILLIS: u32 = 1000;").unwrap();
        }

        // Default keymaps as byte arrays resolved via libhmk_core::keycodes::parse_keycode_expr
        writeln!(
            out,
            "pub const DEFAULT_KEYMAPS: [[[u8; NUM_KEYS]; NUM_LAYERS]; NUM_PROFILES] = ["
        )
        .unwrap();
        for profile in &keymaps {
            writeln!(out, "    [").unwrap();
            for layer in profile {
                write!(out, "        [").unwrap();
                for kc in layer {
                    let byte = resolve_keycode_name(kc)?;
                    write!(out, "0x{byte:02X}, ").unwrap();
                }
                writeln!(out, "],").unwrap();
            }
            writeln!(out, "    ],").unwrap();
        }
        writeln!(out, "];").unwrap();

        // Gzip-compressed keyboard metadata for hmkconf
        writeln!(out, "pub const KEYBOARD_METADATA: &[u8] = &{metadata:?};").unwrap();

        Ok(out)
    }

    /// Generate `memory.x` for `cortex-m-rt` with Flash reserved for wear leveling.
    pub fn generate_memory_x(&self) -> Result<String, String> {
        let driver = self.driver()?;
        let wl_base = self.wl_base_address()?;
        let flash_total = driver.flash.total_size();
        Ok(format!(
            "/* Auto-generated by libhmk-config for {} */\n\
             MEMORY\n\
             {{\n\
               FLASH    : ORIGIN = 0x{:08X}, LENGTH = 0x{:08X}\n\
               FLASH_WL : ORIGIN = 0x{:08X}, LENGTH = 0x{:08X}\n\
               RAM      : ORIGIN = 0x{:08X}, LENGTH = 0x{:08X}\n\
             }}\n",
            driver.name,
            driver.flash.base_addr,
            wl_base,
            driver.flash.base_addr + wl_base,
            flash_total - wl_base,
            driver.ram_origin,
            driver.ram_size,
        ))
    }
}

fn parse_hex_u16(s: &str) -> Result<u16, String> {
    let trimmed = s
        .strip_prefix("0x")
        .or_else(|| s.strip_prefix("0X"))
        .ok_or_else(|| format!("Expected 0x-prefixed hex u16, got '{s}'"))?;
    u16::from_str_radix(trimmed, 16).map_err(|e| format!("Invalid hex u16 '{s}': {e}"))
}

/// Resolve a `keyboard.json` keycode name or macro (`MO(1)`, `PF(2)`) into its `u8` keycode byte.
pub fn resolve_keycode_name(name: &str) -> Result<u8, String> {
    if let Some(inner) = name.strip_prefix("MO(").and_then(|s| s.strip_suffix(')')) {
        let layer: u8 = inner
            .trim()
            .parse()
            .map_err(|_| format!("Invalid MO layer: '{name}'"))?;
        return Ok(0xC0 | (layer & 0x07));
    }
    if let Some(inner) = name.strip_prefix("PF(").and_then(|s| s.strip_suffix(')')) {
        let profile: u8 = inner
            .trim()
            .parse()
            .map_err(|_| format!("Invalid PF profile: '{name}'"))?;
        return Ok(0xC8 | (profile & 0x07));
    }

    let code = match name {
        "KC_NO" | "XXXXXXX" => 0x00,
        "KC_TRANSPARENT" | "_______" | "KC_TRNS" => 0x01,
        "KC_A" => 0x02,
        "KC_B" => 0x03,
        "KC_C" => 0x04,
        "KC_D" => 0x05,
        "KC_E" => 0x06,
        "KC_F" => 0x07,
        "KC_G" => 0x08,
        "KC_H" => 0x09,
        "KC_I" => 0x0A,
        "KC_J" => 0x0B,
        "KC_K" => 0x0C,
        "KC_L" => 0x0D,
        "KC_M" => 0x0E,
        "KC_N" => 0x0F,
        "KC_O" => 0x10,
        "KC_P" => 0x11,
        "KC_Q" => 0x12,
        "KC_R" => 0x13,
        "KC_S" => 0x14,
        "KC_T" => 0x15,
        "KC_U" => 0x16,
        "KC_V" => 0x17,
        "KC_W" => 0x18,
        "KC_X" => 0x19,
        "KC_Y" => 0x1A,
        "KC_Z" => 0x1B,
        "KC_1" => 0x1C,
        "KC_2" => 0x1D,
        "KC_3" => 0x1E,
        "KC_4" => 0x1F,
        "KC_5" => 0x20,
        "KC_6" => 0x21,
        "KC_7" => 0x22,
        "KC_8" => 0x23,
        "KC_9" => 0x24,
        "KC_0" => 0x25,
        "KC_ENTER" | "KC_ENT" => 0x26,
        "KC_ESCAPE" | "KC_ESC" => 0x27,
        "KC_BACKSPACE" | "KC_BSPC" => 0x28,
        "KC_TAB" => 0x29,
        "KC_SPACE" | "KC_SPC" => 0x2A,
        "KC_MINUS" | "KC_MINS" => 0x2B,
        "KC_EQUAL" | "KC_EQL" => 0x2C,
        "KC_LEFT_BRACKET" | "KC_LBRC" => 0x2D,
        "KC_RIGHT_BRACKET" | "KC_RBRC" => 0x2E,
        "KC_BACKSLASH" | "KC_BSLS" => 0x2F,
        "KC_NONUS_HASH" | "KC_NUHS" => 0x30,
        "KC_SEMICOLON" | "KC_SCLN" => 0x31,
        "KC_QUOTE" | "KC_QUOT" => 0x32,
        "KC_GRAVE" | "KC_GRV" => 0x33,
        "KC_COMMA" | "KC_COMM" => 0x34,
        "KC_DOT" => 0x35,
        "KC_SLASH" | "KC_SLSH" => 0x36,
        "KC_CAPS_LOCK" | "KC_CAPS" => 0x37,
        "KC_F1" => 0x38,
        "KC_F2" => 0x39,
        "KC_F3" => 0x3A,
        "KC_F4" => 0x3B,
        "KC_F5" => 0x3C,
        "KC_F6" => 0x3D,
        "KC_F7" => 0x3E,
        "KC_F8" => 0x3F,
        "KC_F9" => 0x40,
        "KC_F10" => 0x41,
        "KC_F11" => 0x42,
        "KC_F12" => 0x43,
        "KC_PRINT_SCREEN" | "KC_PSCR" => 0x44,
        "KC_SCROLL_LOCK" | "KC_SCRL" | "KC_BRMD" => 0x45,
        "KC_PAUSE" | "KC_PAUS" | "KC_BRK" | "KC_BRMU" => 0x46,
        "KC_INSERT" | "KC_INS" => 0x47,
        "KC_HOME" => 0x48,
        "KC_PAGE_UP" | "KC_PGUP" => 0x49,
        "KC_DELETE" | "KC_DEL" => 0x4A,
        "KC_END" => 0x4B,
        "KC_PAGE_DOWN" | "KC_PGDN" => 0x4C,
        "KC_RIGHT" | "KC_RGHT" => 0x4D,
        "KC_LEFT" => 0x4E,
        "KC_DOWN" => 0x4F,
        "KC_UP" => 0x50,
        "KC_NUM_LOCK" | "KC_NUM" => 0x51,
        "KC_KP_SLASH" | "KC_PSLS" => 0x52,
        "KC_KP_ASTERISK" | "KC_PAST" => 0x53,
        "KC_KP_MINUS" | "KC_PMNS" => 0x54,
        "KC_KP_PLUS" | "KC_PPLS" => 0x55,
        "KC_KP_ENTER" | "KC_PENT" => 0x56,
        "KC_KP_1" | "KC_P1" => 0x57,
        "KC_KP_2" | "KC_P2" => 0x58,
        "KC_KP_3" | "KC_P3" => 0x59,
        "KC_KP_4" | "KC_P4" => 0x5A,
        "KC_KP_5" | "KC_P5" => 0x5B,
        "KC_KP_6" | "KC_P6" => 0x5C,
        "KC_KP_7" | "KC_P7" => 0x5D,
        "KC_KP_8" | "KC_P8" => 0x5E,
        "KC_KP_9" | "KC_P9" => 0x5F,
        "KC_KP_0" | "KC_P0" => 0x60,
        "KC_KP_DOT" | "KC_PDOT" => 0x61,
        "KC_NONUS_BACKSLASH" | "KC_NUBS" => 0x62,
        "KC_APPLICATION" | "KC_APP" => 0x63,
        "KC_F13" => 0x64,
        "KC_F14" => 0x65,
        "KC_F15" => 0x66,
        "KC_F16" => 0x67,
        "KC_F17" => 0x68,
        "KC_F18" => 0x69,
        "KC_F19" => 0x6A,
        "KC_F20" => 0x6B,
        "KC_F21" => 0x6C,
        "KC_F22" => 0x6D,
        "KC_F23" => 0x6E,
        "KC_F24" => 0x6F,
        "KC_INTERNATIONAL_1" | "KC_INT1" => 0x70,
        "KC_INTERNATIONAL_2" | "KC_INT2" => 0x71,
        "KC_INTERNATIONAL_3" | "KC_INT3" => 0x72,
        "KC_INTERNATIONAL_4" | "KC_INT4" => 0x73,
        "KC_INTERNATIONAL_5" | "KC_INT5" => 0x74,
        "KC_INTERNATIONAL_6" | "KC_INT6" => 0x75,
        "KC_LANGUAGE_1" | "KC_LNG1" => 0x76,
        "KC_LANGUAGE_2" | "KC_LNG2" => 0x77,
        "KC_LANGUAGE_3" | "KC_LNG3" => 0x78,
        "KC_LANGUAGE_4" | "KC_LNG4" => 0x79,
        "KC_LANGUAGE_5" | "KC_LNG5" => 0x7A,
        "KC_LEFT_CTRL" | "KC_LCTL" => 0x7B,
        "KC_LEFT_SHIFT" | "KC_LSFT" => 0x7C,
        "KC_LEFT_ALT" | "KC_LALT" | "KC_LOPT" => 0x7D,
        "KC_LEFT_GUI" | "KC_LGUI" | "KC_LCMD" | "KC_LWIN" => 0x7E,
        "KC_RIGHT_CTRL" | "KC_RCTL" => 0x7F,
        "KC_RIGHT_SHIFT" | "KC_RSFT" => 0x80,
        "KC_RIGHT_ALT" | "KC_RALT" | "KC_ROPT" | "KC_ALGR" => 0x81,
        "KC_RIGHT_GUI" | "KC_RGUI" | "KC_RCMD" | "KC_RWIN" => 0x82,
        "KC_SYSTEM_POWER" | "KC_PWR" => 0x83,
        "KC_SYSTEM_SLEEP" | "KC_SLEP" => 0x84,
        "KC_SYSTEM_WAKE" | "KC_WAKE" => 0x85,
        "KC_AUDIO_MUTE" | "KC_MUTE" => 0x86,
        "KC_AUDIO_VOL_UP" | "KC_VOLU" => 0x87,
        "KC_AUDIO_VOL_DOWN" | "KC_VOLD" => 0x88,
        "KC_MEDIA_NEXT_TRACK" | "KC_MNXT" => 0x89,
        "KC_MEDIA_PREV_TRACK" | "KC_MPRV" => 0x8A,
        "KC_MEDIA_STOP" | "KC_MSTP" => 0x8B,
        "KC_MEDIA_PLAY_PAUSE" | "KC_MPLY" => 0x8C,
        "KC_MEDIA_SELECT" | "KC_MSEL" => 0x8D,
        "KC_MAIL" => 0x8E,
        "KC_CALCULATOR" | "KC_CALC" => 0x8F,
        "KC_MY_COMPUTER" | "KC_MYCM" => 0x90,
        "KC_WWW_SEARCH" | "KC_WSCH" => 0x91,
        "KC_WWW_HOME" | "KC_WHOM" => 0x92,
        "KC_WWW_BACK" | "KC_WBAK" => 0x93,
        "KC_WWW_FORWARD" | "KC_WFWD" => 0x94,
        "KC_WWW_REFRESH" | "KC_WREF" => 0x95,
        "KC_WWW_FAVORITES" | "KC_WFAV" => 0x96,
        "KC_BRIGHTNESS_UP" | "KC_BRIU" => 0x97,
        "KC_BRIGHTNESS_DOWN" | "KC_BRID" => 0x98,
        "KC_MISSION_CONTROL" | "KC_MCTL" => 0x99,
        "KC_LAUNCHPAD" | "KC_LPAD" => 0x9A,
        "SP_MOUSE_BUTTON_1" | "MS_BTN1" => 0x9B,
        "SP_MOUSE_BUTTON_2" | "MS_BTN2" => 0x9C,
        "SP_MOUSE_BUTTON_3" | "MS_BTN3" => 0x9D,
        "SP_MOUSE_BUTTON_4" | "MS_BTN4" => 0x9E,
        "SP_MOUSE_BUTTON_5" | "MS_BTN5" => 0x9F,
        "SP_KEY_LOCK" | "KY_LOCK" => 0xD0,
        "SP_LAYER_LOCK" | "LY_LOCK" => 0xD1,
        "SP_PROFILE_SWAP" | "PF_SWAP" => 0xD2,
        "SP_PROFILE_NEXT" | "PF_NEXT" => 0xD3,
        "SP_BOOT" => 0xD4,
        "SP_BT_TOGGLE" | "BT_TOGG" => 0xD5,
        "SP_BT_USB" | "BT_USB" => 0xD6,
        "SP_BT_BLE" | "BT_BLE" => 0xD7,
        "SP_BT_PAIR" | "BT_PAIR" => 0xD8,
        other => return Err(format!("Unknown keycode: '{other}'")),
    };
    Ok(code)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    #[test]
    fn validate_all_repository_keyboards() {
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../keyboards");
        for kb in ["he16", "he60", "he60-v2", "m256-whe", "nrf52840-dk-he16"] {
            let path = root.join(kb).join("keyboard.json");
            let cfg = KeyboardConfig::from_file(&path)
                .unwrap_or_else(|e| panic!("Failed to validate {kb}: {e}"));
            let gen_rs = cfg
                .generate_rust()
                .unwrap_or_else(|e| panic!("Codegen failed for {kb}: {e}"));
            assert!(gen_rs.contains("pub const DEFAULT_KEYMAPS:"));
            assert!(gen_rs.contains("pub const KEYBOARD_METADATA:"));
            let mem_x = cfg
                .generate_memory_x()
                .unwrap_or_else(|e| panic!("memory.x failed for {kb}: {e}"));
            assert!(mem_x.contains("FLASH_WL"));
        }
    }
}
