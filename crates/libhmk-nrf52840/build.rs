use std::env;
use std::fs;
use std::path::PathBuf;

use libhmk_config::KeyboardConfig;

const PROBE_EXAMPLE: &str = "P0.26";

fn main() {
    let keyboard = env::var("LIBHMK_KEYBOARD").unwrap_or_else(|_| "nrf52840-dk-he16".to_string());
    println!("cargo:rerun-if-env-changed=LIBHMK_KEYBOARD");

    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let kb_path = manifest_dir
        .join("../../keyboards")
        .join(&keyboard)
        .join("keyboard.json");
    println!("cargo:rerun-if-changed={}", kb_path.display());

    let cfg = KeyboardConfig::from_file(&kb_path).expect("Invalid keyboard.json");
    assert_eq!(
        cfg.hardware.driver, "nrf52840xx",
        "libhmk-nrf52840 requires a keyboard with driver 'nrf52840xx', got '{}'",
        cfg.hardware.driver
    );

    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
    fs::write(
        out_dir.join("keyboard_gen.rs"),
        cfg.generate_rust().expect("Failed to generate keyboard_gen.rs"),
    )
    .expect("Failed to write keyboard_gen.rs");

    // One flash page below the wear-leveling area holds the wireless link settings.
    const LINK_SETTINGS_PAGE: u32 = 4096;
    let link_addr = cfg
        .reserved_below_wl(LINK_SETTINGS_PAGE)
        .expect("No room for the link settings page");
    fs::write(
        out_dir.join("memory.x"),
        cfg.generate_memory_x_reserving(LINK_SETTINGS_PAGE)
            .expect("Failed to generate memory.x"),
    )
    .expect("Failed to write memory.x");
    fs::write(
        out_dir.join("link_gen.rs"),
        format!(
            "/// Flash offset of the wireless link settings page.\n\
             pub const LINK_SETTINGS_ADDR: u32 = 0x{link_addr:08X};\n"
        ),
    )
    .expect("Failed to write link_gen.rs");

    // SAADC / multiplexer peripheral bindings generated from keyboard.json
    let pins = cfg.adc_pins().expect("Invalid analog pin configuration");
    // "P0.02" -> "P0_02"
    let periph = |name: &str| name.replace('.', "_");
    let inputs: Vec<String> = pins.mux_input.iter().chain(pins.raw_input.iter()).map(|n| periph(n)).collect();
    let mut gen = String::new();
    gen.push_str(&format!(
        "/// Number of SAADC channels sampled per scan step (mux inputs + raw inputs).\n\
         pub const NUM_SAADC_CHANNELS: usize = {};\n",
        inputs.len()
    ));
    gen.push_str("/// SAADC input pins owned by the scanner task.\npub struct AdcInputs {\n");
    for (i, n) in inputs.iter().enumerate() {
        gen.push_str(&format!("    pub in{i}: Peri<'static, peripherals::{n}>,\n"));
    }
    if let Some(b) = &pins.battery {
        gen.push_str(&format!("    pub battery: Peri<'static, peripherals::{}>,\n", periph(b)));
    }
    gen.push_str("}\n\nimpl AdcInputs {\n");
    gen.push_str("    /// Channel configs for the matrix sequence (mux inputs first, then raw inputs).\n");
    gen.push_str("    pub fn matrix_configs(&mut self) -> [ChannelConfig<'_>; NUM_SAADC_CHANNELS] {\n        [");
    for i in 0..inputs.len() {
        gen.push_str(&format!("matrix_channel_config(self.in{i}.reborrow()), "));
    }
    gen.push_str("]\n    }\n\n");
    gen.push_str("    /// Channel config for the battery voltage measurement.\n");
    gen.push_str("    pub fn battery_config(&mut self) -> ChannelConfig<'_> {\n");
    if pins.battery.is_some() {
        gen.push_str("        battery_channel_config(self.battery.reborrow())\n");
    } else {
        gen.push_str("        battery_channel_config(VddInput)\n");
    }
    gen.push_str("    }\n}\n\n");
    gen.push_str("/// Move the SAADC input pins out of `embassy_nrf::Peripherals`.\n");
    gen.push_str("macro_rules! adc_inputs { ($p:ident) => { $crate::analog::AdcInputs {");
    for (i, n) in inputs.iter().enumerate() {
        gen.push_str(&format!(" in{i}: $p.{n},"));
    }
    if let Some(b) = &pins.battery {
        gen.push_str(&format!(" battery: $p.{},", periph(b)));
    }
    gen.push_str(" } }; }\n");
    gen.push_str("/// Mux select pins (LSB first) as `[Peri<'static, AnyPin>; N]`.\n");
    gen.push_str("macro_rules! adc_mux_select_pins { ($p:ident) => { [");
    for n in &pins.mux_select {
        gen.push_str(&format!("$p.{}.into(), ", periph(n)));
    }
    gen.push_str("] }; }\n");

    // Optional GPIO timing probes (`scan-probe` feature), see docs/performance.md
    println!("cargo:rerun-if-env-changed=LIBHMK_PROBE_SCAN_PIN");
    println!("cargo:rerun-if-env-changed=LIBHMK_PROBE_LOOP_PIN");
    let probe = if env::var_os("CARGO_FEATURE_SCAN_PROBE").is_some() {
        let probe = cfg.probe_pins_from_env().expect("Invalid probe pin");
        assert!(
            probe.scan.is_some(),
            "feature `scan-probe` requires LIBHMK_PROBE_SCAN_PIN (e.g. {})",
            PROBE_EXAMPLE
        );
        probe
    } else {
        if env::var_os("LIBHMK_PROBE_SCAN_PIN").is_some() {
            println!("cargo:warning=LIBHMK_PROBE_SCAN_PIN is ignored without `--features scan-probe`");
        }
        libhmk_config::ProbePins::default()
    };
    if env::var_os("CARGO_FEATURE_SCAN_PROBE").is_some() {
        let pin = |p: &Option<String>| match p {
            Some(n) => format!("Some($p.{}.into())", n.replace('.', "_")),
            None => "None".to_string(),
        };
        gen.push_str(&format!(
            "macro_rules! probe_scan_pin {{ ($p:ident) => {{ {} }}; }}\n\
             macro_rules! probe_loop_pin {{ ($p:ident) => {{ {} }}; }}\n",
            pin(&probe.scan),
            pin(&probe.main_loop)
        ));
    }
    fs::write(out_dir.join("pins_gen.rs"), gen).expect("Failed to write pins_gen.rs");

    println!("cargo:rustc-link-search={}", out_dir.display());
}
