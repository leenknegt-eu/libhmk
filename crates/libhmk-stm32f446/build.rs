use std::env;
use std::fs;
use std::path::PathBuf;

use libhmk_config::KeyboardConfig;

const PROBE_EXAMPLE: &str = "B5";

fn main() {
    let keyboard = env::var("LIBHMK_KEYBOARD").unwrap_or_else(|_| "he60".to_string());
    println!("cargo:rerun-if-env-changed=LIBHMK_KEYBOARD");

    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let kb_path = manifest_dir
        .join("../../keyboards")
        .join(&keyboard)
        .join("keyboard.json");
    println!("cargo:rerun-if-changed={}", kb_path.display());

    let cfg = KeyboardConfig::from_file(&kb_path).expect("Invalid keyboard.json");
    assert_eq!(
        cfg.hardware.driver, "stm32f446xx",
        "libhmk-stm32f446 requires a keyboard with driver 'stm32f446xx', got '{}'",
        cfg.hardware.driver
    );

    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
    fs::write(
        out_dir.join("keyboard_gen.rs"),
        cfg.generate_rust().expect("Failed to generate keyboard_gen.rs"),
    )
    .expect("Failed to write keyboard_gen.rs");

    fs::write(
        out_dir.join("memory.x"),
        cfg.generate_memory_x().expect("Failed to generate memory.x"),
    )
    .expect("Failed to write memory.x");

    // Peripheral bindings for the ADC/mux pins declared in keyboard.json
    let pins = cfg.adc_pins().expect("Invalid analog pin configuration");
    let periph = |name: &str| format!("P{name}");
    let list = |names: &[String], suffix: &str| {
        names
            .iter()
            .map(|n| format!("$p.{}{suffix}", periph(n)))
            .collect::<Vec<_>>()
            .join(", ")
    };
    let gen = format!(
        "/// Mux select pins (LSB first) as `[Peri<'static, AnyPin>; N]`.\n\
         macro_rules! adc_mux_select_pins {{ ($p:ident) => {{ [{}] }}; }}\n\
         /// Multiplexed ADC inputs as `[AnyAdcChannel<'static, ADC1>; N]`.\n\
         macro_rules! adc_mux_input_channels {{ ($p:ident) => {{ [{}] }}; }}\n\
         /// Direct (non-multiplexed) ADC inputs as `[AnyAdcChannel<'static, ADC1>; N]`.\n\
         macro_rules! adc_raw_input_channels {{ ($p:ident) => {{ [{}] }}; }}\n",
        list(&pins.mux_select, ".into()"),
        list(&pins.mux_input, ".degrade_adc()"),
        list(&pins.raw_input, ".degrade_adc()"),
    );
    let mut gen = gen;
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
            Some(n) => format!("Some($p.P{n}.into())"),
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
