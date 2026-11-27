use std::env;
use std::fs;
use std::path::PathBuf;

use libhmk_config::KeyboardConfig;

fn main() {
    let keyboard = env::var("LIBHMK_KEYBOARD").unwrap_or_else(|_| "he16".to_string());
    println!("cargo:rerun-if-env-changed=LIBHMK_KEYBOARD");

    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let kb_path = manifest_dir
        .join("../../keyboards")
        .join(&keyboard)
        .join("keyboard.json");
    println!("cargo:rerun-if-changed={}", kb_path.display());

    let cfg = KeyboardConfig::from_file(&kb_path).expect("Invalid keyboard.json");
    assert_eq!(
        cfg.hardware.driver, "at32f405xx",
        "libhmk-at32f405 requires a keyboard with driver 'at32f405xx', got '{}'",
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

    // GPIO tables for the ADC/mux pins declared in keyboard.json: (port index, pin number)
    let pins = cfg.adc_pins().expect("Invalid analog pin configuration");
    let gpio = |name: &str| -> (u8, u8) {
        let mut chars = name.chars();
        let port = chars.next().expect("empty pin name");
        let num: u8 = chars.as_str().parse().expect("invalid pin number");
        assert!(num < 16, "invalid pin '{name}'");
        let idx = match port {
            'A' => 0,
            'B' => 1,
            'C' => 2,
            'D' => 3,
            'F' => 5,
            _ => panic!("unsupported GPIO port in '{name}'"),
        };
        (idx, num)
    };
    let table = |names: &[String]| {
        names
            .iter()
            .map(|n| format!("{:?}", gpio(n)))
            .collect::<Vec<_>>()
            .join(", ")
    };
    let gen = format!(
        "/// Mux select pins (LSB first) as `(port, pin)`; port 0 = GPIOA.\n\
         pub const MUX_SELECT_GPIO: [(u8, u8); {}] = [{}];\n\
         /// ADC input pins (mux inputs, then raw inputs) as `(port, pin)`.\n\
         pub const ADC_INPUT_GPIO: [(u8, u8); {}] = [{}];\n",
        pins.mux_select.len(),
        table(&pins.mux_select),
        pins.mux_input.len() + pins.raw_input.len(),
        table(&[pins.mux_input.clone(), pins.raw_input.clone()].concat()),
    );
    fs::write(out_dir.join("pins_gen.rs"), gen).expect("Failed to write pins_gen.rs");

    println!("cargo:rustc-link-search={}", out_dir.display());
}
