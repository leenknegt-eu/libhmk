use std::env;
use std::fs;
use std::path::PathBuf;

use libhmk_config::KeyboardConfig;

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

    fs::write(
        out_dir.join("memory.x"),
        cfg.generate_memory_x().expect("Failed to generate memory.x"),
    )
    .expect("Failed to write memory.x");

    println!("cargo:rustc-link-search={}", out_dir.display());
}
