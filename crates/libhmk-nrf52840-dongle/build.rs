use std::env;
use std::fs;
use std::path::PathBuf;

use libhmk_config::KeyboardConfig;

/// nRF52840 Dongle (PCA10059) with the stock Nordic open USB bootloader:
/// MBR at 0x0000_0000..0x0000_1000, bootloader at 0x000E_0000.., MBR uses the first 8 bytes of RAM.
const MEMORY_X: &str = "/* nRF52840 Dongle (PCA10059), Nordic MBR + open USB DFU bootloader */
MEMORY
{
  FLASH : ORIGIN = 0x00001000, LENGTH = 0x000DF000
  RAM   : ORIGIN = 0x20000008, LENGTH = 0x0003FFF8
}
";

fn main() {
    // The dongle is built for a keyboard: it takes its USB identity and 2.4 GHz channel from
    // the same keyboard.json.
    let keyboard = env::var("LIBHMK_KEYBOARD").unwrap_or_else(|_| "nrf52840-dk-he16".to_string());
    println!("cargo:rerun-if-env-changed=LIBHMK_KEYBOARD");

    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let kb_path = manifest_dir
        .join("../../keyboards")
        .join(&keyboard)
        .join("keyboard.json");
    println!("cargo:rerun-if-changed={}", kb_path.display());

    let cfg = KeyboardConfig::from_file(&kb_path).expect("Invalid keyboard.json");
    let esb_enabled = cfg
        .wireless
        .as_ref()
        .is_some_and(|w| w.esb.enabled);
    assert!(
        esb_enabled,
        "keyboard '{keyboard}' has no 2.4 GHz link (add a `wireless.esb` section to keyboard.json)"
    );

    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
    fs::write(
        out_dir.join("keyboard_gen.rs"),
        cfg.generate_rust().expect("Failed to generate keyboard_gen.rs"),
    )
    .expect("Failed to write keyboard_gen.rs");
    fs::write(out_dir.join("memory.x"), MEMORY_X).expect("Failed to write memory.x");
    println!("cargo:rustc-link-search={}", out_dir.display());
}
