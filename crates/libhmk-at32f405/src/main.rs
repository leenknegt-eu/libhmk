//! Artery AT32F405xx bare-metal Hall-effect keyboard firmware entry point.

#![no_std]
#![no_main]

mod analog;
mod board;
mod flash;

pub mod kb {
    include!(concat!(env!("OUT_DIR"), "/keyboard_gen.rs"));
}

use at32f4xx_pac as pac;
use cortex_m_rt::entry;
use defmt::info;
use defmt_rtt as _;
use libhmk_core::eeconfig::{EeconfigDefaults, EeconfigLayout};
use libhmk_core::matrix::MATRIX_CALIBRATION_DURATION_MS;
use libhmk_core::traits::{BoardControl, NoopBleControl};
use libhmk_core::types::Calibration;
use libhmk_core::Firmware;
use panic_probe as _;
use static_cell::StaticCell;

use crate::analog::At32Analog;
use crate::board::{At32Board, SYSCLK_HZ};
use crate::flash::At32Flash;

#[entry]
fn main() -> ! {
    At32Board::check_bootloader_jump();

    let mut cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    // Enable peripheral clocks for GPIOA/C, ADC1, and PWC
    dp.CRM.ahben1().modify(|_, w| {
        w.gpioaen().set_bit().gpiocen().set_bit()
    });
    dp.CRM.apb2en().modify(|_, w| w.adc1en().set_bit());

    At32Board::init_systick(&mut cp.SYST, SYSCLK_HZ);
    info!("libhmk-at32f405 starting ({})", kb::USB_PRODUCT_NAME);

    let mut flash = At32Flash::new(dp.FLASH);
    let mut analog = At32Analog::new(dp.ADC1, dp.GPIOC);
    let mut board = At32Board;
    let mut ble = NoopBleControl;

    let layout_cfg = EeconfigLayout {
        num_profiles: kb::NUM_PROFILES,
        num_layers: kb::NUM_LAYERS,
        num_keys: kb::NUM_KEYS,
        num_advanced_keys: kb::NUM_ADVANCED_KEYS,
        num_dks_max_bindings: kb::NUM_DKS_MAX_BINDINGS,
        num_macro_nodes: kb::NUM_MACRO_NODES,
    };

    static FLAT_KEYMAPS: StaticCell<[u8; kb::NUM_PROFILES * kb::NUM_LAYERS * kb::NUM_KEYS]> =
        StaticCell::new();
    let flat_km = FLAT_KEYMAPS.init([0u8; kb::NUM_PROFILES * kb::NUM_LAYERS * kb::NUM_KEYS]);
    let mut idx = 0;
    for p_map in &kb::DEFAULT_KEYMAPS {
        for l_map in p_map {
            for &kc in l_map {
                flat_km[idx] = kc;
                idx += 1;
            }
        }
    }

    let defaults = EeconfigDefaults {
        calibration: Calibration {
            initial_rest_value: kb::DEFAULT_INITIAL_REST_VALUE,
            initial_bottom_out_threshold: kb::DEFAULT_INITIAL_BOTTOM_OUT_THRESHOLD,
        },
        actuation_point: kb::DEFAULT_ACTUATION_POINT,
        default_keymaps: flat_km,
    };

    static FW: StaticCell<
        Firmware<
            { kb::NUM_KEYS },
            { kb::NUM_LAYERS },
            { kb::NUM_ADVANCED_KEYS },
            { kb::NUM_DKS_MAX_BINDINGS },
            { kb::WL_VIRTUAL_SIZE },
            { kb::WL_WRITE_LOG_SIZE },
        >,
    > = StaticCell::new();
    let fw = FW.init(Firmware::new(
        layout_cfg,
        kb::ADC_RESOLUTION,
        kb::MATRIX_INVERT_ADC_VALUES,
    ));
    assert!(fw.init(&mut flash, &defaults));

    // Initial Hall-effect auto-calibration
    fw.matrix.begin_recalibrate(&mut fw.eeconfig, &mut flash, false);
    let start = board.now_ms();
    while board.now_ms().wrapping_sub(start) < MATRIX_CALIBRATION_DURATION_MS {
        analog.scan_all();
        fw.matrix.calibrate_step(&analog, &fw.eeconfig);
    }

    loop {
        analog.scan_all();
        let _out = fw.step(&analog, &mut flash, &mut board, &mut ble);
    }
}
