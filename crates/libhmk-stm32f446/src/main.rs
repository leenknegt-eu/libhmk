//! STM32F446xx Embassy Hall-effect keyboard firmware entry point.

#![no_std]
#![no_main]

mod analog;
mod board;
mod flash;
mod usb;

pub mod kb {
    include!(concat!(env!("OUT_DIR"), "/keyboard_gen.rs"));
}

use defmt::{info, unwrap};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::adc::{Adc, SampleTime};
use embassy_stm32::flash::Flash;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::rcc::{
    AHBPrescaler, APBPrescaler, Hse, HseMode, Pll, PllMul, PllPDiv, PllPreDiv, PllQDiv,
    PllSource, Sysclk,
};
use embassy_stm32::time::Hertz;
use embassy_stm32::usb::{Config as Stm32UsbConfig, Driver};
use embassy_stm32::{bind_interrupts, peripherals, usb as stm32_usb, Config};
use embassy_time::{Duration, Timer};
use libhmk_core::eeconfig::{EeconfigDefaults, EeconfigLayout};
use libhmk_core::matrix::MATRIX_CALIBRATION_DURATION_MS;
use libhmk_core::traits::{CommandTransport, NoopBleControl};
use libhmk_core::types::Calibration;
use libhmk_core::Firmware;
use panic_probe as _;
use static_cell::StaticCell;

use crate::analog::SharedAnalogSamples;
use crate::board::Stm32Board;
use crate::flash::Stm32Flash;
use crate::usb::{usb_task, USB_CMD_RX, USB_HID_TX, USB_RAW_TX};

bind_interrupts!(struct Irqs {
    OTG_FS => stm32_usb::InterruptHandler<peripherals::USB_OTG_FS>;
});

static SAMPLES: SharedAnalogSamples = SharedAnalogSamples::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    Stm32Board::check_bootloader_jump();

    // Configure 180 MHz SYSCLK + 48 MHz PLLSAI USB clock from HSE
    let mut cfg = Config::default();
    cfg.rcc.hse = Some(Hse {
        freq: Hertz(kb::BOARD_HSE_VALUE),
        mode: HseMode::Oscillator,
    });
    cfg.rcc.pll_src = PllSource::HSE;
    cfg.rcc.pll = Some(Pll {
        prediv: PllPreDiv::DIV8,
        mul: PllMul::MUL360,
        divp: Some(PllPDiv::DIV2),
        divq: Some(PllQDiv::DIV8),
        divr: None,
    });
    cfg.rcc.sys = Sysclk::PLL1_P;
    cfg.rcc.ahb_pre = AHBPrescaler::DIV1;
    cfg.rcc.apb1_pre = APBPrescaler::DIV4;
    cfg.rcc.apb2_pre = APBPrescaler::DIV2;

    let p = embassy_stm32::init(cfg);
    info!("libhmk-stm32f446 starting ({})", kb::USB_PRODUCT_NAME);

    // USB OTG FS
    static EP_OUT_BUF: StaticCell<[u8; 256]> = StaticCell::new();
    let mut usb_cfg = Stm32UsbConfig::default();
    usb_cfg.vbus_detection = false;
    let usb_driver = Driver::new_fs(
        p.USB_OTG_FS,
        Irqs,
        p.PA12,
        p.PA11,
        EP_OUT_BUF.init([0; 256]),
        usb_cfg,
    );

    static SERIAL_BUF: StaticCell<[u8; 32]> = StaticCell::new();
    let serial_buf = SERIAL_BUF.init([0u8; 32]);
    let serial_len = Stm32Board::format_serial_hex(serial_buf);
    let serial_str = core::str::from_utf8(&serial_buf[..serial_len]).unwrap_or("00000000");
    spawner.must_spawn(usb_task(usb_driver, serial_str));

    // Flash & Firmware initialization
    let mut flash = Stm32Flash::new(Flash::new_blocking(p.FLASH));
    let mut board = Stm32Board;
    let mut ble = NoopBleControl;

    let mut adc = Adc::new(p.ADC1);
    adc.set_sample_time(SampleTime::CYCLES3);
    let mut mux_s0 = Output::new(p.PB12, Level::Low, Speed::VeryHigh);
    let mut mux_s1 = Output::new(p.PB13, Level::Low, Speed::VeryHigh);
    let mut mux_s2 = Output::new(p.PB14, Level::Low, Speed::VeryHigh);
    let mut mux_s3 = Output::new(p.PB15, Level::Low, Speed::VeryHigh);
    let mut adc_in0 = p.PA0;
    let mut adc_in1 = p.PA1;
    let mut adc_in2 = p.PA2;
    let mut adc_in3 = p.PA3;

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
    unwrap!(fw.init(&mut flash, &defaults));

    let mut scan_once = |adc: &mut Adc<'_, peripherals::ADC1>| {
        for ch in 0..kb::ADC_MUX_NUM_CHANNELS {
            mux_s0.set_level(if (ch & 1) != 0 { Level::High } else { Level::Low });
            mux_s1.set_level(if (ch & 2) != 0 { Level::High } else { Level::Low });
            mux_s2.set_level(if (ch & 4) != 0 { Level::High } else { Level::Low });
            mux_s3.set_level(if (ch & 8) != 0 { Level::High } else { Level::Low });
            cortex_m::asm::delay(180 * kb::ADC_SAMPLE_DELAY_US);

            let readings = [
                adc.blocking_read(&mut adc_in0),
                adc.blocking_read(&mut adc_in1),
                adc.blocking_read(&mut adc_in2),
                adc.blocking_read(&mut adc_in3),
            ];
            for (i, &val) in readings.iter().enumerate() {
                if i < kb::ADC_MUX_INPUT_MATRIX[ch].len() {
                    let key = kb::ADC_MUX_INPUT_MATRIX[ch][i] as usize;
                    if key != 0 {
                        SAMPLES.store(key - 1, val);
                    }
                }
            }
        }
    };

    fw.matrix.begin_recalibrate(&mut fw.eeconfig, &mut flash, false);
    let cal_start = embassy_time::Instant::now();
    while cal_start.elapsed().as_millis() < MATRIX_CALIBRATION_DURATION_MS as u64 {
        scan_once(&mut adc);
        fw.matrix.calibrate_step(&SAMPLES, &fw.eeconfig);
        Timer::after(Duration::from_micros(200)).await;
    }

    loop {
        if let Ok(req) = USB_CMD_RX.try_receive() {
            let resp = fw.handle_command(
                &req,
                CommandTransport::Usb,
                kb::KEYBOARD_METADATA,
                &defaults,
                &mut flash,
                &mut board,
                &mut ble,
            );
            let _ = USB_RAW_TX.try_send(resp);
        }

        scan_once(&mut adc);
        let out = fw.step(&SAMPLES, &mut flash, &mut board, &mut ble);
        if !out.hid.is_empty() {
            let _ = USB_HID_TX.try_send(out.hid);
        }
        Timer::after(Duration::from_micros(100)).await;
    }
}
