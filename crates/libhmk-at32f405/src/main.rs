//! Artery AT32F405xx Hall-effect keyboard firmware entry point (Embassy executor + PAC drivers).

#![no_std]
#![no_main]

mod analog;
mod board;
mod flash;
mod time_driver;
mod usb;

pub mod kb {
    include!(concat!(env!("OUT_DIR"), "/keyboard_gen.rs"));
}

use at32f4xx_pac::at32f405 as pac;
use defmt::{info, unwrap};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_futures::yield_now;
use libhmk_core::eeconfig::{EeconfigDefaults, EeconfigLayout};
use libhmk_core::matrix::MATRIX_CALIBRATION_DURATION_MS;
use libhmk_core::traits::{CommandTransport, NoopBleControl};
use libhmk_core::types::Calibration;
use libhmk_core::Firmware;
use panic_probe as _;
use static_cell::StaticCell;

use crate::analog::SAMPLES;
use crate::board::At32Board;
use crate::flash::At32Flash;
use crate::usb::{usb_task, At32UsbDriver, USB_CMD_RX, USB_HID_TX, USB_RAW_TX};

type Fw = Firmware<
    { kb::NUM_KEYS },
    { kb::NUM_LAYERS },
    { kb::NUM_ADVANCED_KEYS },
    { kb::NUM_DKS_MAX_BINDINGS },
    { kb::WL_VIRTUAL_SIZE },
    { kb::WL_WRITE_LOG_SIZE },
>;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    At32Board::check_bootloader_jump();

    // SAFETY: single call at reset, before any other peripheral access.
    let dp = unsafe { pac::Peripherals::steal() };
    At32Board::init_clocks(&dp.CRM, &dp.FLASH, &dp.PWC);
    time_driver::init(&dp.CRM);
    info!("libhmk-at32f405 starting ({})", kb::USB_PRODUCT_NAME);

    // Persistent configuration + firmware engine
    let mut flash = At32Flash::new(dp.FLASH);
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
    for (dst, &kc) in flat_km
        .iter_mut()
        .zip(kb::DEFAULT_KEYMAPS.iter().flatten().flatten())
    {
        *dst = kc;
    }

    let defaults = EeconfigDefaults {
        calibration: Calibration {
            initial_rest_value: kb::DEFAULT_INITIAL_REST_VALUE,
            initial_bottom_out_threshold: kb::DEFAULT_INITIAL_BOTTOM_OUT_THRESHOLD,
        },
        actuation_point: kb::DEFAULT_ACTUATION_POINT,
        default_keymaps: flat_km,
    };

    static FW: StaticCell<Fw> = StaticCell::new();
    let fw = FW.init(Firmware::new(
        layout_cfg,
        kb::ADC_RESOLUTION,
        kb::MATRIX_INVERT_ADC_VALUES,
    ));
    defmt::assert!(fw.init(&mut flash, &defaults));

    // USB (HS port: 8 kHz polling, or 1 kHz = bInterval 4 when high polling rate is off)
    At32Board::init_usb_clocks(&dp.CRM);
    let poll_ms = if kb::USB_HIGH_SPEED && !fw.eeconfig.options().high_polling_rate_enabled() {
        4
    } else {
        1
    };
    static EP_OUT_BUF: StaticCell<[u8; 256]> = StaticCell::new();
    static SERIAL_BUF: StaticCell<[u8; 32]> = StaticCell::new();
    let serial_buf = SERIAL_BUF.init([0u8; 32]);
    let serial_len = At32Board::format_serial_hex(serial_buf);
    let serial_str = core::str::from_utf8(&serial_buf[..serial_len]).unwrap_or("00000000");
    let usb_driver = At32UsbDriver::new(EP_OUT_BUF.init([0; 256]));
    usb::enable_interrupt();
    spawner.spawn(unwrap!(usb_task(usb_driver, serial_str, poll_ms)));

    // Interrupt-driven ADC scanner
    analog::init(&dp.CRM);
    while SAMPLES.generation() == 0 {
        yield_now().await;
    }
    calibrate(fw, &mut flash, false).await;

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
            if fw.commands.recalibrate_requested {
                fw.commands.recalibrate_requested = false;
                calibrate(fw, &mut flash, true).await;
            }
        }

        let out = fw.step(&SAMPLES, &mut flash, &mut board, &mut ble);
        if !out.hid.is_empty() {
            let _ = USB_HID_TX.try_send(out.hid);
        }
        // Free-running loop like the C superloop; yield so the USB task gets serviced.
        yield_now().await;
    }
}

/// Rest-value calibration: filter samples for `MATRIX_CALIBRATION_DURATION_MS`.
async fn calibrate(fw: &mut Fw, flash: &mut At32Flash, reset_bottom_out: bool) {
    fw.matrix.begin_recalibrate(&mut fw.eeconfig, flash, reset_bottom_out);
    let start = embassy_time::Instant::now();
    while start.elapsed().as_millis() < MATRIX_CALIBRATION_DURATION_MS as u64 {
        fw.matrix.calibrate_step(&SAMPLES, &fw.eeconfig);
        yield_now().await;
    }
}
