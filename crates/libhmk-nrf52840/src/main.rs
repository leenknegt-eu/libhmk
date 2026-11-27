//! Nordic nRF52840 Embassy + TrouBLE BLE + USB Hall-effect keyboard firmware entry point.

#![no_std]
#![no_main]

mod analog;
mod ble;
mod board;
mod flash;
mod usb;

pub mod kb {
    include!(concat!(env!("OUT_DIR"), "/keyboard_gen.rs"));
}

use defmt::{info, unwrap};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_nrf::mode::Async;
use embassy_nrf::nvmc::Nvmc;
use embassy_nrf::peripherals::{RNG, USBD};
use embassy_nrf::saadc::Saadc;
use embassy_nrf::usb::vbus_detect::SoftwareVbusDetect;
use embassy_nrf::{bind_interrupts, pac, rng, saadc, usb as nrf_usb};
use embassy_time::{Duration, Timer};
use libhmk_core::eeconfig::{EeconfigDefaults, EeconfigLayout};
use libhmk_core::matrix::MATRIX_CALIBRATION_DURATION_MS;
use libhmk_core::traits::CommandTransport;
use libhmk_core::types::Calibration;
use libhmk_core::Firmware;
use nrf_sdc::mpsl::MultiprotocolServiceLayer;
use nrf_sdc::{self as sdc, mpsl};
use panic_probe as _;
use static_cell::StaticCell;
use trouble_host::prelude::DefaultPacketPool;

use crate::analog::{
    make_select_output, matrix_channel_config, run_mux_scanner, saadc_config,
    vdd_battery_channel_config, SharedAnalogSamples,
};
use crate::ble::{run_ble, NrfBleController, BLE_HID_TX, BLE_RAW_TX};
use crate::board::NrfBoard;
use crate::flash::NrfFlash;
use crate::usb::{usb_task, CMD_RX, USB_HID_TX, USB_RAW_TX};

bind_interrupts!(struct Irqs {
    RNG => rng::InterruptHandler<RNG>;
    SAADC => saadc::InterruptHandler;
    USBD => nrf_usb::InterruptHandler<USBD>;
    EGU0_SWI0 => nrf_sdc::mpsl::LowPrioInterruptHandler;
    CLOCK_POWER => nrf_sdc::mpsl::ClockInterruptHandler;
    RADIO => nrf_sdc::mpsl::HighPrioInterruptHandler;
    TIMER0 => nrf_sdc::mpsl::HighPrioInterruptHandler;
    RTC0 => nrf_sdc::mpsl::HighPrioInterruptHandler;
});

static SAMPLES: SharedAnalogSamples = SharedAnalogSamples::new();

#[embassy_executor::task]
async fn mpsl_task(mpsl: &'static MultiprotocolServiceLayer<'static>) -> ! {
    mpsl.run().await
}

#[embassy_executor::task]
async fn vbus_poll_task(vbus: &'static SoftwareVbusDetect) -> ! {
    // Poll POWER.USBREGSTATUS so USB works seamlessly while nrf-mpsl owns CLOCK_POWER IRQ
    let mut prev_detected = false;
    let mut prev_ready = false;
    loop {
        let reg = pac::POWER.usbregstatus().read();
        let detected = reg.vbusdetect();
        let ready = reg.outputrdy();
        if detected != prev_detected {
            vbus.detected(detected);
            prev_detected = detected;
        }
        if ready != prev_ready {
            if ready {
                vbus.ready();
            }
            prev_ready = ready;
        }
        Timer::after_millis(20).await;
    }
}

#[embassy_executor::task]
async fn saadc_task(
    saadc: Saadc<'static, 3>,
    select_pins: [embassy_nrf::gpio::Output<'static>; 3],
) -> ! {
    run_mux_scanner(saadc, select_pins, &SAMPLES).await
}

#[embassy_executor::task]
async fn ble_task(sdc: sdc::SoftdeviceController<'static>) -> ! {
    run_ble(sdc, &SAMPLES).await
}

fn build_sdc<'d, const N: usize>(
    p: sdc::Peripherals<'d>,
    rng: &'d mut rng::Rng<Async>,
    mpsl: &'d MultiprotocolServiceLayer<'d>,
    mem: &'d mut sdc::Mem<N>,
) -> Result<sdc::SoftdeviceController<'d>, sdc::Error> {
    sdc::Builder::new()?
        .support_adv()
        .support_peripheral()
        .peripheral_count(1)?
        .buffer_cfg(
            DefaultPacketPool::MTU as u16,
            DefaultPacketPool::MTU as u16,
            3,
            3,
        )?
        .build(p, rng, mpsl, mem)
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());
    info!("libhmk-nrf52840 starting ({})", kb::USB_PRODUCT_NAME);

    // 1. Initialize Nordic Multiprotocol Service Layer (MPSL) for BLE radio timing
    let mpsl_p = mpsl::Peripherals::new(
        p.RTC0, p.TIMER0, p.TEMP, p.PPI_CH19, p.PPI_CH30, p.PPI_CH31,
    );
    let lfclk_cfg = mpsl::raw::mpsl_clock_lfclk_cfg_t {
        source: mpsl::raw::MPSL_CLOCK_LF_SRC_XTAL as u8,
        rc_ctiv: 0,
        rc_temp_ctiv: 0,
        accuracy_ppm: 20,
        skip_wait_lfclk_started: false,
    };
    static MPSL: StaticCell<MultiprotocolServiceLayer<'static>> = StaticCell::new();
    let mpsl = MPSL.init(unwrap!(MultiprotocolServiceLayer::new(
        mpsl_p, Irqs, lfclk_cfg
    )));
    spawner.must_spawn(mpsl_task(mpsl));

    // 2. Initialize Nordic SoftDevice Controller (SDC) + TrouBLE BLE task
    static RNG_CELL: StaticCell<rng::Rng<Async>> = StaticCell::new();
    let ble_rng = RNG_CELL.init(rng::Rng::new(p.RNG, Irqs));
    static SDC_MEM: StaticCell<sdc::Mem<4720>> = StaticCell::new();
    let sdc_p = sdc::Peripherals::new(
        p.PPI_CH17, p.PPI_CH18, p.PPI_CH20, p.PPI_CH21, p.PPI_CH22, p.PPI_CH23, p.PPI_CH24,
        p.PPI_CH25, p.PPI_CH26, p.PPI_CH27, p.PPI_CH28, p.PPI_CH29,
    );
    let sdc = unwrap!(build_sdc(
        sdc_p,
        ble_rng,
        mpsl,
        SDC_MEM.init(sdc::Mem::new())
    ));
    if kb::BLUETOOTH_ENABLED {
        spawner.must_spawn(ble_task(sdc));
    }

    // 3. Initialize USB Full-Speed (SoftwareVbusDetect polled alongside MPSL)
    static VBUS: StaticCell<SoftwareVbusDetect> = StaticCell::new();
    let vbus = VBUS.init(SoftwareVbusDetect::new(true, true));
    spawner.must_spawn(vbus_poll_task(vbus));

    static SERIAL_BUF: StaticCell<[u8; 32]> = StaticCell::new();
    let serial_buf = SERIAL_BUF.init([0u8; 32]);
    let serial_len = NrfBoard::format_serial_hex(serial_buf);
    let serial_str = core::str::from_utf8(&serial_buf[..serial_len]).unwrap_or("00000000");

    let usb_driver = nrf_usb::Driver::new(p.USBD, Irqs, &*vbus);
    spawner.must_spawn(usb_task(usb_driver, serial_str));

    // 4. Start asynchronous SAADC matrix + battery voltage scanner
    let saadc = Saadc::new(
        p.SAADC,
        Irqs,
        saadc_config(),
        [
            matrix_channel_config(p.P0_02),
            matrix_channel_config(p.P0_03),
            vdd_battery_channel_config(),
        ],
    );
    let select_pins = [
        make_select_output(p.P0_11.into()),
        make_select_output(p.P0_12.into()),
        make_select_output(p.P0_13.into()),
    ];
    spawner.must_spawn(saadc_task(saadc, select_pins));

    // 5. Initialize libhmk-core Firmware engine
    let mut flash = NrfFlash::new(Nvmc::new(p.NVMC));
    let mut board = NrfBoard;
    let mut ble_ctrl = NrfBleController::new(&SAMPLES);

    let layout_cfg = EeconfigLayout {
        num_profiles: kb::NUM_PROFILES,
        num_layers: kb::NUM_LAYERS,
        num_keys: kb::NUM_KEYS,
        num_advanced_keys: kb::NUM_ADVANCED_KEYS,
        num_dks_max_bindings: kb::NUM_DKS_MAX_BINDINGS,
        num_macro_nodes: kb::NUM_MACRO_NODES,
    };

    // Flatten DEFAULT_KEYMAPS for EeconfigDefaults
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

    // Initial Hall-effect auto-calibration
    fw.matrix.begin_recalibrate(&mut fw.eeconfig, &mut flash, false);
    let cal_start = embassy_time::Instant::now();
    while cal_start.elapsed().as_millis() < MATRIX_CALIBRATION_DURATION_MS as u64 {
        fw.matrix.calibrate_step(&SAMPLES, &fw.eeconfig);
        Timer::after(Duration::from_micros(500)).await;
    }

    info!("Calibration complete; entering main keyboard loop");

    loop {
        // Process any incoming Raw HID configurator command (USB or BLE)
        if let Ok(cmd) = CMD_RX.try_receive() {
            let resp = fw.handle_command(
                &cmd.data,
                cmd.transport,
                kb::KEYBOARD_METADATA,
                &defaults,
                &mut flash,
                &mut board,
                &mut ble_ctrl,
            );
            match cmd.transport {
                CommandTransport::Usb => {
                    let _ = USB_RAW_TX.try_send(resp);
                }
                CommandTransport::Ble => {
                    let _ = BLE_RAW_TX.try_send(resp);
                }
            }
            if fw.commands.recalibrate_requested {
                fw.commands.recalibrate_requested = false;
                fw.matrix.begin_recalibrate(&mut fw.eeconfig, &mut flash, true);
                let start = embassy_time::Instant::now();
                while start.elapsed().as_millis() < MATRIX_CALIBRATION_DURATION_MS as u64 {
                    fw.matrix.calibrate_step(&SAMPLES, &fw.eeconfig);
                    Timer::after(Duration::from_micros(500)).await;
                }
            }
        }

        // Run Hall-effect matrix scan + layout + advanced keys
        let out = fw.step(&SAMPLES, &mut flash, &mut board, &mut ble_ctrl);
        if !out.hid.is_empty() {
            if ble_ctrl.is_usb_active() {
                let _ = USB_HID_TX.try_send(out.hid);
            } else if ble_ctrl.is_ble_connected() {
                let _ = BLE_HID_TX.try_send(out.hid);
            }
        }

        ble_ctrl.update_power_state();
        Timer::after(Duration::from_micros(250)).await;
    }
}
