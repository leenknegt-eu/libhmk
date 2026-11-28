//! Nordic nRF52840 Hall-effect keyboard firmware: USB, 2.4 GHz dongle link (Enhanced ShockBurst,
//! primary wireless) and optional Bluetooth LE (TrouBLE + SoftDevice Controller).
//!
//! The wireless backend is chosen at boot from the stored link settings: both stacks need the
//! RADIO and TIMER0, so only one runs at a time and switching (`BT_24G` / `BT_BLE`) reboots.

#![no_std]
#![no_main]

#[macro_use]
mod analog;
mod ble;
mod board;
mod flash;
mod probe;
mod rf;
mod usb;
mod wireless;

pub mod kb {
    include!(concat!(env!("OUT_DIR"), "/keyboard_gen.rs"));
    include!(concat!(env!("OUT_DIR"), "/link_gen.rs"));
}

use defmt::{info, unwrap};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_nrf::mode::Async;
use embassy_nrf::nvmc::Nvmc;
use embassy_nrf::peripherals::{RNG, USBD};
use embassy_nrf::usb::vbus_detect::SoftwareVbusDetect;
use embassy_nrf::interrupt;
use embassy_nrf::interrupt::typelevel::{self as irq, Binding, Handler};
use embassy_nrf::{bind_interrupts, pac, rng, saadc, usb as nrf_usb};
use embassy_futures::select::select;
use embassy_futures::yield_now;
use embassy_time::Timer;
use libhmk_core::eeconfig::{EeconfigDefaults, EeconfigLayout};
use libhmk_core::matrix::MATRIX_CALIBRATION_DURATION_MS;
use libhmk_core::traits::CommandTransport;
use libhmk_core::types::Calibration;
use libhmk_core::Firmware;
use libhmk_link::{Backend, LinkSettings};
use nrf_sdc::mpsl::MultiprotocolServiceLayer;
use nrf_sdc::{self as sdc, mpsl};
use panic_probe as _;
use static_cell::StaticCell;
use trouble_host::prelude::DefaultPacketPool;
use trouble_host::PacketPool;

use crate::analog::{make_select_output, run_scanner, AdcInputs, SettleTimer, SharedAnalogSamples, SCAN_DONE};
use crate::ble::{run_ble, BLE_HID_TX, BLE_RAW_TX};
use crate::board::NrfBoard;
use crate::flash::NrfFlash;
use crate::probe::Probe;
use crate::rf::RF_RAW_TX;
use crate::usb::{usb_task, CMD_RX, USB_HID_TX, USB_RAW_TX};
use crate::wireless::WirelessController;

bind_interrupts!(pub struct Irqs {
    RNG => rng::InterruptHandler<RNG>;
    SAADC => saadc::InterruptHandler;
    USBD => nrf_usb::InterruptHandler<USBD>;
    EGU0_SWI0 => nrf_sdc::mpsl::LowPrioInterruptHandler;
    CLOCK_POWER => nrf_sdc::mpsl::ClockInterruptHandler;
    RTC0 => nrf_sdc::mpsl::HighPrioInterruptHandler;
});

// RADIO and TIMER0 are shared by the two radio stacks: the vectors below dispatch to the ESB
// driver or to MPSL, whichever was started at boot.
// SAFETY: `RADIO` / `TIMER0` call `HighPrioInterruptHandler::on_interrupt` whenever MPSL owns
// the radio (the ESB driver is not active).
unsafe impl Binding<irq::RADIO, mpsl::HighPrioInterruptHandler> for Irqs {}
unsafe impl Binding<irq::TIMER0, mpsl::HighPrioInterruptHandler> for Irqs {}

#[interrupt]
unsafe fn RADIO() {
    if libhmk_nrf_esb::is_active() {
        libhmk_nrf_esb::on_radio_interrupt();
    } else {
        <mpsl::HighPrioInterruptHandler as Handler<irq::RADIO>>::on_interrupt();
    }
}

#[interrupt]
unsafe fn TIMER0() {
    if libhmk_nrf_esb::is_active() {
        libhmk_nrf_esb::on_timer_interrupt();
    } else {
        <mpsl::HighPrioInterruptHandler as Handler<irq::TIMER0>>::on_interrupt();
    }
}

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
    saadc: embassy_nrf::Peri<'static, embassy_nrf::peripherals::SAADC>,
    inputs: AdcInputs,
    select_pins: [embassy_nrf::gpio::Output<'static>; kb::ADC_MUX_SELECT_PINS.len()],
    settle_timer: SettleTimer,
    probe: Probe,
) -> ! {
    run_scanner(saadc, inputs, select_pins, settle_timer, probe, &SAMPLES).await
}

#[embassy_executor::task]
async fn ble_task(sdc: sdc::SoftdeviceController<'static>) -> ! {
    run_ble(sdc, &SAMPLES).await
}

#[embassy_executor::task]
async fn rf_task(ptx: libhmk_nrf_esb::Ptx, paired: bool) -> ! {
    rf::run_ptx(ptx, paired, &SAMPLES).await
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

    // 1. Link settings decide which wireless stack owns the radio for this boot.
    let mut flash = NrfFlash::new(Nvmc::new(p.NVMC));
    let stored = flash.read_link_settings();
    let mut settings = stored.unwrap_or(LinkSettings::new(wireless::default_backend()));
    let backend = if kb::WIRELESS_ENABLED {
        wireless::resolve_backend(settings.backend)
    } else {
        None
    };
    if let Some(b) = backend {
        settings.backend = b;
    }
    wireless::init(settings);
    info!(
        "wireless backend: {}",
        match backend {
            Some(Backend::Esb) => "2.4 GHz (ESB)",
            Some(Backend::Ble) => "Bluetooth LE",
            None => "none (USB only)",
        }
    );

    match backend {
        Some(Backend::Esb) => {
            // 2a. 2.4 GHz dongle link
            let ptx = rf::make_ptx(p.RADIO, p.TIMER0, settings.paired);
            spawner.spawn(unwrap!(rf_task(ptx, settings.paired.is_some())));
        }
        Some(Backend::Ble) => {
            // 2b. Nordic Multiprotocol Service Layer (MPSL) + SoftDevice Controller + TrouBLE
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
            spawner.spawn(unwrap!(mpsl_task(mpsl)));

            static RNG_CELL: StaticCell<rng::Rng<Async>> = StaticCell::new();
            let ble_rng = RNG_CELL.init(rng::Rng::new(p.RNG, Irqs));
            static SDC_MEM: StaticCell<sdc::Mem<4720>> = StaticCell::new();
            let sdc_p = sdc::Peripherals::new(
                p.PPI_CH17, p.PPI_CH18, p.PPI_CH20, p.PPI_CH21, p.PPI_CH22, p.PPI_CH23,
                p.PPI_CH24, p.PPI_CH25, p.PPI_CH26, p.PPI_CH27, p.PPI_CH28, p.PPI_CH29,
            );
            let sdc = unwrap!(build_sdc(
                sdc_p,
                ble_rng,
                mpsl,
                SDC_MEM.init(sdc::Mem::new())
            ));
            spawner.spawn(unwrap!(ble_task(sdc)));
        }
        None => {}
    }

    // 3. Initialize USB Full-Speed (SoftwareVbusDetect polled alongside MPSL)
    static VBUS: StaticCell<SoftwareVbusDetect> = StaticCell::new();
    let vbus = VBUS.init(SoftwareVbusDetect::new(true, true));
    spawner.spawn(unwrap!(vbus_poll_task(vbus)));

    static SERIAL_BUF: StaticCell<[u8; 32]> = StaticCell::new();
    let serial_buf = SERIAL_BUF.init([0u8; 32]);
    let serial_len = NrfBoard::format_serial_hex(serial_buf);
    let serial_str = core::str::from_utf8(&serial_buf[..serial_len]).unwrap_or("00000000");

    let usb_driver = nrf_usb::Driver::new(p.USBD, Irqs, &*vbus);
    spawner.spawn(unwrap!(usb_task(usb_driver, serial_str)));

    // 4. Start asynchronous SAADC matrix + battery voltage scanner (pins from keyboard.json)
    let inputs: AdcInputs = adc_inputs!(p);
    let select_pins: [embassy_nrf::Peri<'static, embassy_nrf::gpio::AnyPin>; kb::ADC_MUX_SELECT_PINS.len()] =
        adc_mux_select_pins!(p);
    let settle_timer = SettleTimer::new(p.TIMER1);
    // Optional timing probes (`scan-probe` feature, see docs/performance.md)
    #[cfg(feature = "scan-probe")]
    let (scan_probe, mut loop_probe) = (Probe::new(probe_scan_pin!(p)), Probe::new(probe_loop_pin!(p)));
    #[cfg(not(feature = "scan-probe"))]
    let (scan_probe, mut loop_probe) = (Probe::disabled(), Probe::disabled());
    spawner.spawn(unwrap!(saadc_task(
        p.SAADC,
        inputs,
        select_pins.map(make_select_output),
        settle_timer,
        scan_probe
    )));

    // 5. Initialize libhmk-core Firmware engine
    let mut board = NrfBoard;
    let mut ble_ctrl = WirelessController::new(&SAMPLES, backend);

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
    defmt::assert!(fw.init(&mut flash, &defaults));

    // Wait for one complete matrix pass, then run the initial rest-value calibration
    while SAMPLES.generation() == 0 {
        Timer::after_micros(100).await;
    }
    calibrate(fw, &mut flash, false).await;

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
                CommandTransport::Rf => {
                    let _ = RF_RAW_TX.try_send(resp);
                }
            }
            if fw.commands.recalibrate_requested {
                fw.commands.recalibrate_requested = false;
                calibrate(fw, &mut flash, true).await;
            }
        }

        // Run Hall-effect matrix scan + layout + advanced keys
        loop_probe.toggle();
        let out = fw.step(&SAMPLES, &mut flash, &mut board, &mut ble_ctrl);
        if !out.hid.is_empty() {
            if ble_ctrl.is_usb_active() {
                let _ = USB_HID_TX.try_send(out.hid);
            } else {
                match ble_ctrl.backend() {
                    // State-based link: always hand over, the link task coalesces and retries.
                    Some(Backend::Esb) => rf::push_hid(&out.hid),
                    Some(Backend::Ble) if ble_ctrl.is_wireless_connected() => {
                        let _ = BLE_HID_TX.try_send(out.hid);
                    }
                    _ => {}
                }
            }
        }

        // Persist link settings changes (transport, backend switch, pairing).
        if let Some((s, reboot)) = wireless::take_pending_write() {
            if !flash.write_link_settings(&s) {
                defmt::warn!("failed to store link settings");
            }
            if reboot {
                info!("rebooting to apply wireless settings");
                Timer::after_millis(20).await;
                cortex_m::peripheral::SCB::sys_reset();
            }
        }

        ble_ctrl.update_power_state();
        if ble_ctrl.is_usb_active() {
            // Bus powered: free-running loop like the C superloop.
            yield_now().await;
        } else {
            // Battery: sleep (WFE) until the scanner delivers a new matrix pass or a
            // configurator command arrives, like the C firmware's `sd_app_evt_wait()`.
            select(SCAN_DONE.wait(), CMD_RX.ready_to_receive()).await;
        }
    }
}

type Fw = Firmware<
    { kb::NUM_KEYS },
    { kb::NUM_LAYERS },
    { kb::NUM_ADVANCED_KEYS },
    { kb::NUM_DKS_MAX_BINDINGS },
    { kb::WL_VIRTUAL_SIZE },
    { kb::WL_WRITE_LOG_SIZE },
>;

/// Rest-value calibration: filter samples for `MATRIX_CALIBRATION_DURATION_MS`.
async fn calibrate(fw: &mut Fw, flash: &mut NrfFlash<'_>, reset_bottom_out: bool) {
    fw.matrix.begin_recalibrate(&mut fw.eeconfig, flash, reset_bottom_out);
    let start = embassy_time::Instant::now();
    while start.elapsed().as_millis() < MATRIX_CALIBRATION_DURATION_MS as u64 {
        fw.matrix.calibrate_step(&SAMPLES, &fw.eeconfig);
        yield_now().await;
    }
}
