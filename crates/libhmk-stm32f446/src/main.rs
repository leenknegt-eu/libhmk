//! STM32F446xx Embassy Hall-effect keyboard firmware entry point.

#![no_std]
#![no_main]

mod analog;
mod board;
mod flash;
mod probe;
mod usb;

pub mod kb {
    include!(concat!(env!("OUT_DIR"), "/keyboard_gen.rs"));
}

include!(concat!(env!("OUT_DIR"), "/pins_gen.rs"));

use defmt::{info, unwrap};
use defmt_rtt as _;
use embassy_executor::{InterruptExecutor, Spawner};
use embassy_futures::yield_now;
use embassy_stm32::adc::{Adc, AdcChannel};
use embassy_stm32::flash::Flash;
use embassy_stm32::gpio::{AnyPin, Level, Output, Speed};
use embassy_stm32::interrupt;
use embassy_stm32::interrupt::{InterruptExt, Priority};
use embassy_stm32::rcc::{
    AHBPrescaler, APBPrescaler, Hse, HseMode, Pll, PllMul, PllPDiv, PllPreDiv, PllQDiv,
    PllSource, Sysclk,
};
use embassy_stm32::time::Hertz;
use embassy_stm32::usb::{Config as Stm32UsbConfig, Driver};
use embassy_stm32::{bind_interrupts, peripherals, usb as stm32_usb, Config};
use libhmk_core::eeconfig::{EeconfigDefaults, EeconfigLayout};
use libhmk_core::matrix::MATRIX_CALIBRATION_DURATION_MS;
use libhmk_core::traits::{CommandTransport, NoopBleControl};
use libhmk_core::types::Calibration;
use libhmk_core::Firmware;
use panic_probe as _;
use static_cell::StaticCell;

use crate::analog::{run_scanner, MuxInputs, RawInputs, SelectPins, SequenceAdc, SharedAnalogSamples};
use crate::board::Stm32Board;
use crate::flash::Stm32Flash;
use crate::probe::Probe;
use crate::usb::{usb_task, USB_CMD_RX, USB_HID_TX, USB_RAW_TX};

bind_interrupts!(struct Irqs {
    OTG_FS => stm32_usb::InterruptHandler<peripherals::USB_OTG_FS>;
    DMA2_STREAM0 => embassy_stm32::dma::InterruptHandler<peripherals::DMA2_CH0>;
});

static SAMPLES: SharedAnalogSamples = SharedAnalogSamples::new();

/// High-priority executor for the ADC scanner (preempts the main keyboard loop).
static EXECUTOR_ADC: InterruptExecutor = InterruptExecutor::new();

#[interrupt]
unsafe fn UART5() {
    EXECUTOR_ADC.on_interrupt()
}

#[embassy_executor::task]
async fn adc_task(adc: SequenceAdc, select: SelectPins, probe: Probe) -> ! {
    run_scanner(adc, select, probe, &SAMPLES).await
}

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
    spawner.spawn(unwrap!(usb_task(usb_driver, serial_str)));

    // Flash & Firmware initialization
    let mut flash = Stm32Flash::new(Flash::new_blocking(p.FLASH));
    let mut board = Stm32Board;
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
    defmt::assert!(fw.init(&mut flash, &defaults));

    // ADC scanner on the high-priority executor (pins generated from keyboard.json)
    let adc = Adc::new(p.ADC1);
    let mut mux_inputs: MuxInputs = adc_mux_input_channels!(p);
    let mut raw_inputs: RawInputs = adc_raw_input_channels!(p);
    let request = embassy_stm32::adc::RxDma::<peripherals::ADC1>::request(&*p.DMA2_CH0);
    let dma = embassy_stm32::dma::Channel::new(p.DMA2_CH0, Irqs);
    let adc = SequenceAdc::new(adc, &mut mux_inputs, &mut raw_inputs, dma, request);
    let select: SelectPins = {
        let pins: [embassy_stm32::Peri<'static, AnyPin>; kb::ADC_MUX_SELECT_PINS.len()] =
            adc_mux_select_pins!(p);
        pins.map(|pin| Output::new(pin, Level::Low, Speed::VeryHigh))
    };
    // Optional timing probes (`scan-probe` feature, see docs/performance.md)
    #[cfg(feature = "scan-probe")]
    let (scan_probe, mut loop_probe) = (Probe::new(probe_scan_pin!(p)), Probe::new(probe_loop_pin!(p)));
    #[cfg(not(feature = "scan-probe"))]
    let (scan_probe, mut loop_probe) = (Probe::disabled(), Probe::disabled());

    interrupt::UART5.set_priority(Priority::P6);
    let adc_spawner = EXECUTOR_ADC.start(interrupt::UART5);
    adc_spawner.spawn(unwrap!(adc_task(adc, select, scan_probe)));

    // Wait for one complete matrix pass before calibrating
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

        loop_probe.toggle();
        let out = fw.step(&SAMPLES, &mut flash, &mut board, &mut ble);
        if !out.hid.is_empty() {
            let _ = USB_HID_TX.try_send(out.hid);
        }
        // Free-running loop like the C superloop; yield so USB tasks get serviced.
        yield_now().await;
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
async fn calibrate(fw: &mut Fw, flash: &mut Stm32Flash<'_>, reset_bottom_out: bool) {
    fw.matrix.begin_recalibrate(&mut fw.eeconfig, flash, reset_bottom_out);
    let start = embassy_time::Instant::now();
    while start.elapsed().as_millis() < MATRIX_CALIBRATION_DURATION_MS as u64 {
        fw.matrix.calibrate_step(&SAMPLES, &fw.eeconfig);
        yield_now().await;
    }
}
