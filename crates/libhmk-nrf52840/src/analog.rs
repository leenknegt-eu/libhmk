//! nRF52840 SAADC matrix scanner with multiplexer control, power-saving scan throttling,
//! and battery voltage measurement (VDD or external voltage divider pin).

use core::sync::atomic::{AtomicU16, AtomicU32, Ordering};

use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::peripherals;
use embassy_nrf::saadc::{
    ChannelConfig, Config, Gain, Input, Reference, Resolution, Saadc, Time, VddInput,
};
use embassy_nrf::Peri;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer};
use libhmk_core::traits::AnalogInput;

use crate::kb;

/// Signalled after every complete matrix pass (wakes the main loop on battery power).
pub static SCAN_DONE: Signal<CriticalSectionRawMutex, ()> = Signal::new();

const ADC_MAX_VALUE: u16 = ((1u32 << kb::ADC_RESOLUTION) - 1) as u16;

/// Shared lock-free latest ADC sample buffer updated by the async SAADC scanner task.
pub struct SharedAnalogSamples {
    values: [AtomicU16; kb::NUM_KEYS],
    scan_interval_us: AtomicU32,
    battery_mv: AtomicU16,
    /// Incremented after every complete matrix pass.
    generation: AtomicU32,
}

impl SharedAnalogSamples {
    pub const fn new() -> Self {
        #[allow(clippy::declare_interior_mutable_const)]
        const ZERO: AtomicU16 = AtomicU16::new(0);
        Self {
            values: [ZERO; kb::NUM_KEYS],
            scan_interval_us: AtomicU32::new(0),
            battery_mv: AtomicU16::new(3000),
            generation: AtomicU32::new(0),
        }
    }

    /// Store a sample for a 1-based key index from the generated matrix (0 = not connected).
    #[inline]
    fn store_key(&self, key: u16, sample: i16) {
        if let Some(slot) = (key as usize).checked_sub(1).and_then(|k| self.values.get(k)) {
            let clamped = if sample <= 0 {
                0
            } else {
                (sample as u16).min(ADC_MAX_VALUE)
            };
            slot.store(clamped, Ordering::Relaxed);
        }
    }

    pub fn generation(&self) -> u32 {
        self.generation.load(Ordering::Acquire)
    }

    pub fn set_scan_interval_us(&self, us: u32) {
        self.scan_interval_us.store(us, Ordering::Relaxed);
    }

    pub fn scan_interval_us(&self) -> u32 {
        self.scan_interval_us.load(Ordering::Relaxed)
    }

    pub fn set_battery_mv(&self, mv: u16) {
        self.battery_mv.store(mv, Ordering::Relaxed);
    }

    pub fn battery_mv(&self) -> u16 {
        self.battery_mv.load(Ordering::Relaxed)
    }

    pub fn battery_percent(&self) -> u8 {
        let mv = self.battery_mv();
        if kb::BLE_BATTERY_ADC_CHANNEL.is_some() {
            // Single-cell LiPo curve (4200 mV .. 3300 mV)
            if mv >= 4200 {
                100
            } else if mv <= 3300 {
                0
            } else if mv >= 3950 {
                (75 + ((mv - 3950) as u32 * 25) / 250) as u8
            } else if mv >= 3700 {
                (25 + ((mv - 3700) as u32 * 50) / 250) as u8
            } else {
                (((mv - 3300) as u32 * 25) / 400) as u8
            }
        } else {
            // Direct VDD measurement (3000 mV .. 2000 mV)
            if mv >= 3000 {
                100
            } else if mv <= 2000 {
                0
            } else {
                (((mv - 2000) as u32 * 100) / 1000) as u8
            }
        }
    }
}

impl AnalogInput for SharedAnalogSamples {
    #[inline]
    fn read_raw(&self, key: usize) -> u16 {
        match self.values.get(key) {
            Some(v) => v.load(Ordering::Relaxed),
            None => 0,
        }
    }
}

pub fn saadc_config() -> Config {
    let mut cfg = Config::default();
    cfg.resolution = match kb::ADC_RESOLUTION {
        8 => Resolution::_8bit,
        10 => Resolution::_10bit,
        14 => Resolution::_14bit,
        _ => Resolution::_12bit,
    };
    cfg
}

pub fn matrix_channel_config<'d>(pin: impl Input + 'd) -> ChannelConfig<'d> {
    let mut ch = ChannelConfig::single_ended(pin);
    // Ratiometric 0..VDD range for Hall-effect sensors
    ch.gain = Gain::Gain1_4;
    ch.reference = Reference::VDD1_4;
    // Matches the C firmware (`SAADC_CH_CONFIG_TACQ_3us`)
    ch.time = Time::_3US;
    ch
}

pub fn battery_channel_config<'d>(pin: impl Input + 'd) -> ChannelConfig<'d> {
    let mut ch = ChannelConfig::single_ended(pin);
    // 0.6V internal ref + 1/6 gain => 0..3.6V full scale
    ch.gain = Gain::Gain1_6;
    ch.reference = Reference::Internal;
    ch.time = Time::_10US;
    ch
}

include!(concat!(env!("OUT_DIR"), "/pins_gen.rs"));

/// How often the battery voltage is measured (outside the matrix scan sequence).
const BATTERY_INTERVAL: Duration = Duration::from_secs(10);

/// Measure the battery voltage once with a dedicated single-channel SAADC configuration.
async fn measure_battery_mv(
    saadc_p: &mut Peri<'static, peripherals::SAADC>,
    inputs: &mut AdcInputs,
) -> u16 {
    let mut saadc = Saadc::new(saadc_p.reborrow(), crate::Irqs, saadc_config(), [inputs.battery_config()]);
    let mut buf = [0i16; 1];
    saadc.sample(&mut buf).await;
    let raw = buf[0].max(0) as u32;
    let adc_mv = (raw * 3600) >> kb::ADC_RESOLUTION;
    ((adc_mv * kb::BLE_BATTERY_DIVIDER_MILLIS) / 1000) as u16
}

/// Continuously scan the multiplexed and raw SAADC inputs into `samples`.
///
/// The battery voltage is sampled every [`BATTERY_INTERVAL`] with its own SAADC
/// configuration, so the per-mux-step sequence only contains matrix channels.
pub async fn run_scanner(
    mut saadc_p: Peri<'static, peripherals::SAADC>,
    mut inputs: AdcInputs,
    mut select_pins: [Output<'static>; kb::ADC_MUX_SELECT_PINS.len()],
    samples: &'static SharedAnalogSamples,
) -> ! {
    let settle = Duration::from_micros(kb::ADC_SAMPLE_DELAY_US as u64);
    let num_mux_inputs = kb::ADC_MUX_INPUT_CHANNELS.len();
    let mut buf = [0i16; NUM_SAADC_CHANNELS];

    loop {
        if kb::BLUETOOTH_ENABLED {
            samples.set_battery_mv(measure_battery_mv(&mut saadc_p, &mut inputs).await);
        }

        let mut saadc = Saadc::new(saadc_p.reborrow(), crate::Irqs, saadc_config(), inputs.matrix_configs());
        saadc.calibrate().await;
        let battery_due = Instant::now() + BATTERY_INTERVAL;

        while !kb::BLUETOOTH_ENABLED || Instant::now() < battery_due {
            for (mux_ch, keys) in kb::ADC_MUX_INPUT_MATRIX.iter().enumerate() {
                for (bit, pin) in select_pins.iter_mut().enumerate() {
                    if (mux_ch >> bit) & 1 != 0 {
                        pin.set_high();
                    } else {
                        pin.set_low();
                    }
                }
                Timer::after(settle).await;
                saadc.sample(&mut buf).await;
                for (&key, &val) in keys.iter().zip(buf.iter()) {
                    samples.store_key(key, val);
                }
            }

            if kb::ADC_MUX_NUM_CHANNELS == 0 {
                saadc.sample(&mut buf).await;
            }
            for (&key, &val) in kb::ADC_RAW_INPUT_VECTOR.iter().zip(buf[num_mux_inputs..].iter()) {
                samples.store_key(key, val);
            }

            samples.generation.fetch_add(1, Ordering::Release);
            SCAN_DONE.signal(());

            let interval = samples.scan_interval_us();
            if interval > 0 {
                Timer::after(Duration::from_micros(interval as u64)).await;
            } else if kb::ADC_MUX_NUM_CHANNELS == 0 {
                Timer::after(settle).await;
            }
        }
    }
}

pub fn make_select_output(pin: Peri<'static, embassy_nrf::gpio::AnyPin>) -> Output<'static> {
    Output::new(pin, Level::Low, OutputDrive::HighDrive)
}
