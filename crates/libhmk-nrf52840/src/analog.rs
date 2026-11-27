//! nRF52840 SAADC matrix scanner with multiplexer control, power-saving scan throttling,
//! and battery voltage measurement (VDD or external voltage divider pin).

use core::sync::atomic::{AtomicU16, AtomicU32, Ordering};

use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::saadc::{ChannelConfig, Config, Gain, Reference, Resolution, Saadc, Time};
use embassy_time::{Duration, Timer};
use libhmk_core::traits::AnalogInput;

use crate::kb;

const ADC_MAX_VALUE: u16 = ((1u32 << kb::ADC_RESOLUTION) - 1) as u16;

/// Shared lock-free latest ADC sample buffer updated by the async SAADC scanner task.
pub struct SharedAnalogSamples {
    values: [AtomicU16; kb::NUM_KEYS],
    scan_interval_us: AtomicU32,
    battery_mv: AtomicU16,
}

impl SharedAnalogSamples {
    pub const fn new() -> Self {
        #[allow(clippy::declare_interior_mutable_const)]
        const ZERO: AtomicU16 = AtomicU16::new(0);
        Self {
            values: [ZERO; kb::NUM_KEYS],
            scan_interval_us: AtomicU32::new(0),
            battery_mv: AtomicU16::new(3000),
        }
    }

    #[inline]
    pub fn store(&self, key: usize, sample: i16) {
        if key < kb::NUM_KEYS {
            let clamped = if sample <= 0 {
                0
            } else {
                (sample as u16).min(ADC_MAX_VALUE)
            };
            self.values[key].store(clamped, Ordering::Relaxed);
        }
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
    fn read_raw(&self, key: usize) -> u16 {
        if key < kb::NUM_KEYS {
            self.values[key].load(Ordering::Relaxed)
        } else {
            0
        }
    }
}

pub fn saadc_config() -> Config {
    let mut cfg = Config::default();
    cfg.resolution = match kb::ADC_RESOLUTION {
        8 => Resolution::_8BIT,
        10 => Resolution::_10BIT,
        14 => Resolution::_14BIT,
        _ => Resolution::_12BIT,
    };
    cfg
}

pub fn matrix_channel_config<P: embassy_nrf::saadc::Input>(pin: P) -> ChannelConfig<'static> {
    let mut ch = ChannelConfig::single_ended(pin);
    // Ratiometric 0..VDD range for Hall-effect sensors
    ch.gain = Gain::GAIN1_4;
    ch.reference = Reference::VDD1_4;
    ch.time = Time::_3US;
    ch
}

pub fn vdd_battery_channel_config() -> ChannelConfig<'static> {
    let mut ch = ChannelConfig::single_ended(embassy_nrf::saadc::VddInput);
    // 0.6V internal ref + 1/6 gain => 0..3.6V full scale
    ch.gain = Gain::GAIN1_6;
    ch.reference = Reference::INTERNAL;
    ch.time = Time::_10US;
    ch
}

/// Continuously scan multiplexed SAADC channels + VDD battery channel into `samples`.
pub async fn run_mux_scanner<const N: usize>(
    mut saadc: Saadc<'static, N>,
    mut select_pins: [Output<'static>; 3],
    samples: &'static SharedAnalogSamples,
) -> ! {
    saadc.calibrate().await;
    let mut buf = [0i16; N];
    let num_mux_inputs = kb::ADC_MUX_INPUT_CHANNELS.len();

    loop {
        for mux_ch in 0..kb::ADC_MUX_NUM_CHANNELS {
            for (bit, pin) in select_pins.iter_mut().enumerate() {
                if ((mux_ch >> bit) & 1) != 0 {
                    pin.set_high();
                } else {
                    pin.set_low();
                }
            }
            Timer::after(Duration::from_micros(kb::ADC_SAMPLE_DELAY_US as u64)).await;

            saadc.sample(&mut buf).await;

            for i in 0..num_mux_inputs {
                let key = kb::ADC_MUX_INPUT_MATRIX[mux_ch][i] as usize;
                if key != 0 {
                    samples.store(key - 1, buf[i]);
                }
            }

            // Last channel in `buf` carries internal VDD sample for Battery Service
            if N > num_mux_inputs {
                let raw = buf[num_mux_inputs].max(0) as u32;
                let adc_mv = (raw * 3600) >> kb::ADC_RESOLUTION;
                let mv = (adc_mv * kb::BLE_BATTERY_DIVIDER_MILLIS) / 1000;
                samples.set_battery_mv(mv as u16);
            }
        }

        let interval = samples.scan_interval_us();
        if interval > 0 {
            Timer::after(Duration::from_micros(interval as u64)).await;
        }
    }
}

pub fn make_select_output(pin: embassy_nrf::Peri<'static, embassy_nrf::gpio::AnyPin>) -> Output<'static> {
    Output::new(pin, Level::Low, OutputDrive::HighDrive)
}
