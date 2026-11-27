//! STM32F446xx ADC1 Hall-effect key matrix scanner.
//!
//! The scanner runs on a high-priority interrupt executor, mirroring the C firmware's
//! timer + ADC interrupt pipeline: the main loop only ever reads the latest sample of
//! each key and never waits on the ADC or on multiplexer settling.

use core::sync::atomic::{AtomicU16, AtomicU32, Ordering};

use embassy_stm32::adc::{Adc, AnyAdcChannel, SampleTime};
use embassy_stm32::gpio::Output;
use embassy_stm32::peripherals::ADC1;
use embassy_time::{Duration, Timer};
use libhmk_core::traits::AnalogInput;

use crate::kb;

/// Sample time per conversion (C firmware default: `ADC_SAMPLETIME_3CYCLES`).
const SAMPLE_TIME: SampleTime = SampleTime::CYCLES3;

pub struct SharedAnalogSamples {
    values: [AtomicU16; kb::NUM_KEYS],
    /// Incremented after every complete matrix pass.
    generation: AtomicU32,
}

impl SharedAnalogSamples {
    pub const fn new() -> Self {
        #[allow(clippy::declare_interior_mutable_const)]
        const ZERO: AtomicU16 = AtomicU16::new(0);
        Self {
            values: [ZERO; kb::NUM_KEYS],
            generation: AtomicU32::new(0),
        }
    }

    /// Store a sample for a 1-based key index from the generated matrix (0 = not connected).
    #[inline]
    fn store_key(&self, key: u16, val: u16) {
        if let Some(slot) = (key as usize).checked_sub(1).and_then(|k| self.values.get(k)) {
            slot.store(val, Ordering::Relaxed);
        }
    }

    pub fn generation(&self) -> u32 {
        self.generation.load(Ordering::Acquire)
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

pub type MuxInputs = [AnyAdcChannel<'static, ADC1>; kb::ADC_MUX_INPUT_CHANNELS.len()];
pub type RawInputs = [AnyAdcChannel<'static, ADC1>; kb::ADC_RAW_INPUT_CHANNELS.len()];
pub type SelectPins = [Output<'static>; kb::ADC_MUX_SELECT_PINS.len()];

/// Continuously sample every multiplexer channel and raw input into `samples`.
pub async fn run_scanner(
    mut adc: Adc<'static, ADC1>,
    mut mux_inputs: MuxInputs,
    mut raw_inputs: RawInputs,
    mut select: SelectPins,
    samples: &'static SharedAnalogSamples,
) -> ! {
    let settle = Duration::from_micros(kb::ADC_SAMPLE_DELAY_US as u64);
    loop {
        for (ch, keys) in kb::ADC_MUX_INPUT_MATRIX.iter().enumerate() {
            for (bit, pin) in select.iter_mut().enumerate() {
                if (ch >> bit) & 1 != 0 {
                    pin.set_high();
                } else {
                    pin.set_low();
                }
            }
            // Yield while the multiplexer output settles instead of busy-waiting.
            Timer::after(settle).await;
            for (input, &key) in mux_inputs.iter_mut().zip(keys.iter()) {
                let val = adc.blocking_read(input, SAMPLE_TIME);
                samples.store_key(key, val);
            }
        }

        for (input, &key) in raw_inputs.iter_mut().zip(kb::ADC_RAW_INPUT_VECTOR.iter()) {
            let val = adc.blocking_read(input, SAMPLE_TIME);
            samples.store_key(key, val);
        }

        samples.generation.fetch_add(1, Ordering::Release);

        if kb::ADC_MUX_NUM_CHANNELS == 0 {
            // Raw-only boards: pace the loop so the high-priority executor cannot starve
            // the thread-mode tasks.
            Timer::after(settle).await;
        }
    }
}
