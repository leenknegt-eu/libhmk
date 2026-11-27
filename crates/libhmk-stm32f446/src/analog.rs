//! STM32F446xx ADC1 Hall-effect key matrix scanner.
//!
//! The scanner runs on a high-priority interrupt executor, mirroring the C firmware's
//! timer + ADC interrupt pipeline: the main loop only ever reads the latest sample of
//! each key and never waits on the ADC or on multiplexer settling.

use core::sync::atomic::{AtomicU16, AtomicU32, Ordering};

use embassy_stm32::adc::{Adc, AnyAdcChannel, SampleTime};
use embassy_stm32::dma;
use embassy_stm32::pac;
use embassy_stm32::pac::adc::vals::{Dds, Eocs};
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

const NUM_MUX_INPUTS: usize = kb::ADC_MUX_INPUT_CHANNELS.len();
const NUM_INPUTS: usize = NUM_MUX_INPUTS + kb::ADC_RAW_INPUT_CHANNELS.len();
const _: () = assert!(NUM_INPUTS > 0 && NUM_INPUTS <= 16, "1..=16 ADC inputs supported");

/// ADC1 configured once for a regular scan sequence over every input (mux inputs, then raw
/// inputs), read out by DMA2 stream 0 — the same pipeline as the C firmware.
///
/// `embassy_stm32::adc::Adc::blocking_read` powers the ADC down and up (3 us stabilization
/// delay) around every single conversion; this keeps it powered and converts the whole
/// sequence back to back (~0.7 us per input at 22.5 MHz / 3-cycle sampling).
pub struct SequenceAdc {
    _adc: Adc<'static, ADC1>,
    dma: dma::Channel<'static>,
    request: dma::Request,
}

impl SequenceAdc {
    pub fn new(
        mut adc: Adc<'static, ADC1>,
        mux_inputs: &mut MuxInputs,
        raw_inputs: &mut RawInputs,
        dma: dma::Channel<'static>,
        request: dma::Request,
    ) -> Self {
        // One conversion per input puts its GPIO in analog mode via the HAL.
        for input in mux_inputs.iter_mut().chain(raw_inputs.iter_mut()) {
            let _ = adc.blocking_read(input, SAMPLE_TIME);
        }

        let r = pac::ADC1;
        r.cr2().modify(|w| w.set_adon(false));
        let channels = kb::ADC_MUX_INPUT_CHANNELS
            .iter()
            .chain(kb::ADC_RAW_INPUT_CHANNELS.iter());
        for (rank, &ch) in channels.enumerate() {
            match rank {
                0..=5 => r.sqr3().modify(|w| w.set_sq(rank, ch)),
                6..=11 => r.sqr2().modify(|w| w.set_sq(rank - 6, ch)),
                _ => r.sqr1().modify(|w| w.set_sq(rank - 12, ch)),
            }
            if ch <= 9 {
                r.smpr2().modify(|w| w.set_smp(ch as usize, SampleTime::CYCLES3));
            } else {
                r.smpr1().modify(|w| w.set_smp(ch as usize - 10, SampleTime::CYCLES3));
            }
        }
        r.sqr1().modify(|w| w.set_l((NUM_INPUTS - 1) as u8));
        r.cr1().modify(|w| {
            w.set_scan(true);
            w.set_eocie(false);
            w.set_ovrie(false);
        });
        r.cr2().modify(|w| {
            w.set_cont(false);
            w.set_dds(Dds::SINGLE);
            w.set_eocs(Eocs::EACH_SEQUENCE);
            w.set_adon(true);
        });
        // ADC power-up stabilization (tSTAB), once.
        cortex_m::asm::delay(180 * 3);

        Self { _adc: adc, dma, request }
    }

    /// Convert the whole sequence once into `buf`.
    pub async fn sample(&mut self, buf: &mut [u16; NUM_INPUTS]) {
        let r = pac::ADC1;
        // Re-arm ADC DMA requests (DDS = single) and clear a stale overrun.
        r.cr2().modify(|w| w.set_dma(false));
        r.sr().modify(|w| {
            w.set_ovr(false);
            w.set_eoc(false);
        });
        r.cr2().modify(|w| w.set_dma(true));
        let transfer = unsafe {
            self.dma.read(
                self.request,
                r.dr().as_ptr() as *mut u16,
                buf,
                dma::TransferOptions::default(),
            )
        };
        r.cr2().modify(|w| w.set_swstart(true));
        transfer.await;
    }
}

/// Continuously sample every multiplexer channel and raw input into `samples`.
pub async fn run_scanner(
    mut adc: SequenceAdc,
    mut select: SelectPins,
    samples: &'static SharedAnalogSamples,
) -> ! {
    let settle = Duration::from_micros(kb::ADC_SAMPLE_DELAY_US as u64);
    let mut buf = [0u16; NUM_INPUTS];
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
            adc.sample(&mut buf).await;
            for (&key, &val) in keys.iter().zip(buf.iter()) {
                samples.store_key(key, val);
            }
        }

        if kb::ADC_MUX_NUM_CHANNELS == 0 {
            adc.sample(&mut buf).await;
        }
        for (&key, &val) in kb::ADC_RAW_INPUT_VECTOR.iter().zip(buf[NUM_MUX_INPUTS..].iter()) {
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
