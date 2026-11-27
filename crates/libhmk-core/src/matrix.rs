//! Hall-effect key matrix scanning, EMA filtering, calibration, and Rapid Trigger.

use crate::distance::adc_to_distance;
use crate::eeconfig::Eeconfig;
use crate::traits::{AnalogInput, FlashStorage};
use crate::types::Actuation;

pub const MATRIX_CALIBRATION_DURATION_MS: u32 = 500;
pub const MATRIX_EMA_ALPHA_EXPONENT: u32 = 4;
pub const MATRIX_CALIBRATION_EPSILON: u16 = 5;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum KeyDir {
    #[default]
    Inactive = 0,
    Down = 1,
    Up = 2,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct KeyState {
    pub adc_filtered: u16,
    pub adc_rest_value: u16,
    pub adc_bottom_out_value: u16,
    pub distance: u8,
    pub extremum: u8,
    pub key_dir: KeyDir,
    pub is_pressed: bool,
}

#[inline]
fn ema(x: u16, y: u16) -> u16 {
    let num = (x as u32) + ((y as u32) * ((1u32 << MATRIX_EMA_ALPHA_EXPONENT) - 1));
    (num >> MATRIX_EMA_ALPHA_EXPONENT) as u16
}

/// One Hall-effect key update: EMA filter, bottom-out tracking, distance and Rapid Trigger.
///
/// Fields are only stored when they change (like the C firmware), and all comparisons are
/// done in `u32`, which is exact for the 8-bit distances and <= 16-bit ADC values involved
/// (matching C integer promotion).
#[inline(always)]
fn update_key(s: &mut KeyState, raw: u16, act: Actuation, rt_disabled: bool) {
    let filtered = ema(raw, s.adc_filtered);
    s.adc_filtered = filtered;
    let mut bottom = s.adc_bottom_out_value;
    if filtered as u32 >= bottom as u32 + MATRIX_CALIBRATION_EPSILON as u32 {
        bottom = filtered;
        s.adc_bottom_out_value = bottom;
    }
    let d = adc_to_distance(filtered, s.adc_rest_value, bottom);
    s.distance = d;

    if rt_disabled || act.rt_down == 0 {
        s.key_dir = KeyDir::Inactive;
        s.is_pressed = d >= act.actuation_point;
        return;
    }

    let d32 = d as u32;
    let reset_point = if act.continuous { 0 } else { act.actuation_point as u32 };
    match s.key_dir {
        KeyDir::Inactive => {
            if d > act.actuation_point {
                s.extremum = d;
                s.key_dir = KeyDir::Down;
                s.is_pressed = true;
            }
        }
        KeyDir::Down => {
            let e32 = s.extremum as u32;
            let rt_up = if act.rt_up == 0 { act.rt_down } else { act.rt_up } as u32;
            if d32 <= reset_point {
                s.extremum = d;
                s.key_dir = KeyDir::Inactive;
                s.is_pressed = false;
            } else if d32 + rt_up < e32 {
                s.extremum = d;
                s.key_dir = KeyDir::Up;
                s.is_pressed = false;
            } else if d32 > e32 {
                s.extremum = d;
            }
        }
        KeyDir::Up => {
            let e32 = s.extremum as u32;
            if d32 <= reset_point {
                s.extremum = d;
                s.key_dir = KeyDir::Inactive;
                s.is_pressed = false;
            } else if e32 + (act.rt_down as u32) < d32 {
                s.extremum = d;
                s.key_dir = KeyDir::Down;
                s.is_pressed = true;
            } else if d32 < e32 {
                s.extremum = d;
            }
        }
    }
}

pub struct Matrix<const NUM_KEYS: usize> {
    pub keys: [KeyState; NUM_KEYS],
    rapid_trigger_disabled: [bool; NUM_KEYS],
    adc_max_value: u16,
    invert_adc: bool,
}

impl<const NUM_KEYS: usize> Matrix<NUM_KEYS> {
    pub const fn new(adc_resolution: u8, invert_adc: bool) -> Self {
        Self {
            keys: [KeyState {
                adc_filtered: 0,
                adc_rest_value: 0,
                adc_bottom_out_value: 0,
                distance: 0,
                extremum: 0,
                key_dir: KeyDir::Inactive,
                is_pressed: false,
            }; NUM_KEYS],
            rapid_trigger_disabled: [false; NUM_KEYS],
            adc_max_value: ((1u32 << adc_resolution) - 1) as u16,
            invert_adc,
        }
    }

    #[inline]
    fn read_adc<A: AnalogInput>(&self, analog: &A, key: usize) -> u16 {
        let raw = analog.read_raw(key).min(self.adc_max_value);
        if self.invert_adc {
            self.adc_max_value - raw
        } else {
            raw
        }
    }

    #[inline]
    fn bottom_out_value<const VS: usize, const WL: usize>(
        &self,
        ee: &Eeconfig<VS, WL>,
        key: usize,
        rest_value: u16,
    ) -> u16 {
        let cal = ee.calibration();
        let saved = ee.bottom_out_threshold(key);
        let threshold = cal.initial_bottom_out_threshold.max(saved);
        rest_value.saturating_add(threshold).min(self.adc_max_value)
    }

    /// Reset calibration baselines (call `calibrate_step` for `MATRIX_CALIBRATION_DURATION_MS`).
    pub fn begin_recalibrate<F: FlashStorage, const VS: usize, const WL: usize>(
        &mut self,
        ee: &mut Eeconfig<VS, WL>,
        flash: &mut F,
        reset_bottom_out_threshold: bool,
    ) {
        if reset_bottom_out_threshold {
            for k in 0..NUM_KEYS {
                ee.set_bottom_out_threshold(flash, k, 0);
            }
        }
        let cal = ee.calibration();
        for k in 0..NUM_KEYS {
            self.keys[k] = KeyState {
                adc_filtered: cal.initial_rest_value,
                adc_rest_value: cal.initial_rest_value,
                adc_bottom_out_value: self.bottom_out_value(ee, k, cal.initial_rest_value),
                distance: 0,
                extremum: 0,
                key_dir: KeyDir::Inactive,
                is_pressed: false,
            };
        }
    }

    /// Run one calibration sample iteration during startup / recalibration.
    pub fn calibrate_step<A: AnalogInput, const VS: usize, const WL: usize>(
        &mut self,
        analog: &A,
        ee: &Eeconfig<VS, WL>,
    ) {
        for k in 0..NUM_KEYS {
            let new_filtered = ema(self.read_adc(analog, k), self.keys[k].adc_filtered);
            self.keys[k].adc_filtered = new_filtered;

            if new_filtered.saturating_add(MATRIX_CALIBRATION_EPSILON)
                <= self.keys[k].adc_rest_value
            {
                self.keys[k].adc_rest_value = new_filtered;
            }
            self.keys[k].adc_bottom_out_value =
                self.bottom_out_value(ee, k, self.keys[k].adc_rest_value);
        }
    }

    /// Perform one full Hall-effect matrix scan with Rapid Trigger & Continuous RT.
    pub fn scan<A: AnalogInput, const VS: usize, const WL: usize>(
        &mut self,
        analog: &A,
        ee: &Eeconfig<VS, WL>,
    ) {
        let profile = ee.current_profile() as usize;
        // One bounds check for the whole profile instead of one slice per key.
        let (actuations, _) = ee.actuation_map(profile).as_chunks::<{ Actuation::SIZE }>();
        // Hoisted out of the loop: `max - raw == raw ^ max` for `raw <= max = 2^n - 1`.
        let adc_max = self.adc_max_value;
        let invert_mask = if self.invert_adc { adc_max } else { 0 };
        let keys = self
            .keys
            .iter_mut()
            .zip(self.rapid_trigger_disabled.iter())
            .zip(actuations.iter());
        for (k, ((state, &rt_disabled), act)) in keys.enumerate() {
            let raw = analog.read_raw(k).min(adc_max) ^ invert_mask;
            update_key(state, raw, Actuation::from_array(*act), rt_disabled);
        }
    }

    pub fn set_rapid_trigger_disabled(&mut self, key: usize, disabled: bool) {
        if key < NUM_KEYS {
            self.rapid_trigger_disabled[key] = disabled;
        }
    }
}
