//! Hall-effect key matrix scanning, EMA filtering, calibration, and Rapid Trigger.

use crate::distance::adc_to_distance;
use crate::eeconfig::Eeconfig;
use crate::traits::{AnalogInput, FlashStorage};

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
        for k in 0..NUM_KEYS {
            let new_filtered = ema(self.read_adc(analog, k), self.keys[k].adc_filtered);
            let act = ee.actuation(profile, k);
            let state = &mut self.keys[k];

            state.adc_filtered = new_filtered;
            if new_filtered
                >= state
                    .adc_bottom_out_value
                    .saturating_add(MATRIX_CALIBRATION_EPSILON)
            {
                state.adc_bottom_out_value = new_filtered;
            }

            state.distance = adc_to_distance(
                new_filtered,
                state.adc_rest_value,
                state.adc_bottom_out_value,
            );

            if self.rapid_trigger_disabled[k] || act.rt_down == 0 {
                state.key_dir = KeyDir::Inactive;
                state.is_pressed = state.distance >= act.actuation_point;
            } else {
                let reset_point = if act.continuous { 0 } else { act.actuation_point };
                let rt_up = if act.rt_up == 0 { act.rt_down } else { act.rt_up };

                match state.key_dir {
                    KeyDir::Inactive => {
                        if state.distance > act.actuation_point {
                            state.extremum = state.distance;
                            state.key_dir = KeyDir::Down;
                            state.is_pressed = true;
                        }
                    }
                    KeyDir::Down => {
                        if state.distance <= reset_point {
                            state.extremum = state.distance;
                            state.key_dir = KeyDir::Inactive;
                            state.is_pressed = false;
                        } else if state.distance.saturating_add(rt_up) < state.extremum {
                            state.extremum = state.distance;
                            state.key_dir = KeyDir::Up;
                            state.is_pressed = false;
                        } else if state.distance > state.extremum {
                            state.extremum = state.distance;
                        }
                    }
                    KeyDir::Up => {
                        if state.distance <= reset_point {
                            state.extremum = state.distance;
                            state.key_dir = KeyDir::Inactive;
                            state.is_pressed = false;
                        } else if state.extremum.saturating_add(act.rt_down) < state.distance {
                            state.extremum = state.distance;
                            state.key_dir = KeyDir::Down;
                            state.is_pressed = true;
                        } else if state.distance < state.extremum {
                            state.extremum = state.distance;
                        }
                    }
                }
            }
        }
    }

    pub fn set_rapid_trigger_disabled(&mut self, key: usize, disabled: bool) {
        if key < NUM_KEYS {
            self.rapid_trigger_disabled[key] = disabled;
        }
    }
}
