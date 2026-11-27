//! STM32F446xx ADC1 Hall-effect key matrix scanner.

use core::sync::atomic::{AtomicU16, Ordering};

use libhmk_core::traits::AnalogInput;

use crate::kb;

pub struct SharedAnalogSamples {
    values: [AtomicU16; kb::NUM_KEYS],
}

impl SharedAnalogSamples {
    pub const fn new() -> Self {
        #[allow(clippy::declare_interior_mutable_const)]
        const ZERO: AtomicU16 = AtomicU16::new(0);
        Self {
            values: [ZERO; kb::NUM_KEYS],
        }
    }

    #[inline]
    pub fn store(&self, key: usize, val: u16) {
        if key < kb::NUM_KEYS {
            self.values[key].store(val, Ordering::Relaxed);
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
