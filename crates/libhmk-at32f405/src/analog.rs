//! AT32F405xx ADC1 multiplexed Hall-effect sensor scanner via `at32f4xx-pac`.

use at32f4xx_pac::{ADC1, GPIOC};
use libhmk_core::traits::AnalogInput;

use crate::kb;

pub struct At32Analog {
    adc: ADC1,
    gpioc: GPIOC,
    values: [u16; kb::NUM_KEYS],
}

impl At32Analog {
    pub fn new(adc: ADC1, gpioc: GPIOC) -> Self {
        // Enable ADC1 and perform internal calibration
        adc.ctrl2().modify(|_, w| w.adcen().set_bit());
        adc.ctrl2().modify(|_, w| w.adcalinit().set_bit());
        while adc.ctrl2().read().adcalinit().bit_is_set() {}
        adc.ctrl2().modify(|_, w| w.adcal().set_bit());
        while adc.ctrl2().read().adcal().bit_is_set() {}

        Self {
            adc,
            gpioc,
            values: [0; kb::NUM_KEYS],
        }
    }

    fn read_channel(&self, ch: u8) -> u16 {
        self.adc
            .osq3()
            .write(|w| unsafe { w.osn1().bits(ch) });
        self.adc.ctrl2().modify(|_, w| w.ocswtrg().set_bit());
        while self.adc.sts().read().occe().bit_is_clear() {}
        (self.adc.odt().read().bits() & 0x0FFF) as u16
    }

    pub fn scan_all(&mut self) {
        // Default HE16 mux select pins on PC13..PC15
        for mux_ch in 0..kb::ADC_MUX_NUM_CHANNELS {
            let set_bits = ((mux_ch as u32) & 0x07) << 13;
            let clr_bits = ((!(mux_ch as u32)) & 0x07) << 29;
            self.gpioc
                .scr()
                .write(|w| unsafe { w.bits(set_bits | clr_bits) });
            cortex_m::asm::delay(216 * kb::ADC_SAMPLE_DELAY_US);

            for (i, &ch) in kb::ADC_MUX_INPUT_CHANNELS.iter().enumerate() {
                let sample = self.read_channel(ch);
                let key = kb::ADC_MUX_INPUT_MATRIX[mux_ch][i] as usize;
                if key != 0 && key <= kb::NUM_KEYS {
                    self.values[key - 1] = sample;
                }
            }
        }
    }
}

impl AnalogInput for At32Analog {
    fn read_raw(&self, key: usize) -> u16 {
        self.values.get(key).copied().unwrap_or(0)
    }
}
