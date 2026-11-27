//! AT32F405xx ADC1 Hall-effect matrix scanner (interrupt driven, no CPU polling).
//!
//! Port of the C firmware pipeline: the ADC converts the whole input sequence (mux inputs,
//! then raw inputs) into a DMA buffer; the DMA full-transfer interrupt stores the samples,
//! advances the multiplexer and starts TMR6 as the settle delay; the TMR6 overflow interrupt
//! triggers the next conversion. The main loop only reads the latest samples.

use core::sync::atomic::{AtomicU16, AtomicU32, Ordering};

use at32f4xx_pac::at32f405::{interrupt, Interrupt, ADC1, ADCCOM, CRM, DMA1, TMR6};
use libhmk_core::traits::AnalogInput;

use crate::board::APB1_TIMER_HZ;
use crate::kb;

include!(concat!(env!("OUT_DIR"), "/pins_gen.rs"));

const NUM_MUX_INPUTS: usize = kb::ADC_MUX_INPUT_CHANNELS.len();
const NUM_INPUTS: usize = NUM_MUX_INPUTS + kb::ADC_RAW_INPUT_CHANNELS.len();
const _: () = assert!(NUM_INPUTS > 0 && NUM_INPUTS <= 16, "1..=16 ADC inputs supported");
const NUM_SELECT_PINS: usize = MUX_SELECT_GPIO.len();
/// TMR6 period for the multiplexer settle delay (16-bit counter at 216 MHz).
const SETTLE_TICKS: u16 = {
    let ticks = APB1_TIMER_HZ / 1_000_000 * kb::ADC_SAMPLE_DELAY_US - 1;
    assert!(ticks <= u16::MAX as u32, "ADC_SAMPLE_DELAY_US too large for TMR6");
    ticks as u16
};

/// C firmware default `ADC_NUM_SAMPLE_CYCLES` = `ADC_SAMPLETIME_7_5`.
const SAMPLE_TIME_7_5: u32 = 0x01;
/// `ADC_DIV_8`: ADCCLK = HCLK / 8 = 27 MHz.
const ADC_DIV_8: u32 = 0x06;
/// `DMAMUX_DMAREQ_ID_ADC1`
const DMAREQ_ADC1: u32 = 0x05;

// ADC_CTRL1 / ADC_CTRL2
const CTRL1_SQEN: u32 = 1 << 8;
const CTRL2_ADCEN: u32 = 1 << 0;
const CTRL2_ADCAL: u32 = 1 << 2;
const CTRL2_ADCALINIT: u32 = 1 << 3;
const CTRL2_OCDMAEN: u32 = 1 << 8;
const CTRL2_OCTESEL_SOFTWARE: u32 = 0b111 << 17;
const CTRL2_OCTEN: u32 = 1 << 20;
const CTRL2_OCSWTRG: u32 = 1 << 22;
// DMA channel CTRL
const DMA_CHEN: u32 = 1 << 0;
const DMA_FDTIEN: u32 = 1 << 1;
const DMA_LM: u32 = 1 << 5;
const DMA_MINCM: u32 = 1 << 7;
const DMA_PWIDTH_16: u32 = 0b01 << 8;
const DMA_MWIDTH_16: u32 = 0b01 << 10;
const DMA_CHPL_HIGH: u32 = 0b10 << 12;
const DMA_FDT1_FLAG: u32 = 1 << 1;
// TMR6
const TMR_EN: u32 = 1 << 0;
const TMR_OVF: u32 = 1 << 0;

const GPIO_BASE: usize = 0x4002_0000;
const GPIO_STRIDE: usize = 0x400;
const GPIO_CFGR: usize = 0x00;
const GPIO_ODRVR: usize = 0x08;
const GPIO_PULL: usize = 0x0C;
const GPIO_ODT: usize = 0x14;
const GPIO_SCR: usize = 0x18;

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

pub static SAMPLES: SharedAnalogSamples = SharedAnalogSamples::new();

/// DMA target buffer (written by hardware, read in the DMA interrupt).
static mut ADC_BUFFER: [u16; NUM_INPUTS] = [0; NUM_INPUTS];
static CURRENT_MUX_CHANNEL: AtomicU32 = AtomicU32::new(0);

#[inline]
fn gpio_reg(port: u8, offset: usize) -> *mut u32 {
    (GPIO_BASE + port as usize * GPIO_STRIDE + offset) as *mut u32
}

unsafe fn gpio_modify_2bit(port: u8, offset: usize, pin: u8, value: u32) {
    let reg = gpio_reg(port, offset);
    let shift = pin as u32 * 2;
    let v = (reg.read_volatile() & !(0b11 << shift)) | (value << shift);
    reg.write_volatile(v);
}

/// Drive a timing probe pin (`scan-probe` feature; `None` compiles to nothing).
#[inline(always)]
fn probe_set(gpio: Option<(u8, u8)>, high: bool) {
    if let Some((port, pin)) = gpio {
        let mask = 1u32 << pin;
        unsafe { gpio_reg(port, GPIO_SCR).write_volatile(if high { mask } else { mask << 16 }) };
    }
}

/// Toggle the main-loop probe pin (`LIBHMK_PROBE_LOOP_PIN`), once per main-loop iteration.
#[inline(always)]
pub fn probe_loop_toggle() {
    if let Some((port, pin)) = PROBE_LOOP_GPIO {
        let high = unsafe { gpio_reg(port, GPIO_ODT).read_volatile() } & (1 << pin) == 0;
        probe_set(PROBE_LOOP_GPIO, high);
    }
}

/// Drive the multiplexer select pins for `channel`.
#[inline]
fn set_mux_channel(channel: u32) {
    for (bit, &(port, pin)) in MUX_SELECT_GPIO.iter().enumerate() {
        let mask = 1u32 << pin;
        let value = if (channel >> bit) & 1 != 0 { mask } else { mask << 16 };
        unsafe { gpio_reg(port, GPIO_SCR).write_volatile(value) };
    }
}

fn adc() -> &'static at32f4xx_pac::at32f405::adc1::RegisterBlock {
    unsafe { &*ADC1::ptr() }
}

fn dma() -> &'static at32f4xx_pac::at32f405::dma1::RegisterBlock {
    unsafe { &*DMA1::ptr() }
}

fn tmr6() -> &'static at32f4xx_pac::at32f405::tmr6::RegisterBlock {
    unsafe { &*TMR6::ptr() }
}

/// Configure GPIO, ADC1, DMA1 channel 1 and TMR6 and start the continuous scan.
pub fn init(crm: &CRM) {
    unsafe {
        crm.ahben1()
            .modify(|_, w| {
                w.gpioa().set_bit();
                w.gpiob().set_bit();
                w.gpioc().set_bit();
                w.gpiod().set_bit();
                w.gpiof().set_bit();
                w.dma1().set_bit()
            });
        crm.apb2en().modify(|_, w| w.adc1().set_bit());
        crm.apb1en().modify(|_, w| w.tmr6().set_bit());

        // Analog inputs (mode 0b11, no pull)
        for &(port, pin) in ADC_INPUT_GPIO.iter() {
            gpio_modify_2bit(port, GPIO_CFGR, pin, 0b11);
            gpio_modify_2bit(port, GPIO_PULL, pin, 0b00);
        }
        // Mux select outputs (mode 0b01, push-pull, stronger drive), start at channel 0
        set_mux_channel(0);
        for &(port, pin) in MUX_SELECT_GPIO.iter() {
            gpio_modify_2bit(port, GPIO_CFGR, pin, 0b01);
            gpio_modify_2bit(port, GPIO_ODRVR, pin, 0b01);
            gpio_modify_2bit(port, GPIO_PULL, pin, 0b00);
        }

        // Timing probes: push-pull outputs, start low
        for &(port, pin) in PROBE_SCAN_GPIO.iter().chain(PROBE_LOOP_GPIO.iter()) {
            gpio_reg(port, GPIO_SCR).write_volatile(1 << (pin + 16));
            gpio_modify_2bit(port, GPIO_CFGR, pin, 0b01);
            gpio_modify_2bit(port, GPIO_ODRVR, pin, 0b01);
            gpio_modify_2bit(port, GPIO_PULL, pin, 0b00);
        }

        // ADC clock, sequence of all inputs with 7.5 cycle sampling
        (*ADCCOM::ptr())
            .cctrl()
            .modify(|r, w| w.bits((r.bits() & !(0xF << 16)) | (ADC_DIV_8 << 16)));
        let a = adc();
        a.ctrl1().write(|w| w.bits(CTRL1_SQEN));
        let channels = kb::ADC_MUX_INPUT_CHANNELS
            .iter()
            .chain(kb::ADC_RAW_INPUT_CHANNELS.iter());
        let (mut osq1, mut osq2, mut osq3) = (((NUM_INPUTS as u32) - 1) << 20, 0u32, 0u32);
        let (mut spt1, mut spt2) = (0u32, 0u32);
        for (rank, &ch) in channels.enumerate() {
            let ch = ch as u32;
            match rank {
                0..=5 => osq3 |= ch << (rank * 5),
                6..=11 => osq2 |= ch << ((rank - 6) * 5),
                _ => osq1 |= ch << ((rank - 12) * 5),
            }
            if ch <= 9 {
                spt2 |= SAMPLE_TIME_7_5 << (ch * 3);
            } else {
                spt1 |= SAMPLE_TIME_7_5 << ((ch - 10) * 3);
            }
        }
        a.osq1().write(|w| w.bits(osq1));
        a.osq2().write(|w| w.bits(osq2));
        a.osq3().write(|w| w.bits(osq3));
        a.spt1().write(|w| w.bits(spt1));
        a.spt2().write(|w| w.bits(spt2));
        a.ctrl2()
            .write(|w| w.bits(CTRL2_OCTESEL_SOFTWARE | CTRL2_OCTEN | CTRL2_OCDMAEN));

        // DMA1 channel 1: ADC1 -> ADC_BUFFER, circular, full-transfer interrupt
        let d = dma();
        let ch = d.channel(0);
        ch.ctrl().write(|w| w.bits(0));
        ch.dtcnt().write(|w| w.bits(NUM_INPUTS as u16));
        ch.paddr().write(|w| w.bits(a.odt().as_ptr() as u32));
        ch.maddr().write(|w| w.bits(core::ptr::addr_of_mut!(ADC_BUFFER) as u32));
        ch.ctrl().write(|w| {
            w.bits(DMA_CHPL_HIGH | DMA_MWIDTH_16 | DMA_PWIDTH_16 | DMA_MINCM | DMA_LM | DMA_FDTIEN)
        });
        d.dma_muxsel().write(|w| w.bits(1));
        d.muxc1ctrl().write(|w| w.bits(DMAREQ_ADC1));

        // TMR6: one settle delay per mux step
        let t = tmr6();
        t.div().write(|w| w.bits(0));
        t.pr().write(|w| w.bits(SETTLE_TICKS));
        t.swevt().write(|w| w.bits(TMR_OVF));
        t.ists().write(|w| w.bits(0));
        t.iden().write(|w| w.bits(TMR_OVF));

        let mut nvic = cortex_m::Peripherals::steal().NVIC;
        nvic.set_priority(Interrupt::DMA1_CHANNEL1, 0x10);
        nvic.set_priority(Interrupt::TMR6, 0x10);
        cortex_m::peripheral::NVIC::unmask(Interrupt::DMA1_CHANNEL1);
        cortex_m::peripheral::NVIC::unmask(Interrupt::TMR6);

        // Enable and calibrate the ADC
        a.ctrl2().modify(|r, w| w.bits(r.bits() | CTRL2_ADCEN));
        a.ctrl2().modify(|r, w| w.bits(r.bits() | CTRL2_ADCALINIT));
        while a.ctrl2().read().bits() & CTRL2_ADCALINIT != 0 {}
        a.ctrl2().modify(|r, w| w.bits(r.bits() | CTRL2_ADCAL));
        while a.ctrl2().read().bits() & CTRL2_ADCAL != 0 {}

        ch.ctrl().modify(|r, w| w.bits(r.bits() | DMA_CHEN));
        probe_set(PROBE_SCAN_GPIO, true);
        if NUM_SELECT_PINS > 0 {
            // First conversion after the multiplexer has settled on channel 0
            t.ctrl1().write(|w| w.bits(TMR_EN));
        } else {
            a.ctrl2().modify(|r, w| w.bits(r.bits() | CTRL2_OCSWTRG));
        }
    }
}

#[interrupt]
fn DMA1_CHANNEL1() {
    let d = dma();
    if d.sts().read().bits() & DMA_FDT1_FLAG == 0 {
        return;
    }
    unsafe { d.clr().write(|w| w.bits(DMA_FDT1_FLAG)) };

    let buf = unsafe { core::ptr::addr_of!(ADC_BUFFER).read_volatile() };
    let mux_ch = CURRENT_MUX_CHANNEL.load(Ordering::Relaxed);

    if let Some(keys) = kb::ADC_MUX_INPUT_MATRIX.get(mux_ch as usize) {
        for (&key, &val) in keys.iter().zip(buf.iter()) {
            SAMPLES.store_key(key, val);
        }
    }
    for (&key, &val) in kb::ADC_RAW_INPUT_VECTOR.iter().zip(buf[NUM_MUX_INPUTS..].iter()) {
        SAMPLES.store_key(key, val);
    }
    // Probe: low once this step's samples are stored, high again at the next mux switch.
    probe_set(PROBE_SCAN_GPIO, false);

    if NUM_SELECT_PINS > 0 {
        let next = (mux_ch + 1) & ((1 << NUM_SELECT_PINS) - 1);
        CURRENT_MUX_CHANNEL.store(next, Ordering::Relaxed);
        if next == 0 {
            SAMPLES.generation.fetch_add(1, Ordering::Release);
        }
        set_mux_channel(next);
        probe_set(PROBE_SCAN_GPIO, true);
        // Settle delay, then TMR6 starts the next conversion
        unsafe { tmr6().ctrl1().write(|w| w.bits(TMR_EN)) };
    } else {
        SAMPLES.generation.fetch_add(1, Ordering::Release);
        probe_set(PROBE_SCAN_GPIO, true);
        unsafe { adc().ctrl2().modify(|r, w| w.bits(r.bits() | CTRL2_OCSWTRG)) };
    }
}

#[interrupt]
fn TMR6() {
    let t = tmr6();
    if t.ists().read().bits() & TMR_OVF == 0 {
        return;
    }
    unsafe {
        t.ists().write(|w| w.bits(!TMR_OVF));
        t.ctrl1().write(|w| w.bits(0));
        adc().ctrl2().modify(|r, w| w.bits(r.bits() | CTRL2_OCSWTRG));
    }
}
