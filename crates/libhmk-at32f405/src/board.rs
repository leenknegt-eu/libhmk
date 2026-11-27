//! AT32F405xx board support: SysTick 1 ms timer, 96-bit UID serial, and bootloader jump.

use core::sync::atomic::{AtomicU32, Ordering};

use cortex_m::peripheral::{syst::SystClkSource, SYST};
use cortex_m_rt::exception;
use libhmk_core::traits::BoardControl;

use crate::kb;

/// Top 4 bytes of AT32F405 102 KB SRAM reserved for bootloader magic handoff.
const BOOTLOADER_FLAG_ADDR: usize = 0x2001_97FC;
const AT32_UID_BASE: usize = 0x1FFF_F7E8;
pub const SYSCLK_HZ: u32 = 216_000_000;

static MS_TICKS: AtomicU32 = AtomicU32::new(0);

#[exception]
fn SysTick() {
    MS_TICKS.fetch_add(1, Ordering::Relaxed);
}

pub struct At32Board;

impl At32Board {
    pub fn check_bootloader_jump() {
        let flag_ptr = BOOTLOADER_FLAG_ADDR as *mut u32;
        let flag = unsafe { core::ptr::read_volatile(flag_ptr) };
        if flag == kb::BOOTLOADER_MAGIC {
            unsafe {
                core::ptr::write_volatile(flag_ptr, 0);
                cortex_m::interrupt::disable();
                cortex_m::asm::bootload(kb::BOOTLOADER_ADDR as *const u32);
            }
        }
    }

    pub fn init_systick(syst: &mut SYST, sysclk_hz: u32) {
        syst.set_clock_source(SystClkSource::Core);
        syst.set_reload((sysclk_hz / 1000) - 1);
        syst.clear_current();
        syst.enable_counter();
        syst.enable_interrupt();
    }
}

impl BoardControl for At32Board {
    fn reset(&mut self) {
        cortex_m::peripheral::SCB::sys_reset();
    }

    fn enter_bootloader(&mut self) {
        unsafe {
            core::ptr::write_volatile(BOOTLOADER_FLAG_ADDR as *mut u32, kb::BOOTLOADER_MAGIC);
        }
        cortex_m::peripheral::SCB::sys_reset();
    }

    fn serial(&self, buf: &mut [u8; 32]) -> usize {
        const HEX: &[u8; 16] = b"0123456789ABCDEF";
        let uid = unsafe { core::slice::from_raw_parts(AT32_UID_BASE as *const u8, 12) };
        for (i, &b) in uid.iter().enumerate() {
            buf[i * 2] = HEX[(b >> 4) as usize];
            buf[i * 2 + 1] = HEX[(b & 0x0F) as usize];
        }
        24
    }

    fn now_ms(&self) -> u32 {
        MS_TICKS.load(Ordering::Relaxed)
    }
}
