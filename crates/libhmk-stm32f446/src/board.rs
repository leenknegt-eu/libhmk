//! STM32F446xx board control: 96-bit UID serial, reset, and system DFU bootloader jump.

use embassy_time::Instant;
use libhmk_core::traits::BoardControl;

use crate::kb;

/// Top 4 bytes of STM32F446 128 KB SRAM reserved for bootloader magic handoff.
const BOOTLOADER_FLAG_ADDR: usize = 0x2001_FFFC;
const STM32_UID_BASE: usize = 0x1FFF_7A10;

pub struct Stm32Board;

impl Stm32Board {
    /// Call at the very beginning of `main()` before clock initialization to jump to
    /// STM32 system DFU bootloader if requested prior to reset.
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

    pub fn format_serial_hex(buf: &mut [u8; 32]) -> usize {
        const HEX: &[u8; 16] = b"0123456789ABCDEF";
        let uid = unsafe { core::slice::from_raw_parts(STM32_UID_BASE as *const u8, 12) };
        for (i, &b) in uid.iter().enumerate() {
            buf[i * 2] = HEX[(b >> 4) as usize];
            buf[i * 2 + 1] = HEX[(b & 0x0F) as usize];
        }
        24
    }
}

impl BoardControl for Stm32Board {
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
        Self::format_serial_hex(buf)
    }

    fn now_ms(&self) -> u32 {
        Instant::now().as_millis() as u32
    }
}
