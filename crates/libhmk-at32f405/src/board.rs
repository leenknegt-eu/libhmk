//! AT32F405xx board support: 216 MHz clock tree, USB clocking, 96-bit UID serial, bootloader jump.
//!
//! Register sequences mirror the C firmware's `board_clock_init()` / `board_usb_init()`
//! (Artery firmware library `crm_*`, `pwc_*` and `flash_*` helpers).

use at32f4xx_pac::at32f405::{CRM, FLASH, PWC};
use libhmk_core::traits::BoardControl;

use crate::kb;

/// Top 4 bytes of AT32F405 102 KB SRAM reserved for bootloader magic handoff.
const BOOTLOADER_FLAG_ADDR: usize = 0x2001_97FC;
const AT32_UID_BASE: usize = 0x1FFF_F7E8;

pub const SYSCLK_HZ: u32 = 216_000_000;
/// APB1 runs at SYSCLK / 2; timers on APB1 are clocked at 2 x PCLK1 = SYSCLK.
pub const APB1_TIMER_HZ: u32 = SYSCLK_HZ;

// CRM_CTRL
const CTRL_HEXTEN: u32 = 1 << 16;
const CTRL_HEXTSTBL: u32 = 1 << 17;
const CTRL_PLLEN: u32 = 1 << 24;
const CTRL_PLLSTBL: u32 = 1 << 25;
const CTRL_PLLUSTBL: u32 = 1 << 26;
// CRM_PLLCFG
const PLLCFG_PLLU_EN: u32 = 1 << 29;
const PLLCFG_PLLRCS_HEXT: u32 = 1 << 30;
// CRM_MISC2
const MISC2_AUTO_STEP_MASK: u32 = 0b11 << 4;
const MISC2_AUTO_STEP_EN: u32 = 0b11 << 4;
const MISC2_USB48_SEL_HICK: u32 = 1 << 10;

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

    /// Configure SYSCLK = 216 MHz from a 12 MHz HEXT (PLL: /1 x72 /4) with PLLU = 48 MHz (/18).
    pub fn init_clocks(crm: &CRM, flash: &FLASH, pwc: &PWC) {
        const _: () = assert!(
            kb::BOARD_HSE_VALUE == 12_000_000,
            "AT32F405 clock tree assumes a 12 MHz HSE (as the C firmware does)"
        );

        unsafe {
            // 6 flash wait states for 193..216 MHz (flash_psr_set: 0x150 | wtcyc)
            flash.psr().write(|w| w.bits(0x150 | 6));

            // LDO 1.3 V for 216 MHz operation
            crm.apb1en().modify(|_, w| w.pwc().set_bit());
            pwc.ldoov().modify(|r, w| w.bits((r.bits() & !0b11) | 0b11));

            // HEXT on
            crm.ctrl().modify(|r, w| w.bits(r.bits() | CTRL_HEXTEN));
            while crm.ctrl().read().bits() & CTRL_HEXTSTBL == 0 {}

            // PLL: source HEXT, MS = 1, NS = 72, FP = /4 (0x02), FU = /18 (0x05)
            crm.pllcfg().write(|w| {
                w.bits(PLLCFG_PLLRCS_HEXT | (72 << 6) | 1 | (0x02 << 16) | (0x05 << 20))
            });
            crm.ctrl().modify(|r, w| w.bits(r.bits() | CTRL_PLLEN));
            while crm.ctrl().read().bits() & CTRL_PLLSTBL == 0 {}

            // AHB /1, APB2 /1, APB1 /2 (0b100)
            crm.cfg()
                .modify(|r, w| w.bits((r.bits() & !(0xF << 4 | 0b111 << 10 | 0b111 << 13)) | (0b100 << 10)));

            // Switch SYSCLK to PLL with auto-step enabled
            crm.misc2()
                .modify(|r, w| w.bits((r.bits() & !MISC2_AUTO_STEP_MASK) | MISC2_AUTO_STEP_EN));
            crm.cfg().modify(|r, w| w.bits((r.bits() & !0b11) | 0b10));
            while (crm.cfg().read().bits() >> 2) & 0b11 != 0b10 {}
            crm.misc2().modify(|r, w| w.bits(r.bits() & !MISC2_AUTO_STEP_MASK));
        }
    }

    /// Enable the USB controller clock and the 48 MHz PLLU path (`board_usb_init`).
    pub fn init_usb_clocks(crm: &CRM) {
        unsafe {
            if kb::USB_HIGH_SPEED {
                crm.ahben1().modify(|_, w| w.gpiob().set_bit().otghs().set_bit());
            } else {
                crm.ahben1().modify(|_, w| w.gpioa().set_bit());
                crm.ahben2().modify(|_, w| w.otgfs1().set_bit());
            }
            crm.pllcfg().modify(|r, w| w.bits(r.bits() | PLLCFG_PLLU_EN));
            while crm.ctrl().read().bits() & CTRL_PLLUSTBL == 0 {}
            // USB 48 MHz clock source = PLLU
            crm.misc2().modify(|r, w| w.bits(r.bits() & !MISC2_USB48_SEL_HICK));
        }
    }

    pub fn format_serial_hex(buf: &mut [u8; 32]) -> usize {
        const HEX: &[u8; 16] = b"0123456789ABCDEF";
        let uid = unsafe { core::slice::from_raw_parts(AT32_UID_BASE as *const u8, 12) };
        for (i, &b) in uid.iter().enumerate() {
            buf[i * 2] = HEX[(b >> 4) as usize];
            buf[i * 2 + 1] = HEX[(b & 0x0F) as usize];
        }
        24
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
        Self::format_serial_hex(buf)
    }

    fn now_ms(&self) -> u32 {
        embassy_time::Instant::now().as_millis() as u32
    }
}
