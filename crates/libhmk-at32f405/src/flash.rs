//! AT32F405xx internal flash driver (uniform 2 KB sectors) via `at32f4xx-pac`.

use at32f4xx_pac::at32f405::FLASH;
use libhmk_core::traits::FlashStorage;

use crate::kb;

const FLASH_UNLOCK_KEY1: u32 = 0x4567_0123;
const FLASH_UNLOCK_KEY2: u32 = 0xCDEF_89AB;

pub struct At32Flash {
    flash: FLASH,
}

impl At32Flash {
    pub fn new(flash: FLASH) -> Self {
        Self { flash }
    }

    fn unlock(&mut self) {
        self.flash.unlock().write(|w| unsafe { w.bits(FLASH_UNLOCK_KEY1) });
        self.flash.unlock().write(|w| unsafe { w.bits(FLASH_UNLOCK_KEY2) });
    }

    fn lock(&mut self) {
        self.flash.ctrl().modify(|_, w| w.oplk().set_bit());
    }

    fn wait_ready(&self) {
        while self.flash.sts().read().obf().bit_is_set() {}
    }
}

impl FlashStorage for At32Flash {
    fn flash_size(&self) -> u32 {
        kb::FLASH_SIZE
    }

    fn num_sectors(&self) -> u32 {
        kb::FLASH_NUM_SECTORS
    }

    fn empty_value(&self) -> u32 {
        kb::FLASH_EMPTY_VAL
    }

    fn sector_size(&self, sector: u32) -> u32 {
        if sector < kb::FLASH_NUM_SECTORS {
            kb::FLASH_SECTOR_SIZE
        } else {
            0
        }
    }

    fn erase_sector(&mut self, sector: u32) -> bool {
        if sector >= kb::FLASH_NUM_SECTORS {
            return false;
        }
        let offset = sector * kb::FLASH_SECTOR_SIZE;
        if offset < kb::WL_BASE_ADDRESS {
            return false;
        }
        self.unlock();
        self.wait_ready();
        self.flash.ctrl().modify(|_, w| w.secers().set_bit());
        self.flash
            .addr()
            .write(|w| unsafe { w.bits(kb::FLASH_BASE + offset) });
        self.flash.ctrl().modify(|_, w| w.erstr().set_bit());
        self.wait_ready();
        self.flash.ctrl().modify(|_, w| w.secers().clear_bit());
        self.lock();
        true
    }

    fn read(&self, addr: u32, buf: &mut [u8]) -> bool {
        if addr.saturating_add(buf.len() as u32) > kb::FLASH_SIZE {
            return false;
        }
        let src = (kb::FLASH_BASE + addr) as *const u8;
        unsafe {
            core::ptr::copy_nonoverlapping(src, buf.as_mut_ptr(), buf.len());
        }
        true
    }

    fn write(&mut self, addr: u32, data: &[u8]) -> bool {
        if addr < kb::WL_BASE_ADDRESS
            || addr.saturating_add(data.len() as u32) > kb::FLASH_SIZE
            || (addr & 3) != 0
            || (data.len() & 3) != 0
        {
            return false;
        }
        self.unlock();
        self.wait_ready();
        self.flash.ctrl().modify(|_, w| w.fprgm().set_bit());

        for (i, chunk) in data.chunks_exact(4).enumerate() {
            let word = u32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
            let dst = (kb::FLASH_BASE + addr + (i as u32) * 4) as *mut u32;
            unsafe {
                core::ptr::write_volatile(dst, word);
            }
            self.wait_ready();
        }

        self.flash.ctrl().modify(|_, w| w.fprgm().clear_bit());
        self.lock();
        true
    }
}
