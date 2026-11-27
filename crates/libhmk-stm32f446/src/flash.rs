//! STM32F446xx non-uniform sector internal flash driver implementing `FlashStorage`.

use embassy_stm32::flash::{Blocking, Flash};
use embedded_storage::nor_flash::NorFlash;
use libhmk_core::traits::FlashStorage;

use crate::kb;

pub struct Stm32Flash<'d> {
    flash: Flash<'d, Blocking>,
}

impl<'d> Stm32Flash<'d> {
    pub fn new(flash: Flash<'d, Blocking>) -> Self {
        Self { flash }
    }

    fn sector_offset(sector: u32) -> Option<(u32, u32)> {
        let mut off = 0u32;
        for (idx, &size) in kb::FLASH_SECTOR_SIZES.iter().enumerate() {
            if idx as u32 == sector {
                return Some((off, size));
            }
            off += size;
        }
        None
    }
}

impl FlashStorage for Stm32Flash<'_> {
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
        kb::FLASH_SECTOR_SIZES
            .get(sector as usize)
            .copied()
            .unwrap_or(0)
    }

    fn erase_sector(&mut self, sector: u32) -> bool {
        let Some((start, size)) = Self::sector_offset(sector) else {
            return false;
        };
        if start < kb::WL_BASE_ADDRESS {
            return false;
        }
        self.flash.erase(start, start + size).is_ok()
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
        self.flash.write(addr, data).is_ok()
    }
}
