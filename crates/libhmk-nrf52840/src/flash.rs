//! nRF52840 internal NVMC flash driver implementing `libhmk_core::traits::FlashStorage`.

use embassy_nrf::nvmc::Nvmc;
use embedded_storage::nor_flash::NorFlash;
use libhmk_core::traits::FlashStorage;
use libhmk_link::LinkSettings;

use crate::kb;

pub struct NrfFlash<'d> {
    nvmc: Nvmc<'d>,
}

impl<'d> NrfFlash<'d> {
    pub fn new(nvmc: Nvmc<'d>) -> Self {
        Self { nvmc }
    }

    /// Wireless link settings from their dedicated page (below the wear-leveling area).
    pub fn read_link_settings(&self) -> Option<LinkSettings> {
        let mut buf = [0u8; LinkSettings::SIZE];
        let src = (kb::FLASH_BASE + kb::LINK_SETTINGS_ADDR) as *const u8;
        unsafe { core::ptr::copy_nonoverlapping(src, buf.as_mut_ptr(), buf.len()) };
        LinkSettings::decode(&buf)
    }

    pub fn write_link_settings(&mut self, settings: &LinkSettings) -> bool {
        let start = kb::LINK_SETTINGS_ADDR;
        self.nvmc.erase(start, start + kb::FLASH_SECTOR_SIZE).is_ok()
            && self.nvmc.write(start, &settings.encode()).is_ok()
    }
}

impl FlashStorage for NrfFlash<'_> {
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
        let start = sector * kb::FLASH_SECTOR_SIZE;
        // Protect application flash below wear-leveling region
        if start < kb::WL_BASE_ADDRESS {
            return false;
        }
        self.nvmc.erase(start, start + kb::FLASH_SECTOR_SIZE).is_ok()
    }

    fn read(&self, addr: u32, buf: &mut [u8]) -> bool {
        if addr.saturating_add(buf.len() as u32) > kb::FLASH_SIZE {
            return false;
        }
        // Safe read-only access to internal memory-mapped flash on nRF52840
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
        self.nvmc.write(addr, data).is_ok()
    }
}
