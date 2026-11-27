//! Flash wear-leveling engine, binary-compatible with C `src/wear_leveling.c`.

use crate::crc32;
use crate::traits::FlashStorage;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum WlStatus {
    Failed,
    Ok,
    Consolidated,
}

/// Virtual EEPROM wear leveling backed by internal flash sectors.
pub struct WearLeveling<const VIRTUAL_SIZE: usize, const WRITE_LOG_SIZE: usize> {
    pub cache: [u8; VIRTUAL_SIZE],
    starting_sector: u32,
    base_address: u32,
    write_address: u32,
}

impl<const VIRTUAL_SIZE: usize, const WRITE_LOG_SIZE: usize>
    WearLeveling<VIRTUAL_SIZE, WRITE_LOG_SIZE>
{
    pub const BACKING_STORE_SIZE: u32 = (VIRTUAL_SIZE + WRITE_LOG_SIZE) as u32;
    const CRC_ADDR: u32 = VIRTUAL_SIZE as u32;
    const LOG_START_ADDR: u32 = Self::CRC_ADDR + 4;
    const MAX_BYTES_PER_ENTRY: usize = if VIRTUAL_SIZE <= 8192 { 6 } else { 5 };

    pub const fn new() -> Self {
        Self {
            cache: [0xFF; VIRTUAL_SIZE],
            starting_sector: 0,
            base_address: 0,
            write_address: Self::LOG_START_ADDR,
        }
    }

    pub fn init<F: FlashStorage>(&mut self, flash: &mut F) -> bool {
        let mut reserved_size = 0u32;
        for i in (0..flash.num_sectors()).rev() {
            reserved_size += flash.sector_size(i);
            if reserved_size >= Self::BACKING_STORE_SIZE {
                self.starting_sector = i;
                break;
            }
        }
        self.base_address = flash.flash_size() - reserved_size;
        self.clear_cache(flash.empty_value());

        let mut status = self.read_consolidated(flash);
        if status != WlStatus::Failed {
            status = self.replay_log(flash);
        } else if self.erase(flash) {
            status = WlStatus::Consolidated;
        } else {
            status = WlStatus::Failed;
        }

        status != WlStatus::Failed
    }

    pub fn erase<F: FlashStorage>(&mut self, flash: &mut F) -> bool {
        self.clear_cache(flash.empty_value());
        self.consolidate_force(flash) != WlStatus::Failed
    }

    pub fn read(&self, addr: usize, dst: &mut [u8]) -> bool {
        if addr + dst.len() > VIRTUAL_SIZE {
            return false;
        }
        dst.copy_from_slice(&self.cache[addr..addr + dst.len()]);
        true
    }

    pub fn write<F: FlashStorage>(&mut self, flash: &mut F, mut addr: usize, mut data: &[u8]) -> bool {
        if addr + data.len() > VIRTUAL_SIZE {
            return false;
        }

        // Trim unchanged leading/trailing bytes to minimize flash wear
        while !data.is_empty() && data[0] == self.cache[addr] {
            data = &data[1..];
            addr += 1;
        }
        while !data.is_empty() && data[data.len() - 1] == self.cache[addr + data.len() - 1] {
            data = &data[..data.len() - 1];
        }
        if data.is_empty() {
            return true;
        }

        self.cache[addr..addr + data.len()].copy_from_slice(data);

        let mut status = self.write_raw(flash, addr, data);
        if status == WlStatus::Ok {
            status = self.consolidate_if_needed(flash);
        }
        status != WlStatus::Failed
    }

    fn clear_cache(&mut self, empty_val: u32) {
        let bytes = empty_val.to_le_bytes();
        for chunk in self.cache.chunks_exact_mut(4) {
            chunk.copy_from_slice(&bytes);
        }
        self.write_address = Self::LOG_START_ADDR;
    }

    fn flash_erase_all<F: FlashStorage>(&self, flash: &mut F) -> bool {
        for sector in self.starting_sector..flash.num_sectors() {
            if !flash.erase_sector(sector) {
                return false;
            }
        }
        true
    }

    fn read_consolidated<F: FlashStorage>(&mut self, flash: &F) -> WlStatus {
        let mut status = WlStatus::Ok;
        if !flash.read(self.base_address, &mut self.cache) {
            status = WlStatus::Failed;
        }
        if status != WlStatus::Failed {
            let expected = crc32::compute(&self.cache, 0);
            let mut crc_bytes = [0u8; 4];
            if !flash.read(self.base_address + Self::CRC_ADDR, &mut crc_bytes)
                || u32::from_le_bytes(crc_bytes) != expected
            {
                status = WlStatus::Failed;
            }
        }
        if status == WlStatus::Failed {
            self.clear_cache(flash.empty_value());
        }
        status
    }

    fn write_consolidated<F: FlashStorage>(&self, flash: &mut F) -> WlStatus {
        if !flash.write(self.base_address, &self.cache) {
            return WlStatus::Failed;
        }
        let checksum = crc32::compute(&self.cache, 0).to_le_bytes();
        if !flash.write(self.base_address + Self::CRC_ADDR, &checksum) {
            return WlStatus::Failed;
        }
        WlStatus::Consolidated
    }

    fn consolidate_force<F: FlashStorage>(&mut self, flash: &mut F) -> WlStatus {
        if !self.flash_erase_all(flash) {
            return WlStatus::Failed;
        }
        let status = self.write_consolidated(flash);
        self.write_address = Self::LOG_START_ADDR;
        status
    }

    fn consolidate_if_needed<F: FlashStorage>(&mut self, flash: &mut F) -> WlStatus {
        if self.write_address >= Self::BACKING_STORE_SIZE {
            return self.consolidate_force(flash);
        }
        WlStatus::Ok
    }

    fn decode_entry_header(word0: u32) -> (usize, usize, [u8; 2]) {
        let bytes = word0.to_le_bytes();
        if VIRTUAL_SIZE <= 8192 {
            let hdr = u16::from_le_bytes([bytes[0], bytes[1]]);
            let addr = (hdr & 0x1FFF) as usize;
            let len = ((hdr >> 13) & 0x07) as usize;
            (addr, len, [bytes[2], bytes[3]])
        } else {
            let hdr = u32::from_le_bytes([bytes[0], bytes[1], bytes[2], 0]);
            let addr = (hdr & 0x3FFF) as usize;
            let len = ((hdr >> 14) & 0x3FF) as usize;
            (addr, len, [bytes[3], 0])
        }
    }

    fn encode_entry(addr: usize, data: &[u8]) -> [u32; 2] {
        let mut raw = [0u8; 8];
        if VIRTUAL_SIZE <= 8192 {
            let hdr = ((addr as u16) & 0x1FFF) | (((data.len() as u16) & 0x07) << 13);
            raw[0..2].copy_from_slice(&hdr.to_le_bytes());
            raw[2..2 + data.len()].copy_from_slice(data);
        } else {
            let hdr = ((addr as u32) & 0x3FFF) | (((data.len() as u32) & 0x3FF) << 14);
            let h = hdr.to_le_bytes();
            raw[0..3].copy_from_slice(&h[0..3]);
            raw[3..3 + data.len()].copy_from_slice(data);
        }
        [
            u32::from_le_bytes([raw[0], raw[1], raw[2], raw[3]]),
            u32::from_le_bytes([raw[4], raw[5], raw[6], raw[7]]),
        ]
    }

    fn replay_log<F: FlashStorage>(&mut self, flash: &mut F) -> WlStatus {
        let mut status = WlStatus::Ok;
        let mut addr = Self::LOG_START_ADDR;
        let empty = flash.empty_value();

        while addr < Self::BACKING_STORE_SIZE {
            let mut w0_bytes = [0u8; 4];
            if !flash.read(self.base_address + addr, &mut w0_bytes) {
                status = WlStatus::Failed;
                break;
            }
            let w0 = u32::from_le_bytes(w0_bytes);
            if w0 == empty {
                break;
            }
            addr += 4;

            let (entry_addr, entry_len, first_bytes) = Self::decode_entry_header(w0);
            if entry_len == 0
                || entry_len > Self::MAX_BYTES_PER_ENTRY
                || entry_addr + entry_len > VIRTUAL_SIZE
            {
                status = WlStatus::Failed;
                break;
            }

            let first_count = if VIRTUAL_SIZE <= 8192 { 2 } else { 1 };
            let mut entry_data = [0u8; 6];
            entry_data[..first_count].copy_from_slice(&first_bytes[..first_count]);

            if entry_len > first_count {
                let mut w1_bytes = [0u8; 4];
                if !flash.read(self.base_address + addr, &mut w1_bytes) {
                    status = WlStatus::Failed;
                    break;
                }
                addr += 4;
                entry_data[first_count..entry_len]
                    .copy_from_slice(&w1_bytes[..entry_len - first_count]);
            }

            self.cache[entry_addr..entry_addr + entry_len]
                .copy_from_slice(&entry_data[..entry_len]);
        }

        self.write_address = addr;
        if status == WlStatus::Failed {
            self.consolidate_force(flash)
        } else {
            self.consolidate_if_needed(flash)
        }
    }

    fn append_word<F: FlashStorage>(&mut self, flash: &mut F, word: u32) -> WlStatus {
        if !flash.write(self.base_address + self.write_address, &word.to_le_bytes()) {
            return WlStatus::Failed;
        }
        self.write_address += 4;
        self.consolidate_if_needed(flash)
    }

    fn write_raw<F: FlashStorage>(&mut self, flash: &mut F, mut addr: usize, mut data: &[u8]) -> WlStatus {
        let first_count = if VIRTUAL_SIZE <= 8192 { 2 } else { 1 };
        while !data.is_empty() {
            let chunk_len = data.len().min(Self::MAX_BYTES_PER_ENTRY);
            let words = Self::encode_entry(addr, &data[..chunk_len]);

            let status = self.append_word(flash, words[0]);
            if status != WlStatus::Ok {
                return status;
            }
            if chunk_len > first_count {
                let status = self.append_word(flash, words[1]);
                if status != WlStatus::Ok {
                    return status;
                }
            }

            addr += chunk_len;
            data = &data[chunk_len..];
        }
        WlStatus::Ok
    }
}
