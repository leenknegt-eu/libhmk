//! nRF52840 board control: FICR device serial, reboot, and Nordic DFU bootloader entry.

use embassy_nrf::pac;
use embassy_time::Instant;
use libhmk_core::traits::BoardControl;

use crate::kb;

pub struct NrfBoard;

impl NrfBoard {
    pub fn format_serial_hex(buf: &mut [u8; 32]) -> usize {
        let id0 = pac::FICR.deviceid(0).read();
        let id1 = pac::FICR.deviceid(1).read();
        let mut bytes = [0u8; 8];
        bytes[0..4].copy_from_slice(&id1.to_be_bytes());
        bytes[4..8].copy_from_slice(&id0.to_be_bytes());

        const HEX: &[u8; 16] = b"0123456789ABCDEF";
        for (i, &b) in bytes.iter().enumerate() {
            buf[i * 2] = HEX[(b >> 4) as usize];
            buf[i * 2 + 1] = HEX[(b & 0x0F) as usize];
        }
        16
    }

    pub fn ble_static_random_address() -> [u8; 6] {
        let id0 = pac::FICR.deviceid(0).read().to_le_bytes();
        let id1 = pac::FICR.deviceid(1).read().to_le_bytes();
        // Top 2 bits set to 11 => BLE Static Random Address
        [id0[0], id0[1], id0[2], id0[3], id1[0], id1[1] | 0xC0]
    }
}

impl BoardControl for NrfBoard {
    fn reset(&mut self) {
        cortex_m::peripheral::SCB::sys_reset();
    }

    fn enter_bootloader(&mut self) {
        // Write Nordic DFU bootloader magic (0xB1) to GPREGRET and reset
        pac::POWER.gpregret().write(|w| w.set_gpregret(kb::BOOTLOADER_MAGIC as u8));
        cortex_m::peripheral::SCB::sys_reset();
    }

    fn serial(&self, buf: &mut [u8; 32]) -> usize {
        Self::format_serial_hex(buf)
    }

    fn now_ms(&self) -> u32 {
        Instant::now().as_millis() as u32
    }
}
