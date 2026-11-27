//! Software CRC32 (zlib polynomial `0xEDB88320`), matching C `src/crc32.c` bit-for-bit.

const CRC32_POLY: u32 = 0xEDB8_8320;

#[inline(always)]
fn crc32_update(mut crc: u32, k: u32) -> u32 {
    crc ^= k;
    for _ in 0..32 {
        let mask = (crc & 1).wrapping_neg();
        crc = (crc >> 1) ^ (CRC32_POLY & mask);
    }
    crc
}

/// Compute CRC32 over `buf` seeded with `crc`, matching `libhmk`'s word-wise C CRC32.
pub fn compute(buf: &[u8], crc: u32) -> u32 {
    let mut state = !crc;
    let chunks = buf.chunks_exact(4);
    let rem = chunks.remainder();
    for chunk in chunks {
        let k = u32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
        state = crc32_update(state, k);
    }
    if !rem.is_empty() {
        let mut tail = [0u8; 4];
        tail[..rem.len()].copy_from_slice(rem);
        state = crc32_update(state, u32::from_le_bytes(tail));
    }
    !state
}
