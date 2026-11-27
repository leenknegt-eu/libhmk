//! Fast integer square root routines ported from `include/lib/usqrt.h`.

#[inline]
pub fn usqrt16(mut x: u16) -> u16 {
    let mut g: u16 = 0;
    for n in (0..8).rev() {
        let t = (g << (n + 1)) + (1u16 << (n << 1));
        if x >= t {
            g += 1u16 << n;
            x -= t;
        }
    }
    g
}

#[inline]
pub fn usqrt32(mut x: u32) -> u32 {
    let mut g: u32 = 0;
    for n in (0..16).rev() {
        let t = (g << (n + 1)) + (1u32 << (n << 1));
        if x >= t {
            g += 1u32 << n;
            x -= t;
        }
    }
    g
}
