//! Fixed-size per-key bit set (up to 256 keys: key indices are `u8` on the wire).
//!
//! Lets the layout engine detect press/release edges with a few word operations per scan
//! instead of one branchy comparison per key.

pub const MAX_KEYS: usize = 256;
pub const KEY_MASK_WORDS: usize = MAX_KEYS / 32;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct KeyMask {
    pub words: [u32; KEY_MASK_WORDS],
}

impl KeyMask {
    pub const fn new() -> Self {
        Self { words: [0; KEY_MASK_WORDS] }
    }

    /// Number of words needed to cover `num_keys` keys.
    pub const fn words_for(num_keys: usize) -> usize {
        num_keys.div_ceil(32)
    }

    #[inline]
    pub fn get(&self, key: usize) -> bool {
        (self.words[key >> 5] >> (key & 31)) & 1 != 0
    }

    #[inline]
    pub fn set(&mut self, key: usize, value: bool) {
        let bit = 1u32 << (key & 31);
        let w = &mut self.words[key >> 5];
        if value {
            *w |= bit;
        } else {
            *w &= !bit;
        }
    }

    #[inline]
    pub fn toggle(&mut self, key: usize) {
        self.words[key >> 5] ^= 1u32 << (key & 31);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn set_get_toggle_across_words() {
        let mut m = KeyMask::new();
        for k in [0usize, 31, 32, 66, 255] {
            assert!(!m.get(k));
            m.set(k, true);
            assert!(m.get(k));
        }
        m.toggle(32);
        assert!(!m.get(32));
        m.set(0, false);
        assert_eq!(m.words[0], 1 << 31);
        assert_eq!(m.words[2], 1 << 2);
        assert_eq!(m.words[7], 1 << 31);
        assert_eq!(KeyMask::words_for(67), 3);
        assert_eq!(KeyMask::words_for(64), 2);
    }
}
