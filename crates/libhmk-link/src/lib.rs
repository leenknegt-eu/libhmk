//! 2.4 GHz keyboard <-> dongle link protocol (carried over Nordic Enhanced ShockBurst).
//!
//! The keyboard is the ESB primary transmitter (PTX) and the dongle the primary receiver (PRX).
//! Every transaction is started by the keyboard; the dongle can only answer inside ACK payloads.
//!
//! * **Upstream** (keyboard → dongle, ESB payload): HID report state, raw HID responses,
//!   heartbeats and pairing requests.
//! * **Downstream** (dongle → keyboard, ESB ACK payload): raw HID requests from the host and
//!   the pairing answer.
//!
//! HID reports are sent as *state* (the full current report), never as deltas, so a lost or
//! repeated packet can always be repaired by sending the latest state again
//! ([`HidFrame::merge`]).
//!
//! This crate is hardware independent and unit tested on the host.

#![cfg_attr(not(test), no_std)]

use libhmk_core::hid::{KeyboardReport, MouseReport, PendingHidReports, RAW_HID_EP_SIZE};

/// Largest ESB payload used by the link (raw HID message = 1 + 64 bytes).
pub const MAX_PAYLOAD: usize = 72;

/// Message types (first payload byte).
pub mod msg {
    /// Upstream: HID report state ([`super::HidFrame`]).
    pub const UP_HID: u8 = 0x01;
    /// Upstream: 64-byte raw HID (configurator) response.
    pub const UP_RAW: u8 = 0x02;
    /// Upstream: heartbeat (battery, flags); also polls for downstream data.
    pub const UP_HEARTBEAT: u8 = 0x03;
    /// Upstream: pairing request (sent on the pairing pipe).
    pub const UP_PAIR_REQUEST: u8 = 0x04;
    /// Downstream: 64-byte raw HID (configurator) request from the host.
    pub const DOWN_RAW: u8 = 0x81;
    /// Downstream: pairing answer carrying the dongle's link address.
    pub const DOWN_PAIR_ACK: u8 = 0x82;
}

/// Link timing (milliseconds).
pub mod timing {
    /// Heartbeat interval while any key/button is held (keeps the dongle's failsafe armed).
    pub const HEARTBEAT_HELD_MS: u32 = 50;
    /// Heartbeat interval while active with nothing held (bounds raw HID request latency).
    pub const HEARTBEAT_ACTIVE_MS: u32 = 100;
    /// Heartbeat interval in the battery Idle/Sleep power states.
    pub const HEARTBEAT_SLEEP_MS: u32 = 1000;
    /// Poll interval right after downstream raw HID traffic (configurator session).
    pub const FAST_POLL_MS: u32 = 2;
    /// How long fast polling continues after the last downstream raw HID message.
    pub const FAST_POLL_WINDOW_MS: u32 = 2000;
    /// The dongle releases all held keys after this long without any packet.
    pub const DONGLE_RELEASE_TIMEOUT_MS: u32 = 250;
    /// The link counts as up while an ACK was received within this window.
    pub const LINK_UP_WINDOW_MS: u32 = 1000;
    /// Pairing window on both sides.
    pub const PAIRING_WINDOW_MS: u32 = 30_000;
    /// Interval between pairing requests from the keyboard.
    pub const PAIR_REQUEST_INTERVAL_MS: u32 = 10;
}

// --------------------------------------------------------------------------------------------
// Addresses
// --------------------------------------------------------------------------------------------

/// ESB address of one pipe: 4-byte base + 1-byte prefix (5-byte on-air address).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Address {
    pub base: [u8; 4],
    pub prefix: u8,
}

/// ESB pipe used for normal traffic (dongle-unique address).
pub const PIPE_LINK: u8 = 0;
/// ESB pipe used for pairing (well-known [`PAIRING_ADDRESS`]).
pub const PIPE_PAIRING: u8 = 1;

/// Well-known address both sides use while pairing.
pub const PAIRING_ADDRESS: Address = Address {
    base: [0x9A, 0x3C, 0x5E, 0x71],
    prefix: 0xA3,
};

/// Placeholder link address of an unpaired keyboard (never used by a dongle).
pub const UNPAIRED_ADDRESS: Address = Address {
    base: [0x6D, 0xB4, 0x2B, 0x96],
    prefix: 0x4E,
};

/// Bytes the radio's address correlator handles poorly (no bit transitions / pure alternation).
const fn weak_byte(b: u8) -> bool {
    matches!(b, 0x00 | 0xFF | 0x55 | 0xAA)
}

/// Derive a dongle's unique link address from its 64-bit factory device ID (FICR DEVICEID).
///
/// The result avoids weak correlator bytes and never collides with [`PAIRING_ADDRESS`] or
/// [`UNPAIRED_ADDRESS`].
pub fn dongle_address(device_id: [u8; 8]) -> Address {
    // FNV-1a over the device ID, then a splitmix-style finalizer to spread the bits.
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for b in device_id {
        h ^= b as u64;
        h = h.wrapping_mul(0x0000_0100_0000_01b3);
    }
    let mut attempt = 0u64;
    loop {
        let mut z = h.wrapping_add(attempt.wrapping_mul(0x9e37_79b9_7f4a_7c15));
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        z ^= z >> 31;
        let bytes = z.to_le_bytes();
        let addr = Address {
            base: [bytes[0], bytes[1], bytes[2], bytes[3]],
            prefix: bytes[4],
        };
        let ok = !weak_byte(addr.prefix)
            && !weak_byte(addr.base[0])
            && addr != PAIRING_ADDRESS
            && addr != UNPAIRED_ADDRESS;
        if ok {
            return addr;
        }
        attempt += 1;
    }
}

// --------------------------------------------------------------------------------------------
// HID state frames
// --------------------------------------------------------------------------------------------

const HID_FLAG_KEYBOARD: u8 = 1 << 0;
const HID_FLAG_SYSTEM: u8 = 1 << 1;
const HID_FLAG_CONSUMER: u8 = 1 << 2;
const HID_FLAG_MOUSE: u8 = 1 << 3;

/// HID report state carried upstream. Each present field is the complete current report.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct HidFrame {
    pub keyboard: Option<[u8; KeyboardReport::SIZE]>,
    pub system: Option<u16>,
    pub consumer: Option<u16>,
    pub mouse: Option<[u8; MouseReport::SIZE]>,
}

impl HidFrame {
    /// Largest encoded size (type + flags + all reports).
    pub const MAX_SIZE: usize = 2 + KeyboardReport::SIZE + 2 + 2 + MouseReport::SIZE;

    pub fn from_pending(p: &PendingHidReports) -> Self {
        Self {
            keyboard: p.keyboard.map(|r| r.to_bytes()),
            system: p.system,
            consumer: p.consumer,
            mouse: p.mouse.map(|m| m.to_bytes()),
        }
    }

    pub fn is_empty(&self) -> bool {
        self.keyboard.is_none()
            && self.system.is_none()
            && self.consumer.is_none()
            && self.mouse.is_none()
    }

    /// Fold `newer` into `self`: newer report state replaces older state per report.
    pub fn merge(&mut self, newer: &HidFrame) {
        if newer.keyboard.is_some() {
            self.keyboard = newer.keyboard;
        }
        if newer.system.is_some() {
            self.system = newer.system;
        }
        if newer.consumer.is_some() {
            self.consumer = newer.consumer;
        }
        if newer.mouse.is_some() {
            self.mouse = newer.mouse;
        }
    }

    /// Re-queue a frame that failed to send under state that was produced after it.
    pub fn merge_older(&mut self, older: &HidFrame) {
        let mut combined = *older;
        combined.merge(self);
        *self = combined;
    }

    pub fn encode(&self, out: &mut [u8]) -> usize {
        let mut flags = 0u8;
        let mut n = 2;
        if let Some(kb) = &self.keyboard {
            flags |= HID_FLAG_KEYBOARD;
            out[n..n + kb.len()].copy_from_slice(kb);
            n += kb.len();
        }
        if let Some(sys) = self.system {
            flags |= HID_FLAG_SYSTEM;
            out[n..n + 2].copy_from_slice(&sys.to_le_bytes());
            n += 2;
        }
        if let Some(cons) = self.consumer {
            flags |= HID_FLAG_CONSUMER;
            out[n..n + 2].copy_from_slice(&cons.to_le_bytes());
            n += 2;
        }
        if let Some(m) = &self.mouse {
            flags |= HID_FLAG_MOUSE;
            out[n..n + m.len()].copy_from_slice(m);
            n += m.len();
        }
        out[0] = msg::UP_HID;
        out[1] = flags;
        n
    }

    /// Decode the body of an [`msg::UP_HID`] message (`data` excludes the type byte).
    fn decode_body(data: &[u8]) -> Option<Self> {
        let (&flags, mut rest) = data.split_first()?;
        if flags & !(HID_FLAG_KEYBOARD | HID_FLAG_SYSTEM | HID_FLAG_CONSUMER | HID_FLAG_MOUSE) != 0
        {
            return None;
        }
        let mut take = |n: usize| -> Option<&[u8]> {
            if rest.len() < n {
                return None;
            }
            let (head, tail) = rest.split_at(n);
            rest = tail;
            Some(head)
        };
        let mut frame = HidFrame::default();
        if flags & HID_FLAG_KEYBOARD != 0 {
            frame.keyboard = Some(take(KeyboardReport::SIZE)?.try_into().ok()?);
        }
        if flags & HID_FLAG_SYSTEM != 0 {
            frame.system = Some(u16::from_le_bytes(take(2)?.try_into().ok()?));
        }
        if flags & HID_FLAG_CONSUMER != 0 {
            frame.consumer = Some(u16::from_le_bytes(take(2)?.try_into().ok()?));
        }
        if flags & HID_FLAG_MOUSE != 0 {
            frame.mouse = Some(take(MouseReport::SIZE)?.try_into().ok()?);
        }
        if !rest.is_empty() {
            return None;
        }
        Some(frame)
    }
}

/// Whether any key, button or usage is held in the given report state.
pub fn reports_held(
    keyboard: &[u8; KeyboardReport::SIZE],
    system: u16,
    consumer: u16,
    mouse_buttons: u8,
) -> bool {
    keyboard.iter().any(|&b| b != 0) || system != 0 || consumer != 0 || mouse_buttons != 0
}

// --------------------------------------------------------------------------------------------
// Messages
// --------------------------------------------------------------------------------------------

/// Heartbeat flag: some key/button is currently held.
pub const HEARTBEAT_FLAG_HELD: u8 = 1 << 0;

/// Upstream message (keyboard → dongle).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Upstream {
    Hid(HidFrame),
    Raw([u8; RAW_HID_EP_SIZE]),
    Heartbeat { battery_percent: u8, flags: u8 },
    PairRequest { device_id: [u8; 8] },
}

impl Upstream {
    pub fn encode(&self, out: &mut [u8; MAX_PAYLOAD]) -> usize {
        match self {
            Upstream::Hid(frame) => frame.encode(out),
            Upstream::Raw(data) => {
                out[0] = msg::UP_RAW;
                out[1..1 + RAW_HID_EP_SIZE].copy_from_slice(data);
                1 + RAW_HID_EP_SIZE
            }
            Upstream::Heartbeat { battery_percent, flags } => {
                out[0] = msg::UP_HEARTBEAT;
                out[1] = *battery_percent;
                out[2] = *flags;
                3
            }
            Upstream::PairRequest { device_id } => {
                out[0] = msg::UP_PAIR_REQUEST;
                out[1..9].copy_from_slice(device_id);
                9
            }
        }
    }

    pub fn decode(data: &[u8]) -> Option<Self> {
        let (&kind, body) = data.split_first()?;
        match kind {
            msg::UP_HID => HidFrame::decode_body(body).map(Upstream::Hid),
            msg::UP_RAW => Some(Upstream::Raw(body.try_into().ok()?)),
            msg::UP_HEARTBEAT if body.len() == 2 => Some(Upstream::Heartbeat {
                battery_percent: body[0],
                flags: body[1],
            }),
            msg::UP_PAIR_REQUEST => Some(Upstream::PairRequest {
                device_id: body.try_into().ok()?,
            }),
            _ => None,
        }
    }
}

/// Downstream message (dongle → keyboard, inside an ESB ACK payload).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Downstream {
    Raw([u8; RAW_HID_EP_SIZE]),
    PairAck { address: Address },
}

impl Downstream {
    pub fn encode(&self, out: &mut [u8; MAX_PAYLOAD]) -> usize {
        match self {
            Downstream::Raw(data) => {
                out[0] = msg::DOWN_RAW;
                out[1..1 + RAW_HID_EP_SIZE].copy_from_slice(data);
                1 + RAW_HID_EP_SIZE
            }
            Downstream::PairAck { address } => {
                out[0] = msg::DOWN_PAIR_ACK;
                out[1..5].copy_from_slice(&address.base);
                out[5] = address.prefix;
                6
            }
        }
    }

    /// Decode an ACK payload; an empty ACK payload decodes to `None`.
    pub fn decode(data: &[u8]) -> Option<Self> {
        let (&kind, body) = data.split_first()?;
        match kind {
            msg::DOWN_RAW => Some(Downstream::Raw(body.try_into().ok()?)),
            msg::DOWN_PAIR_ACK if body.len() == 5 => Some(Downstream::PairAck {
                address: Address {
                    base: [body[0], body[1], body[2], body[3]],
                    prefix: body[4],
                },
            }),
            _ => None,
        }
    }
}

// --------------------------------------------------------------------------------------------
// Persistent keyboard link settings
// --------------------------------------------------------------------------------------------

/// Wireless backend a keyboard boots into.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Backend {
    /// 2.4 GHz Enhanced ShockBurst to a USB dongle.
    Esb = 0,
    /// Bluetooth Low Energy (HOGP).
    Ble = 1,
}

/// Preferred output transport (persisted so a backend switch, which reboots, keeps it).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TransportPref {
    /// USB while enumerated, wireless otherwise.
    Auto = 0,
    /// Always USB.
    Usb = 1,
    /// Always wireless, even while USB is enumerated.
    Wireless = 2,
}

/// Keyboard-side link settings stored in their own flash page (outside `eeconfig`, so the
/// configurator layout stays unchanged).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LinkSettings {
    pub backend: Backend,
    pub transport: TransportPref,
    /// Address of the paired dongle, if any.
    pub paired: Option<Address>,
}

impl LinkSettings {
    /// Encoded size; a multiple of 4 for word-wise flash programming.
    pub const SIZE: usize = 16;
    const MAGIC: [u8; 4] = *b"HMKL";
    const VERSION: u8 = 1;

    pub const fn new(backend: Backend) -> Self {
        Self {
            backend,
            transport: TransportPref::Auto,
            paired: None,
        }
    }

    pub fn encode(&self) -> [u8; Self::SIZE] {
        let mut out = [0u8; Self::SIZE];
        out[0..4].copy_from_slice(&Self::MAGIC);
        out[4] = Self::VERSION;
        out[5] = self.backend as u8;
        out[12] = self.transport as u8;
        if let Some(a) = self.paired {
            out[6] = 1;
            out[7..11].copy_from_slice(&a.base);
            out[11] = a.prefix;
        }
        let crc = crc16(&out[..14]);
        out[14..16].copy_from_slice(&crc.to_le_bytes());
        out
    }

    /// Decode a stored record; erased or corrupted flash yields `None`.
    pub fn decode(b: &[u8]) -> Option<Self> {
        if b.len() < Self::SIZE || b[0..4] != Self::MAGIC || b[4] != Self::VERSION {
            return None;
        }
        if crc16(&b[..14]).to_le_bytes() != [b[14], b[15]] {
            return None;
        }
        let backend = match b[5] {
            0 => Backend::Esb,
            1 => Backend::Ble,
            _ => return None,
        };
        let transport = match b[12] {
            0 => TransportPref::Auto,
            1 => TransportPref::Usb,
            2 => TransportPref::Wireless,
            _ => return None,
        };
        let paired = match b[6] {
            0 => None,
            1 => Some(Address {
                base: [b[7], b[8], b[9], b[10]],
                prefix: b[11],
            }),
            _ => return None,
        };
        Some(Self {
            backend,
            transport,
            paired,
        })
    }
}

/// CRC-16/CCITT-FALSE.
fn crc16(data: &[u8]) -> u16 {
    let mut crc = 0xFFFFu16;
    for &b in data {
        crc ^= (b as u16) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 { (crc << 1) ^ 0x1021 } else { crc << 1 };
        }
    }
    crc
}

// --------------------------------------------------------------------------------------------
// Keyboard-side link scheduling
// --------------------------------------------------------------------------------------------

/// Keyboard-side link bookkeeping: what to send next and when (pure logic, no I/O).
#[derive(Debug, Clone, Default)]
pub struct PtxScheduler {
    pending_hid: HidFrame,
    in_flight_hid: Option<HidFrame>,
    held: bool,
    last_tx_ms: u32,
    last_ack_ms: Option<u32>,
    fast_poll_until_ms: Option<u32>,
}

impl PtxScheduler {
    pub const fn new() -> Self {
        Self {
            pending_hid: HidFrame {
                keyboard: None,
                system: None,
                consumer: None,
                mouse: None,
            },
            in_flight_hid: None,
            held: false,
            last_tx_ms: 0,
            last_ack_ms: None,
            fast_poll_until_ms: None,
        }
    }

    /// Queue new HID state from the firmware.
    pub fn push_hid(&mut self, frame: &HidFrame, held: bool) {
        self.pending_hid.merge(frame);
        self.held = held;
    }

    pub fn has_pending_hid(&self) -> bool {
        !self.pending_hid.is_empty()
    }

    /// Take the pending HID state for transmission (kept until [`Self::on_result`]).
    pub fn take_hid(&mut self) -> Option<HidFrame> {
        if self.pending_hid.is_empty() {
            return None;
        }
        let frame = core::mem::take(&mut self.pending_hid);
        self.in_flight_hid = Some(frame);
        Some(frame)
    }

    /// Record a transmission at `now_ms`.
    pub fn on_sent(&mut self, now_ms: u32) {
        self.last_tx_ms = now_ms;
    }

    /// Report the outcome of the last transmission. A failed HID frame is re-queued beneath
    /// any newer state.
    pub fn on_result(&mut self, now_ms: u32, acked: bool) {
        if acked {
            self.last_ack_ms = Some(now_ms);
            self.in_flight_hid = None;
        } else if let Some(failed) = self.in_flight_hid.take() {
            self.pending_hid.merge_older(&failed);
        }
    }

    /// Note downstream raw HID traffic: poll fast for a while to keep a configurator snappy.
    pub fn on_downstream_raw(&mut self, now_ms: u32) {
        self.fast_poll_until_ms = Some(now_ms.wrapping_add(timing::FAST_POLL_WINDOW_MS));
    }

    pub fn link_up(&self, now_ms: u32) -> bool {
        self.last_ack_ms
            .is_some_and(|t| now_ms.wrapping_sub(t) < timing::LINK_UP_WINDOW_MS)
    }

    /// Current poll/heartbeat interval.
    pub fn poll_interval_ms(&self, now_ms: u32, low_power: bool) -> u32 {
        if let Some(until) = self.fast_poll_until_ms {
            if (until.wrapping_sub(now_ms) as i32) > 0 {
                return timing::FAST_POLL_MS;
            }
        }
        if self.held {
            timing::HEARTBEAT_HELD_MS
        } else if low_power {
            timing::HEARTBEAT_SLEEP_MS
        } else {
            timing::HEARTBEAT_ACTIVE_MS
        }
    }

    /// Milliseconds until the next heartbeat/poll is due (0 = now).
    pub fn heartbeat_due_in_ms(&self, now_ms: u32, low_power: bool) -> u32 {
        let elapsed = now_ms.wrapping_sub(self.last_tx_ms);
        self.poll_interval_ms(now_ms, low_power).saturating_sub(elapsed)
    }

    pub fn held(&self) -> bool {
        self.held
    }
}

// --------------------------------------------------------------------------------------------
// Dongle-side failsafe
// --------------------------------------------------------------------------------------------

/// Dongle-side stuck-key protection: release everything if the keyboard goes silent while
/// something is held (e.g. out of range or battery died mid-press).
#[derive(Debug, Clone, Default)]
pub struct ReleaseGuard {
    held: bool,
    last_rx_ms: u32,
}

impl ReleaseGuard {
    pub const fn new() -> Self {
        Self { held: false, last_rx_ms: 0 }
    }

    /// Any packet received from the keyboard.
    pub fn on_packet(&mut self, now_ms: u32) {
        self.last_rx_ms = now_ms;
    }

    /// Current "something held" state, from forwarded reports or heartbeats.
    pub fn set_held(&mut self, held: bool) {
        self.held = held;
    }

    /// Returns `true` exactly once when held keys must be released.
    pub fn poll(&mut self, now_ms: u32) -> bool {
        if self.held && now_ms.wrapping_sub(self.last_rx_ms) >= timing::DONGLE_RELEASE_TIMEOUT_MS {
            self.held = false;
            return true;
        }
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn kb(first: u8) -> [u8; KeyboardReport::SIZE] {
        let mut r = [0u8; KeyboardReport::SIZE];
        r[2] = first;
        r
    }

    #[test]
    fn hid_frame_roundtrip_all_combinations() {
        for mask in 0u8..16 {
            let frame = HidFrame {
                keyboard: (mask & 1 != 0).then(|| kb(0x04)),
                system: (mask & 2 != 0).then_some(0x0081),
                consumer: (mask & 4 != 0).then_some(0x00E9),
                mouse: (mask & 8 != 0).then_some([1, 2, 0xFE, 4, 5]),
            };
            let mut buf = [0u8; MAX_PAYLOAD];
            let n = Upstream::Hid(frame).encode(&mut buf);
            assert!(n <= HidFrame::MAX_SIZE);
            assert_eq!(Upstream::decode(&buf[..n]), Some(Upstream::Hid(frame)));
            // Truncated or padded frames are rejected.
            if n > 2 {
                assert_eq!(Upstream::decode(&buf[..n - 1]), None);
            }
            assert_eq!(Upstream::decode(&buf[..n + 1]), None);
        }
    }

    #[test]
    fn messages_roundtrip_and_fit_payload() {
        let mut raw = [0u8; RAW_HID_EP_SIZE];
        raw.iter_mut().enumerate().for_each(|(i, b)| *b = i as u8);
        let ups = [
            Upstream::Raw(raw),
            Upstream::Heartbeat { battery_percent: 87, flags: HEARTBEAT_FLAG_HELD },
            Upstream::PairRequest { device_id: [1, 2, 3, 4, 5, 6, 7, 8] },
        ];
        for m in ups {
            let mut buf = [0u8; MAX_PAYLOAD];
            let n = m.encode(&mut buf);
            assert!(n <= MAX_PAYLOAD);
            assert_eq!(Upstream::decode(&buf[..n]), Some(m));
        }
        let downs = [
            Downstream::Raw(raw),
            Downstream::PairAck { address: dongle_address([9; 8]) },
        ];
        for m in downs {
            let mut buf = [0u8; MAX_PAYLOAD];
            let n = m.encode(&mut buf);
            assert_eq!(Downstream::decode(&buf[..n]), Some(m));
        }
        assert_eq!(Downstream::decode(&[]), None, "empty ACK payload");
        assert_eq!(Upstream::decode(&[0x7F, 1, 2]), None, "unknown type");
        assert!(HidFrame::MAX_SIZE <= MAX_PAYLOAD);
    }

    #[test]
    fn dongle_addresses_are_unique_and_strong() {
        let mut seen = std::collections::HashSet::new();
        for i in 0u32..5000 {
            let mut id = [0u8; 8];
            id[..4].copy_from_slice(&i.to_le_bytes());
            let a = dongle_address(id);
            assert!(!weak_byte(a.prefix) && !weak_byte(a.base[0]));
            assert_ne!(a, PAIRING_ADDRESS);
            assert_ne!(a, UNPAIRED_ADDRESS);
            seen.insert((a.base, a.prefix));
        }
        assert_eq!(seen.len(), 5000, "address collision");
        assert_eq!(dongle_address([7; 8]), dongle_address([7; 8]), "deterministic");
    }

    #[test]
    fn link_settings_roundtrip_and_corruption() {
        for s in [
            LinkSettings::new(Backend::Esb),
            LinkSettings {
                backend: Backend::Ble,
                transport: TransportPref::Wireless,
                paired: Some(dongle_address([3; 8])),
            },
        ] {
            let mut b = s.encode();
            assert_eq!(LinkSettings::decode(&b), Some(s));
            b[8] ^= 1;
            assert_eq!(LinkSettings::decode(&b), None, "CRC must catch corruption");
        }
        assert_eq!(LinkSettings::decode(&[0xFF; 16]), None, "erased flash");
    }

    #[test]
    fn scheduler_coalesces_and_requeues_failed_state() {
        let mut s = PtxScheduler::new();
        s.push_hid(&HidFrame { keyboard: Some(kb(4)), ..Default::default() }, true);
        let sent = s.take_hid().unwrap();
        s.on_sent(10);
        // Newer state arrives while the old frame is in flight.
        s.push_hid(&HidFrame { consumer: Some(0xE9), ..Default::default() }, true);
        s.on_result(11, false);
        // Failed keyboard report is re-queued, newer consumer state kept.
        let retry = s.take_hid().unwrap();
        assert_eq!(retry.keyboard, sent.keyboard);
        assert_eq!(retry.consumer, Some(0xE9));
        // A newer keyboard report must win over the re-queued failed one.
        s.push_hid(&HidFrame { keyboard: Some(kb(5)), ..Default::default() }, true);
        s.on_result(12, false);
        assert_eq!(s.take_hid().unwrap().keyboard, Some(kb(5)));
        s.on_result(13, true);
        assert!(!s.has_pending_hid());
        assert!(s.link_up(13));
        assert!(!s.link_up(13 + timing::LINK_UP_WINDOW_MS));
    }

    #[test]
    fn scheduler_poll_intervals() {
        let mut s = PtxScheduler::new();
        assert_eq!(s.poll_interval_ms(0, false), timing::HEARTBEAT_ACTIVE_MS);
        assert_eq!(s.poll_interval_ms(0, true), timing::HEARTBEAT_SLEEP_MS);
        s.push_hid(&HidFrame::default(), true);
        assert_eq!(s.poll_interval_ms(0, true), timing::HEARTBEAT_HELD_MS);
        s.on_downstream_raw(100);
        assert_eq!(s.poll_interval_ms(150, false), timing::FAST_POLL_MS);
        assert_eq!(
            s.poll_interval_ms(100 + timing::FAST_POLL_WINDOW_MS, false),
            timing::HEARTBEAT_HELD_MS
        );
        // Past the fast-poll window: held interval, counted from the last transmission.
        s.on_sent(3000);
        assert_eq!(s.heartbeat_due_in_ms(3010, false), timing::HEARTBEAT_HELD_MS - 10);
        assert_eq!(s.heartbeat_due_in_ms(5000, false), 0);
    }

    #[test]
    fn release_guard_fires_once_after_silence() {
        let mut g = ReleaseGuard::new();
        g.on_packet(0);
        g.set_held(true);
        assert!(!g.poll(timing::DONGLE_RELEASE_TIMEOUT_MS - 1));
        assert!(g.poll(timing::DONGLE_RELEASE_TIMEOUT_MS));
        assert!(!g.poll(timing::DONGLE_RELEASE_TIMEOUT_MS + 10), "only once");
        g.on_packet(1000);
        g.set_held(false);
        assert!(!g.poll(5000), "nothing held, nothing to release");
    }
}
