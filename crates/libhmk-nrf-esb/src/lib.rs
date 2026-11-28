//! Minimal, interrupt-driven Nordic **Enhanced ShockBurst** (ESB) driver for the nRF52840.
//!
//! * 2 Mbit/s, dynamic payload length (up to [`MAX_PAYLOAD`] bytes), 2-bit PID, 16-bit CRC,
//!   5-byte addresses, automatic acknowledgement with ACK payloads, retransmissions.
//! * Register configuration and on-air format follow Nordic's `nrf_esb` (`ESB_PROTOCOL_ESB_DPL`,
//!   8-bit length field, bit-reversed addresses), aiming to be compatible with it. Compatibility
//!   with Nordic's implementation has **not** been verified on hardware.
//! * Timing-critical work (TX→RX turnaround, ACK payload selection) happens in the `RADIO`
//!   interrupt using the radio's hardware shortcuts; the application side is `async`.
//!
//! # Usage
//!
//! The firmware owns the interrupt vectors and must forward them (both at the highest priority,
//! which [`Ptx::new`] / [`Prx::new`] configure):
//!
//! ```ignore
//! #[embassy_nrf::interrupt]
//! fn RADIO() { unsafe { libhmk_nrf_esb::on_radio_interrupt() } }
//! #[embassy_nrf::interrupt]
//! fn TIMER0() { unsafe { libhmk_nrf_esb::on_timer_interrupt() } }
//! ```
//!
//! Only one role (PTX or PRX) can exist at a time.

#![no_std]

use core::cell::RefCell;
use core::ptr::addr_of_mut;
use core::sync::atomic::{AtomicBool, Ordering};

use critical_section::Mutex;
use embassy_nrf::interrupt::{self, InterruptExt, Priority};
use embassy_nrf::pac;
use embassy_nrf::pac::radio::vals as rv;
use embassy_nrf::peripherals::{RADIO, TIMER0};
use embassy_nrf::Peri;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;

/// Largest payload supported by this driver (8-bit length field allows more; kept small to
/// bound ISR copy time and RAM).
pub const MAX_PAYLOAD: usize = 72;
/// Number of pipes (addresses) supported by the radio.
pub const NUM_PIPES: usize = 8;
/// Depth of the PRX ACK payload queue.
pub const ACK_QUEUE_DEPTH: usize = 4;
/// Depth of the PRX received-packet queue.
pub const RX_QUEUE_DEPTH: usize = 8;

/// 5-byte ESB address of a pipe: 4-byte base + 1-byte prefix.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Address {
    pub base: [u8; 4],
    pub prefix: u8,
}

/// Radio configuration shared by both roles.
#[derive(Debug, Clone, Copy)]
pub struct Config {
    /// RF channel: 2400 MHz + `channel` MHz (0..=100).
    pub channel: u8,
    /// Transmit power in dBm (nRF52840: -40, -20, -16, -12, -8, -4, 0, 2..=8).
    pub tx_power_dbm: i8,
    /// PTX: retransmissions after the first attempt before giving up.
    pub max_retransmits: u8,
    /// PTX: how long to wait for the start of an ACK after the packet was sent (µs).
    pub ack_timeout_us: u32,
    /// PTX: delay between a failed attempt and the retransmission (µs).
    pub retransmit_delay_us: u32,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            channel: 40,
            tx_power_dbm: 0,
            max_retransmits: 8,
            ack_timeout_us: 200,
            retransmit_delay_us: 250,
        }
    }
}

/// Transmission error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// No ACK after all retransmissions.
    MaxRetransmits,
    /// Payload longer than [`MAX_PAYLOAD`].
    PayloadTooLong,
    /// The PRX ACK payload queue is full.
    QueueFull,
}

/// A received packet (PRX) or ACK payload (PTX).
#[derive(Debug, Clone, Copy)]
pub struct Packet {
    pub pipe: u8,
    len: u8,
    data: [u8; MAX_PAYLOAD],
}

impl Packet {
    const fn empty() -> Self {
        Self {
            pipe: 0,
            len: 0,
            data: [0; MAX_PAYLOAD],
        }
    }

    pub fn payload(&self) -> &[u8] {
        &self.data[..self.len as usize]
    }
}

// ---------------------------------------------------------------------------------------------
// Shared driver state
// ---------------------------------------------------------------------------------------------

/// DMA buffers: `[length][S1 = PID << 1 | ack_requested][payload...]`.
const BUF_LEN: usize = 2 + MAX_PAYLOAD;
static mut TX_BUF: [u8; BUF_LEN] = [0; BUF_LEN];
static mut RX_BUF: [u8; BUF_LEN] = [0; BUF_LEN];
static mut ACK_BUF: [u8; BUF_LEN] = [0; BUF_LEN];

#[derive(Clone, Copy, PartialEq, Eq)]
enum PtxState {
    Idle,
    /// Packet on air; the radio switches to RX by shortcut when done.
    Tx,
    /// Listening for the ACK (ack timer running).
    RxAck,
    /// ACK timeout hit; radio is being disabled.
    AckTimeout,
    /// Waiting for the retransmit delay.
    RetransmitWait,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum PrxState {
    Idle,
    /// Listening; on a packet the radio turns around to TX by shortcut.
    Rx,
    /// ACK on air; the radio returns to RX by shortcut when done.
    TxAck,
    /// Packet needs no ACK (or failed CRC): the TX ramp-up is being aborted.
    Restart,
}

#[derive(Clone, Copy)]
struct AckEntry {
    pipe: u8,
    len: u8,
    data: [u8; MAX_PAYLOAD],
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Role {
    None,
    Ptx,
    Prx,
}

struct State {
    role: Role,
    config: Config,
    // PTX
    ptx: PtxState,
    pid: u8,
    attempts: u8,
    // PRX
    prx: PrxState,
    enabled_pipes: u8,
    /// Last (PID, CRC) per pipe for duplicate detection; `None` until the first packet.
    last_rx: [Option<(u8, u32)>; NUM_PIPES],
    /// The front ACK entry of a pipe was sent and is popped on the next new packet.
    ack_sent: [bool; NUM_PIPES],
    ack_queue: [Option<AckEntry>; ACK_QUEUE_DEPTH],
}

static STATE: Mutex<RefCell<State>> = Mutex::new(RefCell::new(State {
    role: Role::None,
    config: Config {
        channel: 40,
        tx_power_dbm: 0,
        max_retransmits: 8,
        ack_timeout_us: 200,
        retransmit_delay_us: 250,
    },
    ptx: PtxState::Idle,
    pid: 0,
    attempts: 0,
    prx: PrxState::Idle,
    enabled_pipes: 0,
    last_rx: [None; NUM_PIPES],
    ack_sent: [false; NUM_PIPES],
    ack_queue: [None; ACK_QUEUE_DEPTH],
}));

static PTX_DONE: Signal<CriticalSectionRawMutex, Result<Packet, Error>> = Signal::new();
static PRX_RX: Channel<CriticalSectionRawMutex, Packet, RX_QUEUE_DEPTH> = Channel::new();
static ACTIVE: AtomicBool = AtomicBool::new(false);

/// Whether a PTX or PRX instance currently owns the radio (for interrupt dispatch).
pub fn is_active() -> bool {
    ACTIVE.load(Ordering::Acquire)
}

// ---------------------------------------------------------------------------------------------
// Radio configuration
// ---------------------------------------------------------------------------------------------

/// `nrf_esb` `addr_conv` (`__REV(__REV(__RBIT(le32)))`): the whole little-endian word
/// bit-reversed, e.g. `[0x01, 0, 0, 0]` -> `0x8000_0000`.
fn base_reg(base: [u8; 4]) -> u32 {
    u32::from_le_bytes(base).reverse_bits()
}

/// `nrf_esb` `bytewise_bit_swap`: each prefix byte bit-reversed in place (`0xC2` -> `0x43`).
fn prefix_byte(prefix: u8) -> u8 {
    prefix.reverse_bits()
}

fn start_hfxo() {
    let clock = pac::CLOCK;
    if clock.hfclkstat().read().src() != pac::clock::vals::HfclkstatSrc::Xtal {
        clock.events_hfclkstarted().write_value(0);
        clock.tasks_hfclkstart().write_value(1);
        while clock.events_hfclkstarted().read() == 0 {}
    }
}

fn configure_radio(cfg: &Config, addresses: &[(u8, Address)]) {
    let r = pac::RADIO;
    r.power().write(|w| w.set_power(false));
    r.power().write(|w| w.set_power(true));

    r.mode().write(|w| w.set_mode(rv::Mode::Nrf2mbit));
    r.modecnf0().write(|w| {
        w.set_ru(rv::Ru::Fast);
        w.set_dtx(rv::Dtx::Center);
    });
    r.txpower()
        .write(|w| w.set_txpower(rv::Txpower::from_bits(cfg.tx_power_dbm as u8)));
    r.frequency().write(|w| w.set_frequency(cfg.channel.min(100)));

    // Dynamic payload length: 8-bit LENGTH, no S0, 3-bit S1 (PID + no-ACK flag).
    r.pcnf0().write(|w| {
        w.set_lflen(8);
        w.set_s0len(false);
        w.set_s1len(3);
        w.set_s1incl(rv::S1incl::Automatic);
        w.set_plen(rv::Plen::_8bit);
    });
    r.pcnf1().write(|w| {
        w.set_maxlen(MAX_PAYLOAD as u8);
        w.set_statlen(0);
        w.set_balen(4);
        w.set_endian(rv::Endian::Big);
        w.set_whiteen(false);
    });
    r.crccnf().write(|w| {
        w.set_len(rv::Len::Two);
        w.set_skipaddr(rv::Skipaddr::Include);
    });
    r.crcinit().write(|w| w.set_crcinit(0xFFFF));
    r.crcpoly().write(|w| w.set_crcpoly(0x11021));

    // Pipe 0 uses BASE0, pipes 1..7 share BASE1 (as in ESB).
    let mut prefixes = [0u8; NUM_PIPES];
    for &(pipe, addr) in addresses {
        let pipe = pipe as usize;
        if pipe == 0 {
            r.base0().write_value(base_reg(addr.base));
        } else {
            r.base1().write_value(base_reg(addr.base));
        }
        prefixes[pipe] = prefix_byte(addr.prefix);
    }
    r.prefix0().write(|w| {
        w.set_ap0(prefixes[0]);
        w.set_ap1(prefixes[1]);
        w.set_ap2(prefixes[2]);
        w.set_ap3(prefixes[3]);
    });
    r.prefix1().write(|w| {
        w.set_ap4(prefixes[4]);
        w.set_ap5(prefixes[5]);
        w.set_ap6(prefixes[6]);
        w.set_ap7(prefixes[7]);
    });

    r.intenclr().write(|w| w.0 = 0xFFFF_FFFF);
    r.events_disabled().write_value(0);
    r.intenset().write(|w| w.set_disabled(true));

    interrupt::RADIO.set_priority(Priority::P0);
    interrupt::RADIO.unpend();
    unsafe { interrupt::RADIO.enable() };
}

fn configure_timer() {
    let t = pac::TIMER0;
    t.tasks_stop().write_value(1);
    t.mode().write(|w| w.set_mode(pac::timer::vals::Mode::Timer));
    t.bitmode().write(|w| w.set_bitmode(pac::timer::vals::Bitmode::_32bit));
    t.prescaler().write(|w| w.set_prescaler(4)); // 1 MHz
    t.shorts().write(|w| {
        w.set_compare_clear(0, true);
        w.set_compare_stop(0, true);
    });
    t.events_compare(0).write_value(0);
    t.intenset().write(|w| w.set_compare(0, true));
    interrupt::TIMER0.set_priority(Priority::P0);
    interrupt::TIMER0.unpend();
    unsafe { interrupt::TIMER0.enable() };
}

fn timer_start(us: u32) {
    let t = pac::TIMER0;
    t.tasks_stop().write_value(1);
    t.tasks_clear().write_value(1);
    t.events_compare(0).write_value(0);
    t.cc(0).write_value(us.max(1));
    t.tasks_start().write_value(1);
}

fn timer_stop() {
    let t = pac::TIMER0;
    t.tasks_stop().write_value(1);
    t.events_compare(0).write_value(0);
}

fn shutdown() {
    interrupt::RADIO.disable();
    interrupt::TIMER0.disable();
    timer_stop();
    let r = pac::RADIO;
    r.shorts().write(|_| {});
    r.intenclr().write(|w| w.0 = 0xFFFF_FFFF);
    r.tasks_disable().write_value(1);
    ACTIVE.store(false, Ordering::Release);
    critical_section::with(|cs| {
        let mut st = STATE.borrow_ref_mut(cs);
        st.role = Role::None;
        st.ptx = PtxState::Idle;
        st.prx = PrxState::Idle;
    });
}

// ---------------------------------------------------------------------------------------------
// PTX
// ---------------------------------------------------------------------------------------------

/// Primary transmitter (keyboard side). Every packet requests an ACK.
pub struct Ptx {
    _radio: Peri<'static, RADIO>,
    _timer: Peri<'static, TIMER0>,
}

impl Ptx {
    /// Configure the radio as PTX. `addresses` lists `(pipe, address)`; pipe 0 has its own base,
    /// pipes 1..=7 share one base. Starts the HFXO if needed (the radio requires it).
    pub fn new(
        radio: Peri<'static, RADIO>,
        timer: Peri<'static, TIMER0>,
        config: Config,
        addresses: &[(u8, Address)],
    ) -> Self {
        start_hfxo();
        critical_section::with(|cs| {
            let mut st = STATE.borrow_ref_mut(cs);
            st.role = Role::Ptx;
            st.config = config;
            st.ptx = PtxState::Idle;
        });
        configure_radio(&config, addresses);
        configure_timer();
        PTX_DONE.reset();
        ACTIVE.store(true, Ordering::Release);
        Self {
            _radio: radio,
            _timer: timer,
        }
    }

    /// Send `payload` on `pipe` and wait for the ACK. Returns the ACK payload (possibly empty).
    pub async fn send(&mut self, pipe: u8, payload: &[u8]) -> Result<Packet, Error> {
        if payload.len() > MAX_PAYLOAD {
            return Err(Error::PayloadTooLong);
        }
        PTX_DONE.reset();
        critical_section::with(|cs| {
            let mut st = STATE.borrow_ref_mut(cs);
            // New packet: next PID (retransmissions keep the PID).
            st.pid = (st.pid + 1) & 0x03;
            st.attempts = 0;
            let buf = unsafe { &mut *addr_of_mut!(TX_BUF) };
            buf[0] = payload.len() as u8;
            buf[1] = (st.pid << 1) | 1;
            buf[2..2 + payload.len()].copy_from_slice(payload);
            let r = pac::RADIO;
            r.txaddress().write(|w| w.set_txaddress(pipe & 0x07));
            // The ACK comes back on the same address: listen on that pipe only.
            r.rxaddresses().write(|w| w.0 = 1 << (pipe & 0x07));
            ptx_start_tx(&mut st);
        });
        PTX_DONE.wait().await
    }
}

impl Drop for Ptx {
    fn drop(&mut self) {
        shutdown();
    }
}

fn ptx_start_tx(st: &mut State) {
    let r = pac::RADIO;
    r.packetptr().write_value(addr_of_mut!(TX_BUF) as u32);
    r.shorts().write(|w| {
        w.set_ready_start(true);
        w.set_end_disable(true);
        w.set_disabled_rxen(true);
    });
    r.events_disabled().write_value(0);
    r.events_address().write_value(0);
    st.ptx = PtxState::Tx;
    r.tasks_txen().write_value(1);
}

fn ptx_retransmit_or_fail(st: &mut State) {
    st.attempts += 1;
    if st.attempts > st.config.max_retransmits {
        st.ptx = PtxState::Idle;
        PTX_DONE.signal(Err(Error::MaxRetransmits));
    } else {
        st.ptx = PtxState::RetransmitWait;
        timer_start(st.config.retransmit_delay_us);
    }
}

fn ptx_on_disabled(st: &mut State) {
    let r = pac::RADIO;
    match st.ptx {
        PtxState::Tx => {
            // TX done; the shortcut is ramping the radio up for RX: point DMA at the RX buffer
            // before READY->START fires (~40 µs with fast ramp-up).
            r.packetptr().write_value(addr_of_mut!(RX_BUF) as u32);
            r.shorts().write(|w| {
                w.set_ready_start(true);
                w.set_end_disable(true);
            });
            r.events_address().write_value(0);
            st.ptx = PtxState::RxAck;
            timer_start(st.config.ack_timeout_us);
        }
        PtxState::RxAck => {
            timer_stop();
            if r.crcstatus().read().crcstatus() == rv::Crcstatus::CrcOk {
                let buf = unsafe { &*addr_of_mut!(RX_BUF) };
                let mut ack = Packet::empty();
                ack.pipe = r.rxmatch().read().rxmatch();
                ack.len = buf[0].min(MAX_PAYLOAD as u8);
                ack.data[..ack.len as usize].copy_from_slice(&buf[2..2 + ack.len as usize]);
                st.ptx = PtxState::Idle;
                PTX_DONE.signal(Ok(ack));
            } else {
                ptx_retransmit_or_fail(st);
            }
        }
        PtxState::AckTimeout => ptx_retransmit_or_fail(st),
        PtxState::Idle | PtxState::RetransmitWait => {}
    }
}

fn ptx_on_timer(st: &mut State) {
    let r = pac::RADIO;
    match st.ptx {
        PtxState::RxAck => {
            // An address match means an ACK is being received: let END->DISABLE finish it.
            if r.events_address().read() == 0 {
                st.ptx = PtxState::AckTimeout;
                r.shorts().write(|_| {});
                r.tasks_disable().write_value(1);
            }
        }
        PtxState::RetransmitWait => ptx_start_tx(st),
        _ => {}
    }
}

// ---------------------------------------------------------------------------------------------
// PRX
// ---------------------------------------------------------------------------------------------

/// Primary receiver (dongle side). Received packets are acknowledged automatically, carrying the
/// next queued ACK payload for that pipe.
pub struct Prx {
    _radio: Peri<'static, RADIO>,
}

impl Prx {
    /// Configure the radio as PRX listening on the pipes in `addresses`.
    pub fn new(radio: Peri<'static, RADIO>, config: Config, addresses: &[(u8, Address)]) -> Self {
        start_hfxo();
        let enabled = addresses.iter().fold(0u8, |m, &(p, _)| m | (1 << (p & 7)));
        critical_section::with(|cs| {
            let mut st = STATE.borrow_ref_mut(cs);
            st.role = Role::Prx;
            st.config = config;
            st.prx = PrxState::Idle;
            st.enabled_pipes = enabled;
            st.last_rx = [None; NUM_PIPES];
            st.ack_sent = [false; NUM_PIPES];
            st.ack_queue = [None; ACK_QUEUE_DEPTH];
        });
        configure_radio(&config, addresses);
        pac::RADIO.rxaddresses().write(|w| w.0 = enabled as u32);
        ACTIVE.store(true, Ordering::Release);
        critical_section::with(|cs| prx_start_rx(&mut STATE.borrow_ref_mut(cs)));
        Self { _radio: radio }
    }

    /// Wait for the next (non-duplicate) packet.
    pub async fn receive(&mut self) -> Packet {
        PRX_RX.receive().await
    }

    /// Queue a payload to be returned in the ACK of the next packet received on `pipe`.
    pub fn queue_ack_payload(&mut self, pipe: u8, payload: &[u8]) -> Result<(), Error> {
        if payload.len() > MAX_PAYLOAD {
            return Err(Error::PayloadTooLong);
        }
        critical_section::with(|cs| {
            let mut st = STATE.borrow_ref_mut(cs);
            let slot = st
                .ack_queue
                .iter_mut()
                .find(|e| e.is_none())
                .ok_or(Error::QueueFull)?;
            let mut entry = AckEntry {
                pipe,
                len: payload.len() as u8,
                data: [0; MAX_PAYLOAD],
            };
            entry.data[..payload.len()].copy_from_slice(payload);
            *slot = Some(entry);
            Ok(())
        })
    }

    /// Number of ACK payloads waiting to be delivered.
    pub fn ack_queue_len(&self) -> usize {
        critical_section::with(|cs| STATE.borrow_ref(cs).ack_queue.iter().flatten().count())
    }

    /// Drop the pending ACK payloads of one pipe.
    pub fn clear_ack_payloads_for(&mut self, pipe: u8) {
        critical_section::with(|cs| {
            let mut st = STATE.borrow_ref_mut(cs);
            while ack_front(&st, pipe).is_some() {
                ack_pop(&mut st, pipe);
            }
            st.ack_sent[(pipe & 7) as usize] = false;
        });
    }

    /// Whether an ACK payload is queued for `pipe`.
    pub fn has_ack_payload(&self, pipe: u8) -> bool {
        critical_section::with(|cs| ack_front(&STATE.borrow_ref(cs), pipe).is_some())
    }

    /// Drop all pending ACK payloads.
    pub fn clear_ack_payloads(&mut self) {
        critical_section::with(|cs| {
            let mut st = STATE.borrow_ref_mut(cs);
            st.ack_queue = [None; ACK_QUEUE_DEPTH];
            st.ack_sent = [false; NUM_PIPES];
        });
    }
}

impl Drop for Prx {
    fn drop(&mut self) {
        shutdown();
    }
}

fn prx_start_rx(st: &mut State) {
    let r = pac::RADIO;
    r.packetptr().write_value(addr_of_mut!(RX_BUF) as u32);
    r.shorts().write(|w| {
        w.set_ready_start(true);
        w.set_end_disable(true);
        w.set_disabled_txen(true);
    });
    r.events_disabled().write_value(0);
    st.prx = PrxState::Rx;
    r.tasks_rxen().write_value(1);
}

/// Index of the oldest queued ACK entry for `pipe` (queue order = slot order).
fn ack_front(st: &State, pipe: u8) -> Option<usize> {
    st.ack_queue
        .iter()
        .position(|e| e.is_some_and(|e| e.pipe == pipe))
}

fn ack_pop(st: &mut State, pipe: u8) {
    if let Some(i) = ack_front(st, pipe) {
        // Keep FIFO order by shifting later entries down.
        for j in i..ACK_QUEUE_DEPTH - 1 {
            st.ack_queue[j] = st.ack_queue[j + 1];
        }
        st.ack_queue[ACK_QUEUE_DEPTH - 1] = None;
    }
}

fn prx_on_disabled(st: &mut State) {
    let r = pac::RADIO;
    match st.prx {
        PrxState::Rx => {
            // A packet ended; the shortcut is already ramping up TX for the ACK.
            if r.crcstatus().read().crcstatus() != rv::Crcstatus::CrcOk {
                st.prx = PrxState::Restart;
                r.shorts().write(|_| {});
                r.tasks_disable().write_value(1);
                return;
            }
            let rx = unsafe { &*addr_of_mut!(RX_BUF) };
            let pipe = r.rxmatch().read().rxmatch() & 0x07;
            let crc = r.rxcrc().read().rxcrc();
            let len = rx[0].min(MAX_PAYLOAD as u8);
            let pid = (rx[1] >> 1) & 0x03;
            let ack_requested = rx[1] & 1 != 0;
            let duplicate = st.last_rx[pipe as usize] == Some((pid, crc));

            if ack_requested {
                if !duplicate && st.ack_sent[pipe as usize] {
                    // The previous ACK payload reached the PTX (it moved on to a new packet).
                    ack_pop(st, pipe);
                    st.ack_sent[pipe as usize] = false;
                }
                // Build the ACK first: the TX ramp-up is already running.
                let front = ack_front(st, pipe).and_then(|i| st.ack_queue[i]);
                let ack_len = front.map_or(0, |e| e.len);
                let ack_buf = unsafe { &mut *addr_of_mut!(ACK_BUF) };
                ack_buf[0] = ack_len;
                ack_buf[1] = pid << 1;
                if let Some(e) = front {
                    ack_buf[2..2 + e.len as usize].copy_from_slice(&e.data[..e.len as usize]);
                    st.ack_sent[pipe as usize] = true;
                }
                r.packetptr().write_value(addr_of_mut!(ACK_BUF) as u32);
                r.txaddress().write(|w| w.set_txaddress(pipe));
                r.shorts().write(|w| {
                    w.set_ready_start(true);
                    w.set_end_disable(true);
                    w.set_disabled_rxen(true);
                });
                st.prx = PrxState::TxAck;
            } else {
                st.prx = PrxState::Restart;
                r.shorts().write(|_| {});
                r.tasks_disable().write_value(1);
            }

            if !duplicate || !ack_requested {
                st.last_rx[pipe as usize] = Some((pid, crc));
                let mut p = Packet::empty();
                p.pipe = pipe;
                p.len = len;
                p.data[..len as usize].copy_from_slice(&rx[2..2 + len as usize]);
                // A full queue drops the packet; the PTX's state-based protocol recovers.
                let _ = PRX_RX.try_send(p);
            }
        }
        PrxState::TxAck => {
            // ACK sent; the shortcut is ramping RX up again.
            r.packetptr().write_value(addr_of_mut!(RX_BUF) as u32);
            r.shorts().write(|w| {
                w.set_ready_start(true);
                w.set_end_disable(true);
                w.set_disabled_txen(true);
            });
            st.prx = PrxState::Rx;
        }
        PrxState::Restart => prx_start_rx(st),
        PrxState::Idle => {}
    }
}

// ---------------------------------------------------------------------------------------------
// Interrupt entry points
// ---------------------------------------------------------------------------------------------

/// Call from the `RADIO` interrupt handler.
///
/// # Safety
/// Must only be called from the `RADIO` interrupt while a [`Ptx`] or [`Prx`] exists.
pub unsafe fn on_radio_interrupt() {
    let r = pac::RADIO;
    if r.events_disabled().read() == 0 {
        return;
    }
    r.events_disabled().write_value(0);
    critical_section::with(|cs| {
        let mut st = STATE.borrow_ref_mut(cs);
        match st.role {
            Role::Ptx => ptx_on_disabled(&mut st),
            Role::Prx => prx_on_disabled(&mut st),
            Role::None => {}
        }
    });
}

/// Call from the `TIMER0` interrupt handler.
///
/// # Safety
/// Must only be called from the `TIMER0` interrupt while a [`Ptx`] exists.
pub unsafe fn on_timer_interrupt() {
    let t = pac::TIMER0;
    if t.events_compare(0).read() == 0 {
        return;
    }
    t.events_compare(0).write_value(0);
    critical_section::with(|cs| {
        let mut st = STATE.borrow_ref_mut(cs);
        if st.role == Role::Ptx {
            ptx_on_timer(&mut st);
        }
    });
}
