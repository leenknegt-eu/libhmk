//! 2.4 GHz link to the USB dongle over Enhanced ShockBurst (keyboard = PTX).
//!
//! The main loop hands HID state to [`push_hid`] (coalesced: only the latest state per report
//! matters) and raw HID responses to [`RF_RAW_TX`]. This task sends them to the dongle, keeps
//! the link alive with heartbeats (which also poll for downstream raw HID requests carried in
//! ACK payloads) and runs the pairing exchange.

use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use critical_section::Mutex;
use defmt::{info, warn};
use embassy_futures::select::select3;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer};
use libhmk_core::hid::{KeyboardReport, PendingHidReports, RAW_HID_EP_SIZE};
use libhmk_core::traits::CommandTransport;
use libhmk_link::{
    reports_held, timing, Address, Downstream, HidFrame, PtxScheduler, Upstream,
    HEARTBEAT_FLAG_HELD, MAX_PAYLOAD, PAIRING_ADDRESS, PIPE_LINK, PIPE_PAIRING, UNPAIRED_ADDRESS,
};
use libhmk_nrf_esb::{Config, Ptx};

use crate::analog::SharedAnalogSamples;
use crate::usb::{UsbRawCommand, CMD_RX};
use crate::{kb, wireless};

/// Raw HID responses to send to the dongle.
pub static RF_RAW_TX: Channel<CriticalSectionRawMutex, [u8; RAW_HID_EP_SIZE], 4> = Channel::new();

static HID_MAILBOX: Mutex<RefCell<HidFrame>> = Mutex::new(RefCell::new(HidFrame {
    keyboard: None,
    system: None,
    consumer: None,
    mouse: None,
}));
static WAKE: Signal<CriticalSectionRawMutex, ()> = Signal::new();
static PAIRING_UNTIL_MS: AtomicU32 = AtomicU32::new(0);
static PAIRING: AtomicBool = AtomicBool::new(false);
static LAST_ACK_MS: AtomicU32 = AtomicU32::new(0);
static LINK_EVER_UP: AtomicBool = AtomicBool::new(false);

/// Back-off after a failed send: short while the link was up (transient interference), longer
/// while the dongle is unreachable (saves battery).
const LINK_UP_RETRY_MS: u32 = 5;
const LINK_DOWN_RETRY_MS: u32 = 50;
/// Give up re-sending one raw HID response after this many failed sends.
const RAW_MAX_ATTEMPTS: u8 = 20;

fn now_ms() -> u32 {
    Instant::now().as_millis() as u32
}

/// Hand new HID report state to the link (latest state per report wins).
pub fn push_hid(p: &PendingHidReports) {
    let frame = HidFrame::from_pending(p);
    critical_section::with(|cs| HID_MAILBOX.borrow_ref_mut(cs).merge(&frame));
    WAKE.signal(());
}

/// Open the pairing window (`BT_PAIR` while in 2.4 GHz mode).
pub fn start_pairing() {
    PAIRING_UNTIL_MS.store(now_ms().wrapping_add(timing::PAIRING_WINDOW_MS), Ordering::Relaxed);
    PAIRING.store(true, Ordering::Release);
    info!("[rf] pairing for {} s", timing::PAIRING_WINDOW_MS / 1000);
    WAKE.signal(());
}

/// The dongle acknowledged a packet recently.
pub fn link_up() -> bool {
    LINK_EVER_UP.load(Ordering::Acquire)
        && now_ms().wrapping_sub(LAST_ACK_MS.load(Ordering::Relaxed)) < timing::LINK_UP_WINDOW_MS
}

fn pairing_active(now: u32) -> bool {
    if !PAIRING.load(Ordering::Acquire) {
        return false;
    }
    if (PAIRING_UNTIL_MS.load(Ordering::Relaxed).wrapping_sub(now) as i32) <= 0 {
        PAIRING.store(false, Ordering::Release);
        info!("[rf] pairing window closed");
        return false;
    }
    true
}

fn to_esb(a: Address) -> libhmk_nrf_esb::Address {
    libhmk_nrf_esb::Address {
        base: a.base,
        prefix: a.prefix,
    }
}

/// Create the PTX radio for the stored pairing (or the unpaired placeholder address).
pub fn make_ptx(
    radio: embassy_nrf::Peri<'static, embassy_nrf::peripherals::RADIO>,
    timer: embassy_nrf::Peri<'static, embassy_nrf::peripherals::TIMER0>,
    paired: Option<Address>,
) -> Ptx {
    let config = Config {
        channel: kb::ESB_CHANNEL,
        tx_power_dbm: kb::ESB_TX_POWER,
        ..Config::default()
    };
    let link = paired.unwrap_or(UNPAIRED_ADDRESS);
    Ptx::new(
        radio,
        timer,
        config,
        &[
            (PIPE_LINK, to_esb(link)),
            (PIPE_PAIRING, to_esb(PAIRING_ADDRESS)),
        ],
    )
}

fn device_id() -> [u8; 8] {
    let id0 = embassy_nrf::pac::FICR.deviceid(0).read().to_le_bytes();
    let id1 = embassy_nrf::pac::FICR.deviceid(1).read().to_le_bytes();
    [id0[0], id0[1], id0[2], id0[3], id1[0], id1[1], id1[2], id1[3]]
}

pub async fn run_ptx(mut ptx: Ptx, paired: bool, samples: &'static SharedAnalogSamples) -> ! {
    let mut sched = PtxScheduler::new();
    // Full current report state (to know whether anything is held).
    let mut state = HidFrame {
        keyboard: Some([0; KeyboardReport::SIZE]),
        system: Some(0),
        consumer: Some(0),
        mouse: Some([0; 5]),
    };
    let mut pending_raw: Option<([u8; RAW_HID_EP_SIZE], u8)> = None;
    let mut retry_at: Option<u32> = None;
    let mut last_pair_req = 0u32;
    let mut buf = [0u8; MAX_PAYLOAD];
    info!("[rf] 2.4 GHz link on channel {} ({})", kb::ESB_CHANNEL, if paired { "paired" } else { "unpaired" });

    loop {
        // Collect new HID state.
        let fresh = critical_section::with(|cs| core::mem::take(&mut *HID_MAILBOX.borrow_ref_mut(cs)));
        if !fresh.is_empty() {
            state.merge(&fresh);
            let held = reports_held(
                &state.keyboard.unwrap_or_default(),
                state.system.unwrap_or(0),
                state.consumer.unwrap_or(0),
                state.mouse.map_or(0, |m| m[0]),
            );
            sched.push_hid(&fresh, held);
        }
        if pending_raw.is_none() {
            if let Ok(raw) = RF_RAW_TX.try_receive() {
                pending_raw = Some((raw, 0));
            }
        }

        let now = now_ms();
        let pairing = pairing_active(now);
        let backoff = retry_at.is_some_and(|t| (t.wrapping_sub(now) as i32) > 0);

        // Pick the next message: pairing > raw HID response > HID state > heartbeat.
        let next: Option<(u8, Upstream)> = if pairing
            && now.wrapping_sub(last_pair_req) >= timing::PAIR_REQUEST_INTERVAL_MS
        {
            last_pair_req = now;
            Some((PIPE_PAIRING, Upstream::PairRequest { device_id: device_id() }))
        } else if !paired || backoff {
            None
        } else if let Some((raw, _)) = pending_raw {
            Some((PIPE_LINK, Upstream::Raw(raw)))
        } else if let Some(frame) = sched.take_hid() {
            Some((PIPE_LINK, Upstream::Hid(frame)))
        } else if sched.heartbeat_due_in_ms(now, wireless::low_power()) == 0 {
            let flags = if sched.held() { HEARTBEAT_FLAG_HELD } else { 0 };
            Some((
                PIPE_LINK,
                Upstream::Heartbeat {
                    battery_percent: samples.battery_percent(),
                    flags,
                },
            ))
        } else {
            None
        };

        let Some((pipe, msg)) = next else {
            // Nothing due: sleep until new data, the next heartbeat, a retry or a pairing tick.
            let mut wait_ms = if paired {
                sched.heartbeat_due_in_ms(now, wireless::low_power()).max(1)
            } else {
                u32::MAX
            };
            if let Some(t) = retry_at {
                wait_ms = wait_ms.min(t.wrapping_sub(now).max(1));
            }
            if pairing {
                wait_ms = wait_ms.min(timing::PAIR_REQUEST_INTERVAL_MS);
            }
            let timeout = Timer::after(Duration::from_millis(wait_ms.min(60_000) as u64));
            select3(WAKE.wait(), RF_RAW_TX.ready_to_receive(), timeout).await;
            continue;
        };

        let n = msg.encode(&mut buf);
        sched.on_sent(now);
        let result = ptx.send(pipe, &buf[..n]).await;
        let done = now_ms();
        let acked = result.is_ok();

        if pipe == PIPE_LINK {
            sched.on_result(done, acked);
            if acked {
                LAST_ACK_MS.store(done, Ordering::Relaxed);
                LINK_EVER_UP.store(true, Ordering::Release);
                retry_at = None;
                if matches!(msg, Upstream::Raw(_)) {
                    pending_raw = None;
                }
            } else {
                let backoff = if link_up() { LINK_UP_RETRY_MS } else { LINK_DOWN_RETRY_MS };
                retry_at = Some(done.wrapping_add(backoff));
                if let Some((_, attempts)) = &mut pending_raw {
                    *attempts += 1;
                    if *attempts >= RAW_MAX_ATTEMPTS {
                        warn!("[rf] dropping raw HID response after {} attempts", RAW_MAX_ATTEMPTS);
                        pending_raw = None;
                    }
                }
            }
        }

        // ACK payloads carry downstream data.
        if let Ok(ack) = result {
            match Downstream::decode(ack.payload()) {
                Some(Downstream::Raw(data)) if pipe == PIPE_LINK => {
                    sched.on_downstream_raw(done);
                    let _ = CMD_RX.try_send(UsbRawCommand {
                        data,
                        transport: CommandTransport::Rf,
                    });
                }
                Some(Downstream::PairAck { address }) if pipe == PIPE_PAIRING => {
                    info!("[rf] paired with dongle {:?}", address);
                    PAIRING.store(false, Ordering::Release);
                    let mut s = wireless::settings();
                    s.paired = Some(address);
                    // Restart with the dongle's address on the link pipe.
                    wireless::request_settings_write(s, true);
                }
                _ => {}
            }
        }
    }
}
