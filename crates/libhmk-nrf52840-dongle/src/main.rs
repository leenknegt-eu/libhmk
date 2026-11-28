//! 2.4 GHz USB receiver firmware for libhmk keyboards on the nRF52840 Dongle (PCA10059).
//!
//! * Radio: Enhanced ShockBurst primary receiver on a dongle-unique address derived from the chip
//!   ID (pipe 0) plus the well-known pairing address (pipe 1).
//! * USB: the keyboard's composite HID device; reports are forwarded as received and raw HID
//!   (configurator) traffic is relayed through ACK payloads.
//! * Pairing: press SW1, then `BT_PAIR` on the keyboard (in 2.4 GHz mode) within 30 s.
//! * LEDs: LD1 green = keyboard link up, LD2 blue blinking = pairing window open.
//! * Safety: held keys are released if the keyboard goes silent for 250 ms.

#![no_std]
#![no_main]

mod usb;

pub mod kb {
    include!(concat!(env!("OUT_DIR"), "/keyboard_gen.rs"));
}

use defmt::{info, unwrap};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_futures::select::{select4, Either4};
use embassy_nrf::config::{Config as NrfConfig, HfclkSource, LfclkSource};
use embassy_nrf::gpio::{Input, Level, Output, OutputDrive, Pull};
use embassy_nrf::usb::vbus_detect::{self, HardwareVbusDetect};
use embassy_nrf::{bind_interrupts, interrupt, pac, peripherals, usb as nrf_usb};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Ticker, Timer};
use libhmk_core::hid::KeyboardReport;
use libhmk_link::{
    dongle_address, reports_held, timing, Downstream, HidFrame, ReleaseGuard, Upstream,
    HEARTBEAT_FLAG_HELD, MAX_PAYLOAD, PAIRING_ADDRESS, PIPE_LINK, PIPE_PAIRING,
};
use libhmk_nrf_esb::{Config as EsbConfig, Prx};
use static_cell::StaticCell;

/// USB product string of the receiver.
pub const DONGLE_PRODUCT_NAME: &str = "libhmk 2.4 GHz Receiver";

bind_interrupts!(struct Irqs {
    USBD => nrf_usb::InterruptHandler<peripherals::USBD>;
    CLOCK_POWER => vbus_detect::InterruptHandler;
});

#[interrupt]
fn RADIO() {
    unsafe { libhmk_nrf_esb::on_radio_interrupt() }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    // No debugger on a dongle: recover by resetting.
    cortex_m::peripheral::SCB::sys_reset()
}

static PAIR_BUTTON: Signal<CriticalSectionRawMutex, ()> = Signal::new();

fn now_ms() -> u32 {
    Instant::now().as_millis() as u32
}

fn device_id() -> [u8; 8] {
    let id0 = pac::FICR.deviceid(0).read().to_le_bytes();
    let id1 = pac::FICR.deviceid(1).read().to_le_bytes();
    [id0[0], id0[1], id0[2], id0[3], id1[0], id1[1], id1[2], id1[3]]
}

fn to_esb(a: libhmk_link::Address) -> libhmk_nrf_esb::Address {
    libhmk_nrf_esb::Address {
        base: a.base,
        prefix: a.prefix,
    }
}

/// Report state with everything released.
fn released_frame() -> HidFrame {
    HidFrame {
        keyboard: Some([0; KeyboardReport::SIZE]),
        system: Some(0),
        consumer: Some(0),
        mouse: Some([0; 5]),
    }
}

fn held(state: &HidFrame) -> bool {
    reports_held(
        &state.keyboard.unwrap_or_default(),
        state.system.unwrap_or(0),
        state.consumer.unwrap_or(0),
        state.mouse.map_or(0, |m| m[0]),
    )
}

#[embassy_executor::task]
async fn button_task(mut sw1: Input<'static>) -> ! {
    loop {
        sw1.wait_for_falling_edge().await;
        Timer::after_millis(30).await; // debounce
        if sw1.is_low() {
            PAIR_BUTTON.signal(());
            sw1.wait_for_high().await;
        }
    }
}

#[embassy_executor::task]
async fn radio_task(mut prx: Prx, mut led_link: Output<'static>, mut led_pair: Output<'static>) -> ! {
    let my_address = dongle_address(device_id());
    let mut guard = ReleaseGuard::new();
    let mut state = released_frame();
    let mut last_rx_ms: Option<u32> = None;
    let mut pairing_until: Option<u32> = None;
    let mut pending_down: Option<[u8; libhmk_core::hid::RAW_HID_EP_SIZE]> = None;
    let mut tick = Ticker::every(Duration::from_millis(10));
    let mut buf = [0u8; MAX_PAYLOAD];
    info!("[dongle] listening on channel {} as {:?}", kb::ESB_CHANNEL, my_address);

    loop {
        // Move host raw HID requests into the ACK payload queue as space allows.
        if pending_down.is_none() {
            pending_down = usb::RAW_TO_KEYBOARD.try_receive().ok();
        }
        if let Some(data) = pending_down {
            let n = Downstream::Raw(data).encode(&mut buf);
            if prx.queue_ack_payload(PIPE_LINK, &buf[..n]).is_ok() {
                pending_down = None;
            }
        }

        match select4(prx.receive(), PAIR_BUTTON.wait(), tick.next(), usb::RAW_TO_KEYBOARD.ready_to_receive()).await {
            Either4::First(packet) => {
                let now = now_ms();
                guard.on_packet(now);
                last_rx_ms = Some(now);
                match (packet.pipe, Upstream::decode(packet.payload())) {
                    (PIPE_LINK, Some(Upstream::Hid(frame))) => {
                        state.merge(&frame);
                        guard.set_held(held(&state));
                        usb::push_hid(&frame);
                    }
                    (PIPE_LINK, Some(Upstream::Raw(data))) => {
                        let _ = usb::RAW_TO_HOST.try_send(data);
                    }
                    (PIPE_LINK, Some(Upstream::Heartbeat { flags, .. })) => {
                        guard.set_held(held(&state) || flags & HEARTBEAT_FLAG_HELD != 0);
                    }
                    (PIPE_PAIRING, Some(Upstream::PairRequest { device_id }))
                        if pairing_until.is_some() =>
                    {
                        info!("[dongle] pairing request from {:02X}", device_id);
                        // The PairAck went out in this packet's ACK; keep the window open
                        // briefly for retransmissions, then close it.
                        pairing_until = Some(now.wrapping_add(500));
                    }
                    _ => {}
                }
            }
            Either4::Second(()) => {
                info!("[dongle] pairing window open");
                prx.clear_ack_payloads_for(PIPE_PAIRING);
                let n = Downstream::PairAck { address: my_address }.encode(&mut buf);
                unwrap!(prx.queue_ack_payload(PIPE_PAIRING, &buf[..n]));
                pairing_until = Some(now_ms().wrapping_add(timing::PAIRING_WINDOW_MS));
            }
            Either4::Third(()) | Either4::Fourth(_) => {}
        }

        let now = now_ms();
        if guard.poll(now) {
            info!("[dongle] keyboard silent with keys held: releasing");
            state = released_frame();
            usb::push_hid(&state);
        }
        if let Some(until) = pairing_until {
            if (until.wrapping_sub(now) as i32) <= 0 {
                info!("[dongle] pairing window closed");
                pairing_until = None;
                prx.clear_ack_payloads_for(PIPE_PAIRING);
            }
        }

        // LEDs are active low.
        let link_up = last_rx_ms.is_some_and(|t| now.wrapping_sub(t) < timing::LINK_UP_WINDOW_MS);
        led_link.set_level(if link_up { Level::Low } else { Level::High });
        let blink_on = pairing_until.is_some() && (now / 250).is_multiple_of(2);
        led_pair.set_level(if blink_on { Level::Low } else { Level::High });
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = NrfConfig::default();
    config.hfclk_source = HfclkSource::ExternalXtal;
    config.lfclk_source = LfclkSource::InternalRC;
    let p = embassy_nrf::init(config);
    info!("libhmk dongle for {}", kb::USB_PRODUCT_NAME);

    // LD1 green (P0.06), LD2 red/green/blue (P0.08 / P1.09 / P0.12); all active low.
    let led_link = Output::new(p.P0_06, Level::High, OutputDrive::Standard);
    let led_pair = Output::new(p.P0_12, Level::High, OutputDrive::Standard);
    let _led_red = Output::new(p.P0_08, Level::High, OutputDrive::Standard);
    let _led_green2 = Output::new(p.P1_09, Level::High, OutputDrive::Standard);
    // SW1 (P1.06), active low.
    let sw1 = Input::new(p.P1_06, Pull::Up);

    let esb_config = EsbConfig {
        channel: kb::ESB_CHANNEL,
        tx_power_dbm: kb::ESB_TX_POWER,
        ..EsbConfig::default()
    };
    let prx = Prx::new(
        p.RADIO,
        esb_config,
        &[
            (PIPE_LINK, to_esb(dongle_address(device_id()))),
            (PIPE_PAIRING, to_esb(PAIRING_ADDRESS)),
        ],
    );

    static SERIAL: StaticCell<[u8; 16]> = StaticCell::new();
    let serial = SERIAL.init([0; 16]);
    const HEX: &[u8; 16] = b"0123456789ABCDEF";
    for (i, b) in device_id().iter().rev().enumerate() {
        serial[i * 2] = HEX[(b >> 4) as usize];
        serial[i * 2 + 1] = HEX[(b & 0x0F) as usize];
    }
    let serial_str = unwrap!(core::str::from_utf8(serial));

    let driver = nrf_usb::Driver::new(p.USBD, Irqs, HardwareVbusDetect::new(Irqs));
    spawner.spawn(unwrap!(usb::usb_task(driver, serial_str)));
    spawner.spawn(unwrap!(button_task(sw1)));
    spawner.spawn(unwrap!(radio_task(prx, led_link, led_pair)));
}
