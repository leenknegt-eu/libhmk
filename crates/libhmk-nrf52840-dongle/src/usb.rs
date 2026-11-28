//! USB side of the dongle: the same composite HID device as the keyboard (boot + NKRO keyboard,
//! system/consumer/mouse, raw HID for the configurator). Reports arrive over the radio as
//! complete report state and are forwarded unchanged; raw HID is relayed in both directions.

use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, Ordering};

use critical_section::Mutex;
use embassy_futures::join::join4;
use embassy_nrf::usb::vbus_detect::HardwareVbusDetect;
use embassy_nrf::usb::Driver;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use embassy_usb::class::hid::{
    Config as HidConfig, HidBootProtocol, HidReaderWriter, HidSubclass, HidWriter, ReportId,
    RequestHandler, State,
};
use embassy_usb::control::OutResponse;
use embassy_usb::{Builder, Config as UsbConfig, Handler};
use libhmk_core::hid::{
    KeyboardReport, MouseReport, DESC_HID_REPORT, DESC_KEYBOARD_REPORT, DESC_RAW_HID_REPORT,
    RAW_HID_EP_SIZE, REPORT_ID_CONSUMER_CONTROL, REPORT_ID_MOUSE, REPORT_ID_SYSTEM_CONTROL,
};
use libhmk_link::HidFrame;
use static_cell::StaticCell;

use crate::kb;

pub static USB_MOUNTED: AtomicBool = AtomicBool::new(false);

/// Raw HID responses from the keyboard, to the host.
pub static RAW_TO_HOST: Channel<CriticalSectionRawMutex, [u8; RAW_HID_EP_SIZE], 4> =
    Channel::new();
/// Raw HID requests from the host, to the keyboard (sent in ACK payloads).
pub static RAW_TO_KEYBOARD: Channel<CriticalSectionRawMutex, [u8; RAW_HID_EP_SIZE], 4> =
    Channel::new();

/// Latest HID report state not yet written to USB (coalesced: newest state per report wins).
static HID_MAILBOX: Mutex<RefCell<HidFrame>> = Mutex::new(RefCell::new(HidFrame {
    keyboard: None,
    system: None,
    consumer: None,
    mouse: None,
}));
static HID_PENDING: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Queue HID report state for the host.
pub fn push_hid(frame: &HidFrame) {
    critical_section::with(|cs| HID_MAILBOX.borrow_ref_mut(cs).merge(frame));
    HID_PENDING.signal(());
}

struct DeviceHandler;

impl Handler for DeviceHandler {
    fn enabled(&mut self, enabled: bool) {
        if !enabled {
            USB_MOUNTED.store(false, Ordering::Release);
        }
    }

    fn reset(&mut self) {
        USB_MOUNTED.store(false, Ordering::Release);
    }

    fn configured(&mut self, configured: bool) {
        USB_MOUNTED.store(configured, Ordering::Release);
    }
}

struct RawHidRequestHandler;

impl RequestHandler for RawHidRequestHandler {
    fn set_report(&mut self, _id: ReportId, data: &[u8]) -> OutResponse {
        if data.len() == RAW_HID_EP_SIZE {
            let mut buf = [0u8; RAW_HID_EP_SIZE];
            buf.copy_from_slice(data);
            let _ = RAW_TO_KEYBOARD.try_send(buf);
            OutResponse::Accepted
        } else {
            OutResponse::Rejected
        }
    }
}

#[embassy_executor::task]
pub async fn usb_task(driver: Driver<'static, HardwareVbusDetect>, serial: &'static str) -> ! {
    // Same VID/PID as the keyboard so the configurator recognises it; the serial number is
    // the dongle's own chip ID.
    let mut usb_cfg = UsbConfig::new(kb::USB_VENDOR_ID, kb::USB_PRODUCT_ID);
    usb_cfg.manufacturer = Some(kb::USB_MANUFACTURER_NAME);
    usb_cfg.product = Some(crate::DONGLE_PRODUCT_NAME);
    usb_cfg.serial_number = Some(serial);
    usb_cfg.max_power = 100;
    usb_cfg.max_packet_size_0 = 64;
    usb_cfg.composite_with_iads = false;
    usb_cfg.device_class = 0;
    usb_cfg.device_sub_class = 0;
    usb_cfg.device_protocol = 0;

    static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static MSOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
    static DEV_HANDLER: StaticCell<DeviceHandler> = StaticCell::new();
    static KB_STATE: StaticCell<State> = StaticCell::new();
    static HID_STATE: StaticCell<State> = StaticCell::new();
    static RAW_STATE: StaticCell<State> = StaticCell::new();
    static RAW_REQ_HANDLER: StaticCell<RawHidRequestHandler> = StaticCell::new();

    let mut builder = Builder::new(
        driver,
        usb_cfg,
        CONFIG_DESC.init([0; 256]),
        BOS_DESC.init([0; 256]),
        MSOS_DESC.init([0; 256]),
        CONTROL_BUF.init([0; 64]),
    );
    builder.handler(DEV_HANDLER.init(DeviceHandler));

    let mut kb_writer = HidWriter::<_, { KeyboardReport::SIZE }>::new(
        &mut builder,
        KB_STATE.init(State::new()),
        HidConfig {
            report_descriptor: DESC_KEYBOARD_REPORT,
            request_handler: None,
            poll_ms: 1,
            max_packet_size: 64,
            hid_subclass: HidSubclass::Boot,
            hid_boot_protocol: HidBootProtocol::Keyboard,
        },
    );
    let mut hid_writer = HidWriter::<_, { MouseReport::SIZE + 1 }>::new(
        &mut builder,
        HID_STATE.init(State::new()),
        HidConfig {
            report_descriptor: DESC_HID_REPORT,
            request_handler: None,
            poll_ms: 1,
            max_packet_size: 64,
            hid_subclass: HidSubclass::No,
            hid_boot_protocol: HidBootProtocol::None,
        },
    );
    let raw_hid = HidReaderWriter::<_, RAW_HID_EP_SIZE, RAW_HID_EP_SIZE>::new(
        &mut builder,
        RAW_STATE.init(State::new()),
        HidConfig {
            report_descriptor: DESC_RAW_HID_REPORT,
            request_handler: Some(RAW_REQ_HANDLER.init(RawHidRequestHandler)),
            poll_ms: 1,
            max_packet_size: 64,
            hid_subclass: HidSubclass::No,
            hid_boot_protocol: HidBootProtocol::None,
        },
    );

    let mut usb = builder.build();
    let (raw_reader, mut raw_writer) = raw_hid.split();

    let hid_fut = async {
        loop {
            HID_PENDING.wait().await;
            let frame = critical_section::with(|cs| core::mem::take(&mut *HID_MAILBOX.borrow_ref_mut(cs)));
            if let Some(kb_rep) = frame.keyboard {
                let _ = kb_writer.write(&kb_rep).await;
            }
            if let Some(sys) = frame.system {
                let b = sys.to_le_bytes();
                let _ = hid_writer.write(&[REPORT_ID_SYSTEM_CONTROL, b[0], b[1]]).await;
            }
            if let Some(cons) = frame.consumer {
                let b = cons.to_le_bytes();
                let _ = hid_writer.write(&[REPORT_ID_CONSUMER_CONTROL, b[0], b[1]]).await;
            }
            if let Some(m) = frame.mouse {
                let _ = hid_writer
                    .write(&[REPORT_ID_MOUSE, m[0], m[1], m[2], m[3], m[4]])
                    .await;
            }
        }
    };
    let raw_rx_fut = async {
        let mut handler = RawHidRequestHandler;
        raw_reader.run(false, &mut handler).await;
    };
    let raw_tx_fut = async {
        loop {
            let resp = RAW_TO_HOST.receive().await;
            let _ = raw_writer.write(&resp).await;
        }
    };

    join4(usb.run(), hid_fut, raw_rx_fut, raw_tx_fut).await;
    unreachable!()
}
