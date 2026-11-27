//! Composite `embassy-usb` HID transport for nRF52840 (Keyboard, Generic HID, Raw HID).

use core::sync::atomic::{AtomicBool, Ordering};

use embassy_futures::join::join4;
use embassy_nrf::usb::vbus_detect::SoftwareVbusDetect;
use embassy_nrf::usb::Driver;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_usb::class::hid::{
    Config as HidConfig, HidBootProtocol, HidReaderWriter, HidSubclass, HidWriter, ReportId,
    RequestHandler, State,
};
use embassy_usb::control::OutResponse;
use embassy_usb::{Builder, Config as UsbConfig, Handler};
use libhmk_core::hid::{
    KeyboardReport, MouseReport, PendingHidReports, DESC_HID_REPORT, DESC_KEYBOARD_REPORT,
    DESC_RAW_HID_REPORT, RAW_HID_EP_SIZE, REPORT_ID_CONSUMER_CONTROL, REPORT_ID_MOUSE,
    REPORT_ID_SYSTEM_CONTROL,
};
use libhmk_core::traits::CommandTransport;
use static_cell::StaticCell;

use crate::kb;

pub static USB_MOUNTED: AtomicBool = AtomicBool::new(false);
pub static USB_SUSPENDED: AtomicBool = AtomicBool::new(false);

pub static USB_HID_TX: Channel<CriticalSectionRawMutex, PendingHidReports, 8> = Channel::new();
pub static USB_RAW_TX: Channel<CriticalSectionRawMutex, [u8; RAW_HID_EP_SIZE], 4> = Channel::new();

pub struct UsbRawCommand {
    pub data: [u8; RAW_HID_EP_SIZE],
    pub transport: CommandTransport,
}

pub static CMD_RX: Channel<CriticalSectionRawMutex, UsbRawCommand, 4> = Channel::new();

struct DeviceHandler;

impl Handler for DeviceHandler {
    fn enabled(&mut self, enabled: bool) {
        if !enabled {
            USB_MOUNTED.store(false, Ordering::Release);
            USB_SUSPENDED.store(false, Ordering::Release);
        }
    }

    fn reset(&mut self) {
        USB_MOUNTED.store(false, Ordering::Release);
    }

    fn configured(&mut self, configured: bool) {
        USB_MOUNTED.store(configured, Ordering::Release);
    }

    fn suspended(&mut self, suspended: bool) {
        USB_SUSPENDED.store(suspended, Ordering::Release);
    }
}

struct RawHidRequestHandler;

impl RequestHandler for RawHidRequestHandler {
    fn set_report(&mut self, _id: ReportId, data: &[u8]) -> OutResponse {
        if data.len() == RAW_HID_EP_SIZE {
            let mut buf = [0u8; RAW_HID_EP_SIZE];
            buf.copy_from_slice(data);
            let _ = CMD_RX.try_send(UsbRawCommand {
                data: buf,
                transport: CommandTransport::Usb,
            });
            OutResponse::Accepted
        } else {
            OutResponse::Rejected
        }
    }
}

pub fn is_usb_mounted() -> bool {
    USB_MOUNTED.load(Ordering::Acquire) && !USB_SUSPENDED.load(Ordering::Acquire)
}

#[embassy_executor::task]
pub async fn usb_task(
    driver: Driver<'static, &'static SoftwareVbusDetect>,
    serial: &'static str,
) -> ! {
    let mut usb_cfg = UsbConfig::new(kb::USB_VENDOR_ID, kb::USB_PRODUCT_ID);
    usb_cfg.manufacturer = Some(kb::USB_MANUFACTURER_NAME);
    usb_cfg.product = Some(kb::USB_PRODUCT_NAME);
    usb_cfg.serial_number = Some(serial);
    usb_cfg.max_power = 500;
    usb_cfg.max_packet_size_0 = 64;
    usb_cfg.supports_remote_wakeup = true;
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

    // Interface 0: Boot + NKRO Keyboard
    let kb_writer = HidWriter::<_, { KeyboardReport::SIZE }>::new(
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

    // Interface 1: System / Consumer / Mouse HID
    let hid_writer = HidWriter::<_, { MouseReport::SIZE + 1 }>::new(
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

    // Interface 2: Raw HID (64-byte IN + OUT for hmkconf)
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

    let usb_fut = usb.run();

    let hid_tx_fut = async {
        let mut kb_writer = kb_writer;
        let mut hid_writer = hid_writer;
        loop {
            let pending = USB_HID_TX.receive().await;
            if let Some(kb_rep) = pending.keyboard {
                let _ = kb_writer.write(&kb_rep.to_bytes()).await;
            }
            if let Some(sys) = pending.system {
                let b = sys.to_le_bytes();
                let _ = hid_writer.write(&[REPORT_ID_SYSTEM_CONTROL, b[0], b[1]]).await;
            }
            if let Some(cons) = pending.consumer {
                let b = cons.to_le_bytes();
                let _ = hid_writer.write(&[REPORT_ID_CONSUMER_CONTROL, b[0], b[1]]).await;
            }
            if let Some(mouse) = pending.mouse {
                let m = mouse.to_bytes();
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
            let resp = USB_RAW_TX.receive().await;
            let _ = raw_writer.write(&resp).await;
        }
    };

    join4(usb_fut, hid_tx_fut, raw_rx_fut, raw_tx_fut).await;
    unreachable!()
}
