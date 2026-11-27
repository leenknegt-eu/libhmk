//! Composite `embassy-usb` HID transport for AT32F405xx (Keyboard, Generic HID, Raw HID).
//!
//! The AT32F405 OTGFS1 / OTGHS controllers are Synopsys DWC2 cores (the C firmware drives
//! them with TinyUSB's `dwc2` port), so the HAL-independent `embassy-usb-synopsys-otg`
//! driver is used with AT32-specific PHY setup.

use embassy_futures::join::join4;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_usb::class::hid::{
    Config as HidConfig, HidBootProtocol, HidReaderWriter, HidSubclass, HidWriter, ReportId,
    RequestHandler, State,
};
use embassy_usb::control::OutResponse;
use embassy_usb::driver::{
    EndpointAddress, EndpointAllocError, EndpointType, Event, Unsupported,
};
use embassy_usb::{Builder, Config as UsbConfig};
use embassy_usb_synopsys_otg::otg_v1::{vals::Dspd, Otg};
use embassy_usb_synopsys_otg::{
    on_interrupt, Bus as OtgBus, Config as OtgConfig, ControlPipe, Driver as OtgDriver, Endpoint,
    In, OtgInstance, Out, PhyType, StateStorage,
};
use libhmk_core::hid::{
    KeyboardReport, MouseReport, PendingHidReports, DESC_HID_REPORT, DESC_KEYBOARD_REPORT,
    DESC_RAW_HID_REPORT, RAW_HID_EP_SIZE, REPORT_ID_CONSUMER_CONTROL, REPORT_ID_MOUSE,
    REPORT_ID_SYSTEM_CONTROL,
};
use static_cell::StaticCell;

use at32f4xx_pac::at32f405::{interrupt, Interrupt};

use crate::kb;

const OTGFS1_BASE: usize = 0x5000_0000;
const OTGHS_BASE: usize = 0x4004_0000;
const OTG_BASE: usize = if kb::USB_HIGH_SPEED { OTGHS_BASE } else { OTGFS1_BASE };
/// Device endpoints incl. EP0 (GHWCFG2 `num_dev_ep` = 7).
const ENDPOINT_COUNT: usize = 8;
/// Data FIFO depth in words (TinyUSB `dwc2_at32.h`: 1280 B FS; GHWCFG3: 1008 words HS).
const FIFO_DEPTH_WORDS: u16 = if kb::USB_HIGH_SPEED { 1008 } else { 320 };
const RX_FIFO_EXTRA_SIZE_WORDS: u16 = 30;
/// GCCFG (offset 0x38): bit 16 transceiver power (PWRDOWN, active high), bit 21 VBUS ignored.
const GCCFG_OFFSET: usize = 0x38;
const GCCFG_PWRDOWN: u32 = 1 << 16;
const GCCFG_VBUSIG: u32 = 1 << 21;
/// GRSTCTL.AHBIDLE
const GRSTCTL_OFFSET: usize = 0x10;
const GRSTCTL_AHBIDLE: u32 = 1 << 31;

static OTG_STATE: StateStorage<ENDPOINT_COUNT> = StateStorage::new();

fn otg_regs() -> Otg {
    unsafe { Otg::from_ptr(OTG_BASE as *mut ()) }
}

fn calculate_trdt(speed: Dspd) -> u8 {
    match speed {
        Dspd::HIGH_SPEED => 9,
        // AHB = 216 MHz (>= 32 MHz)
        _ => 6,
    }
}

/// Interrupt priority of the USB controller.
pub fn enable_interrupt() {
    let irq = if kb::USB_HIGH_SPEED { Interrupt::OTGHS } else { Interrupt::OTGFS };
    unsafe {
        let mut nvic = cortex_m::Peripherals::steal().NVIC;
        nvic.set_priority(irq, 0x20);
        cortex_m::peripheral::NVIC::unmask(irq);
    }
}

#[interrupt]
fn OTGHS() {
    if kb::USB_HIGH_SPEED {
        unsafe { on_interrupt(otg_regs(), &OTG_STATE.as_state()) };
    }
}

#[interrupt]
fn OTGFS() {
    if !kb::USB_HIGH_SPEED {
        unsafe { on_interrupt(otg_regs(), &OTG_STATE.as_state()) };
    }
}

/// `embassy-usb` driver for the AT32F405 OTG controller selected by `keyboard.json`.
pub struct At32UsbDriver {
    inner: OtgDriver<'static>,
}

impl At32UsbDriver {
    pub fn new(ep_out_buffer: &'static mut [u8]) -> Self {
        let instance = OtgInstance {
            regs: otg_regs(),
            state: OTG_STATE.as_state(),
            fifo_depth_words: FIFO_DEPTH_WORDS,
            phy_type: if kb::USB_HIGH_SPEED {
                PhyType::InternalHighSpeed
            } else {
                PhyType::InternalFullSpeed
            },
            extra_rx_fifo_words: RX_FIFO_EXTRA_SIZE_WORDS,
            calculate_trdt_fn: calculate_trdt,
        };
        let mut config = OtgConfig::default();
        config.vbus_detection = false;
        Self {
            inner: OtgDriver::new(ep_out_buffer, instance, config),
        }
    }
}

impl embassy_usb::driver::Driver<'static> for At32UsbDriver {
    type EndpointOut = Endpoint<'static, Out>;
    type EndpointIn = Endpoint<'static, In>;
    type ControlPipe = ControlPipe<'static>;
    type Bus = At32UsbBus;

    fn alloc_endpoint_in(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointIn, EndpointAllocError> {
        self.inner.alloc_endpoint_in(ep_type, ep_addr, max_packet_size, interval_ms)
    }

    fn alloc_endpoint_out(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointOut, EndpointAllocError> {
        self.inner.alloc_endpoint_out(ep_type, ep_addr, max_packet_size, interval_ms)
    }

    fn start(self, control_max_packet_size: u16) -> (Self::Bus, Self::ControlPipe) {
        let (bus, cp) = self.inner.start(control_max_packet_size);
        (At32UsbBus { inner: bus, inited: false }, cp)
    }
}

pub struct At32UsbBus {
    inner: OtgBus<'static>,
    inited: bool,
}

impl At32UsbBus {
    fn init(&mut self) {
        let base = OTG_BASE;
        unsafe {
            let grstctl = (base + GRSTCTL_OFFSET) as *const u32;
            while grstctl.read_volatile() & GRSTCTL_AHBIDLE == 0 {}
        }
        self.inner.configure_as_device();
        // AT32 specific (TinyUSB `dwc2_phy_update` + C `board_usb_init`): power the
        // transceiver and ignore VBUS sensing (bus-powered keyboard).
        unsafe {
            let gccfg = (base + GCCFG_OFFSET) as *mut u32;
            gccfg.write_volatile(gccfg.read_volatile() | GCCFG_PWRDOWN | GCCFG_VBUSIG);
        }
    }
}

impl embassy_usb::driver::Bus for At32UsbBus {
    async fn poll(&mut self) -> Event {
        if !self.inited {
            self.init();
            self.inited = true;
        }
        self.inner.poll().await
    }

    fn endpoint_set_stalled(&mut self, ep_addr: EndpointAddress, stalled: bool) {
        self.inner.endpoint_set_stalled(ep_addr, stalled)
    }

    fn endpoint_is_stalled(&mut self, ep_addr: EndpointAddress) -> bool {
        self.inner.endpoint_is_stalled(ep_addr)
    }

    fn endpoint_set_enabled(&mut self, ep_addr: EndpointAddress, enabled: bool) {
        self.inner.endpoint_set_enabled(ep_addr, enabled)
    }

    async fn enable(&mut self) {
        self.inner.enable().await
    }

    async fn disable(&mut self) {
        self.inner.disable().await
    }

    async fn remote_wakeup(&mut self) -> Result<(), Unsupported> {
        self.inner.remote_wakeup().await
    }
}

pub static USB_HID_TX: Channel<CriticalSectionRawMutex, PendingHidReports, 8> = Channel::new();
pub static USB_RAW_TX: Channel<CriticalSectionRawMutex, [u8; RAW_HID_EP_SIZE], 4> = Channel::new();
pub static USB_CMD_RX: Channel<CriticalSectionRawMutex, [u8; RAW_HID_EP_SIZE], 4> = Channel::new();

struct RawHidRequestHandler;

impl RequestHandler for RawHidRequestHandler {
    fn set_report(&mut self, _id: ReportId, data: &[u8]) -> OutResponse {
        if data.len() == RAW_HID_EP_SIZE {
            let mut buf = [0u8; RAW_HID_EP_SIZE];
            buf.copy_from_slice(data);
            let _ = USB_CMD_RX.try_send(buf);
            OutResponse::Accepted
        } else {
            OutResponse::Rejected
        }
    }
}

#[embassy_executor::task]
pub async fn usb_task(driver: At32UsbDriver, serial: &'static str, poll_ms: u8) -> ! {
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
    static KB_STATE: StaticCell<State> = StaticCell::new();
    static HID_STATE: StaticCell<State> = StaticCell::new();
    static RAW_STATE: StaticCell<State> = StaticCell::new();
    static RAW_REQ: StaticCell<RawHidRequestHandler> = StaticCell::new();

    let mut builder = Builder::new(
        driver,
        usb_cfg,
        CONFIG_DESC.init([0; 256]),
        BOS_DESC.init([0; 256]),
        MSOS_DESC.init([0; 256]),
        CONTROL_BUF.init([0; 64]),
    );

    let mut kb_writer = HidWriter::<_, { KeyboardReport::SIZE }>::new(
        &mut builder,
        KB_STATE.init(State::new()),
        HidConfig {
            report_descriptor: DESC_KEYBOARD_REPORT,
            request_handler: None,
            poll_ms,
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
            poll_ms,
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
            request_handler: Some(RAW_REQ.init(RawHidRequestHandler)),
            poll_ms,
            max_packet_size: 64,
            hid_subclass: HidSubclass::No,
            hid_boot_protocol: HidBootProtocol::None,
        },
    );

    let mut usb = builder.build();
    let (raw_reader, mut raw_writer) = raw_hid.split();

    let usb_fut = usb.run();
    let hid_tx_fut = async {
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
