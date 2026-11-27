//! TrouBLE BLE GATT server (HOGP + Raw HID + DIS + BAS) + dual-mode transport & power manager.

use core::sync::atomic::{AtomicBool, AtomicU32, AtomicU8, Ordering};

use defmt::{info, warn};
use embassy_futures::join::join;
use embassy_futures::select::{select, select3, Either, Either3};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer};
use libhmk_core::hid::{
    KeyboardReport, MouseReport, PendingHidReports, BLE_HID_REPORT_MAP, BLE_REPORT_ID_CONSUMER,
    BLE_REPORT_ID_KEYBOARD, BLE_REPORT_ID_MOUSE, BLE_REPORT_ID_RAW_HID, BLE_REPORT_ID_SYSTEM,
    RAW_HID_EP_SIZE,
};
use libhmk_core::traits::{BleControl, BlePowerState, BleTransportMode, CommandTransport};
use trouble_host::prelude::*;

use crate::analog::SharedAnalogSamples;
use crate::board::NrfBoard;
use crate::kb;
use crate::usb::{is_usb_mounted, UsbRawCommand, CMD_RX};

const CONNECTIONS_MAX: usize = 1;
const L2CAP_CHANNELS_MAX: usize = 3; // Signal + ATT + SMP

pub static BLE_HID_TX: Channel<CriticalSectionRawMutex, PendingHidReports, 8> = Channel::new();
pub static BLE_RAW_TX: Channel<CriticalSectionRawMutex, [u8; RAW_HID_EP_SIZE], 4> = Channel::new();
static BLE_DISCONNECT_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

static BLE_CONNECTED: AtomicBool = AtomicBool::new(false);
static TRANSPORT_MODE: AtomicU8 = AtomicU8::new(0); // 0 = Auto, 1 = Usb, 2 = Ble
static POWER_STATE: AtomicU8 = AtomicU8::new(0); // 0 = Active, 1 = Idle, 2 = Sleep
static LAST_ACTIVITY_MS: AtomicU32 = AtomicU32::new(0);

//--------------------------------------------------------------------+
// GATT Server Definition (DIS 0x180A, BAS 0x180F, HIDS 0x1812)
//--------------------------------------------------------------------+

#[gatt_server]
struct KeyboardGattServer {
    dis: DeviceInfoService,
    bas: BatteryService,
    hids: HidService,
}

#[gatt_service(uuid = service::DEVICE_INFORMATION)]
struct DeviceInfoService {
    #[characteristic(uuid = characteristic::MANUFACTURER_NAME_STRING, read, value = kb::USB_MANUFACTURER_NAME)]
    manufacturer: &'static str,
    #[characteristic(uuid = characteristic::MODEL_NUMBER_STRING, read, value = kb::USB_PRODUCT_NAME)]
    model: &'static str,
    #[characteristic(uuid = characteristic::PNP_ID, read, value = [
        0x02,
        (kb::USB_VENDOR_ID & 0xFF) as u8,
        (kb::USB_VENDOR_ID >> 8) as u8,
        (kb::USB_PRODUCT_ID & 0xFF) as u8,
        (kb::USB_PRODUCT_ID >> 8) as u8,
        (libhmk_core::types::FIRMWARE_VERSION & 0xFF) as u8,
        (libhmk_core::types::FIRMWARE_VERSION >> 8) as u8,
    ])]
    pnp_id: [u8; 7],
}

#[gatt_service(uuid = service::BATTERY)]
struct BatteryService {
    #[characteristic(uuid = characteristic::BATTERY_LEVEL, read, notify, value = 100)]
    level: u8,
}

#[gatt_service(uuid = service::HUMAN_INTERFACE_DEVICE)]
struct HidService {
    #[characteristic(uuid = characteristic::HID_INFORMATION, read, value = [0x11, 0x01, 0x00, 0x03])]
    hid_info: [u8; 4],

    #[characteristic(uuid = characteristic::REPORT_MAP, read, value = BLE_HID_REPORT_MAP)]
    report_map: &'static [u8],

    #[characteristic(uuid = characteristic::PROTOCOL_MODE, read, write_without_response, value = 1)]
    protocol_mode: u8,

    #[characteristic(uuid = characteristic::HID_CONTROL_POINT, write_without_response, value = 0)]
    control_point: u8,

    #[characteristic(uuid = characteristic::BOOT_KEYBOARD_INPUT_REPORT, read, notify, value = [0u8; 8])]
    boot_kb_in: [u8; 8],

    #[characteristic(uuid = characteristic::BOOT_KEYBOARD_OUTPUT_REPORT, read, write, write_without_response, value = 0)]
    boot_kb_out: u8,

    // Report ID 1: Keyboard NKRO (28 bytes)
    #[descriptor(uuid = descriptors::REPORT_REFERENCE, read, value = [BLE_REPORT_ID_KEYBOARD, 0x01])]
    #[characteristic(uuid = characteristic::REPORT, read, notify, value = [0u8; KeyboardReport::SIZE])]
    kb_in: [u8; KeyboardReport::SIZE],

    #[descriptor(uuid = descriptors::REPORT_REFERENCE, read, value = [BLE_REPORT_ID_KEYBOARD, 0x02])]
    #[characteristic(uuid = characteristic::REPORT, read, write, write_without_response, value = 0)]
    kb_out: u8,

    // Report ID 2: System Control (2 bytes)
    #[descriptor(uuid = descriptors::REPORT_REFERENCE, read, value = [BLE_REPORT_ID_SYSTEM, 0x01])]
    #[characteristic(uuid = characteristic::REPORT, read, notify, value = [0u8; 2])]
    sys_in: [u8; 2],

    // Report ID 3: Consumer Control (2 bytes)
    #[descriptor(uuid = descriptors::REPORT_REFERENCE, read, value = [BLE_REPORT_ID_CONSUMER, 0x01])]
    #[characteristic(uuid = characteristic::REPORT, read, notify, value = [0u8; 2])]
    consumer_in: [u8; 2],

    // Report ID 4: Mouse (5 bytes)
    #[descriptor(uuid = descriptors::REPORT_REFERENCE, read, value = [BLE_REPORT_ID_MOUSE, 0x01])]
    #[characteristic(uuid = characteristic::REPORT, read, notify, value = [0u8; MouseReport::SIZE])]
    mouse_in: [u8; MouseReport::SIZE],

    // Report ID 5: Raw HID (64 bytes IN + OUT for hmkconf over BLE)
    #[descriptor(uuid = descriptors::REPORT_REFERENCE, read, value = [BLE_REPORT_ID_RAW_HID, 0x01])]
    #[characteristic(uuid = characteristic::REPORT, read, notify, value = [0u8; RAW_HID_EP_SIZE])]
    raw_in: [u8; RAW_HID_EP_SIZE],

    #[descriptor(uuid = descriptors::REPORT_REFERENCE, read, value = [BLE_REPORT_ID_RAW_HID, 0x02])]
    #[characteristic(uuid = characteristic::REPORT, read, write, write_without_response, value = [0u8; RAW_HID_EP_SIZE])]
    raw_out: [u8; RAW_HID_EP_SIZE],
}

//--------------------------------------------------------------------+
// Dual-Mode Transport & Power Management Controller
//--------------------------------------------------------------------+

pub struct NrfBleController {
    samples: &'static SharedAnalogSamples,
}

impl NrfBleController {
    pub const fn new(samples: &'static SharedAnalogSamples) -> Self {
        Self { samples }
    }

    pub fn transport_mode(&self) -> BleTransportMode {
        match TRANSPORT_MODE.load(Ordering::Relaxed) {
            1 => BleTransportMode::Usb,
            2 => BleTransportMode::Ble,
            _ => BleTransportMode::Auto,
        }
    }

    pub fn is_usb_active(&self) -> bool {
        match self.transport_mode() {
            BleTransportMode::Usb => true,
            BleTransportMode::Ble => false,
            BleTransportMode::Auto => is_usb_mounted(),
        }
    }

    pub fn is_ble_connected(&self) -> bool {
        BLE_CONNECTED.load(Ordering::Acquire)
    }

    /// Update Active / Idle / Sleep SAADC scan throttling when running on battery.
    pub fn update_power_state(&self) {
        if self.is_usb_active() {
            if POWER_STATE.swap(BlePowerState::Active as u8, Ordering::Relaxed)
                != BlePowerState::Active as u8
            {
                self.samples.set_scan_interval_us(0);
            }
            return;
        }

        let now = Instant::now().as_millis() as u32;
        let inactive = now.wrapping_sub(LAST_ACTIVITY_MS.load(Ordering::Relaxed));

        if kb::BLE_SLEEP_TIMEOUT_MS > 0 && inactive >= kb::BLE_SLEEP_TIMEOUT_MS {
            if POWER_STATE.swap(BlePowerState::Sleep as u8, Ordering::Relaxed)
                != BlePowerState::Sleep as u8
            {
                self.samples.set_scan_interval_us(20_000);
            }
        } else if kb::BLE_IDLE_TIMEOUT_MS > 0 && inactive >= kb::BLE_IDLE_TIMEOUT_MS {
            if POWER_STATE.swap(BlePowerState::Idle as u8, Ordering::Relaxed)
                != BlePowerState::Idle as u8
            {
                self.samples.set_scan_interval_us(5_000);
            }
        }
    }
}

impl BleControl for NrfBleController {
    fn toggle_transport(&mut self) {
        let next = match self.transport_mode() {
            BleTransportMode::Auto => {
                if self.is_usb_active() {
                    BleTransportMode::Ble
                } else {
                    BleTransportMode::Usb
                }
            }
            BleTransportMode::Usb => BleTransportMode::Ble,
            BleTransportMode::Ble => BleTransportMode::Auto,
        };
        self.set_transport_mode(next);
    }

    fn set_transport_mode(&mut self, mode: BleTransportMode) {
        let val = match mode {
            BleTransportMode::Auto => 0,
            BleTransportMode::Usb => 1,
            BleTransportMode::Ble => 2,
        };
        TRANSPORT_MODE.store(val, Ordering::Relaxed);
        self.notify_activity();
    }

    fn start_advertising(&mut self, disconnect_current: bool) {
        self.notify_activity();
        if disconnect_current && self.is_ble_connected() {
            BLE_DISCONNECT_SIGNAL.signal(());
        }
    }

    fn notify_activity(&mut self) {
        let now = Instant::now().as_millis() as u32;
        LAST_ACTIVITY_MS.store(now, Ordering::Relaxed);
        if POWER_STATE.swap(BlePowerState::Active as u8, Ordering::Relaxed)
            != BlePowerState::Active as u8
        {
            self.samples.set_scan_interval_us(0);
        }
    }
}

//--------------------------------------------------------------------+
// TrouBLE Async Runner Task
//--------------------------------------------------------------------+

pub async fn run_ble<C: Controller>(controller: C, samples: &'static SharedAnalogSamples) -> ! {
    let addr = Address::random(NrfBoard::ble_static_random_address());
    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
        HostResources::new();
    let stack = trouble_host::new(controller, &mut resources)
        .set_random_address(addr)
        .build();
    let mut runner = stack.runner();
    let mut peripheral = stack.peripheral();

    let server = KeyboardGattServer::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: kb::BLE_DEVICE_NAME,
        appearance: &appearance::human_interface_device::KEYBOARD,
    }))
    .unwrap();

    let runner_fut = async {
        loop {
            if let Err(e) = runner.run().await {
                warn!("[ble] runner error: {:?}", defmt::Debug2Format(&e));
            }
        }
    };

    let app_fut = async {
        loop {
            let mut adv_data = [0u8; 31];
            let adv_len = AdStructure::encode_slice(
                &[
                    AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
                    AdStructure::CompleteServiceUuids16(&[
                        0x1812u16.to_le_bytes(), // HID
                        0x180Fu16.to_le_bytes(), // Battery
                    ]),
                    AdStructure::CompleteLocalName(kb::BLE_DEVICE_NAME.as_bytes()),
                ],
                &mut adv_data,
            )
            .unwrap_or(0);

            let Ok(advertiser) = peripheral
                .advertise(
                    &Default::default(),
                    Advertisement::ConnectableScannableUndirected {
                        adv_data: &adv_data[..adv_len],
                        scan_data: &[],
                    },
                )
                .await
            else {
                Timer::after_millis(200).await;
                continue;
            };

            info!("[ble] advertising as {}", kb::BLE_DEVICE_NAME);
            let Ok(conn) = advertiser.accept().await else {
                continue;
            };
            let Ok(gatt_conn) = conn.with_attribute_server(&server) else {
                continue;
            };

            BLE_CONNECTED.store(true, Ordering::Release);
            info!("[ble] connected");

            let gatt_rx = gatt_events_task(&server, &gatt_conn);
            let hid_tx = ble_tx_and_battery_task(&server, &gatt_conn, samples);
            let disconnect_req = BLE_DISCONNECT_SIGNAL.wait();

            match select3(gatt_rx, hid_tx, disconnect_req).await {
                Either3::Third(()) => {
                    info!("[ble] disconnect requested for re-pairing");
                }
                _ => {}
            }

            BLE_CONNECTED.store(false, Ordering::Release);
            info!("[ble] disconnected");
        }
    };

    join(runner_fut, app_fut).await;
    unreachable!()
}

async fn gatt_events_task<P: PacketPool>(
    server: &KeyboardGattServer<'_>,
    conn: &GattConnection<'_, '_, P>,
) {
    let raw_out_handle = server.hids.raw_out.handle;
    loop {
        match conn.next().await {
            GattConnectionEvent::Disconnected { .. } => break,
            GattConnectionEvent::Gatt { event } => {
                let reply = match event {
                    GattEvent::Write(ev) => {
                        if ev.handle() == raw_out_handle {
                            ev.with_data(|_off, data| {
                                if !data.is_empty() && data.len() <= RAW_HID_EP_SIZE {
                                    let mut buf = [0u8; RAW_HID_EP_SIZE];
                                    buf[..data.len()].copy_from_slice(data);
                                    let _ = CMD_RX.try_send(UsbRawCommand {
                                        data: buf,
                                        transport: CommandTransport::Ble,
                                    });
                                }
                            });
                        }
                        ev.accept()
                    }
                    GattEvent::Read(ev) => ev.accept(),
                    _ => event.accept(),
                };
                if let Ok(r) = reply {
                    r.send().await;
                }
            }
            _ => {}
        }
    }
}

async fn ble_tx_and_battery_task<P: PacketPool>(
    server: &KeyboardGattServer<'_>,
    conn: &GattConnection<'_, '_, P>,
    samples: &'static SharedAnalogSamples,
) {
    let mut last_batt = samples.battery_percent();
    let _ = server.bas.level.notify(conn, &last_batt, true).await;

    let mut batt_ticker = Timer::after_secs(30);

    loop {
        match select(
            select(BLE_HID_TX.receive(), BLE_RAW_TX.receive()),
            &mut batt_ticker,
        )
        .await
        {
            Either::First(Either::First(pending)) => {
                if let Some(kb_rep) = pending.keyboard {
                    let proto = conn.get(&server.hids.protocol_mode).unwrap_or(1);
                    if proto == 0 {
                        let _ = server
                            .hids
                            .boot_kb_in
                            .notify(conn, &kb_rep.to_boot_bytes(), true)
                            .await;
                    } else {
                        let _ = server
                            .hids
                            .kb_in
                            .notify(conn, &kb_rep.to_bytes(), true)
                            .await;
                    }
                }
                if let Some(sys) = pending.system {
                    let _ = server.hids.sys_in.notify(conn, &sys.to_le_bytes(), true).await;
                }
                if let Some(cons) = pending.consumer {
                    let _ = server
                        .hids
                        .consumer_in
                        .notify(conn, &cons.to_le_bytes(), true)
                        .await;
                }
                if let Some(mouse) = pending.mouse {
                    let _ = server
                        .hids
                        .mouse_in
                        .notify(conn, &mouse.to_bytes(), true)
                        .await;
                }
            }
            Either::First(Either::Second(raw_resp)) => {
                let _ = server.hids.raw_in.notify(conn, &raw_resp, true).await;
            }
            Either::Second(()) => {
                let pct = samples.battery_percent();
                if pct != last_batt {
                    last_batt = pct;
                    let _ = server.bas.level.notify(conn, &pct, true).await;
                }
                batt_ticker = Timer::after_secs(30);
            }
        }
    }
}
