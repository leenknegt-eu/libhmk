//! Backend-neutral wireless control: USB/wireless transport selection, battery power states,
//! persistent link settings and switching between the 2.4 GHz (ESB) and BLE backends.
//!
//! Only one radio stack runs at a time (the BLE controller owns the RADIO once started), so the
//! backend is chosen at boot from the stored [`LinkSettings`]; switching saves the new choice
//! and reboots.

use core::cell::RefCell;
use core::sync::atomic::{AtomicU32, AtomicU8, Ordering};

use critical_section::Mutex;
use embassy_time::Instant;
use libhmk_core::traits::{BleControl, BlePowerState, BleTransportMode, WirelessBackend};
use libhmk_link::{Backend, LinkSettings, TransportPref};

use crate::analog::SharedAnalogSamples;
use crate::{ble, kb, rf};

static POWER_STATE: AtomicU8 = AtomicU8::new(BlePowerState::Active as u8);
static LAST_ACTIVITY_MS: AtomicU32 = AtomicU32::new(0);

/// Settings currently in effect (loaded at boot, updated by key presses).
static SETTINGS: Mutex<RefCell<LinkSettings>> =
    Mutex::new(RefCell::new(LinkSettings::new(Backend::Esb)));
/// Settings change waiting for the main loop to write it to flash: (settings, reboot after).
static PENDING_WRITE: Mutex<RefCell<Option<(LinkSettings, bool)>>> = Mutex::new(RefCell::new(None));

/// Backend this firmware build boots into when nothing is stored.
pub const fn default_backend() -> Backend {
    if kb::ESB_ENABLED && !kb::WIRELESS_DEFAULT_BLE {
        Backend::Esb
    } else if kb::BLUETOOTH_ENABLED {
        Backend::Ble
    } else {
        Backend::Esb
    }
}

/// Map a stored backend to one this build supports.
pub const fn resolve_backend(stored: Backend) -> Option<Backend> {
    match stored {
        Backend::Esb if kb::ESB_ENABLED => Some(Backend::Esb),
        Backend::Ble if kb::BLUETOOTH_ENABLED => Some(Backend::Ble),
        _ if kb::ESB_ENABLED => Some(Backend::Esb),
        _ if kb::BLUETOOTH_ENABLED => Some(Backend::Ble),
        _ => None,
    }
}

/// Install the settings loaded at boot.
pub fn init(settings: LinkSettings) {
    critical_section::with(|cs| *SETTINGS.borrow_ref_mut(cs) = settings);
    LAST_ACTIVITY_MS.store(Instant::now().as_millis() as u32, Ordering::Relaxed);
}

pub fn settings() -> LinkSettings {
    critical_section::with(|cs| *SETTINGS.borrow_ref(cs))
}

/// Queue a settings change; the main loop writes it to flash (and reboots if requested).
pub fn request_settings_write(settings: LinkSettings, reboot: bool) {
    critical_section::with(|cs| {
        *SETTINGS.borrow_ref_mut(cs) = settings;
        let mut pending = PENDING_WRITE.borrow_ref_mut(cs);
        let reboot = reboot || pending.is_some_and(|(_, r)| r);
        *pending = Some((settings, reboot));
    });
}

/// Take a queued settings change (called by the main loop, which owns the flash).
pub fn take_pending_write() -> Option<(LinkSettings, bool)> {
    critical_section::with(|cs| PENDING_WRITE.borrow_ref_mut(cs).take())
}

/// Battery Idle/Sleep power state is active (radio heartbeats slow down).
pub fn low_power() -> bool {
    POWER_STATE.load(Ordering::Relaxed) != BlePowerState::Active as u8
}

pub struct WirelessController {
    samples: &'static SharedAnalogSamples,
    backend: Option<Backend>,
}

impl WirelessController {
    /// `backend` is the stack that was started at boot (`None` for USB-only builds).
    pub const fn new(samples: &'static SharedAnalogSamples, backend: Option<Backend>) -> Self {
        Self { samples, backend }
    }

    pub fn backend(&self) -> Option<Backend> {
        self.backend
    }

    pub fn transport_mode(&self) -> BleTransportMode {
        match settings().transport {
            TransportPref::Usb => BleTransportMode::Usb,
            TransportPref::Wireless => BleTransportMode::Wireless,
            TransportPref::Auto => BleTransportMode::Auto,
        }
    }

    pub fn is_usb_active(&self) -> bool {
        if self.backend.is_none() {
            return true;
        }
        match self.transport_mode() {
            BleTransportMode::Usb => true,
            BleTransportMode::Wireless => false,
            BleTransportMode::Auto => crate::usb::is_usb_mounted(),
        }
    }

    pub fn is_wireless_connected(&self) -> bool {
        match self.backend {
            Some(Backend::Ble) => ble::is_connected(),
            Some(Backend::Esb) => rf::link_up(),
            None => false,
        }
    }

    /// Update Active / Idle / Sleep SAADC scan throttling when running on battery.
    pub fn update_power_state(&self) {
        if self.is_usb_active() {
            self.set_power_state(BlePowerState::Active, 0);
            return;
        }
        let now = Instant::now().as_millis() as u32;
        let inactive = now.wrapping_sub(LAST_ACTIVITY_MS.load(Ordering::Relaxed));
        if kb::BLE_SLEEP_TIMEOUT_MS > 0 && inactive >= kb::BLE_SLEEP_TIMEOUT_MS {
            self.set_power_state(BlePowerState::Sleep, 20_000);
        } else if kb::BLE_IDLE_TIMEOUT_MS > 0 && inactive >= kb::BLE_IDLE_TIMEOUT_MS {
            self.set_power_state(BlePowerState::Idle, 5_000);
        }
    }

    fn set_power_state(&self, state: BlePowerState, scan_interval_us: u32) {
        if POWER_STATE.swap(state as u8, Ordering::Relaxed) != state as u8 {
            self.samples.set_scan_interval_us(scan_interval_us);
        }
    }
}

impl BleControl for WirelessController {
    fn toggle_transport(&mut self) {
        let next = match self.transport_mode() {
            BleTransportMode::Auto => {
                if self.is_usb_active() {
                    BleTransportMode::Wireless
                } else {
                    BleTransportMode::Usb
                }
            }
            BleTransportMode::Usb => BleTransportMode::Wireless,
            BleTransportMode::Wireless => BleTransportMode::Auto,
        };
        self.set_transport_mode(next);
    }

    fn set_transport_mode(&mut self, mode: BleTransportMode) {
        let transport = match mode {
            BleTransportMode::Auto => TransportPref::Auto,
            BleTransportMode::Usb => TransportPref::Usb,
            BleTransportMode::Wireless => TransportPref::Wireless,
        };
        let mut s = settings();
        if s.transport != transport {
            s.transport = transport;
            request_settings_write(s, false);
        }
        self.notify_activity();
    }

    fn start_advertising(&mut self, disconnect_current: bool) {
        self.notify_activity();
        match self.backend {
            Some(Backend::Ble) => {
                if disconnect_current && ble::is_connected() {
                    ble::request_disconnect();
                }
            }
            Some(Backend::Esb) => rf::start_pairing(),
            None => {}
        }
    }

    fn select_wireless_backend(&mut self, backend: WirelessBackend) {
        let wanted = match backend {
            WirelessBackend::Esb => Backend::Esb,
            WirelessBackend::Ble => Backend::Ble,
        };
        let available = match wanted {
            Backend::Esb => kb::ESB_ENABLED,
            Backend::Ble => kb::BLUETOOTH_ENABLED,
        };
        if !available || self.backend == Some(wanted) {
            return;
        }
        let mut s = settings();
        s.backend = wanted;
        // The other radio stack can only be started from reset.
        request_settings_write(s, true);
    }

    fn notify_activity(&mut self) {
        LAST_ACTIVITY_MS.store(Instant::now().as_millis() as u32, Ordering::Relaxed);
        self.set_power_state(BlePowerState::Active, 0);
    }
}
