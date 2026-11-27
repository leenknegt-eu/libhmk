//! Hardware abstraction traits implemented by each MCU firmware crate.

/// Output transport mode for dual-mode USB/BLE keyboards.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum BleTransportMode {
    /// Prefer USB when mounted, automatically fall back to BLE otherwise.
    #[default]
    Auto,
    /// Force USB transport.
    Usb,
    /// Force BLE transport even when USB VBUS is present.
    Ble,
}

/// Current BLE link state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum BleState {
    #[default]
    Disabled,
    Idle,
    Advertising,
    Connected,
}

/// Low-power state for battery-powered operation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum BlePowerState {
    #[default]
    Active,
    Idle,
    Sleep,
}

/// Transport over which a raw HID configurator command was received.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum CommandTransport {
    #[default]
    Usb,
    Ble,
}

/// Internal flash driver abstraction used by wear leveling.
pub trait FlashStorage {
    /// Total usable flash size in bytes.
    fn flash_size(&self) -> u32;
    /// Total number of erasable flash sectors.
    fn num_sectors(&self) -> u32;
    /// Empty word value after erase (typically `0xFFFF_FFFF`).
    fn empty_value(&self) -> u32 {
        0xFFFF_FFFF
    }
    /// Size in bytes of the given sector index, or `0` if out of range.
    fn sector_size(&self, sector: u32) -> u32;
    /// Erase a single flash sector.
    fn erase_sector(&mut self, sector: u32) -> bool;
    /// Read `buf.len()` bytes starting at byte offset `addr` from flash base.
    fn read(&self, addr: u32, buf: &mut [u8]) -> bool;
    /// Write `data` (word-aligned length) starting at byte offset `addr`.
    fn write(&mut self, addr: u32, data: &[u8]) -> bool;
}

/// Board-level control and identity hooks.
pub trait BoardControl {
    /// Reboot the MCU immediately.
    fn reset(&mut self);
    /// Reboot into the MCU bootloader (DFU / UF2).
    fn enter_bootloader(&mut self);
    /// Write up to 32 ASCII serial characters into `buf` and return the length.
    fn serial(&self, buf: &mut [u8; 32]) -> usize;
    /// Current monotonic timestamp in milliseconds.
    fn now_ms(&self) -> u32;
}

/// Analog key sensor input abstraction.
pub trait AnalogInput {
    /// Read the latest raw ADC sample for `key`.
    fn read_raw(&self, key: usize) -> u16;
}

/// Optional Bluetooth control hooks invoked by layout keycodes and commands.
pub trait BleControl {
    fn toggle_transport(&mut self) {}
    fn set_transport_mode(&mut self, _mode: BleTransportMode) {}
    fn start_advertising(&mut self, _disconnect_current: bool) {}
    fn notify_activity(&mut self) {}
}

/// No-op Bluetooth controller for USB-only MCUs (STM32F446, AT32F405).
#[derive(Debug, Default, Clone, Copy)]
pub struct NoopBleControl;

impl BleControl for NoopBleControl {}
