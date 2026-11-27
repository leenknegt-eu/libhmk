//! MCU driver hardware metadata (flash geometry, ADC pin mapping, bootloader).

#[derive(Debug, Clone)]
pub enum FlashSectors {
    Uniform { size: u32, num_sectors: u32 },
    NonUniform { sizes: &'static [u32] },
}

#[derive(Debug, Clone)]
pub struct FlashMetadata {
    pub base_addr: u32,
    pub sectors: FlashSectors,
    pub empty_value: u32,
}

impl FlashMetadata {
    pub fn num_sectors(&self) -> u32 {
        match self.sectors {
            FlashSectors::Uniform { num_sectors, .. } => num_sectors,
            FlashSectors::NonUniform { sizes } => sizes.len() as u32,
        }
    }

    pub fn total_size(&self) -> u32 {
        match self.sectors {
            FlashSectors::Uniform { size, num_sectors } => size * num_sectors,
            FlashSectors::NonUniform { sizes } => sizes.iter().sum(),
        }
    }

    pub fn sector_size(&self, sector: u32) -> u32 {
        match self.sectors {
            FlashSectors::Uniform { size, num_sectors } => {
                assert!(sector < num_sectors);
                size
            }
            FlashSectors::NonUniform { sizes } => sizes[sector as usize],
        }
    }

    /// Round up `required_size` to whole sectors starting from the end of flash.
    pub fn round_up_to_sectors(&self, required_size: u32) -> u32 {
        assert!(required_size <= self.total_size());
        let mut total = 0u32;
        for i in (0..self.num_sectors()).rev() {
            total += self.sector_size(i);
            if total >= required_size {
                return total;
            }
        }
        total
    }
}

#[derive(Debug, Clone)]
pub struct BootloaderMetadata {
    pub address: u32,
    pub magic: u32,
}

#[derive(Debug, Clone)]
pub struct AdcMetadata {
    pub max_resolution: u8,
    pub input_pins: &'static [&'static str],
}

impl AdcMetadata {
    pub fn pin_to_channel(&self, pin: &str) -> Result<u8, String> {
        self.input_pins
            .iter()
            .position(|&p| p == pin)
            .map(|idx| idx as u8)
            .ok_or_else(|| format!("Unknown ADC input pin '{pin}'"))
    }
}

#[derive(Debug, Clone)]
pub struct DriverMetadata {
    pub name: &'static str,
    pub ram_origin: u32,
    pub ram_size: u32,
    pub bootloader: BootloaderMetadata,
    pub flash: FlashMetadata,
    pub adc: AdcMetadata,
    /// Pins that must never be repurposed (USB, SWD, crystals, reset), as written in
    /// `keyboard.json` pin names.
    pub reserved_pins: &'static [&'static str],
}

pub const STM32F446XX: DriverMetadata = DriverMetadata {
    name: "stm32f446xx",
    ram_origin: 0x2000_0000,
    ram_size: 128 * 1024 - 4, // 4 bytes reserved at top of RAM for bootloader magic
    bootloader: BootloaderMetadata {
        address: 0x1FFF_0000,
        magic: 0xDEAD_BEEF,
    },
    flash: FlashMetadata {
        base_addr: 0x0800_0000,
        sectors: FlashSectors::NonUniform {
            sizes: &[
                16 * 1024,
                16 * 1024,
                16 * 1024,
                16 * 1024,
                64 * 1024,
                128 * 1024,
                128 * 1024,
                128 * 1024,
            ],
        },
        empty_value: 0xFFFF_FFFF,
    },
    adc: AdcMetadata {
        max_resolution: 12,
        input_pins: &[
            "A0", "A1", "A2", "A3", "A4", "A5", "A6", "A7", "B0", "B1", "C0", "C1", "C2", "C3",
            "C4", "C5",
        ],
    },
    reserved_pins: &["A11", "A12", "A13", "A14"],
};

pub const AT32F405XX: DriverMetadata = DriverMetadata {
    name: "at32f405xx",
    ram_origin: 0x2000_0000,
    ram_size: 102 * 1024 - 4,
    bootloader: BootloaderMetadata {
        address: 0x1FFF_A400,
        magic: 0xDEAD_BEEF,
    },
    flash: FlashMetadata {
        base_addr: 0x0800_0000,
        sectors: FlashSectors::Uniform {
            size: 2048,
            num_sectors: 128,
        },
        empty_value: 0xFFFF_FFFF,
    },
    adc: AdcMetadata {
        max_resolution: 12,
        input_pins: &[
            "A0", "A1", "A2", "A3", "A4", "A5", "A6", "A7", "B0", "B1", "C0", "C1", "C2", "C3",
            "C4", "C5",
        ],
    },
    reserved_pins: &["A11", "A12", "A13", "A14", "B14", "B15"],
};

pub const NRF52840XX: DriverMetadata = DriverMetadata {
    name: "nrf52840xx",
    ram_origin: 0x2000_0000,
    ram_size: 256 * 1024,
    bootloader: BootloaderMetadata {
        address: 0x000F_4000,
        magic: 0xB1,
    },
    flash: FlashMetadata {
        base_addr: 0x0000_0000,
        // 244 x 4 KB = 976 KB usable below optional 48 KB Nordic DFU bootloader region
        sectors: FlashSectors::Uniform {
            size: 4096,
            num_sectors: 244,
        },
        empty_value: 0xFFFF_FFFF,
    },
    adc: AdcMetadata {
        max_resolution: 12,
        input_pins: &[
            "P0.02", "P0.03", "P0.04", "P0.05", "P0.28", "P0.29", "P0.30", "P0.31",
        ],
    },
    reserved_pins: &["P0.00", "P0.01", "P0.18"],
};

pub fn driver_by_name(name: &str) -> Result<&'static DriverMetadata, String> {
    match name {
        "stm32f446xx" => Ok(&STM32F446XX),
        "at32f405xx" => Ok(&AT32F405XX),
        "nrf52840xx" => Ok(&NRF52840XX),
        other => Err(format!("Unsupported hardware driver: '{other}'")),
    }
}
