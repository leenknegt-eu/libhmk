//! Optional GPIO timing probes (`scan-probe` feature) for scope / logic analyzer measurements.
//!
//! Without the feature every method is an empty inline function, so the probes cost nothing.
//! See `docs/performance.md` for the measurement procedure.

#[cfg(feature = "scan-probe")]
use embassy_nrf::gpio::{AnyPin, Level, Output, OutputDrive};
#[cfg(feature = "scan-probe")]
use embassy_nrf::Peri;

pub struct Probe {
    #[cfg(feature = "scan-probe")]
    pin: Option<Output<'static>>,
}

impl Probe {
    /// A probe that does nothing (builds without `scan-probe`).
    #[cfg(not(feature = "scan-probe"))]
    pub const fn disabled() -> Self {
        Self {}
    }

    #[cfg(feature = "scan-probe")]
    pub fn new(pin: Option<Peri<'static, AnyPin>>) -> Self {
        Self {
            pin: pin.map(|p| Output::new(p, Level::Low, OutputDrive::Standard)),
        }
    }

    #[inline(always)]
    pub fn high(&mut self) {
        #[cfg(feature = "scan-probe")]
        if let Some(p) = &mut self.pin {
            p.set_high();
        }
    }

    #[inline(always)]
    pub fn low(&mut self) {
        #[cfg(feature = "scan-probe")]
        if let Some(p) = &mut self.pin {
            p.set_low();
        }
    }

    #[inline(always)]
    pub fn toggle(&mut self) {
        #[cfg(feature = "scan-probe")]
        if let Some(p) = &mut self.pin {
            p.toggle();
        }
    }
}
