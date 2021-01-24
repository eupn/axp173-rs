pub(crate) const VBUS_VOLTAGE_COEFF: u16 = 17;
pub(crate) const BATT_VOLTAGE_COEFF: u16 = 11;

pub(crate) const VBUS_CURRENT_COEFF: u16 = 375;
pub(crate) const VBUS_CURRENT_DIV: u16 = 1000;

pub(crate) const BATT_CURRENT_COEFF: u16 = 1;
pub(crate) const BATT_CURRENT_DIV: u16 = 2;

/// The weight of the least significant bit of the current measurement in milliAmps.
pub(crate) const CURRENT_LSB: f32 = 0.5;

/// Defines a 12-bit voltage reading.
#[derive(Debug, Copy, Clone)]
pub struct Voltage {
    raw: u16,
    mul: u16,
}

impl Voltage {
    pub(crate) fn new(raw: u16, mul: u16) -> Self {
        Self { raw, mul }
    }

    /// Returns milliVolts value.
    #[allow(clippy::trivially_copy_pass_by_ref)] // On 32-bit ARMs it is not efficient to pass by value
    pub fn as_millivolts(&self) -> u16 {
        self.raw * self.mul / 10
    }

    /// Returns volts value.
    #[allow(clippy::trivially_copy_pass_by_ref)] // On 32-bit ARMs it is not efficient to pass by value
    pub fn as_volts(&self) -> f32 {
        self.as_millivolts() as f32 / 1000.0_f32
    }

    /// Returns raw ADC reading value.
    #[allow(clippy::trivially_copy_pass_by_ref)] // On 32-bit ARMs it is not efficient to pass by value
    pub fn raw(&self) -> u16 {
        self.raw
    }
}

/// Defines a 13-bit current reading.
#[derive(Debug, Copy, Clone)]
pub struct Current {
    raw: u16,
    mul: u16,
    div: u16,
}

impl Current {
    pub(crate) fn new(raw: u16, mul: u16, div: u16) -> Self {
        assert!(div > 0);

        Self { raw, mul, div }
    }

    /// Returns milliAmperes value.
    #[allow(clippy::trivially_copy_pass_by_ref)] // On 32-bit ARMs it is not efficient to pass by value
    pub fn as_milliamps(&self) -> f32 {
        self.raw as f32 * self.mul as f32 / self.div as f32
    }

    /// Returns Amperes value.
    #[allow(clippy::trivially_copy_pass_by_ref)] // On 32-bit ARMs it is not efficient to pass by value
    pub fn as_amperes(&self) -> f32 {
        self.as_milliamps() as f32 / 1000.0_f32
    }

    /// Returns raw ADC reading value.
    #[allow(clippy::trivially_copy_pass_by_ref)] // On 32-bit ARMs it is not efficient to pass by value
    pub fn raw(&self) -> u16 {
        self.raw
    }
}
