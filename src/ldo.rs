/// Newtype wrapper for MilliVolts.
#[derive(Debug, Copy, Clone)]
pub struct MilliVolts(pub usize);

/// Voltage regulators in AXP173 that can be enabled or disabled.
#[derive(Debug)]
pub enum LdoKind {
    /// LDO2.
    LDO2,

    /// LDO3.
    LDO3,

    /// LDO4.
    LDO4,
}

/// LDO of AXP173.
#[derive(Debug)]
pub struct Ldo {
    pub(crate) kind: LdoKind,
    pub(crate) voltage: u8,
    pub(crate) enabled: bool,
}

impl Ldo {
    /// Selects LDO2 voltage.
    /// `voltage`: four bit voltage: 1.8 - 3.3V, 100 mV per LSB.
    /// # Panics
    /// Panics if supplied voltage value is greater than 2^4 due to failed assertion.
    pub fn ldo2_with_voltage(voltage: u8, enabled: bool) -> Self {
        assert!(voltage <= 0b1111); // Only 4-bit wide

        Self {
            kind: LdoKind::LDO2,
            voltage,
            enabled,
        }
    }

    /// Selects LDO3 voltage.
    /// `voltage`: four bit voltage: 1.8 - 3.3V, 100 mV per LSB.
    ///
    /// # Panics
    /// Panics if supplied voltage value is greater than 2^4 due to failed assertion.
    pub fn ldo3_with_voltage(voltage: u8, enabled: bool) -> Self {
        assert!(voltage <= 0b1111); // Only 4-bit wide

        Self {
            kind: LdoKind::LDO3,
            voltage,
            enabled,
        }
    }

    /// Selects LDO4 voltage.
    /// `voltage`: seven bit voltage: 0.7 - 3.5V, 25 mV per LSB.
    ///
    /// # Panics
    /// Panics if supplied voltage value is greater than 2^7 due to failed assertion.
    pub fn ldo4_with_voltage(voltage: u8, enabled: bool) -> Self {
        assert!(voltage <= 0b111_1111); // Only 7-bit wide

        Self {
            kind: LdoKind::LDO4,
            voltage,
            enabled,
        }
    }

    /// Returns LDO voltage in milliVolts.
    pub fn voltage(&self) -> MilliVolts {
        match self.kind {
            LdoKind::LDO2 => {
                assert!(self.voltage <= 0b1111);
                MilliVolts(1800 + self.voltage as usize * 100)
            }
            LdoKind::LDO3 => {
                assert!(self.voltage <= 0b1111);
                MilliVolts(1800 + self.voltage as usize * 100)
            }
            LdoKind::LDO4 => {
                assert!(self.voltage <= 0b111_1111);
                MilliVolts(700 + self.voltage as usize * 25)
            }
        }
    }

    /// Returns `true` if this LDO is currently enabled.
    pub fn enabled(&self) -> bool {
        self.enabled
    }

    pub(crate) fn from_voltage_and_enabled(
        kind: LdoKind,
        voltage_bits: u8,
        ldo_enabled: bool,
    ) -> Self {
        match kind {
            LdoKind::LDO2 => Ldo::ldo2_with_voltage(voltage_bits, ldo_enabled),
            LdoKind::LDO3 => Ldo::ldo3_with_voltage(voltage_bits, ldo_enabled),
            LdoKind::LDO4 => Ldo::ldo4_with_voltage(voltage_bits, ldo_enabled),
        }
    }
}
