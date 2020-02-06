/// Voltage regulators in AXP173 that can be enabled or disabled.
pub enum LdoKind {
    /// LDO2.
    LDO2,

    /// LDO3.
    LDO3,

    /// LDO4.
    LDO4,
}

/// LDO of AXP173.
pub struct Ldo {
    pub(crate) kind: LdoKind,
    pub(crate) voltage: u8,
}

impl Ldo {
    /// Selects LDO2 voltage.
    /// `voltage`: four bit voltage: 1.8 - 3.3V, 100 mV per LSB.
    /// # Panics
    /// Panics if supplied voltage value is greater than 2^4 due to failed assertion.
    pub fn ldo2_with_voltage(voltage: u8) -> Self {
        assert!(voltage <= 0b1111); // Only 4-bit wide

        Self {
            kind: LdoKind::LDO2,
            voltage,
        }
    }

    /// Selects LDO3 voltage.
    /// `voltage`: four bit voltage: 1.8 - 3.3V, 100 mV per LSB.
    ///
    /// # Panics
    /// Panics if supplied voltage value is greater than 2^4 due to failed assertion.
    pub fn ldo3_with_voltage(voltage: u8) -> Self {
        assert!(voltage <= 0b1111); // Only 4-bit wide

        Self {
            kind: LdoKind::LDO3,
            voltage,
        }
    }

    /// Selects LDO4 voltage.
    /// `voltage`: seven bit voltage: 0.7 - 3.5V, 25 mV per LSB.
    ///
    /// # Panics
    /// Panics if supplied voltage value is greater than 2^7 due to failed assertion.
    pub fn ldo4_with_voltage(voltage: u8) -> Self {
        assert!(voltage <= 0b111_1111); // Only 7-bit wide

        Self {
            kind: LdoKind::LDO4,
            voltage,
        }
    }
}
