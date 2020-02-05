#![no_std]

///! X-Powers AXP173 Power Management IC
///! Datasheet: [/doc/AXP173 Datasheet v1.12_cn.zh-CN.en.pdf]

use core::ops::Range;

use embedded_hal::blocking::i2c::{Write, WriteRead};

use bit_field::BitField;
use bitflags::bitflags;
use byteorder::{ByteOrder, LittleEndian};

/// Default values for the on-chip buffer.
/// Datasheet, p. 28, section 9.11.
const AXP173_ON_CHIP_BUFFER_DEFAULT: [u8; 6] = [0xf0, 0x0f, 0x00, 0xff, 0x00, 0x00];

/// All possible errors in this crate
#[derive(Debug)]
pub enum Error<E> {
    /// I2C bus error
    I2c(E),

    /// Communication failure or invalid chip used
    InvalidChip([u8; 6]),
}

pub struct Axp173<I> {
    i2c: I,
}

impl<I, E> Axp173<I>
where
    I: WriteRead<Error = E> + Write<Error = E>,
{
    /// Side-effect-free constructor.
    /// Nothing will be read or written before `init()` call.
    pub fn new(i2c: I) -> Self {
        let axp = Axp173 { i2c };

        axp
    }

    /// Checks the I2C connection to the AXP173 chip.
    /// AXP173 doesn't have a dedicated chip ID register and connection is checked by reading
    /// on-chip buffer for a presence of default values.
    pub fn init(&mut self) -> Result<(), Error<E>> {
        self.read_u8(POWER_DATA_BUFFER1).map_err(Error::I2c)?;

        let buf = self.read_onchip_buffer()?;

        match buf {
            AXP173_ON_CHIP_BUFFER_DEFAULT => Ok(()),
            _ => Err(Error::InvalidChip(buf)),
        }
    }

    pub fn read_onchip_buffer(&mut self) -> Result<[u8; 6], Error<E>> {
        let mut buf = [0u8; 6];
        self.read_bytes(POWER_DATA_BUFFER1, &mut buf)
            .map_err(Error::I2c)?;

        Ok(buf)
    }

    /// Returns `true` if device is connected to the USB power source.
    pub fn vbus_present(&mut self) -> Result<bool, Error<E>> {
        let reg_val = self.read_u8(POWER_STATUS).map_err(Error::I2c)?;

        Ok(reg_val.get_bit(POWER_STATUS_VBUS_PRESENT))
    }

    /// Returns `true` if lithium battery is connected.
    pub fn battery_present(&mut self) -> Result<bool, Error<E>> {
        let reg_val = self.read_u8(POWER_MODE_CHGSTATUS).map_err(Error::I2c)?;

        Ok(reg_val.get_bit(POWER_MODE_CHGSTATUS_BATTERY_PRESENT))
    }

    /// Returns `true` if lithium battery is connected and currently charging.
    pub fn battery_charging(&mut self) -> Result<bool, Error<E>> {
        let reg_val = self.read_u8(POWER_MODE_CHGSTATUS).map_err(Error::I2c)?;

        Ok(reg_val.get_bit(POWER_MODE_CHGSTATUS_IS_CHARGING))
    }

    /// Enables selected LDO with selected output voltage.
    pub fn enable_ldo(&mut self, ldo: &Ldo) -> Result<(), Error<E>> {
        self.set_ldo_voltage(&ldo)?;
        self.switch_ldo(&ldo.kind, true)
    }

    /// Disables selected LDO.
    pub fn disable_ldo(&mut self, ldo: &LdoKind) -> Result<(), Error<E>> {
        self.switch_ldo(ldo, false)
    }

    fn switch_ldo(&mut self, ldo: &LdoKind, enable: bool) -> Result<(), Error<E>> {
        let bit = match ldo {
            LdoKind::LDO2 => POWER_ON_OFF_REG_LDO2_ON,
            LdoKind::LDO3 => POWER_ON_OFF_REG_LDO3_ON,
            LdoKind::LDO4 => POWER_ON_OFF_REG_LDO4_ON,
        };

        let mut bits = self.read_u8(POWER_ON_OFF_REG).map_err(Error::I2c)?;

        bits.set_bit(bit, enable);

        self.write_u8(POWER_ON_OFF_REG, bits).map_err(Error::I2c)?;

        Ok(())
    }

    fn set_ldo_voltage(&mut self, ldo: &Ldo) -> Result<(), Error<E>> {
        let reg = match &ldo.kind {
            LdoKind::LDO2 | LdoKind::LDO3 => LDO2_LDO3_OUT_VOL_REG,
            LdoKind::LDO4 => LDO4_OUT_VOL_REG,
        };

        let voltage = ldo.voltage;

        let mut bits = self.read_u8(reg).map_err(Error::I2c)?;

        let bits_range = match &ldo.kind {
            LdoKind::LDO2 => 4..8,
            LdoKind::LDO3 => 0..4,
            LdoKind::LDO4 => 0..7,
        };

        bits.set_bits(bits_range, voltage);

        self.write_u8(reg, bits).map_err(Error::I2c)?;

        Ok(())
    }

    /// Sets charging current of the battery. Adjust this for an efficient and safe charging of
    /// your lithium battery's capacity.
    pub fn set_charging_current(&mut self, current: ChargingCurrent) -> Result<(), Error<E>> {
        let mut bits = self.read_u8(POWER_CHARGE1).map_err(Error::I2c)?;
        bits.set_bits(POWER_CHARGE1_CURRENT_SETTING, current.bits());

        self.write_u8(POWER_CHARGE1, bits).map_err(Error::I2c)?;

        Ok(())
    }

    /// Sets battery charging regulation voltage. This value varies depending on battery's chemistry
    /// and should be as close to the value from battery's datasheet as possible to ensure
    /// efficient charging and long battery life.
    pub fn set_charging_voltage(&mut self, voltage: ChargingVoltage) -> Result<(), Error<E>> {
        let mut bits = self.read_u8(POWER_CHARGE1).map_err(Error::I2c)?;
        bits.set_bits(POWER_CHARGE1_VOLTAGE_SETTING, voltage.bits());

        self.write_u8(POWER_CHARGE1, bits).map_err(Error::I2c)?;

        Ok(())
    }

    /// Enables or disables battery charging.
    pub fn set_charging(&mut self, enabled: bool) -> Result<(), Error<E>> {
        let mut bits = self.read_u8(POWER_CHARGE1).map_err(Error::I2c)?;
        bits.set_bit(POWER_CHARGE1_ENABLE_CHARGING, enabled);

        self.write_u8(POWER_CHARGE1, bits).map_err(Error::I2c)?;

        Ok(())
    }

    /// Enables or disables certain ADC functions of AXP173.
    pub fn set_adc_settings(&mut self, adc_settings: &AdcSettings) -> Result<(), Error<E>> {
        // Set the contents of ADC enable/disable register.
        let mut bits = self.read_u8(POWER_ADC_EN1).map_err(Error::I2c)?;
        adc_settings.write_adc_en_bits(&mut bits);
        self.write_u8(POWER_ADC_EN1, bits).map_err(Error::I2c)?;

        // Set ADC sample rate
        let mut bits = self.read_u8(POWER_ADC_SPEED_TS).map_err(Error::I2c)?;
        adc_settings.write_adc_sample_rate_and_ts_bits(&mut bits);
        self.write_u8(POWER_ADC_SPEED_TS, bits).map_err(Error::I2c)?;

        Ok(())
    }

    fn read_u8(&mut self, reg: u8) -> Result<u8, E> {
        let mut byte: [u8; 1] = [0; 1];

        match self.i2c.write_read(AXP173_ADDR, &[reg], &mut byte) {
            Ok(_) => Ok(byte[0]),
            Err(e) => Err(e),
        }
    }

    fn read_bytes(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), E> {
        self.i2c.write_read(AXP173_ADDR, &[reg], buf)
    }

    fn write_u8(&mut self, reg: u8, value: u8) -> Result<(), E> {
        self.i2c.write(AXP173_ADDR, &[reg, value])?;

        Ok(())
    }
}

/// Voltage regulators in AXP173 that can be enabled or disabled.
pub enum LdoKind {
    LDO2,
    LDO3,
    LDO4,
}

/// LDO of AXP173.
pub struct Ldo {
    kind: LdoKind,
    voltage: u8,
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
        assert!(voltage <= 0b1111111); // Only 7-bit wide

        Self {
            kind: LdoKind::LDO4,
            voltage,
        }
    }
}

/// ADC settings structure
#[derive(Debug)]
pub struct AdcSettings {
    batt_voltage_adc_enable: Option<bool>,
    batt_current_adc_enable: Option<bool>,

    acin_voltage_adc_enable: Option<bool>,
    acin_current_adc_enable: Option<bool>,

    vbus_voltage_adc_enable: Option<bool>,
    vbus_current_adc_enable: Option<bool>,

    aps_voltage_adc_enable: Option<bool>,
    ts_adc_enable: Option<bool>,

    sample_rate: Option<AdcSampleRate>,
    use_ts_for_battery_temperature: Option<bool>,
    ts_pin_mode: Option<TsPinMode>,
}

impl Default for AdcSettings {
    /// Default state of settings, nothing is affected.
    fn default() -> Self {
        Self {
            batt_voltage_adc_enable: None,
            batt_current_adc_enable: None,
            acin_voltage_adc_enable: None,
            acin_current_adc_enable: None,
            vbus_voltage_adc_enable: None,
            vbus_current_adc_enable: None,
            aps_voltage_adc_enable: None,
            ts_adc_enable: None,

            sample_rate: None,
            use_ts_for_battery_temperature: None,
            ts_pin_mode: None,
        }
    }
}

impl AdcSettings {
    /// Battery voltage sensing ADC on/off.
    pub fn batt_voltage_adc(&mut self, enabled: bool) -> &mut Self {
        self.batt_voltage_adc_enable = Some(enabled);
        self
    }

    /// Battery current sensing ADC on/off.
    pub fn batt_current_adc(&mut self, enabled: bool) -> &mut Self {
        self.batt_current_adc_enable = Some(enabled);
        self
    }

    /// AC voltage ADC on/off.
    pub fn acin_voltage_adc(&mut self, enabled: bool) -> &mut Self {
        self.acin_voltage_adc_enable = Some(enabled);
        self
    }

    /// AC current sensing ADC on/off.
    pub fn acin_current_adc(&mut self, enabled: bool) -> &mut Self {
        self.acin_current_adc_enable = Some(enabled);
        self
    }

    /// VBUS voltage ADC on/off.
    pub fn vbus_voltage_adc(&mut self, enabled: bool) -> &mut Self {
        self.vbus_voltage_adc_enable = Some(enabled);
        self
    }

    /// VBUS current sensing ADC on/off.
    pub fn vbus_current_adc(&mut self, enabled: bool) -> &mut Self {
        self.vbus_current_adc_enable = Some(enabled);
        self
    }

    /// APS ADC on/off.
    pub fn aps_voltage_adc(&mut self, enabled: bool) -> &mut Self {
        self.aps_voltage_adc_enable = Some(enabled);
        self
    }

    /// Battery temperature sensor (thermistor on TS pin) ADC on/off.
    pub fn ts_adc(&mut self, enabled: bool) -> &mut Self {
        self.ts_adc_enable = Some(enabled);
        self
    }

    /// Sets ADC sampling rate.
    pub fn set_adc_sample_rate(&mut self, sample_rate: AdcSampleRate) -> &mut Self {
        self.sample_rate = Some(sample_rate);
        self
    }

    pub fn use_ts_for_batt_temperature(&mut self, enabled: bool) -> &mut Self {
        self.use_ts_for_battery_temperature = Some(enabled);
        self
    }

    pub fn set_ts_pin_mode(&mut self, mode: TsPinMode) -> &mut Self {
        self.ts_pin_mode = Some(mode);
        self
    }

    /// Sets or remove corresponding bits in `previous_value`.
    fn write_adc_en_bits(&self, previous_value: &mut u8) {
        if let Some(enabled) = self.batt_voltage_adc_enable {
            previous_value.set_bit(ADC_ENABLE1_BATT_VOLTAGE_ADC_ENABLE, enabled);
        }
        if let Some(enabled) = self.batt_current_adc_enable {
            previous_value.set_bit(ADC_ENABLE1_BATT_CURRENT_ADC_ENABLE, enabled);
        }

        if let Some(enabled) = self.acin_voltage_adc_enable {
            previous_value.set_bit(ADC_ENABLE1_ACIN_VOLTAGE_ADC_ENABLE, enabled);
        }
        if let Some(enabled) = self.acin_current_adc_enable {
            previous_value.set_bit(ADC_ENABLE1_ACIN_CURRENT_ADC_ENABLE, enabled);
        }

        if let Some(enabled) = self.vbus_voltage_adc_enable {
            previous_value.set_bit(ADC_ENABLE1_VBUS_VOLTAGE_ADC_ENABLE, enabled);
        }
        if let Some(enabled) = self.vbus_current_adc_enable {
            previous_value.set_bit(ADC_ENABLE1_VBUS_CURRENT_ADC_ENABLE, enabled);
        }

        if let Some(enabled) = self.aps_voltage_adc_enable {
            previous_value.set_bit(ADC_ENABLE1_APS_VOLTAGE_ADC_ENABLE, enabled);
        }
        if let Some(enabled) = self.ts_adc_enable {
            previous_value.set_bit(ADC_ENABLE1_TS_ADC_ENABLE, enabled);
        }
    }

    fn write_adc_sample_rate_and_ts_bits(&self, bits: &mut u8) {
        if let Some(rate) = self.sample_rate {
            bits.set_bits(ADC_SAMPLE_RATE_BITS, rate.bits());
        }

        if let Some(enable) = self.use_ts_for_battery_temperature {
            bits.set_bit(TS_BATT_TEMPERATURE_FUNCTION, enable);
        }

        if let Some(ts_mode) = self.ts_pin_mode {
            bits.set_bits(TS_MODE_BITS, ts_mode.bits());
        }
    }
}

// --- Register bitflags ---

bitflags! {
    /// Possible values of charger's regulation voltage in volts.
    pub struct ChargingVoltage: u8 {
        const V4_1  = 0b00;
        const V4_15 = 0b01;
        const V4_2  = 0b10; // default
        const V4_36 = 0b11;
    }
}

bitflags! {
    /// Possible values of charging current of AXP173 in milliAmperes
    pub struct ChargingCurrent: u8 {
        const CURRENT_100MA = 0b0000;
        const CURRENT_190MA = 0b0001;
        const CURRENT_280MA = 0b0010;
        const CURRENT_360MA = 0b0011;
        const CURRENT_450MA = 0b0100;
        const CURRENT_550MA = 0b0101;
        const CURRENT_630MA = 0b0110;
        const CURRENT_700MA = 0b0111;
        const CURRENT_780MA = 0b1000; // default
        const CURRENT_880MA = 0b1001;
        const CURRENT_960MA = 0b1010;
        const CURRENT_1000MA = 0b1011;
        const CURRENT_1080MA = 0b1100;
        const CURRENT_1160MA = 0b1101;
        const CURRENT_1240MA = 0b1110;
        const CURRENT_1320MA = 0b1111;
    }
}

bitflags! {
    /// Possible values of ADC sample rate in Hertz.
    pub struct AdcSampleRate: u8 {
        const RATE_20HZ = 0b00; // default
        const RATE_50HZ = 0b01;
        const RATE_100HZ = 0b10;
        const RATE_200HZ = 0b11;
    }
}

bitflags! {
    /// Possible thermistor pin function modes.
    pub struct TsPinMode: u8 {
        const SHUT_DOWN = 0b00;
        const ON_DURING_CHARGING = 0b01;
        const ON_DURING_ADC_SAMPLING = 0b10; // default
        const ALWAYS_ON = 0b11;
    }
}

// --- Regs definition ---

pub const AXP173_ADDR: u8 = 0x68; // Read = 0x69, write: 0x68. 7-bit: 0x34

pub const POWER_STATUS: u8 = 0x00;
pub const POWER_MODE_CHGSTATUS: u8 = 0x01;
pub const POWER_OTG_STATUS: u8 = 0x02;

pub const POWER_DATA_BUFFER1: u8 = 0x06;
pub const POWER_DATA_BUFFER2: u8 = 0x07;
pub const POWER_DATA_BUFFER3: u8 = 0x08;
pub const POWER_DATA_BUFFER4: u8 = 0x09;

pub const POWER_VERSION: u8 = 0x0C;

pub const POWER_LDO3_DC2_CTL: u8 = 0x10;
pub const POWER_LDO24_DC13_CTL: u8 = 0x12;
pub const POWER_DC2OUT_VOL: u8 = 0x23;
pub const POWER_LDO3_DC2_VRC: u8 = 0x25;
pub const POWER_DC1OUT_VOL: u8 = 0x26;
pub const POWER_DC3OUT_VOL: u8 = 0x27;
pub const LDO2_LDO3_OUT_VOL_REG: u8 = 0x28;
pub const LDO4_OUT_VOL_REG: u8 = 0x29;
pub const POWER_IPS_SET: u8 = 0x30;
pub const POWER_VOFF_SET: u8 = 0x31;
pub const POWER_OFF_CTL: u8 = 0x32;
pub const POWER_CHARGE1: u8 = 0x33;
pub const POWER_CHARGE2: u8 = 0x34;
pub const POWER_BACKUP_CHG: u8 = 0x35;
pub const POWER_PEK_SET: u8 = 0x36;
pub const POWER_DCDC_FREQSET: u8 = 0x37;
pub const POWER_VLTF_CHGSET: u8 = 0x38;
pub const POWER_VHTF_CHGSET: u8 = 0x39;
pub const POWER_APS_WARNING1: u8 = 0x3A;
pub const POWER_APS_WARNING2: u8 = 0x3B;
pub const POWER_VLTF_DISCHGSET: u8 = 0x3C;
pub const POWER_VHTF_DISCHGSET: u8 = 0x3D;
pub const POWER_DCDC_MODESET: u8 = 0x80;
pub const POWER_VOUT_MONITOR: u8 = 0x81;
pub const POWER_ADC_EN1: u8 = 0x82;
pub const POWER_ADC_EN2: u8 = 0x83;
pub const POWER_ADC_SPEED_TS: u8 = 0x84;
pub const POWER_ADC_INPUTRANGE: u8 = 0x85;
pub const POWER_TIMER_CTL: u8 = 0x8A;
pub const POWER_VBUS_DET_SRP: u8 = 0x8B;
pub const POWER_HOTOVER_CTL: u8 = 0x8F;
pub const POWER_GPIO0_CTL: u8 = 0x90;
pub const POWER_GPIO0_VOL: u8 = 0x91;
pub const POWER_GPIO1_CTL: u8 = 0x92;
pub const POWER_GPIO2_CTL: u8 = 0x93;
pub const POWER_GPIO_SIGNAL: u8 = 0x94;
pub const POWER_SENSE_CTL: u8 = 0x95;
pub const POWER_SENSE_SIGNAL: u8 = 0x96;
pub const POWER_GPIO20_PDCTL: u8 = 0x97;
pub const POWER_PWM1_FREQ: u8 = 0x98;
pub const POWER_PWM1_DUTYDE: u8 = 0x99;
pub const POWER_PWM1_DUTY: u8 = 0x9A;
pub const POWER_PWM2_FREQ: u8 = 0x9B;
pub const POWER_PWM2_DUTYDE: u8 = 0x9C;
pub const POWER_PWM2_DUTY: u8 = 0x9D;
pub const POWER_RSTO_CTL: u8 = 0x9E;
pub const POWER_GPIO67_CTL: u8 = 0x9F;
pub const POWER_INTEN1: u8 = 0x40;
pub const POWER_INTEN2: u8 = 0x41;
pub const POWER_INTEN3: u8 = 0x42;
pub const POWER_INTEN4: u8 = 0x43;
pub const POWER_INTEN5: u8 = 0x4a;
pub const POWER_INTSTS1: u8 = 0x44;
pub const POWER_INTSTS2: u8 = 0x45;
pub const POWER_INTSTS3: u8 = 0x46;
pub const POWER_INTSTS4: u8 = 0x47;
pub const POWER_INTSTS5: u8 = 0x4d;
pub const POWER_COULOMB_CTL: u8 = 0xB8;

pub const POWER_ACIN_VOL_H8: u8 = 0x56;
pub const POWER_ACIN_VOL_L4: u8 = 0x57;
pub const POWER_ACIN_CUR_H8: u8 = 0x58;
pub const POWER_ACIN_CUR_L4: u8 = 0x59;

pub const POWER_VBUS_VOL_H8: u8 = 0x5A;
pub const POWER_VBUS_VOL_L4: u8 = 0x5B;
pub const POWER_VBUS_CUR_H8: u8 = 0x5C;
pub const POWER_VBUS_CUR_L4: u8 = 0x5D;

pub const POWER_INT_TEMP_H8: u8 = 0x5E;
pub const POWER_INT_TEMP_L4: u8 = 0x5F;

pub const POWER_TS_VOL_H8: u8 = 0x62;
pub const POWER_TS_VOL_L4: u8 = 0x63;

pub const POWER_GPIO0_VOL_H8: u8 = 0x64;
pub const POWER_GPIO0_VOL_L4: u8 = 0x65;
pub const POWER_GPIO1_VOL_H8: u8 = 0x66;
pub const POWER_GPIO1_VOL_L4: u8 = 0x67;
pub const POWER_GPIO2_VOL_H8: u8 = 0x68;
pub const POWER_GPIO2_VOL_L4: u8 = 0x69;

pub const POWER_BATSENSE_VOL_H8: u8 = 0x6A;
pub const POWER_BATSENSE_VOL_L4: u8 = 0x6B;

pub const POWER_BAT_AVERVOL_H8: u8 = 0x78;
pub const POWER_BAT_AVERVOL_L4: u8 = 0x79;

pub const POWER_BAT_AVERCHGCUR_H8: u8 = 0x7A;
pub const POWER_BAT_AVERCHGCUR_L5: u8 = 0x7B;

pub const POWER_BAT_AVERDISCHGCUR_H8: u8 = 0x7C;
pub const POWER_BAT_AVERDISCHGCUR_L5: u8 = 0x7D;

pub const POWER_APS_AVERVOL_H8: u8 = 0x7E;
pub const POWER_APS_AVERVOL_L4: u8 = 0x7F;

pub const POWER_BAT_CHGCOULOMB3: u8 = 0xB0;
pub const POWER_BAT_CHGCOULOMB2: u8 = 0xB1;
pub const POWER_BAT_CHGCOULOMB1: u8 = 0xB2;
pub const POWER_BAT_CHGCOULOMB0: u8 = 0xB3;

pub const POWER_BAT_DISCHGCOULOMB3: u8 = 0xB4;
pub const POWER_BAT_DISCHGCOULOMB2: u8 = 0xB5;
pub const POWER_BAT_DISCHGCOULOMB1: u8 = 0xB6;
pub const POWER_BAT_DISCHGCOULOMB0: u8 = 0xB7;

pub const POWER_BAT_POWERH8: u8 = 0x70;
pub const POWER_BAT_POWERM8: u8 = 0x71;
pub const POWER_BAT_POWERL8: u8 = 0x72;

pub const POWER_ON_OFF_REG: u8 = 0x12;
pub const EXTEN: u8 = 1 << 6;

pub const DC1_ON: u8 = 1 << 0;
pub const DC2_ON: u8 = 1 << 4;

pub const DC2_REG: u8 = 0x23;
pub const DC1_REG: u8 = 0x26;
pub const LDO4_REG: u8 = 0x27;
pub const LDO2_REG: u8 = 0x28;
pub const LDO3_REG: u8 = 0x28;
pub const VBUS_IPSOUT_REG: u8 = 0x30;
pub const VOFF_REG: u8 = 0x31;
pub const POWER_BATDETECT_CHGLED_REG: u8 = 0x32;

pub const VBUS_VHOLD_EN: u8 = 1 << 6;
pub const VBUS_CL_EN: u8 = 1 << 1;
pub const BATTERY_DETE: u8 = 1 << 6;
pub const BAT_ADC_EN_V: u8 = 1 << 7;
pub const BAT_ADC_EN_C: u8 = 1 << 6;
pub const AC_ADC_EN_V: u8 = 1 << 5;
pub const AC_ADC_EN_C: u8 = 1 << 4;
pub const USB_ADC_EN_V: u8 = 1 << 3;
pub const USB_ADC_EN_C: u8 = 1 << 2;
pub const APS_ADC_EN_V: u8 = 1 << 1;
pub const ADC_SPEED_BIT1: u8 = 1 << 7;
pub const ADC_SPEED_BIT2: u8 = 1 << 6;
pub const INT1_AC_IN: u8 = 1 << 6;
pub const INT1_AC_OUT: u8 = 1 << 5;
pub const INT1_USB_IN: u8 = 1 << 3;
pub const INT1_USB_OUT: u8 = 1 << 2;
pub const INT1_VBUS_VHOLD: u8 = 1 << 1;
pub const INT2_BATTERY_IN: u8 = 1 << 7;
pub const INT2_BATTERY_OUT: u8 = 1 << 6;
pub const INT2_BATTERY_CHARGING: u8 = 1 << 3;
pub const INT2_BATTERY_CHARGED: u8 = 1 << 2;
pub const INT2_BATTERY_HIGH_TEMP: u8 = 1 << 1;
pub const INT2_BATTERY_LOW_TEMP: u8 = 1 << 0;
pub const INT3_SHORTPRESS_PEK: u8 = 1 << 1;
pub const INT3_LONGPRESS_PEK: u8 = 1 << 0;
pub const INT3_LOWVOLTAGE_WARNING: u8 = 1 << 4;
pub const INT4_LOWVOLTAGE_WARNING1: u8 = 1 << 1;
pub const INT4_LOWVOLTAGE_WARNING2: u8 = 1 << 0;
pub const INT5_TIMER: u8 = 1 << 7;

// POWER_STATUS register bits
pub const POWER_STATUS_START_SOURCE_AC_VBUS: usize = 0;
pub const POWER_STATUS_BATT_CHARGE_DISCHARGE: usize = 2;
pub const POWER_STATUS_VBUS_PRESENT: usize = 5;

// POWER_MODE_CHGSTATUS register bits
pub const POWER_MODE_CHGSTATUS_IS_CHARGING: usize = 6;
pub const POWER_MODE_CHGSTATUS_BATTERY_PRESENT: usize = 5;

// POWER_ON_OFF_REG register bits
pub const POWER_ON_OFF_REG_LDO2_ON: usize = 2;
pub const POWER_ON_OFF_REG_LDO3_ON: usize = 3;
pub const POWER_ON_OFF_REG_LDO4_ON: usize = 1;
pub const POWER_ON_OFF_REG_POWER_OFF: usize = 7;

// POWER_CHARGE1 register bits
pub const POWER_CHARGE1_CURRENT_SETTING: Range<usize> = 0..4;
pub const POWER_CHARGE1_VOLTAGE_SETTING: Range<usize> = 5..6;
pub const POWER_CHARGE1_ENABLE_CHARGING: usize = 7; // enabled by default

// ADC_ENABLE1 register bits
pub const ADC_ENABLE1_BATT_VOLTAGE_ADC_ENABLE: usize = 7; // enabled by default
pub const ADC_ENABLE1_BATT_CURRENT_ADC_ENABLE: usize = 6;
pub const ADC_ENABLE1_ACIN_VOLTAGE_ADC_ENABLE: usize = 5;
pub const ADC_ENABLE1_ACIN_CURRENT_ADC_ENABLE: usize = 4;
pub const ADC_ENABLE1_VBUS_VOLTAGE_ADC_ENABLE: usize = 3;
pub const ADC_ENABLE1_VBUS_CURRENT_ADC_ENABLE: usize = 2;
pub const ADC_ENABLE1_APS_VOLTAGE_ADC_ENABLE: usize = 1; // enabled by default
pub const ADC_ENABLE1_TS_ADC_ENABLE: usize = 0; // enabled by default

// POWER_ADC_SPEED bits
pub const ADC_SAMPLE_RATE_BITS: Range<usize> = 6..8;
pub const TS_BATT_TEMPERATURE_FUNCTION: usize = 2;
pub const TS_MODE_BITS: Range<usize> = 0..2;
