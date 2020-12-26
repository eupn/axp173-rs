#![allow(dead_code)]

use core::ops::Range;

use bitflags::bitflags;

// --- Register bitflags ---

bitflags! {
    /// Possible values of charger's regulation voltage in volts.
    pub struct ChargingVoltage: u8 {
        /// 4.1 Volts
        const V4_1  = 0b00;

        /// 4.15 Volts
        const V4_15 = 0b01;

        /// 4.2 Volts (default)
        const V4_2  = 0b10; // default

        /// 4.36 Volts
        const V4_36 = 0b11;
    }
}

bitflags! {
    /// Possible values of charging current of AXP173 in milliAmperes
    pub struct ChargingCurrent: u8 {
        /// 100 mA current.
        const CURRENT_100MA = 0b0000;

        /// 190 mA current.
        const CURRENT_190MA = 0b0001;

        /// 280 mA current.
        const CURRENT_280MA = 0b0010;

        /// 360 mA current.
        const CURRENT_360MA = 0b0011;

        /// 450 mA current.
        const CURRENT_450MA = 0b0100;

        /// 550 mA current.
        const CURRENT_550MA = 0b0101;

        /// 630 mA current.
        const CURRENT_630MA = 0b0110;

        /// 700 mA current.
        const CURRENT_700MA = 0b0111;

        /// 780 mA current. Default.
        const CURRENT_780MA = 0b1000;

        /// 880 mA current.
        const CURRENT_880MA = 0b1001;

        /// 960 mA current.
        const CURRENT_960MA = 0b1010;

        /// 1000 mA current.
        const CURRENT_1000MA = 0b1011;

        /// 1080 mA current.
        const CURRENT_1080MA = 0b1100;

        /// 1160 mA current.
        const CURRENT_1160MA = 0b1101;

        /// 1240 mA current.
        const CURRENT_1240MA = 0b1110;

        /// 1320 mA current.
        const CURRENT_1320MA = 0b1111;
    }
}

bitflags! {
    /// Possible values of ADC sample rate in Hertz.
    pub struct AdcSampleRate: u8 {
        /// 25 Hz sample rate. Default.
        const RATE_25HZ = 0b00;

        /// 50 Hz sample rate.
        const RATE_50HZ = 0b01;

        /// 100 Hz sample rate.
        const RATE_100HZ = 0b10;

        /// 200 Hz sample rate.
        const RATE_200HZ = 0b11;
    }
}

bitflags! {
    /// Possible thermistor pin function modes.
    pub struct TsPinMode: u8 {
        /// TS pin is shut down.
        const SHUT_DOWN = 0b00;

        /// TS is used during charging.
        const ON_DURING_CHARGING = 0b01;

        /// TS is used only during ADC sampling. Default.
        const ON_DURING_ADC_SAMPLING = 0b10;

        /// TS is always used.
        const ALWAYS_ON = 0b11;
    }
}

// --- Regs definition ---

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

pub const DC1_ON: u8 = 1;
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
pub const INT2_BATTERY_LOW_TEMP: u8 = 1;
pub const INT3_SHORTPRESS_PEK: u8 = 1 << 1;
pub const INT3_LONGPRESS_PEK: u8 = 1;
pub const INT3_LOWVOLTAGE_WARNING: u8 = 1 << 4;
pub const INT4_LOWVOLTAGE_WARNING1: u8 = 1 << 1;
pub const INT4_LOWVOLTAGE_WARNING2: u8 = 1;
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
pub const POWER_ON_OFF_REG_DCDC2_ON: usize = 4;
pub const POWER_ON_OFF_REG_EXTEN: usize = 6;
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

// POWER_COULOMB_CTL bits
pub const COULOMB_ENABLE: usize = 7;
pub const COULOMB_PAUSE: usize = 6;
pub const COULOMB_RESET: usize = 5;

// POWER_PEK_SET bits
pub const POWER_PEK_BOOT_TIME_BITS: Range<usize> = 6..8;
pub const POWER_PEK_LONG_PRESS_TIME_BITS: Range<usize> = 4..6;
pub const POWER_PEK_LONG_PRESS_SHUTDOWN_BIT: usize = 3;
pub const POWER_PEK_SHUTDOWN_LONG_PRESS_TIME_BITS: Range<usize> = 0..2;
