use bit_field::BitField;

use crate::regs::*;

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

    /// Use TS pin for thermistor-based battery temperature measurement.
    pub fn use_ts_for_batt_temperature(&mut self, enabled: bool) -> &mut Self {
        self.use_ts_for_battery_temperature = Some(enabled);
        self
    }

    /// Sets TS pin mode.
    pub fn set_ts_pin_mode(&mut self, mode: TsPinMode) -> &mut Self {
        self.ts_pin_mode = Some(mode);
        self
    }

    /// Sets or remove corresponding bits in `previous_value`.
    pub(crate) fn write_adc_en_bits(&self, previous_value: &mut u8) {
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

    pub(crate) fn write_adc_sample_rate_and_ts_bits(&self, bits: &mut u8) {
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
