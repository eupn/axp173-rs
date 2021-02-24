//! Interrupts (IRQs).

use async_embedded_traits::i2c::{AsyncI2cTransfer, AsyncI2cWrite, I2cAddress7Bit};
use bit_field::BitField;

use crate::{Axp173, Axp173Result, Error, OperationResult};

/// An AXP173 interrupt.
#[derive(Debug, Copy, Clone)]
#[allow(missing_docs)] // TODO: document IRQs
pub enum Irq {
    AcinOvervoltage,
    AcinPluggedIn,
    AcinUnplugged,

    VbusOvervoltage,
    VbusPluggedIn,
    VbusUnplugged,
    VbusUndervoltage,

    BatteryPlugged,
    BatteryUnplugged,
    EnteredBattRecoveryMode,
    ExitedBattRecoveryMode,
    BatteryCharging,
    BatteryCharged,
    BatteryOverheat,
    BatteryTooCold,

    ChipOverheat,

    InsufficientChargeCurrent,

    Dcdc1Undervoltage,
    Dcdc2Undervoltage,

    Ldo4Undervoltage,

    // Irq21, // Reserved
    ButtonShortPress,
    ButtonLongPress,

    // Irq24, // Reserved
    // Irq25, // Reserved
    VbusEffective,
    VbusInvalid,
    VbusSessionValid,
    VbusSessionInvalid,

    LowBatteryWarning,
}

impl Irq {
    #[rustfmt::skip] // To preserve table formatting
    pub(crate) fn to_reg_and_bit(&self) -> (u8, usize) {
        match self {
            Irq::AcinOvervoltage            => (0x44, 7),
            Irq::AcinPluggedIn              => (0x44, 6),
            Irq::AcinUnplugged              => (0x44, 5),
            Irq::VbusOvervoltage            => (0x44, 4),
            Irq::VbusPluggedIn              => (0x44, 3),
            Irq::VbusUnplugged              => (0x44, 2),
            Irq::VbusUndervoltage           => (0x44, 1),

            Irq::BatteryPlugged             => (0x45, 7),
            Irq::BatteryUnplugged           => (0x45, 6),
            Irq::EnteredBattRecoveryMode    => (0x45, 5),
            Irq::ExitedBattRecoveryMode     => (0x45, 4),
            Irq::BatteryCharging            => (0x45, 3),
            Irq::BatteryCharged             => (0x45, 2),
            Irq::BatteryOverheat            => (0x45, 1),
            Irq::BatteryTooCold             => (0x45, 0),

            Irq::ChipOverheat               => (0x46, 7),
            Irq::InsufficientChargeCurrent  => (0x46, 6),
            Irq::Dcdc1Undervoltage          => (0x46, 5),
            Irq::Dcdc2Undervoltage          => (0x46, 4),
            Irq::Ldo4Undervoltage           => (0x46, 3),
            // Irq21 is reserved
            Irq::ButtonShortPress           => (0x46, 1),
            Irq::ButtonLongPress            => (0x46, 0),
            // Irq24 is reserved
            // Irq25 is reserved

            Irq::VbusEffective              => (0x47, 5),
            Irq::VbusInvalid                => (0x47, 4),
            Irq::VbusSessionValid           => (0x47, 3),
            Irq::VbusSessionInvalid         => (0x47, 2),
            Irq::LowBatteryWarning          => (0x47, 0),
        }
    }
}

impl<I, E> Axp173<I>
where
    I: AsyncI2cTransfer<I2cAddress7Bit, Error = E> + AsyncI2cWrite<I2cAddress7Bit, Error = E>,
{
    /// Enables or disables (masks) selected IRQ.
    pub async fn set_irq(&mut self, irq: Irq, enabled: bool) -> OperationResult<E> {
        let (status_reg, bit) = irq.to_reg_and_bit();
        let mask_reg = status_reg - 4; // Convert status register to mask register

        let mut bits = self.read_u8(mask_reg).await.map_err(Error::I2c)?;
        bits.set_bit(bit, enabled);
        self.write_u8(mask_reg, bits).await.map_err(Error::I2c)?;

        Ok(())
    }

    /// Clears previously fired selected IRQ.
    pub async fn clear_irq(&mut self, irq: Irq) -> OperationResult<E> {
        let (status_reg, bit) = irq.to_reg_and_bit();

        let mut bits = self.read_u8(status_reg).await.map_err(Error::I2c)?;
        bits.set_bit(bit, true); // Clear the IRQ by writing '1' bit
        self.write_u8(status_reg, bits).await.map_err(Error::I2c)?;

        Ok(())
    }

    /// Clears ALL pending IRQs.
    pub async fn clear_all_irq(&mut self) -> OperationResult<E> {
        self.write_u8(0x44, 0xff).await.map_err(Error::I2c)?;
        self.write_u8(0x45, 0xff).await.map_err(Error::I2c)?;
        self.write_u8(0x46, 0xff).await.map_err(Error::I2c)?;
        self.write_u8(0x47, 0xff).await.map_err(Error::I2c)?;

        Ok(())
    }

    /// Checks whether selected IRQ has fired or not.
    /// Note: one should clear the IRQ after checking or it will fire indefinitely
    pub async fn check_irq(&mut self, irq: Irq) -> Axp173Result<bool, E> {
        let (status_reg, bit) = irq.to_reg_and_bit();
        let reg_val = self.read_u8(status_reg).await.map_err(Error::I2c)?;
        Ok(reg_val.get_bit(bit))
    }
}
