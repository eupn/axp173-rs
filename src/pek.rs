///! Power Enable Key definitions.
use bitflags::bitflags;

bitflags! {
    /// Delay before power-on from shutdown settings.
    pub struct BootTime : u8 {
        /// 128 ms.
        const MS_128 = 0b00;

        /// 512 ms.
        const MS_512 = 0b01; // default

        /// 1 second.
        const SEC_1 = 0b10;

        /// 2 seconds.
        const SEC_2 = 0b11;
    }
}

bitflags! {
    /// Button press time to be considered as "long" press.
    pub struct LongPressTime : u8 {
        /// 1 second.
        const SEC_1 = 0b00;

        /// 1.5 seconds.
        const SEC_1_5 = 0b01; // default

        /// 2 seconds.
        const SEC_2 = 0b10;

        /// 2.5 seconds.
        const SEC_2_5 = 0b11;
    }
}

bitflags! {
    /// Button press time in seconds to initiate shutdown or power-on.
    pub struct ShutdownLongPressTime : u8 {
        /// 4 seconds.
        const SEC_4 = 0b00;

        /// 6 seconds.
        const SEC_6 = 0b01; // default

        /// 8 seconds.
        const SEC_8 = 0b10;

        /// 10 seconds.
        const SEC_10 = 0b11;
    }
}
