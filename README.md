## `axp173`

<!-- [![](https://img.shields.io/travis/eupn/axp173.svg?style=flat)](https://travis-ci.org/eupn/axp173) -->
[![](https://img.shields.io/crates/v/axp173.svg?style=flat)](https://crates.io/crates/axp173)
[![](https://img.shields.io/crates/d/axp173.svg?maxAge=3600)](https://crates.io/crates/axp173)

<img src="doc/axp173.jpg" width="200" height="200"><img src="doc/axp173_block_diagram.png" width="400" height="800">

## What is this?

This is a [embedded-hal](https://github.com/rust-embedded/embedded-hal) driver 
for X-Powers' Power Management IC [AXP173](http://www.x-powers.com/en.php/Info/product_detail/article_id/27).

It's device-agnostic and uses embedded-hal's `Write`/`WriteRead` for I2C communication.

## Usage

1. Add dependency to `Cargo.toml`:

    ```bash
    cargo add axp173
    ```
    
2. Instantiate and init the device:

    ```rust
    // ... declare and configure your I2c peripheral ...
    
    // Init BNO055 IMU
    let axp173 = axp173::Axp173::new(i2c);
    
    axp173.init()?;
    
    Ok(axp173)
    ```

3. TODO

## Details and examples

TODO

## Status

What is done and tested and what is not yet:

- [ ] Coulomb counter reading
- [ ] Coulomb counter control
- [ ] DC/DC settings
- [ ] IRQs
- [ ] ADC readings
- [x] AXP173 on-chip buffer check
- [x] AXP173 LDO2, LDO3, LDO4 enable/disable
- [x] LDO voltage setup
- [x] VBUS presence
- [x] Battery presence
- [x] Battery charging status
- [x] Charging current setup
- [x] Charging regulated voltage setup
- [x] Internal ADC settings:
  - [x] Sample rate
  - [x] Enable/Disable various ADC channels (batt. voltage, current, etc.)
- [ ] Button settings
