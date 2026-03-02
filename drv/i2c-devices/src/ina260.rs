// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Driver for the TI INA260 current/voltage/power monitor
//!
//! The INA260 has a built-in 2 mΩ shunt resistor and digitizes current,
//! bus voltage, and power internally.

use crate::{CurrentSensor, PowerSensor, Validate, VoltageSensor};
use drv_i2c_api::*;
use userlib::units::*;

#[allow(dead_code)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Register {
    Configuration = 0x00,
    Current = 0x01,
    BusVoltage = 0x02,
    Power = 0x03,
    MaskEnable = 0x06,
    AlertLimit = 0x07,
    ManufacturerId = 0xFE,
    DieId = 0xFF,
}

#[derive(Debug)]
pub enum Error {
    BadRegisterRead { reg: Register, code: ResponseCode },
}

impl From<Error> for ResponseCode {
    fn from(err: Error) -> Self {
        match err {
            Error::BadRegisterRead { code, .. } => code,
        }
    }
}

pub struct Ina260 {
    device: I2cDevice,
}

impl core::fmt::Display for Ina260 {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "ina260: {}", &self.device)
    }
}

impl Ina260 {
    pub fn new(device: &I2cDevice) -> Self {
        Self { device: *device }
    }

    fn read_reg(&self, reg: Register) -> Result<[u8; 2], Error> {
        self.device
            .read_reg::<u8, [u8; 2]>(reg as u8)
            .map_err(|code| Error::BadRegisterRead { reg, code })
    }
}

impl Validate<Error> for Ina260 {
    fn validate(device: &I2cDevice) -> Result<bool, Error> {
        let ina = Ina260::new(device);
        let mfr = ina.read_reg(Register::ManufacturerId)?;
        let die = ina.read_reg(Register::DieId)?;
        // Manufacturer ID = 0x5449 ("TI"), Die ID = 0x2270
        Ok(u16::from_be_bytes(mfr) == 0x5449
            && u16::from_be_bytes(die) == 0x2270)
    }
}

impl CurrentSensor<Error> for Ina260 {
    fn read_iout(&self) -> Result<Amperes, Error> {
        let raw = self.read_reg(Register::Current)?;
        // Signed 16-bit, LSB = 1.25 mA
        let signed = i16::from_be_bytes(raw);
        Ok(Amperes(f32::from(signed) * 1.25e-3))
    }
}

impl VoltageSensor<Error> for Ina260 {
    fn read_vout(&self) -> Result<Volts, Error> {
        let raw = self.read_reg(Register::BusVoltage)?;
        // Unsigned 16-bit, LSB = 1.25 mV
        let unsigned = u16::from_be_bytes(raw);
        Ok(Volts(f32::from(unsigned) * 1.25e-3))
    }
}

impl PowerSensor<Error> for Ina260 {
    fn read_power(&mut self) -> Result<Watts, Error> {
        let raw = self.read_reg(Register::Power)?;
        // Unsigned 16-bit, LSB = 10 mW
        let unsigned = u16::from_be_bytes(raw);
        Ok(Watts(f32::from(unsigned) * 10.0e-3))
    }
}
