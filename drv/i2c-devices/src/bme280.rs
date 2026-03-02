// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Driver for the Bosch BME280/BMP280 environmental sensor
//!
//! Supports both BME280 (temperature, pressure, humidity) and BMP280
//! (temperature, pressure only). Uses forced mode with 1x oversampling.
//! Calibration data is cached after first read.
//!
//! Compensation formulas are from the Bosch BME280 datasheet (BST-BME280-
//! DS002), Section 4.2.3, using 32-bit integer arithmetic (with 64-bit
//! intermediates for pressure).

use core::cell::Cell;

use crate::{HumiditySensor, PressureSensor, TempSensor, Validate};
use drv_i2c_api::*;
use userlib::units::*;

#[allow(dead_code)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Register {
    CalibT1 = 0x88,   // 26 bytes: T1..T3, P1..P9 (0x88..0xA1)
    CalibH1 = 0xA1,   // 1 byte
    ChipId = 0xD0,
    Reset = 0xE0,
    CalibH2 = 0xE1,   // 7 bytes: H2..H6 (0xE1..0xE7)
    CtrlHum = 0xF2,
    Status = 0xF3,
    CtrlMeas = 0xF4,
    Config = 0xF5,
    DataStart = 0xF7,  // 8 bytes: press[2:0], temp[2:0], hum[1:0]
}

/// BME280 chip ID
const CHIP_ID_BME280: u8 = 0x60;
/// BMP280 chip ID (no humidity sensor)
const CHIP_ID_BMP280: u8 = 0x58;

/// Forced mode: measure once then return to sleep
/// Oversampling: 1x for all channels
const CTRL_HUM: u8 = 0b001;           // osrs_h = 1x
const CTRL_MEAS: u8 = 0b001_001_01;   // osrs_t=1x, osrs_p=1x, mode=forced

#[derive(Copy, Clone, Debug)]
struct CalibrationData {
    // Temperature
    dig_t1: u16,
    dig_t2: i16,
    dig_t3: i16,
    // Pressure
    dig_p1: u16,
    dig_p2: i16,
    dig_p3: i16,
    dig_p4: i16,
    dig_p5: i16,
    dig_p6: i16,
    dig_p7: i16,
    dig_p8: i16,
    dig_p9: i16,
    // Humidity (BME280 only; zeroed for BMP280)
    dig_h1: u8,
    dig_h2: i16,
    dig_h3: u8,
    dig_h4: i16,
    dig_h5: i16,
    dig_h6: i8,
    // True for BME280, false for BMP280
    has_humidity: bool,
}

#[derive(Debug)]
pub enum Error {
    BadRegisterRead { reg: u8, code: ResponseCode },
    BadRegisterWrite { reg: u8, code: ResponseCode },
}

impl From<Error> for ResponseCode {
    fn from(err: Error) -> Self {
        match err {
            Error::BadRegisterRead { code, .. } => code,
            Error::BadRegisterWrite { code, .. } => code,
        }
    }
}

pub struct Bme280 {
    device: I2cDevice,
    calibration: Cell<Option<CalibrationData>>,
}

impl core::fmt::Display for Bme280 {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "bme280: {}", &self.device)
    }
}

impl Bme280 {
    pub fn new(device: &I2cDevice) -> Self {
        Self {
            device: *device,
            calibration: Cell::new(None),
        }
    }

    /// Load calibration data from the device, caching it for future use.
    fn load_calibration(&self) -> Result<CalibrationData, Error> {
        if let Some(cal) = self.calibration.get() {
            return Ok(cal);
        }

        // Detect chip variant
        let id: [u8; 1] = self
            .device
            .read_reg::<u8, [u8; 1]>(Register::ChipId as u8)
            .map_err(|code| Error::BadRegisterRead {
                reg: Register::ChipId as u8,
                code,
            })?;
        let has_humidity = id[0] == CHIP_ID_BME280;

        // Read temperature and pressure calibration (26 bytes from 0x88)
        let tp: [u8; 26] = self
            .device
            .read_reg::<u8, [u8; 26]>(Register::CalibT1 as u8)
            .map_err(|code| Error::BadRegisterRead {
                reg: Register::CalibT1 as u8,
                code,
            })?;

        // Read humidity calibration only for BME280
        let (h1, hx) = if has_humidity {
            let h1: [u8; 1] = self
                .device
                .read_reg::<u8, [u8; 1]>(Register::CalibH1 as u8)
                .map_err(|code| Error::BadRegisterRead {
                    reg: Register::CalibH1 as u8,
                    code,
                })?;
            let hx: [u8; 7] = self
                .device
                .read_reg::<u8, [u8; 7]>(Register::CalibH2 as u8)
                .map_err(|code| Error::BadRegisterRead {
                    reg: Register::CalibH2 as u8,
                    code,
                })?;
            (h1, hx)
        } else {
            ([0u8; 1], [0u8; 7])
        };

        let cal = CalibrationData {
            dig_t1: u16::from_le_bytes([tp[0], tp[1]]),
            dig_t2: i16::from_le_bytes([tp[2], tp[3]]),
            dig_t3: i16::from_le_bytes([tp[4], tp[5]]),

            dig_p1: u16::from_le_bytes([tp[6], tp[7]]),
            dig_p2: i16::from_le_bytes([tp[8], tp[9]]),
            dig_p3: i16::from_le_bytes([tp[10], tp[11]]),
            dig_p4: i16::from_le_bytes([tp[12], tp[13]]),
            dig_p5: i16::from_le_bytes([tp[14], tp[15]]),
            dig_p6: i16::from_le_bytes([tp[16], tp[17]]),
            dig_p7: i16::from_le_bytes([tp[18], tp[19]]),
            dig_p8: i16::from_le_bytes([tp[20], tp[21]]),
            dig_p9: i16::from_le_bytes([tp[22], tp[23]]),

            dig_h1: h1[0],
            dig_h2: i16::from_le_bytes([hx[0], hx[1]]),
            dig_h3: hx[2],
            dig_h4: (i16::from(hx[3]) << 4) | (i16::from(hx[4]) & 0x0F),
            dig_h5: (i16::from(hx[5]) << 4) | (i16::from(hx[4]) >> 4),
            dig_h6: hx[6] as i8,
            has_humidity,
        };

        self.calibration.set(Some(cal));
        Ok(cal)
    }

    /// Trigger a forced measurement and read all raw data.
    ///
    /// Returns compensated (temperature, pressure, humidity).
    /// For BMP280, humidity is always 0.0.
    pub fn read_all(
        &self,
    ) -> Result<(Celsius, Hectopascals, RelativeHumidity), Error> {
        let cal = self.load_calibration()?;

        // Set humidity oversampling (BME280 only; must be written before
        // ctrl_meas)
        if cal.has_humidity {
            self.device
                .write(&[Register::CtrlHum as u8, CTRL_HUM])
                .map_err(|code| Error::BadRegisterWrite {
                    reg: Register::CtrlHum as u8,
                    code,
                })?;
        }

        // Trigger forced measurement
        self.device
            .write(&[Register::CtrlMeas as u8, CTRL_MEAS])
            .map_err(|code| Error::BadRegisterWrite {
                reg: Register::CtrlMeas as u8,
                code,
            })?;

        // At 1x oversampling, typical measurement time is ~8ms.
        // We poll the status register rather than using a fixed delay.
        for _ in 0..20 {
            let status: [u8; 1] = self
                .device
                .read_reg::<u8, [u8; 1]>(Register::Status as u8)
                .map_err(|code| Error::BadRegisterRead {
                    reg: Register::Status as u8,
                    code,
                })?;
            if status[0] & 0x08 == 0 {
                break;
            }
            // Small delay between polls — the I2C transaction itself
            // provides enough delay in practice.
        }

        // BMP280: 6 bytes (press[2:0], temp[2:0])
        // BME280: 8 bytes (press[2:0], temp[2:0], hum[1:0])
        let data: [u8; 8] = if cal.has_humidity {
            self.device
                .read_reg::<u8, [u8; 8]>(Register::DataStart as u8)
                .map_err(|code| Error::BadRegisterRead {
                    reg: Register::DataStart as u8,
                    code,
                })?
        } else {
            let d: [u8; 6] = self
                .device
                .read_reg::<u8, [u8; 6]>(Register::DataStart as u8)
                .map_err(|code| Error::BadRegisterRead {
                    reg: Register::DataStart as u8,
                    code,
                })?;
            [d[0], d[1], d[2], d[3], d[4], d[5], 0, 0]
        };

        let adc_p =
            ((data[0] as i32) << 12) | ((data[1] as i32) << 4) | ((data[2] as i32) >> 4);
        let adc_t =
            ((data[3] as i32) << 12) | ((data[4] as i32) << 4) | ((data[5] as i32) >> 4);

        let (temp_cdeg, t_fine) = compensate_temperature(adc_t, &cal);
        let press_pa = compensate_pressure(adc_p, t_fine, &cal);

        let humidity = if cal.has_humidity {
            let adc_h = ((data[6] as i32) << 8) | (data[7] as i32);
            let hum_pct = compensate_humidity(adc_h, t_fine, &cal);
            RelativeHumidity(hum_pct as f32 / 1024.0)
        } else {
            RelativeHumidity(0.0)
        };

        // temp_cdeg is in centidegrees C (e.g. 2534 = 25.34 °C)
        let temperature = Celsius(temp_cdeg as f32 / 100.0);
        // press_pa is pressure in Pa (e.g. 101325)
        let pressure = Hectopascals(press_pa as f32 / 100.0);

        Ok((temperature, pressure, humidity))
    }
}

/// Compensate raw temperature ADC value.
/// Returns (temperature in centidegrees C, t_fine for pressure/humidity).
fn compensate_temperature(adc_t: i32, cal: &CalibrationData) -> (i32, i32) {
    let var1 = (((adc_t >> 3) - ((cal.dig_t1 as i32) << 1))
        * (cal.dig_t2 as i32))
        >> 11;
    let var2 = (((((adc_t >> 4) - (cal.dig_t1 as i32))
        * ((adc_t >> 4) - (cal.dig_t1 as i32)))
        >> 12)
        * (cal.dig_t3 as i32))
        >> 14;
    let t_fine = var1 + var2;
    let t = (t_fine * 5 + 128) >> 8;
    (t, t_fine)
}

/// Compensate raw pressure ADC value.
/// Returns pressure in Pa (unsigned 32-bit).
/// Uses 64-bit intermediates per the datasheet to avoid overflow.
fn compensate_pressure(adc_p: i32, t_fine: i32, cal: &CalibrationData) -> u32 {
    let var1 = i64::from(t_fine) - 128000;
    let var2 = var1 * var1 * i64::from(cal.dig_p6);
    let var2 = var2 + ((var1 * i64::from(cal.dig_p5)) << 17);
    let var2 = var2 + (i64::from(cal.dig_p4) << 35);
    let var1 = ((var1 * var1 * i64::from(cal.dig_p3)) >> 8)
        + ((var1 * i64::from(cal.dig_p2)) << 12);
    let var1 = ((((1i64) << 47) + var1) * i64::from(cal.dig_p1)) >> 33;

    if var1 == 0 {
        return 0; // avoid division by zero
    }

    let p = 1048576i64 - i64::from(adc_p);
    let p = (((p << 31) - var2) * 3125) / var1;
    let var1 = (i64::from(cal.dig_p9) * (p >> 13) * (p >> 13)) >> 25;
    let var2 = (i64::from(cal.dig_p8) * p) >> 19;
    let p = ((p + var1 + var2) >> 8) + (i64::from(cal.dig_p7) << 4);

    (p >> 8) as u32
}

/// Compensate raw humidity ADC value.
/// Returns humidity as Q22.10 fixed point (value * 1024).
fn compensate_humidity(adc_h: i32, t_fine: i32, cal: &CalibrationData) -> u32 {
    let v = t_fine - 76800i32;

    let v = ((((adc_h << 14)
        - ((cal.dig_h4 as i32) << 20)
        - ((cal.dig_h5 as i32) * v))
        + 16384)
        >> 15)
        * (((((((v * (cal.dig_h6 as i32)) >> 10)
            * (((v * (cal.dig_h3 as i32)) >> 11) + 32768))
            >> 10)
            + 2097152)
            * (cal.dig_h2 as i32)
            + 8192)
            >> 14);

    let v = v - (((((v >> 15) * (v >> 15)) >> 7) * (cal.dig_h1 as i32)) >> 4);

    // Clamp to 0..100% (in Q22.10)
    let v = if v < 0 { 0 } else { v };
    let v = if v > 419430400 { 419430400 } else { v };

    (v >> 12) as u32
}

impl Validate<Error> for Bme280 {
    fn validate(device: &I2cDevice) -> Result<bool, Error> {
        let id: [u8; 1] = device
            .read_reg::<u8, [u8; 1]>(Register::ChipId as u8)
            .map_err(|code| Error::BadRegisterRead {
                reg: Register::ChipId as u8,
                code,
            })?;
        Ok(id[0] == CHIP_ID_BME280 || id[0] == CHIP_ID_BMP280)
    }
}

impl TempSensor<Error> for Bme280 {
    fn read_temperature(&self) -> Result<Celsius, Error> {
        let (t, _, _) = self.read_all()?;
        Ok(t)
    }
}

impl PressureSensor<Error> for Bme280 {
    fn read_pressure(&self) -> Result<Hectopascals, Error> {
        let (_, p, _) = self.read_all()?;
        Ok(p)
    }
}

impl HumiditySensor<Error> for Bme280 {
    fn read_humidity(&self) -> Result<RelativeHumidity, Error> {
        let (_, _, h) = self.read_all()?;
        Ok(h)
    }
}
