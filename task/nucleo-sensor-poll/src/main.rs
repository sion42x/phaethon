// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Sensor polling task for Nucleo board
//!
//! Reads BME280 (temperature, pressure, humidity) and INA260
//! (current, voltage, power) at 1 Hz and posts to the sensor task.

#![no_std]
#![no_main]

use drv_i2c_devices::bme280::Bme280;
use drv_i2c_devices::ina260::Ina260;
use drv_i2c_devices::{CurrentSensor, PowerSensor, VoltageSensor};
use ringbuf::*;
use task_sensor_api::{NoData, Sensor as SensorApi, SensorId};
use userlib::*;

task_slot!(I2C, i2c_driver);
task_slot!(SENSOR, sensor);

const TIMER_INTERVAL: u64 = 1000;

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
enum Trace {
    None,
    Start,
    Bme280TempOk,
    Bme280PressOk,
    Bme280HumOk,
    Bme280Error(drv_i2c_api::ResponseCode),
    Ina260CurrentOk,
    Ina260VoltageOk,
    Ina260PowerOk,
    Ina260Error(drv_i2c_api::ResponseCode),
}
ringbuf!(Trace, 32, Trace::None);

#[export_name = "main"]
fn main() -> ! {
    let i2c_task = I2C.get_task_id();
    let sensor_api = SensorApi::from(SENSOR.get_task_id());

    // Create device handles once — BME280 calibration data is cached
    // in the struct, avoiding repeated chip ID + calibration register
    // reads on a bus with no external pull-ups.
    let bme_dev = i2c_config::devices::bme280_env(i2c_task);
    let bme = Bme280::new(&bme_dev);
    let ina_dev = i2c_config::devices::ina260_nas_power(i2c_task);
    let mut ina = Ina260::new(&ina_dev);

    ringbuf_entry!(Trace::Start);

    loop {
        hl::sleep_for(TIMER_INTERVAL);

        // Poll BME280 (environmental sensor)
        // Retry once on failure — breadboard without external
        // pull-ups has marginal signal integrity
        {
            let result = bme.read_all().or_else(|_| bme.read_all());
            match result {
                Ok((temp, press, hum)) => {
                    ringbuf_entry!(Trace::Bme280TempOk);
                    sensor_api.post_now(
                        i2c_config::sensors::BME280_ENV_SENSORS.temperature,
                        temp.0,
                    );
                    ringbuf_entry!(Trace::Bme280PressOk);
                    sensor_api.post_now(
                        i2c_config::sensors::BME280_ENV_SENSORS.pressure,
                        press.0,
                    );
                    ringbuf_entry!(Trace::Bme280HumOk);
                    sensor_api.post_now(
                        i2c_config::sensors::BME280_ENV_SENSORS.humidity,
                        hum.0,
                    );
                }
                Err(e) => {
                    // Log error but don't post nodata — keep last good
                    // reading in sensor task on marginal bus
                    let code = drv_i2c_api::ResponseCode::from(e);
                    ringbuf_entry!(Trace::Bme280Error(code));
                }
            }
        }

        // Poll INA260 (power monitor)
        {
            // Current (retry once on failure, same pull-up issue)
            match ina.read_iout().or_else(|_| ina.read_iout()) {
                Ok(v) => {
                    ringbuf_entry!(Trace::Ina260CurrentOk);
                    sensor_api.post_now(
                        i2c_config::sensors::INA260_NAS_POWER_SENSORS.current,
                        v.0,
                    );
                }
                Err(e) => {
                    let code = drv_i2c_api::ResponseCode::from(e);
                    ringbuf_entry!(Trace::Ina260Error(code));
                }
            }

            // Voltage
            match ina.read_vout().or_else(|_| ina.read_vout()) {
                Ok(v) => {
                    ringbuf_entry!(Trace::Ina260VoltageOk);
                    sensor_api.post_now(
                        i2c_config::sensors::INA260_NAS_POWER_SENSORS.voltage,
                        v.0,
                    );
                }
                Err(e) => {
                    let code = drv_i2c_api::ResponseCode::from(e);
                    ringbuf_entry!(Trace::Ina260Error(code));
                }
            }

            // Power
            match ina.read_power().or_else(|_| ina.read_power()) {
                Ok(v) => {
                    ringbuf_entry!(Trace::Ina260PowerOk);
                    sensor_api.post_now(
                        i2c_config::sensors::INA260_NAS_POWER_SENSORS.power,
                        v.0,
                    );
                }
                Err(e) => {
                    let code = drv_i2c_api::ResponseCode::from(e);
                    ringbuf_entry!(Trace::Ina260Error(code));
                }
            }
        }
    }
}

include!(concat!(env!("OUT_DIR"), "/i2c_config.rs"));
