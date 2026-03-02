// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! NAS power state monitoring and control
//!
//! Reads PE10 (optocoupler output) to determine NAS power state,
//! and drives PE12 (MOSFET gate) to simulate power button presses.
//!
//! PE10: Input — low = NAS is on, high = NAS is off
//! PE12: Output — pulse high to simulate power button press

#![no_std]
#![no_main]

use drv_stm32xx_sys_api::{OutputType, Pull, Speed, Sys};
use idol_runtime::RequestError;
use ringbuf::*;
use task_nas_power_ctrl_api::NasPowerCtrlError;
use userlib::*;

task_slot!(SYS, sys);

/// PE10: optocoupler output (NAS power state)
const POWER_STATE_PIN: drv_stm32xx_sys_api::PinSet =
    drv_stm32xx_sys_api::Port::E.pin(10);

/// PE12: MOSFET gate (power button simulation)
const POWER_BUTTON_PIN: drv_stm32xx_sys_api::PinSet =
    drv_stm32xx_sys_api::Port::E.pin(12);

/// Maximum allowed pulse duration (5 seconds)
const MAX_PULSE_MS: u32 = 5000;

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
enum Trace {
    None,
    Start,
    PowerStateRead(bool),
    ButtonPress(u32),
    ButtonRelease,
}
ringbuf!(Trace, 16, Trace::None);

struct ServerImpl {
    sys: Sys,
}

impl idl::InOrderNasPowerCtrlImpl for ServerImpl {
    fn power_state(
        &mut self,
        _: &RecvMessage,
    ) -> Result<bool, RequestError<NasPowerCtrlError>> {
        // Read PE10: low = NAS on, high = NAS off
        let raw = self.sys.gpio_read(POWER_STATE_PIN);
        let nas_on = raw == 0;
        ringbuf_entry!(Trace::PowerStateRead(nas_on));
        Ok(nas_on)
    }

    fn power_button_press(
        &mut self,
        _: &RecvMessage,
        duration_ms: u32,
    ) -> Result<(), RequestError<NasPowerCtrlError>> {
        if duration_ms == 0 || duration_ms > MAX_PULSE_MS {
            return Err(NasPowerCtrlError::InvalidDuration.into());
        }

        ringbuf_entry!(Trace::ButtonPress(duration_ms));

        // Drive MOSFET gate high (simulate button press)
        self.sys.gpio_set(POWER_BUTTON_PIN);

        // Hold for requested duration
        hl::sleep_for(u64::from(duration_ms));

        // Release (drive low)
        self.sys.gpio_reset(POWER_BUTTON_PIN);

        ringbuf_entry!(Trace::ButtonRelease);
        Ok(())
    }
}

impl idol_runtime::NotificationHandler for ServerImpl {
    fn current_notification_mask(&self) -> u32 {
        // No notifications to handle; timer is used via hl::sleep_for
        0
    }

    fn handle_notification(&mut self, _bits: NotificationBits) {}
}

#[export_name = "main"]
fn main() -> ! {
    let sys = Sys::from(SYS.get_task_id());

    // Configure PE10 as input (no pull — optocoupler has external pull-up)
    sys.gpio_configure_input(POWER_STATE_PIN, Pull::None);

    // Configure PE12 as push-pull output, initially low
    sys.gpio_reset(POWER_BUTTON_PIN);
    sys.gpio_configure_output(
        POWER_BUTTON_PIN,
        OutputType::PushPull,
        Speed::Low,
        Pull::None,
    );

    ringbuf_entry!(Trace::Start);

    let mut server = ServerImpl { sys };
    let mut incoming = [0u8; idl::INCOMING_SIZE];
    loop {
        idol_runtime::dispatch(&mut incoming, &mut server);
    }
}

mod idl {
    use super::NasPowerCtrlError;

    include!(concat!(env!("OUT_DIR"), "/server_stub.rs"));
}
