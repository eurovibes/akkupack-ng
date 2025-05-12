// SPDX-License-Identifier: Apache-2.0
//
// SPDX-FileCopyrightText: 2025 Benedikt Spranger <b.spranger@linutronix.de>

#![no_std]

use defmt::*;
use embassy_stm32::i2c::{Error, I2c};
use embassy_stm32::mode::Blocking;
use embassy_stm32::time::Hertz;
use embassy_stm32::Peripherals;
use {defmt_rtt as _, panic_probe as _};

const AI: [u8; 1] = [0x0a];
const CNTL: u8 = 0x00;
const FCC: [u8; 1] = [0x06];
const FLAGS: [u8; 1] = [0x0e];
const FLAGSB: [u8; 1] = [0x12];
const I: [u8; 1] = [0x10];
const ME: [u8; 1] = [0x03];
const RM: [u8; 1] = [0x04];
const SOC: [u8; 1] = [0x02];
const TEMP: [u8; 1] = [0x0c];
const VOLT: [u8; 1] = [0x04];

macro_rules! def_enum {
    (
        $(#[$attr:meta])*
        $vis:vis $name:ident => $ty:ty {
            $($variant:ident => $val:expr),+
            $(,)?
        }
    ) => {
        $(#[$attr])*
        $vis struct $name($ty);

        impl $name {
            $(
                pub const $variant: Self = Self($val);
            )+

            pub const VARIANTS: &'static [Self] = &[$(Self::$variant),+];

            pub const fn get(self) -> $ty {
                self.0
            }
        }
    };
}

/*
Gauge control subcommands:
CONTROL_STATUS 0x0000 Yes Reports the status of key features.
DEVICE_TYPE 0x0001 Yes Reports the device type of 0x100 (indicating BQ34Z100-G1)
FW_VERSION 0x0002 Yes Reports the firmware version on the device type
HW_VERSION 0x0003 Yes Reports the hardware version of the device type
RESET_DATA 0x0005 Yes Returns reset data
PREV_MACWRITE 0x0007 Yes Returns previous Control() command code
CHEM_ID 0x0008 Yes Reports the chemical identifier of the Impedance Track configuration
BOARD_OFFSET 0x0009 Yes Forces the device to measure and store the board offset
CC_OFFSET 0x000A Yes Forces the device to measure the internal CC offset
CNTL FUNCTION CNTL DATA SEALED ACCESS DESCRIPTION
CC_OFFSET_SAVE 0x000B Yes Forces the device to store the internal CC offset
DF_VERSION 0x000C Yes Reports the data flash version on the device
SET_FULLSLEEP 0x0010 Yes Set the [FULLSLEEP] bit in the control register to 1
STATIC_CHEM_CHKSUM 0x0017 Yes Calculates chemistry checksum
SEALED 0x0020 No Places the device in SEALED access mode
IT_ENABLE 0x0021 No Enables the Impedance Track algorithm
CAL_ENABLE 0x002D No Toggle CALIBRATION mode enable
RESET 0x0041 No Forces a full reset of the BQ34Z100-G1
EXIT_CAL 0x0080 No Exit CALIBRATION mode
ENTER_CAL 0x0081 No Enter CALIBRATION mode
OFFSET_CAL 0x0082 No Reports internal CC offset in CALIBRATION mod
*/

def_enum!(
    #[derive(PartialEq,Eq, Debug)]
    pub SubCmds => u16 {
        SUB_CMD_CONTROL_STATUS => 0x0000,
        SUB_CMD_DEVICE_TYPE => 0x0001,
        SUB_CMD_FW_VERSION => 0x0002,
        SUB_CMD_HW_VERSION => 0x0003,
        SUB_CMD_RESET_DATA => 0x0005,
        SUB_CMD_PREV_MACWRITE => 0x0007,
        SUB_CMD_CHEM_ID => 0x0008,
        SUB_CMD_BOARD_OFFSET => 0x0009,
        SUB_CMD_CC_OFFSET => 0x000A,
        SUB_CMD_CC_OFFSET_SAVE => 0x000B,
        SUB_CMD_DF_VERSION => 0x000C,
        SUB_CMD_SET_FULLSLEEP => 0x0010,
        SUB_CMD_STATIC_CHEM_CHKSUM => 0x0017,
        SUB_CMD_SEALED => 0x0020,
        SUB_CMD_IT_ENABLE => 0x0021,
        SUB_CMD_CAL_ENABLE => 0x002D,
        SUB_CMD_RESET => 0x0041,
        SUB_CMD_EXIT_CAL => 0x0080,
        SUB_CMD_ENTER_CAL => 0x0081,
        SUB_CMD_OFFSET_CAL => 0x0082,
    }
);

pub trait Gauge {
    fn cntrl(&mut self, subcmd: SubCmds) -> u16;
    fn get_cap_full(&mut self) -> u16;
    fn get_cap_remain(&mut self) -> u16;
    fn get_current(&mut self) -> u16;
    fn get_current_avg(&mut self) -> u16;
    fn get_flags(&mut self) -> u32;
    fn get_max_error(&mut self) -> u8;
    fn get_state_of_charge(&mut self) -> u8;
    fn get_temp(&mut self) -> u16;
    fn get_voltage(&mut self) -> u16;
}

pub struct BqXxx<'a> {
    addr: u8,
    i2c: I2c<'a, Blocking>,
}

impl<'a> BqXxx<'a> {
    pub fn new(p: Peripherals, addr: u8) -> Self {
        let this = Self {
            addr: addr,
            i2c: I2c::new_blocking(p.I2C1, p.PB6, p.PB7, Hertz(100_000), Default::default()),
        };
        this
    }

    fn read(&mut self, cmd: [u8; 1]) -> u16 {
        let mut data = [0u8; 2];

        match self.i2c.blocking_write_read(self.addr, &cmd, &mut data) {
            Ok(()) => trace!("voltage(): {}", (data[0] as u16) << 8 | data[1] as u16),
            Err(Error::Timeout) => error!("Operation timed out"),
            Err(e) => error!("I2c Error: {:?}", e),
        }

        let result = ((data[0] as u16) << 8) | data[1] as u16;
        result
    }
}

impl Gauge for BqXxx<'_> {
    fn cntrl(&mut self, subcmd: SubCmds) -> u16 {
        let sc: u16 = subcmd.get();
        let cmd = [CNTL, sc as u8, (sc >> 8) as u8, CNTL];
        let mut data = [0u8; 2];

        match self.i2c.blocking_write_read(self.addr, &cmd, &mut data) {
            Ok(()) => trace!("cntrl({}): {}", sc, (data[0] as u16) << 8 | data[1] as u16),
            Err(Error::Timeout) => error!("Operation timed out"),
            Err(e) => error!("I2c Error: {:?}", e),
        }

        ((data[0] as u16) << 8) | data[1] as u16
    }

    fn get_cap_full(&mut self) -> u16 {
        self.read(FCC)
    }

    fn get_cap_remain(&mut self) -> u16 {
        self.read(RM)
    }

    fn get_current(&mut self) -> u16 {
        self.read(I)
    }

    fn get_current_avg(&mut self) -> u16 {
        self.read(AI)
    }

    fn get_flags(&mut self) -> u32 {
        let flags = self.read(FLAGS);
        let flagsb = self.read(FLAGSB);

        ((flags as u32) << 16) | flagsb as u32
    }

    fn get_max_error(&mut self) -> u8 {
        let mut data = [0u8; 1];

        match self
            .i2c
            .blocking_write_read(self.addr, &ME, &mut data)
        {
            Ok(()) => trace!("MaxError: {}", data[0]),
            Err(Error::Timeout) => error!("Operation timed out"),
            Err(e) => error!("I2c Error: {:?}", e),
        }
        data[0]
    }

    fn get_state_of_charge(&mut self) -> u8 {
        let mut data = [0u8; 1];

        match self.i2c.blocking_write_read(self.addr, &SOC, &mut data) {
            Ok(()) => trace!("StateOfCharge: {}", data[0]),
            Err(Error::Timeout) => error!("Operation timed out"),
            Err(e) => error!("I2c Error: {:?}", e),
        }
        data[0]
    }

    fn get_temp(&mut self) -> u16 {
        self.read(TEMP)
    }

    fn get_voltage(&mut self) -> u16 {
        self.read(VOLT)
    }
}
