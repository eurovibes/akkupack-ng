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

def_enum!(
    #[derive(PartialEq,Eq, Debug)]
    pub Cmd => u8 {
        CMD_AI => 0x0a,
        CMD_CNTL => 0x00,
        CMD_FCC => 0x06,
        CMD_FLAGS => 0x0e,
        CMD_FLAGSB => 0x12,
        CMD_I => 0x10,
        CMD_ME => 0x03,
        CMD_RM => 0x04,
        CMD_SOC => 0x02,
        CMD_TEMP => 0x0c,
        CMD_VOLT => 0x04,
        CMD_ATTE => 0x18,
        CMD_ATTF => 0x1A,
        CMD_PCHG => 0x1C,
        CMD_DOD0T => 0x1E,
        CMD_AE => 0x24,
        CMD_AP => 0x26,
        CMD_SERNUM => 0x28,
        CMD_INTTEMP => 0x2A,
        CMD_CC => 0x2C,
        CMD_SOH => 0x2E,
        CMD_CHGV => 0x30,
        CMD_CHGI => 0x32,
        CMD_PKCFG => 0x3A,
        CMD_DCAP => 0x3C,
        CMD_DFCLS => 0x3E,
        CMD_DFBLK => 0x3F,
        CMD_A_DF => 0x40,
        CMD_ACKS_DFD => 0x54,
        CMD_DFD => 0x55,
        CMD_DFDCKS => 0x60,
        CMD_DFDCNTL => 0x61,
        CMD_GN => 0x62,
        CMD_LS => 0x63,
        CMD_DEOC => 0x64,
        CMD_QS => 0x66,
        CMD_TRC => 0x68,
        CMD_TFCC => 0x6A,
        CMD_ST => 0x6C,
        CMD_QPC => 0x6E,
        CMD_DOD0 => 0x70,
        CMD_QD0 => 0x72,
        CMD_QT => 0x74,
    }
);

/*
Gauge control subcommands:
Control function   Subcmd Sealed access Description 
CONTROL_STATUS     0x0000 Yes Reports the status of key features.
DEVICE_TYPE        0x0001 Yes Reports the device type of 0x100 (indicating BQ34Z100-G1)
FW_VERSION         0x0002 Yes Reports the firmware version on the device type
HW_VERSION         0x0003 Yes Reports the hardware version of the device type
RESET_DATA         0x0005 Yes Returns reset data
PREV_MACWRITE      0x0007 Yes Returns previous Control() command code
CHEM_ID            0x0008 Yes Reports the chemical identifier of the Impedance Track configuration
BOARD_OFFSET       0x0009 Yes Forces the device to measure and store the board offset
CC_OFFSET          0x000A Yes Forces the device to measure the internal CC offset
CC_OFFSET_SAVE     0x000B Yes Forces the device to store the internal CC offset
DF_VERSION         0x000C Yes Reports the data flash version on the device
SET_FULLSLEEP      0x0010 Yes Set the [FULLSLEEP] bit in the control register to 1
STATIC_CHEM_CHKSUM 0x0017 Yes Calculates chemistry checksum
SEALED             0x0020 No  Places the device in SEALED access mode
IT_ENABLE          0x0021 No  Enables the Impedance Track algorithm
CAL_ENABLE         0x002D No  Toggle CALIBRATION mode enable
RESET              0x0041 No  Forces a full reset of the BQ34Z100-G1
EXIT_CAL           0x0080 No  Exit CALIBRATION mode
ENTER_CAL          0x0081 No  Enter CALIBRATION mode
OFFSET_CAL         0x0082 No  Reports internal CC offset in CALIBRATION mod
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
    fn cntrl(&mut self, subcmd: SubCmds) -> Result<u16, Error>;
    fn get_cap_full(&mut self) -> Result<u16, Error>;
    fn get_cap_remain(&mut self) -> Result<u16, Error>;
    fn get_current(&mut self) -> Result<u16, Error>;
    fn get_current_avg(&mut self) -> Result<u16, Error>;
    fn get_flags(&mut self) -> Result<u32, Error>;
    fn get_max_error(&mut self) -> Result<u8, Error>;
    fn get_state_of_charge(&mut self) -> Result<u8, Error>;
    fn get_temp(&mut self) -> Result<u16, Error>;
    fn get_voltage(&mut self) -> Result<u16, Error>;
    fn read_data_flash_class(&mut self, class: u8, data: &mut [u8]) -> Result<(), Error>;
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

    fn read(&mut self, cmd: Cmd) -> Result<u16, Error> {
        let mut data = [0u8; 2];
        let c = cmd.get();

        match self.i2c.blocking_write_read(self.addr, &[c], &mut data) {
            Ok(()) => trace!("cmd({}): {}", c, (data[1] as u16) << 8 | data[0] as u16),
            Err(Error::Timeout) => {
                error!("Operation timed out");
                return Err(Error::Timeout);
            },
            Err(e) => {
                error!("I2c Error: {:?}", e);
                return Err(e);
            },
        }

        let result = ((data[1] as u16) << 8) | data[0] as u16;
        Ok(result)
    }

    pub fn dev_type(&mut self) -> u16 {
        let mut rx_buffer: [u8; 2] = [0; 2];

        match self.i2c.blocking_write(0x55, &[0, 1, 0]) {
            Ok(()) => debug!("Successful command write"),
            Err(Error::Timeout) => error!("Operation timed out"),
            Err(e) => error!("I2c Error: {:?}", e),
        }

        match self.i2c.blocking_read(self.addr, &mut rx_buffer) {
            Ok(()) => debug!("read(): {:#04x} {:#04x}", rx_buffer[0], rx_buffer[1]),
            Err(Error::Timeout) => error!("Operation timed out"),
            Err(e) => error!("I2c Error: {:?}", e),
        }

        ((rx_buffer[1] as u16) << 8) | rx_buffer[0] as u16
    }
}

impl Gauge for BqXxx<'_> {
    fn cntrl(&mut self, subcmd: SubCmds) -> Result<u16, Error> {
        let c = Cmd::CMD_CNTL.get();
        let sc: u16 = subcmd.get();
        let cmd = [c, sc as u8, (sc >> 8) as u8];
        let mut data = [0u8; 2];

        match self.i2c.blocking_write_read(self.addr, &cmd, &mut data) {
            Ok(()) => debug!("cntrl({}): {:#06x}", sc, (data[1] as u16) << 8 | data[0] as u16),
            Err(Error::Timeout) => {
                error!("Operation timed out");
                return Err(Error::Timeout);
            },
            Err(e) => {
                error!("I2c Error: {:?}", e);
                return Err(e);
            },
        }

        Ok(((data[1] as u16) << 8) | data[0] as u16)
    }

    fn get_cap_full(&mut self) -> Result<u16, Error> {
        self.read(Cmd::CMD_FCC)
    }

    fn get_cap_remain(&mut self) -> Result<u16, Error> {
        self.read(Cmd::CMD_RM)
    }

    fn get_current(&mut self) -> Result<u16, Error> {
        self.read(Cmd::CMD_I)
    }

    fn get_current_avg(&mut self) -> Result<u16, Error> {
        self.read(Cmd::CMD_AI)
    }

    fn get_flags(&mut self) -> Result<u32, Error> {
        let flags = match self.read(Cmd::CMD_FLAGS) {
            Ok(flags) => flags,
            Err(e) => return Err(e),
        };
        let flagsb = match self.read(Cmd::CMD_FLAGSB) {
            Ok(flagsb) => flagsb,
            Err(e) => return Err(e),
        };

        Ok(((flagsb as u32) << 16) | flags as u32)
    }

    fn get_max_error(&mut self) -> Result<u8, Error> {
        let mut data = [0u8; 1];

        match self.i2c.blocking_write(self.addr, &[Cmd::CMD_ME.get()]) {
            Ok(()) => trace!("write OK!"),
            Err(Error::Timeout) => {
                error!("Operation timed out");
                return Err(Error::Timeout);
            },
            Err(e) => {
                error!("I2c Error: {:?}", e);
                return Err(e);
            },
        }

        match self
            .i2c
            .blocking_write_read(self.addr, &[Cmd::CMD_ME.get()], &mut data)
        {
            Ok(()) => trace!("MaxError: {}", data[0]),
            Err(Error::Timeout) => {
                error!("Operation timed out");
                return Err(Error::Timeout);
            },
            Err(e) => {
                error!("I2c Error: {:?}", e);
                return Err(e);
            },
        }
        Ok(data[0])
    }

    fn get_state_of_charge(&mut self) -> Result<u8, Error> {
        let mut data = [0u8; 1];

        match self
            .i2c
            .blocking_write_read(self.addr, &[Cmd::CMD_SOC.get()], &mut data)
        {
            Ok(()) => trace!("StateOfCharge: {}", data[0]),
            Err(Error::Timeout) => {
                error!("Operation timed out");
                return Err(Error::Timeout);
            },
            Err(e) => {
                error!("I2c Error: {:?}", e);
                return Err(e);
            },
        }
        Ok(data[0])
    }

    fn get_temp(&mut self) -> Result<u16, Error> {
        self.read(Cmd::CMD_TEMP)
    }

    fn get_voltage(&mut self) -> Result<u16, Error> {
        self.read(Cmd::CMD_VOLT)
    }

    fn read_data_flash_class(&mut self, class: u8, data: &mut [u8]) -> Result<(), Error> {
        let mut start: usize = 0;
        let mut block: usize = 0;
        let mut len: usize;

        loop {
            len = data.len() - start;
            if (len > 32) {
                len = 32;
            }


        }

        Ok(())
    }
}
