// SPDX-License-Identifier: Apache-2.0
//
// SPDX-FileCopyrightText: 2025 Benedikt Spranger <b.spranger@linutronix.de>

#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::rcc::{
    AHBPrescaler, APBPrescaler, Hse, HseMode, Pll, PllPreDiv, PllSource, Sysclk,
};
use embassy_stm32::time::Hertz;
use embassy_stm32::Config;
use {defmt_rtt as _, panic_probe as _};

use akkupack_ng::{BqXxx, Gauge, SubCmds};

const ADDRESS: u8 = 0x55;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Hello BQ34Z100-G1!");
    let mut config = Config::default();
    {
        config.rcc.hse = Some(Hse {
            freq: Hertz(8_000_000),
            mode: HseMode::Oscillator,
        });
        config.rcc.pll = Some(Pll {
            src: PllSource::HSE,
            prediv: PllPreDiv::DIV1,
            mul: embassy_stm32::rcc::PllMul::MUL9,
        });
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV2;
        config.rcc.apb2_pre = APBPrescaler::DIV1;
        config.rcc.sys = Sysclk::PLL1_P;
    }
    let p = embassy_stm32::init(config);

    let mut bq = BqXxx::new(p, ADDRESS);

    info!("MaxError       : {} %", bq.get_max_error());
    info!("State of charge: {} %", bq.get_state_of_charge());
    info!("FW version     : {}", bq.cntrl(SubCmds::SUB_CMD_FW_VERSION));
    info!("Voltage        : {} V", bq.get_voltage());
    info!("Control status : {:#06x}", bq.cntrl(SubCmds::SUB_CMD_CONTROL_STATUS));
}
