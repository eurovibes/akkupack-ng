// SPDX-License-Identifier: Apache-2.0
//
// SPDX-FileCopyrightText: 2025 Benedikt Spranger <b.spranger@linutronix.de>

#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::Config;
use {defmt_rtt as _, panic_probe as _};

use akkupack_ng::{BqXxx, Gauge, SubCmds};

const ADDRESS: u8 = 0x55;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Hello BQ34Z100-G1!");
    let config = Config::default();
    let p = embassy_stm32::init(config);

    let mut bq = BqXxx::new(p, ADDRESS);

    info!("Device type    : {}", bq.dev_type());

    info!("MaxError       : {} %", bq.get_max_error());
    info!("State of charge: {} %", bq.get_state_of_charge());
    info!("Device type    : {}", bq.cntrl(SubCmds::SUB_CMD_DEVICE_TYPE));
    info!("FW version     : {:#06x}", bq.cntrl(SubCmds::SUB_CMD_FW_VERSION));
    info!("HW version     : {}", bq.cntrl(SubCmds::SUB_CMD_HW_VERSION));
    info!("DF version     : {}", bq.cntrl(SubCmds::SUB_CMD_DF_VERSION));
    info!("Voltage        : {} V", bq.get_voltage());
    info!("Full capacity  : {}", bq.get_cap_full());
    info!("Rem. capacity  : {}", bq.get_cap_remain());
    info!("Current        : {}", bq.get_current());
    info!("Current Avg    : {}", bq.get_current_avg());
    info!("Flags          : {:#010x}", bq.get_flags());
    info!("Temperature    : {}", bq.get_temp());
    info!("Control status : {:#06x}", bq.cntrl(SubCmds::SUB_CMD_CONTROL_STATUS));

    // gauge_execute_fs(pHandle, pFileBuffer)

    //read data class DC_STATE:
    // n = gauge_read_data_class(pHandle, DC_STATE, pData, DC_STATE_LENGTH);
    // if (n) printf("Error reading data class, %d\n\r", n);
    // printf("Data Class 'State' (0x52):\n\r");
    // print_data(pData, DC_STATE_LENGTH);
    // this was for bq2742x - change offsets for your gauge
    // pData[10] = (DESIGN_CAPACITY & 0xFF00) >> 8;
    // pData[11] = DESIGN_CAPACITY & 0xFF;
    // pData[12] = (DESIGN_ENERGY & 0xFF00) >> 8;
    // pData[13] = DESIGN_ENERGY & 0xFF;
    // pData[16] = (TERMINATE_VOLTAGE & 0xFF00) >>8;
    // pData[17] = TERMINATE_VOLTAGE & 0xFF;
    // pData[27] = (TAPER_RATE & 0xFF00) >> 8;
    // pData[28] = TAPER_RATE & 0xFF;
    //write data class DC_STATE:
    // gauge_cfg_update(pHandle);
    // n = gauge_write_data_class(pHandle, DC_STATE, pData, DC_STATE_LENGTH);
    // if (n) printf("Error writing data class, %d\n\r", n);
    // gauge_exit(pHandle, SOFT_RESET);

}
