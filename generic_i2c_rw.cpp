//==========================================================================
// Copyright 2017 Pete Cervasio
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
//==========================================================================

#include <driver/gpio.h>
#include <driver/i2c.h>

#include "generic_i2c_rw.h"

// Turn this off to not use the internal pullup resistors on the SDA/SCL
// lines.
#define GENERIC_I2C_USE_PULLUPS

/**
 * i2c master initialization
 */
esp_err_t generic_i2c_master_init(int portnum, int sclpin, int sdapin, int i2c_frequency)
{
    i2c_port_t i2c_master_port = (i2c_port_t)portnum;
    i2c_config_t conf;

    conf.mode = I2C_MODE_MASTER;

    conf.sda_io_num = (gpio_num_t)sdapin;
	conf.scl_io_num = (gpio_num_t)sclpin;

#ifdef GENERIC_I2C_USE_PULLUPS
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
#endif

    conf.master.clk_speed = i2c_frequency;
    
	i2c_param_config(i2c_master_port, &conf);
    
	return i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}


/**
 * Read a register value from an i2c device
 */
uint8_t generic_read_i2c_register(uint8_t hwaddr, uint8_t regaddr)
{
    uint8_t retval;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (hwaddr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, regaddr, NACK_VAL);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, BEGIN_TIMEOUT);
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (hwaddr << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &retval, NACK_VAL);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, BEGIN_TIMEOUT);
    i2c_cmd_link_delete(cmd);
    return retval;
}


/**
 * Read a register value from an i2c device
 */
uint16_t generic_read_i2c_register_word(uint8_t hwaddr, uint8_t regaddr)
{
    uint16_t res = 0;
    uint8_t res2 = 0;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (hwaddr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, regaddr, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, BEGIN_TIMEOUT);
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (hwaddr << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, (uint8_t *)&res, ACK_VAL);
    i2c_master_read_byte(cmd, &res2, NACK_VAL);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, BEGIN_TIMEOUT);
    i2c_cmd_link_delete(cmd);

    return (res << 8) | res2;
}


/**
 * Write an 8 bit value to an i2c device
 */
void generic_write_i2c_register(uint8_t hwaddr, uint8_t regaddr, uint8_t value)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (hwaddr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, regaddr, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, value, NACK_VAL);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, BEGIN_TIMEOUT);
    i2c_cmd_link_delete(cmd);
}


/**
 * Write a 16 bit value to an i2c device
 */
void generic_write_i2c_register_word(uint8_t hwaddr, uint8_t regaddr, uint16_t value)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (hwaddr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, regaddr, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, value & 0xff, ACK_VAL);
    i2c_master_write_byte(cmd, value >> 8, NACK_VAL);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, BEGIN_TIMEOUT);
    i2c_cmd_link_delete(cmd);
}

