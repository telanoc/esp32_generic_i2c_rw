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

#ifndef _GENERIC_I2C_RW_H_
#define _GENERIC_I2C_RW_H_

#include "freertos/FreeRTOS.h"
#include <driver/gpio.h>
#include <driver/i2c.h>

/* this is just a timeout.  It doesn't actually wait 1 second */
#define BEGIN_TIMEOUT  (1000 / portTICK_RATE_MS)

#define ACK_CHECK_EN   (1)   /* I2C master will check ack from slave */
#define ACK_CHECK_DIS  (0)   /* I2C master will not check ack from slave */
#define ACK_VAL        (0)   /* I2C ack value */
#define NACK_VAL       (1)   /* I2C nack value */


/**
 * @brief i2c master initialization
 *
 * @param portnum - Either I2C_NUM_0 or I2C_NUM_1
 * @param sclpin  - GPIO pin number for the SCL signal
 * @param stapin  - GPIO pin number for the SDA signal
 * @param i2c_frequency - Frequency of the I2C buss (100000 or 400000)
 *
 * returns:
 *    ESP_OK - Driver initialized
 *    ESP_FAIL - problem occurred
 */
esp_err_t generic_i2c_master_init(int portnum, int sclpin, int sdapin, int i2c_frequency);


/**
 * @brief Read a register value from an i2c device
 *
 * @param hwaddr - Device address  e.g. 0x20 for an MCP23017
 * @param regaddr - Register to read from
 *
 * @return
 *     Value read from i2c device register
 */
uint8_t generic_read_i2c_register(uint8_t hwaddr, uint8_t regaddr);


/**
 * @brief Read a register value from an i2c device
 *
 * @param hwaddr - Device address  e.g. 0x20 for an MCP23017
 * @param regaddr - Register to read from
 *
 * @return
 *     Value read from i2c device register
 */
uint16_t generic_read_i2c_register_word(uint8_t hwaddr, uint8_t regaddr);


/**
 * @brief Write an 8 bit value to an i2c device
 *
 * @param hwaddr - Device address  e.g. 0x20 for an MCP23017
 * @param regaddr - Register to write to
 * @param value - The data to write
 *
 * @return
 *     n/a
 */
void generic_write_i2c_register(uint8_t hwaddr, uint8_t regaddr, uint8_t value);


/**
 * @brief Write a 16 bit value to an i2c device
 *
 * @param hwaddr - Device address  e.g. 0x20 for an MCP23017
 * @param regaddr - Register to write to
 * @param value - The data to write
 *
 * @return
 *     n/a
 */
void generic_write_i2c_register_word(uint8_t hwaddr, uint8_t regaddr, uint16_t value);


#endif /* _GENERIC_I2C_RW_H_ */
