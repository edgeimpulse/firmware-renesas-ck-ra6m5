/* Edge Impulse ingestion SDK
 * Copyright (c) 2022 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef PERIPHERAL_I2C_HANDLER_H_
#define PERIPHERAL_I2C_HANDLER_H_

#include "common_utils.h"
#include "fsp_common_api.h"

FSP_CPP_HEADER

#define I2C_NO_WAIT_BETWEEN_W_R     (0)

extern int ei_i2c_init(void);
extern int ei_i2c_deinit(void);
extern int ei_i2c_write(rm_comms_ctrl_t * const p_api_ctrl, uint8_t*data, uint16_t bytes);
extern int ei_i2c_read_byte_command(rm_comms_ctrl_t * const p_api_ctrl, uint8_t read_cmd, uint8_t* read_data, uint16_t bytes, uint32_t delay_ms);
extern int ei_i2c_read_word_command(rm_comms_ctrl_t * const p_api_ctrl, uint16_t read_cmd, uint8_t* read_data, uint16_t bytes, uint32_t delay_ms);

FSP_CPP_FOOTER

#endif /* PERIPHERAL_I2C_HANDLER_H_ */
