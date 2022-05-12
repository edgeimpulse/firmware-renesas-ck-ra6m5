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
#ifndef PERIPHERAL_I2S_H_
#define PERIPHERAL_I2S_H_

#include "common_utils.h"
#include "fsp_common_api.h"

FSP_CPP_HEADER

#define EI_I2S_SAMPLES_PER_READ         (1024)
#define EI_I2S_READ_SAMPLES_BYTE        (EI_I2S_SAMPLES_PER_READ*4u)    /* samples are on 4 byte */

extern int ei_i2s_driver_init(void);
extern int ei_i2s_driver_deinit(void);
extern int ei_i2s_init(void);
extern int ei_i2s_deinit(void);
extern int ei_i2s_read(volatile int32_t* buffer, uint32_t max_read);
extern i2s_event_t ei_i2s_get_status(void);
extern uint16_t ei_i2s_get_buffer(int32_t* dst, uint16_t max_byte);

FSP_CPP_FOOTER

#endif /* PERIPHERAL_I2S_H_ */
