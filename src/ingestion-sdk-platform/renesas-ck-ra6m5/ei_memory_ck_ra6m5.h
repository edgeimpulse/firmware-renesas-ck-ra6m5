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

#ifndef INGESTION_SDK_PLATFORM_RENESAS_CK_RA6M5_EI_MEMORY_CK_RA6M5_H_
#define INGESTION_SDK_PLATFORM_RENESAS_CK_RA6M5_EI_MEMORY_CK_RA6M5_H_

#include "firmware-sdk/ei_device_memory.h"
#include "peripheral/flash_handler.h"

class EiFlashMemory : public EiDeviceMemory {
private:
    const uint16_t flash_type;
    const uint32_t base_address;
    const uint16_t write_size_multiple;

    uint8_t residual_to_write;
    uint8_t residual_array[128];
    uint32_t last_offset;

    uint32_t bytes_written;

protected:

public:
    void write_residual(void);
    uint32_t read_sample_data(uint8_t *sample_data, uint32_t address, uint32_t sample_data_size);
    uint32_t write_sample_data(uint8_t *sample_data, uint32_t address, uint32_t sample_data_size);
    uint32_t erase_sample_data(uint32_t address, uint32_t num_bytes);

    uint32_t read_data(uint8_t *data, uint32_t address, uint32_t num_bytes);
    uint32_t write_data(const uint8_t *data, uint32_t address, uint32_t num_bytes);
    uint32_t erase_data(uint32_t address, uint32_t num_bytes);

public:
    EiFlashMemory(uint16_t to_set_flash_type, uint32_t config_struct_size);
};

#endif /* INGESTION_SDK_PLATFORM_RENESAS_CK_RA6M5_EI_MEMORY_CK_RA6M5_H_ */
