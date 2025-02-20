/* The Clear BSD License
 *
 * Copyright (c) 2025 EdgeImpulse Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 *   * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
