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
/* */
#include <renesas-ck-ra6m5/ei_memory_ck_ra6m5.h>
#include "peripheral/flash_handler.h"
#include <string.h>

/* */

/* flash size */
const uint32_t flash_memory_size[]
                           = {FLASH_HP_CODE_FLASH_BLOCK_SIZE_32KB * FLASH_HP_CODE_32KB_BLOCK_NUM,
                              FLASH_HP_DATA_FLASH_SIZE };

/* block size */
const uint32_t flash_block_size[]
                           = {FLASH_HP_CODE_FLASH_BLOCK_SIZE_32KB, FLASH_HP_DATA_FLASH_BLOCK_SIZE};

/* Min write size */
const uint16_t flash_write_size[]
                           = {FLASH_HP_CODE_FLASH_BLOCK_WR_SIZE, FLASH_HP_DATA_FLASH_BLOCK_WR_SIZE};

/* base address */
const uint32_t flash_base_address[]
                           = {FLASH_HP_CODE_FLASH_START_ADDRESS_SECOND_HALF, FLASH_HP_DATA_FLASH_START_ADDRESS};

EiFlashMemory::EiFlashMemory(uint16_t to_set_flash_type, uint32_t config_struct_size):
    EiDeviceMemory(config_struct_size, FLASH_HANDLER_ERASE_TIME, flash_memory_size[to_set_flash_type], flash_block_size[to_set_flash_type]),
    flash_type(to_set_flash_type),
    base_address(flash_base_address[to_set_flash_type]),
    write_size_multiple(flash_write_size[to_set_flash_type])
{
    residual_to_write = 0;
    memset(residual_array, 0, sizeof(residual_array));
    last_offset = 0;
    bytes_written = 0;
    //flash_handler_init();
}

/**
 *
 * @param data      pointer to where to put data
 * @param address   need to specify
 * @param num_bytes how many bytes
 * @return
 */
uint32_t EiFlashMemory::read_data(uint8_t *data, uint32_t address, uint32_t num_bytes)
{
    uint32_t i;
    uint8_t* p_memory;

    p_memory = (uint8_t *)(base_address + address);

    if(address + num_bytes > this->memory_size) {
        num_bytes = this->memory_size - address;
    }

    for (i = 0; i < num_bytes ; i++)
    {
        data[i] = p_memory[i];
    }

    return num_bytes;
}

/**
 *
 * @param data
 * @param address
 * @param num_bytes
 * @return
 */
uint32_t EiFlashMemory::write_data(const uint8_t *data, uint32_t address, uint32_t num_bytes)
{
    uint32_t written = 0;
    const uint8_t* p_write;
    uint32_t to_write = num_bytes;

    p_write = data;

    if (residual_to_write != 0) /* still some residual */
    {
        if ((to_write + residual_to_write) < write_size_multiple)   /* even whit this write, we don't read the minimum to write */
        {
            for (uint8_t i = 0; i < to_write; i ++)
            {
                residual_array[residual_to_write + i] = data[i];  /* complete residual array and write it*/
            }
            residual_to_write+=(uint8_t)to_write;

            last_offset+=to_write;

            return num_bytes;   /* and we finish here...*/
        }
        else    /* ok we can write residual plus something new */
        {
            for (uint8_t i = residual_to_write; i < write_size_multiple; i ++)
            {
                residual_array[i] = data[i - residual_to_write];  /* complete residual array and write it*/

            }

            written = flash_handler_write((t_flash_memory)flash_type, (base_address + bytes_written), residual_array, write_size_multiple);
            bytes_written += written;
            p_write = &data[(written - residual_to_write)]; /* let's move it */

            to_write -= (write_size_multiple - residual_to_write);  /* subtract the amount written - BUG !*/
            num_bytes -= (write_size_multiple - residual_to_write);

            residual_to_write = 0;
            memset(residual_array, 0xFF, sizeof(residual_array));   /* fill with 0xFF */

            last_offset += written;   /* update it */
        }

    }

    to_write = flash_handler_get_blocks_number(to_write, write_size_multiple)*write_size_multiple;

    if (to_write != 0)  /* if any, write them */
    {
        /* the handler expect the number of bytes */
        written = flash_handler_write((t_flash_memory)flash_type, (base_address + bytes_written), p_write, to_write);
        bytes_written += written;
        last_offset += written;  // what if not updated ?
    }

    if (to_write != num_bytes)
    {
        residual_to_write = static_cast<uint8_t>(num_bytes - to_write);   /* calc the residual bytes */
        for (uint8_t i = 0; i < residual_to_write; i ++)
        {
            residual_array[i] = p_write[to_write + i];
        }
    }

    return num_bytes;
}

/**
 * @brief Write last data (if any)
 */
void EiFlashMemory::write_residual(void)
{
    uint32_t written = 0;

    if (residual_to_write > 0)
    {
        for (uint8_t i = residual_to_write; i < write_size_multiple; i++ ){

            residual_array[i] = 0xFF;   // fill

        }

        written = flash_handler_write((t_flash_memory)flash_type, (base_address + bytes_written), residual_array, write_size_multiple);
        last_offset += written;
        bytes_written += written;

        residual_to_write = 0;
        memset(residual_array, 0xFF, sizeof(residual_array));   /* fill with 0xFF */
    }

}

/**
 *
 * @param address - memory offset
 * @param num_bytes bytes to be erased
 * @return
 */
uint32_t EiFlashMemory::erase_data(uint32_t address, uint32_t num_bytes)
{
    last_offset = 0;    // ??
    residual_to_write = 0;
    memset(residual_array, 0xFF, sizeof(residual_array));   /* fill with 0xFF */

    bytes_written = 0;

    /* the handler expect the number of bytes */
    return flash_handler_erase((t_flash_memory)flash_type, (base_address + address), num_bytes);
}

/**
 *
 * @param sample_data
 * @param address
 * @param sample_data_size
 * @return
 */
uint32_t EiFlashMemory::read_sample_data(uint8_t *sample_data, uint32_t address, uint32_t sample_data_size)
{
    return read_data(sample_data, address, sample_data_size);
}

/**
 *
 * @param sample_data
 * @param address
 * @param sample_data_size
 * @return
 */
uint32_t EiFlashMemory::write_sample_data(uint8_t *sample_data, uint32_t address, uint32_t sample_data_size)
{
    return write_data(sample_data, address, sample_data_size);
}

/**
 *
 * @param address
 * @param num_bytes
 * @return
 */
uint32_t EiFlashMemory::erase_sample_data(uint32_t address, uint32_t num_bytes)
{
    return erase_data(address, num_bytes);
}
