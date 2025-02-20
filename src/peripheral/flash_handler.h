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

#ifndef PERIPHERAL_FLASH_HANDLER_H_
#define PERIPHERAL_FLASH_HANDLER_H_

#include "common_utils.h"
#include "fsp_common_api.h"
#include <stdint.h>

typedef enum e_flash_memory
{
    e_flash_code = 0,
    e_flash_data,
    e_flash_max
}t_flash_memory;

/*
 * programming data flash   : 4/8/16 bytes
 * erase data flash         : 64/182/256 bytes
 */
FSP_CPP_HEADER

#define FLASH_HANDLER_ERASE_TIME                        (250u)      /* in ms */

#define FLASH_HANDLER_AFTER_CLEAR                       (0xFF)

/* Code Flash */
#define FLASH_HP_CODE_FLASH_START_ADDRESS_FIRST_HALF    (0x00000000U)
#define FLASH_HP_CODE_FLASH_END_ADDRESS_FIRST_HALF      (0x00100000U)

#define FLASH_HP_CODE_FLASH_START_ADDRESS_SECOND_HALF   (0x00100000U)
#define FLASH_HP_CODE_FLASH_END_ADDRESS_SECOND_HALF     (0x00200000U)

#define FLASH_HP_CODE_FLASH_BLOCK_SIZE_32KB             (32*1024)   /* Block Size 32 KB */
#define FLASH_HP_CODE_FLASH_BLOCK_SIZE_8KB              (8*1024)    /* Block Size 8KB */

#define FLASH_HP_CODE_FLASH_BLOCK_WR_SIZE               (128u)

#define FLASH_HP_CODE_8KB_BLOCK_NUM                     (8u)
#define FLASH_HP_CODE_32KB_BLOCK_NUM                    (61u)

/* Data Flash */
#define FLASH_HP_DATA_FLASH_BLOCK_SIZE              (64)
#define FLASH_HP_DATA_FLASH_BLOCK_NUM               (128)

#define FLASH_HP_DATA_FLASH_BLOCK_WR_SIZE           (4u)

#define FLASH_HP_DATA_FLASH_SIZE                    (FLASH_HP_DATA_FLASH_BLOCK_SIZE * FLASH_HP_DATA_FLASH_BLOCK_NUM)

#define FLASH_HP_DATA_FLASH_START_ADDRESS           (0x08000000U)
#define FLASH_HP_DATA_FLASH_END_ADDRESS             (0x08000000U + FLASH_HP_DF_SIZE)

extern int flash_handler_init(void);
extern uint32_t flash_handler_erase(t_flash_memory flash_mem, uint32_t address, uint32_t bytes_to_clear);
extern uint32_t flash_handler_write(t_flash_memory flash_mem, uint32_t address, const uint8_t *data, uint32_t size);
extern uint32_t flash_handler_get_write_size(uint32_t desired_size, uint32_t write_size);
extern uint32_t flash_handler_get_blocks_number(uint32_t desired_size, uint32_t block_size);

FSP_CPP_FOOTER

#endif /* PERIPHERAL_FLASH_HANDLER_H_ */
