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
