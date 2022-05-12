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
/* Include ----------------------------------------------------------------- */
#include "flash_handler.h"
#include "r_flash_hp.h"

/* Private functions declaration ------------------------------------------- */
static uint32_t flash_handler_get_clear_size(uint32_t desired_size, uint32_t write_size);
static void flash_handler_wait(void);

static bool flash_handler_is_init = false;

/* Public functions -------------------------------------------------------- */
/**
 * @brief Initializes the flash controller
 *
 * @return
 */
int flash_handler_init(void)
{
    fsp_err_t error = FSP_SUCCESS;
    /* not supported in this MCU */
    //error = R_FLASH_HP_AccessWindowSet(&g_flash0_ctrl, FLASH_HP_CODE_FLASH_START_ADDRESS_SECOND_HALF, (FLASH_HP_CODE_FLASH_END_ADDRESS_SECOND_HALF-FLASH_HP_CODE_FLASH_BLOCK_SIZE_32KB));
    return error;
//
//    if (flash_handler_is_init == false)
//    {
//        error = R_FLASH_HP_Open(&g_flash0_ctrl, &g_flash0_cfg);
//        if (error == FSP_SUCCESS)
//        {
//            flash_handler_is_init = true;
//        }
//    }


    return (int)error;
}

/**
 *
 * @param address starting address
 * @param bytes_to_clear how many bytes
 * @return number of bytes erased
 */
uint32_t flash_handler_erase(t_flash_memory flash_mem, uint32_t address, uint32_t bytes_to_clear)
{
    fsp_err_t error = FSP_SUCCESS;
    uint32_t blocks_number = 0;


    switch(flash_mem)
    {
        case e_flash_code:
        {
            blocks_number = flash_handler_get_clear_size(bytes_to_clear, FLASH_HP_CODE_FLASH_BLOCK_SIZE_32KB);
            __disable_irq();
            /* TODO
             * address check:
             * - the correct flash area
             * - not erasing to much
             * */
            R_FLASH_HP_Open(&g_flash0_ctrl, &g_flash0_cfg);
            flash_handler_wait();
            error = R_FLASH_HP_Erase(&g_flash0_ctrl, address, blocks_number);
            R_FLASH_HP_Close(&g_flash0_ctrl);
            __enable_irq();
        }
        break;
        case e_flash_data:
        {
            blocks_number = flash_handler_get_clear_size(bytes_to_clear, FLASH_HP_DATA_FLASH_BLOCK_SIZE);
            R_FLASH_HP_Open(&g_flash0_ctrl, &g_flash0_cfg);
            flash_handler_wait();
            error = R_FLASH_HP_Erase(&g_flash0_ctrl, address, blocks_number);
            R_FLASH_HP_Close(&g_flash0_ctrl);
        }
        break;
        default:
        case e_flash_max:
        {

        }
        break;
    }

    if (error != FSP_SUCCESS)
    {
        bytes_to_clear = 0;
    }

    return bytes_to_clear;
}

/**
 * @brief write in data flash
 *
 * @param address   starting address
 * @param data      pointer to data to write
 * @param size      how many BYTES - this number must be a multiple of the programming size.
 * @return The number of bytes written. 0 if failed
 */
uint32_t flash_handler_write(t_flash_memory flash_mem, uint32_t address, const uint8_t *data, uint32_t size)
{
    uint32_t bytes_written = 0;
    fsp_err_t error = FSP_SUCCESS;
    uint32_t total_bytes_to_write;
    uint32_t normal_bytes_to_write;



    switch(flash_mem)
    {
        case e_flash_code:
        {
            /*
             * check address range
             * size is multiple of FLASH_MIN_PGM_SIZE_CF ?
             */
            if (size < FLASH_HP_CODE_FLASH_BLOCK_WR_SIZE)
            {
                uint8_t dummy_block[FLASH_HP_CODE_FLASH_BLOCK_WR_SIZE] = {0};

                for(uint16_t i = 0;  i < size ; i++)
                {
                    dummy_block[i] = data[i];  /* copy whats left */
                }

                __disable_irq();
                R_FLASH_HP_Open(&g_flash0_ctrl, &g_flash0_cfg);
                flash_handler_wait();
                error = R_FLASH_HP_Write(&g_flash0_ctrl, (uint32_t)dummy_block, address, FLASH_HP_CODE_FLASH_BLOCK_WR_SIZE);    /* data is passed as address value */
                R_FLASH_HP_Close(&g_flash0_ctrl);
                __enable_irq();

                if (error == FSP_SUCCESS)
                {
                    bytes_written = FLASH_HP_CODE_FLASH_BLOCK_WR_SIZE;
                }
            }
            else
            {
                total_bytes_to_write = flash_handler_get_write_size(size, FLASH_HP_CODE_FLASH_BLOCK_WR_SIZE);
                normal_bytes_to_write = (flash_handler_get_blocks_number(size, FLASH_HP_CODE_FLASH_BLOCK_WR_SIZE) * FLASH_HP_CODE_FLASH_BLOCK_WR_SIZE);

                __disable_irq();
                R_FLASH_HP_Open(&g_flash0_ctrl, &g_flash0_cfg);
                flash_handler_wait();
                error = R_FLASH_HP_Write(&g_flash0_ctrl, (uint32_t)data, address, normal_bytes_to_write);    /* data is passed as address value */
                R_FLASH_HP_Close(&g_flash0_ctrl);
                __enable_irq();

                if ((total_bytes_to_write != normal_bytes_to_write) &&  /* need to write some dummy stuff */
                        (error == FSP_SUCCESS))
                {
                    uint8_t dummy_block[FLASH_HP_CODE_FLASH_BLOCK_WR_SIZE] = {0};

                    for(uint16_t i = 0; i < (total_bytes_to_write - normal_bytes_to_write); i++)
                    {
                        dummy_block[i] = data[normal_bytes_to_write +i];  /* copy whats left */
                    }

                    address += normal_bytes_to_write; /* address now is moved */
                    /* we need to write an addition block*/
                    __disable_irq();
                    R_FLASH_HP_Open(&g_flash0_ctrl, &g_flash0_cfg);
                    flash_handler_wait();
                    error = R_FLASH_HP_Write(&g_flash0_ctrl, (uint32_t)dummy_block, address, FLASH_HP_CODE_FLASH_BLOCK_WR_SIZE);    /* data is passed as address value */
                    R_FLASH_HP_Close(&g_flash0_ctrl);
                    __enable_irq();

                    if (error == FSP_SUCCESS)
                    {
                        normal_bytes_to_write += FLASH_HP_CODE_FLASH_BLOCK_WR_SIZE;
                    }
                }

                if (error == FSP_SUCCESS)
                {
                    bytes_written = normal_bytes_to_write;
                }
            }
        }
        break;
        case e_flash_data:
        {
            /*
             * check address range
             * size is multiple of FLASH_MIN_PGM_SIZE_CF ?
             */
            if (size < FLASH_HP_DATA_FLASH_BLOCK_WR_SIZE)
            {
                uint8_t dummy_block[FLASH_HP_DATA_FLASH_BLOCK_WR_SIZE] = {0};

                for(uint16_t i = 0;  i < size ; i++)
                {
                    dummy_block[i] = data[i];  /* copy whats left */
                }

                R_FLASH_HP_Open(&g_flash0_ctrl, &g_flash0_cfg);
                flash_handler_wait();
                error = R_FLASH_HP_Write(&g_flash0_ctrl, (uint32_t)dummy_block, address, FLASH_HP_DATA_FLASH_BLOCK_WR_SIZE);    /* data is passed as address value */
                R_FLASH_HP_Close(&g_flash0_ctrl);

                if (error == FSP_SUCCESS)
                {
                    bytes_written = FLASH_HP_DATA_FLASH_BLOCK_WR_SIZE;
                }
            }
            else
            {
                total_bytes_to_write = flash_handler_get_write_size(size, FLASH_HP_DATA_FLASH_BLOCK_WR_SIZE);
                normal_bytes_to_write = (flash_handler_get_blocks_number(size, FLASH_HP_DATA_FLASH_BLOCK_WR_SIZE) * FLASH_HP_DATA_FLASH_BLOCK_WR_SIZE);

                R_FLASH_HP_Open(&g_flash0_ctrl, &g_flash0_cfg);
                flash_handler_wait();
                error = R_FLASH_HP_Write(&g_flash0_ctrl, (uint32_t)data, address, normal_bytes_to_write);    /* data is passed as address value */
                R_FLASH_HP_Close(&g_flash0_ctrl);

                if ((total_bytes_to_write != normal_bytes_to_write) &&
                        (error == FSP_SUCCESS))                           /* need to write some dummy stuff */
                {
                    uint8_t dummy_block[FLASH_HP_DATA_FLASH_BLOCK_WR_SIZE] = {0};

                    for(uint16_t i = 0; i < (total_bytes_to_write - normal_bytes_to_write); i++)
                    {
                        dummy_block[i] = data[normal_bytes_to_write +i];  /* copy whats left */
                    }

                    address += normal_bytes_to_write; /* address now is moved */
                    /* we need to write an addition block*/
                    R_FLASH_HP_Open(&g_flash0_ctrl, &g_flash0_cfg);
                    flash_handler_wait();
                    error = R_FLASH_HP_Write(&g_flash0_ctrl, (uint32_t)dummy_block, address, FLASH_HP_DATA_FLASH_BLOCK_WR_SIZE);    /* data is passed as address value */
                    R_FLASH_HP_Close(&g_flash0_ctrl);

                    if (error == FSP_SUCCESS)
                    {
                        normal_bytes_to_write += FLASH_HP_CODE_FLASH_BLOCK_WR_SIZE;
                    }
                }

                if (error == FSP_SUCCESS)
                {
                    bytes_written = normal_bytes_to_write;
                }

            }
        }
        break;
        default:
        case e_flash_max:
        {

        }
        break;
    }

    R_FLASH_HP_Close(&g_flash0_ctrl);

    return bytes_written;
}

/**
 *
 * @param desired_size
 * @param write_size
 * @return
 */
uint32_t flash_handler_get_write_size(uint32_t desired_size, uint32_t write_size)
{
    if (desired_size < write_size)
    {
        return write_size;
    }
    return (desired_size % write_size == 0) ? desired_size : (desired_size + 1);
}

/**
 * @brief Calculate the number of block that can be written
 *
 * @param desired_size
 * @param block_size
 * @return
 */
uint32_t flash_handler_get_blocks_number(uint32_t desired_size, uint32_t block_size)
{
    return (desired_size/block_size);   /*doesn't round up */
}

/**
 *
 * @param desired_size
 * @param write_size
 * @return
 */
static uint32_t flash_handler_get_clear_size(uint32_t desired_size, uint32_t write_size)
{
    if (desired_size < write_size)
    {
        return 1u;
    }
    else
    {
        return (desired_size % write_size == 0) ? desired_size/write_size : (desired_size/write_size + 1);
    }
}

/**
 *
 */
static void flash_handler_wait(void)
{
    fsp_err_t error = FSP_SUCCESS;
    flash_status_t status;
    do
    {
        error = R_FLASH_HP_StatusGet(&g_flash0_ctrl, &status);
    } while ((FSP_SUCCESS == error) && (FLASH_STATUS_BUSY == status));
}
