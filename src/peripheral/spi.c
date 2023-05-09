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
#include "peripheral/spi.h"

/* define and const */
#define MAX_TRANSFER_SIZE                   (256u)

/* variables */
static volatile bool g_transfer_complete;

/**
 *
 * @return
 */
int ei_spi_init(void)
{
    fsp_err_t err     = FSP_SUCCESS;
    g_transfer_complete = false;

    // CS high
    R_BSP_PinAccessEnable();
    R_BSP_PinWrite(CS_ACC_SPI0, BSP_IO_LEVEL_HIGH);
    R_BSP_PinWrite(CS_GYR_SPI0, BSP_IO_LEVEL_HIGH);
    R_BSP_PinWrite(CS_MAG_SPI0, BSP_IO_LEVEL_HIGH);
    R_BSP_PinAccessDisable();

    err = R_SPI_Open(&g_spi0_ctrl, &g_spi0_cfg);

    return (int)err;
}

/**
 *
 * @return
 */
int ei_spi_deinit(void)
{
    fsp_err_t err     = FSP_SUCCESS;

    R_SPI_Close(&g_spi0_ctrl);

    return (int)err;
}

/**
 *
 * @param p_args
 */
void spi_callback(spi_callback_args_t * p_args)
{
    if (SPI_EVENT_TRANSFER_COMPLETE == p_args->event)
    {
        g_transfer_complete = true;
    }
    else
    {
        // error
    }
}

/**
 *
 * @param command
 * @param transfer_size
 */
void spi_write(uint32_t cs_pin, uint8_t tx_register, uint8_t value)
{
    fsp_err_t err = FSP_SUCCESS;
    uint8_t tx_buffer[MAX_TRANSFER_SIZE] = {0};

    tx_buffer[0] = tx_register;
    tx_buffer[1] = value;
    g_transfer_complete = false;

    R_BSP_PinAccessEnable();
    R_BSP_PinWrite(cs_pin, BSP_IO_LEVEL_LOW);
    R_BSP_PinAccessDisable();

    err = R_SPI_Write(&g_spi0_ctrl, tx_buffer, (2), SPI_BIT_WIDTH_8_BITS);

    if (err != FSP_SUCCESS){

    }

    /* Wait for SPI_EVENT_TRANSFER_COMPLETE callback event. */
    while (false == g_transfer_complete)
    {
        ;
    }

    R_BSP_PinAccessEnable();
    R_BSP_PinWrite(cs_pin, BSP_IO_LEVEL_HIGH);
    R_BSP_PinAccessDisable();
}

/**
 *
 * @param command
 * @param data
 * @param transfer_size
 */
void spi_read(uint32_t cs_pin, uint8_t rx_register, uint8_t* data, uint32_t transfer_size)
{
    fsp_err_t err = FSP_SUCCESS;
    uint8_t tx_buffer[MAX_TRANSFER_SIZE] = {0};
    uint8_t rx_buffer[MAX_TRANSFER_SIZE] = {0};

    g_transfer_complete = false;
    tx_buffer[0] = rx_register;  // Read flag

    R_BSP_PinAccessEnable();
    R_BSP_PinWrite(cs_pin, BSP_IO_LEVEL_LOW);
    R_BSP_PinAccessDisable();

    err = R_SPI_WriteRead(&g_spi0_ctrl, tx_buffer, rx_buffer, (transfer_size + 1), SPI_BIT_WIDTH_8_BITS);
    assert(FSP_SUCCESS == err);
    /* Wait for SPI_EVENT_TRANSFER_COMPLETE callback event. */
    while (false == g_transfer_complete)
    {
        ;
    }

    if (err != FSP_SUCCESS){

    }

    memcpy(data, &rx_buffer[1], transfer_size);

    R_BSP_PinAccessEnable();
    R_BSP_PinWrite(cs_pin, BSP_IO_LEVEL_HIGH);
    R_BSP_PinAccessDisable();
}
