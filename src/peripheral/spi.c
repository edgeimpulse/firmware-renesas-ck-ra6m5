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
