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
#include "i2s.h"
#include "peripheral/led.h"
#include "ingestion-sdk-platform/sensors/ei_dc_blocking.h"

#define DEBUG_LED                       0

typedef enum
{
    e_buffer_first_one   = 0,
    e_buffer_second_one = 1,
    e_buffer_max        = 2
}t_double_buffer;

/* Global variables */
volatile i2s_event_t g_i2s_event = I2S_EVENT_IDLE;  //an actual event updates in callback
volatile uint32_t _buffer[e_buffer_max][EI_I2S_SAMPLES_PER_READ]  BSP_ALIGN_VARIABLE(8);
volatile uint32_t* _p_buffer;
volatile t_double_buffer actual_buffer;

volatile bool _close_driver = false;
volatile bool _read_enabled = false;

#define get_read_buffer(x)  x == e_buffer_first_one ? e_buffer_second_one : e_buffer_first_one;

/**
 * @brief Initialize peripherals (ssi and timer channel 2)
 * @return
 */
int ei_i2s_driver_init(void)
{
    fsp_err_t err = FSP_SUCCESS;
    memset((void*)_buffer, 0, sizeof(_buffer));

    /* start timer */
    err = R_GPT_Open(&g_timer2_ctrl, &g_timer2_cfg);

    if (FSP_SUCCESS != err)
    {
        //
    }

    /* Open SSI module */
    err = R_SSI_Open(&g_i2s0_ctrl, &g_i2s0_cfg);
    if (FSP_SUCCESS != err)
    {
        //
    }
    //R_SSI0->SSICR |= R_SSI0_SSICR_BCKP_Msk; /* set bit clock polarity */
    g_i2s0_ctrl.p_reg->SSICR_b.PDTA = 0;   /* left alignment! */

    /* start timer */
    R_GPT_Start(&g_timer2_ctrl);
    _read_enabled = false;

    actual_buffer = e_buffer_first_one;
    _p_buffer = _buffer[actual_buffer];

    /* Handle error */
    if (FSP_SUCCESS != err)
    {
        /*
         * print error
         */
    }

    return (int)err;
}

/**
 *
 * @return
 */
int ei_i2s_init(void)
{
    fsp_err_t err = FSP_SUCCESS;
    actual_buffer = e_buffer_first_one;

    memset((void*)_buffer, 0, sizeof(_buffer));
    _p_buffer = _buffer[actual_buffer];

    /* */
    err = R_SSI_Read(&g_i2s0_ctrl, _p_buffer, EI_I2S_READ_SAMPLES_BYTE);    /* start reading */
    if (FSP_SUCCESS != err)
    {
        //
    }
    _read_enabled = true;

    return (int)err;
}

/**
 *
 * @return
 */
int ei_i2s_deinit(void)
{
    _read_enabled = false;

    while(_close_driver == false)
    {
        __NOP();
    }

    return 0;
}

/**
 *
 * @return
 */
int ei_i2s_driver_deinit(void)
{
    fsp_err_t err = FSP_SUCCESS;
    _close_driver = true;

    while(_close_driver)
    {
        __NOP();
    };

    /* Disable & close DTC */
    R_DTC_Disable(&g_transfer0_ctrl);
    R_DTC_Close(&g_transfer0_ctrl);

    /* Close SSI Module */
    err = R_SSI_Close(&g_i2s0_ctrl);

    /* Stop & close timer */
    R_GPT_Stop(&g_timer2_ctrl);
    R_GPT_Close(&g_timer2_ctrl);

    return (int)err;
}

/**
 *
 * @return
 */
i2s_event_t ei_i2s_get_status(void)
{
    return g_i2s_event;
}

/**
 *
 * @param dst
 * @param max_byte
 * @return
 */
uint16_t ei_i2s_get_buffer(int32_t* dst, uint16_t max_byte)
{
    uint16_t i;
    uint16_t valid_samples = 0;
    uint16_t invalid_samples = 0;
    t_double_buffer read_buffer = get_read_buffer(actual_buffer);

    for (i = 0; i < (max_byte) ; i++)
    {
        if (i%2 == 0)
        {
            //dst[valid_samples] = ((int32_t)_buffer[read_buffer][i]);
            if (((int32_t)_buffer[read_buffer][i]) != 0)
            {
                dst[valid_samples] = ((int32_t)_buffer[read_buffer][i]);
                valid_samples++;
            }
            else
            {
                invalid_samples++;
            }


        }

    }
    memset((void*)_buffer[read_buffer], 0, sizeof(_buffer[read_buffer]));

    return (valid_samples);
}

/**
 *
 * @param p_args
 */
void i2s_callback(i2s_callback_args_t *p_args)
{
#if DEBUG_LED == 1
    static bool led_blue = false;
    static bool led_red = false;
#endif

    if( NULL != p_args)
    {

        /* capture callback event for validating the i2s transfer event*/
        g_i2s_event = p_args->event;
        if (_read_enabled == true)
        {
            switch(g_i2s_event)
            {
                case I2S_EVENT_IDLE:
                {
                    //actual_buffer = get_write_buffer(actual_buffer);
                    //_p_buffer = _buffer[actual_buffer];
                    R_SSI_Read(&g_i2s0_ctrl, _p_buffer, EI_I2S_READ_SAMPLES_BYTE);  // not sure here

    #if DEBUG_LED == 1
                    led_red = !led_red;

                    if (led_red == true)
                    {
                        ei_led_turn_on(e_user_led_red);
                    }
                    else
                    {
                        ei_led_turn_off(e_user_led_red);
                    }
    #endif
                }
                break;
                case I2S_EVENT_RX_FULL:
                {
                    actual_buffer = get_read_buffer(actual_buffer); /* switch buffer */
                    _p_buffer = _buffer[actual_buffer];
                    R_SSI_Read(&g_i2s0_ctrl, _p_buffer, EI_I2S_READ_SAMPLES_BYTE);

    #if DEBUG_LED == 1
                    led_blue = !led_blue;

                    if (led_blue == true)
                    {
                        ei_led_turn_on(e_user_led_blue);
                    }
                    else
                    {
                        ei_led_turn_off(e_user_led_blue);
                    }
    #endif
                }
                break;
                default:
                {

                }
                break;
            }
        }
        else
        {
            _close_driver = true;
        }
    }
}
