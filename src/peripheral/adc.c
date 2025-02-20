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

#include "adc.h"
#include "hal_data.h"

/* Flag to notify that adc scan is started, so start reading adc */
volatile bool b_ready_to_read = false;
volatile bool b_scan_ended = false;

/**
 *
 * @return
 */
uint32_t adc_init(void)
{
    fsp_err_t err = FSP_SUCCESS;

    err = R_ADC_Open (&g_adc_ctrl, &g_adc_cfg);

    return (uint32_t)err;
}

/**
 *
 * @return
 */
uint32_t adc_deinit(void)
{
    fsp_err_t err = FSP_SUCCESS;

    err = R_ADC_Close(&g_adc_ctrl);

    return (uint32_t)err;
}

/**
 *
 * @return
 */
uint32_t adc_start_scan(void)
{
    fsp_err_t err = FSP_SUCCESS;

    err = R_ADC_ScanCfg (&g_adc_ctrl, &g_adc_channel_cfg);
    /* handle error */
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    /* Start the ADC scan*/
    b_scan_ended = false;
    err = R_ADC_ScanStart (&g_adc_ctrl);
    b_ready_to_read = true;

    return (uint32_t)err;
}

/**
 *
 * @return
 */
uint32_t adc_stop_scan(void)
{
    fsp_err_t err = FSP_SUCCESS;

    // if we run in single scan, this is not needed.
    if((ADC_MODE_SINGLE_SCAN != g_adc_cfg.mode) && (true == b_ready_to_read ))
    {
        err = R_ADC_ScanStop (&g_adc_ctrl);
        /* handle error */
        if (FSP_SUCCESS != err) {
            /* ADC Failure message */
            return err;
        }

        /* reset to indicate stop reading the adc data */
        b_ready_to_read = false;
        b_scan_ended = false;
    }

    return (uint32_t)err;
}

/**
 *
 * @param adc_reading
 * @return
 */
uint32_t adc_read(uint16_t* adc_reading)
{
    fsp_err_t err = FSP_ERR_INVALID_MODE;
    //adc_status_t adc_status;

    if (b_ready_to_read == false) {
        // scan not started :(
    }
    else {
        while(!b_scan_ended) {
            // exit strategy maybe ?
        };

        err = R_ADC_Read (&g_adc_ctrl, ADC_CHANNEL_0, adc_reading);
        // i have to close ?
        if (ADC_MODE_SINGLE_SCAN == g_adc_cfg.mode) {
            b_ready_to_read = false;    // unvalidate
        }
    }

    return (uint32_t)err;
}

/**
 *
 * @param p_args
 */
void adc_callback(adc_callback_args_t *p_args)
{
    if((NULL != p_args) && (ADC_EVENT_SCAN_COMPLETE == p_args->event)) {
        b_scan_ended = true;
    }
}

