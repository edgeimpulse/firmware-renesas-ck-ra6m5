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

