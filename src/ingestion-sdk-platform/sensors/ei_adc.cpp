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
#include "ei_adc.h"
#include "peripheral/adc.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "hal_data.h"

static float adc_fusion_data;

/**
 *
 * @return
 */
bool ei_adc_init(void)
{
    if (adc_init() != FSP_SUCCESS) {
    	ei_printf("ERR: failed to init ADC sensor!\n");
    }
    else {
        if(ei_add_sensor_to_fusion_list(adc_sensor) == false) {
            ei_printf("ERR: failed to register ADC sensor!\n");
            return false;
        }
    }

    return true;
}

/**
 *
 * @param n_samples
 * @return
 */
float *ei_fusion_adc_read_data(int n_samples)
{
    uint16_t reading = 0;
    uint32_t err;
    (void)n_samples;

    adc_fusion_data = 0;

    if (adc_start_scan() != FSP_SUCCESS) {
    	ei_printf("ERR: error starting ADC scan!\n");
    }

    if (adc_read(&reading) != FSP_SUCCESS) {
    	ei_printf("ERR: ferror in ADC read!\n");
    }

    adc_fusion_data = ((float)(reading/4096.0f))*3.3f;

    return &adc_fusion_data;
}

/**
 * @brief test function
 */
void ei_adc_test(void)
{
    uint16_t reading = 0;
    uint32_t err = 0;

    while(1)
    {
        err = adc_start_scan();
        ei_printf("adc_start_scan: %d\n", err);

        err = adc_read(&reading);
        ei_printf("adc_read: %d\n", err);

        ei_printf("ADC: %d\n", reading);
        ei_sleep(500);
    }
}
