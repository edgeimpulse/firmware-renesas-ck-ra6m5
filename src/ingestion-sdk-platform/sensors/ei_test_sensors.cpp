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

#include "ei_test_sensors.h"
#include "ei_environmental.h"
#include "ei_air_quality_indoor.h"
#include "ei_air_quality_outdoor.h"
#include "ei_barometric_sensor.h"
#include "ei_microphone.h"
#include "ei_inertial_sensor.h"


/* for printf */
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"

/**
 *
 */
void ei_test_sensors(void)
{
    bool test_sensors = true;
    uint32_t start_time = 0;
    uint32_t stop_time = 0;

    ei_printf("Ok let's test sensor\r\n");
#if EI_TEST_INERTIAL == 1    
    ei_inertial_read_acc_test();
#endif
#if EI_TEST_MIC == 1
    ei_mic_test();
#endif

    while(test_sensors == true)
    {
        float* pread_value;
#if EI_TEST_TEMP_HUM_TEST == 1
        start_time = ei_read_timer_ms();
        stop_time = ei_read_timer_ms();

        pread_value = ei_fusion_environment_sensor_read_data(1u);
        ei_printf("Reading took %d\r\n", (stop_time-start_time));
        ei_printf("Temperature: ");
        ei_printf_float(pread_value[0]);

        ei_printf("\r\nRel humidity: ");
        ei_printf_float(pread_value[1]);
        ei_printf("\r\n");
        ei_sleep(500);
#endif

#if EI_TEST_INDOOR_AQS == 1
        start_time = ei_read_timer_ms();
        pread_value = ei_fusion_iaq_sensor_read_data(1);
        stop_time = ei_read_timer_ms();

        ei_printf("Reading took %d\r\n", (stop_time-start_time));
        ei_printf("Iaq: ");
        ei_printf_float(pread_value[0]);
        ei_printf("\r\n");
        ei_sleep(500);
#endif

#if EI_TEST_OUTDOOR_AQS == 1
        start_time = ei_read_timer_ms();
        pread_value = ei_fusion_oaq_sensor_read_data(1u);
        stop_time = ei_read_timer_ms();

        ei_printf("Reading took %d\r\n", (stop_time-start_time));
        stop_time = ei_read_timer_ms();
        ei_printf("oaq: ");
        ei_printf_float(pread_value[0]);
        ei_printf("\r\n");

        ei_sleep(500);
#endif
#if EI_TEST_BAROMETRIC == 1
        pread_value = ei_barometric_sensor_read_data(1);

        ei_printf("Pressure: ");
        ei_printf_float(pread_value[0]);
        ei_printf("kPa\r\n");

        ei_printf("Temp: ");
        ei_printf_float(pread_value[1]);
        ei_printf("°C\r\n");
        ei_sleep(500);
#endif
    }

}

