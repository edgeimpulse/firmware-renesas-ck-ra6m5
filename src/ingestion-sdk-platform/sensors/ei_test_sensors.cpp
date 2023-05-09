/* Edge Impulse ingestion SDK
 * Copyright (c) 2022 EdgeImpulse Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
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

