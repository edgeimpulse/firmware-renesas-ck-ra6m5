/* Edge Impulse ingestion SDK
 * Copyright (c) 2023 EdgeImpulse Inc.
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

#ifndef EDGE_IMPULSE_INGESTION_SDK_PLATFORM_SENSORS_EI_ADC_H_
#define EDGE_IMPULSE_INGESTION_SDK_PLATFORM_SENSORS_EI_ADC_H_

/* Include ----------------------------------------------------------------- */
#include "firmware-sdk/ei_fusion.h"

/** Number of axis used and sample data format */
#define ADC_AXIS_SAMPLED       1
#define SIZEOF_ADC_AXIS_SAMPLED   (sizeof(float) * INERTIAL_AXIS_SAMPLED)

/* Function prototypes ----------------------------------------------------- */
bool ei_adc_init(void);
void ei_adc_test(void);

float *ei_fusion_adc_read_data(int n_samples);

static const ei_device_fusion_sensor_t adc_sensor = {
    "ADC",
    // number of sensor module axis
    ADC_AXIS_SAMPLED,
    // sampling frequencies
    { 100.0f, 500.0f, 1000.0f },
    // axis name and units payload (must be same order as read in)
    { {"adc", "mV"}},
    // reference to read data function
    &ei_fusion_adc_read_data,
    0
};

#endif /* EDGE_IMPULSE_INGESTION_SDK_PLATFORM_SENSORS_EI_ADC_H_ */
