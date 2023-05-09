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

#ifndef INGESTION_SDK_PLATFORM_SENSORS_EI_ENVIRONMENTAL_H_
#define INGESTION_SDK_PLATFORM_SENSORS_EI_ENVIRONMENTAL_H_

/* Include ----------------------------------------------------------------- */
#include "common_utils.h"
#include "fsp_common_api.h"
#include "firmware-sdk/ei_fusion.h"

FSP_CPP_HEADER

/* Function prototypes ----------------------------------------------------- */

/* Quick getting humidity and temperature values for g_hs300x_sensor0.
 * - g_hs300x_sensor0 must be setup before calling this function.
 */
extern bool ei_environmental_sensor_init(void);
extern float* ei_fusion_environment_sensor_read_data(int n_samples);

#define ENVIRONMENTAL_SENSOR_NUMBER (2u)

static const ei_device_fusion_sensor_t environment_sensor = {
    // name of sensor module to be displayed in fusion list
    "Environmental",
    // number of sensor module axis
    ENVIRONMENTAL_SENSOR_NUMBER,
    // sampling frequencies
    { 0.25f,},
    // axis name and units payload (must be same order as read in)
    { {"temperature", "degC"}, {"humidity", "%"} },
    // reference to read data function
    &ei_fusion_environment_sensor_read_data,
    0
};

FSP_CPP_FOOTER

#endif /* INGESTION_SDK_PLATFORM_SENSORS_EI_ENVIRONMENTAL_H_ */
