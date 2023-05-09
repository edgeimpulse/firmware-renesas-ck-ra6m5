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

#ifndef INGESTION_SDK_PLATFORM_SENSORS_EI_BAROMETRIC_SENSOR_H_
#define INGESTION_SDK_PLATFORM_SENSORS_EI_BAROMETRIC_SENSOR_H_

/* Include ----------------------------------------------------------------- */
#include "common_utils.h"
#include "fsp_common_api.h"
#include "firmware-sdk/ei_fusion.h"

FSP_CPP_HEADER

/* Function prototypes ----------------------------------------------------- */
extern bool ei_barometric_sensor_init(void);
extern float* ei_barometric_sensor_read_data(int n_samples);

static const ei_device_fusion_sensor_t barometric_sensor = {
    // name of sensor module to be displayed in fusion list
    "Barometric",
    // number of sensor module axis
    1,
    // sampling frequencies
    { 5.0f },
    // axis name and units payload (must be same order as read in)
    { {"pressure", "kPa"}, },
    // reference to read data function
    &ei_barometric_sensor_read_data,
    0
};

FSP_CPP_FOOTER



#endif /* INGESTION_SDK_PLATFORM_SENSORS_EI_BAROMETRIC_SENSOR_H_ */
