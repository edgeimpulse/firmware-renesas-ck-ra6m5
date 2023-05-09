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

#ifndef INGESTION_SDK_PLATFORM_SENSORS_EI_INERTIAL_BMX055_SENSOR_H_
#define INGESTION_SDK_PLATFORM_SENSORS_EI_INERTIAL_BMX055_SENSOR_H_

/* Include ----------------------------------------------------------------- */
#include "firmware-sdk/ei_fusion.h"

/** Number of axis used and sample data format */
#define INERTIAL_AXIS_SAMPLED            9
#define SIZEOF_ACCEL_AXIS_SAMPLED   (sizeof(float) * INERTIAL_AXIS_SAMPLED)

/* Function prototypes ----------------------------------------------------- */
bool ei_inertial_bmx_init(void);
void ei_inertial_bmx_read_acc_test(void);

float *ei_fusion_inertial_bmx_read_data(int n_samples);

static const ei_device_fusion_sensor_t bmx_sensor = {
    "Inertial",
    // number of sensor module axis
    INERTIAL_AXIS_SAMPLED,
    // sampling frequencies
    { 20.0f, 62.5f, 100.0f },
    // axis name and units payload (must be same order as read in)
    { {"accX", "m/s2"}, {"accY", "m/s2"}, {"accZ", "m/s2"}, {"gyrX", "dps"}, {"gyrY", "dps"}, {"gyrZ", "dps"}, {"magX", "uT"}, {"magY", "uT"}, {"magZ", "uT"} },
    // reference to read data function
    &ei_fusion_inertial_bmx_read_data,
    0
};


#endif /* INGESTION_SDK_PLATFORM_SENSORS_EI_INERTIAL_BMX055_SENSOR_H_ */
