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
