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

#include "ei_environmental.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"

/* TODO: Enable if you want to open HS300X */
#define G_HS300X_SENSOR0_NON_BLOCKING (1)

/* TODO: Enable if you want to use a callback */
#define G_HS300X_SENSOR0_CALLBACK_ENABLE (1)

#if G_HS300X_SENSOR0_NON_BLOCKING
volatile bool g_hs300x_completed = false;
#endif

#if RM_HS300X_CFG_PROGRAMMING_MODE
uint32_t g_hs300x_sensor_id;
#endif

static float env_data[ENVIRONMENTAL_SENSOR_NUMBER];

/* Quick setup for g_hs300x_sensor0.
 * - g_comms_i2c_bus0 must be setup before calling this function
 *     (See Developer Assistance -> g_hs300x_sensor0 -> g_comms_i2c_device2 -> g_comms_i2c_bus0 -> Quick Setup).
 */
static void g_hs300x_sensor0_quick_setup(void);
static void g_hs300x_sensor0_quick_getting_humidity_and_temperature(rm_hs300x_data_t * p_data);

/**
 *
 * @return
 */
bool ei_environmental_sensor_init(void)
{
    g_hs300x_sensor0_quick_setup();

    if(ei_add_sensor_to_fusion_list(environment_sensor) == false) {
        ei_printf("ERR: failed to register Environmental sensor!\r\n");
        return false;
    }

    return true;
}

/**
 *
 * @param n_samples
 * @return
 */
float* ei_fusion_environment_sensor_read_data(int n_samples)
{
    rm_hs300x_data_t local_ev_data;

    (void)n_samples;

    g_hs300x_sensor0_quick_getting_humidity_and_temperature(&local_ev_data);

    env_data[0] = local_ev_data.temperature.integer_part + local_ev_data.temperature.decimal_part*0.001f;
    env_data[1] = local_ev_data.humidity.integer_part + local_ev_data.humidity.decimal_part*0.001f;

    return env_data;
}

#if G_HS300X_SENSOR0_CALLBACK_ENABLE
void hs300x_callback(rm_hs300x_callback_args_t * p_args)
{
#if G_HS300X_SENSOR0_NON_BLOCKING
    if (RM_HS300X_EVENT_SUCCESS == p_args->event)
    {
        g_hs300x_completed = true;
    }
#else
    FSP_PARAMETER_NOT_USED(p_args);
#endif
}
#endif

/* Quick setup for g_hs300x_sensor0. */
static void g_hs300x_sensor0_quick_setup(void)
{
    fsp_err_t err;

    /* Open HS300X sensor instance, this must be done before calling any HS300X API */
    err = g_hs300x_sensor0.p_api->open(g_hs300x_sensor0.p_ctrl, g_hs300x_sensor0.p_cfg);
    assert(FSP_SUCCESS == err);

#if RM_HS300X_CFG_PROGRAMMING_MODE
    /* Enter the programming mode. This must be called within 10ms after applying power. */
    err = g_hs300x_sensor0.p_api->programmingModeEnter(g_hs300x_sensor0.p_ctrl);
    assert(FSP_SUCCESS == err);

#if G_HS300X_SENSOR0_NON_BLOCKING
    while (!g_hs300x_completed)
    {
        ;
    }
    g_hs300x_completed = false;
#endif

    /* Delay 120us. Entering the programming mode takes 120us. */
    R_BSP_SoftwareDelay(120, BSP_DELAY_UNITS_MICROSECONDS);

    /* Get the sensor ID */
    err = g_hs300x_sensor0.p_api->sensorIdGet(g_hs300x_sensor0.p_ctrl, (uint32_t *)&g_hs300x_sensor_id);
    assert(FSP_SUCCESS == err);

    /* Change the humidity resolution to 8 bit */
    err = g_hs300x_sensor0.p_api->resolutionChange(g_hs300x_sensor0.p_ctrl, RM_HS300X_HUMIDITY_DATA, RM_HS300X_RESOLUTION_8BIT);
    assert(FSP_SUCCESS == err);

#if G_HS300X_SENSOR0_NON_BLOCKING
    while (!g_hs300x_completed)
    {
        ;
    }
    g_hs300x_completed = false;
#endif

    /* Delay 14ms. Failure to comply with these times may result in data corruption and introduce errors in sensor measurements. */
    R_BSP_SoftwareDelay(14, BSP_DELAY_UNITS_MILLISECONDS);

    /* Change the temperature resolution to 8 bit */
    err = g_hs300x_sensor0.p_api->resolutionChange(g_hs300x_sensor0.p_ctrl, RM_HS300X_TEMPERATURE_DATA, RM_HS300X_RESOLUTION_8BIT);
    assert(FSP_SUCCESS == err);

#if G_HS300X_SENSOR0_NON_BLOCKING
    while (!g_hs300x_completed)
    {
        ;
    }
    g_hs300x_completed = false;
#endif

    /* Delay 14ms. Failure to comply with these times may result in data corruption and introduce errors in sensor measurements. */
    R_BSP_SoftwareDelay(14, BSP_DELAY_UNITS_MILLISECONDS);

    /* Exit the programming mode */
    err = g_hs300x_sensor0.p_api->programmingModeExit(g_hs300x_sensor0.p_ctrl);
    assert(FSP_SUCCESS == err);

#if G_HS300X_SENSOR0_NON_BLOCKING
    while (!g_hs300x_completed)
    {
        ;
    }
    g_hs300x_completed = false;
#endif
#endif
}

/*  */
/**
 * @brief Quick getting humidity and temperature for g_hs300x_sensor0.
 *
 * @param p_data
 */
static void g_hs300x_sensor0_quick_getting_humidity_and_temperature(rm_hs300x_data_t * p_data)
{
    fsp_err_t            err;
    rm_hs300x_raw_data_t hs300x_raw_data;
    bool is_valid_data = false;

    /* Start the measurement */
    err = g_hs300x_sensor0.p_api->measurementStart(g_hs300x_sensor0.p_ctrl);
    assert(FSP_SUCCESS == err);
#if G_HS300X_SENSOR0_NON_BLOCKING
    while (!g_hs300x_completed)
    {
        ;
    }
    g_hs300x_completed = false;
#endif

    do
    {
        /* Read ADC data from HS300X sensor */
        err = g_hs300x_sensor0.p_api->read(g_hs300x_sensor0.p_ctrl, &hs300x_raw_data);
        assert(FSP_SUCCESS == err);
#if G_HS300X_SENSOR0_NON_BLOCKING
        while (!g_hs300x_completed)
        {
            ;
        }
        g_hs300x_completed = false;
#endif

        /* Calculate humidity and temperature values from ADC data */
        err = g_hs300x_sensor0.p_api->dataCalculate(g_hs300x_sensor0.p_ctrl, &hs300x_raw_data, p_data);
        if (FSP_SUCCESS == err)
        {
            is_valid_data = true;
        }
        else if (FSP_ERR_SENSOR_INVALID_DATA == err)
        {
            is_valid_data = false;
        }
        else
        {
            assert(false);
        }
    }
    while (false == is_valid_data);

    /* Wait 4 seconds. See table 4 on the page 6 of the datasheet. */
    //R_BSP_SoftwareDelay(4, BSP_DELAY_UNITS_SECONDS);
    // will be handled by sampler thread
}
