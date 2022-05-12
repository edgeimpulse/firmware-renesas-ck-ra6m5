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

#include "ei_air_quality_indoor.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"

/* TODO: Enable if you want to open ZMOD4XXX */
#define G_ZMOD4XXX_SENSOR0_NON_BLOCKING (1)
#define G_ZMOD4XXX_SENSOR0_IRQ_ENABLE   (0)

/* TODO: Enable if you want to use a I2C callback */
#define G_ZMOD4XXX_SENSOR0_I2C_CALLBACK_ENABLE (1)

/* TODO: Enable if you want to use a IRQ callback */
#define G_ZMOD4XXX_SENSOR0_IRQ_CALLBACK_ENABLE (0)

#if G_ZMOD4XXX_SENSOR0_NON_BLOCKING
volatile bool g_zmod4410_iaq_i2c_completed = false;
#endif
#if G_ZMOD4XXX_SENSOR0_IRQ_ENABLE
volatile bool g_zmod4xxx_irq_completed = false;
#endif

static rm_zmod4xxx_iaq_2nd_data_t iaq_data;
static bool _is_warmup = false;

/* Quick setup for g_zmod4xxx_sensor0.
 * - g_comms_i2c_bus0 must be setup before calling this function
 *     (See Developer Assistance -> g_zmod4xxx_sensor0 -> ZMOD4xxx ***** on rm_zmod4xxx -> g_comms_i2c_device0 -> g_comms_i2c_bus0 -> Quick Setup).
 */
static void g_zmod4410_iaq_sensor_quick_setup(void);
/* Quick getting IAQ 2nd Gen. values for g_zmod4xxx_sensor0.
 * - g_zmod4xxx_sensor0 must be setup before calling this function.
 */
static void g_zmod4410_iaq_sensor_quick_getting_iaq_2nd_gen_data(rm_zmod4xxx_iaq_2nd_data_t * p_gas_data);
static void ei_initial_warmup(void);

/**
 *
 * @return
 */
bool ei_air_quality_indoor_sensor_init(void)
{
    g_zmod4410_iaq_sensor_quick_setup();
    //ei_initial_warmup();
    _is_warmup = false;

#if EI_AIR_QUALITY_INDOOR_ENABLED == 1
    if(ei_add_sensor_to_fusion_list(indoor_air_quality_sensor) == false) {
        ei_printf("ERR: failed to register Indoor air quality sensor!\r\n");
        return false;
    }
#endif

    return true;
}

/**
 *
 * @param n_samples
 * @return
 */
float* ei_fusion_iaq_sensor_read_data(int n_samples)
{
    (void)n_samples;

    g_zmod4410_iaq_sensor_quick_getting_iaq_2nd_gen_data(&iaq_data);
    _is_warmup = true;
    /*
     * TODO
     * ret value !
     */
    return &iaq_data.iaq;
}

/**
 *
 */
void ei_air_quality_indoor_warmup(void)
{
    rm_zmod4xxx_iaq_2nd_data_t gas_data;

    g_zmod4410_iaq_sensor_quick_getting_iaq_2nd_gen_data(&gas_data);
    _is_warmup = true;
    ei_printf("Warmup for indoor quality of air completed.\n");
}

/**
 *
 * @return
 */
bool ei_air_quality_indoor_is_warmup(void)
{
    return _is_warmup;
}

/**
 *
 */
static void ei_initial_warmup(void)
{
    rm_zmod4xxx_iaq_2nd_data_t gas_data;
    fsp_err_t            err;
    rm_zmod4xxx_raw_data_t zmod4xxx_raw_data;

    g_zmod4410_iaq_i2c_completed = false;


    //if (in_warmup == false)
    {
        /* Start the measurement */
        err = g_zmod4410_iaq_sensor.p_api->measurementStart(g_zmod4410_iaq_sensor.p_ctrl);
        assert(FSP_SUCCESS == err);
#if G_ZMOD4XXX_SENSOR0_NON_BLOCKING
        while (!g_zmod4410_iaq_i2c_completed)
        {
            ;
        }
        g_zmod4410_iaq_i2c_completed = false;
#endif



        do
        {
            /* Wait for the measurement to complete */
#if G_ZMOD4XXX_SENSOR0_IRQ_ENABLE
            while (!g_zmod4xxx_irq_completed)
            {
                ;
            }
            g_zmod4xxx_irq_completed = false;
#else
            err = g_zmod4410_iaq_sensor.p_api->statusCheck(g_zmod4410_iaq_sensor.p_ctrl);
            assert(FSP_SUCCESS == err);
#if G_ZMOD4XXX_SENSOR0_NON_BLOCKING
            while (!g_zmod4410_iaq_i2c_completed)
            {
                ;
            }
            g_zmod4410_iaq_i2c_completed = false;
#endif
#endif
            /* Read ADC data from ZMOD4xxx sensor */
            err = g_zmod4410_iaq_sensor.p_api->read(g_zmod4410_iaq_sensor.p_ctrl, &zmod4xxx_raw_data);
            if (err == FSP_ERR_SENSOR_MEASUREMENT_NOT_FINISHED)
            {
                R_BSP_SoftwareDelay(50, BSP_DELAY_UNITS_MILLISECONDS);
            }
        }
        while (err == FSP_ERR_SENSOR_MEASUREMENT_NOT_FINISHED);
        assert(FSP_SUCCESS == err);

#if G_ZMOD4XXX_SENSOR0_NON_BLOCKING
        while (!g_zmod4410_iaq_i2c_completed)
        {
            ;
        }
        g_zmod4410_iaq_i2c_completed = false;
#endif
    }
    /* Calculate IAQ 2nd Gen. values from ZMOD4xxx ADC data */
    err = g_zmod4410_iaq_sensor.p_api->iaq2ndGenDataCalculate(g_zmod4410_iaq_sensor.p_ctrl, &zmod4xxx_raw_data, &gas_data);
    if (err == FSP_SUCCESS)
    {
    }
    else if(err == FSP_ERR_SENSOR_IN_STABILIZATION)
    {
        /* Delay required time. See Table 4 in the ZMOD4410 Programming Manual. */
        R_BSP_SoftwareDelay(1990, BSP_DELAY_UNITS_MILLISECONDS);
    }
    else
    {
        assert(false);
    }
    //in_warmup = false;
}

/**
 * @brief Quick getting gas data for g_zmod4410_iaq_sensor.
 *
 * @param p_gas_data
 */
static void g_zmod4410_iaq_sensor_quick_getting_iaq_2nd_gen_data(rm_zmod4xxx_iaq_2nd_data_t * p_gas_data)
{
    fsp_err_t            err;
    rm_zmod4xxx_raw_data_t zmod4xxx_raw_data;
    bool in_stabilization = false;

    /* Clear callback flags */
#if G_ZMOD4XXX_SENSOR0_IRQ_ENABLE
    g_zmod4xxx_irq_completed = false;
#endif
#if G_ZMOD4XXX_SENSOR0_NON_BLOCKING
    g_zmod4410_iaq_i2c_completed = false;
#endif

    do
    {
        /* Start the measurement */
        err = g_zmod4410_iaq_sensor.p_api->measurementStart(g_zmod4410_iaq_sensor.p_ctrl);
        assert(FSP_SUCCESS == err);
#if G_ZMOD4XXX_SENSOR0_NON_BLOCKING
        while (!g_zmod4410_iaq_i2c_completed)
        {
            ;
        }
        g_zmod4410_iaq_i2c_completed = false;
#endif

        do
        {
            /* Wait for the measurement to complete */
#if G_ZMOD4XXX_SENSOR0_IRQ_ENABLE
            while (!g_zmod4xxx_irq_completed)
            {
                ;
            }
            g_zmod4xxx_irq_completed = false;
#else
            err = g_zmod4410_iaq_sensor.p_api->statusCheck(g_zmod4410_iaq_sensor.p_ctrl);
            assert(FSP_SUCCESS == err);
#if G_ZMOD4XXX_SENSOR0_NON_BLOCKING
            while (!g_zmod4410_iaq_i2c_completed)
            {
                ;
            }
            g_zmod4410_iaq_i2c_completed = false;
#endif
#endif
            /* Read ADC data from ZMOD4xxx sensor */
            err = g_zmod4410_iaq_sensor.p_api->read(g_zmod4410_iaq_sensor.p_ctrl, &zmod4xxx_raw_data);
            if (err == FSP_ERR_SENSOR_MEASUREMENT_NOT_FINISHED)
            {
                R_BSP_SoftwareDelay(50, BSP_DELAY_UNITS_MILLISECONDS);
            }
        }
        while (err == FSP_ERR_SENSOR_MEASUREMENT_NOT_FINISHED);
        assert(FSP_SUCCESS == err);

#if G_ZMOD4XXX_SENSOR0_NON_BLOCKING
        while (!g_zmod4410_iaq_i2c_completed)
        {
            ;
        }
        g_zmod4410_iaq_i2c_completed = false;
#endif

        /* Calculate IAQ 2nd Gen. values from ZMOD4xxx ADC data */
        err = g_zmod4410_iaq_sensor.p_api->iaq2ndGenDataCalculate(g_zmod4410_iaq_sensor.p_ctrl, &zmod4xxx_raw_data, p_gas_data);
        if (err == FSP_SUCCESS)
        {
            in_stabilization = false;
        }
        else if(err == FSP_ERR_SENSOR_IN_STABILIZATION)
        {
            in_stabilization = true;
            /* Delay required time. See Table 4 in the ZMOD4410 Programming Manual. */
            R_BSP_SoftwareDelay(1990, BSP_DELAY_UNITS_MILLISECONDS);
        }
        else
        {
            assert(false);
        }

    }
    while (true == in_stabilization);
}

#if G_ZMOD4XXX_SENSOR0_I2C_CALLBACK_ENABLE
void zmod4410_iaq_comms_i2c_callback(rm_zmod4xxx_callback_args_t * p_args)
{
#if G_ZMOD4XXX_SENSOR0_NON_BLOCKING
    if (RM_ZMOD4XXX_EVENT_ERROR != p_args->event)
    {
        g_zmod4410_iaq_i2c_completed = true;
    }
#else
    FSP_PARAMETER_NOT_USED(p_args);
#endif
}
#endif

#if G_ZMOD4XXX_SENSOR0_IRQ_CALLBACK_ENABLE
void zmod4xxx_irq_callback(rm_zmod4xxx_callback_args_t * p_args)
{
#if G_ZMOD4XXX_SENSOR0_IRQ_ENABLE
    if (RM_ZMOD4XXX_EVENT_MEASUREMENT_COMPLETE == p_args->event)
    {
        g_zmod4xxx_irq_completed = true;
    }
#else
    FSP_PARAMETER_NOT_USED(p_args);
#endif
}
#endif

/**
 * @brief Quick setup for g_zmod4410_iaq_sensor.
 */
static void g_zmod4410_iaq_sensor_quick_setup(void)
{
    fsp_err_t err;

    /* Open ZMOD4XXX sensor instance, this must be done before calling any ZMOD4XXX API */
    err = g_zmod4410_iaq_sensor.p_api->open(g_zmod4410_iaq_sensor.p_ctrl, g_zmod4410_iaq_sensor.p_cfg);
    assert(FSP_SUCCESS == err);
}
