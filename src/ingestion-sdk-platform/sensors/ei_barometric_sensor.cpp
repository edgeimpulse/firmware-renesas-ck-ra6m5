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
/* Include ----------------------------------------------------------------- */
#include "ei_barometric_sensor.h"
#include "peripheral/i2c.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"

/* Constant ---------------------------------------------------------------- */
#define BAROMETRIC_SENSOR_ADDRESS               (0x63)

#define BAROMETRIC_REGISTER_ID                  (0xEFC8)
#define BAROMETRIC_ID_MASK                      (0b001000)
#define BAROMETRIC_REGISTER_P_FIRST_NORMAL      (0x48A3)
#define BAROMETRIC_REGISTER_T_FIRST_NORMAL      (0x6825)

#define BAROMETRIC_REGISTER_PREPARE_OTP_READ    (0xC595)
#define BAROMETRIC_REGISTER_READ_OTP            (0xC7F7)

// constants for presure calculation


/* Private variables ------------------------------------------------------- */
static float barometric_values[2u];
static float corr_factor[4u];

/* Private functions prototype ---------------------------------------------- */
static void g_comms_i2c_barometric_quick_setup(void);
static uint8_t ei_barometric_crc_calc(uint16_t data);
static bool ei_barometric_read_opt_parameter(void);

/**
 * @brief Initializes ICP 10101
 *
 * @return true if init ok, else false
 */
bool ei_barometric_sensor_init(void)
{
    uint8_t rx_buffer[3] = {0};
    g_comms_i2c_barometric_quick_setup();

    if(ei_i2c_read_word_command(&g_comms_i2c_barometric_ctrl, BAROMETRIC_REGISTER_ID, rx_buffer, 3u, I2C_NO_WAIT_BETWEEN_W_R) != 0)
    {
        ei_printf("ERR: Barometric read failed\r\n");
        return false;
    }

    if ((rx_buffer[1] & 0b111111) != BAROMETRIC_ID_MASK)
    {
        ei_printf("ERR: Barometric ID not OK\r\n");
        return false;
    }

    if (ei_barometric_read_opt_parameter() == false)
    {
        ei_printf("Error in reading OPT param for Barometric sensor\r\n");
        return false;
    }

    if(ei_add_sensor_to_fusion_list(barometric_sensor) == false) {
        ei_printf("ERR: failed to register Barometric sensor!\n");
        return false;
    }

    return true;
}

/**
 *
 * @param n_samples
 * @return
 */
float* ei_barometric_sensor_read_data(int n_samples)
{
    const float _pcal[3] = { 45000.0, 80000.0, 105000.0 };
    const float _lut_lower = 3.5 * 0x100000;    // 1<<20
    const float _lut_upper = 11.5 * 0x100000;   // 1<<20
    const float _quadr_factor = 1 / 16777216.0;
    const float _offst_factor = 2048.0;

#define BAROMETRIC_TEMPERATURE_FIRST 0

#if BAROMETRIC_TEMPERATURE_FIRST == 1

#define BAROMETRIC_PRESSURE_MMSB        (3)
#define BAROMETRIC_PRESSURE_MSB         (4)
#define BAROMETRIC_PRESSURE_LSB         (6)

#define BAROMETRIC_TEMPERATURE_MSB      (0)
#define BAROMETRIC_TEMPERATURE_LSB      (1)
#else

#define BAROMETRIC_PRESSURE_MMSB        (0)
#define BAROMETRIC_PRESSURE_MSB         (1)
#define BAROMETRIC_PRESSURE_LSB         (3)

#define BAROMETRIC_TEMPERATURE_MSB      (6)
#define BAROMETRIC_TEMPERATURE_LSB      (7)

#endif

    uint8_t rx_buffer[9u];
    int err;
    (void)n_samples;

#if BAROMETRIC_TEMPERATURE_FIRST == 1
    err = ei_i2c_read_word_command(&g_comms_i2c_barometric_ctrl, BAROMETRIC_REGISTER_T_FIRST_NORMAL, rx_buffer, sizeof(rx_buffer), 10u);
#else
    err = ei_i2c_read_word_command(&g_comms_i2c_barometric_ctrl, BAROMETRIC_REGISTER_P_FIRST_NORMAL, rx_buffer, sizeof(rx_buffer), 10u);
#endif


    if (err != 0)
    {
        barometric_values[0] = 0;
        barometric_values[1] = 0;
    }
    else
    {
        /* get values */
        uint16_t temp = (uint16_t)((rx_buffer[BAROMETRIC_TEMPERATURE_MSB] << 8) + rx_buffer[BAROMETRIC_TEMPERATURE_LSB]);
        uint32_t pressure = (uint32_t)((uint32_t)(rx_buffer[BAROMETRIC_PRESSURE_MMSB] << 16) + (uint32_t)(rx_buffer[BAROMETRIC_PRESSURE_MSB] << 8) + rx_buffer[BAROMETRIC_PRESSURE_LSB]);


        float t = (float)(temp - 32768);
        float s1 = _lut_lower + (float)(corr_factor[0] * t * t) * _quadr_factor;
        float s2 = _offst_factor * corr_factor[3] + (float)(corr_factor[1] * t * t) * _quadr_factor;
        float s3 = _lut_upper + (float)(corr_factor[2] * t * t) * _quadr_factor;
        float c = (s1 * s2 * (_pcal[0] - _pcal[1]) +
                   s2 * s3 * (_pcal[1] - _pcal[2]) +
                   s3 * s1 * (_pcal[2] - _pcal[0])) /
                  (s3 * (_pcal[0] - _pcal[1]) +
                   s1 * (_pcal[1] - _pcal[2]) +
                   s2 * (_pcal[2] - _pcal[0]));
        float a = (_pcal[0] * s1 - _pcal[1] * s2 - (_pcal[1] - _pcal[0]) * c) / (s1 - s2);
        float b = (_pcal[0] - a) * (s1 + c);

        barometric_values[0] = (a + b / (c + pressure))/1000.0f;
        barometric_values[1] = -45.f + 175.f / 65536.f * temp;
    }


    return barometric_values;
}

/* Quick setup for g_comms_i2c_barometric.
 * - g_comms_i2c_bus0 must be setup before calling this function
 *     (See Developer Assistance -> g_comms_i2c_barometric -> g_comms_i2c_bus0 -> Quick Setup).
 */

/* Quick setup for g_comms_i2c_barometric. */
static void g_comms_i2c_barometric_quick_setup(void)
{
    fsp_err_t err;

    /* Open I2C Communications device instance, this must be done before calling any COMMS_I2C API */
    err = g_comms_i2c_barometric.p_api->open(g_comms_i2c_barometric.p_ctrl, g_comms_i2c_barometric.p_cfg);
    assert(FSP_SUCCESS == err);
}

/**
 *
 * @param data
 * @param read_crc
 * @return
 */
static uint8_t ei_barometric_crc_calc(uint16_t data)
{
    uint8_t calc_crc = 0xFF;

    return (calc_crc);
}

/**
 *
 * @return
 */
static bool ei_barometric_read_opt_parameter(void)
{
    uint8_t move_pointer_cmd[5] = {0xC5, 0x95, 0x00, 0x66, 0x9C};  /* see datasheet page 19 */
    //uint8_t increment_readout_otp[] = {0xC7, 0xF7};
    uint8_t rx_buffer[3u] = {0};
    int err;

    /* setup otp read */
    err = ei_i2c_write(&g_comms_i2c_barometric_ctrl, move_pointer_cmd, sizeof(move_pointer_cmd));

    for (uint8_t i = 0; i < 4; i++)
    {
        /* readout parameters */
        err = ei_i2c_read_word_command(&g_comms_i2c_barometric_ctrl, BAROMETRIC_REGISTER_READ_OTP, rx_buffer, 3u, I2C_NO_WAIT_BETWEEN_W_R);

        if (err != 0)
        {
            corr_factor[i] = 0.0;
        }
        else
        {
            corr_factor[i] = (float)((rx_buffer[0] << 8) + (rx_buffer[1]));
        }
    }

    return (err == 0);
}
