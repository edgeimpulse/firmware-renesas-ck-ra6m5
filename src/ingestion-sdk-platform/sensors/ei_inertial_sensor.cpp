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
#include "ei_inertial_sensor.h"
#include "peripheral/i2c.h"
#include "peripheral/timer_handler.h"
#include "ingestion-sdk-platform/renesas-ck-ra6m5/ei_device_renesas_ck_ra6m5.h"
#include "ingestion-sdk/ei_sampler.h"
#include "firmware-sdk/sensor_aq.h"
#include "firmware-sdk/ei_device_info_lib.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include <math.h>

/* for printf */
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"


static float inertial_fusion_data[INERTIAL_AXIS_SAMPLED];
/* Constant ---------------------------------------------------------------- */
#define INERTIAL_SENSOR_ADDRESS     (0x68)
#define INERTIAL_SENSOR_ID          (0xEA)

#define MAGNOMETER_SENSOR_ADDRESS   (0x0C)
#define MAGNOMETER_SENSOR_ID_1      (0x48)
#define MAGNOMETER_SENSOR_ID_2      (0x09)

#define CONVERT_G_TO_MS2            (9.80665f)
#define ACC_RAW_SCALING             (32767.5f)
#define INERTIAL_ACCEL_LSB_G        (16.384f)

#define ACC_SCALE_FACTOR            (2.0f*CONVERT_G_TO_MS2)/ACC_RAW_SCALING

#define GYRO_SCALE_FACTOR           (250.0f/32768.0f)

#define MAG_SCALE_FACTOR            (0.15f)

#define MAG_MANUALLY_POWER_ON       (0u)

/* Change bank register */
#define REGISTER_REG_BANK_SEL   (0x7F)
#define BANK_0                  (0)
#define BANK_1                  (1u)
#define BANK_2                  (2u)
#define BANK_3                  (3u)

/* Bank 0 register - General config and sensor read */
#define BK_0_REGISTER_WHO_AM_I       (0x00)
#define BK_0_REGISTER_USER_CTRL      (0x03)
#define BK_0_REGISTER_LP_CONFIG      (0x05)
#define BK_0_REGISTER_PWR_MGMT_1     (0x06)
#define BK_0_REGISTER_PWR_MGMT_2     (0x07)
#define BK_0_REGISTER_INT_PIN_CFG    (0x0F)

#define BK_0_REGISTER_ACCEL_XOUT_H   (0x2D)
#define BK_0_REGISTER_ACCEL_XOUT_L   (0x2E)
#define BK_0_REGISTER_ACCEL_YOUT_H   (0x2F)
#define BK_0_REGISTER_ACCEL_YOUT_L   (0x30)
#define BK_0_REGISTER_ACCEL_ZOUT_L   (0x32)

#define BK_0_REGISTER_GYRO_XOUT_H    (0x33)
#define BK_0_REGISTER_GYRO_XOUT_L    (0x34)
#define BK_0_REGISTER_GYRO_YOUT_H    (0x35)
#define BK_0_REGISTER_GYRO_YOUT_L    (0x36)
#define BK_0_REGISTER_GYRO_ZOUT_H    (0x37)
#define BK_0_REGISTER_GYRO_ZOUT_L    (0x38)

#define BK_0_REGISTER_TEMP_OUT_H     (0x39)
#define BK_0_REGISTER_TEMP_OUT_L     (0x3A)

#define BK_0_REGISTER_EXT_SLV_SENS_DATA_00     (0x3B)


/* Bank 1 register - Self test and offset */

/* Bank 2 register - config for separate sensor */
#define BK_2_REGISTER_GYRO_SMPLRT_DIV    (0x00)
#define BK_2_REGISTER_GYRO_CONFIG_1      (0x01)
#define BK_2_REGISTER_GYRO_CONFIG_2      (0x02)

#define BK_2_REGISTER_XG_OFFS_USRH       (0x03)
#define BK_2_REGISTER_XG_OFFS_USRL       (0x04)
#define BK_2_REGISTER_YG_OFFS_USRH       (0x05)
#define BK_2_REGISTER_YG_OFFS_USRL       (0x06)
#define BK_2_REGISTER_ZG_OFFS_USRH       (0x07)
#define BK_2_REGISTER_ZG_OFFS_USRL       (0x08)

#define BK_2_REGISTER_ODR_ALIGN_CONFIG      (0x09)

#define BK_2_REGISTER_ACCEL_SMPLRT_DIV_1 (0x10)
#define BK_2_REGISTER_ACCEL_SMPLRT_DIV_2 (0x11)
#define BK_2_REGISTER_ACCEL_CONFIG       (0x14)
#define BK_2_REGISTER_ACCEL_CONFIG_2     (0x15)

/* Bank 3 register - config for additional I2C slave */
#define BK_3_REGISTER_I2C_MST_ODR_CONFIG    (0x00)
#define BK_3_REGISTER_I2C_MST_CTRL          (0x01)
#define BK_3_REGISTER_I2C_SLV0_ADDR         (0x03)
#define BK_3_REGISTER_I2C_SLV0_REG          (0x04)
#define BK_3_REGISTER_I2C_SLV0_CTRL         (0x05)
#define BK_3_REGISTER_I2C_SLV0_DO           (0x06)

/* Magnetometer registers */
#define MAG_REGISTER_WIA1               (0x00)
#define MAG_REGISTER_WIA2               (0x01)
#define MAG_REGISTER_ST1                (0x10)
#define MAG_REGISTER_HXL                (0x11)
#define MAG_REGISTER_HXH                (0x12)
#define MAG_REGISTER_HYL                (0x13)
#define MAG_REGISTER_HYH                (0x14)
#define MAG_REGISTER_HZL                (0x15)
#define MAG_REGISTER_HZH                (0x16)
#define MAG_REGISTER_ST2                (0x18)
#define MAG_REGISTER_CNTL2              (0x31)
#define MAG_REGISTER_CNTL3              (0x32)

/* Private functions ------------------------------------------------ */
static bool ei_inertial_sensor_set(bool accel_on, bool gyro_on);
static bool ei_inertial_init_config(void);
static bool ei_inertial_read_accelerometer(float* acc_data);
static bool ei_inertial_read_gyro(float* gyro_data);
static bool ei_inertial_read_mag(float* mag_data);
static inline int ei_inertial_change_bank(uint8_t bank_nr);
static int ei_inertial_write_register(uint8_t reg, uint8_t data);
static int ei_inertial_write_mag_register(uint8_t reg, uint8_t data);
static int ei_inertial_read_mag_register(uint8_t reg, uint8_t* data, uint8_t n_bytes);
static bool ei_inertial_read_internal_temp(float* temp);
static void ei_gyro_offset(void);
static bool ei_inertial_init_mag(void);

/* Public functions ------------------------------------------------ */
/**
 * @brief Configure inertial sensor
 *
 * @return
 */
bool ei_inertial_init(void)
{
    g_comms_i2c_accelerometer.p_api->open(g_comms_i2c_accelerometer.p_ctrl, g_comms_i2c_accelerometer.p_cfg);

    if (ei_inertial_check_presence() == false)
    {
        ei_printf("ERR: failed to init Inertial sensor!\n");
        return false;
    }

    ei_inertial_init_config();
    ei_inertial_sensor_set(true, true);    /* turn on accelerometer and gyro */
    ei_inertial_init_mag();

    if(ei_add_sensor_to_fusion_list(inertial_sensor) == false) {
        ei_printf("ERR: failed to register Inertial sensor!\n");
        return false;
    }

    return true;
}

/**
 * @brief test function. read the WHO_AM_I register and check if equal to
 *
 * @return true if test pass, fail if not INERTIAL_SENSOR_ID
 */
bool ei_inertial_check_presence(void)
{
    bool test_ok = false;
    uint8_t rx_buffer;
    int ret_val;

    ei_inertial_change_bank(BANK_0);

    ei_inertial_write_register(BK_0_REGISTER_PWR_MGMT_1, 0x80);      /* reset */
    R_BSP_SoftwareDelay(100u, BSP_DELAY_UNITS_MILLISECONDS);    /* and wait 100ms */

    ret_val = ei_i2c_read_byte_command(&g_comms_i2c_accelerometer_ctrl, BK_0_REGISTER_WHO_AM_I, &rx_buffer, 1u, I2C_NO_WAIT_BETWEEN_W_R);

    if (ret_val == 0)
    {
        if (rx_buffer == INERTIAL_SENSOR_ID)
        {
            test_ok = true;
        }
        else
        {
            ei_printf("TEST FAILED: I2C read OK, but ID read is NOK 0x%2x\r\n", rx_buffer);
        }
    }
    else
    {
        ei_printf("TEST FAILED: I2C read NOK, error code %d\r\n", ret_val);
    }
    return test_ok;
}

/**
 * @brief Test function for ICM 20948
 *
 */
void ei_inertial_read_acc_test(void)
{
    static bool first_time = true;
    float acc_data[3] = {0};
    float gyr_data[3] = {0};
    float mag_data[3] = {0};
    float temp = 0;
    bool testing = true;

    if (first_time == true)
    {
        ei_inertial_sensor_set(true, true);    /* turn on accelerometer */
        //ei_gyro_offset();
        first_time = false;
    }

    while(testing == true)
    {
        ei_inertial_read_accelerometer(acc_data);
        ei_inertial_read_gyro(gyr_data);
        ei_inertial_read_mag(mag_data);

        ei_printf("Test read accelerometer...\r\n");
        for (uint8_t i = 0; i < 3; i++)
        {
            ei_printf("data read axis %d: ", i);
            ei_printf_float(acc_data[i]);
            ei_printf("\r\n");
        }
        ei_printf("Test done...\r\n");

        ei_printf("Test read gyro...\r\n");
        for (uint8_t i = 0; i < 3; i++)
        {
            ei_printf("data read axis %d: ", i);
            ei_printf_float(gyr_data[i]);
            ei_printf("\r\n");
        }
        ei_printf("Test done...\r\n");

        ei_printf("Test mag gyro...\r\n");
        for (uint8_t i = 0; i < 3; i++)
        {
            ei_printf("data mag axis %d: ", i);
            ei_printf_float(mag_data[i]);
            ei_printf("\r\n");
        }
        ei_printf("Test done...\r\n");

        ei_inertial_read_internal_temp(&temp);
        ei_printf("temperature: ");
        ei_printf_float(temp);
        ei_printf("\r\n");

        ei_sleep(1000);
    }

}

/**
 *
 * @param n_samples
 * @return
 */
float *ei_fusion_inertial_read_data(int n_samples)
{
    if (n_samples >= 3)
    {
        if (ei_inertial_read_accelerometer(&inertial_fusion_data[0]) == false)
        {
            ei_printf("ERR: no Accel data!\n");
            inertial_fusion_data[0u] = 0.0f;
            inertial_fusion_data[1u] = 0.0f;
            inertial_fusion_data[2u] = 0.0f;
        }
    }

    if (n_samples >= 6)
    {
        if (ei_inertial_read_gyro(&inertial_fusion_data[3u]) == false)
        {
            ei_printf("ERR: no Gyro data!\n");
            inertial_fusion_data[3u] = 0.0f;
            inertial_fusion_data[4u] = 0.0f;
            inertial_fusion_data[5u] = 0.0f;
        }
    }

    if (n_samples == 9)
    {
        if (ei_inertial_read_mag(&inertial_fusion_data[6u]) == false)
        {
            ei_printf("ERR: no Magnetometer data!\n");
            inertial_fusion_data[6u] = 0.0f;
            inertial_fusion_data[7u] = 0.0f;
            inertial_fusion_data[8u] = 0.0f;
        }
    }

    return inertial_fusion_data;
}
/* ----------------------------------------------------------------------- */
/**
 *
 * @param accel_on
 * @param gyro_on
 * @return
 */
static bool ei_inertial_sensor_set(bool accel_on, bool gyro_on)
{
    bool configured = false;
    uint8_t config = 0;

    ei_inertial_change_bank(BANK_0);

    if (accel_on | gyro_on) /* if any of the 2 on ...*/
    {
        config = 0x01;  /* wake up*/
    }
    else
    {
        config = 0x41;  /* sleep */
    }
    ei_inertial_write_register(BK_0_REGISTER_PWR_MGMT_1, config);
    R_BSP_SoftwareDelay((uint32_t)20, BSP_DELAY_UNITS_MILLISECONDS);

    config = 0; /* start from all on */
    if (accel_on == false)  /* turn off accel */
    {
        config |= 0x38;
    }

    if (gyro_on == false)   /* turn off gyro */
    {
        config |= 0x7;
    }

    if (ei_inertial_write_register(BK_0_REGISTER_PWR_MGMT_2, config) == 0)
    {
        uint8_t rx_data;
        if (ei_i2c_read_byte_command(&g_comms_i2c_accelerometer_ctrl, BK_0_REGISTER_PWR_MGMT_2, &rx_data, sizeof(rx_data), I2C_NO_WAIT_BETWEEN_W_R) == 0)
        {
            configured = true;
        }
        else
        {
            ei_printf("Inertial NOT configured\r\n");
        }
    }
    else
    {
        ei_printf("Inertial NOT configured\r\n");
    }

    return configured;
}

/**
 *
 * @return
 */
static bool ei_inertial_init_config(void)
{
    bool configured = false;
    uint8_t config = 0;

    //ei_inertial_sensor_set(false, false);   /* turn off - set bank to 0 */

#if 0
    config = 0x70;  /* accelerator in duty cycled mode and Gyro in duty cycled mode */
    if (ei_inertial_write_register(BK_0_REGISTER_LP_CONFIG, config) == 0)
    {
        ei_printf("Inertial configured\r\n");
        configured = true;
    }
    else
    {
        ei_printf("Inertial NOT configured\r\n");
    }
#endif

    ei_inertial_change_bank(BANK_2);    /* bank 2 is for config */
    ei_inertial_write_register(BK_2_REGISTER_ODR_ALIGN_CONFIG, 1u); /* enable ODR align*/

    /*
     * Gyro config:
     * - GYRO_SMPLRT_DIV
     *
     * - GYRO_CONFIG_1
     * lpf, full scale, DLPF
     *
     * - GYRO_CONFIG_2
     * selt test enabled, Averaging filter config
     *
     * OFFSET CANCELLATION
     */
    config = 5u;    /* 183 */
    ei_inertial_write_register(BK_2_REGISTER_GYRO_SMPLRT_DIV, config);
    ei_inertial_write_register(BK_2_REGISTER_ACCEL_CONFIG, (2 << 3) | 1);

    /*
     * Accel config:
     * default config is:
     * ACCEL_SMPLRT_DIV = 0
     * range +-2g
     * DLPG enabled
     * LP config 0
     * Num of avg: 0
     * Enabled DLPF : 1
     *
     * Rate: 1125 Hz
     */
    ei_inertial_write_register(BK_2_REGISTER_ACCEL_CONFIG, (2 << 3) | 1);

    config = 5;
    if (ei_inertial_write_register(BK_2_REGISTER_ACCEL_SMPLRT_DIV_2, config) == 0)
    {
        configured = true;
    }

    return configured;
}

/**
 *
 * @param acc_data
 * @return
 */
static bool ei_inertial_read_accelerometer(float* acc_data)
{
    bool read = false;
    uint8_t rx_data[6u] = {0};
    uint8_t i;

    ei_inertial_change_bank(BANK_0);

    if (acc_data != nullptr)
    {
        if (ei_i2c_read_byte_command(&g_comms_i2c_accelerometer_ctrl, BK_0_REGISTER_ACCEL_XOUT_H, &rx_data[0], sizeof(rx_data), I2C_NO_WAIT_BETWEEN_W_R) == 0)
        {
            for (i = 0; i < 3; i++)
            {
                acc_data[i] = (float)((int16_t)(rx_data[i*2] << 8) | rx_data[i*2 + 1]);
                acc_data[i] *= ACC_SCALE_FACTOR;
            }
            read = true;
        }
    }

    return read;
}

/**
 * @brief Read gyro
 *
 * @param gyro_data
 * @return
 */
static bool ei_inertial_read_gyro(float* gyro_data)
{
    bool read = false;
    uint8_t rx_data[6u] = {0};
    uint8_t i;

    ei_inertial_change_bank(BANK_0);

    if (gyro_data != nullptr)
    {
        if (ei_i2c_read_byte_command(&g_comms_i2c_accelerometer_ctrl, BK_0_REGISTER_GYRO_XOUT_H, &rx_data[0], sizeof(rx_data), I2C_NO_WAIT_BETWEEN_W_R) == 0)
        {
            for (i = 0; i < 3; i++)
            {
                gyro_data[i] = (float)( (int16_t)((rx_data[i*2] << 8) | rx_data[(i*2) + 1])) ;
                gyro_data[i] *= GYRO_SCALE_FACTOR;
            }
            read = true;
        }
    }

    return read;
}

/**
 * @brief Read from mag sensors
 *
 * @param mag_data
 * @return
 */
static bool ei_inertial_read_mag(float* mag_data)
{
    bool data_ready = false;
    bool data_ovr = false;
    bool overflow = false;
    bool read = false;
    uint8_t rx_data[9u] = {0};
    uint8_t i;

    if (mag_data != nullptr)
    {
#if MAG_MANUALLY_POWER_ON == 1
        ei_inertial_write_mag_register(MAG_REGISTER_CNTL2, (1));   /* manual trigger */
#endif
        //ei_sleep(8);    /* datasheet say measurement time is typ 7 ms */

        ei_inertial_change_bank(BANK_0);
        ei_i2c_read_byte_command(&g_comms_i2c_accelerometer_ctrl, BK_0_REGISTER_EXT_SLV_SENS_DATA_00, rx_data, 9, I2C_NO_WAIT_BETWEEN_W_R);

        data_ready = ((rx_data[0] & 1) == 1);
        data_ovr = ((rx_data[0] & 2) == 2);
        overflow = ((rx_data[8] & 1) == 1);

        //if ((data_ready == true)
        //        || (data_ovr == true))
        {
            for (i = 0; i < 3; i++)
            {
                mag_data[i] = (float)( (int16_t)(rx_data[i*2+1] | (rx_data[(i*2) + 2] << 8) )) ;
                mag_data[i] *= (MAG_SCALE_FACTOR);
            }

            if (overflow)  /* no overflow */
            {
                read = true;
            }
            else
            {
                if ( (abs(mag_data[0]) + abs(mag_data[1]) + abs(mag_data[2])) > 4312.0)     /* double check */
                {
                    ei_printf("Mag: data overflow!");
                }
                else
                {
                    read = true;
                }

            }
        }
    }

    return read;
}

/**
 *
 * @param temp
 * @return
 */
static bool ei_inertial_read_internal_temp(float* temp)
{
    bool read = false;
    uint8_t rx_data[2u] = {0};
    int16_t raw_temp = 0;

    ei_inertial_change_bank(BANK_0);

    if (temp != nullptr)
    {
        if (ei_i2c_read_byte_command(&g_comms_i2c_accelerometer_ctrl, BK_0_REGISTER_TEMP_OUT_H, &rx_data[0], sizeof(rx_data), I2C_NO_WAIT_BETWEEN_W_R) == 0)
        {
            raw_temp =  (int16_t)((rx_data[0] << 8) | rx_data[1]) ;
            *temp = 21.0f + (raw_temp/333.87f);
            read = true;
        }
    }

    return read;
}

/**
 *
 * @param bank_nr
 * @return
 */
static inline int ei_inertial_change_bank(uint8_t bank_nr)
{
    return ei_inertial_write_register(REGISTER_REG_BANK_SEL, (bank_nr << 4));
}

/**f
 * @brief Interface to write to i2c and keep the i2c interface generic.
 *
 * @return
 * @note If all connected I2C devices act the same, can be integrated inside i2c
 */
static int ei_inertial_write_register(uint8_t reg, uint8_t data)
{
    uint8_t tx_buffer[2];
    tx_buffer[0] = reg;
    tx_buffer[1] = data;

    return ei_i2c_write(&g_comms_i2c_accelerometer_ctrl, tx_buffer, 2u);
}

/**
 * @brief
 *
 * @param reg
 * @param data
 * @return
 */
static int ei_inertial_write_mag_register(uint8_t reg, uint8_t data)
{
    ei_inertial_change_bank(BANK_3);

    ei_inertial_write_register(BK_3_REGISTER_I2C_SLV0_ADDR, MAGNOMETER_SENSOR_ADDRESS);     /* write address for Mag */
    ei_inertial_write_register(BK_3_REGISTER_I2C_SLV0_REG, reg);                            /* reg for Mag */
    return ei_inertial_write_register(BK_3_REGISTER_I2C_SLV0_DO, data);                     /* write data to Mag */
}

/**
 * @brief Set up reading from slave 0
 *
 * @param reg
 * @param data
 * @param n_bytes
 * @return
 */
static int ei_inertial_read_mag_register(uint8_t reg, uint8_t* data, uint8_t n_bytes)
{
    int ret_val;

    ei_inertial_change_bank(BANK_3);

    ret_val = ei_inertial_write_register(BK_3_REGISTER_I2C_SLV0_ADDR, 0x80 + MAGNOMETER_SENSOR_ADDRESS);    /* read address for Mag */
    ret_val += ei_inertial_write_register(BK_3_REGISTER_I2C_SLV0_REG, reg);    /* reg for Mag */
    ret_val += ei_inertial_write_register(BK_3_REGISTER_I2C_SLV0_CTRL, (0x80 + (n_bytes &0x0F)));    /* reg for Mag */

    ei_sleep(1);

    //ei_inertial_change_bank(BANK_0);
    //ret_val += ei_i2c_read_byte_command(&g_comms_i2c_accelerometer_ctrl, BK_0_REGISTER_EXT_SLV_SENS_DATA_00, data, n_bytes, I2C_NO_WAIT_BETWEEN_W_R);

    return ret_val;
}

/**
 * @brief Init the magnetometer
 *
 * @return
 */
static bool ei_inertial_init_mag(void)
{
    bool set = false;
    uint8_t config_reg;
    uint8_t read_data[10] = {0};
    uint8_t retries = 10;

    /* enables master operation */
    ei_inertial_change_bank(BANK_0);

    ei_i2c_read_byte_command(&g_comms_i2c_accelerometer_ctrl, BK_0_REGISTER_USER_CTRL, &read_data[0], 1, I2C_NO_WAIT_BETWEEN_W_R);
    config_reg = read_data[0] | (2u); /* I2C MST reset */
    ei_inertial_write_register(BK_0_REGISTER_USER_CTRL, config_reg);

    ei_i2c_read_byte_command(&g_comms_i2c_accelerometer_ctrl, BK_0_REGISTER_USER_CTRL, &read_data[0], 1, I2C_NO_WAIT_BETWEEN_W_R);
    config_reg = read_data[0] | (1 << 5); /* I2C MST enabled */
    ei_inertial_write_register(BK_0_REGISTER_USER_CTRL, config_reg);


    /* set I2C master freq */
    ei_inertial_change_bank(BANK_3);
    config_reg = (7u);                /* clock freq for master circa 400 KHz */
    ei_inertial_write_register(BK_3_REGISTER_I2C_MST_CTRL, config_reg);

    ei_inertial_write_mag_register(MAG_REGISTER_CNTL3, 1u); /* soft reset */
    ei_sleep(1000u);

    ei_inertial_read_mag_register(MAG_REGISTER_WIA1, &read_data[0], 2u);   /* read ID of mag */

    do
    {
        ei_inertial_change_bank(BANK_0);
        ei_i2c_read_byte_command(&g_comms_i2c_accelerometer_ctrl, BK_0_REGISTER_EXT_SLV_SENS_DATA_00, &read_data[0], 2, I2C_NO_WAIT_BETWEEN_W_R);

        if ((read_data[0] == MAGNOMETER_SENSOR_ID_1)
                && (read_data[1] == MAGNOMETER_SENSOR_ID_2))
        {
            set = true;
    #if MAG_MANUALLY_POWER_ON == 0
            ei_inertial_write_mag_register(MAG_REGISTER_CNTL2, (1 << 3));   /* cont mode 4 */
    #endif
            ei_inertial_read_mag_register(MAG_REGISTER_ST1, &read_data[0], 9u);   /* read ID of mag */
        }
        else
        {
            ei_inertial_write_register(BK_0_REGISTER_USER_CTRL, (0x22)); /* reset I2C master */
            ei_sleep(10u);
            retries--;
        }
    }while ((set == false) && (retries > 0));



    if (set == false)
    {
        ei_printf("Unable to read from Magnetometer\r\n");
    }

    return set;
}

/**
 * @brief Offset calculation for Gyro
 */
static void ei_gyro_offset(void)
{
    const uint8_t offset_cycles = 10;
    float raw_gyro[3] = {0};
    uint32_t gyro_offset[3] = {0};
    uint8_t i;
    uint8_t cycles;
    uint8_t tx_buffer[2] = {0};

    for (cycles = 0; cycles < offset_cycles ; cycles++){
        ei_inertial_read_gyro(raw_gyro);

        gyro_offset[0] += (uint32_t)raw_gyro[0];
        gyro_offset[1] += (uint32_t)raw_gyro[1];
        gyro_offset[2] += (uint32_t)raw_gyro[2];
        ei_sleep(50);
    }

    gyro_offset[0] /= offset_cycles;
    gyro_offset[1] /= offset_cycles;
    gyro_offset[2] /= offset_cycles;

    ei_inertial_change_bank(BANK_2);

    for (i = 0; i < 3; i++){
        tx_buffer[0] = (gyro_offset[i] >> 8) & 0xFF;
        tx_buffer[1] = gyro_offset[i] & 0xFF;
        ei_inertial_write_register(BK_2_REGISTER_XG_OFFS_USRH + (i*2), tx_buffer[0]);
        ei_inertial_write_register(BK_2_REGISTER_XG_OFFS_USRL + (i*2), tx_buffer[1]);
    }
}
