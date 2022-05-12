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

/* */
sampler_callback  cb_acc_sampler;
uint32_t _samples_interval;
volatile bool _sampling_finished;
typedef void(*local_sampler_callback)(void);

static local_sampler_callback local_cb;
static float inertial_fusion_data[INERTIAL_AXIS_SAMPLED];
/* Constant ---------------------------------------------------------------- */
#define INERTIAL_SENSOR_ADDRESS     (0x68)
#define INERTIAL_SENSOR_ID          (0xEA)

#define CONVERT_G_TO_MS2            (9.80665f)
#define ACC_RAW_SCALING             (32767.5f)
#define INERTIAL_ACCEL_LSB_G        (16.384f)

#define GYRO_SCALE_FACTOR           (65.5f)

/* Change bank register */
#define REGISTER_REG_BANK_SEL   (0x7F)
#define BANK_0                  (0)
#define BANK_1                  (1u)
#define BANK_2                  (2u)
#define BANK_3                  (3u)

/* Bank 0 register - General config and sensor read */
#define REGISTER_WHO_AM_I       (0x00)
#define REGISTER_LP_CONFIG      (0x05)
#define REGISTER_PWR_MGMT_1     (0x06)
#define REGISTER_PWR_MGMT_2     (0x07)
#define REGISTER_INT_PIN_CFG    (0x0F)

#define REGISTER_ACCEL_XOUT_H   (0x2D)
#define REGISTER_ACCEL_XOUT_L   (0x2E)
#define REGISTER_ACCEL_YOUT_H   (0x2F)
#define REGISTER_ACCEL_YOUT_L   (0x30)
#define REGISTER_ACCEL_ZOUT_H   (0x31)
#define REGISTER_ACCEL_ZOUT_L   (0x32)

#define REGISTER_GYRO_XOUT_H    (0x33)
#define REGISTER_GYRO_XOUT_L    (0x34)
#define REGISTER_GYRO_YOUT_H    (0x35)
#define REGISTER_GYRO_YOUT_L    (0x36)
#define REGISTER_GYRO_ZOUT_H    (0x37)
#define REGISTER_GYRO_ZOUT_L    (0x38)

#define REGISTER_TEMP_OUT_H     (0x39)
#define REGISTER_TEMP_OUT_L     (0x3A)

/* Bank 1 register - Self test and offset */

/* Bank 2 register - config for separate sensor */
#define REGISTER_GYRO_SMPLRT_DIV    (0x00)
#define REGISTER_GYRO_CONFIG_1      (0x01)
#define REGISTER_GYRO_CONFIG_2      (0x02)

#define REGISTER_XG_OFFS_USRH       (0x03)
#define REGISTER_XG_OFFS_USRL       (0x04)
#define REGISTER_YG_OFFS_USRH       (0x05)
#define REGISTER_YG_OFFS_USRL       (0x06)
#define REGISTER_ZG_OFFS_USRH       (0x07)
#define REGISTER_ZG_OFFS_USRL       (0x08)

#define REGISTER_ACCEL_SMPLRT_DIV_1 (0x10)
#define REGISTER_ACCEL_SMPLRT_DIV_2 (0x11)
#define REGISTER_ACCEL_CONFIG       (0x14)
#define REGISTER_ACCEL_CONFIG_2     (0x15)

/* Bank 3 register - config for additional I2C slave */

/* Private functions ------------------------------------------------ */
static bool ei_inertial_sensor_set(bool accel_on, bool gyro_on);
static bool ei_inertial_init_config(void);
static bool ei_inertial_read_accelerometer(float* acc_data);
static bool ei_inertial_read_gyro(float* gyro_data);
static inline int ei_inertial_change_bank(uint8_t bank_nr);
static int ei_inertial_write_register(uint8_t reg, uint8_t data);
static bool ei_inertial_read_internal_temp(float* temp);
static void ei_inertial_read_acc_cb(void);
static void ei_gyro_offset(void);

/**
 * @brief Configure inertial sensor
 *
 * @return
 */
bool ei_inertial_init(void)
{
    g_comms_i2c_accelerometer.p_api->open(g_comms_i2c_accelerometer.p_ctrl, g_comms_i2c_accelerometer.p_cfg);

    cb_acc_sampler = nullptr;

    if (ei_inertial_check_presence() == false)
    {
        ei_printf("ERR: failed to init Inertial sensor!\n");
    }

    ei_inertial_init_config();
    ei_inertial_sensor_set(true, true);    /* turn on accelerometer and gyro */

    if(ei_add_sensor_to_fusion_list(inertial_sensor) == false) {
        ei_printf("ERR: failed to register Inertial sensor!\n");
        return false;
    }

    return true;
}

/**
 * @brief 
 * 
 * @param callsampler callback to ei_sampler
 * @param sample_interval_ms 
 * @return true 
 * @return false 
 */
bool ei_inertial_accel_sample_start(sampler_callback callsampler, float sample_interval_ms)
{
    EiDeviceCKRA6M5 *ei_device =  EiDeviceCKRA6M5::get_device();
    cb_acc_sampler = callsampler;

    _sampling_finished = false;
    _samples_interval = (uint32_t)sample_interval_ms;

    ei_device->start_sample_thread(&ei_inertial_read_acc_cb, sample_interval_ms);
    return true;
}

/**
 *
 */
void ei_sampler_thread(void)
{
    if (_timer_1_set == true)
    {
        ei_timer1_stop();
        if (local_cb != nullptr)
        {
            local_cb();
        }
    }
}

/**
 * @brief Setup sampling for just the accelerometer
 * 
 * @return true 
 * @return false 
 */
bool ei_intertial_accel_setup_data_sampling(void)
{
    bool sample_start = false;
    EiDeviceCKRA6M5 *ei_device =  EiDeviceCKRA6M5::get_device();
    EiDeviceMemory* mem = ei_device->get_memory();

    if (ei_device->get_sample_interval_ms() < 10.0f ) {
        ei_device->set_sample_interval_ms(10.0f);
    }

    uint32_t available_bytes = (mem->get_available_sample_blocks() - 1) * mem->block_size;  //
    uint32_t requested_bytes = ceil((ei_device->get_sample_length_ms() / ei_device->get_sample_interval_ms()) * SIZEOF_ACCEL_AXIS_SAMPLED * 2);

    if(requested_bytes > available_bytes) {
        ei_printf("ERR: Sample length is too long. Maximum allowed is %ims at %.1fHz.\r\n",
            (int)floor(available_bytes / ((SIZEOF_ACCEL_AXIS_SAMPLED * 2) / ei_device->get_sample_interval_ms())),
            (1000 / ei_device->get_sample_interval_ms()));
        return false;
    }

    sensor_aq_payload_info payload = {
        // Unique device ID (optional), set this to e.g. MAC address or device EUI **if** your device has one
                                      ei_device->get_device_id().c_str(),
        // Device type (required), use the same device type for similar devices
                                      ei_device->get_device_type().c_str(),
        // How often new data is sampled in ms. (100Hz = every 10 ms.)
        ei_device->get_sample_interval_ms(),
        // The axes which you'll use. The units field needs to comply to SenML units (see https://www.iana.org/assignments/senml/senml.xhtml)
        { { "accX", "m/s2" }, { "accY", "m/s2" }, { "accZ", "m/s2" }},
    };

    sample_start = ei_sampler_start_sampling(&payload, &ei_inertial_accel_sample_start, SIZEOF_ACCEL_AXIS_SAMPLED);

    return sample_start;
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

    ei_inertial_write_register(REGISTER_PWR_MGMT_1, 0x80);      /* reset */
    R_BSP_SoftwareDelay(100u, BSP_DELAY_UNITS_MILLISECONDS);    /* and wait 100ms */

    ret_val = ei_i2c_read_byte_command(&g_comms_i2c_accelerometer_ctrl, REGISTER_WHO_AM_I, &rx_buffer, 1u, I2C_NO_WAIT_BETWEEN_W_R);

    if (ret_val == 0)
    {
        if (rx_buffer == INERTIAL_SENSOR_ID)
        {
            test_ok = true;
        }
        else
        {
            ei_printf("TEST FAILED: I2C read OK, but ID read is NOK :( 0x%2x \r\n", rx_buffer);
        }
    }
    else
    {
        ei_printf("TEST FAILED: I2C read NOK, error code %d :( \r\n", ret_val);
    }
    return test_ok;
}

/**
 *
 */
void ei_inertial_read_acc_test(void)
{
    static bool first_time = true;
    float acc_data[3] = {0};
    float gyr_data[3] = {0};
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
            inertial_fusion_data[0] = 0.0f;
            inertial_fusion_data[1] = 0.0f;
            inertial_fusion_data[2] = 0.0f;
        }
    }

    if (n_samples == 6)
    {
        if (ei_inertial_read_gyro(&inertial_fusion_data[3]) == false)
        {
            ei_printf("ERR: no Gyro data!\n");
            inertial_fusion_data[3] = 0.0f;
            inertial_fusion_data[4] = 0.0f;
            inertial_fusion_data[5] = 0.0f;
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
    ei_inertial_write_register(REGISTER_PWR_MGMT_1, config);
    R_BSP_SoftwareDelay((uint32_t)20, BSP_DELAY_UNITS_MILLISECONDS);

    if (accel_on == false)  /* turn off accel */
    {
        config |= 0x38;
    }

    if (gyro_on == false)   /* turn off gyro */
    {
        config |= 0x7;
    }

    if (ei_inertial_write_register(REGISTER_PWR_MGMT_2, config) == 0)
    {
        uint8_t rx_data;
        if (ei_i2c_read_byte_command(&g_comms_i2c_accelerometer_ctrl, REGISTER_PWR_MGMT_2, &rx_data, sizeof(rx_data), I2C_NO_WAIT_BETWEEN_W_R) == 0)
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
    if (ei_inertial_write_register(REGISTER_LP_CONFIG, config) == 0)
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
    ei_inertial_write_register(REGISTER_GYRO_SMPLRT_DIV, config);
    ei_inertial_write_register(REGISTER_ACCEL_CONFIG, (2 << 3) | 1);

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
    ei_inertial_write_register(REGISTER_ACCEL_CONFIG, (2 << 3) | 1);

    config = 5;
    if (ei_inertial_write_register(REGISTER_ACCEL_SMPLRT_DIV_2, config) == 0)
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
        if (ei_i2c_read_byte_command(&g_comms_i2c_accelerometer_ctrl, REGISTER_ACCEL_XOUT_H, &rx_data[0], sizeof(rx_data), I2C_NO_WAIT_BETWEEN_W_R) == 0)
        {
            for (i = 0; i < 3; i++)
            {
                acc_data[i] = (float)((int16_t)(rx_data[i*2] << 8) | rx_data[i*2 + 1]);
                acc_data[i] *= (2.0*CONVERT_G_TO_MS2)/ACC_RAW_SCALING;
            }
            read = true;
        }
    }

    return read;
}

/**
 *
 * @return
 */
static void ei_inertial_read_acc_cb(void)
{
    float acc_data[3] = {0};
    uint8_t rx_data[6u] = {0};
    uint8_t i;

    ei_inertial_change_bank(BANK_0);

    if (ei_i2c_read_byte_command(&g_comms_i2c_accelerometer_ctrl, REGISTER_ACCEL_XOUT_H, &rx_data[0], sizeof(rx_data), I2C_NO_WAIT_BETWEEN_W_R) == 0)
    {
        for (i = 0; i < 3; i++)
        {
            acc_data[i] = (float)((int16_t)(rx_data[i*2] << 8) | rx_data[i*2 + 1]);
            acc_data[i] *= (2.0*CONVERT_G_TO_MS2)/ACC_RAW_SCALING;
        }
    }

    if (cb_acc_sampler != nullptr)
    {
        if (cb_acc_sampler((const void *)&acc_data[0], SIZEOF_ACCEL_AXIS_SAMPLED) == true)
        {
            _sampling_finished = true;
            ei_timer1_stop();
        }
        else
        {
            ei_timer1_start((uint32_t)_samples_interval);
        }
    }
}

/**
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
        if (ei_i2c_read_byte_command(&g_comms_i2c_accelerometer_ctrl, REGISTER_GYRO_XOUT_H, &rx_data[0], sizeof(rx_data), I2C_NO_WAIT_BETWEEN_W_R) == 0)
        {
            for (i = 0; i < 3; i++)
            {
                gyro_data[i] = (float)( (int16_t)((rx_data[i*2] << 8) | rx_data[(i*2) + 1])) ;
                gyro_data[i] *= (250.0/32768.0);
            }
            read = true;
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
        if (ei_i2c_read_byte_command(&g_comms_i2c_accelerometer_ctrl, REGISTER_TEMP_OUT_H, &rx_data[0], sizeof(rx_data), I2C_NO_WAIT_BETWEEN_W_R) == 0)
        {
            raw_temp =  (int16_t)((rx_data[0] << 8) | rx_data[1]) ;
            *temp = 21.0 + (raw_temp/333.87);
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
    return ei_inertial_write_register(REGISTER_REG_BANK_SEL, bank_nr);
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
 * @brief Offset calculation for Gyro
 */
static void ei_gyro_offset(void)
{
    float raw_gyro[3] = {0};
    uint32_t gyro_offset[3] = {0};
    uint8_t i;
    uint8_t cycles;
    uint8_t tx_buffer[2] = {0};

    for (cycles = 0; cycles < 10 ; cycles++){
        ei_inertial_read_internal_temp(raw_gyro);

        gyro_offset[0] += raw_gyro[0];
        gyro_offset[1] += raw_gyro[1];
        gyro_offset[2] += raw_gyro[2];
        ei_sleep(50);
    }

    gyro_offset[0] /= 10;
    gyro_offset[1] /= 10;
    gyro_offset[2] /= 10;

    ei_inertial_change_bank(BANK_2);

    for (i = 0; i < 3; i++){
        tx_buffer[0] = (gyro_offset[i] >> 8) & 0xFF;
        tx_buffer[1] = gyro_offset[i] & 0xFF;
        ei_inertial_write_register(REGISTER_XG_OFFS_USRH + (i*2), tx_buffer[0]);
        ei_inertial_write_register(REGISTER_XG_OFFS_USRL + (i*2), tx_buffer[1]);
    }
}
