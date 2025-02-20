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
/* Include ----------------------------------------------------------------- */
#include "ei_inertial_bmx055_sensor.h"
#include "peripheral/spi.h"
#include "peripheral/timer_handler.h"
#include "ingestion-sdk-platform/renesas-ck-ra6m5/ei_device_renesas_ck_ra6m5.h"
#include "firmware-sdk/ei_device_info_lib.h"
#include "firmware-sdk/ei_fusion.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"

/* define and const */
#define CONVERT_G_TO_MS2            (9.80665f)
#define CONVERT_ACC_TO_G            (float)(2.0/2048.0f)
#define CONVERT_ACC_TO_MS2          (float)(CONVERT_G_TO_MS2*CONVERT_ACC_TO_G)

#define CONVERT_ADC_GYR             (float)(250.0f/32768.0f)
#define CONVERT_ADC_MAG_XY          (float)(1300.0f/4096.0f)
#define CONVERT_ADC_MAG_Z           (float)(2500.0f/16384.0f)

/* register */
// accelerometer
#define BMX055_ACC_REGISTER_CHIP        (0x00)
#define BMX055_ACC_REGISTER_X_LSB       (0x02)
#define BMX055_ACC_REGISTER_Y_LSB       (0x04)
#define BMX055_ACC_REGISTER_Z_LSB       (0x06)
#define BMX055_ACC_REGISTER_SOFT_RST    (0x14)
#define BMX055_ACC_REGISTER_PMU_RANGE   (0x0F)
#define BMX055_ACC_REGISTER_PMU_BW      (0x10)
#define BMX055_ACC_REGISTER_PMU_LPW     (0x11)
#define BMX055_ACC_REGISTER_ACCD_HBW    (0x13)

#define ACC_SW_RESET                    (0xB6)

// gyroscope
#define BMX055_GYR_REGISTER_CHIP        (0x00)
#define BMX055_GYR_REGISTER_X_LSB       (0x02)
#define BMX055_GYR_REGISTER_Y_LSB       (0x04)
#define BMX055_GYR_REGISTER_Z_LSB       (0x06)
#define BMX055_GYR_REGISTER_RANGE       (0x0F)
#define BMX055_GYR_REGISTER_BW          (0x10)
#define BMX055_GYR_REGISTER_LPM1        (0x11)
#define BMX055_GYR_REGISTER_LPM2        (0x12)
#define BMX055_GYR_REGISTER_RATE_HBW    (0x12)
#define BMX055_GYR_REGISTER_SFT_RST     (0x14)

// magnetometer
#define BMX055_MAG_REGISTER_CHIP        (0x40)
#define BMX055_MAG_REGISTER_X_LSB       (0x42)
#define BMX055_MAG_REGISTER_4B          (0x4B)
#define BMX055_MAG_REGISTER_4C          (0x4C)
#define BMX055_MAG_REGISTER_51          (0x51)
#define BMX055_MAG_REGISTER_52          (0x52)

#define BMX055_ACC_CHIP_ID              (0b11111010)
#define BMX055_GYR_CHIP_ID              (0b00001111)
#define BMX055_MAG_CHIP_ID              (0b00110010)

#define BMX055_READ_FLAG                (0x80)

/* private functions */
static bool ei_inertial_bmx_config(void);
static bool ei_read_acc(float* acc_data);
static bool ei_read_gyr(float* gyr_data);
static bool ei_read_mag(float* gyr_data);

/* variables */
static float inertial_data[INERTIAL_AXIS_SAMPLED];


/**
 * Connected to PMOD1:
 * CS: P205
 * SPI:
 */

/**
 *
 * @return
 */
bool ei_inertial_bmx_init(void)
{
    if (ei_inertial_bmx_config() == false){
        ei_printf("Inertial BMX055 not found\r\n");
        return false;
    }

    if(ei_add_sensor_to_fusion_list(bmx_sensor) == false) {
        ei_printf("ERR: failed to register Inertial BMX sensor!\r\n");
        return false;
    }

    return true;
}

/**
 *
 * @param n_samples
 * @return
 */
float *ei_fusion_inertial_bmx_read_data(int n_samples)
{
    float local_data[3u];

    if (n_samples >= 3){    /* acc */
        if (ei_read_acc(local_data) == true){
            memcpy(inertial_data, local_data, sizeof(local_data));
        }
        else{
            memset(inertial_data, 0, sizeof(local_data));
        }

    }

    if (n_samples >= 6){    /* gyr */
        ei_read_gyr(local_data);
        memcpy(&inertial_data[3], local_data, sizeof(local_data));
    }

    if (n_samples >= 9){    /* mag */
        ei_read_mag(local_data);
        memcpy(&inertial_data[6], local_data, sizeof(local_data));
    }

    return inertial_data;
}


/**
 *
 */
void ei_inertial_bmx_read_acc_test(void)
{
    float acc_data[3];
    uint8_t i;

    while(1)
    {
        ei_read_acc(acc_data);
        ei_printf("\nAcc data:");
        for (i = 0; i < 3; i++){
            ei_printf("\n");
            ei_printf_float(acc_data[i]);
        }
        ei_read_gyr(acc_data);
        ei_printf("\nGyr data:");
        for (i = 0; i < 3; i++){
            ei_printf("\n");
            ei_printf_float(acc_data[i]);
        }

        ei_read_mag(acc_data);
        ei_printf("\nMag data:");
        for (i = 0; i < 3; i++){
            ei_printf("\n");
            ei_printf_float(acc_data[i]);
        }

        R_BSP_SoftwareDelay(500, BSP_DELAY_UNITS_MILLISECONDS);
    }
}

/**
 *
 * @return
 */
static bool ei_inertial_bmx_config(void)
{
    uint8_t rx_buffer[2u] = {0};

    /* accelerometer */
    spi_read(CS_ACC_SPI0, BMX055_ACC_REGISTER_CHIP | BMX055_READ_FLAG, rx_buffer, 1);
    if (rx_buffer[0] != BMX055_ACC_CHIP_ID){
        ei_printf("[ERR] Wrong chip id for accelerometer BMX055, read: %d\n", rx_buffer[0]);

        return false;
    }
    else
    {
        spi_write(CS_ACC_SPI0, BMX055_ACC_REGISTER_SOFT_RST, ACC_SW_RESET);   // soft reset
        R_BSP_SoftwareDelay(10, BSP_DELAY_UNITS_MILLISECONDS);                 // wait 10 ms
        spi_write(CS_ACC_SPI0, BMX055_ACC_REGISTER_PMU_RANGE, 0b00000011);      // 2g range
        spi_write(CS_ACC_SPI0, BMX055_ACC_REGISTER_PMU_BW, 0b00001101);         // bandwith 250 Hz
        spi_write(CS_ACC_SPI0, BMX055_ACC_REGISTER_PMU_LPW, 0b00001010);        // power mode - seep 10 ms
        spi_write(CS_ACC_SPI0, BMX055_ACC_REGISTER_ACCD_HBW, 0b00000000);       // filtered - shadowing enabled
    }

    /* gyroscope */
    spi_read(CS_GYR_SPI0, BMX055_GYR_REGISTER_CHIP | BMX055_READ_FLAG, rx_buffer, 1);
    if (rx_buffer[0] != BMX055_GYR_CHIP_ID){
        ei_printf("[ERR] Wrong chip id for gyroscope BMX055, read: %d\n", rx_buffer[0]);

        return false;
    }
    else{
        spi_write(CS_GYR_SPI0, BMX055_GYR_REGISTER_SFT_RST, ACC_SW_RESET);   // soft reset
        R_BSP_SoftwareDelay(10, BSP_DELAY_UNITS_MILLISECONDS);                 // wait 10 ms
        spi_write(CS_GYR_SPI0, BMX055_GYR_REGISTER_RANGE, 0b00000011);  // +- 250
        spi_write(CS_GYR_SPI0, BMX055_GYR_REGISTER_BW, 0b00000111);
        spi_write(CS_GYR_SPI0, BMX055_GYR_REGISTER_LPM1, 0b00000001);   // normal 4ms
        spi_write(CS_GYR_SPI0, BMX055_GYR_REGISTER_RATE_HBW, 0b00000000);   // shadowing enabled
    }

    /* magnetometer */
    spi_write(CS_MAG_SPI0, BMX055_MAG_REGISTER_4B, 0b00000001);   // wake up magnetometer
    R_BSP_SoftwareDelay(10, BSP_DELAY_UNITS_MILLISECONDS);                 // wait 10 ms
    spi_read(CS_MAG_SPI0, BMX055_MAG_REGISTER_CHIP | BMX055_READ_FLAG, rx_buffer, 1);
    // MAG chip id can be read only if power bit = 1 (address 0x4B)
    if (rx_buffer[0] != BMX055_MAG_CHIP_ID){
        ei_printf("[ERR] Wrong chip id for magnetometer BMX055, read: %d\n", rx_buffer[0]);
        return false;
    }
    else{
        spi_write(CS_MAG_SPI0, BMX055_MAG_REGISTER_4C, 0b00000000); // 10 Hz
        spi_write(CS_MAG_SPI0, BMX055_MAG_REGISTER_51, 0b00000100);   // 1+2*reg_val
        spi_write(CS_MAG_SPI0, BMX055_MAG_REGISTER_52, 0b00000100);   // 1+reg_val
    }

    return true;
}

/**
 *
 * @param acc_data
 * @return
 */
static bool ei_read_acc(float* acc_data)
{
    uint8_t rx_buffer[8u];
    uint8_t i;
    bool retval = false;

    if (acc_data != nullptr)
    {
        spi_read(CS_ACC_SPI0, BMX055_ACC_REGISTER_X_LSB | BMX055_READ_FLAG, rx_buffer, 6);

        for (i = 0; i < 3; i++)
        {
            acc_data[i] = (((int16_t)(((rx_buffer[i*2] & 0xF0)) | (rx_buffer[i*2 + 1] << 8))) >> 4);
            acc_data[i] *= CONVERT_ACC_TO_MS2;
        }

        retval = true;
    }
    else
    {
        // error
    }

    return retval;
}

/**
 *
 * @param gyr_data
 * @return
 */
static bool ei_read_gyr(float* gyr_data)
{
    uint8_t rx_buffer[8u];
    uint8_t i;
    bool retval = false;

    if (gyr_data != nullptr){
        spi_read(CS_GYR_SPI0, BMX055_GYR_REGISTER_X_LSB | BMX055_READ_FLAG, rx_buffer, 6);

        for (i = 0; i < 3; i++)
        {
            gyr_data[i] = (int16_t)(rx_buffer[i*2] | (rx_buffer[i*2 + 1] << 8));
            gyr_data[i] *= CONVERT_ADC_GYR;
        }

        retval = true;
    }


    return retval;
}

/**
 *
 * @param mag_data
 * @return
 */
static bool ei_read_mag(float* mag_data)
{
    bool retval = false;
    uint8_t rx_buffer[8u];
    uint8_t i;

    if (mag_data != nullptr){
        spi_read(CS_MAG_SPI0, BMX055_MAG_REGISTER_X_LSB | BMX055_READ_FLAG, rx_buffer, 6);

        for (i = 0; i < 2; i++){
            mag_data[i] = (((int16_t)(((rx_buffer[i*2] & 0xF8)) | (rx_buffer[i*2 + 1] << 8))) >> 3);    /* x & y axis are 13 bit */
            mag_data[i] *= CONVERT_ADC_MAG_XY;
        }
        mag_data[2] = (((int16_t)(((rx_buffer[i*2] & 0xFE)) | (rx_buffer[i*2 + 1] << 8))) >> 1);    /* z axis is 15 bit */
        mag_data[2] *= CONVERT_ADC_MAG_Z;

        retval = true;
    }


    return retval;
}

