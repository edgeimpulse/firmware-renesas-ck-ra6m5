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
/* Include --------------------------------------------------------------------*/
#include "ei_main.h"
#include "renesas-ck-ra6m5/ei_at_handlers.h"
#include "peripheral/uart_ep.h"
#include "peripheral/led.h"
#include "peripheral/timer_handler.h"
#include "peripheral/i2c.h"
#include "peripheral/i2s.h"
#include "peripheral/spi.h"
#include "sensors/ei_inertial_bmx055_sensor.h"
#include "sensors/ei_inertial_sensor.h"
#include "sensors/ei_microphone.h"
#include "sensors/ei_air_quality_indoor.h"
#include "sensors/ei_air_quality_outdoor.h"
#include "sensors/ei_environmental.h"
#include "sensors/ei_barometric_sensor.h"
#include "sensors/ei_adc.h"

#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "ingestion-sdk-platform/renesas-ck-ra6m5/ei_device_renesas_ck_ra6m5.h"
#include "inference/ei_run_impulse.h"
#include "peripheral/flash_handler.h"

#include "sensors/ei_test_sensors.h"

/* Private variables -------------------------------------------------------------------- */
static ATServer *at;
EiDeviceCKRA6M5* p_dev;

/* Function definition -------------------------------------------------------------------- */
/**
 *
 * @return
 */
int ei_init(void)
{
    p_dev = static_cast<EiDeviceCKRA6M5*>(EiDeviceInfo::get_device());

    /* turns off user led - init of GPIO is already done */
    ei_led_turn_off(e_user_led_blue);
    ei_led_turn_off(e_user_led_green);
    ei_led_turn_off(e_user_led_red);

    /* init timer */
    ei_timer_init();
    ei_timer0_start();

    /* init i2c */
    ei_i2c_init();

    /* init spi */
    ei_spi_init();

    /* init i2s */
    ei_i2s_driver_init();

    /* ADC */
    ei_adc_init();

    if (uart_initialize() == FSP_SUCCESS)  /* for now not checking ret value */
    {
        at = ei_at_init(p_dev);
        ei_led_turn_on(e_user_led_green);
    }
    else
    {
        ei_led_turn_on(e_user_led_red);
        at = NULL;
        // error handling
    }
    flash_handler_init();

    return 0;
}

/**
 *
 * @return
 */
int ei_main(void)
{
    ei_printf("Type AT+HELP to see a list of commands.\r\n");
    ei_printf("Starting main loop\r\n");

    at->print_prompt();

    /* sensors init */
    if (ei_inertial_bmx_init() == false){
        ei_spi_deinit();    // can de init SPI, only BMX uses it
        // bmx055 not present, use the old one
        ei_inertial_init();
    }

    ei_environmental_sensor_init();
#if EI_AIR_QUALITY_INDOOR_ENABLED == 1
    ei_air_quality_indoor_sensor_init();
#endif
#if EI_AIR_QUALITY_OUTDOOR_ENABLED == 1
    ei_air_quality_outdoor_sensor_init();
#endif
    ei_barometric_sensor_init();

    /* debug sensors, remove ! */

#if EI_TEST_INDOOR_AQS == 1 || EI_TEST_OUTDOOR_AQS == 1 || EI_TEST_TEMP_HUM_TEST == 1 || EI_TEST_BAROMETRIC == 1 || EI_TEST_BAROMETRIC == 1 || EI_TEST_MIC == 1 || EI_TEST_INERTIAL == 1
    ei_test_sensors();
#endif

    while(1)
    {
        /* handle command comming from uart */
        char data = ei_get_serial_byte((uint8_t)is_inference_running());

        while ((uint8_t)data != 0xFF) {
            ei_led_turn_on(e_user_led_blue);

            if(is_inference_running() && data == 'b') {
              ei_stop_impulse();
              at->print_prompt();
              continue;
            }

            at->handle(data);
            data = ei_get_serial_byte((uint8_t)is_inference_running());
        }
        ei_led_turn_off(e_user_led_blue);


        if (is_inference_running() == true)
        {
            ei_run_impulse();
        }
    }

    return 0;
}

