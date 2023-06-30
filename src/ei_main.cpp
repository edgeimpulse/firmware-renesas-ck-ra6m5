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

