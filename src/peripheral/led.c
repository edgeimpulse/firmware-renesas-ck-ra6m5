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
#include "led.h"
#include "bsp_pin_cfg.h"

static const bsp_io_port_pin_t _led_vector[e_user_led_max] =
{
 LED_RED,
 LED_GREEN,
 LED_BLUE
};


/**
 * @brief turn led off
 *
 * @param led
 */
void ei_led_turn_off(t_user_led led)
{
    if (led < e_user_led_max){
        R_BSP_PinAccessEnable();
        R_BSP_PinWrite((bsp_io_port_pin_t)_led_vector[led], BSP_IO_LEVEL_LOW);
        R_BSP_PinAccessDisable();
    }
}

/**
 * @brief turn led on
 *
 * @param led
 */
void ei_led_turn_on(t_user_led led)
{
    if (led < e_user_led_max){
        R_BSP_PinAccessEnable();
        R_BSP_PinWrite((bsp_io_port_pin_t)_led_vector[led], BSP_IO_LEVEL_HIGH);
        R_BSP_PinAccessDisable();
    }
}
