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
/* Includes */
#include "timer_handler.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include <stdint.h>

#define MICROSECONDS_TO_SECONDS 1000000

static uint64_t timer_overflow_times;
static uint64_t div_ratio = 0;

volatile bool _timer_1_set = false;

static inline void set_timer_overflow_times(uint64_t value);
static inline uint64_t get_timer_overflow_times(void);
static inline uint64_t get_timer_count(void);

/* public functions */
/**
 *
 */
void ei_timer_init(void)
{
    fsp_err_t err = FSP_SUCCESS;
    timer_info_t info;

    err = R_GPT_Open (&g_timer_us_ctrl, &g_timer_us_cfg);
    err &= R_GPT_Open (&g_timer1_ctrl, &g_timer1_cfg);

    if (FSP_SUCCESS != err) {
        /* GPT Timer failure message */
        APP_ERR_PRINT ("\r\n** R_GPT_TimerOpen API failed **\r\n");
    }

    (void) R_GPT_InfoGet(&g_timer_us_ctrl, &info);

    div_ratio = (info.clock_frequency / MICROSECONDS_TO_SECONDS);
}

/**
 *
 */
void ei_timer0_start(void)
{
    fsp_err_t err = FSP_SUCCESS;

    /* Start GPT module - no error control for now */
    err = R_GPT_Start (&g_timer_us_ctrl);
    set_timer_overflow_times(0);
}

/**
 * @brief timer used for sampling
 *
 * @param count Time in ms
 */
void ei_timer1_start(uint32_t count)
{
    /* Get the source clock frequency (in Hz). There are 3 ways to do this in FSP:
     *  - If the PCLKD frequency has not changed since reset, the source clock frequency is
     *    BSP_STARTUP_PCLKD_HZ >> timer_cfg_t::source_div
     *  - Use the R_GPT_InfoGet function (it accounts for the divider).
     *  - Calculate the current PCLKD frequency using R_FSP_SystemClockHzGet(FSP_PRIV_CLOCK_PCLKD) and right shift
     *    by timer_cfg_t::source_div.
     *
     * This example uses the 3rd option (R_FSP_SystemClockHzGet).
     */
    uint32_t pclkd_freq_hz = R_FSP_SystemClockHzGet(FSP_PRIV_CLOCK_PCLKD) >> g_timer1_cfg.source_div;

    uint32_t period_counts = (uint32_t) (((uint64_t) pclkd_freq_hz * count) / 1000);
    //callback_timer1 = cb_toset;
    _timer_1_set = false;

    R_GPT_PeriodSet(&g_timer1_ctrl, period_counts); /* set new period */

    /* Start GPT module - no error control for now */
    R_GPT_Start (&g_timer1_ctrl);
}

/**
 *
 */
void ei_timer0_stop(void)
{
    fsp_err_t err = FSP_SUCCESS;

    /* Stop GPT module - no error control for now */
    err =  R_GPT_Stop (&g_timer_us_ctrl);
}

/**
 *
 */
void ei_timer1_stop(void)
{
    _timer_1_set = false;
    /* Stop GPT module - no error control for now */
    R_GPT_Stop (&g_timer1_ctrl);
}

/**
 * @brief callback function for interrupt
 *
 * @param p_args
 */
void periodic_timer_msgq_cb(timer_callback_args_t *p_args)
{
    if (TIMER_EVENT_CYCLE_END == p_args->event) {
        set_timer_overflow_times(get_timer_overflow_times() + 1);
    }
}

/**
 *
 * @param p_args
 */
void timer1_interrupt(timer_callback_args_t *p_args)
{
    FSP_PARAMETER_NOT_USED (p_args);
    R_GPT_Stop (&g_timer1_ctrl);
    _timer_1_set = true;
}

/**
 *
 * @return
 */
uint32_t timer_get_ms(void)
{
    return (timer_get_us()/1000u);
}

/**
 *
 * @return
 */
uint32_t timer_get_us(void)
{
    uint64_t overflow_time = ((uint64_t)1 << 32) / div_ratio;
    return (uint32_t)((get_timer_overflow_times() * overflow_time)
         + (get_timer_count()/div_ratio));
}

/**
 *
 * @param value
 */
static inline void set_timer_overflow_times(uint64_t value)
{
    timer_overflow_times = value;
}

/**
 *
 * @return
 */
static inline uint64_t get_timer_overflow_times(void)
{
    return timer_overflow_times;
}

/**
 *
 * @return
 */
static inline uint64_t get_timer_count(void)
{
    timer_status_t status;
    R_GPT_StatusGet(&g_timer_us_ctrl, &status);
    return status.counter;
}

