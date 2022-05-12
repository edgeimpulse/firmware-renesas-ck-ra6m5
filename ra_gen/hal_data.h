/* generated HAL header file - do not edit */
#ifndef HAL_DATA_H_
#define HAL_DATA_H_
#include <stdint.h>
#include "bsp_api.h"
#include "common_data.h"
#include "rm_comms_i2c.h"
#include "rm_comms_api.h"
#include "rm_hs300x.h"
#include "rm_hs300x_api.h"
#include "../ra/fsp/src/rm_zmod4xxx/zmod4xxx_types.h"
#include "../ra/fsp/src/rm_zmod4xxx/oaq_2nd_gen/oaq_2nd_gen.h"
#include "rm_zmod4xxx.h"
#include "rm_zmod4xxx_api.h"
#include "../ra/fsp/src/rm_zmod4xxx/zmod4xxx_types.h"
#include "../ra/fsp/src/rm_zmod4xxx/iaq_2nd_gen/iaq_2nd_gen.h"
#include "r_gpt.h"
#include "r_timer_api.h"
#include "r_flash_hp.h"
#include "r_flash_api.h"
#include "r_dtc.h"
#include "r_transfer_api.h"
#include "r_i2s_api.h"
            #include "r_ssi.h"
#include "r_sci_uart.h"
            #include "r_uart_api.h"
FSP_HEADER
/* I2C Communication Device */
extern const rm_comms_instance_t g_comms_i2c_biometric;
extern rm_comms_i2c_instance_ctrl_t g_comms_i2c_biometric_ctrl;
extern const rm_comms_cfg_t g_comms_i2c_biometric_cfg;
#ifndef comms_i2c_callback
void comms_i2c_callback(rm_comms_callback_args_t * p_args);
#endif
/* I2C Communication Device */
extern const rm_comms_instance_t g_comms_i2c_barometric;
extern rm_comms_i2c_instance_ctrl_t g_comms_i2c_barometric_ctrl;
extern const rm_comms_cfg_t g_comms_i2c_barometric_cfg;
#ifndef comms_i2c_callback
void comms_i2c_callback(rm_comms_callback_args_t * p_args);
#endif
/* I2C Communication Device */
extern const rm_comms_instance_t g_comms_i2c_accelerometer;
extern rm_comms_i2c_instance_ctrl_t g_comms_i2c_accelerometer_ctrl;
extern const rm_comms_cfg_t g_comms_i2c_accelerometer_cfg;
#ifndef comms_i2c_callback
void comms_i2c_callback(rm_comms_callback_args_t * p_args);
#endif
/* I2C Communication Device */
extern const rm_comms_instance_t g_comms_i2c_env_device;
extern rm_comms_i2c_instance_ctrl_t g_comms_i2c_env_device_ctrl;
extern const rm_comms_cfg_t g_comms_i2c_env_device_cfg;
#ifndef rm_hs300x_callback
void rm_hs300x_callback(rm_comms_callback_args_t * p_args);
#endif
/* HS300X Sensor */
extern const rm_hs300x_instance_t g_hs300x_sensor0;
extern rm_hs300x_instance_ctrl_t g_hs300x_sensor0_ctrl;
extern const rm_hs300x_cfg_t g_hs300x_sensor0_cfg;
#ifndef hs300x_callback
void hs300x_callback(rm_hs300x_callback_args_t * p_args);
#endif
/* I2C Communication Device */
extern const rm_comms_instance_t g_comms_i2c_oaq_device;
extern rm_comms_i2c_instance_ctrl_t g_comms_i2c_oaq_device_ctrl;
extern const rm_comms_cfg_t g_comms_i2c_oaq_device_cfg;
#ifndef rm_zmod4xxx_comms_i2c_callback
void rm_zmod4xxx_comms_i2c_callback(rm_comms_callback_args_t * p_args);
#endif
/* ZMOD4510 OAQ 2nd Gen. */
extern zmod4xxx_dev_t           g_zmod4510_oaq_sensor_dev;            // This will be removed in FSP v4.0.0.
extern oaq_2nd_gen_handle_t     g_zmod4510_oaq_sensor_lib_handle;     // This will be removed in FSP v4.0.0.
extern oaq_2nd_gen_results_t    g_zmod4510_oaq_sensor_lib_results;    // This will be removed in FSP v4.0.0.
extern rm_zmod4xxx_lib_extended_cfg_t g_zmod4510_oaq_sensor_extended_cfg;
/* ZMOD4XXX Sensor */
extern const rm_zmod4xxx_instance_t g_zmod4510_oaq_sensor;
extern rm_zmod4xxx_instance_ctrl_t g_zmod4510_oaq_sensor_ctrl;
extern const rm_zmod4xxx_cfg_t g_zmod4510_oaq_sensor_cfg;
#ifndef zmod4510_oaq_comms_i2c_callback
void zmod4510_oaq_comms_i2c_callback(rm_zmod4xxx_callback_args_t * p_args);
#endif
#ifndef RA_NOT_DEFINED
void RA_NOT_DEFINED(external_irq_callback_args_t * p_args);
#endif
#ifndef zmod4510_oaq_irq_callback
void zmod4510_oaq_irq_callback(rm_zmod4xxx_callback_args_t *p_args);
#endif
/* I2C Communication Device */
extern const rm_comms_instance_t g_comms_i2c_iaq_device;
extern rm_comms_i2c_instance_ctrl_t g_comms_i2c_iaq_device_ctrl;
extern const rm_comms_cfg_t g_comms_i2c_iaq_device_cfg;
#ifndef rm_zmod4xxx_comms_i2c_callback
void rm_zmod4xxx_comms_i2c_callback(rm_comms_callback_args_t * p_args);
#endif
/* ZMOD4410 IAQ 2nd Gen. */
extern zmod4xxx_dev_t           g_zmod4410_iaq_sensor_dev;            // This will be removed in FSP v4.0.0.
extern iaq_2nd_gen_handle_t     g_zmod4410_iaq_sensor_lib_handle;     // This will be removed in FSP v4.0.0.
extern iaq_2nd_gen_results_t    g_zmod4410_iaq_sensor_lib_results;    // This will be removed in FSP v4.0.0.
extern rm_zmod4xxx_lib_extended_cfg_t g_zmod4410_iaq_sensor_extended_cfg;
/* ZMOD4XXX Sensor */
extern const rm_zmod4xxx_instance_t g_zmod4410_iaq_sensor;
extern rm_zmod4xxx_instance_ctrl_t g_zmod4410_iaq_sensor_ctrl;
extern const rm_zmod4xxx_cfg_t g_zmod4410_iaq_sensor_cfg;
#ifndef zmod4410_iaq_comms_i2c_callback
void zmod4410_iaq_comms_i2c_callback(rm_zmod4xxx_callback_args_t * p_args);
#endif
#ifndef RA_NOT_DEFINED
void RA_NOT_DEFINED(external_irq_callback_args_t * p_args);
#endif
#ifndef zmod4410_iaq_irq_callback
void zmod4410_iaq_irq_callback(rm_zmod4xxx_callback_args_t *p_args);
#endif
/** Timer on GPT Instance. */
extern const timer_instance_t g_timer2;

/** Access the GPT instance using these structures when calling API functions directly (::p_api is not used). */
extern gpt_instance_ctrl_t g_timer2_ctrl;
extern const timer_cfg_t g_timer2_cfg;

#ifndef NULL
void NULL(timer_callback_args_t * p_args);
#endif
/** Timer on GPT Instance. */
extern const timer_instance_t g_timer1;

/** Access the GPT instance using these structures when calling API functions directly (::p_api is not used). */
extern gpt_instance_ctrl_t g_timer1_ctrl;
extern const timer_cfg_t g_timer1_cfg;

#ifndef timer1_interrupt
void timer1_interrupt(timer_callback_args_t * p_args);
#endif
/* Flash on Flash HP Instance */
extern const flash_instance_t g_flash0;

/** Access the Flash HP instance using these structures when calling API functions directly (::p_api is not used). */
extern flash_hp_instance_ctrl_t g_flash0_ctrl;
extern const flash_cfg_t g_flash0_cfg;

#ifndef NULL
void NULL(flash_callback_args_t * p_args);
#endif
/* Transfer on DTC Instance. */
extern const transfer_instance_t g_transfer0;

/** Access the DTC instance using these structures when calling API functions directly (::p_api is not used). */
extern dtc_instance_ctrl_t g_transfer0_ctrl;
extern const transfer_cfg_t g_transfer0_cfg;
/** SSI Instance. */
            extern const i2s_instance_t      g_i2s0;

            /** Access the SSI instance using these structures when calling API functions directly (::p_api is not used). */
            extern ssi_instance_ctrl_t g_i2s0_ctrl;
            extern const i2s_cfg_t g_i2s0_cfg;

            #ifndef i2s_callback
            void i2s_callback(i2s_callback_args_t * p_args);
            #endif
/** Timer on GPT Instance. */
extern const timer_instance_t g_timer0;

/** Access the GPT instance using these structures when calling API functions directly (::p_api is not used). */
extern gpt_instance_ctrl_t g_timer0_ctrl;
extern const timer_cfg_t g_timer0_cfg;

#ifndef periodic_timer_msgq_cb
void periodic_timer_msgq_cb(timer_callback_args_t * p_args);
#endif
/** UART on SCI Instance. */
            extern const uart_instance_t      g_uart3;

            /** Access the UART instance using these structures when calling API functions directly (::p_api is not used). */
            extern sci_uart_instance_ctrl_t     g_uart3_ctrl;
            extern const uart_cfg_t g_uart3_cfg;
            extern const sci_uart_extended_cfg_t g_uart3_cfg_extend;

            #ifndef user_uart_callback
            void user_uart_callback(uart_callback_args_t * p_args);
            #endif
void hal_entry(void);
void g_hal_init(void);
FSP_FOOTER
#endif /* HAL_DATA_H_ */
