/* generated HAL source file - do not edit */
#include "hal_data.h"

/* I2C Communication Device */
#define RA_NOT_DEFINED (1)
rm_comms_i2c_instance_ctrl_t g_comms_i2c_biometric_ctrl;

/* Lower level driver configuration */
const i2c_master_cfg_t g_comms_i2c_biometric_lower_level_cfg =
{ .slave = 0x53, .addr_mode = I2C_MASTER_ADDR_MODE_7BIT, };

const rm_comms_cfg_t g_comms_i2c_biometric_cfg =
{ .semaphore_timeout = 0xFFFFFFFF, .p_lower_level_cfg = (void*) &g_comms_i2c_biometric_lower_level_cfg, .p_extend =
          (void*) &g_comms_i2c_bus0_extended_cfg,
  .p_callback = comms_i2c_callback,
#if defined(NULL)
    .p_context          = NULL,
#else
  .p_context = (void*) &NULL,
#endif
        };

const rm_comms_instance_t g_comms_i2c_biometric =
{ .p_ctrl = &g_comms_i2c_biometric_ctrl, .p_cfg = &g_comms_i2c_biometric_cfg, .p_api = &g_comms_on_comms_i2c, };
/* I2C Communication Device */
#define RA_NOT_DEFINED (1)
rm_comms_i2c_instance_ctrl_t g_comms_i2c_barometric_ctrl;

/* Lower level driver configuration */
const i2c_master_cfg_t g_comms_i2c_barometric_lower_level_cfg =
{ .slave = 0x63, .addr_mode = I2C_MASTER_ADDR_MODE_7BIT, };

const rm_comms_cfg_t g_comms_i2c_barometric_cfg =
{ .semaphore_timeout = 0xFFFFFFFF, .p_lower_level_cfg = (void*) &g_comms_i2c_barometric_lower_level_cfg, .p_extend =
          (void*) &g_comms_i2c_bus0_extended_cfg,
  .p_callback = comms_i2c_callback,
#if defined(NULL)
    .p_context          = NULL,
#else
  .p_context = (void*) &NULL,
#endif
        };

const rm_comms_instance_t g_comms_i2c_barometric =
{ .p_ctrl = &g_comms_i2c_barometric_ctrl, .p_cfg = &g_comms_i2c_barometric_cfg, .p_api = &g_comms_on_comms_i2c, };
/* I2C Communication Device */
#define RA_NOT_DEFINED (1)
rm_comms_i2c_instance_ctrl_t g_comms_i2c_accelerometer_ctrl;

/* Lower level driver configuration */
const i2c_master_cfg_t g_comms_i2c_accelerometer_lower_level_cfg =
{ .slave = 0x68, .addr_mode = I2C_MASTER_ADDR_MODE_7BIT, };

const rm_comms_cfg_t g_comms_i2c_accelerometer_cfg =
{ .semaphore_timeout = 0xFFFFFFFF, .p_lower_level_cfg = (void*) &g_comms_i2c_accelerometer_lower_level_cfg, .p_extend =
          (void*) &g_comms_i2c_bus0_extended_cfg,
  .p_callback = comms_i2c_callback,
#if defined(NULL)
    .p_context          = NULL,
#else
  .p_context = (void*) &NULL,
#endif
        };

const rm_comms_instance_t g_comms_i2c_accelerometer =
{ .p_ctrl = &g_comms_i2c_accelerometer_ctrl, .p_cfg = &g_comms_i2c_accelerometer_cfg, .p_api = &g_comms_on_comms_i2c, };
/* I2C Communication Device */
#define RA_NOT_DEFINED (1)
rm_comms_i2c_instance_ctrl_t g_comms_i2c_env_device_ctrl;

/* Lower level driver configuration */
const i2c_master_cfg_t g_comms_i2c_env_device_lower_level_cfg =
{ .slave = 0x44, .addr_mode = I2C_MASTER_ADDR_MODE_7BIT, };

const rm_comms_cfg_t g_comms_i2c_env_device_cfg =
{ .semaphore_timeout = 0xFFFFFFFF, .p_lower_level_cfg = (void*) &g_comms_i2c_env_device_lower_level_cfg, .p_extend =
          (void*) &g_comms_i2c_bus0_extended_cfg,
  .p_callback = rm_hs300x_callback,
#if defined(g_hs300x_sensor0_ctrl)
    .p_context          = g_hs300x_sensor0_ctrl,
#else
  .p_context = (void*) &g_hs300x_sensor0_ctrl,
#endif
        };

const rm_comms_instance_t g_comms_i2c_env_device =
{ .p_ctrl = &g_comms_i2c_env_device_ctrl, .p_cfg = &g_comms_i2c_env_device_cfg, .p_api = &g_comms_on_comms_i2c, };
rm_hs300x_instance_ctrl_t g_hs300x_sensor0_ctrl;
const rm_hs300x_cfg_t g_hs300x_sensor0_cfg =
{ .p_instance = &g_comms_i2c_env_device, .p_callback = hs300x_callback,
#if defined(NULL)
    .p_context           = NULL,
#else
  .p_context = &NULL,
#endif
        };

const rm_hs300x_instance_t g_hs300x_sensor0 =
{ .p_ctrl = &g_hs300x_sensor0_ctrl, .p_cfg = &g_hs300x_sensor0_cfg, .p_api = &g_hs300x_on_hs300x, };
/* I2C Communication Device */
#define RA_NOT_DEFINED (1)
rm_comms_i2c_instance_ctrl_t g_comms_i2c_oaq_device_ctrl;

/* Lower level driver configuration */
const i2c_master_cfg_t g_comms_i2c_oaq_device_lower_level_cfg =
{ .slave = 0x33, .addr_mode = I2C_MASTER_ADDR_MODE_7BIT, };

const rm_comms_cfg_t g_comms_i2c_oaq_device_cfg =
{ .semaphore_timeout = 0xFFFFFFFF, .p_lower_level_cfg = (void*) &g_comms_i2c_oaq_device_lower_level_cfg, .p_extend =
          (void*) &g_comms_i2c_bus0_extended_cfg,
  .p_callback = rm_zmod4xxx_comms_i2c_callback,
#if defined(g_zmod4510_oaq_sensor_ctrl)
    .p_context          = g_zmod4510_oaq_sensor_ctrl,
#else
  .p_context = (void*) &g_zmod4510_oaq_sensor_ctrl,
#endif
        };

const rm_comms_instance_t g_comms_i2c_oaq_device =
{ .p_ctrl = &g_comms_i2c_oaq_device_ctrl, .p_cfg = &g_comms_i2c_oaq_device_cfg, .p_api = &g_comms_on_comms_i2c, };
zmod4xxx_dev_t g_zmod4510_oaq_sensor_dev;
oaq_2nd_gen_handle_t g_zmod4510_oaq_sensor_lib_handle;
oaq_2nd_gen_results_t g_zmod4510_oaq_sensor_lib_results;
uint8_t g_zmod4510_oaq_sensor_product_data[10];
extern rm_zmod4xxx_api_t const g_zmod4xxx_on_zmod4510_oaq_2nd_gen;
extern zmod4xxx_conf g_zmod4510_oaq_2nd_gen_sensor_type[];
rm_zmod4xxx_lib_extended_cfg_t g_zmod4510_oaq_sensor_extended_cfg =
{ .lib_type = RM_ZMOD4510_LIB_TYPE_OAQ_2ND_GEN,
  .product_id = 0x6320,
  .p_api = (void*) &g_zmod4xxx_on_zmod4510_oaq_2nd_gen,
  .p_data_set = (void*) g_zmod4510_oaq_2nd_gen_sensor_type,
  .p_product_data = g_zmod4510_oaq_sensor_product_data,
  .p_device = (void*) &g_zmod4510_oaq_sensor_dev,
  .p_handle = (void*) &g_zmod4510_oaq_sensor_lib_handle,
  .p_results = (void*) &g_zmod4510_oaq_sensor_lib_results, };
rm_zmod4xxx_instance_ctrl_t g_zmod4510_oaq_sensor_ctrl;
const rm_zmod4xxx_cfg_t g_zmod4510_oaq_sensor_cfg =
{ .p_comms_instance = &g_comms_i2c_oaq_device,
#define RA_NOT_DEFINED (1)
#if (RA_NOT_DEFINED == RA_NOT_DEFINED)
  .p_irq_instance = NULL,
  .p_irq_callback = NULL,
#else
    .p_irq_instance = &RA_NOT_DEFINED,
    .p_irq_callback = zmod4510_oaq_irq_callback,
#endif
#undef RA_NOT_DEFINED
  .p_comms_callback = zmod4510_oaq_comms_i2c_callback,
  .p_zmod4xxx_device = (void*) &g_zmod4510_oaq_sensor_dev, // This will be removed in FSP v4.0.0.
  .p_zmod4xxx_handle = (void*) &g_zmod4510_oaq_sensor_lib_handle, // This will be removed in FSP v4.0.0.
  .p_zmod4xxx_results = (void*) &g_zmod4510_oaq_sensor_lib_results, // This will be removed in FSP v4.0.0.
#if defined(NULL)
    .p_context           = NULL,
#else
  .p_context = &NULL,
#endif
  .p_extend = (void*) &g_zmod4510_oaq_sensor_extended_cfg, };

const rm_zmod4xxx_instance_t g_zmod4510_oaq_sensor =
{ .p_ctrl = &g_zmod4510_oaq_sensor_ctrl, .p_cfg = &g_zmod4510_oaq_sensor_cfg, .p_api = &g_zmod4xxx_on_zmod4xxx, };
/* I2C Communication Device */
#define RA_NOT_DEFINED (1)
rm_comms_i2c_instance_ctrl_t g_comms_i2c_iaq_device_ctrl;

/* Lower level driver configuration */
const i2c_master_cfg_t g_comms_i2c_iaq_device_lower_level_cfg =
{ .slave = 0x32, .addr_mode = I2C_MASTER_ADDR_MODE_7BIT, };

const rm_comms_cfg_t g_comms_i2c_iaq_device_cfg =
{ .semaphore_timeout = 0xFFFFFFFF, .p_lower_level_cfg = (void*) &g_comms_i2c_iaq_device_lower_level_cfg, .p_extend =
          (void*) &g_comms_i2c_bus0_extended_cfg,
  .p_callback = rm_zmod4xxx_comms_i2c_callback,
#if defined(g_zmod4410_iaq_sensor_ctrl)
    .p_context          = g_zmod4410_iaq_sensor_ctrl,
#else
  .p_context = (void*) &g_zmod4410_iaq_sensor_ctrl,
#endif
        };

const rm_comms_instance_t g_comms_i2c_iaq_device =
{ .p_ctrl = &g_comms_i2c_iaq_device_ctrl, .p_cfg = &g_comms_i2c_iaq_device_cfg, .p_api = &g_comms_on_comms_i2c, };
zmod4xxx_dev_t g_zmod4410_iaq_sensor_dev;
iaq_2nd_gen_handle_t g_zmod4410_iaq_sensor_lib_handle;
iaq_2nd_gen_results_t g_zmod4410_iaq_sensor_lib_results;
uint8_t g_zmod4410_iaq_sensor_product_data[7];
extern rm_zmod4xxx_api_t const g_zmod4xxx_on_zmod4410_iaq_2nd_gen;
extern zmod4xxx_conf g_zmod4410_iaq_2nd_gen_sensor_type[];
rm_zmod4xxx_lib_extended_cfg_t g_zmod4410_iaq_sensor_extended_cfg =
{ .lib_type = RM_ZMOD4410_LIB_TYPE_IAQ_2ND_GEN,
  .product_id = 0x2310,
  .p_api = (void*) &g_zmod4xxx_on_zmod4410_iaq_2nd_gen,
  .p_data_set = (void*) g_zmod4410_iaq_2nd_gen_sensor_type,
  .p_product_data = g_zmod4410_iaq_sensor_product_data,
  .p_device = (void*) &g_zmod4410_iaq_sensor_dev,
  .p_handle = (void*) &g_zmod4410_iaq_sensor_lib_handle,
  .p_results = (void*) &g_zmod4410_iaq_sensor_lib_results, };
rm_zmod4xxx_instance_ctrl_t g_zmod4410_iaq_sensor_ctrl;
const rm_zmod4xxx_cfg_t g_zmod4410_iaq_sensor_cfg =
{ .p_comms_instance = &g_comms_i2c_iaq_device,
#define RA_NOT_DEFINED (1)
#if (RA_NOT_DEFINED == RA_NOT_DEFINED)
  .p_irq_instance = NULL,
  .p_irq_callback = NULL,
#else
    .p_irq_instance = &RA_NOT_DEFINED,
    .p_irq_callback = zmod4410_iaq_irq_callback,
#endif
#undef RA_NOT_DEFINED
  .p_comms_callback = zmod4410_iaq_comms_i2c_callback,
  .p_zmod4xxx_device = (void*) &g_zmod4410_iaq_sensor_dev, // This will be removed in FSP v4.0.0.
  .p_zmod4xxx_handle = (void*) &g_zmod4410_iaq_sensor_lib_handle, // This will be removed in FSP v4.0.0.
  .p_zmod4xxx_results = (void*) &g_zmod4410_iaq_sensor_lib_results, // This will be removed in FSP v4.0.0.
#if defined(NULL)
    .p_context           = NULL,
#else
  .p_context = &NULL,
#endif
  .p_extend = (void*) &g_zmod4410_iaq_sensor_extended_cfg, };

const rm_zmod4xxx_instance_t g_zmod4410_iaq_sensor =
{ .p_ctrl = &g_zmod4410_iaq_sensor_ctrl, .p_cfg = &g_zmod4410_iaq_sensor_cfg, .p_api = &g_zmod4xxx_on_zmod4xxx, };
gpt_instance_ctrl_t g_timer2_ctrl;
#if 0
const gpt_extended_pwm_cfg_t g_timer2_pwm_extend =
{
    .trough_ipl          = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_GPT2_COUNTER_UNDERFLOW)
    .trough_irq          = VECTOR_NUMBER_GPT2_COUNTER_UNDERFLOW,
#else
    .trough_irq          = FSP_INVALID_VECTOR,
#endif
    .poeg_link           = GPT_POEG_LINK_POEG0,
    .output_disable      =  GPT_OUTPUT_DISABLE_NONE,
    .adc_trigger         =  GPT_ADC_TRIGGER_NONE,
    .dead_time_count_up  = 0,
    .dead_time_count_down = 0,
    .adc_a_compare_match = 0,
    .adc_b_compare_match = 0,
    .interrupt_skip_source = GPT_INTERRUPT_SKIP_SOURCE_NONE,
    .interrupt_skip_count  = GPT_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_adc    = GPT_INTERRUPT_SKIP_ADC_NONE,
    .gtioca_disable_setting = GPT_GTIOC_DISABLE_PROHIBITED,
    .gtiocb_disable_setting = GPT_GTIOC_DISABLE_PROHIBITED,
};
#endif
const gpt_extended_cfg_t g_timer2_extend =
        { .gtioca =
        { .output_enabled = true, .stop_level = GPT_PIN_LEVEL_LOW },
          .gtiocb =
          { .output_enabled = false, .stop_level = GPT_PIN_LEVEL_LOW },
          .start_source = (gpt_source_t) (GPT_SOURCE_NONE), .stop_source = (gpt_source_t) (GPT_SOURCE_NONE), .clear_source =
                  (gpt_source_t) (GPT_SOURCE_NONE),
          .count_up_source = (gpt_source_t) (GPT_SOURCE_NONE), .count_down_source = (gpt_source_t) (GPT_SOURCE_NONE), .capture_a_source =
                  (gpt_source_t) (GPT_SOURCE_NONE),
          .capture_b_source = (gpt_source_t) (GPT_SOURCE_NONE), .capture_a_ipl = (BSP_IRQ_DISABLED), .capture_b_ipl =
                  (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_GPT2_CAPTURE_COMPARE_A)
    .capture_a_irq       = VECTOR_NUMBER_GPT2_CAPTURE_COMPARE_A,
#else
          .capture_a_irq = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_GPT2_CAPTURE_COMPARE_B)
    .capture_b_irq       = VECTOR_NUMBER_GPT2_CAPTURE_COMPARE_B,
#else
          .capture_b_irq = FSP_INVALID_VECTOR,
#endif
          .capture_filter_gtioca = GPT_CAPTURE_FILTER_NONE,
          .capture_filter_gtiocb = GPT_CAPTURE_FILTER_NONE,
#if 0
    .p_pwm_cfg                   = &g_timer2_pwm_extend,
#else
          .p_pwm_cfg = NULL,
#endif
#if 0
    .gtior_setting.gtior_b.gtioa  = (0U << 4U) | (0U << 2U) | (0U << 0U),
    .gtior_setting.gtior_b.oadflt = (uint32_t) GPT_PIN_LEVEL_LOW,
    .gtior_setting.gtior_b.oahld  = 0U,
    .gtior_setting.gtior_b.oae    = (uint32_t) true,
    .gtior_setting.gtior_b.oadf   = (uint32_t) GPT_GTIOC_DISABLE_PROHIBITED,
    .gtior_setting.gtior_b.nfaen  = ((uint32_t) GPT_CAPTURE_FILTER_NONE & 1U),
    .gtior_setting.gtior_b.nfcsa  = ((uint32_t) GPT_CAPTURE_FILTER_NONE >> 1U),
    .gtior_setting.gtior_b.gtiob  = (0U << 4U) | (0U << 2U) | (0U << 0U),
    .gtior_setting.gtior_b.obdflt = (uint32_t) GPT_PIN_LEVEL_LOW,
    .gtior_setting.gtior_b.obhld  = 0U,
    .gtior_setting.gtior_b.obe    = (uint32_t) false,
    .gtior_setting.gtior_b.obdf   = (uint32_t) GPT_GTIOC_DISABLE_PROHIBITED,
    .gtior_setting.gtior_b.nfben  = ((uint32_t) GPT_CAPTURE_FILTER_NONE & 1U),
    .gtior_setting.gtior_b.nfcsb  = ((uint32_t) GPT_CAPTURE_FILTER_NONE >> 1U),
#else
          .gtior_setting.gtior = 0U,
#endif
        };
const timer_cfg_t g_timer2_cfg =
{ .mode = TIMER_MODE_PERIODIC,
/* Actual period: 9.7e-7 seconds. Actual duty: 49.48453608247423%. */.period_counts = (uint32_t) 0x61,
  .duty_cycle_counts = 0x30, .source_div = (timer_source_div_t) 0, .channel = 2, .p_callback = NULL,
  /** If NULL then do not add & */
#if defined(NULL)
    .p_context           = NULL,
#else
  .p_context = &NULL,
#endif
  .p_extend = &g_timer2_extend,
  .cycle_end_ipl = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_GPT2_COUNTER_OVERFLOW)
    .cycle_end_irq       = VECTOR_NUMBER_GPT2_COUNTER_OVERFLOW,
#else
  .cycle_end_irq = FSP_INVALID_VECTOR,
#endif
        };
/* Instance structure to use this module. */
const timer_instance_t g_timer2 =
{ .p_ctrl = &g_timer2_ctrl, .p_cfg = &g_timer2_cfg, .p_api = &g_timer_on_gpt };
gpt_instance_ctrl_t g_timer1_ctrl;
#if 0
const gpt_extended_pwm_cfg_t g_timer1_pwm_extend =
{
    .trough_ipl          = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_GPT1_COUNTER_UNDERFLOW)
    .trough_irq          = VECTOR_NUMBER_GPT1_COUNTER_UNDERFLOW,
#else
    .trough_irq          = FSP_INVALID_VECTOR,
#endif
    .poeg_link           = GPT_POEG_LINK_POEG0,
    .output_disable      =  GPT_OUTPUT_DISABLE_NONE,
    .adc_trigger         =  GPT_ADC_TRIGGER_NONE,
    .dead_time_count_up  = 0,
    .dead_time_count_down = 0,
    .adc_a_compare_match = 0,
    .adc_b_compare_match = 0,
    .interrupt_skip_source = GPT_INTERRUPT_SKIP_SOURCE_NONE,
    .interrupt_skip_count  = GPT_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_adc    = GPT_INTERRUPT_SKIP_ADC_NONE,
    .gtioca_disable_setting = GPT_GTIOC_DISABLE_PROHIBITED,
    .gtiocb_disable_setting = GPT_GTIOC_DISABLE_PROHIBITED,
};
#endif
const gpt_extended_cfg_t g_timer1_extend =
        { .gtioca =
        { .output_enabled = false, .stop_level = GPT_PIN_LEVEL_LOW },
          .gtiocb =
          { .output_enabled = false, .stop_level = GPT_PIN_LEVEL_LOW },
          .start_source = (gpt_source_t) (GPT_SOURCE_NONE), .stop_source = (gpt_source_t) (GPT_SOURCE_NONE), .clear_source =
                  (gpt_source_t) (GPT_SOURCE_NONE),
          .count_up_source = (gpt_source_t) (GPT_SOURCE_NONE), .count_down_source = (gpt_source_t) (GPT_SOURCE_NONE), .capture_a_source =
                  (gpt_source_t) (GPT_SOURCE_NONE),
          .capture_b_source = (gpt_source_t) (GPT_SOURCE_NONE), .capture_a_ipl = (BSP_IRQ_DISABLED), .capture_b_ipl =
                  (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_GPT1_CAPTURE_COMPARE_A)
    .capture_a_irq       = VECTOR_NUMBER_GPT1_CAPTURE_COMPARE_A,
#else
          .capture_a_irq = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_GPT1_CAPTURE_COMPARE_B)
    .capture_b_irq       = VECTOR_NUMBER_GPT1_CAPTURE_COMPARE_B,
#else
          .capture_b_irq = FSP_INVALID_VECTOR,
#endif
          .capture_filter_gtioca = GPT_CAPTURE_FILTER_NONE,
          .capture_filter_gtiocb = GPT_CAPTURE_FILTER_NONE,
#if 0
    .p_pwm_cfg                   = &g_timer1_pwm_extend,
#else
          .p_pwm_cfg = NULL,
#endif
#if 0
    .gtior_setting.gtior_b.gtioa  = (0U << 4U) | (0U << 2U) | (0U << 0U),
    .gtior_setting.gtior_b.oadflt = (uint32_t) GPT_PIN_LEVEL_LOW,
    .gtior_setting.gtior_b.oahld  = 0U,
    .gtior_setting.gtior_b.oae    = (uint32_t) false,
    .gtior_setting.gtior_b.oadf   = (uint32_t) GPT_GTIOC_DISABLE_PROHIBITED,
    .gtior_setting.gtior_b.nfaen  = ((uint32_t) GPT_CAPTURE_FILTER_NONE & 1U),
    .gtior_setting.gtior_b.nfcsa  = ((uint32_t) GPT_CAPTURE_FILTER_NONE >> 1U),
    .gtior_setting.gtior_b.gtiob  = (0U << 4U) | (0U << 2U) | (0U << 0U),
    .gtior_setting.gtior_b.obdflt = (uint32_t) GPT_PIN_LEVEL_LOW,
    .gtior_setting.gtior_b.obhld  = 0U,
    .gtior_setting.gtior_b.obe    = (uint32_t) false,
    .gtior_setting.gtior_b.obdf   = (uint32_t) GPT_GTIOC_DISABLE_PROHIBITED,
    .gtior_setting.gtior_b.nfben  = ((uint32_t) GPT_CAPTURE_FILTER_NONE & 1U),
    .gtior_setting.gtior_b.nfcsb  = ((uint32_t) GPT_CAPTURE_FILTER_NONE >> 1U),
#else
          .gtior_setting.gtior = 0U,
#endif
        };
const timer_cfg_t g_timer1_cfg =
{ .mode = TIMER_MODE_PERIODIC,
/* Actual period: 0.1 seconds. Actual duty: 50%. */.period_counts = (uint32_t) 0x989680,
  .duty_cycle_counts = 0x4c4b40, .source_div = (timer_source_div_t) 0, .channel = 1, .p_callback = timer1_interrupt,
  /** If NULL then do not add & */
#if defined(NULL)
    .p_context           = NULL,
#else
  .p_context = &NULL,
#endif
  .p_extend = &g_timer1_extend,
  .cycle_end_ipl = (2),
#if defined(VECTOR_NUMBER_GPT1_COUNTER_OVERFLOW)
    .cycle_end_irq       = VECTOR_NUMBER_GPT1_COUNTER_OVERFLOW,
#else
  .cycle_end_irq = FSP_INVALID_VECTOR,
#endif
        };
/* Instance structure to use this module. */
const timer_instance_t g_timer1 =
{ .p_ctrl = &g_timer1_ctrl, .p_cfg = &g_timer1_cfg, .p_api = &g_timer_on_gpt };
flash_hp_instance_ctrl_t g_flash0_ctrl;
const flash_cfg_t g_flash0_cfg =
{ .data_flash_bgo = false, .p_callback = NULL, .p_context = NULL,
#if defined(VECTOR_NUMBER_FCU_FRDYI)
    .irq                 = VECTOR_NUMBER_FCU_FRDYI,
#else
  .irq = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_FCU_FIFERR)
    .err_irq             = VECTOR_NUMBER_FCU_FIFERR,
#else
  .err_irq = FSP_INVALID_VECTOR,
#endif
  .err_ipl = (BSP_IRQ_DISABLED),
  .ipl = (BSP_IRQ_DISABLED), };
/* Instance structure to use this module. */
const flash_instance_t g_flash0 =
{ .p_ctrl = &g_flash0_ctrl, .p_cfg = &g_flash0_cfg, .p_api = &g_flash_on_flash_hp };
dtc_instance_ctrl_t g_transfer0_ctrl;

transfer_info_t g_transfer0_info =
{ .dest_addr_mode = TRANSFER_ADDR_MODE_INCREMENTED,
  .repeat_area = TRANSFER_REPEAT_AREA_DESTINATION,
  .irq = TRANSFER_IRQ_END,
  .chain_mode = TRANSFER_CHAIN_MODE_DISABLED,
  .src_addr_mode = TRANSFER_ADDR_MODE_FIXED,
  .size = TRANSFER_SIZE_4_BYTE,
  .mode = TRANSFER_MODE_BLOCK,
  .p_dest = (void*) NULL,
  .p_src = (void const*) NULL,
  .num_blocks = 0,
  .length = 0, };
const dtc_extended_cfg_t g_transfer0_cfg_extend =
{ .activation_source = VECTOR_NUMBER_SSI0_RXI, };
const transfer_cfg_t g_transfer0_cfg =
{ .p_info = &g_transfer0_info, .p_extend = &g_transfer0_cfg_extend, };

/* Instance structure to use this module. */
const transfer_instance_t g_transfer0 =
{ .p_ctrl = &g_transfer0_ctrl, .p_cfg = &g_transfer0_cfg, .p_api = &g_transfer_on_dtc };
ssi_instance_ctrl_t g_i2s0_ctrl;

/** SSI instance configuration */
const ssi_extended_cfg_t g_i2s0_cfg_extend =
{ .audio_clock = SSI_AUDIO_CLOCK_INTERNAL, .bit_clock_div = SSI_CLOCK_DIV_1, };

/** I2S interface configuration */
const i2s_cfg_t g_i2s0_cfg =
{ .channel = 0, .pcm_width = I2S_PCM_WIDTH_32_BITS, .operating_mode = I2S_MODE_MASTER, .word_length =
          I2S_WORD_LENGTH_32_BITS,
  .ws_continue = I2S_WS_CONTINUE_OFF, .p_callback = i2s_callback, .p_context = NULL, .p_extend = &g_i2s0_cfg_extend,
#if (BSP_IRQ_DISABLED) != BSP_IRQ_DISABLED
  #if 0 == 0
                .txi_irq                 = VECTOR_NUMBER_SSI0_TXI,
  #else
                .txi_irq                 = VECTOR_NUMBER_SSI0_TXI_RXI,
  #endif
#else
  .txi_irq = FSP_INVALID_VECTOR,
#endif
#if (0) != BSP_IRQ_DISABLED
  #if 0 == 0
                .rxi_irq                 = VECTOR_NUMBER_SSI0_RXI,
  #else
                .rxi_irq                 = VECTOR_NUMBER_SSI0_TXI_RXI,
  #endif
#else
  .rxi_irq = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_SSI0_INT)
                .int_irq                 = VECTOR_NUMBER_SSI0_INT,
#else
  .int_irq = FSP_INVALID_VECTOR,
#endif
  .txi_ipl = (BSP_IRQ_DISABLED),
  .rxi_ipl = (0), .idle_err_ipl = (0),
#define RA_NOT_DEFINED (1)
#if (RA_NOT_DEFINED == RA_NOT_DEFINED)
  .p_transfer_tx = NULL,
#else
                .p_transfer_tx       = &RA_NOT_DEFINED,
#endif
#if (RA_NOT_DEFINED == g_transfer0)
                .p_transfer_rx       = NULL,
#else
  .p_transfer_rx = &g_transfer0,
#endif
#undef RA_NOT_DEFINED
        };

/* Instance structure to use this module. */
const i2s_instance_t g_i2s0 =
{ .p_ctrl = &g_i2s0_ctrl, .p_cfg = &g_i2s0_cfg, .p_api = &g_i2s_on_ssi };
gpt_instance_ctrl_t g_timer0_ctrl;
#if 0
const gpt_extended_pwm_cfg_t g_timer0_pwm_extend =
{
    .trough_ipl          = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_GPT0_COUNTER_UNDERFLOW)
    .trough_irq          = VECTOR_NUMBER_GPT0_COUNTER_UNDERFLOW,
#else
    .trough_irq          = FSP_INVALID_VECTOR,
#endif
    .poeg_link           = GPT_POEG_LINK_POEG0,
    .output_disable      =  GPT_OUTPUT_DISABLE_NONE,
    .adc_trigger         =  GPT_ADC_TRIGGER_NONE,
    .dead_time_count_up  = 0,
    .dead_time_count_down = 0,
    .adc_a_compare_match = 0,
    .adc_b_compare_match = 0,
    .interrupt_skip_source = GPT_INTERRUPT_SKIP_SOURCE_NONE,
    .interrupt_skip_count  = GPT_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_adc    = GPT_INTERRUPT_SKIP_ADC_NONE,
    .gtioca_disable_setting = GPT_GTIOC_DISABLE_PROHIBITED,
    .gtiocb_disable_setting = GPT_GTIOC_DISABLE_PROHIBITED,
};
#endif
const gpt_extended_cfg_t g_timer0_extend =
        { .gtioca =
        { .output_enabled = false, .stop_level = GPT_PIN_LEVEL_LOW },
          .gtiocb =
          { .output_enabled = false, .stop_level = GPT_PIN_LEVEL_LOW },
          .start_source = (gpt_source_t) (GPT_SOURCE_NONE), .stop_source = (gpt_source_t) (GPT_SOURCE_NONE), .clear_source =
                  (gpt_source_t) (GPT_SOURCE_NONE),
          .count_up_source = (gpt_source_t) (GPT_SOURCE_NONE), .count_down_source = (gpt_source_t) (GPT_SOURCE_NONE), .capture_a_source =
                  (gpt_source_t) (GPT_SOURCE_NONE),
          .capture_b_source = (gpt_source_t) (GPT_SOURCE_NONE), .capture_a_ipl = (BSP_IRQ_DISABLED), .capture_b_ipl =
                  (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_GPT0_CAPTURE_COMPARE_A)
    .capture_a_irq       = VECTOR_NUMBER_GPT0_CAPTURE_COMPARE_A,
#else
          .capture_a_irq = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_GPT0_CAPTURE_COMPARE_B)
    .capture_b_irq       = VECTOR_NUMBER_GPT0_CAPTURE_COMPARE_B,
#else
          .capture_b_irq = FSP_INVALID_VECTOR,
#endif
          .capture_filter_gtioca = GPT_CAPTURE_FILTER_NONE,
          .capture_filter_gtiocb = GPT_CAPTURE_FILTER_NONE,
#if 0
    .p_pwm_cfg                   = &g_timer0_pwm_extend,
#else
          .p_pwm_cfg = NULL,
#endif
#if 0
    .gtior_setting.gtior_b.gtioa  = (0U << 4U) | (0U << 2U) | (0U << 0U),
    .gtior_setting.gtior_b.oadflt = (uint32_t) GPT_PIN_LEVEL_LOW,
    .gtior_setting.gtior_b.oahld  = 0U,
    .gtior_setting.gtior_b.oae    = (uint32_t) false,
    .gtior_setting.gtior_b.oadf   = (uint32_t) GPT_GTIOC_DISABLE_PROHIBITED,
    .gtior_setting.gtior_b.nfaen  = ((uint32_t) GPT_CAPTURE_FILTER_NONE & 1U),
    .gtior_setting.gtior_b.nfcsa  = ((uint32_t) GPT_CAPTURE_FILTER_NONE >> 1U),
    .gtior_setting.gtior_b.gtiob  = (0U << 4U) | (0U << 2U) | (0U << 0U),
    .gtior_setting.gtior_b.obdflt = (uint32_t) GPT_PIN_LEVEL_LOW,
    .gtior_setting.gtior_b.obhld  = 0U,
    .gtior_setting.gtior_b.obe    = (uint32_t) false,
    .gtior_setting.gtior_b.obdf   = (uint32_t) GPT_GTIOC_DISABLE_PROHIBITED,
    .gtior_setting.gtior_b.nfben  = ((uint32_t) GPT_CAPTURE_FILTER_NONE & 1U),
    .gtior_setting.gtior_b.nfcsb  = ((uint32_t) GPT_CAPTURE_FILTER_NONE >> 1U),
#else
          .gtior_setting.gtior = 0U,
#endif
        };
const timer_cfg_t g_timer0_cfg =
{ .mode = TIMER_MODE_PERIODIC,
/* Actual period: 0.001 seconds. Actual duty: 50%. */.period_counts = (uint32_t) 0x186a0,
  .duty_cycle_counts = 0xc350, .source_div = (timer_source_div_t) 0, .channel = 0, .p_callback = periodic_timer_msgq_cb,
  /** If NULL then do not add & */
#if defined(NULL)
    .p_context           = NULL,
#else
  .p_context = &NULL,
#endif
  .p_extend = &g_timer0_extend,
  .cycle_end_ipl = (2),
#if defined(VECTOR_NUMBER_GPT0_COUNTER_OVERFLOW)
    .cycle_end_irq       = VECTOR_NUMBER_GPT0_COUNTER_OVERFLOW,
#else
  .cycle_end_irq = FSP_INVALID_VECTOR,
#endif
        };
/* Instance structure to use this module. */
const timer_instance_t g_timer0 =
{ .p_ctrl = &g_timer0_ctrl, .p_cfg = &g_timer0_cfg, .p_api = &g_timer_on_gpt };
sci_uart_instance_ctrl_t g_uart3_ctrl;

baud_setting_t g_uart3_baud_setting =
{
/* Baud rate calculated with 0.469% error. */.abcse = 0,
  .abcs = 0, .bgdm = 1, .cks = 0, .brr = 53, .mddr = (uint8_t) 256, .brme = false };

/** UART extended configuration for UARTonSCI HAL driver */
const sci_uart_extended_cfg_t g_uart3_cfg_extend =
{ .clock = SCI_UART_CLOCK_INT, .rx_edge_start = SCI_UART_START_BIT_FALLING_EDGE, .noise_cancel =
          SCI_UART_NOISE_CANCELLATION_DISABLE,
  .rx_fifo_trigger = SCI_UART_RX_FIFO_TRIGGER_MAX, .p_baud_setting = &g_uart3_baud_setting, .flow_control =
          SCI_UART_FLOW_CONTROL_RTS,
#if 0xFF != 0xFF
                .flow_control_pin       = BSP_IO_PORT_FF_PIN_0xFF,
                #else
  .flow_control_pin = (bsp_io_port_pin_t) UINT16_MAX,
#endif
        };

/** UART interface configuration */
const uart_cfg_t g_uart3_cfg =
{ .channel = 3, .data_bits = UART_DATA_BITS_8, .parity = UART_PARITY_OFF, .stop_bits = UART_STOP_BITS_1, .p_callback =
          user_uart_callback,
  .p_context = NULL, .p_extend = &g_uart3_cfg_extend,
#define RA_NOT_DEFINED (1)
#if (RA_NOT_DEFINED == RA_NOT_DEFINED)
  .p_transfer_tx = NULL,
#else
                .p_transfer_tx       = &RA_NOT_DEFINED,
#endif
#if (RA_NOT_DEFINED == RA_NOT_DEFINED)
  .p_transfer_rx = NULL,
#else
                .p_transfer_rx       = &RA_NOT_DEFINED,
#endif
#undef RA_NOT_DEFINED
  .rxi_ipl = (3),
  .txi_ipl = (12), .tei_ipl = (4), .eri_ipl = (12),
#if defined(VECTOR_NUMBER_SCI3_RXI)
                .rxi_irq             = VECTOR_NUMBER_SCI3_RXI,
#else
  .rxi_irq = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_SCI3_TXI)
                .txi_irq             = VECTOR_NUMBER_SCI3_TXI,
#else
  .txi_irq = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_SCI3_TEI)
                .tei_irq             = VECTOR_NUMBER_SCI3_TEI,
#else
  .tei_irq = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_SCI3_ERI)
                .eri_irq             = VECTOR_NUMBER_SCI3_ERI,
#else
  .eri_irq = FSP_INVALID_VECTOR,
#endif
        };

/* Instance structure to use this module. */
const uart_instance_t g_uart3 =
{ .p_ctrl = &g_uart3_ctrl, .p_cfg = &g_uart3_cfg, .p_api = &g_uart_on_sci };
void g_hal_init(void)
{
    g_common_init ();
}
