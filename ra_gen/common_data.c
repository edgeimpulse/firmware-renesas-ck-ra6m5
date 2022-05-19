/* generated common source file - do not edit */
#include "common_data.h"
iic_master_instance_ctrl_t g_i2c_master0_ctrl;
const iic_master_extended_cfg_t g_i2c_master0_extend =
{ .timeout_mode = IIC_MASTER_TIMEOUT_MODE_SHORT, .timeout_scl_low = IIC_MASTER_TIMEOUT_SCL_LOW_ENABLED,
/* Actual calculated bitrate: 98425. Actual calculated duty cycle: 50%. */.clock_settings.brl_value = 28,
  .clock_settings.brh_value = 28, .clock_settings.cks_value = 3, };
const i2c_master_cfg_t g_i2c_master0_cfg =
{ .channel = 0, .rate = I2C_MASTER_RATE_STANDARD, .slave = 0x00, .addr_mode = I2C_MASTER_ADDR_MODE_7BIT,
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
  .p_callback = rm_comms_i2c_callback,
  .p_context = NULL,
#if defined(VECTOR_NUMBER_IIC0_RXI)
    .rxi_irq             = VECTOR_NUMBER_IIC0_RXI,
#else
  .rxi_irq = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_IIC0_TXI)
    .txi_irq             = VECTOR_NUMBER_IIC0_TXI,
#else
  .txi_irq = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_IIC0_TEI)
    .tei_irq             = VECTOR_NUMBER_IIC0_TEI,
#else
  .tei_irq = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_IIC0_ERI)
    .eri_irq             = VECTOR_NUMBER_IIC0_ERI,
#else
  .eri_irq = FSP_INVALID_VECTOR,
#endif
  .ipl = (2),
  .p_extend = &g_i2c_master0_extend, };
/* Instance structure to use this module. */
const i2c_master_instance_t g_i2c_master0 =
{ .p_ctrl = &g_i2c_master0_ctrl, .p_cfg = &g_i2c_master0_cfg, .p_api = &g_i2c_master_on_iic };
#if BSP_CFG_RTOS
#if BSP_CFG_RTOS == 1
#if !defined(NULL)
TX_MUTEX NULL_handle;
CHAR NULL_name[] = "g_comms_i2c_bus0 recursive mutex";
#endif
#if !defined(NULL)
TX_SEMAPHORE NULL_handle;
CHAR NULL_name[] = "g_comms_i2c_bus0 blocking semaphore";
#endif
#elif BSP_CFG_RTOS == 2
#if !defined(NULL)
SemaphoreHandle_t NULL_handle;
StaticSemaphore_t NULL_memory;
#endif
#if !defined(NULL)
SemaphoreHandle_t NULL_handle;
StaticSemaphore_t NULL_memory;
#endif
#endif

#if !defined(NULL)
/* Recursive Mutex for I2C bus */
rm_comms_i2c_mutex_t NULL =
{
    .p_mutex_handle = &NULL_handle,
#if BSP_CFG_RTOS == 1 // ThradX
    .p_mutex_name = &NULL_name[0],
#elif BSP_CFG_RTOS == 2 // FreeRTOS
    .p_mutex_memory = &NULL_memory,
#endif
};
#endif

#if !defined(NULL)
/* Semaphore for blocking */
rm_comms_i2c_semaphore_t NULL =
{
    .p_semaphore_handle = &NULL_handle,
#if BSP_CFG_RTOS == 1 // ThreadX
    .p_semaphore_name = &NULL_name[0],
#elif BSP_CFG_RTOS == 2 // FreeRTOS
    .p_semaphore_memory = &NULL_memory,
#endif
};
#endif
#endif

/* Shared I2C Bus */
#define RA_NOT_DEFINED (1)
rm_comms_i2c_bus_extended_cfg_t g_comms_i2c_bus0_extended_cfg =
{
#if !defined(g_i2c_master0)
  .p_driver_instance = (void*) &g_i2c_master0,
#elif !defined(RA_NOT_DEFINED)
    .p_driver_instance      = (void*)&RA_NOT_DEFINED,
#endif
  .p_current_ctrl = NULL,
  .bus_timeout = 0xFFFFFFFF,
#if BSP_CFG_RTOS
#if !defined(NULL)
    .p_blocking_semaphore = &NULL,
#if !defined(NULL)
    .p_bus_recursive_mutex = &NULL,
#else
    .p_bus_recursive_mutex = NULL,
#endif
#else
    .p_bus_recursive_mutex = NULL,
    .p_blocking_semaphore = NULL,
#endif
#else
#endif
        };
ioport_instance_ctrl_t g_ioport_ctrl;
const ioport_instance_t g_ioport =
{ .p_api = &g_ioport_on_ioport, .p_ctrl = &g_ioport_ctrl, .p_cfg = &g_bsp_pin_cfg, };
void g_common_init(void)
{
}
