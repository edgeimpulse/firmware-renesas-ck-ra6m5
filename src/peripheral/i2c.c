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
#include "i2c.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"

/* Global Variables -------------------------------------------------------- */
/* Reading I2C call back event through i2c_Master callback */
static volatile bool _i2c_completed = false;
//static volatile rm_comms_event_t i2c_event = RM_COMMS_EVENT_ERROR;

static fsp_err_t validate_i2c_event(void);
static int ei_i2c_final_read(rm_comms_ctrl_t * const p_api_ctrl, uint8_t* read_data, uint16_t bytes);

static void g_comms_i2c_bus0_quick_setup(void);
static void g_comms_i2c_bus0_quick_shutdown(void);

/* Public functions -------------------------------------------------------- */
/**
 * @brief Init I2C peripheral
 * @return
 *
 * @note default speed is 100kHz
 */
int ei_i2c_init(void)
{
    fsp_err_t               err     = FSP_SUCCESS;
    i2c_master_status_t     status;
    i2c_master_instance_t * p_driver_instance = (i2c_master_instance_t *) g_comms_i2c_bus0_extended_cfg.p_driver_instance;

    /* opening IIC master module */
    g_comms_i2c_bus0_quick_shutdown();
    g_comms_i2c_bus0_quick_setup();
    R_IIC_MASTER_StatusGet(p_driver_instance->p_ctrl, &status);

    if (status.open != true){
        err = FSP_ERR_NOT_OPEN;
    }

    return (int)err;
}

/**
 *
 * @return
 */
int ei_i2c_deinit(void)
{
    fsp_err_t err     = FSP_SUCCESS;
    i2c_master_status_t     status;
    i2c_master_instance_t * p_driver_instance = (i2c_master_instance_t *) g_comms_i2c_bus0_extended_cfg.p_driver_instance;

    g_comms_i2c_bus0_quick_shutdown();

    R_IIC_MASTER_StatusGet(p_driver_instance->p_ctrl, &status);

    if (status.open != false){
        err = FSP_ERR_IN_USE;
    }

    return (int)err;
}

/**
 *
 * @param address
 * @param data
 * @param bytes
 * @return
 */
int ei_i2c_write(rm_comms_ctrl_t * const p_api_ctrl, uint8_t*data, uint16_t bytes)
{
    fsp_err_t err     = FSP_SUCCESS;

    //R_IIC_MASTER_SlaveAddressSet(&g_i2c_master0_ctrl, address, I2C_MASTER_ADDR_MODE_7BIT);  /* to whom we want to communicate */

    _i2c_completed = false;
    err = RM_COMMS_I2C_Write(p_api_ctrl, data, bytes);  /* address to read */
    if (FSP_SUCCESS == err)
    {
        err = validate_i2c_event();
        /* handle error */
        if(FSP_ERR_TRANSFER_ABORTED == err)
        {
            APP_ERR_PRINT("** POWER_CTL reg I2C write failed!!! ** \r\n");
        }
    }
    /* handle error */
    else
    {
        /* Write API returns itself is not successful */
        APP_ERR_PRINT("** R_IIC_MASTER_Write API failed ** \r\n");
    }

    return (int)err;
}

/**
 *
 * @param address
 * @param read_cmd
 * @param read_data
 * @param bytes
 * @return
 */
int ei_i2c_read_word_command(rm_comms_ctrl_t * const p_api_ctrl, uint16_t read_cmd, uint8_t* read_data, uint16_t bytes, uint32_t delay_ms)
{
    uint8_t tx_buff[2] = {0};
    fsp_err_t err     = FSP_SUCCESS;

    //R_IIC_MASTER_SlaveAddressSet(&g_i2c_master0_ctrl, address, I2C_MASTER_ADDR_MODE_7BIT);  /* to whom we want to communicate */

    tx_buff[0] = (uint8_t)((read_cmd & 0xFF00) >> 8);
    tx_buff[1] = (uint8_t)(read_cmd & 0x00FF);

    _i2c_completed = false;
    err = RM_COMMS_I2C_Write(p_api_ctrl, tx_buff, 2u);  /* address to read */

    if (FSP_SUCCESS == err)
    {
        if (delay_ms != 0)
        {
            R_BSP_SoftwareDelay(delay_ms, BSP_DELAY_UNITS_MILLISECONDS);

        }

        err = ei_i2c_final_read(p_api_ctrl, read_data, bytes);
    }
    /* handle error */
    else
    {
        /* Write API returns itself is not successful */
        APP_ERR_PRINT("** R_IIC_MASTER_Write API failed ** \r\n");
    }

    return (int)err;
}

/**
 *
 * @param address
 * @param data
 * @param bytes
 * @return
 */
int ei_i2c_read_byte_command(rm_comms_ctrl_t * const p_api_ctrl, uint8_t read_cmd, uint8_t* read_data, uint16_t bytes, uint32_t delay_ms)
{
    uint8_t tx_buff[2] = {0};
    fsp_err_t err     = FSP_SUCCESS;

    //R_IIC_MASTER_SlaveAddressSet(&g_i2c_master0_ctrl, address, I2C_MASTER_ADDR_MODE_7BIT);  /* to whom we want to communicate */

    tx_buff[0] = read_cmd;

    _i2c_completed = false;
    err = RM_COMMS_I2C_Write(p_api_ctrl, tx_buff, 1u);  /* address to read */

    if (FSP_SUCCESS == err)
    {
        if (delay_ms != 0)
        {
            R_BSP_SoftwareDelay(delay_ms, BSP_DELAY_UNITS_MILLISECONDS);
        }

        err = ei_i2c_final_read(p_api_ctrl, read_data, bytes);
    }
    /* handle error */
    else
    {
        /* Write API returns itself is not successful */
        APP_ERR_PRINT("** R_IIC_MASTER_Write API failed ** \r\n");
    }

    return (int)err;
}

/**
 *
 * @param p_args
 */
void comms_i2c_callback(rm_comms_callback_args_t *p_args)
{
    if (NULL != p_args)
    {
        /* capture callback event for validating the i2c transfer event*/
        if (p_args->event == RM_COMMS_EVENT_OPERATION_COMPLETE)
        {
            _i2c_completed = true;
        }
    }
}
/* Private functions -------------------------------------------------------- */
/**
 *
 * @return
 */
static fsp_err_t validate_i2c_event(void)
{
    uint16_t local_time_out = UINT16_MAX;

    do
    {
        /* This is to avoid infinite loop */
        --local_time_out;

        if(RESET_VALUE == local_time_out)
        {
            return FSP_ERR_TRANSFER_ABORTED;
        }

    }while(_i2c_completed == false);

    if(_i2c_completed == true)
    {
        return FSP_SUCCESS;
    }

    return FSP_ERR_TRANSFER_ABORTED;
}

/* Quick setup for g_comms_i2c_bus0. */
static void g_comms_i2c_bus0_quick_setup(void)
{
    fsp_err_t err;
    i2c_master_instance_t * p_driver_instance = (i2c_master_instance_t *) g_comms_i2c_bus0_extended_cfg.p_driver_instance;

    /* Open I2C driver, this must be done before calling any COMMS API */
    err = p_driver_instance->p_api->open(p_driver_instance->p_ctrl, p_driver_instance->p_cfg);
    assert(FSP_SUCCESS == err);

#if BSP_CFG_RTOS
    /* Create a semaphore for blocking if a semaphore is not NULL */
    if (NULL != g_comms_i2c_bus0_extended_cfg.p_blocking_semaphore)
    {
#if BSP_CFG_RTOS == 1 // AzureOS
        tx_semaphore_create(g_comms_i2c_bus0_extended_cfg.p_blocking_semaphore->p_semaphore_handle,
                            g_comms_i2c_bus0_extended_cfg.p_blocking_semaphore->p_semaphore_name,
                            (ULONG) 0);
#elif BSP_CFG_RTOS == 2 // FreeRTOS
        *(g_comms_i2c_bus0_extended_cfg.p_blocking_semaphore->p_semaphore_handle)
            = xSemaphoreCreateCountingStatic((UBaseType_t) 1, (UBaseType_t) 0, g_comms_i2c_bus0_extended_cfg.p_blocking_semaphore->p_semaphore_memory);
#endif
    }

    /* Create a recursive mutex for bus lock if a recursive mutex is not NULL */
    if (NULL != g_comms_i2c_bus0_extended_cfg.p_bus_recursive_mutex)
    {
#if BSP_CFG_RTOS == 1 // AzureOS
        tx_mutex_create(g_comms_i2c_bus0_extended_cfg.p_bus_recursive_mutex->p_mutex_handle,
                        g_comms_i2c_bus0_extended_cfg.p_bus_recursive_mutex->p_mutex_name,
                        TX_INHERIT);
#elif BSP_CFG_RTOS == 2 // FreeRTOS
        *(g_comms_i2c_bus0_extended_cfg.p_bus_recursive_mutex->p_mutex_handle)
            = xSemaphoreCreateRecursiveMutexStatic(g_comms_i2c_bus0_extended_cfg.p_bus_recursive_mutex->p_mutex_memory);
#endif
    }
#endif
}

/* Quick shutdown for g_comms_i2c_bus0. */
void g_comms_i2c_bus0_quick_shutdown(void)
{
    i2c_master_instance_t * p_driver_instance = (i2c_master_instance_t *) g_comms_i2c_bus0_extended_cfg.p_driver_instance;

    /* Close I2C driver */
    p_driver_instance->p_api->close(p_driver_instance->p_ctrl);

#if BSP_CFG_RTOS
    /* Delete a semaphore for blocking if a semaphore is not NULL */
    if (NULL != g_comms_i2c_bus0_extended_cfg.p_blocking_semaphore)
    {
#if BSP_CFG_RTOS == 1 // AzureOS
        tx_semaphore_delete(g_comms_i2c_bus0_extended_cfg.p_blocking_semaphore->p_semaphore_handle);
#elif BSP_CFG_RTOS == 2 // FreeRTOS
        vSemaphoreDelete(*(g_comms_i2c_bus0_extended_cfg.p_blocking_semaphore->p_semaphore_handle));
#endif
    }

    /* Delete a recursive mutex for bus lock if a recursive mutex is not NULL */
    if (NULL != g_comms_i2c_bus0_extended_cfg.p_bus_recursive_mutex)
    {
#if BSP_CFG_RTOS == 1 // AzureOS
        tx_mutex_delete(g_comms_i2c_bus0_extended_cfg.p_bus_recursive_mutex->p_mutex_handle);
#elif BSP_CFG_RTOS == 2 // FreeRTOS
        vSemaphoreDelete(*(g_comms_i2c_bus0_extended_cfg.p_bus_recursive_mutex->p_mutex_handle));
#endif
    }
#endif
}

/**
 *
 * @param read_data
 * @param bytes
 * @return
 */
static int ei_i2c_final_read(rm_comms_ctrl_t * const p_api_ctrl, uint8_t* read_data, uint16_t bytes)
{
    fsp_err_t err     = FSP_SUCCESS;

    err = validate_i2c_event();
    /* handle error */
    if(FSP_ERR_TRANSFER_ABORTED == err)
    {
        APP_ERR_PRINT("** POWER_CTL reg I2C write failed!!! ** \r\n");
    }
    else
    {
        _i2c_completed = false;
        err  = RM_COMMS_I2C_Read(p_api_ctrl, read_data, bytes);

        /* handle error */
        if (FSP_SUCCESS != err)
        {
            APP_ERR_PRINT("** R_IIC_MASTER_Read API failed ** \r\n");
            //  Do nothing, the error is returned in the end
        }
        else
        {
            err = validate_i2c_event();
            /* handle error */
            if(FSP_ERR_TRANSFER_ABORTED == err)
            {
                APP_ERR_PRINT("ERR ** I2C read failed ** \r\n");
            }
        }
    }

    return (int)err;
}
