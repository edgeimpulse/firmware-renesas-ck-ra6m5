/***********************************************************************************************************************
 * File Name    : uart.c
 * Description  : Contains UART functions definition.
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * DISCLAIMER
 * This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products. No
 * other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
 * applicable laws, including copyright laws.
 * THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING
 * THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED. TO THE MAXIMUM
 * EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES
 * SHALL BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO THIS
 * SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 * Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability of
 * this software. By using this software, you agree to the additional terms and conditions found by accessing the
 * following link:
 * http://www.renesas.com/disclaimer
 *
 * Copyright (C) 2020 Renesas Electronics Corporation. All rights reserved.
 ***********************************************************************************************************************/
#include "uart_ep.h"

/*******************************************************************************************************************//**
 * @addtogroup r_sci_uart_ep
 * @{
 **********************************************************************************************************************/

#define UART_RX_BUFFER_SIZE         512

/*
 * Private function declarations
 */

/*
 * Private global variables
 */
/* Temporary buffer to save data from receive buffer for further processing */
static uint8_t g_temp_buffer[UART_RX_BUFFER_SIZE] = {RESET_VALUE};

/* Flag to check whether data is received or not */
//static volatile uint8_t g_data_received_flag = false;

/* Flag for user callback */
static volatile uint8_t g_uart_event = RESET_VALUE;

/* Flag RX completed */
static volatile uint8_t g_uart_rx_completed = false;

/* Flag TX completed */
static volatile uint8_t g_uart_tx_completed = false;
static volatile uint8_t g_uart_tx_empty = true;

/* Flag error occurred */
static volatile uint8_t g_uart_error = false;

/* Counter to update g_temp_buffer index */
static volatile uint16_t g_rx_index = RESET_VALUE;

/* Index of data sent to at hanlder */
static volatile uint16_t g_uart_rx_read_index = RESET_VALUE;

/*******************************************************************************************************************//**
 * @brief       Initialize  UART.
 * @param[in]   None
 * @retval      FSP_SUCCESS         Upon successful open and start of timer
 * @retval      Any Other Error code apart from FSP_SUCCESS  Unsuccessful open
 ***********************************************************************************************************************/
fsp_err_t uart_initialize(void)
{
    fsp_err_t err = FSP_SUCCESS;

    /* Initialize UART channel with baud rate 115200 */
    err = R_SCI_UART_Open (&g_uart3_ctrl, &g_uart3_cfg);
    if (FSP_SUCCESS != err)
    {
        APP_ERR_PRINT ("\r\n**  R_SCI_UART_Open API failed  **\r\n");
    }
    return err;
}

/**
 *
 * @param p_msg
 * @param msg_len
 * @return
 */
fsp_err_t uart_print_user_msg(uint8_t *p_msg, uint16_t msg_len)
{
    fsp_err_t err   = FSP_SUCCESS;

    /* Reset callback capture variable */
    g_uart_tx_completed = false;
    g_uart_error = false;

    err = R_SCI_UART_Write (&g_uart3_ctrl, p_msg, msg_len);
    if (FSP_SUCCESS != err) {
        return err;
    }

    /* Check for event transfer complete */
    while (g_uart_tx_completed == false) {
        /* Check if any error event occurred */
        if (g_uart_error == true) {
            return FSP_ERR_TRANSFER_ABORTED;
        }

    }

    return err;
}

/**
 *
 * @param c
 */
void uart_putc(uint8_t c)
{
    g_uart_tx_empty = false;
    R_SCI_UART_Write(&g_uart3_ctrl, &c, 1);
    while(g_uart_tx_empty == false){};
}

/**
 * @brief Returns the local rx uart buffer only if reception is complete (ie received a '\r\n') or if inference is running (just care of 'b')
 *
 * @param is_inference_running
 * @return
 */
char uart_get_rx_data(uint8_t is_inference_running)
{
    char ret_val = -1;

    if (g_uart_rx_completed == true)
    {
        if (g_uart_rx_read_index < g_rx_index)
        {
            ret_val = (char)g_temp_buffer[g_uart_rx_read_index++];

            if (g_uart_rx_read_index == g_rx_index)
            {
                g_rx_index = 0;
                g_uart_rx_read_index = 0;
                g_uart_rx_completed = false;
            }
        }
        else
        {
            g_rx_index = 0;
            g_uart_rx_read_index = 0;
        }
    }
    else if (is_inference_running)
    {
        /* need to check if any 'b' */
        uint16_t i;

        for (i = g_uart_rx_read_index; i< g_rx_index; i ++)
        {
            if (g_temp_buffer[i] == 'b')
            {
                ret_val = 'b';
                break;
            }
        }
    }

    return ret_val;
}

/*******************************************************************************************************************//**
 *  @brief       Deinitialize SCI UART module
 *  @param[in]   None
 *  @retval      None
 **********************************************************************************************************************/
void deinit_uart(void)
{
    fsp_err_t err = FSP_SUCCESS;

    /* Close module */
    err =  R_SCI_UART_Close (&g_uart3_ctrl);
    if (FSP_SUCCESS != err)
    {
        APP_ERR_PRINT ("\r\n**  R_SCI_UART_Close API failed  ** \r\n");
    }
}

/*****************************************************************************************************************
 *  @brief      UART user callback
 *  @param[in]  p_args
 *  @retval     None
 ****************************************************************************************************************/
void user_uart_callback(uart_callback_args_t *p_args)
{
    /* Logged the event in global variable */
    g_uart_event = (uint8_t)p_args->event;

    /* Reset g_temp_buffer index if it exceeds than buffer size */
    if(sizeof(g_temp_buffer) == g_rx_index)
    {
        g_rx_index = RESET_VALUE;
    }

    switch(p_args->event)
    {
        case UART_EVENT_RX_CHAR:
        {
            g_temp_buffer[g_rx_index++] = (uint8_t ) p_args->data;
            if (p_args->data == CARRIAGE_ASCII)
            {
                //g_counter_var = RESET_VALUE;
                //g_data_received_flag  = true;
                g_uart_rx_completed = true;
            }
        }
        break;
        case UART_EVENT_RX_COMPLETE:
        {
            g_uart_rx_completed = true;
        }
        break;
        case UART_EVENT_TX_COMPLETE:
        {
            g_uart_tx_completed = true;
        }
        break;
        case UART_EVENT_TX_DATA_EMPTY:
        {
            g_uart_tx_empty = true;
        }
        break;
        case UART_EVENT_ERR_PARITY:
        case UART_EVENT_ERR_FRAMING:
        case UART_EVENT_ERR_OVERFLOW:
        case UART_EVENT_BREAK_DETECT:
        {
            g_uart_error = true;
        }
        break;
        default:
        {

        }
        break;
    }
}

/*******************************************************************************************************************//**
 * @} (end addtogroup r_sci_uart_ep)
 **********************************************************************************************************************/
