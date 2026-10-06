/**
 * @file      HAL_uart.c
 *
 * @brief     HAL implementation of UART functionalities
 *
 * @author    Qorvo Applications
 *
 * @copyright SPDX-FileCopyrightText: Copyright (c) 2024 Qorvo US, Inc.
 *            SPDX-License-Identifier: LicenseRef-QORVO-2
 *
 */

#include "HAL_uart.h"

#include <stdint.h>
#include <stdbool.h>

#include "boards.h"
#include "nrf_uart.h"
#include "app_uart.h"
#include "circular_buffer.h"
#include "HAL_error.h"
#include "qlog.h"

#define UART_RX_BUF_SIZE 0x200

static struct cc_buff uart_rx_buff = {};
static uint8_t uart_rx_buffer[UART_RX_BUF_SIZE] = {};
static CommRxCallback uart_rx_callback = NULL;

static bool discard_next_symbol = false;
static bool UART_is_down = false;

bool IsUartDown(void)
{
    return UART_is_down;
}

void SetUartDown(bool val)
{
    UART_is_down = val;
}

/**
 * @brief Uart event handler
 * @param p_event Pointer to UART event.
 */
void deca_uart_event_handle(app_uart_evt_t *p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        error_handler(0, _ERR_General_Error);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        error_handler(0, _ERR_General_Error);
    }
    /* This event indicates that UART data has been received  */
    else if (p_event->evt_type == APP_UART_DATA_READY)
    {
        deca_uart_receive();
    }
}

/**
 * @brief Function for initializing the UART module.
 */
void deca_uart_init(CommRxCallback callback)
{
    uint32_t err_code;
    const app_uart_comm_params_t comm_params = {
        UART_0_RX_PIN,
        UART_0_TX_PIN,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud230400};

    assert(callback);
    uart_rx_callback = callback;
    cc_buff_init(&uart_rx_buff, uart_rx_buffer, sizeof(uart_rx_buffer), sizeof(uint8_t));

    /* Tx and Rx buffers have the same size. */
    APP_UART_FIFO_INIT(&comm_params, UART_RX_BUF_SIZE, UART_RX_BUF_SIZE, deca_uart_event_handle, APP_IRQ_PRIORITY_LOW, err_code);
    if (err_code)
        QLOGE("UART: Init failed with error code: %u", err_code);
}

void deca_uart_close(void)
{
    app_uart_flush();
    app_uart_close();
}

/**
 * @brief  Function for transmitting data on UART
 *
 * @param  ptr Pointer is contain base address of data.
 */
int deca_uart_transmit(uint8_t *ptr, uint16_t size)
{
    int ret = NRF_SUCCESS;
    for (int i = 0; i < size; i++)
    {
        if (app_uart_put(ptr[i]) != NRF_SUCCESS)
        {
            ret = _ERR_UART_TX;
            break;
        }
    }
    return ret;
}

/**
 * @brief Function for receive data from UART and store into rx_buf (global array).
 */
void deca_uart_receive(void)
{
    uint8_t rx_data;

    if (app_uart_get(&rx_data))
        return;
    if (discard_next_symbol)
    {
        discard_next_symbol = false;
    }
    else
    {
        if (!cc_buff_write_byte(&uart_rx_buff, rx_data))
            QLOGE("UART: RX buffer overflow");

        uart_rx_callback(&uart_rx_buff);
    }
}

/**
 * @brief Discard next incoming symbol, used while wakening up UART
 * from sleep as the first receiving symbol is a garbage).
 */
void deca_discard_next_symbol(void)
{
    discard_next_symbol = true;
}
