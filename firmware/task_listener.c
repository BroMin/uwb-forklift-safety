/**
 * @file      task_listener.c
 *
 * @brief     Listener task functionalities
 *
 * @author    Qorvo Applications
 *
 * @copyright SPDX-FileCopyrightText: Copyright (c) 2024 Qorvo US, Inc.
 *            SPDX-License-Identifier: LicenseRef-QORVO-2
 *
 */

#include <math.h>

#include "app.h"
#include "usb_uart_tx.h"
#include "listener.h"
#include "task_signal.h"
#include "deca_dbg.h"
#include "HAL_error.h"
#include "circular_buffer.h"
#include "usb_uart_tx.h"
#include "cmd_fn.h"
#include "flushTask.h"
#include "cmd.h"
#include "int_priority.h"
#include "qmalloc.h"
#include "qirq.h"
#include "qplatform.h"
#include "qpwr.h"

static task_signal_t listenerTask;

extern const struct command_s known_subcommands_listener;

#define LISTENER_TASK_STACK_SIZE_BYTES 2048
#define MAX_PRINT_FAST_LISTENER        (6)

/**
 * @brief function to report to PC the Listener data received
 * 'JSxxxx{"LSTN":[RxBytes_hex,..,],"TS":"0xTimeStamp32_Hex","O":Offset_dec}'
 * This fn() uses pseudo-JSON format, it appends "+" to the end of.
 */
error_e send_to_pc_listener_info(uint8_t *data, uint8_t size, uint8_t *ts, int16_t cfo, int mode, int rsl100, int fsl100)
{
    error_e ret = _ERR_Cannot_Alloc_Memory;

    uint32_t cnt, flag_plus = 0;
    uint16_t hlen;
    int cfo_pphm;
    char *str;
    mode = 1; //모드 1로 고정하기.

    if (mode == 0)
    {
        /* Speed is a priority. */
        if (size > MAX_PRINT_FAST_LISTENER)
        {
            flag_plus = 1;
            size = MAX_PRINT_FAST_LISTENER;
        }

        str = qmalloc(MAX_STR_SIZE);
    }
    else
    {
        str = qmalloc(MAX_STR_SIZE + MAX_STR_SIZE);
    }

    /* 21 is an overhead. */
    size = MIN((sizeof(str) - 21) / 3, size);

    if (str)
    {
        cfo_pphm = (int)((float)cfo * (CLOCK_OFFSET_PPM_TO_RATIO * 1e6 * 100));
        /* Reserve space for length of JS object. */
        hlen = sprintf(str, "JS%04X", 0x5A5A);
        sprintf(&str[strlen(str)], "{\"LSTN\":[");

        /* Loop over the received data. */
        for (cnt = 0; cnt < size; cnt++)
        {
            /* Add the byte and the delimiter - "XX,". */
            sprintf(&str[strlen(str)], "%02X,", data[cnt]);
        }

        if (flag_plus)
        {
            sprintf(&str[strlen(str)], "+,");
        }

        sprintf(&str[strlen(str) - 1], "],\"TS4ns\":\"0x%02X%02X%02X%02X\",", ts[4], ts[3], ts[2], ts[1]);
        sprintf(&str[strlen(str)], "\"O\":%d", cfo_pphm);
        sprintf(&str[strlen(str)], ",\"rsl\":%d.%02d,\"fsl\":%d.%02d", rsl100 / 100, (rsl100 * -1) % 100, fsl100 / 100, (fsl100 * -1) % 100);
        sprintf(&str[strlen(str)], "%s", "}\r\n");
        /* Add formatted 4X of length, this will erase first '{'. */
        sprintf(&str[2], "%04X", strlen(str) - hlen);
        /* Restore the start bracket. */
        str[hlen] = '{';
        /* Do not notify flush task, only copy the message. */
        ret = copy_tx_msg((uint8_t *)str, strlen(str));

        qfree(str);
    }

    return (ret);
}

/**
 * @brief DW3000 RX : Listener RTOS implementation
 * This is a high-priority task, which will be executed immediately
 * on reception of waiting Signal. Any task with lower priority will be interrupted.
 * No other tasks in the system should have higher priority.
 */
static void ListenerTask(void *arg)
{
    (void)arg;
    int head, tail, size;
    listener_info_t *pListenerInfo;
    int signal_value;
    unsigned int lock;
    while (!(pListenerInfo = getListenerInfoPtr()))
    {
        qtime_msleep(5);
    }

    size = sizeof(pListenerInfo->rxPcktBuf.buf) / sizeof(pListenerInfo->rxPcktBuf.buf[0]);

    listenerTask.Exit = 0;

    lock = qirq_lock();
    /* Start reception on the Listener. */
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    qirq_unlock(lock);

    while (listenerTask.Exit == 0)
    {
        /* ISR is delivering RxPckt via circ_buf & Signal.
         * This is the fastest method. */
        if (qsignal_wait(listenerTask.signal, &signal_value, QOSAL_WAIT_FOREVER) != QERR_SUCCESS)
        {
            continue;
        }

        if (signal_value == STOP_TASK)
        {
            break;
        }

        lock = qirq_lock();
        head = pListenerInfo->rxPcktBuf.head;
        tail = pListenerInfo->rxPcktBuf.tail;
        qirq_unlock(lock);

        if (CIRC_CNT(head, tail, size) > 0)
        {
            rx_listener_pckt_t *pRx_listener_Pckt = &pListenerInfo->rxPcktBuf.buf[tail];

            send_to_pc_listener_info(pRx_listener_Pckt->msg.data, pRx_listener_Pckt->rxDataLen, pRx_listener_Pckt->timeStamp, pRx_listener_Pckt->clock_offset, listener_get_mode(), pRx_listener_Pckt->rsl100, pRx_listener_Pckt->fsl100);
            lock = qirq_lock();
            tail = (tail + 1) & (size - 1);
            pListenerInfo->rxPcktBuf.tail = tail;
            qirq_unlock(lock);

            NotifyFlushTask();
        }

        qthread_yield();
    };
    listenerTask.Exit = 2;
    while (listenerTask.Exit == 2)
    {
        qtime_msleep(1);
    }
}

void listener_task_notify(void)
{
    /* Ensure Listener thread is started. */
    if (listenerTask.thread)
    {
        /* Sends the Signal to the application level via OS kernel.
         * This will add a small delay of few us, but
         * this method make sense from a program structure point of view. */
        if (qsignal_raise(listenerTask.signal, LISTENER_DATA) != QERR_SUCCESS)
        {
            error_handler(1, _ERR_Signal_Bad);
        }
    }
}

bool listener_task_started(void)
{
    return (listenerTask.thread != NULL);
}

/**
 * @brief Setup Listener task, this task will send to received data to UART.
 * Only setup, do not start.
 */
static void listener_setup_tasks(void)
{
    /* Listener task is receive the signal from
     * passing signal from RX IRQ to an actual two-way ranging algorithm.
     * It awaiting of an Rx Signal from RX IRQ ISR and decides what to do next in TWR exchange process. */

    listenerTask.signal = qsignal_init();
    if (!listenerTask.signal)
    {
        error_handler(1, _ERR_Create_Task_Bad);
    }

    /* Create Data Transfer task. */
    size_t task_size = LISTENER_TASK_STACK_SIZE_BYTES;
    listenerTask.task_stack = qmalloc(task_size);

    listenerTask.thread = qthread_create(ListenerTask, NULL, "Listener", listenerTask.task_stack, LISTENER_TASK_STACK_SIZE_BYTES, PRIO_RxTask);
    if (!listenerTask.thread)
    {
        error_handler(1, _ERR_Create_Task_Bad);
    }
}

/**
 * @brief Terminate all tasks and timers related to Node functionality, if any
 * DW3000's RX and IRQ shall be switched off before task termination,
 * that IRQ will not produce unexpected Signal.
 * */
void listener_terminate(void)
{
    /* Need to switch off UWB chip's RX and IRQ before killing tasks. */
    enum qerr r = qplatform_deinit();
    if (r != QERR_SUCCESS)
    {
        diag_printf("qplatform_deinit() failed, error %d.\r\n", r);
        error_handler(1, _ERR_INIT);
    }

    terminate_task(&listenerTask);
    listener_process_terminate();

    /* Reset UWB transceiver and enter sleep mode. */
    qplatform_uwb_reset();
    qpwr_uwb_sleep();
}


/**
 * @brief this is a service function which starts the TWR Node functionality.
 * @note The previous instance of TWR shall be killed with node_terminate_tasks();
 *
 * @note: the node_process_init() will allocate the memory of sizeof(node_info_t)
 * from the <b>caller's</b> task stack, see _malloc_r() !
 */
void listener_helper(void const *argument)
{
    /* Not used. */
    (void)argument;

    /* "RTOS-independent" part : initialization of two-way ranging process */
    error_e err = listener_process_init();
    if (err != _NO_ERR)
    {
        diag_printf("listener_process_init() failed, error %d\r\n", err);
        error_handler(1, err);
    }

    /* "RTOS-based" : setup (not start) all necessary tasks for the Node operation. */
    listener_setup_tasks();
    /* IRQ is enabled from MASTER chip and it may receive UWB immediately after this point. */
    err = listener_process_start();
    if (err != _NO_ERR)
    {
        listener_terminate();
    }
}

static const struct subcommand_group_s listener_subcommands = {"LISTENER Options", &known_subcommands_listener, 1};

const app_definition_t helpers_app_listener[] __attribute__((section(".known_apps"))) = {
    {"LISTENER", mAPP, listener_helper, listener_terminate, waitForCommand, command_parser, &listener_subcommands}};
