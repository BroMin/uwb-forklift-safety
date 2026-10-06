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
#include <string.h>

#include "app.h"
#include "usb_uart_tx.h"
#include "listener.h"
#include "task_signal.h"
#include "deca_dbg.h"
#include "deca_device_api.h"
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

volatile bool g_tsync_req       = false;
volatile bool g_listener_active = false;

/* ------------------------------------------------------------------
 * 동기 마스터
 *
 * 앵커 4대 중 1대가 시각 기준 역할을 겸한다. CLI 'SYNCM 1' 로 켠다.
 *
 * 별도 스레드를 만들지 않고 ListenerTask 안에서 처리한다.
 *  - 이 태스크에서 SPI 를 쓰는 것은 기존 TSYNC 블록으로 이미 검증됨
 *  - qsignal_wait 에 타임아웃을 주어 수신이 없어도 주기적으로 깨어남
 *
 * 송신 간격은 규칙적일 필요가 없다. 실제 송신 시각을 그대로 보고하므로
 * 호스트는 들어온 타임스탬프를 쓰면 된다.
 * ------------------------------------------------------------------ */

volatile bool g_sync_master = false;

/* qsignal_wait 타임아웃(ms). 수신이 전혀 없을 때의 동기 주기가 된다. */
#define SYNC_WAIT_MS   20
/* 수신이 활발할 때: 몇 패킷마다 동기를 보낼지 */
#define SYNC_EVERY_N   10
/* TXFRS 대기 가드 — 무한 루프로 보드가 굳는 것을 방지 */
#define SYNC_TX_GUARD  200000

static uint8_t sync_frame[] = {
    0x41, 0x88, 0x00,        /* 0-2  FC, FC, SeqNum        */
    0x26, 0x00,              /* 3-4  PAN ID                */
    0xFF, 0xFF,              /* 5-6  Broadcast             */
    0x01, 0x00,              /* 7-8  Source MAC = 0x0001   */
    0x00, 0x00               /* 9-10 FCS                   */
};
#define SYNC_FRAME_LEN sizeof(sync_frame)

/**
 * @brief 동기 프레임 1회 송신 후 실제 송신 시각을 UART 로 보고한다.
 *
 * 마스터는 라즈베리파이에 USB 로 직접 연결되어 있으므로 송신 시각을
 * UWB 페이로드에 실을 필요가 없다.
 */
static void sync_tx_once(void)
{
    static uint8_t seq = 0;
    uint8_t  ts[5] = {0};
    uint32_t guard = SYNC_TX_GUARD;
    bool     ok;
    char     line[48];
    int      n;

    unsigned int lock = qirq_lock();

    dwt_forcetrxoff();
    sync_frame[2] = seq;
    dwt_writetxdata(SYNC_FRAME_LEN - 2, sync_frame, 0);
    dwt_writetxfctrl(SYNC_FRAME_LEN, 0, 0);
    dwt_starttx(DWT_START_TX_IMMEDIATE);

    while (!(dwt_readsysstatuslo() & DWT_INT_TXFRS_BIT_MASK) && --guard)
    {
        /* busy wait */
    }
    ok = (guard != 0);

    if (ok)
    {
        dwt_readtxtimestamp(ts);          /* ts[0] = LSB */
    }
    dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
    dwt_rxenable(DWT_START_RX_IMMEDIATE); /* 수신 복귀 */

    qirq_unlock(lock);

    if (ok)
    {
        n = sprintf(line, "TXTS: %02X 0x%02X%02X%02X%02X%02X\r\n",
                    seq, ts[4], ts[3], ts[2], ts[1], ts[0]);
    }
    else
    {
        n = sprintf(line, "TXTS: TIMEOUT\r\n");
    }
    port_tx_msg((uint8_t *)line, n);

    seq++;
}

extern const struct command_s known_subcommands_listener;

#define LISTENER_TASK_STACK_SIZE_BYTES 2048
#define MAX_PRINT_FAST_LISTENER        (6)

/**
 * @brief function to report to PC the Listener data received
 * 'JSxxxx{"LSTN":[RxBytes_hex,..,],"TS40":"0xTimeStamp40_Hex","O":Offset_dec}'
 */
error_e send_to_pc_listener_info(uint8_t *data, uint8_t size, uint8_t *ts, int16_t cfo, int mode, int rsl100, int fsl100)
{
    error_e ret = _ERR_Cannot_Alloc_Memory;

    uint32_t cnt, flag_plus = 0;
    uint16_t hlen;
    int cfo_pphm;
    char *str;
    mode = 1; /* 모드 1로 고정 — 전체 패킷 출력 */

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
            sprintf(&str[strlen(str)], "%02X,", data[cnt]);
        }

        if (flag_plus)
        {
            sprintf(&str[strlen(str)], "+,");
        }

        sprintf(&str[strlen(str) - 1], "],\"TS40\":\"0x%02X%02X%02X%02X%02X\",", ts[4], ts[3], ts[2], ts[1], ts[0]);
        sprintf(&str[strlen(str)], "\"O\":%d", cfo_pphm);
        sprintf(&str[strlen(str)], ",\"rsl\":%d.%02d,\"fsl\":%d.%02d", rsl100 / 100, (rsl100 * -1) % 100, fsl100 / 100, (fsl100 * -1) % 100);
        sprintf(&str[strlen(str)], "%s", "}\r\n");
        sprintf(&str[2], "%04X", strlen(str) - hlen);
        str[hlen] = '{';
        ret = copy_tx_msg((uint8_t *)str, strlen(str));

        qfree(str);
    }

    return (ret);
}

/**
 * @brief DW3000 RX : Listener RTOS implementation
 */
static void ListenerTask(void *arg)
{
    g_listener_active = true;
    (void)arg;
    int head, tail, size;
    listener_info_t *pListenerInfo;
    int signal_value;
    unsigned int lock;
    enum qerr wr;
    uint32_t rx_since_sync = 0;

    while (!(pListenerInfo = getListenerInfoPtr()))
    {
        qtime_msleep(5);
    }

    size = sizeof(pListenerInfo->rxPcktBuf.buf) / sizeof(pListenerInfo->rxPcktBuf.buf[0]);

    listenerTask.Exit = 0;

    lock = qirq_lock();
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    qirq_unlock(lock);

    while (listenerTask.Exit == 0)
    {
        /* 타임아웃을 두어 수신이 없어도 주기적으로 깨어난다. */
        wr = qsignal_wait(listenerTask.signal, &signal_value, SYNC_WAIT_MS);

        /* ---- 동기 마스터: 수신 유무와 무관하게 주기적 송신 ---- */
        if (g_sync_master)
        {
            if (wr != QERR_SUCCESS || ++rx_since_sync >= SYNC_EVERY_N)
            {
                rx_since_sync = 0;
                sync_tx_once();
            }
        }

        if (wr != QERR_SUCCESS)
        {
            continue;               /* 타임아웃 — 처리할 패킷 없음 */
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

            send_to_pc_listener_info(pRx_listener_Pckt->msg.data,
                                     pRx_listener_Pckt->rxDataLen,
                                     pRx_listener_Pckt->timeStamp,
                                     pRx_listener_Pckt->clock_offset,
                                     listener_get_mode(),
                                     pRx_listener_Pckt->rsl100,
                                     pRx_listener_Pckt->fsl100);

            /* CLI TSYNC (구방식, 호환용) */
            if (g_tsync_req)
            {
                g_tsync_req = false;

                uint32_t sys_ts = dwt_readsystimestamphi32();
                uint32_t id_lo  = *(volatile uint32_t *)0x10000060;
                uint32_t id_hi  = *(volatile uint32_t *)0x10000064;

                char tsync_str[80];
                int  len = sprintf(tsync_str, "TSYNC: 0x%08lX ID: %08lX%08lX\r\n",
                                   sys_ts, id_hi, id_lo);
                port_tx_msg((uint8_t *)tsync_str, len);
            }

            lock = qirq_lock();
            tail = (tail + 1) & (size - 1);
            pListenerInfo->rxPcktBuf.tail = tail;
            qirq_unlock(lock);

            NotifyFlushTask();
        }

        qthread_yield();
    };

    g_sync_master = false;
    listenerTask.Exit = 2;
    while (listenerTask.Exit == 2)
    {
        qtime_msleep(1);
    }
    g_listener_active = false;
}

void listener_task_notify(void)
{
    if (listenerTask.thread)
    {
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
 * @brief Setup Listener task. Only setup, do not start.
 */
static void listener_setup_tasks(void)
{
    listenerTask.signal = qsignal_init();
    if (!listenerTask.signal)
    {
        error_handler(1, _ERR_Create_Task_Bad);
    }

    size_t task_size = LISTENER_TASK_STACK_SIZE_BYTES;
    listenerTask.task_stack = qmalloc(task_size);

    listenerTask.thread = qthread_create(ListenerTask, NULL, "Listener", listenerTask.task_stack, LISTENER_TASK_STACK_SIZE_BYTES, PRIO_RxTask);
    if (!listenerTask.thread)
    {
        error_handler(1, _ERR_Create_Task_Bad);
    }
}

void listener_terminate(void)
{
    g_sync_master = false;

    enum qerr r = qplatform_deinit();
    if (r != QERR_SUCCESS)
    {
        diag_printf("qplatform_deinit() failed, error %d.\r\n", r);
        error_handler(1, _ERR_INIT);
    }

    terminate_task(&listenerTask);
    listener_process_terminate();

    qplatform_uwb_reset();
    qpwr_uwb_sleep();
}

void listener_helper(void const *argument)
{
    (void)argument;

    error_e err = listener_process_init();
    if (err != _NO_ERR)
    {
        diag_printf("listener_process_init() failed, error %d\r\n", err);
        error_handler(1, err);
    }

    listener_setup_tasks();

    err = listener_process_start();
    if (err != _NO_ERR)
    {
        listener_terminate();
    }
}

static const struct subcommand_group_s listener_subcommands = {"LISTENER Options", &known_subcommands_listener, 2};

const app_definition_t helpers_app_listener[] __attribute__((section(".known_apps"))) = {
    {"LISTENER", mAPP, listener_helper, listener_terminate, waitForCommand, command_parser, &listener_subcommands}};