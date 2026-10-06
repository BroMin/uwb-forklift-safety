/**
 * @file      listener_fn.c
 *
 * @brief     Command implementation for Listener application
 *
 * @author    Qorvo Applications
 *
 * @copyright SPDX-FileCopyrightText: Copyright (c) 2024 Qorvo US, Inc.
 *            SPDX-License-Identifier: LicenseRef-QORVO-2
 *
 */

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "cmd_fn.h"
#include "cmd.h"
#include "EventManager.h"
#include "listener.h"
#include "reporter.h"
#include "qmalloc.h"
#include "qirq.h"
#include "deca_dbg.h"

const char COMMENT_LSTAT[] = {"Displays the statistics inside the Listener application."};
const char COMMENT_LISTENER[] = {"Listen for the UWB packets using the UWB configuration and displays.\r\n"
                                 "1/ Maximum 127 bytes of a payload. \r\n"
                                 "2/ \"TS4ns\":Timestamp of each valid frame reception. \r\n"
                                 "3/ \"O\": Crystal frequency offset in ppm units. \r\n"
                                 "4/ RSL,FSL: RX Level and First Path Power level (dBm)."};

extern const app_definition_t helpers_app_listener[];

extern volatile bool g_sync_master;

const char COMMENT_SYNCM[] = {"Enable/disable sync master TX. Usage: SYNCM 1"};

REG_FN(f_syncm)
{
    char cmd[10];
    int  v = 0;
    if (sscanf(text, "%9s %d", cmd, &v) == 2)
    {
        g_sync_master = (v != 0);
    }
    diag_printf("SYNCM: %d\r\n", g_sync_master ? 1 : 0);
    return (CMD_FN_RET_OK);
}

/**
 * @brief   defaultTask will start listener user application
 */
REG_FN(f_listen_start)
{
    /* Set mode to 2*/
    listener_set_mode(2);

    app_definition_t *app_ptr = (app_definition_t *)&helpers_app_listener[0];
    EventManagerRegisterApp((void *)&app_ptr);

    return (CMD_FN_RET_OK);
}

REG_FN(f_lstat)
{
    char *str = qmalloc(MAX_STR_SIZE);

    if (str)
    {
        unsigned int lock = qirq_lock();
        int hlen, str_len;
        /* Listener RX Event Counts object. */
        listener_info_t *info = getListenerInfoPtr();
        /* Reserve space for length of JS object. */
        str_len = sprintf(str, "JS%04X", 0x5A5A);
        hlen = str_len;
        str_len += sprintf(&str[str_len], "{\"RX Events\":{\r\n");
        str_len += sprintf(&str[str_len], "\"CRCG\":%d,\r\n", (int)info->event_counts.CRCG);
        str_len += sprintf(&str[str_len], "\"CRCB\":%d,\r\n", (int)info->event_counts.CRCB);
        str_len += sprintf(&str[str_len], "\"ARFE\":%d,\r\n", (int)info->event_counts.ARFE);
        str_len += sprintf(&str[str_len], "\"PHE\":%d,\r\n", (int)info->event_counts.PHE);
        str_len += sprintf(&str[str_len], "\"RSL\":%d,\r\n", (int)info->event_counts.RSL);
        str_len += sprintf(&str[str_len], "\"SFDTO\":%d,\r\n", (int)info->event_counts.SFDTO);
        str_len += sprintf(&str[str_len], "\"PTO\":%d,\r\n", (int)info->event_counts.PTO);
        str_len += sprintf(&str[str_len], "\"FTO\":%d,\r\n", (int)info->event_counts.RTO);
        str_len += sprintf(&str[str_len], "\"SFDD\":%d}}", (int)info->event_counts_sfd_detect);
        /* Add formatted 4X of length, this will erase first '{'. */
        sprintf(&str[2], "%04X", str_len - hlen);
        /* Restore the start bracket. */
        str[hlen] = '{';
        str_len += sprintf(&str[strlen(str)], "\r\n");
        reporter_instance.print((char *)str, str_len);

        assert(str_len <= MAX_STR_SIZE);

        qfree(str);

        qirq_unlock(lock);
    }
    return (CMD_FN_RET_OK);
}

const struct command_s known_app_listener[] __attribute__((section(".known_commands_app"))) = {
    {"LISTENER", mIDLE, f_listen_start, COMMENT_LISTENER},
};

const struct command_s known_subcommands_listener[] __attribute__((section(".known_app_subcommands"))) = {
    {"LSTAT", mAPP, f_lstat,  COMMENT_LSTAT},
    {"SYNCM", mAPP, f_syncm,  COMMENT_SYNCM},
};
