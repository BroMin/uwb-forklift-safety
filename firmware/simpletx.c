#include "simpletx.h"
#include "deca_device_api.h"
#include "deca_dbg.h"
#include "HAL_error.h"
#include "driver_app_config.h"
#include "qplatform.h"
#include "qirq.h"
#include <string.h>

/* ------------------------------------------------------------------
 * 방식 B 태그
 *
 * blink 송신 후 수신을 켜서 마스터(RX1)의 동기 프레임을 한 번 듣는다.
 * 자기 송신 시각 t1 과 동기 프레임 수신 시각 t4 를 다음 blink 에 실어
 * 보고하면, 호스트가 태그 시계를 마스터 시계에 묶어 절대 거리를 낸다.
 *
 * ⚠️ UART 출력을 쓰지 않는다.
 *    SimpleTX 태스크 컨텍스트에서 diag_printf / port_tx_msg 가 동작하지
 *    않는 것으로 확인됐고, sprintf 는 스택(2048)을 넘겨 리셋을 유발했다.
 *    진단은 프레임 페이로드로 한다 — 앵커 로그에서 확인.
 *
 *    idx 10~14 ≠ 0  →  자기 송신 시각 기록 성공
 *    idx 15~20 ≠ 0  →  동기 프레임 수신 성공
 * ------------------------------------------------------------------ */

static uint8_t tx_frame[] = {
    0x41, 0x88, 0x00,             /* 0-2   FC, FC, SeqNum            */
    0x26, 0x00,                   /* 3-4   PAN ID                    */
    0xFF, 0xFF,                   /* 5-6   Broadcast                 */
    0x02, 0x00,                   /* 7-8   Source MAC = 0x0002 (Tag) */
    0x00,                         /* 9     직전 blink SeqNum          */
    0x00, 0x00, 0x00, 0x00, 0x00, /* 10-14 직전 blink 송신 시각 t1     */
    0x00,                         /* 15    들은 동기 프레임 SeqNum     */
    0x00, 0x00, 0x00, 0x00, 0x00, /* 16-20 그 동기 수신 시각 t4        */
    0x00, 0x00                    /* 21-22 FCS                       */
};
#define TX_FRAME_LEN   sizeof(tx_frame)
#define TX_SEQ_IDX     2
#define TX_PSEQ_IDX    9
#define TX_PTS_IDX     10
#define TX_SSEQ_IDX    15
#define TX_STS_IDX     16

/* 수신 프레임에서 볼 위치 (마스터 동기 프레임 기준) */
#define RX_SEQ_IDX     2
#define RX_MAC_IDX     7
#define SYNC_MAC       0x0001
#define RX_PEEK_LEN    9              /* seq 와 MAC 까지만 읽으면 충분 */

/* 폴링 가드
 * dwt_readsysstatuslo() 는 SPI 트랜잭션이라 1회 5~10us.
 * 가드를 크게 잡으면 blink 주기가 수십 초로 늘어난다.
 * 수신 종료는 하드웨어 타임아웃(RXFTO)에 맡기고 가드는 안전망으로만. */
#define TX_GUARD       200000
#define RX_GUARD       4000

/* 수신 타임아웃. 단위 약 1.026 us  →  20000 ≈ 20.5 ms */
#define RX_TIMEOUT_U   20000

#define RX_ERR_BITS  (DWT_INT_RXPHE_BIT_MASK | DWT_INT_RXFCE_BIT_MASK \
                    | DWT_INT_RXFSL_BIT_MASK | DWT_INT_RXSTO_BIT_MASK \
                    | DWT_INT_ARFE_BIT_MASK)


error_e simpletx_process_init(void)
{
    enum qerr err = qplatform_init();
    if (err != QERR_SUCCESS) return _ERR_INIT;

    dwt_config_t *dwt_config = get_dwt_config();
    unsigned int lock = qirq_lock();

    if (dwt_initialise(0) != DWT_SUCCESS) { qirq_unlock(lock); return _ERR_INIT; }
    qplatform_uwb_spi_set_fast_rate_freq();
    if (dwt_configure(dwt_config)) { qirq_unlock(lock); return _ERR_INIT; }

    /* ---- 수신 설정 (방식 B 신규) ----
     * 프레임 필터를 끄면 브로드캐스트 동기 프레임을 그대로 받는다. */
    dwt_setrxaftertxdelay(0);
    dwt_setrxtimeout(0);
    dwt_configureframefilter(DWT_FF_DISABLE, 0);

    dwt_app_config_t *dwt_app_config = get_app_dwt_config();
    dwt_setxtaltrim(dwt_app_config->xtal_trim);

    qirq_unlock(lock);
    return _NO_ERR;
}


error_e simpletx_process_start(void)
{
    enum qerr r = qplatform_uwb_interrupt_enable();
    if (r != QERR_SUCCESS) return _ERR_INIT;
    diag_printf("SimpleTX(TagB): Started\r\n");
    return _NO_ERR;
}

void simpletx_process_terminate(void) { qplatform_deinit(); }


void simpletx_send_frame(void)
{
    static uint8_t prev_seq = 0;
    static uint8_t prev_ts_be[5] = {0};
    static uint8_t sync_seq = 0;
    static uint8_t sync_ts_be[5] = {0};

    uint8_t  ts[5];
    uint8_t  rxbuf[RX_PEEK_LEN];
    uint32_t guard;
    uint32_t st;

    /* ---------------- 송신 ---------------- */
    tx_frame[TX_PSEQ_IDX] = prev_seq;
    memcpy(&tx_frame[TX_PTS_IDX], prev_ts_be, 5);
    tx_frame[TX_SSEQ_IDX] = sync_seq;
    memcpy(&tx_frame[TX_STS_IDX], sync_ts_be, 5);

    dwt_writetxdata(TX_FRAME_LEN - 2, tx_frame, 0);
    dwt_writetxfctrl(TX_FRAME_LEN, 0, 0);

    if (dwt_starttx(DWT_START_TX_IMMEDIATE) != DWT_SUCCESS)
    {
        dwt_forcetrxoff();
        return;
    }

    guard = TX_GUARD;
    while (!(dwt_readsysstatuslo() & DWT_INT_TXFRS_BIT_MASK) && --guard) {}
    if (!guard)
    {
        dwt_forcetrxoff();
        dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
        return;
    }

    /* 하드웨어가 찍은 실제 송신 시각 → 다음 blink 에 실어 보고 */
    dwt_readtxtimestamp(ts);
    prev_ts_be[0]=ts[4]; prev_ts_be[1]=ts[3]; prev_ts_be[2]=ts[2];
    prev_ts_be[3]=ts[1]; prev_ts_be[4]=ts[0];
    dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);

    prev_seq = tx_frame[TX_SEQ_IDX];
    tx_frame[TX_SEQ_IDX]++;

    /* ---------------- 수신: 동기 프레임 한 번 듣기 ---------------- */
    dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK | DWT_INT_RXFTO_BIT_MASK
                         | RX_ERR_BITS);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    guard = RX_GUARD;
    while (--guard)
    {
        st = dwt_readsysstatuslo();

        if (st & DWT_INT_RXFCG_BIT_MASK)
        {
            /* seq 와 MAC 까지만 읽는다. 프레임 길이를 몰라도 안전. */
            dwt_readrxdata(rxbuf, RX_PEEK_LEN, 0);

            if (((uint16_t)rxbuf[RX_MAC_IDX]
                 | ((uint16_t)rxbuf[RX_MAC_IDX + 1] << 8)) == SYNC_MAC)
            {
                dwt_readrxtimestamp(ts, DWT_COMPAT_NONE);
                sync_ts_be[0]=ts[4]; sync_ts_be[1]=ts[3]; sync_ts_be[2]=ts[2];
                sync_ts_be[3]=ts[1]; sync_ts_be[4]=ts[0];
                sync_seq = rxbuf[RX_SEQ_IDX];
                dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);
                break;
            }

            /* 다른 태그의 blink → 버리고 계속 듣는다 */
            dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
            continue;
        }

        /* 수신 오류 → 클리어하고 재무장 */
        if (st & RX_ERR_BITS)
        {
            dwt_writesysstatuslo(RX_ERR_BITS);
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
        }
    }

    dwt_forcetrxoff();
}