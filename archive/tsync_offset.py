#!/usr/bin/env python3
"""
tsync_offset.py — 라즈베리파이 시각 기준 TSYNC 오프셋 측정

김동훈 제안 방식을 그대로 구현한다.

    offset_i = uwb_ts_i - (rpi_ts_i / TS_UNIT)
    RX1<->RX2 오프셋 = (uwb_ts_2 - uwb_ts_1) - (rpi_ts_2 - rpi_ts_1) / TS_UNIT

rpi_ts는 세 가지로 전부 기록한다. 어떤 정의를 쓰든 결과가 같다는 걸
보이기 위한 것이다.
    t_send      : TSYNC 명령을 write한 시각
    t_recv      : 응답 마지막 바이트를 읽은 시각
    rpi_ts_mid  : (t_send + t_recv) / 2   <- Christian's algorithm
                  왕복 지연이 대칭이라 가정할 때의 최선. 이 방식의 상한선.

사용법:
    python3 tsync_offset.py                 # 기본 200라운드
    python3 tsync_offset.py --rounds 500
    python3 tsync_offset.py --gap 0.2       # 라운드 간격(초)

출력:
    ~/uwb_logs/tsync_YYYYmmdd_HHMMSS.csv
"""

import argparse
import csv
import datetime
import os
import re
import sys
import time

import serial

# ---------------------------------------------------------------- 설정

SN_MAP = {
    '760197764': 'RX1',
    '760144486': 'RX2',
    '760143773': 'RX3',
    '760197326': 'RX4',
}
ORDER = ['RX1', 'RX2', 'RX3', 'RX4']       # 순차 전송 순서 (카톡 합의)

BAUD = 230400

# SYS_TIME 1 카운트. DW3000의 SYS_TIME은 40비트 카운터의 상위 32비트만
# 노출하므로 최소 단위가 4.0064ns다. (RX_TIME은 40비트 = 15.65ps)
TS_UNIT = 4.0064e-9
SYS_TIME_WRAP = 2 ** 32                    # 17.2초마다 감김

RESP_TIMEOUT = 1.0                         # TSYNC 응답 대기 (초)

TSYNC_RE = re.compile(
    rb'TSYNC:\s*0x([0-9A-Fa-f]{8})\s*ID:\s*([0-9A-Fa-f]{16})'
)

LOG_DIR = os.path.expanduser('~/uwb_logs')

# ---------------------------------------------------------------- 포트


def find_port(sn):
    base = '/dev/serial/by-id/'
    try:
        for entry in os.listdir(base):
            if sn in entry and 'if00' in entry:
                return os.path.join(base, entry)
    except OSError:
        pass
    return None


def open_ports():
    """SN_MAP 기준으로 전 포트 오픈. 하나라도 실패하면 종료."""
    ports = {}
    for sn, name in SN_MAP.items():
        path = find_port(sn)
        if path is None:
            print(f'[{name}] 포트 못 찾음 (SN={sn})')
            continue
        try:
            ser = serial.Serial(path, BAUD, timeout=0)
            ports[name] = ser
            print(f'[{name}] 접속: {path}')
        except serial.SerialException as e:
            print(f'[{name}] 오픈 실패: {e}')

    missing = [n for n in ORDER if n not in ports]
    if missing:
        print(f'\n누락: {missing}  —  전 보드 연결 후 재실행하세요.')
        for ser in ports.values():
            ser.close()
        sys.exit(1)
    return ports


# ---------------------------------------------------------------- 측정


def tsync_once(ser, timeout=RESP_TIMEOUT):
    """TSYNC 1회 왕복.

    반환: (t_send, t_recv, uwb_ts, dev_id) 또는 None (타임아웃)

    reset_input_buffer()로 LSTN 백로그를 먼저 비운다.
    큐에 쌓인 LSTN 패킷 때문에 응답이 밀리는 걸 막기 위한 것으로,
    이 방식에 최대한 유리한 조건을 주는 셈이다.
    """
    ser.reset_input_buffer()

    t_send = time.perf_counter()
    ser.write(b'TSYNC\r\n')
    ser.flush()

    deadline = t_send + timeout
    buf = b''
    while time.perf_counter() < deadline:
        n = ser.in_waiting
        chunk = ser.read(n if n else 1)
        if not chunk:
            continue
        buf += chunk
        m = TSYNC_RE.search(buf)
        if m:
            t_recv = time.perf_counter()
            return t_send, t_recv, int(m.group(1), 16), m.group(2).decode()
        if len(buf) > 65536:               # LSTN 홍수 방어
            buf = buf[-4096:]
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rounds', type=int, default=200,
                    help='측정 라운드 수 (기본 200)')
    ap.add_argument('--gap', type=float, default=0.5,
                    help='라운드 간 간격 초 (기본 0.5)')
    args = ap.parse_args()

    os.makedirs(LOG_DIR, exist_ok=True)
    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    csv_path = f'{LOG_DIR}/tsync_{stamp}.csv'

    ports = open_ports()

    print(f'\n라운드 {args.rounds}회, 간격 {args.gap}s')
    print(f'예상 소요 {args.rounds * (args.gap + 0.05):.0f}s')
    print(f'CSV: {csv_path}')
    print('Ctrl+C로 중단 (중단해도 그 시점까지 저장됨)\n')

    f = open(csv_path, 'w', newline='')
    w = csv.writer(f)
    w.writerow(['round', 'rx', 'wall_clock',
                't_send', 't_recv', 'rpi_ts_mid', 'rtt_ms',
                'uwb_ts_raw', 'dev_id'])

    t0 = time.perf_counter()
    fails = {n: 0 for n in ORDER}

    try:
        for rnd in range(args.rounds):
            round_start = time.perf_counter()

            for name in ORDER:
                r = tsync_once(ports[name])
                if r is None:
                    fails[name] += 1
                    continue
                t_send, t_recv, uwb_ts, dev_id = r
                w.writerow([
                    rnd, name,
                    datetime.datetime.now().isoformat(timespec='microseconds'),
                    f'{t_send - t0:.9f}',
                    f'{t_recv - t0:.9f}',
                    f'{(t_send + t_recv) / 2 - t0:.9f}',
                    f'{(t_recv - t_send) * 1e3:.3f}',
                    uwb_ts, dev_id,
                ])

            f.flush()

            if (rnd + 1) % 10 == 0:
                el = time.perf_counter() - t0
                nf = sum(fails.values())
                print(f'  라운드 {rnd + 1}/{args.rounds}  '
                      f'{el:.0f}s 경과  실패 {nf}건')

            sleep = args.gap - (time.perf_counter() - round_start)
            if sleep > 0:
                time.sleep(sleep)

    except KeyboardInterrupt:
        print('\n중단됨')

    finally:
        f.close()
        for ser in ports.values():
            ser.close()

    print(f'\n저장 완료: {csv_path}')
    if any(fails.values()):
        print(f'응답 실패: {fails}')
        print('실패가 많으면 패치 2(g_listener_active)가 안 먹은 것일 수 있습니다.')
    print(f'\n다음: python3 tsync_analysis.py {csv_path}')


if __name__ == '__main__':
    main()
