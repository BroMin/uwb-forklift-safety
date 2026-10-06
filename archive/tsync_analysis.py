#!/usr/bin/env python3
"""
tsync_analysis.py — TSYNC 오프셋 측정 결과 분석

tsync_offset.py가 만든 CSV를 읽어 다음을 판정한다.

1. 왕복 지연(RTT) 분포        — 전송 지터 실측
2. SYS_TIME unwrap            — 17.2초 wrap 복원
3. 오프셋의 선형 회귀         — 기울기(=상대 클럭 드리프트), 잔차
4. AndyA 진단                 — 잔차가 선형인가 랜덤인가
5. 최종 판정                  — 잔차를 거리로 환산해 30cm 목표와 비교

사용법:
    python3 tsync_analysis.py ~/uwb_logs/tsync_20260814_170000.csv
    python3 tsync_analysis.py <csv> --plot   # matplotlib 있으면 그래프 저장
"""

import argparse
import csv
import sys
from collections import defaultdict

import numpy as np

C = 299_792_458.0
TS_UNIT = 4.0064e-9          # SYS_TIME 1 카운트 (상위 32비트만 노출)
SYS_TIME_WRAP = 2 ** 32      # 17.2초
COUNT_TO_M = TS_UNIT * C     # 1.2019 m

ORDER = ['RX1', 'RX2', 'RX3', 'RX4']
REF = 'RX1'

# 성능 목표에서 역산한 요구 정밀도
TARGET_ACCURACY_M = 0.30
REQUIRED_NS = TARGET_ACCURACY_M / C * 1e9        # 1.00 ns
# 현 배치(0.4 x 0.4 m)의 최대 TDOA
MAX_BASELINE_M = 0.4 * np.sqrt(2)
MAX_TDOA_NS = MAX_BASELINE_M / C * 1e9           # 1.89 ns


# ---------------------------------------------------------------- 로드


def load(path):
    rows = defaultdict(list)
    with open(path) as f:
        for r in csv.DictReader(f):
            rows[r['rx']].append({
                'round': int(r['round']),
                't_send': float(r['t_send']),
                't_recv': float(r['t_recv']),
                'rpi': float(r['rpi_ts_mid']),
                'rtt': float(r['rtt_ms']),
                'uwb_raw': int(r['uwb_ts_raw']),
            })
    return rows


def unwrap(raw, rpi):
    """SYS_TIME 32비트 wrap 복원.

    라즈베리파이 경과시간으로부터 기대 카운트를 만들고, 그와 가장 가까워지는
    wrap 횟수 k를 고른다. 드리프트가 ppm 수준이라 기대값과의 차이가
    2^32의 절반(8.6초)보다 훨씬 작으므로 안전하다.
    """
    raw = np.asarray(raw, dtype=np.float64)
    rpi = np.asarray(rpi, dtype=np.float64)
    expected = (rpi - rpi[0]) / TS_UNIT + raw[0]
    k = np.round((expected - raw) / SYS_TIME_WRAP)
    return raw + k * SYS_TIME_WRAP


# ---------------------------------------------------------------- 분석


def fmt_counts(counts):
    """카운트를 ns / m 로 같이 표기"""
    ns = counts * TS_UNIT * 1e9
    m = counts * COUNT_TO_M
    return f'{counts:>12,.1f} cnt  {ns:>12,.1f} ns  {m:>12,.1f} m'


def analyze(rows):
    print('=' * 76)
    print('  TSYNC 오프셋 분석 — 라즈베리파이 시각 기준 방식')
    print('=' * 76)

    n_rounds = max(len(v) for v in rows.values())
    print(f'\n라운드 수: {n_rounds}')
    print(f'SYS_TIME 최소 단위: {TS_UNIT * 1e9:.4f} ns  '
          f'(= {COUNT_TO_M:.4f} m/카운트)')
    print(f'30cm 목표 요구 정밀도: {REQUIRED_NS:.2f} ns '
          f'(= {REQUIRED_NS / (TS_UNIT * 1e9):.3f} 카운트)')

    # ---------------- 1. RTT
    print('\n' + '-' * 76)
    print('1. 왕복 지연 (전송 지터 실측)')
    print('-' * 76)
    print(f'{"RX":<6}{"평균(ms)":>11}{"중앙(ms)":>11}'
          f'{"표준편차(ms)":>14}{"최소":>9}{"최대":>9}')
    for name in ORDER:
        rtt = np.array([d['rtt'] for d in rows[name]])
        if rtt.size == 0:
            continue
        print(f'{name:<6}{rtt.mean():>11.3f}{np.median(rtt):>11.3f}'
              f'{rtt.std():>14.3f}{rtt.min():>9.3f}{rtt.max():>9.3f}')

    all_rtt = np.concatenate([[d['rtt'] for d in rows[n]] for n in ORDER])
    jitter_ms = all_rtt.std()
    print(f'\n전체 지터(1σ): {jitter_ms:.3f} ms')
    print(f'  → 카운트 환산: {jitter_ms * 1e-3 / TS_UNIT:,.0f} 카운트')
    print(f'  → 거리 환산  : {jitter_ms * 1e-3 * C / 1000:,.0f} km')

    # ---------------- 2~3. 오프셋
    print('\n' + '-' * 76)
    print('2. 오프셋 선형 회귀')
    print('-' * 76)

    offsets = {}
    resids = {}

    for name in ORDER:
        d = rows[name]
        if len(d) < 10:
            continue
        rpi = np.array([x['rpi'] for x in d])
        uwb = unwrap([x['uwb_raw'] for x in d], rpi)

        # 김동훈 공식: offset = uwb_ts - rpi_ts / TS_UNIT
        off = uwb - rpi / TS_UNIT
        offsets[name] = (rpi, off)

        # 선형 회귀: 기울기 = 상대 클럭 드리프트
        slope, icpt = np.polyfit(rpi, off, 1)
        fit = slope * rpi + icpt
        res = off - fit
        resids[name] = (rpi, res)

        ppm = slope * TS_UNIT * 1e6      # 카운트/초 → ppm

        print(f'\n[{name}]')
        print(f'  오프셋 절대값 (평균)  {fmt_counts(off.mean())}')
        print(f'  드리프트 기울기       {slope:>12,.1f} cnt/s  '
              f'= {ppm:>8.3f} ppm')
        print(f'  회귀 잔차 (1σ)        {fmt_counts(res.std())}')
        print(f'  회귀 잔차 (P2P)       {fmt_counts(res.max() - res.min())}')

    # ---------------- 4. AndyA 진단
    print('\n' + '-' * 76)
    print('4. AndyA 진단 — 잔차가 선형(드리프트)인가 랜덤(버그)인가')
    print('-' * 76)
    print('  기준: "드리프트라면 일정 속도로 변해야 한다. 랜덤하게 튀면 로직 버그."')
    print(f'\n{"RX":<6}{"1차 자기상관":>14}{"판정":>28}')
    for name in ORDER:
        if name not in resids:
            continue
        _, res = resids[name]
        if res.size < 20:
            continue
        # lag-1 자기상관: 구조가 있으면 1에 가깝고, 백색잡음이면 0에 가깝다
        r = np.corrcoef(res[:-1], res[1:])[0, 1]
        if r > 0.7:
            verdict = '구조적 — 추가 보정 여지 있음'
        elif r < 0.2:
            verdict = '백색잡음 — 보정 불가 (전송 지터)'
        else:
            verdict = '혼재'
        print(f'{name:<6}{r:>14.3f}{verdict:>28}')

    # ---------------- 5. 쌍별 (실제 TDOA에 들어가는 값)
    print('\n' + '-' * 76)
    print(f'5. 쌍별 오프셋 잔차 ({REF} 기준) — TDOA에 실제로 들어가는 오차')
    print('-' * 76)

    pair_sigmas = {}
    if REF in offsets:
        rpi_ref, off_ref = offsets[REF]
        for name in ORDER:
            if name == REF or name not in offsets:
                continue
            rpi_i, off_i = offsets[name]
            # 같은 라운드끼리 맞추기 위해 레퍼런스를 보간
            off_ref_at_i = np.interp(rpi_i, rpi_ref, off_ref)
            diff = off_i - off_ref_at_i
            slope, icpt = np.polyfit(rpi_i, diff, 1)
            res = diff - (slope * rpi_i + icpt)
            pair_sigmas[name] = res.std()
            print(f'\n[{REF}-{name}]')
            print(f'  상대 드리프트   {slope * TS_UNIT * 1e6:>8.3f} ppm')
            print(f'  잔차 (1σ)       {fmt_counts(res.std())}')

    # ---------------- 6. 판정
    print('\n' + '=' * 76)
    print('  최종 판정')
    print('=' * 76)

    if pair_sigmas:
        worst = max(pair_sigmas.values())
        worst_ns = worst * TS_UNIT * 1e9
        worst_m = worst * COUNT_TO_M

        print(f'\n최악 쌍별 잔차 (1σ)      {worst_ns:>14,.1f} ns'
              f'{worst_m:>16,.1f} m')
        print(f'30cm 목표 요구 정밀도    {REQUIRED_NS:>14.2f} ns'
              f'{TARGET_ACCURACY_M:>16.2f} m')
        print(f'현 배치 최대 TDOA        {MAX_TDOA_NS:>14.2f} ns'
              f'{MAX_BASELINE_M:>16.2f} m')
        print(f'\n초과 배율               {worst_ns / REQUIRED_NS:>14,.0f} 배')
        print(f'신호 대비 오차 배율      '
              f'{worst_ns / MAX_TDOA_NS:>14,.0f} 배')

        print('\n오차원 분해:')
        print(f'  (a) 전송 지터 (측정)        {jitter_ms * 1e6:>12,.0f} ns'
              '   ← 지배적')
        print(f'  (b) SYS_TIME 양자화 (HW)    '
              f'{TS_UNIT * 1e9:>12.4f} ns   ← (a)를 없애도 남음')
        print(f'  (c) 필요 정밀도             {REQUIRED_NS:>12.2f} ns')
        print('\n  (b) > (c) 이므로, 전송 지터를 완전히 제거해도')
        print('  SYS_TIME 읽기 분해능만으로 이미 목표를 넘어섭니다.')
        print('  DW3000의 SYS_TIME은 40비트 카운터의 상위 32비트만 노출하므로')
        print('  하위 8비트(15.65ps 단위)는 소프트웨어로 접근할 수 없습니다.')

        if worst_m > TARGET_ACCURACY_M:
            print('\n>>> 결론: 이 방식으로는 30cm 달성 불가.')
            print('>>> 동기화 기준을 UART/OS 경로에서 UWB 공중 인터페이스로')
            print('>>> 옮겨야 합니다. 알고리즘 구조(오프셋+드리프트 분리,')
            print('>>> 델타 누적 갱신)는 그대로 재사용 가능합니다.')
        else:
            print('\n>>> 예상과 다른 결과입니다. 데이터를 다시 확인하세요.')
    else:
        print('\n쌍별 계산 불가 — 데이터 부족.')

    return offsets, resids


def plot(offsets, resids, out='tsync_analysis.png'):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print('\nmatplotlib 없음 — 그래프 생략 '
              '(pip3 install matplotlib)')
        return

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 8))

    for name, (rpi, off) in offsets.items():
        ax1.plot(rpi, off - off[0], '.-', ms=3, lw=0.7, label=name)
    ax1.set_ylabel('offset - offset[0]  (counts)')
    ax1.set_title('TSYNC offset over time (raw)')
    ax1.legend()
    ax1.grid(alpha=0.3)

    for name, (rpi, res) in resids.items():
        ax2.plot(rpi, res * COUNT_TO_M, '.', ms=3, label=name)
    req = TARGET_ACCURACY_M
    ax2.axhline(req, color='r', ls='--', lw=1, label=f'±{req}m target')
    ax2.axhline(-req, color='r', ls='--', lw=1)
    ax2.set_xlabel('elapsed (s)')
    ax2.set_ylabel('residual (m)')
    ax2.set_title('Residual after linear drift removal  '
                  '(red = 30cm target)')
    ax2.legend()
    ax2.grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(out, dpi=130)
    print(f'\n그래프 저장: {out}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv')
    ap.add_argument('--plot', action='store_true')
    args = ap.parse_args()

    rows = load(args.csv)
    if not rows:
        print('데이터 없음')
        sys.exit(1)

    offsets, resids = analyze(rows)

    if args.plot:
        plot(offsets, resids)


if __name__ == '__main__':
    main()
