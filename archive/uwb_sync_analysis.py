#!/usr/bin/env python3
"""
uwb_sync_analysis.py — 무선 동기화(AndyA 방식) M2 판정

비콘이 실어보낸 TX 타임스탬프와 각 앵커의 RX 타임스탬프를 짝지어
클럭 오프셋을 구하고, 10사이클 창으로 외삽했을 때의 예측 잔차를 측정한다.

    offset_i(n) = R_i(n) - T(n)
                = b_i + (a_i - 1)·T(n) + a_i·τ_i

τ_i 가 상수이므로 offset 은 시간에 대해 직선이어야 한다.
직선에서 벗어나는 양이 곧 측위 오차다.

판정:
    잔차가 수십 카운트 이하  → 성공
    일정 기울기로 벗어남     → 드리프트 보정 미흡
    랜덤 점프               → 로직 버그 (AndyA 진단)

사용법:
    python3 uwb_sync_analysis.py ~/uwb_logs/raw_20260817_HHMMSS.txt
    python3 uwb_sync_analysis.py <log> --window 10 --plot
"""

import argparse
import re
import sys
from collections import defaultdict, deque

import numpy as np

# ---------------------------------------------------------------- 상수

C = 299_792_458.0
U = 15.65e-12                 # RX_TIME / TX_TIME 1 카운트
WRAP = 2 ** 40                # 17.2 초
CNT_TO_M = U * C              # 4.69 mm

TARGET_M = 0.30               # 1차년도 목표

# 프레임 인덱스
I_SEQ = 2
I_PSEQ = 9
I_PTS = 10                    # 10~14, MSB first

LINE_RE = re.compile(
    r'\[(?P<rx>RX\d)\]\s+JS[0-9A-Fa-f]{4}'
    r'\{"LSTN":\[(?P<lstn>[^\]]+)\].*?'
    r'"TS40":"0x(?P<ts>[0-9A-Fa-f]+)".*?'
    r'"O":(?P<o>-?\d+)'
)


# ---------------------------------------------------------------- 파싱


def unwrapper():
    """40비트 카운터 unwrap.

    임계값으로 wrap을 '감지'하지 않고, 연속 차분을 mod 2^40 으로 취해
    누적한다. 실제 간격이 17.2초 미만이면 항상 옳다. 패킷 손실이
    wrap 근처에서 겹쳐도 안전하다.
    """
    state = {'prev': None, 'acc': 0}

    def f(v):
        if state['prev'] is None:
            state['acc'] = v
        else:
            state['acc'] += (v - state['prev']) % WRAP
        state['prev'] = v
        return state['acc']

    return f


def parse(path):
    """RX별로 (tx_ts, rx_ts, O) 짝 목록을 만든다.

    펌웨어가 항상 prev_seq = seq - 1 로 보내므로, 직전 패킷 하나만
    들고 있으면 된다. dict + seq 키 방식은 SeqNum 8비트 순환(256패킷,
    2.6초) 때문에 유실이 생기면 낡은 항목과 오짝을 만든다.
    """
    last = {}                        # rx -> (seq, rx_ts_unwrapped, O)
    pairs = defaultdict(list)

    unwrap_rx = defaultdict(unwrapper)
    unwrap_tx = defaultdict(unwrapper)

    n_lines = n_match = n_short = n_nopair = 0

    with open(path, errors='ignore') as f:
        for line in f:
            n_lines += 1
            m = LINE_RE.search(line)
            if not m:
                continue
            n_match += 1

            name = m.group('rx')
            try:
                lstn = [int(x, 16) for x in m.group('lstn').split(',')]
            except ValueError:
                continue
            if len(lstn) < 15:
                n_short += 1
                continue

            rx_ts = unwrap_rx[name](int(m.group('ts'), 16))
            seq = lstn[I_SEQ]
            o = int(m.group('o'))

            pseq = lstn[I_PSEQ]
            pts_raw = 0
            for b in lstn[I_PTS:I_PTS + 5]:          # MSB first
                pts_raw = (pts_raw << 8) | b

            prev = last.get(name)
            if pts_raw and prev is not None:
                if prev[0] == pseq:
                    tx_ts = unwrap_tx[name](pts_raw)
                    pairs[name].append((tx_ts, prev[1], prev[2]))
                else:
                    n_nopair += 1                     # 유실 등으로 직전이 아님

            last[name] = (seq, rx_ts, o)

    print(f'읽은 줄 {n_lines:,}  파싱 {n_match:,}  '
          f'길이부족 {n_short:,}  짝없음 {n_nopair:,}')
    return pairs


# ---------------------------------------------------------------- 분석


def segments(tx, lo=0.3, hi=5.0):
    """연속 간격이 중앙값 대비 [lo, hi] 배를 벗어나는 지점에서 구간을 끊는다.

    TX 재시작, 장시간 공백, 드문 파싱 이상 등이 누적 unwrap 을 오염시키는
    것을 막는다. 실배치에서도 공백은 늘 생기므로 필요한 처리다.
    """
    d = np.diff(tx)
    med = np.median(d)
    bad = np.where((d < med * lo) | (d > med * hi))[0]
    bounds = [0] + (bad + 1).tolist() + [len(tx)]
    return [(bounds[i], bounds[i + 1]) for i in range(len(bounds) - 1)], bad, med


def holdout(tx, off, win):
    """앞의 win개로 직선을 맞춰 다음 점을 예측. (예측시각, 잔차) 반환."""
    t_out, r_out = [], []
    for k in range(win, len(off)):
        t = tx[k - win:k]
        o = off[k - win:k]
        beta, alpha = np.polyfit(t, o, 1)
        t_out.append(tx[k])
        r_out.append(off[k] - (beta * tx[k] + alpha))
    return np.array(t_out), np.array(r_out)


def fmt(c):
    return (f'{c:>10,.1f} cnt  {c * U * 1e9:>9.3f} ns  '
            f'{c * CNT_TO_M * 100:>9.2f} cm')


def analyze(pairs, win):
    print('\n' + '=' * 74)
    print('  무선 동기화 분석 — 비콘 TX 타임스탬프 기준')
    print('=' * 74)
    print(f'\n1 카운트 = {U * 1e12:.2f} ps = {CNT_TO_M * 1000:.2f} mm')
    print(f'외삽 창  = {win} 사이클')

    resid = {}
    corrected = {}

    print('\n' + '-' * 74)
    print('1. 클럭 오프셋 및 예측 잔차')
    print('-' * 74)

    for name in sorted(pairs):
        d = pairs[name]
        if len(d) < win + 20:
            print(f'\n[{name}] 샘플 부족 ({len(d)})')
            continue

        tx = np.array([x[0] for x in d], dtype=np.float64)
        rx = np.array([x[1] for x in d], dtype=np.float64)
        oo = np.array([x[2] for x in d], dtype=np.float64)

        off = rx - tx

        # 회귀는 가장 긴 연속 구간에서만 (불연속이 기울기를 오염시킴)
        _segs, _, _ = segments(tx)
        a0, z0 = max(_segs, key=lambda s: s[1] - s[0])
        beta, _ = np.polyfit(tx[a0:z0], off[a0:z0], 1)
        ppm_fit = beta * 1e6
        ppm_o = oo.mean() / 100.0        # pphm -> ppm

        segs, bad, med = segments(tx)
        if len(segs) > 1:
            print(f'\n[{name}] 불연속 {len(bad)}곳 발견 → {len(segs)}구간으로 분할')
            for b in bad[:5]:
                print(f'    idx {b:,}: 간격 {tx[b + 1] - tx[b]:,.0f} cnt '
                      f'(중앙값 {med:,.0f})')

        t_all, r_all = [], []
        for a, z in segs:
            if z - a < win + 20:
                continue
            th, rh = holdout(tx[a:z], off[a:z], win)
            t_all.append(th)
            r_all.append(rh)
        if not t_all:
            print(f'\n[{name}] 유효 구간 없음')
            continue
        t_h = np.concatenate(t_all)
        r_h = np.concatenate(r_all)
        resid[name] = r_h
        corrected[name] = dict(zip(t_h.tolist(), r_h.tolist()))

        print(f'\n[{name}]  샘플 {len(d):,}')
        print(f'  오프셋 평균        {off.mean():>18,.0f} cnt')
        print(f'  드리프트 (회귀)    {ppm_fit:>18.4f} ppm')
        print(f'  드리프트 (O 필드)  {ppm_o:>18.4f} ppm')
        print(f'  예측 잔차 1σ   {fmt(r_h.std())}')
        print(f'  예측 잔차 P2P  {fmt(r_h.max() - r_h.min())}')

    # ---------------- AndyA 진단
    # ---------------- 무결성 진단
    print('\n' + '-' * 74)
    print('1-b. 무결성 진단 — 연속 패킷 간격')
    print('-' * 74)
    nominal = 10.17e-3 / U
    print(f'기대 간격(10.17ms) = {nominal:,.0f} cnt\n')
    print(f'{"RX":<6}{"TX간격 중앙":>16}{"RX간격 중앙":>16}{"간격이상 개수":>14}')
    for name in sorted(pairs):
        d = pairs[name]
        tx = np.diff([x[0] for x in d])
        rx = np.diff([x[1] for x in d])
        bad = int(np.sum((tx < nominal * 0.5) | (tx > nominal * 20)))
        print(f'{name:<6}{np.median(tx):>16,.0f}{np.median(rx):>16,.0f}{bad:>14,}')
    print('\n  간격이상이 0 이 아니면 unwrap 또는 바이트순서 문제입니다.')

    print('\n' + '-' * 74)
    print('2. AndyA 진단 — 잔차가 구조적인가 랜덤인가')
    print('-' * 74)
    print('  "드리프트라면 일정 속도로 변해야 한다. 랜덤하게 튀면 로직 버그."')
    print(f'\n{"RX":<6}{"1차 자기상관":>14}{"판정":>34}')
    for name in sorted(resid):
        r = resid[name]
        if r.size < 20:
            continue
        ac = np.corrcoef(r[:-1], r[1:])[0, 1]
        if abs(ac) > 0.7:
            v = '구조적 — 모델 개선 여지'
        elif abs(ac) < 0.25:
            v = '백색잡음 — 정상 (평균으로 감소)'
        else:
            v = '혼재'
        print(f'{name:<6}{ac:>14.3f}{v:>34}')

    # ---------------- TDOA
    print('\n' + '-' * 74)
    print('3. 쌍별 TDOA 산포 — TX 고정이므로 참값은 상수')
    print('-' * 74)

    names = sorted(corrected)
    worst = 0.0
    if len(names) >= 2:
        ref = names[0]
        for name in names[1:]:
            common = corrected[ref].keys() & corrected[name].keys()
            if len(common) < 50:
                print(f'\n[{ref}-{name}] 공통 패킷 부족 ({len(common)})')
                continue
            d = np.array([corrected[name][k] - corrected[ref][k]
                          for k in sorted(common)])
            s_ = d.std()
            worst = max(worst, s_)
            print(f'\n[{ref}-{name}]  공통 패킷 {len(common):,}')
            print(f'  TDOA 산포 1σ   {fmt(s_)}')

    # ---------------- 판정
    print('\n' + '=' * 74)
    print('  판정')
    print('=' * 74)
    if worst:
        m = worst * CNT_TO_M
        print(f'\n최악 TDOA 산포 1σ   {m * 100:>10.2f} cm')
        print(f'1차년도 목표        {TARGET_M * 100:>10.2f} cm')
        print(f'AndyA 달성          {6.0:>10.2f} cm')
        if m <= TARGET_M:
            print('\n>>> 목표 달성. M3(안테나 지연 캘리브레이션)로 진행.')
        elif m <= TARGET_M * 3:
            print('\n>>> 근접. 창 크기 조정 및 이상치 제거로 개선 여지 있음.')
        else:
            print('\n>>> 미달. 위 2번 자기상관을 먼저 확인할 것.')
            print('    랜덤이면 unwrap / 엔디안 / SeqNum 순환을 의심.')
    else:
        print('\n계산 불가 — 짝이 맞은 샘플이 부족합니다.')
        print('LSTN 배열이 17바이트인지, 인덱스 10~14에 값이 있는지 확인하세요.')

    return resid


def plot(resid, out='sync_residual.png'):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print('\nmatplotlib 없음 (pip3 install matplotlib)')
        return

    fig, ax = plt.subplots(figsize=(11, 5))
    for name in sorted(resid):
        ax.plot(resid[name] * CNT_TO_M * 100, '.', ms=2, label=name)
    ax.axhline(TARGET_M * 100, color='r', ls='--', lw=1, label='±30 cm')
    ax.axhline(-TARGET_M * 100, color='r', ls='--', lw=1)
    ax.set_xlabel('packet index')
    ax.set_ylabel('residual (cm)')
    ax.set_title('Hold-out prediction residual')
    ax.legend()
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    print(f'\n그래프 저장: {out}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('log')
    ap.add_argument('--window', type=int, default=10,
                    help='외삽 창 크기 (기본 10, AndyA와 동일)')
    ap.add_argument('--plot', action='store_true')
    args = ap.parse_args()

    pairs = parse(args.log)
    if not pairs:
        print('짝이 맞은 샘플이 없습니다. 펌웨어 패치를 확인하세요.')
        sys.exit(1)

    resid = analyze(pairs, args.window)
    if args.plot:
        plot(resid)


if __name__ == '__main__':
    main()