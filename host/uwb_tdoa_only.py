#!/usr/bin/env python3
"""
uwb_tdoa_only.py — TDoA 만으로 위치가 풀리는지 직접 확인

TWR 없이, 거리 차이 3개만으로 (x, y, r1) 세 미지수를 푼다.
이론적으로는 풀린다. 앵커가 충분히 벌어져 있다면.

두 가지를 본다.
  ① 자기일관 해   — r1 을 미지수로 두고 풀었을 때 어디로 수렴하는가
  ② 스캔          — r1 을 0.3~10 m 로 바꿔가며 잔차가 얼마나 변하는가

잔차가 r1 에 둔감하면 = 거리 정보가 없다는 뜻.
참값을 알고 있으므로 추정이 얼마나 벗어나는지도 같이 본다.

사용법
    python3 uwb_tdoa_only.py <log> --true-x 1.6 --true-y 0.0
    (uwb_position.py 가 같은 디렉토리에 있어야 함)
"""

import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
src = open(os.path.join(HERE, 'uwb_position.py'), encoding='utf-8').read()
exec(src.split("def main()")[0])          # parse, build_models, ANCHORS ...

NAMES = ['RX1', 'RX2', 'RX3', 'RX4']
A = np.array([ANCHORS[n] for n in NAMES], dtype=np.float64)


def median_tdoa(path):
    """앵커별 거리 차이 (RX1 기준, m) 중앙값."""
    tx, sync, tags = parse(path)
    models, _ = build_models(tx, sync, 10)
    mac = max(tags, key=lambda m: sum(len(v) for v in tags[m].values()))
    tag = tags[mac]
    G = {n: unwrap_seq(tag[n]) for n in NAMES if n in tag}
    if MASTER not in G:
        return None
    g0 = G[MASTER]

    # 앵커 간 SeqNum 바퀴 보정 (uwb_position.py 와 동일 원리)
    shift = {}
    for n in NAMES[1:]:
        if n not in G:
            return None
        best = None
        for c in range(-8, 9):
            hit = sum(1 for g in g0 if g + c * 256 in G[n])
            if best is None or hit > best[1]:
                best = (c * 256, hit)
        shift[n] = best[0]

    out = {}
    for n in NAMES[1:]:
        v = []
        for g, r1 in g0.items():
            ri = G[n].get(g + shift[n])
            if ri is None:
                continue
            off = models[n].predict(ri)
            if np.isfinite(off):
                v.append(((ri - off) - r1 + tof_cnt(ANCHORS[MASTER],
                                                    ANCHORS[n])) * CNT_M)
        if len(v) < 30:
            return None
        out[n] = float(np.median(v))
    return out


def resid(p, d):
    """세 쌍곡선 식의 RMS 잔차 (m)."""
    dd = np.linalg.norm(p - A, axis=1)
    return float(np.sqrt((((dd[1:] - dd[0]) - d) ** 2).mean()))


def solve_fixed_r1(d, r1):
    """r1 을 주면 선형 닫힌 해로 (x, y)."""
    r = np.concatenate([[r1], r1 + d])
    M = 2.0 * (A[1:] - A[0])
    b = (A[1:] ** 2).sum(1) - (A[0] ** 2).sum() - r[1:] ** 2 + r[0] ** 2
    return np.linalg.lstsq(M, b, rcond=None)[0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('log')
    ap.add_argument('--true-x', type=float, required=True)
    ap.add_argument('--true-y', type=float, default=0.0)
    ap.add_argument('--delta', type=float, default=0.0,
                    help='안테나 지연 보정 (m). 미지정 시 0')
    args = ap.parse_args()

    truth = np.array([args.true_x, args.true_y])
    d0 = median_tdoa(args.log)
    if d0 is None:
        print('TDoA 산출 실패')
        sys.exit(1)
    d = np.array([d0[n] for n in NAMES[1:]]) + args.delta

    print('\n' + '=' * 72)
    print('  실측 거리 차이 (RX1 기준)')
    print('=' * 72)
    B = [float(np.hypot(*(np.array(ANCHORS[n]) - ANCHORS[MASTER])))
         for n in NAMES[1:]]
    print(f'\n{"":6}{"측정":>10}{"앵커간 거리":>14}{"삼각부등식":>12}')
    for n, v, b in zip(NAMES[1:], d, B):
        print(f'{n:6}{v:>+9.4f}m{b:>13.3f}m'
              f'{"  OK" if abs(v) <= b else "  위반":>12}')

    print(f'\n참값 태그 위치 ({truth[0]:+.3f}, {truth[1]:+.3f}) m   '
          f'RX1 로부터 {np.linalg.norm(truth):.3f} m')

    print('\n' + '=' * 72)
    print('  r1 을 바꿔가며 — TDoA 에 거리 정보가 있는가')
    print('=' * 72)
    print(f'\n{"가정 r1":>9}{"해 (x, y)":>22}{"해의 거리":>12}'
          f'{"잔차":>10}{"참값과 차이":>12}')
    rows = []
    for r1 in (0.3, 0.5, 0.8, 1.0, 1.5, 2.0, 3.0, 5.0, 8.0, 10.0):
        p = solve_fixed_r1(d, r1)
        rows.append((r1, p, resid(p, d), float(np.linalg.norm(p - truth))))
        print(f'{r1:>8.1f}m ({p[0]:>+7.3f},{p[1]:>+7.3f})'
              f'{np.linalg.norm(p):>11.3f}m{rows[-1][2] * 100:>9.2f}cm'
              f'{rows[-1][3] * 100:>11.1f}cm')

    rs = np.array([r[2] for r in rows])
    print(f'\n잔차 변화폭 {(rs.max() - rs.min()) * 100:.2f} cm '
          f'(r1 을 {rows[0][0]}~{rows[-1][0]} m 로 33배 바꿨을 때)')
    print(f'측정 잡음    약 3 cm')
    if (rs.max() - rs.min()) < 0.06:
        print('\n>>> 잔차가 잡음보다 작게 변합니다.')
        print('    = r1 을 어떤 값으로 두든 데이터가 구분하지 못합니다.')
        print('    = TDoA 만으로는 거리를 정할 수 없습니다.')
    else:
        print('\n>>> 잔차가 r1 에 반응합니다. 거리 정보가 있을 수 있습니다.')

    # 자기일관 해
    print('\n' + '=' * 72)
    print('  자기일관 해 — r1 도 미지수로 두고 풀기')
    print('=' * 72)
    best = None
    for r1 in np.arange(0.2, 20.0, 0.01):
        p = solve_fixed_r1(d, r1)
        gap = abs(np.linalg.norm(p - A[0]) - r1)      # |p−RX1| 이 r1 과 같아야
        if best is None or gap < best[0]:
            best = (gap, r1, p)
    gap, r1, p = best
    print(f'\n|p − RX1| = r1 을 만족하는 r1 = {r1:.2f} m')
    print(f'  해 ({p[0]:+.3f}, {p[1]:+.3f})   RX1 로부터 {np.linalg.norm(p):.3f} m')
    print(f'  참값과 차이 {np.linalg.norm(p - truth) * 100:.1f} cm')
    if r1 > 15:
        print('  ⚠️ 탐색 상한까지 발산 — 해가 정해지지 않습니다.')

    # 방위는 어떤가
    print('\n' + '=' * 72)
    print('  방위는 정해지는가')
    print('=' * 72)
    th_true = np.degrees(np.arctan2(truth[1] - A[0][1], truth[0] - A[0][0]))
    print(f'\n참값 방위 {th_true:+.1f}°\n')
    print(f'{"가정 r1":>9}{"방위":>10}{"참값과 차이":>12}')
    for r1, p, _, _ in rows:
        th = np.degrees(np.arctan2(p[1] - A[0][1], p[0] - A[0][0]))
        print(f'{r1:>8.1f}m{th:>9.1f}°{(th - th_true):>+11.1f}°')


if __name__ == '__main__':
    main()