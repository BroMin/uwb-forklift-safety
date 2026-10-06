#!/usr/bin/env python3
"""
uwb_position.py — RX1 마스터 구조에서 태그 (x, y) 추정

구조
    RX1 (마스터) : 앵커 + 시각 기준. 주기적으로 동기 프레임 송신,
                   자신의 송신 시각을 UART 로 'TXTS: seq 0x...' 보고
    RX2~4        : 동기 프레임을 받아 자기 시계를 RX1 에 맞춤
    태그         : MAC 0x0002~. 앵커 4대가 수신 → TDoA

수식
    offset_i = R_i^sync − T                    (i = RX2,3,4)
             = b_i + drift + a_i·τ_i1          τ_i1 = RX1→앵커i 비행시간

    태그 패킷:
        corrected_1 = R_1^tag                  (마스터는 기준이라 보정 없음)
        corrected_i = R_i^tag − offset_i^pred

        TDoA_1i = corrected_i − corrected_1
        r_i − r_1 = c·U·(TDoA_1i + τ_i1)       ← τ_i1 은 앵커 간 거리, 기구 고정

    r_1 (기준 앵커까지의 거리)은 TWR 로 얻어야 한다. 현재 미구현이라
    --range 로 주입한다. TWR 이 붙으면 이 값만 실측으로 바꾸면 된다.

사용법
    python3 uwb_position.py <log> --range 2.0
    python3 uwb_position.py <log> --range 2.0 --plot
    python3 uwb_position.py <log> --check          # 동기 품질만 점검
"""

import argparse
import re
import sys
from collections import defaultdict

import numpy as np

# ---------------------------------------------------------------- 배치 설정

C = 299_792_458.0
U = 15.65e-12
WRAP = 2 ** 40
CNT_M = U * C                     # 4.69 mm

# 앵커 좌표 (m). 2026.08.28 배치 — 45cm 정사각형, RX1 이 원점.
#   RX4(-0.45, 0.00)   RX1(0.00, 0.00)
#   RX3(-0.45,-0.45)   RX2(0.00,-0.45)
# 태그는 +x 방향(우측)에 위치.
ANCHORS = {
    'RX1': (0.00, 0.00),
    'RX2': (0.00, -0.45),
    'RX3': (-0.50, -0.45),
    'RX4': (-0.50, 0.00),
}
MASTER = 'RX1'                    # 동기 마스터 = 기준 시계 = TDoA 기준

# ⚠️ 안테나 지연 상수 (m). 삼각부등식으로 −64.8 ~ −19.9 cm 범위 확인됨.
#    참값 역산치 −29.6 cm 를 사용하나 여전히 미검증.
DELTA = -0.296

# 태그 안테나 지연 (m). 방식 B 의 왕복 측정에 들어간다.
#    A 지점 역산 −46.4 cm. 두 지점 검증 미완료.
K_TAG = -0.464

SYNC_MAC = 0x0001
I_SEQ, I_MAC, I_PSEQ = 2, 7, 9
I_T1, I_SSEQ, I_T4 = 10, 15, 16

LINE_RE = re.compile(
    r'\[(?P<rx>RX\d)\]\s+JS[0-9A-Fa-f]{4}'
    r'\{"LSTN":\[(?P<lstn>[^\]]+)\].*?'
    r'"TS40":"0x(?P<ts>[0-9A-Fa-f]+)"'
)
TXTS_RE = re.compile(
    r'\[(?P<rx>RX\d)\]\s+TXTS:\s+(?P<seq>[0-9A-Fa-f]{2})\s+0x(?P<ts>[0-9A-Fa-f]{10})'
)


# ---------------------------------------------------------------- 유틸


def unwrapper():
    """40비트 카운터 unwrap.

    차분을 mod 2^40 으로 취하되 2^39 를 넘으면 음수로 되돌린다.
    로거가 멀티스레드라 동기 프레임과 태그 프레임의 기록 순서가
    가끔 뒤바뀌는데, 부호 없는 mod 만 쓰면 그때마다 가짜 wrap 이
    생겨 스케일이 통째로 망가진다. 실제 간격이 ±8.6초 안이면 항상 옳다.
    """
    st = {'prev': None, 'acc': 0}

    def f(v):
        if st['prev'] is None:
            st['acc'] = v
        else:
            d = (v - st['prev']) % WRAP
            if d > WRAP // 2:
                d -= WRAP
            st['acc'] += d
        st['prev'] = v
        return st['acc']

    return f


def tof_cnt(p, q):
    """두 점 사이 비행시간 (카운트)."""
    return float(np.hypot(p[0] - q[0], p[1] - q[1]) / C / U)


def unwrap_seq(lst):
    """8비트 SeqNum 을 단조 증가 인덱스로 편다.

    반환: {g: 값}.  g 는 그 앵커의 첫 수신을 0 바퀴로 본 상대 인덱스이므로
    앵커 간에는 256 의 배수만큼 어긋날 수 있다. 그 바퀴 수는 호출 측에서
    동기 모델로 보정한다.
    """
    out = {}
    prev = None
    w = 0
    for v, s in lst:
        if prev is not None and s < prev - 128:
            w += 1
        elif prev is not None and s > prev + 128:
            w -= 1
        prev = s
        out[w * 256 + s] = v
    return out


def match_by_seq(spine, other, look=12):
    """seq 로 두 시계열을 짝짓는다.

    seq 는 8비트라 256 주기로 순환하므로 dict 로 맞추면 안 된다.
    양쪽 모두 도착 순서대로 정렬되어 있다는 성질을 이용해 앞으로만 훑는다.

    spine, other : [(값, seq), ...]
    반환         : [(spine_값, other_값), ...]
    """
    out = []
    j = 0
    for i, (v, s) in enumerate(spine):
        k = j
        end = min(len(other), j + look)
        while k < end and other[k][1] != s:
            k += 1
        if k < end:
            out.append((i, v, other[k][0]))
            j = k + 1
    return out


# ---------------------------------------------------------------- 파싱


def parse(path):
    """마스터 송신 / 동기 수신 / 태그 수신을 분리한다.

    ⚠️ 마스터의 TX 타임스탬프와 RX 타임스탬프는 같은 카운터이므로
    반드시 같은 unwrap 누적기를 공유해야 한다. 분리하면 기준점이
    어긋나 좌표가 통째로 망가진다. 부호 있는 차분 unwrap 이므로
    로그 순서가 다소 뒤바뀌어도 안전하다.
    """
    tx = []                                  # [(tx_ts, seq)]
    sync = defaultdict(list)                 # rx -> [(rx_ts, seq)]
    tag = defaultdict(lambda: defaultdict(list))

    uw = defaultdict(unwrapper)
    n_ts = n_sync = n_tag = 0

    with open(path, errors='ignore') as f:
        for line in f:
            m = TXTS_RE.search(line)
            if m:
                if m.group('rx') == MASTER:
                    tx.append((uw[MASTER](int(m.group('ts'), 16)),
                               int(m.group('seq'), 16)))
                    n_ts += 1
                continue

            m = LINE_RE.search(line)
            if not m:
                continue
            name = m.group('rx')
            try:
                a = [int(x, 16) for x in m.group('lstn').split(',')]
            except ValueError:
                continue
            if len(a) < 11:
                continue

            rx_ts = uw[name](int(m.group('ts'), 16))
            mac = a[I_MAC] | (a[I_MAC + 1] << 8)
            seq = a[I_SEQ]

            if mac == SYNC_MAC:
                if name != MASTER:
                    sync[name].append((rx_ts, seq))
                    n_sync += 1
            else:
                tag[mac][name].append((rx_ts, seq))
                n_tag += 1

    print(f'TXTS {n_ts:,}   동기수신 {n_sync:,}   태그수신 {n_tag:,}')
    for mac, d in sorted(tag.items()):
        print(f'  태그 0x{mac:04X}  ' +
              '  '.join(f'{k}:{len(v):,}' for k, v in sorted(d.items())))
    return tx, sync, tag


# ---------------------------------------------------------------- 시계 모델


class ClockModel:
    """offset_i(R_i) 를 인과적 구간 선형으로 유지."""

    def __init__(self, rx, off, win=10):
        o = np.argsort(rx)
        self.rx = np.asarray(rx, dtype=np.float64)[o]
        self.off = np.asarray(off, dtype=np.float64)[o]
        self.win = win

    def predict(self, r):
        k = np.searchsorted(self.rx, r)
        a = max(0, k - self.win)
        if k - a < 3:
            return np.nan
        # 카운트값이 1e12 규모라 중심화하지 않으면 소거오차가 커진다
        x0 = self.rx[a]
        x = self.rx[a:k] - x0
        y0 = self.off[a]
        y = self.off[a:k] - y0
        n = x.size
        sx = x.sum()
        d = n * (x @ x) - sx * sx
        if d == 0:
            return np.nan
        b = (n * (x @ y) - sx * y.sum()) / d
        return y0 + b * (r - x0) + (y.sum() - b * sx) / n


class ZeroModel:
    """마스터 자신의 시계 — 보정 불필요."""

    def predict(self, r):
        return 0.0


def build_models(tx, sync, win):
    models = {MASTER: ZeroModel()}
    quality = {}
    for name, lst in sync.items():
        pairs = [(a, b) for _, a, b in match_by_seq(lst, tx)]
        if len(pairs) < win + 10:
            print(f'  [{name}] 동기 짝 부족 ({len(pairs)})')
            continue
        r = np.array([p[0] for p in pairs], dtype=np.float64)
        t = np.array([p[1] for p in pairs], dtype=np.float64)
        models[name] = ClockModel(r, r - t, win)

        # 품질: 실제 사용하는 슬라이딩 창 예측의 잔차
        off = r - t
        res = []
        for k in range(win, len(off)):
            x = t[k - win:k]
            y = off[k - win:k]
            b, a = np.polyfit(x, y, 1)
            res.append(off[k] - (b * t[k] + a))
        res = np.array(res) if res else np.array([np.nan])
        slope, _ = np.polyfit(t, off, 1)
        quality[name] = (len(pairs), slope * 1e6, res.std(),
                         np.median(np.abs(res)))
    return models, quality


# ---------------------------------------------------------------- 좌표


def solve_xy(names, ranges):
    A = np.array([ANCHORS[n] for n in names], dtype=np.float64)
    r = np.asarray(ranges, dtype=np.float64)
    M = 2.0 * (A[1:] - A[0])
    b = (A[1:] ** 2).sum(1) - (A[0] ** 2).sum() - r[1:] ** 2 + r[0] ** 2
    sol, *_ = np.linalg.lstsq(M, b, rcond=None)
    return sol


def locate(tag_rx, models, r_master, verbose=True):
    names = [MASTER] + [n for n in ANCHORS
                        if n != MASTER and n in tag_rx and n in models]
    if MASTER not in tag_rx:
        print(f'마스터 {MASTER} 의 태그 수신 없음')
        return None, None
    if len(names) < 3:
        print(f'앵커 부족: {names}')
        return None, None

    tau = {n: tof_cnt(ANCHORS[MASTER], ANCHORS[n]) for n in names}
    G = {n: unwrap_seq(tag_rx[n]) for n in names}
    g0 = G[MASTER]

    def dts(n, shift):
        out = []
        for g, r1 in g0.items():
            ri = G[n].get(g + shift)
            if ri is None:
                continue
            off = models[n].predict(ri)
            if np.isfinite(off):
                out.append((g, (ri - off) - r1 + tau[n]))
        return out

    # 앵커 간 바퀴 어긋남(256의 배수)을 동기 모델 기준으로 자동 보정
    shift = {}
    for n in names[1:]:
        d = dts(n, 0)
        if len(d) < 20:
            print(f'  [{n}] 짝 부족')
            return None, None
        med = np.median([x[1] for x in d])
        # 태그 송신 주기(카운트) 추정
        gs = sorted(g0)
        per = (g0[gs[-1]] - g0[gs[0]]) / (gs[-1] - gs[0])
        c = int(round(med / (256.0 * per)))
        shift[n] = -c * 256
        if verbose and c:
            print(f'  [{n}] SeqNum {c:+d}바퀴 어긋남 → 보정')

    rows = {n: dict(dts(n, shift[n])) for n in names[1:]}
    common = set(rows[names[1]])
    for n in names[2:]:
        common &= set(rows[n])
    if len(common) < 10:
        print(f'앵커 간 태그 짝 부족 ({len(common)})')
        return None, None

    ts, pts = [], []
    for g in sorted(common):
        ranges = [r_master] + [r_master + rows[n][g] * CNT_M for n in names[1:]]
        ts.append(g0[g])
        pts.append(solve_xy(names, ranges))
    return np.array(ts), np.array(pts)


# ---------------------------------------------------------------- main


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('log')
    ap.add_argument('--range', type=float, default=None,
                    help=f'{MASTER} 까지의 거리 (m). TWR 대체값')
    ap.add_argument('--tag-mac', type=lambda x: int(x, 0), default=None)
    ap.add_argument('--window', type=int, default=10)
    ap.add_argument('--check', action='store_true', help='동기 품질만 점검')
    ap.add_argument('--plot', action='store_true')
    args = ap.parse_args()

    print('앵커: ' + '  '.join(f'{k}{v}' for k, v in ANCHORS.items()))
    print(f'마스터/기준: {MASTER}\n')

    tx, sync, tags = parse(args.log)
    models, q = build_models(tx, sync, args.window)

    print('\n동기 품질 (전체 회귀 기준)')
    print(f'{"RX":<6}{"짝":>8}{"드리프트":>13}{"잔차1σ":>17}{"중앙절대":>14}')
    for n in sorted(q):
        cnt, ppm, res, mad = q[n]
        print(f'{n:<6}{cnt:>8,}{ppm:>10.4f} ppm'
              f'{res:>9.1f}cnt {res * CNT_M * 100:>5.2f}cm'
              f'{mad:>8.1f}cnt {mad * CNT_M * 100:>5.2f}cm')

    if args.check:
        return
    if not tags:
        print('\n태그 패킷 없음')
        sys.exit(1)
    if args.range is None:
        ap.error('--range 필요')

    mac = (args.tag_mac if args.tag_mac
           else max(tags, key=lambda m: sum(len(v) for v in tags[m].values())))
    t, p = locate(tags[mac], models, args.range)
    if p is None:
        sys.exit(1)

    med = np.median(p, axis=0)
    err = np.hypot(p[:, 0] - med[0], p[:, 1] - med[1])
    keep = err < np.percentile(err, 95)

    print(f'\n태그 0x{mac:04X}  좌표 {len(p):,} 개')
    print(f'  중앙값      ({med[0]:+.3f}, {med[1]:+.3f}) m')
    print(f'  산포 1σ     x {p[:, 0].std() * 100:6.1f} cm   '
          f'y {p[:, 1].std() * 100:6.1f} cm')
    print(f'  RMS         {err.mean() * 100:6.1f} cm   '
          f'(이상치 5% 제거 {err[keep].mean() * 100:.1f} cm)')
    print(f'  P95         {np.percentile(err, 95) * 100:6.1f} cm')

    if args.plot:
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
        except ImportError:
            print('\nmatplotlib 없음')
            return
        fig, ax = plt.subplots(figsize=(7, 7))
        A = np.array(list(ANCHORS.values()))
        ax.plot(p[keep, 0], p[keep, 1], '.', ms=3, alpha=0.3, label='tag')
        ax.plot(A[:, 0], A[:, 1], 's', ms=10, label='anchors')
        ax.plot(*ANCHORS[MASTER], 'D', ms=10, label='master')
        ax.plot(*med, 'x', ms=14, mew=3, color='k', label='median')
        ax.set_aspect('equal')
        ax.grid(alpha=0.3)
        ax.legend()
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
        fig.tight_layout()
        fig.savefig('position.png', dpi=130)
        print('\n그래프 저장: position.png')


if __name__ == '__main__':
    main()