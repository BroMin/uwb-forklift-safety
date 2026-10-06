#!/usr/bin/env python3
"""
uwb_realtime.py — 실시간 태그 측위

RX1(마스터)이 동기 프레임을 쏘고 RX2~4가 받아 시계를 맞춘 상태에서,
태그 blink 를 4대가 수신하면 즉시 TDoA → 좌표를 산출한다.

구조
    UART 4채널 수신 (스레드 4개)
        → 줄 파싱 → 시계 모델 갱신(deque)
        → 태그 패킷 짝맞춤 → dt 3개 → 보정 → 좌표
        → 터미널 출력 + CSV 저장 (+ 선택적 실시간 그래프)

⚠️ ANT_DELAY_M 은 아직 검증되지 않은 값이다 (태그 1곳에서 피팅).
   CSV 에 보정 전 원시 dt 를 함께 저장하므로, 나중에 상수가 확정되면
   기존 데이터를 그대로 재계산할 수 있다.

사용법
    python3 uwb_realtime.py --range 1.32
    python3 uwb_realtime.py --range 1.32 --plot
    python3 uwb_realtime.py --range 1.32 --quiet        # 통계만
    python3 uwb_realtime.py --replay <csv> --delta -0.30  # 저장분 재계산
"""

import argparse
import csv
import datetime
import os
import re
import sys
import threading
import time
from collections import defaultdict, deque

import numpy as np

# ---------------------------------------------------------------- 설정

C = 299_792_458.0
U = 15.65e-12
WRAP = 2 ** 40
CNT_M = U * C                      # 4.69 mm

SN_MAP = {
    '760197764': 'RX1',
    '760144486': 'RX2',
    '760143773': 'RX3',
    '760197326': 'RX4',
}
BAUD = 230400

ANCHORS = {
    'RX1': (0.00, 0.00),
    'RX2': (0.00, -0.45),
    'RX3': (-0.50, -0.45),    
    'RX4': (-0.50, 0.00),     
}
MASTER = 'RX1'
SLAVES = [n for n in ANCHORS if n != MASTER]

# ⚠️ 미검증 상수 2개. uwb_xy.py --calib 로 구한 값을 넣을 것.
#    DELTA : 마스터 TX/RX 안테나 지연 차 — 세 TDoA 에 공통으로 더해짐
#    K     : 태그 TX/RX 안테나 지연 차   — 왕복 측정에 들어감
ANT_DELAY_M = -0.296
TAG_K_M = -0.643

# 태그 프레임 추가 필드
I_PSEQ, I_T1, I_SSEQ, I_T4 = 9, 10, 15, 16

WIN = 10                           # 시계 모델 창 (AndyA 와 동일)
PENDING = 600                      # 보류 큐. 앵커 간 로그 순서 어긋남을 흡수
BUFLEN = 1500                      # 앵커별 태그 버퍼
ALIGN_VOTE = 5                     # 바퀴 보정 확정에 필요한 동일 후보 수
REALIGN_AFTER = 300                # 연속 조회 실패 시 재정렬
MAX_EXTRAP_S = 0.3                 # 시계 모델 외삽 허용 시간(초).
                                   # 이보다 오래된 모델로는 좌표를 내지 않는다
SYNC_MAC = 0x0001
I_SEQ, I_MAC = 2, 7

LOG_DIR = os.path.expanduser('~/uwb_logs')

LINE_RE = re.compile(
    r'JS[0-9A-Fa-f]{4}\{"LSTN":\[(?P<lstn>[^\]]+)\].*?'
    r'"TS40":"0x(?P<ts>[0-9A-Fa-f]+)".*?'
    r'"O":(?P<o>-?\d+),"rsl":(?P<rsl>-?[\d.]+),"fsl":(?P<fsl>-?[\d.]+)'
)
TXTS_RE = re.compile(r'TXTS:\s+(?P<seq>[0-9A-Fa-f]{2})\s+0x(?P<ts>[0-9A-Fa-f]{10})')


# ---------------------------------------------------------------- 유틸


def unwrapper():
    """40비트 unwrap. 부호 있는 차분이라 순서가 다소 뒤바뀌어도 안전."""
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
    return float(np.hypot(p[0] - q[0], p[1] - q[1]) / C / U)


TAU = {n: tof_cnt(ANCHORS[MASTER], ANCHORS[n]) for n in ANCHORS}

_A = np.array([ANCHORS[MASTER]] + [ANCHORS[n] for n in SLAVES], dtype=np.float64)
_A2 = (_A ** 2).sum(1)
_M = 2.0 * (_A[1:] - _A[0])
_MP = np.linalg.inv(_M.T @ _M) @ _M.T      # 앵커 고정이므로 미리 계산


def solve_xy(r):
    """거리 4개 → (x, y). 닫힌 해."""
    b = _A2[1:] - _A2[0] - r[1:] ** 2 + r[0] ** 2
    return _MP @ b


def fit_predict(xs, ys, x):
    """1차 회귀 후 x 에서 예측. 중심화로 소거오차 방지."""
    x0, y0 = xs[0], ys[0]
    a = np.fromiter((v - x0 for v in xs), float, len(xs))
    b = np.fromiter((v - y0 for v in ys), float, len(ys))
    n = a.size
    sa = a.sum()
    d = n * (a @ a) - sa * sa
    if d == 0:
        return np.nan
    k = (n * (a @ b) - sa * b.sum()) / d
    return y0 + k * (x - x0) + (b.sum() - k * sa) / n


# ---------------------------------------------------------------- 상태


class Tracker:
    """앵커별 시계 모델과 태그 패킷 버퍼를 유지한다.

    성능: 패킷마다 보류 목록을 전수 검사하면 O(n²) 이 되어 실시간 여유가
    사라진다. 앵커별 수신 여부를 세는 카운터로 O(1) 처리한다.
    """

    CYCLE = 256 * 10.19e-3 / U      # 바퀴 어긋남 판별용 (근사)

    def __init__(self, tag_k, delta):
        self.tag_k = tag_k
        self.delta = delta
        self.lock = threading.Lock()

        self.uw = defaultdict(unwrapper)
        self.tx = deque(maxlen=512)                       # (tx_ts, seq)
        # 시계 모델은 시각순으로 정렬 보관. 예측 시 대상 시각 직전 WIN개만 사용.
        self.model = {n: [] for n in SLAVES}              # [(rx_ts, offset)]
        self.tag = {n: {} for n in ANCHORS}               # g -> (rx_ts, rsl)
        self.order = {n: deque(maxlen=BUFLEN) for n in ANCHORS}
        self.gc = {n: 0 for n in ANCHORS}
        self.pseq = {n: None for n in ANCHORS}
        self.shift = {n: None for n in SLAVES}            # 앵커 간 바퀴 차
        self.vote = {n: defaultdict(int) for n in SLAVES}  # 정렬 후보 투표
        self.miss = {n: 0 for n in SLAVES}                # 연속 조회 실패
        self.ready = defaultdict(set)                     # gm -> 수신한 앵커
        # TXTS 가 아직 안 읽힌 동기 프레임 보류. 스레드 순서가 뒤바뀔 수 있어
        # 한 번 실패하면 버리지 말고 재시도해야 한다.
        self.psync = {n: deque(maxlen=400) for n in SLAVES}

        # --- 방식 B: 태그 시계 모델 ---
        # 태그의 t1(송신)과 t4(동기 수신)는 같은 카운터이므로
        # 반드시 하나의 unwrapper 를 공유해야 한다.
        self.uw_tag = unwrapper()
        self.tmodel = []                                  # [(t4, t4 − T)]
        self.tsync = deque(maxlen=400)                    # TXTS 미도착 보류
        self.tshift = None                                # sync_seq 바퀴 차
        self.tvote = defaultdict(int)
        self.tgc = 0
        self.tps = None
        self.rep = {}                                     # gp -> (t1, gs)
        self.fixes = []
        self.stat = defaultdict(int)

    # ---- 입력

    def on_txts(self, seq, raw):
        with self.lock:
            self.tx.append((self.uw[MASTER](raw), seq))
            self.stat['txts'] += 1
            for n in SLAVES:                       # 보류된 동기 프레임 재시도
                if self.psync[n]:
                    self._flush_sync(n)

    def _flush_sync(self, n):
        left = []
        for ts, seq in self.psync[n]:
            for t, sq in reversed(self.tx):
                if sq == seq:
                    md = self.model[n]
                    md.append((ts, ts - t))
                    if len(md) > 400:
                        del md[:100]
                    break
            else:
                left.append((ts, seq))
        self.psync[n] = deque(left, maxlen=400)
        self.model[n].sort(key=lambda x: x[0])

    def on_lstn(self, name, raw, lstn, rsl):
        with self.lock:
            ts = self.uw[name](raw)
            mac = lstn[I_MAC] | (lstn[I_MAC + 1] << 8)
            seq = lstn[I_SEQ]

            if mac == SYNC_MAC:
                if name == MASTER:
                    return
                self.stat['sync'] += 1
                self.psync[name].append((ts, seq))
                self._flush_sync(name)
                return

            self.stat['tag'] += 1
            ps = self.pseq[name]
            if ps is not None:
                if seq < ps - 128:
                    self.gc[name] += 256
                elif seq > ps + 128:
                    self.gc[name] -= 256
            self.pseq[name] = seq
            g = self.gc[name] + seq

            if len(self.order[name]) == self.order[name].maxlen:
                self.tag[name].pop(self.order[name][0], None)
            self.tag[name][g] = (ts, rsl)
            self.order[name].append(g)

            if name == MASTER:
                gm = g
                self._on_report(g, lstn)
            else:
                if self.shift[name] is None and not self._align(name, g):
                    return
                gm = g - self.shift[name]

            self.ready[gm].add(name)
            self._maybe_fix(gm)
            if len(self.ready) > 6000:                    # 미완성 항목 정리
                for k in sorted(self.ready)[:3000]:
                    self.ready.pop(k, None)

    def _maybe_fix(self, g):
        """앵커 4대 수신 + 태그의 t1 보고가 모두 갖춰지면 계산한다.

        t1 은 지연 보고라 다음 blink 에 실려 오므로, 두 조건의 도착 순서가
        정해져 있지 않다. 양쪽에서 이 함수를 호출한다.
        """
        if len(self.ready.get(g, ())) != len(ANCHORS):
            return
        if g not in self.rep:
            return
        self._fix_one(g)
        self.ready.pop(g, None)
        self.rep.pop(g, None)

    # ---- 내부

    def _on_report(self, gb, lstn):
        """태그가 실어 보낸 t1 · t4 · sync_seq 를 받는다.

        보고된 t1 은 pseq 가 가리키는 직전 blink 의 송신 시각이다.
        """
        t1 = 0
        for b in lstn[I_T1:I_T1 + 5]:
            t1 = (t1 << 8) | b
        t4 = 0
        for b in lstn[I_T4:I_T4 + 5]:
            t4 = (t4 << 8) | b
        if t1 == 0 or t4 == 0:
            self.stat['tag_nodata'] += 1
            return

        v1 = self.uw_tag(t1)                  # t1 < t4 순으로 단조
        v4 = self.uw_tag(t4)

        # g % 256 == seq 이므로 pseq 로 직전 blink 를 특정
        gp = gb - ((gb - lstn[I_PSEQ]) % 256)
        self.rep[gp] = v1
        self._maybe_fix(gp)
        if len(self.rep) > 800:
            for k in sorted(self.rep)[:400]:
                self.rep.pop(k, None)

        self.tsync.append((v4, lstn[I_SSEQ]))
        self._flush_tsync()

    def _flush_tsync(self):
        """t4 를 TXTS 와 짝지어 태그 시계 모델에 넣는다.

        앵커 동기와 같은 방식으로 '가장 최근의 같은 seq' 를 쓴다.
        실시간에서는 이게 항상 옳고, 256 바퀴 모호성도 생기지 않는다.
        """
        left = []
        for v4, sq in self.tsync:
            T = None
            for t, s2 in reversed(self.tx):
                if s2 == sq:
                    T = t
                    break
            if T is None:
                left.append((v4, sq))
                continue
            self.tmodel.append((v4, v4 - T))
            if len(self.tmodel) > 400:
                del self.tmodel[:100]
        self.tsync = deque(left, maxlen=400)
        self.tmodel.sort(key=lambda x: x[0])

    def _tpredict(self, r):
        """태그 offset 을 대상 시각 직전 WIN개로 외삽."""
        md = self.tmodel
        if len(md) < 3:
            return np.nan
        k = len(md)
        while k > 0 and md[k - 1][0] > r:
            k -= 1
        a = max(0, k - WIN)
        if k - a < 3:
            return np.nan
        if (r - md[k - 1][0]) * U > MAX_EXTRAP_S:
            return np.nan
        return fit_predict([x[0] for x in md[a:k]],
                           [x[1] for x in md[a:k]], r)

    def _predict(self, name, r):
        """대상 시각 r 직전 WIN개 샘플로 외삽. 오래된 샘플을 쓰면 정확도가 떨어진다."""
        md = self.model[name]
        if len(md) < 3:
            return np.nan
        k = len(md)
        while k > 0 and md[k - 1][0] > r:
            k -= 1
        a = max(0, k - WIN)
        if k - a < 3:
            return np.nan
        # 모델이 너무 낡으면 외삽 오차가 커진다 → 좌표를 내지 않음
        if (r - md[k - 1][0]) * U > MAX_EXTRAP_S:
            return np.nan
        return fit_predict([x[0] for x in md[a:k]],
                           [x[1] for x in md[a:k]], r)

    def _align(self, n, g):
        """앵커 간 SeqNum 바퀴 차를 정한다.

        두 앵커는 같은 seq 열을 보므로 g 차이는 **반드시 256의 배수**다.
        이 제약을 강제하고, 256 배수 후보 중 시간차가 0에 가장 가까운 것을
        고른다. 주기 추정이 필요 없어 견고하다.
        """
        ri, _ = self.tag[n][g]
        off = self._predict(n, ri)
        if not np.isfinite(off):
            return False

        best = None
        for k in range(-8, 9):
            hit = self.tag[MASTER].get(g + k * 256)
            if hit is None:
                continue
            dt = (ri - off) - hit[0] + TAU[n]
            if best is None or abs(dt) < abs(best[1]):
                best = (k, dt)
        if best is None:
            return False

        cand = -best[0] * 256
        v = self.vote[n]
        v[cand] += 1
        if v[cand] >= ALIGN_VOTE:
            self.shift[n] = cand
            self.stat['align_' + n] = cand
            return True
        return False

    def _fix_one(self, gm):
        hit = self.tag[MASTER].get(gm)
        if hit is None:
            return
        r1, rsl1 = hit

        # --- 거리: 태그 시계 모델로 t1 을 마스터 시간축으로 옮겨 왕복 측정
        t1 = self.rep.get(gm)
        if t1 is None:
            self.stat['no_t1'] += 1
            return
        off_t = self._tpredict(t1)
        if not np.isfinite(off_t):
            self.stat['no_tmodel'] += 1
            return
        tof1 = (r1 - (t1 - off_t)) * CNT_M
        d1 = (tof1 - self.tag_k) / 2.0
        if not (0.05 < d1 < 60.0):
            self.stat['bad_d1'] += 1
            return

        # --- 방위: TDoA
        dts, rsls = [], [rsl1]
        for n in SLAVES:
            got = self.tag[n].get(gm + self.shift[n])
            if got is None:
                self.stat['no_pair'] += 1
                self.miss[n] += 1
                if self.miss[n] > REALIGN_AFTER:
                    self.shift[n] = None
                    self.vote[n].clear()
                    self.miss[n] = 0
                    self.stat['realign'] += 1
                return
            ri, rsli = got
            off = self._predict(n, ri)
            if not np.isfinite(off):
                self.stat['no_model'] += 1
                return
            self.miss[n] = 0
            dts.append((ri - off) - r1 + TAU[n])
            rsls.append(rsli)

        rng = np.array([d1] + [d1 + d * CNT_M + self.delta for d in dts])
        p = solve_xy(rng)
        # Gauss-Newton 다듬기 (산포 30 → 20cm 개선 확인됨)
        for _ in range(6):
            v = p - _A
            dd = np.linalg.norm(v, axis=1)
            try:
                p = p - np.linalg.lstsq(v / dd[:, None], dd - rng,
                                        rcond=None)[0]
            except np.linalg.LinAlgError:
                break
        self.fixes.append((time.time(), gm & 0xFF, p[0], p[1],
                           d1, dts, rsls))
        self.stat['fix'] += 1

    def drain(self):
        with self.lock:
            out, self.fixes = self.fixes, []
            return out


# ---------------------------------------------------------------- 수신


def find_port(sn):
    base = '/dev/serial/by-id/'
    try:
        for e in os.listdir(base):
            if sn in e and 'if00' in e:
                return os.path.join(base, e)
    except OSError:
        pass
    return None


def reader(sn, name, tr, stop):
    import serial
    port = find_port(sn)
    if not port:
        print(f'[{name}] 포트 없음 (SN={sn})')
        return
    try:
        ser = serial.Serial(port, BAUD, timeout=0.2)
    except Exception as e:
        print(f'[{name}] 오픈 실패: {e}')
        return
    print(f'[{name}] 접속 {port}')
    buf = ''
    while not stop.is_set():
        try:
            chunk = ser.read(ser.in_waiting or 1).decode('utf-8', 'ignore')
        except Exception as e:
            print(f'[{name}] 읽기 오류: {e}')
            break
        if not chunk:
            continue
        buf += chunk
        while '\n' in buf:
            line, buf = buf.split('\n', 1)
            line = line.strip()
            if not line:
                continue
            m = TXTS_RE.search(line)
            if m:
                if name == MASTER:
                    tr.on_txts(int(m.group('seq'), 16), int(m.group('ts'), 16))
                continue
            m = LINE_RE.search(line)
            if not m:
                continue
            try:
                a = [int(x, 16) for x in m.group('lstn').split(',')]
            except ValueError:
                continue
            if len(a) < 11:
                continue
            tr.on_lstn(name, int(m.group('ts'), 16), a, float(m.group('rsl')))
        if len(buf) > 8192:
            buf = buf[-1024:]
    ser.close()


# ---------------------------------------------------------------- 재계산


def replay(path, delta, tag_k=None):
    """저장된 CSV 의 원시 dt·ToF1 로 좌표를 다시 계산한다.

    안테나 지연 상수가 확정되면 기존 데이터를 그대로 재활용할 수 있다.
    """
    P = []
    with open(path) as f:
        for row in csv.DictReader(f):
            d1 = (float(row['tof1']) - tag_k) / 2 if tag_k else float(row['d1'])
            dts = [float(row[f'dt_{n}']) for n in SLAVES]
            rng = np.array([d1] + [d1 + d * CNT_M + delta for d in dts])
            p = solve_xy(rng)
            for _ in range(6):
                v = p - _A
                dd = np.linalg.norm(v, axis=1)
                try:
                    p = p - np.linalg.lstsq(v / dd[:, None], dd - rng,
                                            rcond=None)[0]
                except np.linalg.LinAlgError:
                    break
            P.append(p)
    P = np.array(P)
    med = np.median(P, axis=0)
    e = np.hypot(P[:, 0] - med[0], P[:, 1] - med[1])
    k = e < np.percentile(e, 95)
    print(f'재계산 {len(P):,} 개   delta={delta * 100:+.1f} cm'
          + (f'  K={tag_k * 100:+.1f} cm' if tag_k else ''))
    print(f'  중앙값 ({med[0]:+.3f}, {med[1]:+.3f}) m')
    print(f'  산포 {e[k].mean() * 100:.1f} cm (이상치 5% 제거)  '
          f'P95 {np.percentile(e, 95) * 100:.1f} cm')
    return P


# ---------------------------------------------------------------- main


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--k', type=float, default=TAG_K_M,
                    help='태그 안테나 지연 (m). ⚠️ 미검증 기본값')
    ap.add_argument('--delta', type=float, default=ANT_DELAY_M,
                    help='안테나 지연 보정 (m). ⚠️ 미검증 기본값')
    ap.add_argument('--plot', action='store_true', help='실시간 그래프')
    ap.add_argument('--quiet', action='store_true', help='좌표 줄 출력 생략')
    ap.add_argument('--replay', help='저장된 CSV 재계산')
    args = ap.parse_args()

    if args.replay:
        replay(args.replay, args.delta, args.k)
        return

    os.makedirs(LOG_DIR, exist_ok=True)
    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    csv_path = f'{LOG_DIR}/fix_{stamp}.csv'
    fcsv = open(csv_path, 'w', newline='')
    w = csv.writer(fcsv)
    w.writerow(['wall', 'seq', 'x', 'y', 'd1', 'tof1', 'k', 'delta'] +
               [f'dt_{n}' for n in SLAVES] +
               ['rsl_' + MASTER] + [f'rsl_{n}' for n in SLAVES])

    print(f'앵커 ' + '  '.join(f'{k}{v}' for k, v in ANCHORS.items()))
    print(f'마스터 {MASTER}   거리: TWR 실측')
    print(f'K {args.k * 100:+.1f} cm   delta {args.delta * 100:+.1f} cm  '
          f'⚠️ 둘 다 미검증')
    print(f'CSV {csv_path}\nCtrl+C 로 종료\n')

    tr = Tracker(args.k, args.delta)
    stop = threading.Event()
    for sn, name in SN_MAP.items():
        threading.Thread(target=reader, args=(sn, name, tr, stop),
                         daemon=True).start()

    ax = None
    if args.plot:
        try:
            import matplotlib
            import matplotlib.pyplot as plt
            plt.ion()
            fig, ax = plt.subplots(figsize=(7, 7))
            A = np.array(list(ANCHORS.values()))
            ax.plot(A[:, 0], A[:, 1], 's', ms=10, color='tab:orange')
            ax.plot(*ANCHORS[MASTER], 'D', ms=10, color='tab:green')
            sc = ax.plot([], [], '.', ms=4, alpha=0.4, color='tab:blue')[0]
            cur = ax.plot([], [], 'x', ms=14, mew=3, color='k')[0]
            ax.set_aspect('equal')
            ax.grid(alpha=0.3)
            ax.set_xlabel('x (m)')
            ax.set_ylabel('y (m)')
        except ImportError:
            print('matplotlib 없음 — 그래프 생략')
            ax = None

    xs, ys = deque(maxlen=400), deque(maxlen=400)
    t0 = time.time()
    last = t0
    try:
        while True:
            time.sleep(0.05)
            for wall, seq, x, y, d1, dts, rsls in tr.drain():
                w.writerow([f'{wall:.6f}', seq, f'{x:.4f}', f'{y:.4f}',
                            f'{d1:.4f}', f'{2*d1+args.k:.4f}',
                            args.k, args.delta] +
                           [f'{d:.2f}' for d in dts] +
                           [f'{r:.2f}' for r in rsls])
                xs.append(x)
                ys.append(y)
                if not args.quiet:
                    print(f'\r({x:+7.3f}, {y:+7.3f}) m   '
                          f'TWR {d1:5.2f} m   '
                          f'해 {np.hypot(x - ANCHORS[MASTER][0], y - ANCHORS[MASTER][1]):5.2f} m   '
                          f'방위 {np.degrees(np.arctan2(y - ANCHORS[MASTER][1], x - ANCHORS[MASTER][0])):+7.1f}°   '
                          f'rsl {min(rsls):6.1f}dBm', end='')

            now = time.time()
            if now - last >= 0.3:
                fcsv.flush()
                el = now - t0
                s = tr.stat
                if args.quiet or True:
                    print(f'\n[{el:6.1f}s] TXTS {s["txts"]:,}  동기 {s["sync"]:,}  '
                          f'태그 {s["tag"]:,}  좌표 {s["fix"]:,}  '
                          f'({s["fix"] / max(el, 1):.0f}/s)')
                    print(f'          앵커모델 ' +
                          ' '.join(f'{n}:{len(tr.model[n])}' for n in SLAVES) +
                          f'   태그모델 {len(tr.tmodel)}'
                          f'   바퀴 ' +
                          ' '.join(str(tr.shift[n]) for n in SLAVES) +
                          f' / {tr.tshift}')
                    print(f'          짝실패 {s["no_pair"]:,}  '
                          f'앵커모델없음 {s["no_model"]:,}  '
                          f't1없음 {s["no_t1"]:,}  '
                          f'태그모델없음 {s["no_tmodel"]:,}  '
                          f'거리이상 {s["bad_d1"]:,}')
                if ax is not None and xs:
                    sc.set_data(list(xs), list(ys))
                    cur.set_data([xs[-1]], [ys[-1]])
                    ax.relim()
                    ax.autoscale_view()
                    import matplotlib.pyplot as plt
                    plt.pause(0.001)
                last = now
    except KeyboardInterrupt:
        print('\n종료')
    finally:
        stop.set()
        time.sleep(0.3)
        fcsv.close()
        print(f'저장: {csv_path}')
        if xs:
            X = np.array(xs)
            Y = np.array(ys)
            mx, my = np.median(X), np.median(Y)
            e = np.hypot(X - mx, Y - my)
            k = e < np.percentile(e, 95)
            print(f'최근 {len(X)}개  중앙값 ({mx:+.3f}, {my:+.3f}) m   '
                  f'산포 {e[k].mean() * 100:.1f} cm')


if __name__ == '__main__':
    main()