#!/usr/bin/env python3
"""
uwb_range.py — 방식 B 거리 측정 (B2/B3 검증용)

태그가 보고한 t1(자기 송신 시각), t4(동기 수신 시각)로 태그 시계를
마스터(RX1) 시계에 묶어 절대 거리를 낸다. 앵커 좌표가 필요 없다.

    offset_tag = t4 − T                     T = TXTS (마스터 송신 시각)
    t1'        = t1 − offset_tag(t1)        태그 송신 시각을 마스터 시계로
    ToF_i      = S_i' − t1'

    i=1 :  ToF_1 = 2·d₁ + k    →  d₁ = (ToF_1 − k) / 2      ← 왕복
    i≠1 :  d_i = ToF_i − d₁ + d_i1 − k

k(태그 송수신 안테나 지연 차)는 미지수다. 한 지점에서 역산하면 반드시
맞아떨어져 검증이 안 되므로, **두 지점에서 재야 한다.**

    ToF_1(A) = 2·d_A + k
    ToF_1(B) = 2·d_B + k     →  빼면 k 소거, 기울기가 2 면 원리 확인

사용법
    python3 uwb_range.py <log>                    # ToF 만 출력 (k 미지)
    python3 uwb_range.py <log> --true 1.32        # 참값 주면 k 역산
    python3 uwb_range.py <logA> --true 1.0 \
            --pair <logB> --true2 2.0             # 두 지점 → k 와 기울기 검증
"""

import argparse
import re
import sys
from collections import defaultdict, deque

import numpy as np

C = 299_792_458.0
U = 15.65e-12
WRAP = 2 ** 40
CNT_M = U * C

DEBUG = False
MASTER = 'RX1'
SYNC_MAC = 0x0001

# 태그 프레임 인덱스
I_SEQ, I_MAC, I_PSEQ = 2, 7, 9
I_T1, I_SSEQ, I_T4 = 10, 15, 16

WIN = 10                       # 시계 모델 창

LINE_RE = re.compile(
    r'\[(?P<rx>RX\d)\]\s+JS[0-9A-Fa-f]{4}'
    r'\{"LSTN":\[(?P<lstn>[^\]]+)\].*?'
    r'"TS40":"0x(?P<ts>[0-9A-Fa-f]+)"'
)
TXTS_RE = re.compile(
    r'\[(?P<rx>RX\d)\]\s+TXTS:\s+(?P<seq>[0-9A-Fa-f]{2})\s+0x(?P<ts>[0-9A-Fa-f]{10})'
)


def unwrapper():
    """40비트 unwrap. 부호 있는 차분이라 순서가 뒤바뀌어도 안전."""
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


def unwrap_seq(seqs):
    """8비트 SeqNum 열을 단조 인덱스로 편다.

    seq 는 256 마다 순환하므로 그대로 짝을 맞추면 다른 바퀴에 물린다.
    앵커 짝맞춤에서 이미 겪은 함정이므로 태그 쪽도 동일하게 처리한다.
    """
    out = []
    w = 0
    prev = None
    for x in seqs:
        if prev is not None:
            if x < prev - 128:
                w += 256
            elif x > prev + 128:
                w -= 256
        prev = x
        out.append(w + x)
    return out


def be5(a):
    """5바이트 MSB first → 정수."""
    v = 0
    for b in a:
        v = (v << 8) | b
    return v


def fit_predict(xs, ys, x):
    """직전 WIN개로 1차 회귀 후 x 에서 예측. 중심화로 소거오차 방지."""
    n = len(xs)
    if n < 3:
        return np.nan
    x0, y0 = xs[0], ys[0]
    a = np.fromiter((v - x0 for v in xs), float, n)
    b = np.fromiter((v - y0 for v in ys), float, n)
    sa = a.sum()
    d = n * (a @ a) - sa * sa
    if d == 0:
        return np.nan
    k = (n * (a @ b) - sa * b.sum()) / d
    return y0 + k * (x - x0) + (b.sum() - k * sa) / n


def parse(path):
    """로그에서 필요한 것을 모두 뽑는다. seq 는 전부 단조 인덱스로 편다.

    반환
      tx    : {g_tx: T}                        마스터 송신 시각
      blink : {g_blink: S1}                    RX1 의 태그 blink 수신 시각
      rep   : [(g_blink, t1, g_sync, t4)]      태그 보고 (도착 순)
    """
    tx_raw, rep_raw, bl_raw = [], [], []
    uw = defaultdict(unwrapper)
    # ⚠️ t1(송신)과 t4(수신)는 태그의 같은 카운터다.
    #    별개 unwrapper 를 쓰면 스케일이 2^40 배수만큼 어긋나
    #    searchsorted 가 엉뚱한 창을 고른다.
    #    마스터 TX/RX 에서 이미 겪은 것과 같은 버그.
    uw_tag = unwrapper()
    n_sync = n_tag = 0
    seen = set()

    with open(path, errors='ignore') as f:
        for line in f:
            m = TXTS_RE.search(line)
            if m:
                if m.group('rx') == MASTER:
                    tx_raw.append((uw[MASTER](int(m.group('ts'), 16)),
                                   int(m.group('seq'), 16)))
                continue

            m = LINE_RE.search(line)
            if not m:
                continue
            name = m.group('rx')
            try:
                a = [int(x, 16) for x in m.group('lstn').split(',')]
            except ValueError:
                continue

            ts = uw[name](int(m.group('ts'), 16))
            mac = a[I_MAC] | (a[I_MAC + 1] << 8)

            if mac == SYNC_MAC:
                n_sync += 1
                continue
            if len(a) < 21:
                continue
            n_tag += 1
            if name != MASTER:
                continue                       # 거리 계산엔 마스터만 필요

            bl_raw.append((ts, a[I_SEQ]))
            t1 = be5(a[I_T1:I_T1 + 5])
            t4 = be5(a[I_T4:I_T4 + 5])
            if t1 == 0 or t4 == 0:
                rep_raw.append(None)
                continue
            key = (a[I_PSEQ], t1)
            if key in seen:
                rep_raw.append(None)
                continue
            seen.add(key)
            # t1(직전 blink 송신) < t4(그 뒤 동기 수신) 순으로 단조
            v1 = uw_tag(t1)
            v4 = uw_tag(t4)
            rep_raw.append((v1, a[I_SSEQ], v4, a[I_PSEQ]))

    # seq → 단조 인덱스
    g_tx = unwrap_seq([s for _, s in tx_raw])
    tx = {g: t for g, (t, _) in zip(g_tx, tx_raw)}

    g_bl = unwrap_seq([s for _, s in bl_raw])
    blink = {g: t for g, (t, _) in zip(g_bl, bl_raw)}

    ss = [r[1] for r in rep_raw if r is not None]
    g_ss = unwrap_seq(ss)
    rep = []
    k = 0
    for g, r in zip(g_bl, rep_raw):
        if r is None:
            continue
        rep.append((g, r[0], g_ss[k], r[2], r[3]))
        k += 1

    print(f'파싱  TXTS {len(tx):,}   동기수신 {n_sync:,}   '
          f'태그수신 {n_tag:,}   태그보고 {len(rep):,}')
    if not rep:
        print('\n태그 보고 없음. 프레임 23바이트 / t1·t4 확인.')
    return tx, blink, rep


def build_tag_model(tx, rep):
    """태그 sync 인덱스와 마스터 TXTS 인덱스의 바퀴 차이를 맞춘다.

    두 인덱스는 같은 동기 프레임을 세므로 차이는 반드시 256의 배수다.

    선택 기준은 **드리프트가 물리적으로 가능한가**이다.
    크리스탈은 ±20 ppm 을 넘을 수 없으므로, 잘못된 바퀴를 고르면
    드리프트가 수백~수천 ppm 으로 튄다. 잔차만 보면 짝이 적을 때
    엉뚱한 후보가 뽑히므로 이 조건을 먼저 건다.

    반환: ([t4...], [offset...], (shift, ppm))
    """
    cands = []
    span = max(1, len(tx))
    for c in range(-(span // 256 + 4), span // 256 + 5):
        sh = c * 256
        pts = [(t4, t4 - tx[gs + sh]) for _, _, gs, t4, _ in rep if gs + sh in tx]
        if len(pts) < 30:
            continue
        x = np.array([p[0] for p in pts], float)
        y = np.array([p[1] for p in pts], float)
        b, a0 = np.polyfit(x, y, 1)
        ppm = b * 1e6
        res = float(np.std(y - (b * x + a0)))
        cands.append((abs(ppm) > 20, res, sh, ppm, pts, len(pts)))

    if not cands:
        return [], [], None

    # ① 드리프트가 가능한 범위인 것 우선 ② 짝이 많은 것 ③ 잔차 작은 것
    cands.sort(key=lambda z: (z[0], -z[5], z[1]))
    bad, res, sh, ppm, pts, n = cands[0]

    ok = [c for c in cands if not c[0]]
    if len(ok) > 1:
        print(f'  (드리프트 조건을 만족하는 후보 {len(ok)}개 — '
              f'가장 짝이 많은 것 선택)')

    pts.sort()

    # 불연속 구간 제거.
    # 태그 리셋·wrap 오판 등으로 offset 이 통째로 점프하면 그 뒤가
    # 전부 오염된다. 연속된 구간 중 가장 긴 것만 사용한다.
    o = np.array([p[1] for p in pts], float)
    d = np.diff(o)
    med = float(np.median(d))
    ok = np.abs(d - med) < 1e7            # 0.16 ms 이상 튀면 불연속
    seg, best, cur = [], (0, 0), 0
    for i, g in enumerate(ok):
        if not g:
            if i + 1 - cur > best[1] - best[0]:
                best = (cur, i + 1)
            cur = i + 1
    if len(o) - cur > best[1] - best[0]:
        best = (cur, len(o))
    a, b = best
    if b - a < len(pts):
        print(f'  불연속 {int((~ok).sum())}곳 → 최장 구간 {b - a}/{len(pts)} 만 사용')
    pts = pts[a:b]
    return [p[0] for p in pts], [p[1] for p in pts], (sh, ppm / 1e6)


def analyze(path, true_d=None, label=''):
    print('=' * 70)
    print(f'  {label or path}')
    print('=' * 70)
    tx, blink, rep = parse(path)
    if not rep or not tx:
        return None

    mt, mo, info = build_tag_model(tx, rep)
    if info is None:
        print('짝 부족 — sync_seq 와 TXTS 가 안 맞습니다.')
        return None
    sh, _ = info
    # 전체 회귀는 로그 중간의 불연속 하나에 망가진다.
    # 실제 계산과 같은 국소 창의 기울기 중앙값을 쓴다.
    sl = []
    for a in range(0, len(mt) - WIN, max(1, len(mt) // 200)):
        x = np.array(mt[a:a + WIN], float)
        y = np.array(mo[a:a + WIN], float)
        if x[-1] - x[0] > 0:
            sl.append(np.polyfit(x, y, 1)[0])
    ppm = float(np.median(sl)) * 1e6 if sl else float('nan')
    print(f'태그 시계 모델 짝  {len(mt):,}   바퀴보정 {sh:+d}')
    print(f'태그 클럭 드리프트  {ppm:+.4f} ppm  (국소 중앙값, ±20 이내면 정상)')
    if abs(ppm) > 50:
        print('  ⚠️ 물리적으로 불가능. 짝맞춤을 의심하세요.')

    # blink 주기 (진단용)
    gk = sorted(blink)
    per = ((blink[gk[-1]] - blink[gk[0]]) / (gk[-1] - gk[0])
           if len(gk) > 20 else None)

    tofs = []
    n_off = n_miss = 0
    dbg = []
    for gb, t1, gs_, t4, pseq in rep:
        # 태그가 보고한 pseq 로 어느 blink 인지 특정한다.
        # g % 256 == seq 이므로 gb 이하에서 pseq 와 맞는 g 를 찾는다.
        # "무조건 gb−1" 로 가정하면 태그가 송신을 건너뛴 회차에서 어긋난다.
        gp = gb - ((gb - pseq) % 256)
        if gp != gb - 1:
            n_off += 1
        S1 = blink.get(gp)
        if S1 is None:
            n_miss += 1
            continue
        if not (mt[0] <= t1 <= mt[-1]):
            continue                       # 모델 유효 구간 밖
        k = np.searchsorted(mt, t1)
        a = max(0, k - WIN)
        if k - a < 3:
            continue
        off = fit_predict(mt[a:k], mo[a:k], t1)
        if not np.isfinite(off):
            continue
        v = S1 - (t1 - off)
        tofs.append(v)
        if len(dbg) < 6:
            dbg.append((gb, S1, t1, t4, off, v))

    if DEBUG and dbg:
        print('\n  [디버그] 앞 6개 샘플')
        print(f'  {"gb":>6}{"S1(마스터)":>18}{"t1(태그)":>18}'
              f'{"t4(태그)":>18}{"off예측":>18}{"ToF":>16}')
        for g_, a_, b_, c_, d_, e_ in dbg:
            print(f'  {g_:>6}{a_:>18,.0f}{b_:>18,.0f}{c_:>18,.0f}'
                  f'{d_:>18,.0f}{e_:>16,.1f}')
        print(f'  모델 t4 범위 {mt[0]:,.0f} ~ {mt[-1]:,.0f}')
        print(f'  모델 off 범위 {min(mo):,.0f} ~ {max(mo):,.0f}')
        print(f'  blink S1 범위 {min(blink.values()):,.0f} ~ {max(blink.values()):,.0f}')

    if per:
        print(f'blink 주기 {per * U * 1e3:.2f} ms   '
              f'pseq≠gb−1 {n_off:,}   blink 못찾음 {n_miss:,}')

    if len(tofs) < 20:
        print(f'유효 샘플 부족 ({len(tofs)})')
        return None

    t = np.array(tofs)
    med = np.median(t)
    mad = 1.4826 * np.median(np.abs(t - med))
    keep = np.abs(t - med) < 3 * mad
    t = t[keep]

    print(f'\n유효 샘플 {len(t):,} / {len(tofs):,}  (이상치 3σ 제거)')
    print(f'ToF₁  중앙값 {med:>12,.1f} 카운트 = {med * CNT_M:>8.4f} m')
    print(f'      산포   {t.std():>12,.1f} 카운트 = {t.std() * CNT_M * 100:>8.2f} cm')
    print(f'      → d₁ = (ToF₁ − k)/2,  k 미지이므로 아직 절대거리 아님')

    if true_d is not None:
        k_est = med * CNT_M - 2 * true_d
        print(f'\n참값 {true_d:.4f} m 기준')
        print(f'  k = ToF₁ − 2·d = {k_est * 100:+.2f} cm '
              f'= {k_est / CNT_M:+.1f} 카운트 = {k_est / C * 1e9:+.3f} ns')
        print(f'  ⚠️ 한 지점 역산이므로 검증 아님. 두 지점 필요')
        print(f'  거리 산포 {t.std() * CNT_M / 2 * 100:.2f} cm  (왕복이라 ÷2)')

    return {'tof': med, 'std': t.std(), 'n': len(t), 'true': true_d}


def multi(specs):
    """여러 지점을 회귀로 처리한다. 두 점보다 자 측정 오차에 강하다."""
    rows = []
    for i, sp in enumerate(specs):
        path, _, d = sp.rpartition(':')
        r = analyze(path, float(d), f'지점 {chr(65 + i)}  (참값 {d} m)')
        if r:
            rows.append((float(d), r['tof'] * CNT_M, r['std'] * CNT_M))
        print()

    if len(rows) < 2:
        print('유효 지점 부족')
        return

    d = np.array([r[0] for r in rows])
    t = np.array([r[1] for r in rows])
    slope, icpt = np.polyfit(d, t, 1)

    print('=' * 70)
    print('  다지점 회귀 — ToF₁ = slope·d + k')
    print('=' * 70)
    print(f'\n{"참값 d":>10}{"ToF₁":>12}{"산포":>10}{"회귀 예측":>12}{"잔차":>10}')
    for dd, tt, ss in rows:
        pr = slope * dd + icpt
        print(f'{dd:>9.3f}m{tt:>11.4f}m{ss * 100:>9.2f}cm'
              f'{pr:>11.4f}m{(tt - pr) * 100:>9.2f}cm')

    print(f'\n기울기 {slope:.4f}   (이론값 2.0)')
    print(f'k      {icpt * 100:+.2f} cm = {icpt / C * 1e9:+.3f} ns')
    err = abs(slope - 2) / 2 * 100
    print(f'기울기 오차 {err:.1f} %')

    if err < 5:
        print('\n>>> 원리 확인. k 를 상수로 확정할 수 있습니다.')
        print(f'    d₁ = (ToF₁ − ({icpt:+.4f})) / 2')
    else:
        need = np.polyfit(t, d, 1)
        print('\n>>> 기울기가 2 에서 벗어납니다.')
        print('    기울기 2 를 가정하면 각 지점의 실제 거리는:')
        for dd, tt, _ in rows:
            print(f'      입력 {dd:.3f} m  →  추정 {(tt - icpt) / 2:.3f} m  '
                  f'(차이 {((tt - icpt) / 2 - dd) * 100:+.1f} cm)')
        print('    자 측정과 안테나 위상 중심을 확인하세요.')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('log', nargs='?')
    ap.add_argument('--true', type=float, default=None,
                    help='RX1↔태그 실측 거리 (m)')
    ap.add_argument('--pair', help='두 번째 지점 로그')
    ap.add_argument('--true2', type=float, default=None)
    ap.add_argument('--debug', action='store_true')
    ap.add_argument('--multi', nargs='+',
                    help='다지점 회귀. 예: A.txt:1.10 B.txt:2.00 C.txt:3.00')
    args = ap.parse_args()
    global DEBUG
    DEBUG = args.debug

    if args.multi:
        multi(args.multi)
        return
    if not args.log:
        ap.error('로그 경로 또는 --multi 필요')

    a = analyze(args.log, args.true, '지점 A')
    if not args.pair:
        return
    print()
    b = analyze(args.pair, args.true2, '지점 B')
    if not (a and b):
        return

    print('\n' + '=' * 70)
    print('  두 지점 비교 — k 를 소거한 검증')
    print('=' * 70)
    dtof = (b['tof'] - a['tof']) * CNT_M
    print(f'\nΔToF₁ = {dtof:+.4f} m')

    if a['true'] is not None and b['true'] is not None:
        dd = b['true'] - a['true']
        slope = dtof / dd if dd else float('nan')
        print(f'Δd(참값) = {dd:+.4f} m')
        print(f'\n기울기 = ΔToF₁ / Δd = {slope:.4f}')
        print(f'  이론값 2.0  (왕복이므로)')
        print(f'  오차 {abs(slope - 2) / 2 * 100:.1f} %')
        if abs(slope - 2) < 0.1:
            print('\n>>> 원리 확인. k 를 상수로 쓸 수 있습니다.')
            k1 = a['tof'] * CNT_M - 2 * a['true']
            k2 = b['tof'] * CNT_M - 2 * b['true']
            print(f'    k(A) {k1 * 100:+.2f} cm   k(B) {k2 * 100:+.2f} cm   '
                  f'차이 {abs(k1 - k2) * 100:.2f} cm')
            print(f'    → k = {(k1 + k2) / 2 * 100:+.2f} cm 사용 권장')
        else:
            print('\n>>> 기울기가 2 에서 벗어남. 다시 확인 필요.')
            print('    측정 거리, 시계 모델, 짝맞춤 순으로 점검')


if __name__ == '__main__':
    main()