#!/usr/bin/env python3
"""
uwb_xy.py — 좌표 산출 (방식 B 최종)

    거리   TWR   d1 = (ToF1 - K) / 2
    방위   TDoA  r_i = d1 + tdoa_i + DELTA
    좌표         선형 닫힌 해 + Gauss-Newton

두 개의 미지 상수가 있다.
    K      태그의 송수신 안테나 지연 차   (왕복 측정에 들어감)
    DELTA  마스터의 송수신 안테나 지연 차 (세 TDoA 에 공통으로 더해짐)

--calib 로 참값을 아는 지점 여러 개를 주면 두 상수를 동시에 맞춘다.
그 뒤로는 --k / --delta 로 고정해 쓰면 된다.

사용법
    # 캘리브레이션 (참값 x,y 를 아는 지점 2개 이상)
    python3 uwb_xy.py --calib A2.txt:0.5,0 B2.txt:1.0,0 C2.txt:1.6,0

    # 좌표 산출
    python3 uwb_xy.py C2.txt --k -0.84 --delta -0.30
    python3 uwb_xy.py C2.txt --k -0.84 --delta -0.30 --true 1.6,0
"""

import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
exec(open(os.path.join(HERE, 'uwb_position.py'), encoding='utf-8')
     .read().split("def main()")[0])

NAMES = ['RX1', 'RX2', 'RX3', 'RX4']
A = np.array([ANCHORS[n] for n in NAMES], dtype=np.float64)
A2 = (A ** 2).sum(1)
M = 2.0 * (A[1:] - A[0])
MP = np.linalg.inv(M.T @ M) @ M.T

I_PSEQ_, I_T1_, I_SSEQ_, I_T4_ = 9, 10, 15, 16
WIN = 10


def useq(seqs):
    """8비트 seq 리스트 → 단조 인덱스 리스트.

    uwb_position.unwrap_seq 는 [(값, seq)] 를 받아 dict 를 돌려주므로
    인터페이스가 다르다. 혼동을 피하려 별도 이름으로 둔다.
    """
    out, w, prev = [], 0, None
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
    v = 0
    for b in a:
        v = (v << 8) | b
    return v


def fitp(xs, ys, x):
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


def extract(path):
    """로그에서 ToF1 중앙값과 TDoA 중앙값을 뽑는다."""
    import re
    from collections import defaultdict

    TXR = re.compile(r'\[(RX\d)\]\s+TXTS:\s+([0-9A-Fa-f]{2})\s+0x([0-9A-Fa-f]{10})')
    LSR = re.compile(r'\[(RX\d)\]\s+JS[0-9A-Fa-f]{4}\{"LSTN":\[([^\]]+)\].*?'
                     r'"TS40":"0x([0-9A-Fa-f]+)"')

    uw = defaultdict(unwrapper)
    uw_tag = unwrapper()
    txr, blr, rpr = [], [], []
    syn = defaultdict(list)
    seen = set()

    with open(path, errors='ignore') as f:
        for line in f:
            m = TXR.search(line)
            if m:
                if m.group(1) == MASTER:
                    txr.append((uw[MASTER](int(m.group(3), 16)),
                                int(m.group(2), 16)))
                continue
            m = LSR.search(line)
            if not m:
                continue
            name = m.group(1)
            try:
                a = [int(x, 16) for x in m.group(2).split(',')]
            except ValueError:
                continue
            ts = uw[name](int(m.group(3), 16))
            if (a[7] | (a[8] << 8)) == SYNC_MAC:
                if name != MASTER:
                    syn[name].append((ts, a[2]))
                continue
            if len(a) < 21:
                continue
            if name != MASTER:
                syn.setdefault('_tag_' + name, []).append((ts, a[2]))
                continue
            blr.append((ts, a[2]))
            t1 = be5(a[I_T1_:I_T1_ + 5])
            t4 = be5(a[I_T4_:I_T4_ + 5])
            if t1 == 0 or t4 == 0 or (a[I_PSEQ_], t1) in seen:
                rpr.append(None)
                continue
            seen.add((a[I_PSEQ_], t1))
            rpr.append((uw_tag(t1), a[I_SSEQ_], uw_tag(t4), a[I_PSEQ_]))

    if not txr or not blr:
        return None

    g_tx = useq([s for _, s in txr])
    tx = {g: t for g, (t, _) in zip(g_tx, txr)}
    g_bl = useq([s for _, s in blr])
    blink = {g: t for g, (t, _) in zip(g_bl, blr)}

    ss = [r[1] for r in rpr if r]
    if not ss:
        return None
    g_ss = useq(ss)
    rep, k = [], 0
    for g, r in zip(g_bl, rpr):
        if r is None:
            continue
        rep.append((g, r[0], g_ss[k], r[2], r[3]))
        k += 1

    # --- 태그 시계 모델 (256 배수 정렬 + 불연속 최장구간)
    best = None
    span = max(1, len(tx))
    for c in range(-(span // 256 + 4), span // 256 + 5):
        sh = c * 256
        pts = [(t4, t4 - tx[gs + sh]) for _, _, gs, t4, _ in rep if gs + sh in tx]
        if len(pts) < 30:
            continue
        x = np.array([p[0] for p in pts], float)
        y = np.array([p[1] for p in pts], float)
        b = np.polyfit(x, y, 1)[0]
        cands = (abs(b * 1e6) > 20, -len(pts))
        if best is None or cands < best[0]:
            best = (cands, pts)
    if best is None:
        return None
    pts = sorted(best[1])
    o = np.array([p[1] for p in pts], float)
    d = np.diff(o)
    ok = np.abs(d - np.median(d)) < 1e7
    bs, cur = (0, 0), 0
    for i, g in enumerate(ok):
        if not g:
            if i + 1 - cur > bs[1] - bs[0]:
                bs = (cur, i + 1)
            cur = i + 1
    if len(o) - cur > bs[1] - bs[0]:
        bs = (cur, len(o))
    pts = pts[bs[0]:bs[1]]
    mt = [p[0] for p in pts]
    mo = [p[1] for p in pts]

    # --- ToF1
    tofs = []
    for gb, t1, _, t4, pseq in rep:
        gp = gb - ((gb - pseq) % 256)
        S1 = blink.get(gp)
        if S1 is None or not (mt[0] <= t1 <= mt[-1]):
            continue
        kk = np.searchsorted(mt, t1)
        a0 = max(0, kk - WIN)
        if kk - a0 < 3:
            continue
        off = fitp(mt[a0:kk], mo[a0:kk], t1)
        if np.isfinite(off):
            tofs.append(S1 - (t1 - off))
    if len(tofs) < 20:
        return None
    t = np.array(tofs)
    med = np.median(t)
    mad = 1.4826 * np.median(np.abs(t - med))
    t = t[np.abs(t - med) < 3 * mad]
    tof1 = float(np.median(t)) * CNT_M

    # --- TDoA (uwb_position 재사용)
    tx2, sync2, tags2 = parse(path)
    models, _ = build_models(tx2, sync2, WIN)
    # 비트 오류로 MAC 이 깨진 프레임이 섞일 수 있다.
    # 가장 많이 잡힌 태그를 고른다.
    mac = max(tags2, key=lambda m: sum(len(v) for v in tags2[m].values()))
    tag = tags2[mac]
    G = {n: unwrap_seq(tag[n]) for n in NAMES if n in tag}
    g0 = G[MASTER]
    tdoa = []
    for n in NAMES[1:]:
        sh = max(range(-8, 9),
                 key=lambda c: sum(1 for g in g0 if g + c * 256 in G[n])) * 256
        v = []
        for g, r1 in g0.items():
            ri = G[n].get(g + sh)
            if ri is None:
                continue
            off = models[n].predict(ri)
            if np.isfinite(off):
                v.append(((ri - off) - r1
                          + tof_cnt(ANCHORS[MASTER], ANCHORS[n])) * CNT_M)
        if len(v) < 30:
            return None
        tdoa.append(float(np.median(v)))

    return {'tof1': tof1, 'tdoa': np.array(tdoa), 'n': len(t)}


def solve(tof1, tdoa, K, DELTA, gn=8):
    d1 = (tof1 - K) / 2.0
    r = np.concatenate([[d1], d1 + tdoa + DELTA])
    b = A2[1:] - A2[0] - r[1:] ** 2 + r[0] ** 2
    p = MP @ b
    for _ in range(gn):                       # Gauss-Newton 다듬기
        v = p - A
        dd = np.linalg.norm(v, axis=1)
        J = v / dd[:, None]
        try:
            p = p - np.linalg.lstsq(J, dd - r, rcond=None)[0]
        except np.linalg.LinAlgError:
            break
    return p, d1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('log', nargs='?')
    ap.add_argument('--k', type=float, default=None)
    ap.add_argument('--delta', type=float, default=None)
    ap.add_argument('--true', help='참값 "x,y"')
    ap.add_argument('--calib', nargs='+', help='log:x,y 형식 2개 이상')
    args = ap.parse_args()

    print(f'앵커  ' + '  '.join(f'{n}{ANCHORS[n]}' for n in NAMES))

    if args.calib:
        data = []
        for sp in args.calib:
            path, _, xy = sp.rpartition(':')
            t = np.array([float(v) for v in xy.split(',')])
            print(f'\n--- {os.path.basename(path)}  참값 ({t[0]}, {t[1]}) ---')
            e = extract(path)
            if not e:
                print('  추출 실패')
                continue
            print(f'  ToF1 {e["tof1"]:.4f} m   TDoA ' +
                  ' '.join(f'{v:+.4f}' for v in e['tdoa']))
            data.append((e, t))
        if len(data) < 2:
            print('\n캘리브레이션에는 지점 2개 이상 필요')
            sys.exit(1)

        def cost(K, D):
            s = 0.0
            for e, t in data:
                p, _ = solve(e['tof1'], e['tdoa'], K, D)
                s += float(((p - t) ** 2).sum())
            return s

        bK, bD, bc = 0, 0, None
        for K in np.arange(-1.6, 0.61, 0.02):
            for D in np.arange(-1.0, 0.61, 0.02):
                c = cost(K, D)
                if bc is None or c < bc:
                    bK, bD, bc = K, D, c
        for _ in range(4):                     # 세밀화
            st = 0.004
            for K in np.arange(bK - 0.02, bK + 0.021, st):
                for D in np.arange(bD - 0.02, bD + 0.021, st):
                    c = cost(K, D)
                    if c < bc:
                        bK, bD, bc = K, D, c

        print('\n' + '=' * 68)
        print('  캘리브레이션 결과')
        print('=' * 68)
        print(f'\n  K     = {bK:+.4f} m  ({bK * 100:+.1f} cm)   태그 안테나 지연')
        print(f'  DELTA = {bD:+.4f} m  ({bD * 100:+.1f} cm)   마스터 안테나 지연')
        print(f'\n{"로그":>10}{"참값":>16}{"추정":>18}{"오차":>10}{"d1":>10}')
        for e, t in data:
            p, d1 = solve(e['tof1'], e['tdoa'], bK, bD)
            print(f'{"":>10}({t[0]:+.2f},{t[1]:+.2f}){"":>3}'
                  f'({p[0]:+.3f},{p[1]:+.3f})'
                  f'{np.linalg.norm(p - t) * 100:>9.1f}cm{d1:>9.3f}m')
        print(f'\n  다음부터:  --k {bK:.3f} --delta {bD:.3f}')
        return

    if not args.log or args.k is None or args.delta is None:
        ap.error('log 과 --k, --delta 필요 (또는 --calib)')

    e = extract(args.log)
    if not e:
        print('추출 실패')
        sys.exit(1)
    p, d1 = solve(e['tof1'], e['tdoa'], args.k, args.delta)
    print(f'\nToF1 {e["tof1"]:.4f} m  →  d1 = {d1:.4f} m')
    print('TDoA ' + '  '.join(f'{n} {v:+.4f}'
                              for n, v in zip(NAMES[1:], e['tdoa'])))
    print(f'\n좌표  ({p[0]:+.4f}, {p[1]:+.4f}) m')
    print(f'      RX1 로부터 {np.linalg.norm(p):.3f} m, '
          f'방위 {np.degrees(np.arctan2(p[1], p[0])):+.1f}°')
    if args.true:
        t = np.array([float(v) for v in args.true.split(',')])
        print(f'참값  ({t[0]:+.4f}, {t[1]:+.4f}) m   '
              f'오차 {np.linalg.norm(p - t) * 100:.1f} cm')


if __name__ == '__main__':
    main()