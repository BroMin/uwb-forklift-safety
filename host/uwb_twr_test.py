#!/usr/bin/env python3

"""
uwb_twr_test.py — RX1↔태그 거리만 보는 실험

TDoA·좌표를 빼고 거리 하나만 본다.

패킷마다의 거리 분포와, 참값 대비 오차를 확인한다.

    ToF1 = 2·d + K

    d = (ToF1 - K) / 2

K 는 태그 안테나 지연으로 미지수다. 여러 지점을 재면

    ① 기울기 자유
       ToF1 = a·d + K
       → a 가 2 에 가까운지

    ② 기울기 2 고정
       K = mean(ToF1 - 2·d)

두 가지로 맞춰 비교한다.

사용법

    python3 uwb_twr_test.py A.txt:0.5 B.txt:1.0 C.txt:1.6

    python3 uwb_twr_test.py A.txt:0.5 B.txt:1.0 C.txt:1.6 --plot

    python3 uwb_twr_test.py C.txt --k -0.84
        # 참값 없이 거리만
"""

import argparse
import os
import re
import sys
from collections import defaultdict

import numpy as np


# ------------------------------------------------------------
# uwb_position.py 에서 필요한 상수 / 함수 가져오기
# ------------------------------------------------------------

HERE = os.path.dirname(os.path.abspath(__file__))

exec(
    open(
        os.path.join(HERE, "uwb_position.py"),
        encoding="utf-8"
    ).read().split("def main()")[0]
)


# ------------------------------------------------------------
# 패킷 내부 필드 위치
# ------------------------------------------------------------

I_PSEQ_, I_T1_, I_SSEQ_, I_T4_ = 9, 10, 15, 16

WIN = 10


# ------------------------------------------------------------
# 로그 정규식
#
# 예:
# [RX1] TXTS: 17 0x60F6725A15
#
# [RX1] JS....{"LSTN":[...],"TS40":"0x..."}
# ------------------------------------------------------------

TXR = re.compile(
    r"\[(RX\d)\]\s+TXTS:\s+"
    r"([0-9A-Fa-f]{2})\s+"
    r"0x([0-9A-Fa-f]{10})"
)

LSR = re.compile(
    r"\[(RX\d)\]\s+"
    r"JS[0-9A-Fa-f]{4}\{"
    r'"LSTN":\[([^\]]+)\]'
    r".*?"
    r'"TS40":"0x([0-9A-Fa-f]+)"'
)


# ------------------------------------------------------------
# 8-bit sequence number unwrap
# ------------------------------------------------------------

def useq(seqs):
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


# ------------------------------------------------------------
# 5-byte big endian
# ------------------------------------------------------------

def be5(a):
    v = 0

    for b in a:
        v = (v << 8) | b

    return v


# ------------------------------------------------------------
# 선형 보간
# ------------------------------------------------------------

def fitp(xs, ys, x):

    n = len(xs)

    if n < 3:
        return np.nan

    x0, y0 = xs[0], ys[0]

    a = np.fromiter(
        (v - x0 for v in xs),
        float,
        n
    )

    b = np.fromiter(
        (v - y0 for v in ys),
        float,
        n
    )

    sa = a.sum()

    d = n * (a @ a) - sa * sa

    if d == 0:
        return np.nan

    k = (
        n * (a @ b)
        - sa * b.sum()
    ) / d

    return (
        y0
        + k * (x - x0)
        + (b.sum() - k * sa) / n
    )


# ------------------------------------------------------------
# 로그에서 ToF1 시계열 추출
# ------------------------------------------------------------

def tof_series(path):
    """
    패킷마다의 ToF1 (m) 배열을 돌려준다.

    반환:
        t       : ToF1 배열 (m)
        w       : 시간 배열 (s)
        n_drop  : 시계 불연속으로 제거된 개수
        n_rep   : TWR 보고 패킷 수
    """

    # RX별 local timestamp unwrap
    uw = defaultdict(unwrapper)

    # 태그 timestamp unwrap
    uw_tag = unwrapper()

    txr = []
    blr = []
    rpr = []

    seen = set()

    # --------------------------------------------------------
    # 로그 읽기
    # --------------------------------------------------------

    try:
        f = open(path, errors="ignore")

    except OSError as e:
        print(f"파일 열기 실패: {path}")
        print(f"  {e}")
        return None

    with f:

        for line in f:

            # =================================================
            # TXTS
            # =================================================

            m = TXR.search(line)

            if m:

                if m.group(1) == MASTER:

                    txr.append(
                        (
                            uw[MASTER](
                                int(m.group(3), 16)
                            ),
                            int(m.group(2), 16)
                        )
                    )

                continue

            # =================================================
            # LSTN
            # =================================================

            m = LSR.search(line)

            if not m:
                continue

            name = m.group(1)

            # -------------------------------------------------
            # LSTN byte 배열
            # -------------------------------------------------

            try:

                a = [
                    int(x.strip(), 16)
                    for x in m.group(2).split(",")
                ]

            except ValueError:
                continue

            # -------------------------------------------------
            # RX timestamp
            # -------------------------------------------------

            ts = uw[name](
                int(m.group(3), 16)
            )

            if len(a) < 11:
                continue

            # -------------------------------------------------
            # SYNC packet 제거
            # MASTER가 아닌 RX도 제거
            # -------------------------------------------------

            if (
                (a[7] | (a[8] << 8)) == SYNC_MAC
                or name != MASTER
            ):
                continue

            # -------------------------------------------------
            # TWR 관련 필드가 없는 패킷 제거
            # -------------------------------------------------

            if len(a) < 21:
                continue

            # -------------------------------------------------
            # Blink sequence
            # -------------------------------------------------

            blr.append(
                (
                    ts,
                    a[2]
                )
            )

            # -------------------------------------------------
            # T1 / T4
            # -------------------------------------------------

            t1 = be5(
                a[I_T1_:I_T1_ + 5]
            )

            t4 = be5(
                a[I_T4_:I_T4_ + 5]
            )

            # -------------------------------------------------
            # 잘못된 TWR / 중복 패킷 제거
            # -------------------------------------------------

            if (
                t1 == 0
                or t4 == 0
                or (a[I_PSEQ_], t1) in seen
            ):

                rpr.append(None)

                continue

            seen.add(
                (
                    a[I_PSEQ_],
                    t1
                )
            )

            rpr.append(
                (
                    uw_tag(t1),
                    a[I_SSEQ_],
                    uw_tag(t4),
                    a[I_PSEQ_]
                )
            )

    # --------------------------------------------------------
    # 기본 데이터 확인
    # --------------------------------------------------------

    if not txr or not blr:
        return None

    # ========================================================
    # MASTER TX sequence unwrap
    # ========================================================

    g_tx = useq(
        [
            s
            for _, s in txr
        ]
    )

    tx = {
        g: t
        for g, (t, _) in zip(g_tx, txr)
    }

    # ========================================================
    # BLINK sequence unwrap
    # ========================================================

    g_bl = useq(
        [
            s
            for _, s in blr
        ]
    )

    blink = {
        g: t
        for g, (t, _) in zip(g_bl, blr)
    }

    # ========================================================
    # Tag SSEQ unwrap
    # ========================================================

    ss = [
        r[1]
        for r in rpr
        if r is not None
    ]

    if not ss:
        return None

    g_ss = useq(ss)

    # --------------------------------------------------------
    # 각 TWR report 구성
    # --------------------------------------------------------

    rep = []
    k = 0

    for g, r in zip(g_bl, rpr):

        if r is None:
            continue

        rep.append(
            (
                g,
                r[0],
                g_ss[k],
                r[2],
                r[3]
            )
        )

        k += 1

    # ========================================================
    # 태그 시계 모델
    #
    # T4 - Master TX
    # 를 이용하여 태그 시계 offset을 추정
    # ========================================================

    best = None

    span = max(
        1,
        len(tx)
    )

    for c in range(
        -(span // 256 + 4),
        span // 256 + 5
    ):

        sh = c * 256

        pts = [
            (
                t4,
                t4 - tx[gs + sh]
            )
            for _, _, gs, t4, _ in rep
            if gs + sh in tx
        ]

        if len(pts) < 30:
            continue

        x = np.array(
            [p[0] for p in pts],
            float
        )

        y = np.array(
            [p[1] for p in pts],
            float
        )

        # 태그 clock drift
        b = np.polyfit(
            x,
            y,
            1
        )[0]

        key = (
            abs(b * 1e6) > 20,
            -len(pts)
        )

        if (
            best is None
            or key < best[0]
        ):
            best = (
                key,
                pts
            )

    if best is None:
        return None

    # ========================================================
    # 가장 연속적인 구간 선택
    # ========================================================

    pts = sorted(
        best[1]
    )

    o = np.array(
        [p[1] for p in pts],
        float
    )

    d = np.diff(o)

    ok = (
        np.abs(
            d - np.median(d)
        ) < 1e7
    )

    bs = (0, 0)
    cur = 0

    for i, g in enumerate(ok):

        if not g:

            if (
                i + 1 - cur
                > bs[1] - bs[0]
            ):
                bs = (
                    cur,
                    i + 1
                )

            cur = i + 1

    if (
        len(o) - cur
        > bs[1] - bs[0]
    ):
        bs = (
            cur,
            len(o)
        )

    n_drop = (
        len(pts)
        - (bs[1] - bs[0])
    )

    pts = pts[
        bs[0]:bs[1]
    ]

    if not pts:
        return None

    mt = [
        p[0]
        for p in pts
    ]

    mo = [
        p[1]
        for p in pts
    ]

    # ========================================================
    # ToF 계산
    # ========================================================

    tofs = []
    when = []

    for (
        gb,
        t1,
        _,
        t4,
        pseq
    ) in rep:

        # BLINK sequence와 PSEQ를 맞춘다.
        gp = (
            gb
            - ((gb - pseq) % 256)
        )

        S1 = blink.get(gp)

        if S1 is None:
            continue

        if not (
            mt[0]
            <= t1
            <= mt[-1]
        ):
            continue

        # t1 이전의 최근 WIN개를 이용
        kk = np.searchsorted(
            mt,
            t1
        )

        a0 = max(
            0,
            kk - WIN
        )

        if kk - a0 < 3:
            continue

        # 태그 clock offset
        off = fitp(
            mt[a0:kk],
            mo[a0:kk],
            t1
        )

        if np.isfinite(off):

            # -----------------------------------------------
            # ToF1
            # -----------------------------------------------
            tof = (
                S1
                - (t1 - off)
            ) * CNT_M

            tofs.append(tof)

            # 마스터 시계 기준 시간
            when.append(S1)

    # --------------------------------------------------------
    # 최소 샘플 수
    # --------------------------------------------------------

    if len(tofs) < 20:
        return None

    t = np.array(
        tofs,
        float
    )

    w = (
        np.array(
            when,
            float
        )
        - min(when)
    ) * U

    # ========================================================
    # MAD 기반 outlier 제거
    # ========================================================

    med = np.median(t)

    mad = (
        1.4826
        * np.median(
            np.abs(
                t - med
            )
        )
    )

    if mad > 0:

        keep = (
            np.abs(
                t - med
            )
            < 4 * mad
        )

    else:

        # 모든 값이 동일한 경우
        keep = np.ones(
            len(t),
            dtype=bool
        )

    return (
        t[keep],
        w[keep],
        n_drop,
        len(rep)
    )


# ------------------------------------------------------------
# 텍스트 히스토그램
# ------------------------------------------------------------

def hist(v, lo, hi, w=52):

    e = np.linspace(
        lo,
        hi,
        21
    )

    c, _ = np.histogram(
        v,
        e
    )

    mx = (
        c.max()
        if c.max()
        else 1
    )

    for i in range(20):

        bar = "#" * int(
            c[i] / mx * w
        )

        print(
            f"  {e[i]:+7.3f} ~ "
            f"{e[i+1]:+7.3f} m"
            f"  {c[i]:>5}  "
            f"{bar}"
        )


# ------------------------------------------------------------
# main
# ------------------------------------------------------------

def main():

    ap = argparse.ArgumentParser()

    ap.add_argument(
        "specs",
        nargs="+",
        help="log 또는 log:참값거리"
    )

    ap.add_argument(
        "--k",
        type=float,
        default=None
    )

    ap.add_argument(
        "--plot",
        action="store_true"
    )

    args = ap.parse_args()

    rows = []

    # ========================================================
    # 각 로그 분석
    # ========================================================

    for sp in args.specs:

        # ----------------------------------------------------
        # 파일명:참값거리
        # ----------------------------------------------------

        if ":" in sp:

            path, _, d = sp.rpartition(":")

            try:
                true = float(d)

            except ValueError:

                print(
                    f"잘못된 참값 거리: {sp}"
                )

                continue

        else:

            path = sp
            true = None

        # ----------------------------------------------------
        # ToF 추출
        # ----------------------------------------------------

        r = tof_series(path)

        name = os.path.basename(
            path
        )

        print(
            "=" * 68
        )

        print(
            f"  {name}"
            + (
                f"   참값 {true:.3f} m"
                if true is not None
                else ""
            )
        )

        print(
            "=" * 68
        )

        if r is None:

            print(
                "  추출 실패\n"
            )

            continue

        t, w, n_drop, n_rep = r

        print(
            f"  유효 {len(t):,} / "
            f"보고 {n_rep:,}"
            f"   불연속 폐기 {n_drop:,}"
        )

        print(
            f"  ToF1  중앙 "
            f"{np.median(t):.4f} m"
            f"   평균 {t.mean():.4f}"
            f"   산포 "
            f"{t.std() * 100:.2f} cm"
        )

        # ====================================================
        # --k가 있으면 거리로 변환
        # ====================================================

        if args.k is not None:

            d1 = (
                t - args.k
            ) / 2

            print(
                f"  거리  중앙 "
                f"{np.median(d1):.4f} m"
                f"   산포 "
                f"{d1.std() * 100:.2f} cm"
            )

            if true is not None:

                print(
                    f"        참값 "
                    f"{true:.3f} m"
                    f"   편향 "
                    f"{(np.median(d1) - true) * 100:+.1f} cm"
                )

            print()

            hist(
                d1,
                np.median(d1) - 0.25,
                np.median(d1) + 0.25
            )

        else:

            print()

            hist(
                t,
                np.median(t) - 0.5,
                np.median(t) + 0.5
            )

        print()

        rows.append(
            (
                name,
                true,
                t,
                w
            )
        )

    # --------------------------------------------------------
    # 데이터가 없으면 종료
    # --------------------------------------------------------

    if not rows:

        print(
            "분석할 로그가 없습니다."
        )

        return

    # ========================================================
    # 그래프용 K
    #
    # 참값이 2개 이상:
    #
    #     K = mean(ToF1 - 2d)
    #
    # 그 외:
    #     --k
    #     또는 0
    # ========================================================

    have = [
        r
        for r in rows
        if r[1] is not None
    ]

    if len(have) >= 2:

        _d = np.array(
            [
                r[1]
                for r in have
            ]
        )

        _m = np.array(
            [
                np.median(r[2])
                for r in have
            ]
        )

        k2 = float(
            np.mean(
                _m - 2 * _d
            )
        )

    else:

        k2 = (
            args.k
            if args.k is not None
            else 0.0
        )

    # ========================================================
    # 그래프
    # ========================================================

    if args.plot:

        try:

            import matplotlib

            matplotlib.use(
                "Agg"
            )

            import matplotlib.pyplot as plt

        except ImportError:

            print(
                "\nmatplotlib 없음 "
                "(sudo apt install "
                "python3-matplotlib)"
            )

            return

        n = len(rows)

        fig, axes = plt.subplots(
            n,
            1,
            figsize=(
                11,
                3.0 * n
            ),
            squeeze=False
        )

        for ax, (
            name,
            true,
            t,
            w
        ) in zip(
            axes[:, 0],
            rows
        ):

            # ----------------------------------------------
            # ToF1 → 거리
            # ----------------------------------------------

            d1 = (
                t - k2
            ) / 2

            ax.plot(
                w,
                d1,
                ".",
                ms=3,
                alpha=0.5,
                color="tab:blue"
            )

            # ----------------------------------------------
            # 측정 중앙값
            # ----------------------------------------------

            ax.axhline(
                np.median(d1),
                color="k",
                lw=1,
                label=(
                    f"median "
                    f"{np.median(d1):.3f} m"
                )
            )

            # ----------------------------------------------
            # 참값
            # ----------------------------------------------

            if true is not None:

                ax.axhline(
                    true,
                    color="r",
                    ls="--",
                    lw=1,
                    label=(
                        f"true "
                        f"{true:.2f} m"
                    )
                )

            ax.set_ylabel(
                "distance (m)"
            )

            ax.set_title(
                f"{name}   "
                f"n={len(t)}  "
                f"sigma="
                f"{d1.std() * 100:.1f} cm",
                fontsize=10
            )

            ax.grid(
                alpha=0.3
            )

            ax.legend(
                fontsize=8,
                loc="upper right"
            )

        axes[-1, 0].set_xlabel(
            "time (s)"
        )

        fig.tight_layout()

        fig.savefig(
            "twr_time.png",
            dpi=130
        )

        print(
            "\n그래프 저장: "
            "twr_time.png"
            f"   (K = {k2:.3f} m 적용)"
        )

    # ========================================================
    # 참값 2개 미만이면 calibration 분석 종료
    # ========================================================

    if len(have) < 2:
        return

    # ========================================================
    # 참값별 중앙 ToF / 산포
    # ========================================================

    d = np.array(
        [
            r[1]
            for r in have
        ]
    )

    m = np.array(
        [
            np.median(r[2])
            for r in have
        ]
    )

    s = np.array(
        [
            r[2].std()
            for r in have
        ]
    )

    # ========================================================
    # ① 기울기 자유
    #
    #     ToF1 = a*d + b
    # ========================================================

    a, b = np.polyfit(
        d,
        m,
        1
    )

    # ========================================================
    # ② 기울기 2 고정
    #
    #     K = mean(ToF1 - 2d)
    # ========================================================

    k2 = float(
        np.mean(
            m - 2 * d
        )
    )

    print(
        "=" * 68
    )

    print(
        "  거리 대 ToF1"
    )

    print(
        "=" * 68
    )

    print(
        f'\n{"참값":>8}'
        f'{"ToF1":>10}'
        f'{"산포":>9}'
        f'{"자유맞춤 오차":>15}'
        f'{"기울기2 고정 오차":>19}'
    )

    for dd, mm, ss in zip(
        d,
        m,
        s
    ):

        e1 = (
            mm
            - (a * dd + b)
        )

        e2 = (
            mm
            - (2 * dd + k2)
        )

        print(
            f"{dd:>7.3f}m"
            f"{mm:>9.4f}m"
            f"{ss * 100:>8.2f}cm"
            f"{e1 * 100:>13.2f}cm"
            f"{e2 * 100:>17.2f}cm"
        )

    # ========================================================
    # ① 결과
    # ========================================================

    print(
        f"\n① 기울기 자유"
        f"   ToF1 = "
        f"{a:.4f}·d "
        f"{b:+.4f}"
    )

    print(
        f"   기울기 {a:.4f}"
        f"  (이론 2.0, "
        f"오차 "
        f"{abs(a - 2) / 2 * 100:.1f}%)"
    )

    print(
        f"   → K = "
        f"{b * 100:+.1f} cm"
    )

    # ========================================================
    # ② 결과
    # ========================================================

    print(
        f"\n② 기울기 2 고정"
        f"  K = {k2 * 100:+.1f} cm"
    )

    print(
        "   이때 각 지점의 추정 거리"
    )

    for dd, mm in zip(
        d,
        m
    ):

        est = (
            mm - k2
        ) / 2

        print(
            f"     참값 {dd:.3f} m"
            f"  →  추정 {est:.3f} m"
            f"  ({(est - dd) * 100:+.1f} cm)"
        )

    # ========================================================
    # ③ 이동량 검증
    #
    # K와 무관
    # ========================================================

    print(
        "\n③ 이동량 검증  (K 와 무관)"
    )

    for i in range(
        len(d) - 1
    ):

        move_true = (
            d[i + 1]
            - d[i]
        )

        move_meas = (
            m[i + 1]
            - m[i]
        ) / 2

        print(
            f"     기록 "
            f"{move_true:.3f} m"
            f"  →  실측 "
            f"{move_meas:.3f} m"
            f"  ({(move_meas - move_true) * 100:+.1f} cm)"
        )

    print(
        "   이동량이 기록보다 "
        "일관되게 크면 자 측정 기준점 "
        "문제입니다."
    )


if __name__ == "__main__":
    main()