# ============================================================
#  Curve-aware ACC  ·  RDDF analysis / tuning tool
#  RDDF 파일만 바꿔 실행하면 즉시 재계산 + 시각화
#  여기서 잡은 a_lat / preview 값을 그대로 LabVIEW SubVI에 넣으면 됨
#
#  사용법:
#    python curve_acc_tool.py                    # 아래 RDDF_PATH 사용
#    python curve_acc_tool.py path/to/rddf.txt   # 파일 지정
# ============================================================
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
import sys, os

# ─────────────────────────────────────────────
#  ① 여기만 바꾸면 됨 — RDDF 경로 + 파라미터
# ─────────────────────────────────────────────
RDDF_PATH = sys.argv[1] if len(sys.argv) > 1 else "충북대_Inline_UTM_only.txt"

PREVIEW = 8.0    # 앞을 보는 거리 [m]     <- LabVIEW preview_dist
A_LAT   = 2.5    # 허용 횡가속도 [m/s^2]  <- LabVIEW a_lat
V_MAX   = 20.0   # 속도 상한 [km/h]       <- LabVIEW v_max
DELTA   = 10     # 3점 간격 [칸]          <- LabVIEW delta
SHOW    = True   # True면 창 띄움(VSCode 인터랙티브). False면 PNG만 저장
# ─────────────────────────────────────────────


# 한글 폰트 자동 탐색 (없으면 영어 라벨로 폴백)
def setup_font():
    import matplotlib.font_manager as fm
    for name in ["Malgun Gothic", "AppleGothic", "NanumGothic",
                 "Noto Sans CJK KR", "Noto Sans KR"]:
        try:
            if any(name in f.name for f in fm.fontManager.ttflist):
                matplotlib.rcParams["font.family"] = name
                matplotlib.rcParams["axes.unicode_minus"] = False
                return True
        except Exception:
            pass
    return False

HANGUL = setup_font()
def L(ko, en):  # 폰트 있으면 한글, 없으면 영어
    return ko if HANGUL else en


def load_rddf(path):
    """RDDF 로드. 컬럼: idx, UTM_E, UTM_N, ... 앞 3개만 사용."""
    X, Y = [], []
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            p = line.split()
            if len(p) < 3:
                continue
            X.append(float(p[1]))
            Y.append(float(p[2]))
    return np.array(X), np.array(Y)


def menger_curvature(X, Y, c, delta):
    """LabVIEW SubVI와 동일한 3점 외접원(Menger) 곡률."""
    n = len(X)
    p1, p2 = (c - delta) % n, (c + delta) % n
    x1, y1 = X[p1], Y[p1]
    x2, y2 = X[c],  Y[c]
    x3, y3 = X[p2], Y[p2]
    a  = np.hypot(x2 - x1, y2 - y1)
    b  = np.hypot(x3 - x2, y3 - y2)
    cc = np.hypot(x3 - x1, y3 - y1)
    area = 0.5 * abs((x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1))
    denom = a * b * cc
    return 0.0 if denom < 1e-6 else 4 * area / denom


def v_curve(kappa, a_lat, v_max):
    if kappa < 1e-4:
        return v_max
    return min(np.sqrt(a_lat / kappa) * 3.6, v_max)


def analyze(X, Y):
    n = len(X)
    seg = np.hypot(np.diff(X), np.diff(Y))
    gap = float(np.median(seg))
    N = round(PREVIEW / gap)
    s = np.concatenate([[0], np.cumsum(seg)])
    kappa = np.zeros(n); vc = np.zeros(n)
    for wp in range(n):
        c = (wp + N) % n
        k = menger_curvature(X, Y, c, DELTA)
        kappa[wp] = k
        vc[wp] = v_curve(k, A_LAT, V_MAX)
    return s, kappa, vc, gap, N


def find_curves(s, vc, gap):
    n = len(vc); out = []; i = 0
    while i < n:
        if vc[i] < 19:
            j = i
            while j < n and vc[j] < 19:
                j += 1
            out.append((s[i], s[min(j, n - 1)], vc[i:j].min()))
            i = j
        else:
            i += 1
    return out


# ── 실행 ──
if not os.path.exists(RDDF_PATH):
    print(f"[!] 파일을 찾을 수 없음: {RDDF_PATH}")
    sys.exit(1)

X, Y = load_rddf(RDDF_PATH)
s, kappa, vc, gap, N = analyze(X, Y)
curves = find_curves(s, vc, gap)
R = np.where(kappa > 1e-4, 1.0 / kappa, np.inf)
name = os.path.basename(RDDF_PATH)

# ── 콘솔 요약 (한글 그대로) ──
print(f"\n{'='*54}")
print(f"  {name}")
print(f"{'='*54}")
print(f"  점 {len(X)}개 · 길이 {s[-1]:.0f}m · 간격 {gap:.3f}m · preview {N}칸({PREVIEW}m)")
print(f"  파라미터: a_lat={A_LAT} m/s²  v_max={V_MAX} km/h  delta={DELTA}")
print(f"  최대 κ={kappa.max():.3f}  최소 R={R[np.isfinite(R)].min():.1f}m  최저 v={vc.min():.1f} km/h")
print(f"\n  곡선 구간 {len(curves)}개 — 여기서 감속돼야 정상:")
print(f"  {'구간 s(m)':>13} {'최소R(m)':>9} {'κ':>7} {'v(km/h)':>9}")
for s0, s1, vmin in curves:
    m = (s >= s0) & (s <= s1)
    kmax = kappa[m].max()
    rr = 1 / kmax if kmax > 1e-4 else np.inf
    rng = f"{s0:.0f}" if s1 - s0 < gap * 2 else f"{s0:.0f}-{s1:.0f}"
    print(f"  {rng:>13} {rr:>9.1f} {kmax:>7.3f} {vmin:>9.1f}")
print(f"{'='*54}\n")

# ── 시각화 ──
plt.rcParams.update({
    "figure.facecolor": "#16181f", "axes.facecolor": "#1c1f28",
    "axes.edgecolor": "#3a3f4d", "axes.labelcolor": "#c8ccd8",
    "xtick.color": "#8a90a0", "ytick.color": "#8a90a0",
    "text.color": "#c8ccd8", "font.size": 10, "axes.grid": True,
    "grid.color": "#2a2e39", "grid.linewidth": 0.6, "axes.titlecolor": "#e8ead9",
})
fig = plt.figure(figsize=(13, 6.2))
gs = fig.add_gridspec(1, 2, width_ratios=[1.05, 1.0], wspace=0.2)

# 왼쪽: 트랙 곡률 맵
ax1 = fig.add_subplot(gs[0])
pts = np.array([X, Y]).T.reshape(-1, 1, 2)
segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
lc = LineCollection(segs, cmap="turbo", norm=plt.Normalize(0, 0.2))
lc.set_array(kappa[:-1]); lc.set_linewidth(3.5)
ax1.add_collection(lc)
ax1.plot(X[0], Y[0], "o", color="#c6f24e", ms=9, zorder=5, label="start")
ax1.set_aspect("equal"); ax1.autoscale()
ax1.set_title(L("트랙 곡률 맵", "Track curvature map"), fontsize=11, pad=10)
fig.suptitle(name, color="#8a90a0", fontsize=10, y=0.99)
ax1.set_xlabel("UTM E (m)"); ax1.set_ylabel("UTM N (m)")
cb = fig.colorbar(lc, ax=ax1, fraction=0.045, pad=0.03)
cb.set_label(L("곡률 κ (1/m)", "curvature κ (1/m)"), color="#c8ccd8")
ax1.legend(loc="best", facecolor="#1c1f28", edgecolor="#3a3f4d", labelcolor="#c8ccd8")

# 오른쪽: v_curve 프로파일
ax2 = fig.add_subplot(gs[1])
ax2.fill_between(s, vc, V_MAX, color="#c6f24e", alpha=0.10)
ax2.plot(s, vc, color="#c6f24e", lw=1.6, label="v_curve")
ax2.axhline(V_MAX, color="#5dcaa5", ls="--", lw=1, alpha=0.7, label=f"v_max={V_MAX:.0f}")
for s0, s1, _ in curves:
    ax2.axvspan(s0, s1, color="#f2a63c", alpha=0.10)
ax2.set_ylim(vc.min() - 2, V_MAX + 1.5); ax2.set_xlim(0, s[-1])
ax2.set_title(L("곡선 기반 목표속도", "Curve-based target speed")
              + f"  ·  a_lat={A_LAT}, preview={PREVIEW}m", fontsize=11, pad=10)
ax2.set_xlabel(L("경로 거리 s (m)", "path distance s (m)"))
ax2.set_ylabel("v_curve (km/h)")
ax2.legend(loc="lower right", facecolor="#1c1f28", edgecolor="#3a3f4d", labelcolor="#c8ccd8")

fig.subplots_adjust(left=0.08, right=0.96, top=0.88, bottom=0.11)
out = os.path.splitext(name)[0] + "_curve.png"
plt.savefig(out, dpi=110, facecolor="#16181f")
print(f"  그래프 저장: {out}")
if SHOW:
    plt.show()
