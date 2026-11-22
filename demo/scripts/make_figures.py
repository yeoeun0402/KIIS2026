"""REPORT.md 와 함께 볼 그림 — demo/figures/*.png. 데이터는 전부 demo/out 의 시험 결과(T1·T2·T4·T5·T6·T7)."""
import common  # noqa: F401
import csv
import json
import os

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Wedge, Rectangle
import numpy as np

FIG = os.path.join(common.DEMO, "figures"); os.makedirs(FIG, exist_ok=True)
OUT = common.OUT

# ── 토큰 (dataviz 기본 팔레트, 라이트) ──────────────────────────────
SURF, INK, INK2, MUTED, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#8a8984", "#e7e6e2"
BLUE, ORANGE, AQUA = "#2a78d6", "#eb6834", "#1baf7a"     # 1 수정본·현재 / 2 원본·예전 / 3 upstream
CRIT, GOOD = "#d03b3b", "#0ca30c"
# 시스템 Noto CJK 는 TTC 라 matplotlib 3.5 가 한글을 □ 로 그린다 → KR 면만 뽑은 OTF(work/fonts)를 등록
from matplotlib import font_manager as _fm
for _w in ("Regular", "Bold"):
    _fm.fontManager.addfont(os.path.join(common.WORK, "fonts", f"NotoSansCJKkr-{_w}.otf"))
plt.rcParams.update({
    "font.family": "Noto Sans CJK KR","font.size": 10, "axes.unicode_minus": False,
    "figure.facecolor": SURF, "axes.facecolor": SURF, "savefig.facecolor": SURF,
    "axes.edgecolor": MUTED, "axes.labelcolor": INK2, "xtick.color": INK2, "ytick.color": INK2,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.8, "axes.axisbelow": True,
    "axes.spines.top": False, "axes.spines.right": False, "axes.titlesize": 12, "axes.titleweight": "bold",
    "axes.titlecolor": INK, "axes.titlelocation": "left", "legend.frameon": False, "lines.linewidth": 2,
})


def save(fig, name):
    p = os.path.join(FIG, name)
    fig.savefig(p, dpi=160, bbox_inches="tight")
    plt.close(fig)
    print("saved", p)


def note(fig, text):
    fig.text(0.01, -0.02, text, color=MUTED, fontsize=8.5, ha="left", va="top")


# ── F1 EKF 회전 버그 ─────────────────────────────────────────────
def f1():
    t1 = json.load(open(os.path.join(OUT, "t1_ekf_convention.json")))
    from t1_ekf_convention import EKF, EKFFixedInit, TRACE, simulate
    traces = {}
    for name, cls in (("원본 EKF", EKF), ("add_marker 수정본", EKFFixedInit)):
        simulate(cls, seed=0); traces[name] = {k: (v.copy() if hasattr(v, "copy") else v) for k, v in TRACE.items()}

    fig, (a, b) = plt.subplots(1, 2, figsize=(12, 4.8), gridspec_kw={"width_ratios": [1, 1.5]})
    rows = t1["single_add_marker_check"]
    yaw = [r["yaw_deg"] for r in rows]
    x = np.arange(len(yaw)); w = 0.36
    a.bar(x - w / 2 - 0.01, [r["original_landmark_err_m"] for r in rows], w, color=ORANGE, label="원본 EKF")
    a.bar(x + w / 2 + 0.01, [max(r["fixed_init_landmark_err_m"], 0.02) for r in rows], w, color=BLUE, label="add_marker 수정본")
    for i, r in enumerate(rows):
        a.text(i - w / 2, r["original_landmark_err_m"] + 0.08, f'{r["original_landmark_err_m"]:.2f}', ha="center", color=INK, fontsize=9)
        a.text(i + w / 2, 0.1, "0", ha="center", color=INK2, fontsize=9)
    a.set_xticks(x, [f"{y}°" for y in yaw]); a.set_xlabel("마커를 처음 볼 때 카메라 yaw")
    a.set_ylabel("새 마커 초기 위치 오차 (m)"); a.set_ylim(0, 4.8); a.grid(axis="x", visible=False)
    a.set_title("A. 회전한 뒤 처음 본 마커가 틀린 곳에 들어간다"); a.legend(loc="upper left")

    gt = traces["원본 EKF"]["gt"]; marks = traces["원본 EKF"]["marks"]
    b.plot(gt[:, 0], gt[:, 2], color=INK, lw=1.5, ls=(0, (4, 3)), label="참 궤적")
    for name, col in (("원본 EKF", ORANGE), ("add_marker 수정본", BLUE)):
        e = traces[name]["est"]; b.plot(e[:, 0], e[:, 2], color=col, label=name)
        for k, L in traces[name]["lm_est"].items():
            b.plot(L[0], L[2], "o", ms=7, color=col, mec=SURF, mew=1.5)
    for k, L in marks.items():
        b.plot(L[0], L[2], "s", ms=9, mfc="none", mec=INK, mew=1.5)
        b.annotate(f"M{k}", (L[0], L[2]), xytext=(6, 4), textcoords="offset points", color=INK2, fontsize=8.5)
    b.plot([], [], "s", ms=9, mfc="none", mec=INK, label="참 마커 위치")
    b.plot([], [], "o", ms=7, color=MUTED, label="추정 마커 위치")
    s = t1["simulation"]
    b.text(0.98, 0.30, f'선회 후 평균 위치오차\n원본 {s["original"]["cam_err_after_turn_m"]:.2f} m · 수정본 {s["fixed_init"]["cam_err_after_turn_m"]:.2f} m',
           transform=b.transAxes, va="bottom", ha="right", color=INK, fontsize=9.5)
    b.set_aspect("equal"); b.set_xlabel("x (m, 오른쪽)"); b.set_ylabel("z (m, 앞)")
    b.set_title("B. 합성 주행 (직진 2 m → 60° 선회 → 직진 2 m), 위에서 본 그림")
    b.legend(loc="lower right", fontsize=8.5)
    note(fig, "T1 · 합성 데이터(PnP 잡음 1 cm + 1%·거리). M3–M5는 선회 후에야 보이는 마커. 원본 코드는 import만 하고 수정본은 demo 서브클래스.")
    save(fig, "F1_ekf_rotation_bug.png")


# ── F2 캘리브레이션 거리 ──────────────────────────────────────────
def f2():
    t2 = json.load(open(os.path.join(OUT, "t2_calib_distance.json")))
    cal = [("current(recal_20261119)", "현재 (fx 1481)", BLUE), ("old(1080p/results)", "예전 1080p (fx 1244)", ORANGE),
           ("upstream(git HEAD)", "upstream 값 (fx 1415)", AQUA)]
    groups = [("100", ["distance_100cm", "distance_100cm_recheck", "distance_100cm_fixedfocus"]),
              ("150", ["distance_150cm", "distance_150cm_recheck", "distance_150cm_fixedfocus"])]
    lab = ["처음", "재확인", "고정초점"]
    fig, axs = plt.subplots(1, 2, figsize=(11, 4.2), sharey=False)
    for ax, (d, vids) in zip(axs, groups):
        truth = float(d)
        ax.axhspan(truth * 0.98, truth * 1.02, color=GRID, alpha=0.9, lw=0)
        ax.axhline(truth, color=INK, lw=1.2, ls=(0, (4, 3)))
        ax.text(-0.45, truth * 1.021, f"실측 {d} cm (회색 띠 ±2%)", va="bottom", color=INK2, fontsize=8.5)
        for j, (key, name, col) in enumerate(cal):
            ys = [t2["videos"][v]["by_calib"][key]["z_median_cm"] for v in vids]
            xs = np.arange(3) + (j - 1) * 0.18
            ax.plot(xs, ys, "o", ms=8, color=col, mec=SURF, mew=1.5, label=name)
            if key.startswith("current"):
                for xx, yy in zip(xs, ys):
                    ax.annotate(f"{yy:.1f}", (xx, yy), xytext=(-8, 0), textcoords="offset points", ha="right", va="center", fontsize=8, color=INK)
        ax.set_xticks(range(3), lab); ax.set_xlim(-0.5, 2.5); ax.grid(axis="x", visible=False)
        ax.set_ylabel("PnP 거리 z 중앙값 (cm)"); ax.set_title(f"실측 {d} cm 영상")
    h, l = axs[0].get_legend_handles_labels()
    fig.legend(h, l, loc="upper center", ncol=3, fontsize=9, bbox_to_anchor=(0.5, 1.06))
    note(fig, "T2 · 마커 0.15 m, DICT_5X5_50, IPPE_SQUARE (aruco-slam과 동일). 현재 값이 가장 가깝지만 고정초점에서 +4.5% — 마커 실측 크기 확인 필요.")
    save(fig, "F2_calibration_distance.png")


# ── F3 복도 정합 ─────────────────────────────────────────────────
def read_map(p):
    L = [l.strip() for l in open(p) if l.strip() and not l.startswith("#")]
    return {int(L[i]): np.array([float(v) for v in L[i + 1].split(",")]) for i in range(0, len(L), 3)}


def f3():
    T = np.loadtxt(os.path.join(OUT, "corridor_original", "trajectory.txt"))
    z = np.load(os.path.join(OUT, "corridor_original", "slam_to_landmarks_rigid.npz"))
    R, t = z["R"], z["t"]
    cam = T[:, 1:4] @ R.T + t
    k = 15; sm = np.array([cam[max(0, i - k // 2): i + k // 2 + 1].mean(0) for i in range(len(cam))])
    mk = read_map(os.path.join(OUT, "corridor_original", "map.txt"))
    gt = {int(r[0]): r[1:] for r in np.loadtxt(os.path.expanduser("~/car_calibration/corridor_map/aruco_landmarks.csv"), delimiter=",", skiprows=1)}
    summ = json.load(open(os.path.join(OUT, "corridor_original", "summary.json")))
    fig, ax = plt.subplots(figsize=(10, 4.6))
    ax.plot(cam[:, 0], cam[:, 1], color=BLUE, lw=0.8, alpha=0.35, label="카메라 궤적 (원시, 30 Hz)")
    ax.plot(sm[:, 0], sm[:, 1], color=BLUE, lw=2.2, label="카메라 궤적 (0.5 s 평활)")
    ax.plot(*cam[0, :2], "o", ms=9, color=BLUE, mec=SURF, mew=1.5); ax.annotate("시작", cam[0, :2], xytext=(-28, -14), textcoords="offset points", color=INK2)
    for i, g in gt.items():
        e = mk[i] @ R.T + t
        ax.plot([g[0], e[0]], [g[1], e[1]], color=MUTED, lw=1)
        ax.plot(g[0], g[1], "s", ms=11, mfc="none", mec=INK, mew=1.6)
        ax.plot(e[0], e[1], "o", ms=8, color=ORANGE, mec=SURF, mew=1.5)
        ax.annotate(f"M{i}  잔차 {summ['align']['residual_cm'][str(i)]:.1f} cm\n(높이 {g[2]:.2f} m)", (g[0], g[1]),
                    xytext=(10, 6), textcoords="offset points", fontsize=8.5, color=INK)
    ax.plot([], [], "s", ms=11, mfc="none", mec=INK, label="실측 마커 (aruco_landmarks.csv)")
    ax.plot([], [], "o", ms=8, color=ORANGE, label="SLAM 마커 (정합 후)")
    ax.set_aspect("equal"); ax.set_xlabel("복도 x (m)"); ax.set_ylabel("복도 y (m)")
    ax.set_xlim(-0.6, 5.0); ax.set_ylim(-0.9, 1.1)
    ax.set_title(f"복도 실험 재현 — 정합 RMS {summ['align']['rms_cm']:.1f} cm, 실제 이동 ≈ {summ['align']['net_disp_xy_m']:.1f} m, yaw < 2°")
    ax.legend(loc="lower right", fontsize=8.5, ncol=2)
    note(fig, "T4 · corridor_mapping_01.mkv를 원본 EKF로 다시 돌린 결과 (map.txt가 slam_result_01과 동일). 직진만 했기 때문에 회전 버그는 드러나지 않았다.")
    save(fig, "F3_corridor_alignment.png")


# ── F4 정지 안정성 ───────────────────────────────────────────────
def f4():
    T = np.genfromtxt(os.path.join(OUT, "session_20261120_002824", "trajectory_session.csv"), delimiter=",", names=True)
    t = T["t_s"]; P = np.c_[T["x"], T["y"], T["z"]]; P = P - np.median(P[15:], 0)
    fig, axs = plt.subplots(3, 1, figsize=(10, 5.2), sharex=True)
    for ax, i, name in zip(axs, range(3), ["x (오른쪽)", "y (아래)", "z (앞)"]):
        ax.axhspan(-1, 1, color=GRID, lw=0)
        ax.plot(t, P[:, i] * 100, color=BLUE, lw=1.3)
        sd = P[15:, i].std() * 100
        ax.text(1.0, 1.02, f"표준편차 {sd:.2f} cm", transform=ax.transAxes, ha="right", va="bottom", fontsize=8.5, color=INK)
        ax.set_ylabel(f"{name}\n편차 (cm)"); ax.set_ylim(-3.5, 3.5)
    pw = T["pwm"] != 0
    for ax in axs:
        ax.fill_between(t, -3.5, 3.5, where=pw, color=ORANGE, alpha=0.12, lw=0, step="mid")
    axs[0].text(t[pw][0], 2.6, " W 입력 구간 (가짜 보드, 카메라는 고정)", color=INK2, fontsize=8.5)
    axs[-1].set_xlabel("세션 시간 t_s (s) — drive_log.csv 와 같은 시간축")
    axs[0].set_title("정지 상태 SLAM 안정성 — 실제 C920 1080p, 마커 0–3이 화면에 보이는 상태")
    note(fig, "T6+T7 · 회색 띠 = ±1 cm. 카메라가 움직이지 않으므로 이 흔들림이 곧 측정 잡음이다 (yaw 표준편차 0.1°).")
    save(fig, "F4_static_stability.png")


# ── F5 처리량 ────────────────────────────────────────────────────
def f5():
    e = json.load(open(os.path.join(OUT, "t5b_perf_e2e.json")))
    t5 = json.load(open(os.path.join(OUT, "t5_jitter_perf.json")))["ekf_step_ms_vs_landmarks(3 visible)"]
    extra = t5["40"] - t5["10"]
    rows = [("1080p · 마커 4개 (실측)", e["1920x1080"], 0), ("1080p · 마커 20–40개", e["1920x1080"], extra),
            ("720p · 마커 4개 (실측)", e["1280x720"], 0), ("720p · 마커 20–40개", e["1280x720"], extra)]
    fig, ax = plt.subplots(figsize=(10, 3.8))
    for i, (lab, r, ex) in enumerate(rows):
        segs = [(r["decode_resize_ms"], MUTED, "디코드+리사이즈", None), (r["process_frame_ms"], BLUE, "검출+PnP+EKF (실측)", None)]
        if ex:
            segs.append((ex, ORANGE, "EKF 증가분 (랜드마크 20–40개, T5 추정)", "////"))
        left = 0
        for v, c, n, h in segs:
            ax.barh(i, v - 0.4, left=left, height=0.55, color=c if not h else SURF, ec=c, hatch=h, lw=1.2 if h else 0, label=n if (i == 0 or (h and i == 1)) else None)
            left += v
        ax.text(70, i, f"{left:.0f} ms  ≈ {1000 / left:.0f} fps", va="center", ha="left", fontsize=9.5, color=INK)
    ax.set_ylim(3.6, -1.0)
    for ms, lab in ((33.3, "30 fps"), (66.7, "15 fps")):
        ax.axvline(ms, color=INK, lw=1, ls=(0, (4, 3))); ax.text(ms + 0.6, -0.65, lab, fontsize=8.5, color=INK2)
    ax.set_yticks(range(len(rows)), [r[0] for r in rows]); ax.set_xlim(0, 84)
    ax.set_xlabel("프레임당 시간 (ms, 중앙값, 단일 스레드, 다른 작업 없음)"); ax.grid(axis="y", visible=False)
    ax.set_title("실시간 가능성 — 처리량은 충분하다 (막힌 건 카메라 공유)")
    ax.legend(loc="upper center", bbox_to_anchor=(0.45, -0.2), ncol=3, fontsize=8.5)
    note(fig, "T5b · 복도 영상(1080p 원본) 300프레임. 720p 행은 1080p를 디코드한 뒤 축소한 것이라 디코드 시간이 줄지 않았다 — 실제 720p 녹화면 더 빠르다.")
    fig.subplots_adjust(bottom=0.3)
    save(fig, "F5_throughput.png")


# ── F6 스모크: 명령 타임라인 + Ctrl+C ─────────────────────────────
def f6():
    S = os.path.join(OUT, "drive_logs", "20261120_002824")
    rows = list(csv.DictReader(open(os.path.join(S, "drive_log.csv"))))
    tv = [(float(r["t_s"]), int(r["value"][1:])) for r in rows if r["kind"] == "tx" and r["value"].startswith("V")]
    keys = [(float(r["t_s"]), r["value"]) for r in rows if r["kind"] == "key"]
    other = [(float(r["t_s"]), r["value"]) for r in rows if r["kind"] == "tx" and not r["value"].startswith("V")]
    fr = np.loadtxt(os.path.join(S, "video_frames.csv"), delimiter=",", skiprows=1)
    board = json.load(open(os.path.join(OUT, "drive_logs", "fake_board_q_1920x1080.json")))
    end = json.load(open(os.path.join(S, "session.json")))["summary"]["duration_s"]
    fig, (a, b) = plt.subplots(1, 2, figsize=(12, 4.2), gridspec_kw={"width_ratios": [2.1, 1]})
    a.fill_between([fr[0, 1], fr[-1, 1]], -8, 52, color=AQUA, alpha=0.10, lw=0)
    a.text(fr[0, 1] + 0.1, 48, f"영상 녹화 {len(fr)}프레임 (29.4 fps)", color=INK2, fontsize=8.5)
    t, v = zip(*tv); a.step(list(t) + [end], list(v) + [v[-1]], where="post", color=BLUE, label="보낸 속도 명령 V (PWM)")
    a.plot(t, v, "o", ms=3, color=BLUE)
    for j, (tt, k) in enumerate(keys):
        a.axvline(tt, color=GRID, lw=1, zorder=0)
        a.text(tt, 41 + 4 * (j % 2), {" ": "Space", "space": "Space"}.get(k, k.upper()), ha="center", fontsize=8.5, color=INK)
    prev_t, lvl = None, 0
    for tt, c in other:
        lvl = lvl + 1 if prev_t is not None and abs(tt - prev_t) < 0.3 else 0
        a.text(tt + 0.25 * lvl, -5, c, ha="center", fontsize=8.5, color=ORANGE, fontweight="bold"); prev_t = tt
    a.set_ylim(-8, 52); a.set_xlim(0, end + 0.3)
    a.set_xlabel("세션 시간 t_s (s)"); a.set_ylabel("PWM"); a.legend(loc="upper right", fontsize=8.5)
    a.set_title("A. 주행 노드 → 가짜 보드 명령 (Q 종료)")
    a.text(0.01, 0.5, "G보다 V0가 먼저\n→ 기본 속도로 출발하지 않음", transform=a.transAxes, fontsize=8.5, color=INK2)
    a.text(0.99, 0.5, f"종료: V0 · F · S\n가짜 보드 최종 can_go={board['final_can_go']}", transform=a.transAxes, ha="right", fontsize=8.5, color=INK2)

    labels = ["Q 종료", "Ctrl+C\n(현재 코드)", "Ctrl+C\n(수정 사본)"]
    vals = [324, 0, 331]
    cols = [BLUE, CRIT, BLUE]
    b.bar(range(3), [max(x, 4) for x in vals], color=cols, width=0.55)
    for i, x in enumerate(vals):
        b.text(i, max(x, 4) + 8, f"{x}", ha="center", color=INK, fontsize=10, fontweight="bold")
    b.text(1, 60, "× 프레임 시각 손실\nmkv 트레일러 없음", ha="center", color=CRIT, fontsize=8.5)
    b.set_xticks(range(3), labels); b.set_ylim(0, 380); b.grid(axis="x", visible=False)
    b.set_ylabel("video_frames.csv 프레임 수"); b.set_title("B. 종료 방식별 영상 시각 기록")
    note(fig, "T6 · 아두이노 대신 펌웨어 로직을 흉내 내는 가짜 보드(pty), 카메라는 실제 C920. 모터는 구동하지 않음. Ctrl+C로 깨진 세션도 t7이 mkv PTS로 시각을 복구한다 (오차 0 ms).")
    save(fig, "F6_drive_smoke.png")


# ── F7 실제 카메라 프레임 + 검출 ───────────────────────────────────
def f7():
    cap = cv2.VideoCapture(os.path.join(OUT, "drive_logs", "20261120_002824", "video.mkv"))
    cap.set(cv2.CAP_PROP_POS_FRAMES, 60); ok, fr = cap.read()
    K = np.load("calibration/camera_matrix.npy"); D = np.load("calibration/dist_coeffs.npy")
    d = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50), cv2.aruco.DetectorParameters())
    c, ids, _ = d.detectMarkers(fr)
    M = 0.15; OBJ = np.array([[-M / 2, M / 2, 0], [M / 2, M / 2, 0], [M / 2, -M / 2, 0], [-M / 2, -M / 2, 0]], np.float32)
    img = cv2.cvtColor(fr, cv2.COLOR_BGR2RGB)
    fig, ax = plt.subplots(figsize=(11, 6.4))
    ax.imshow(img); ax.axis("off")
    for ci, i in zip(c, ids.flatten()):
        q = ci[0]; poly = np.vstack([q, q[:1]])
        ax.plot(poly[:, 0], poly[:, 1], color="#3df5a8", lw=2)
        _, rv, tv = cv2.solvePnP(OBJ, q, K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        cv2.drawFrameAxes(img, K, D, rv, tv, 0.08, 3)
        ax.annotate(f"ID {i}\n{np.linalg.norm(tv) :.2f} m", q.max(0), xytext=(6, 0), textcoords="offset points",
                    color="white", fontsize=10, fontweight="bold",
                    bbox=dict(boxstyle="round,pad=0.25", fc="#0b0b0bcc", ec="none"))
    ax.imshow(img)
    ax.set_title("현재 C920 시야 (1080p, 고정 초점 19) — 복도 마커 4개 검출, PnP 거리", loc="left")
    ax.text(0, -0.02, "T6 세션 20261120_002824의 60번째 프레임. 축 = 마커 좌표계(빨강 x, 초록 y, 파랑 z). 거리는 현재 캘리브레이션 + 마커 0.15 m 가정.", transform=ax.transAxes, va="top", fontsize=8.5, color=MUTED)
    save(fig, "F7_camera_view.png")


# ── F8 데이터 흐름 (현재 / 내일 / 나중) ─────────────────────────────
def box(ax, x, y, w, h, text, fc=SURF, ec=INK2, tc=INK, lw=1.4, fs=9):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.02,rounding_size=0.08", fc=fc, ec=ec, lw=lw))
    ax.text(x + w / 2, y + h / 2, text, ha="center", va="center", fontsize=fs, color=tc)


def arrow(ax, x0, y0, x1, y1, col=INK2, text=None, ls="-"):
    ax.annotate("", (x1, y1), (x0, y0), arrowprops=dict(arrowstyle="-|>", color=col, lw=1.5, ls=ls))
    if text:
        ax.text((x0 + x1) / 2, (y0 + y1) / 2 + 0.12, text, ha="center", fontsize=8, color=col)


def f8():
    fig, ax = plt.subplots(figsize=(12, 6.2)); ax.set_xlim(0, 12); ax.set_ylim(0, 6.6); ax.axis("off")
    ax.text(0.1, 6.35, "A. 내일 쓰는 경로 — 녹화 후 오프라인 3D 궤적 (검증됨)", fontsize=12, fontweight="bold", color=INK)
    box(ax, 0.2, 4.6, 1.6, 1.0, "C920\n1080p MJPEG")
    box(ax, 2.4, 4.3, 2.6, 1.6, "wasd_drive (ROS 2)\n· ffmpeg 녹화 ← 카메라 독점\n· 키 → V/L/R/F/G/S\n· drive_log·session.json", ec=BLUE)
    box(ax, 5.6, 4.5, 1.5, 1.2, "Mega\n시리얼 9600 bps\n워치독 0.5 s", fs=8.5)
    box(ax, 7.6, 4.3, 1.9, 1.6, "세션 폴더\nvideo.mkv\nvideo_frames.csv\ndrive_log.csv")
    box(ax, 9.9, 4.3, 2.0, 1.6, "t7_session_slam.py\n--fixed --landmarks\n→ t_s,x,y,z,yaw,pwm", ec=BLUE, fc="#eef4fc")
    arrow(ax, 1.8, 5.1, 2.4, 5.1); arrow(ax, 5.0, 5.1, 5.6, 5.1)
    ax.annotate("", (8.55, 4.3), (3.7, 4.3), arrowprops=dict(arrowstyle="-|>", color=INK2, lw=1.5, connectionstyle="arc3,rad=0.18"))
    ax.text(6.1, 3.42, "녹화·로그 저장", ha="center", fontsize=8, color=INK2)
    arrow(ax, 9.5, 5.1, 9.9, 5.1)
    box(ax, 9.9, 3.2, 2.0, 0.75, "track_landmarks.csv\n(현장 실측, 오른손 좌표)", fs=8.5)
    arrow(ax, 10.9, 3.95, 10.9, 4.3)

    ax.text(0.1, 2.75, "B. 실시간이 막히는 이유와 나중 구조", fontsize=12, fontweight="bold", color=INK)
    box(ax, 0.2, 1.1, 1.6, 1.0, "C920")
    box(ax, 2.4, 1.6, 2.2, 0.8, "wasd_drive 의 ffmpeg", ec=BLUE)
    box(ax, 2.4, 0.3, 2.2, 0.8, "run_slam (실시간)", ec=CRIT)
    arrow(ax, 1.8, 1.8, 2.4, 2.0)
    ax.annotate("", (2.4, 0.7), (1.8, 1.4), arrowprops=dict(arrowstyle="-|>", color=CRIT, lw=1.5, ls="--"))
    ax.text(1.5, 0.45, "× 두 번째 열기 실패\n(isOpened False, T6)", fontsize=8.5, color=CRIT, ha="center")
    box(ax, 6.0, 0.6, 1.6, 1.0, "C920")
    box(ax, 8.1, 0.4, 3.8, 1.4, "SLAM 노드 하나가 카메라 소유\n· 캡처 → 검출+EKF → /car/pose (15–25 Hz)\n· 같은 프레임을 video.mkv 로 저장\n· wasd_drive 는 녹화 끔 (-p camera:=none)", ec=AQUA)
    arrow(ax, 7.6, 1.1, 8.1, 1.1)
    ax.text(9.95, 1.95, "제안 (미구현)", ha="center", fontsize=8.5, color=INK2)
    save(fig, "F8_pipeline.png")


# ── F9 마커 배치 가이드 ───────────────────────────────────────────
def f9():
    K = np.load("calibration/camera_matrix.npy")
    hfov = 2 * np.degrees(np.arctan(960 / K[0, 0]))
    fig, ax = plt.subplots(figsize=(10, 5.4))
    ax.add_patch(Wedge((0, 0), 4.0, 90 - hfov / 2, 90 + hfov / 2, width=3.5, fc=BLUE, alpha=0.10, ec=BLUE, lw=1.2))
    ax.add_patch(Wedge((0, 0), 4.5, 90 - hfov / 2, 90 + hfov / 2, width=0.5, fc=ORANGE, alpha=0.08, ec="none"))
    ax.add_patch(Rectangle((-0.15, -0.45), 0.3, 0.45, fc=INK2, ec="none"))
    ax.text(0.22, -0.25, "카메라(차)", fontsize=9, color=INK)
    ax.text(0, 2.3, f"추천 영역\n0.5–4 m, 가로 화각 {hfov:.0f}°", ha="center", fontsize=10, color=INK, fontweight="bold")
    ax.text(2.75, 3.75, "4–4.5 m: 오차 커짐\n(복도 M3 13 cm)", fontsize=8.5, color=INK2)
    ax.text(0, 0.18, "0.5 m 이내: 마커가 화면을 벗어남", ha="center", fontsize=8.5, color=INK2)
    for (x, y) in [(-0.9, 2.0), (0.8, 3.0), (-0.3, 3.6)]:
        ax.add_patch(Rectangle((x - 0.08, y - 0.08), 0.16, 0.16, fc=INK, ec=SURF, lw=1.5))
    ax.text(-0.9, 1.75, "보이는 마커 ≥ 2개\n(어느 위치에서든)", ha="center", va="top", fontsize=8.5, color=INK)
    rules = ["① DICT_5X5_50, ID 0–49, 중복 금지 · 검은 테두리 한 변 15.0 cm 실측",
             "② 마커 면은 주행 방향 쪽으로 (±60° 이내), 무광, 평평하게",
             "③ 곡선 구간은 안쪽·바깥쪽에 더 촘촘히 (회전 중 새 마커 = 위험 구간)",
             "④ 출발 위치에서 3개 이상 보이게 → 3초 정지 후 출발",
             "⑤ 중심 좌표 id,x,y,z 실측 (오른손 좌표: x=z_unity, y=−x_unity, z=y_unity)",
             "⑥ 최소 3개는 서로 일직선이 아니게, 높이를 섞어서 (정합용)"]
    for i, r in enumerate(rules):
        ax.text(4.9, 4.2 - i * 0.55, r, fontsize=9, color=INK, va="top")
    ax.set_xlim(-4.2, 11.8); ax.set_ylim(-0.7, 4.8); ax.set_aspect("equal"); ax.axis("off")
    ax.set_title("내일 트랙 마커 배치 가이드 (위에서 본 그림)", loc="left")
    save(fig, "F9_marker_layout.png")


if __name__ == "__main__":
    for f in (f2, f3, f4, f5, f6, f7, f8, f9, f1):
        f()
