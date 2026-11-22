#!/usr/bin/env python3
"""지도(build_map 결과) 점검 — 마커가 2차선 바깥 선 위에 있는지, 매트 밖으로 나갔는지, 재투영 오차.
필요하면 마커를 선에 맞춰 지도 전체를 돌림 (--realign).

  python3 -B ~/demo/track/tools/map_check.py <지도 폴더>                 점검만 + markers_vs_line.png
  python3 -B ~/demo/track/tools/map_check.py <지도 폴더> --realign        마커가 선 위에 오도록 지도 전체 회전·이동
                                                                           (markers.csv · trajectory.csv · map.yaml 을 고침, 원본 *.orig 로 남김)
기준: 마커를 모두 2차선 바깥 경계선 위에 세웠다 (track.yaml lane.centerline_offset_from_outer_line_m = 0.45 m,
      2026-11-22 사용자 확인). 다른 선에 세운 마커는 --skip 14 처럼 빼거나, 자동으로 버려짐 (맞춤 뒤 > 0.5 m).
회전 맞춤: 회전 + 이동 3자유도, 큰 오차에 둔감한 손실 (soft L1). 크기는 바꾸지 않음 (마커 크기 값이 정함).
"""
import argparse
import csv
import os
import shutil
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import trackmap as tm  # noqa: E402

GOOD_M, OK_M, OUTLIER_M = 0.15, 0.30, 0.50


def lane_lines():
    t, _, cl = tm.load_track()
    P = cl[:, 1:3]
    h = np.radians(cl[:, 3])
    Nr = np.c_[np.sin(h), -np.cos(h)]                 # 진행 방향(ccw) 오른쪽 = 바깥
    off = float(t["lane"].get("centerline_offset_from_outer_line_m", 0.45))
    return P, Nr, off, t


def offsets(X, P, Nr, line_off):
    """마커마다 바깥 선에서의 가로 거리 [m] (+ = 선보다 바깥)."""
    d = np.linalg.norm(X[:, None, :] - P[None], axis=2)
    i = d.argmin(1)
    return np.einsum("ij,ij->i", X - P[i], Nr[i]) - line_off


def read_markers(path):
    rows = list(csv.DictReader(open(path)))
    return rows, np.array([[float(r["x_m"]), float(r["y_m"])] for r in rows]), [int(r["id"]) for r in rows]


def fit_rigid(X, use, P, Nr, line_off, ctr):
    from scipy.optimize import least_squares

    def T(q, Y):
        c, s = np.cos(q[0]), np.sin(q[0])
        return (Y - ctr) @ np.array([[c, -s], [s, c]]).T + ctr + q[1:3]
    r = least_squares(lambda q: offsets(T(q, X[use]), P, Nr, line_off[use]), [0, 0, 0], loss="soft_l1", f_scale=0.1,
                      diff_step=1e-4)
    return r.x, (lambda Y: T(r.x, Y))


def plot(path, X, ids, e, P, Nr, line_off, traj, title):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, a = plt.subplots(figsize=(13, 9.3))
    for o, st, lb in [(line_off, "r-", "lane-2 outer line (markers should be here)"), (0, "k:", "lane-2 center"),
                      (-0.40, "b--", "lane 1|2 divider"), (-1.25, "m-", "lane-1 inner line")]:
        Q = P + o * Nr
        a.plot(Q[:, 0], Q[:, 1], st, lw=1.2, label=lb)
    if traj is not None:
        a.plot(traj[:, 0], traj[:, 1], "g-", lw=0.7, alpha=0.7, label="driven")
    for p, i, v in zip(X, ids, e):
        c = "tab:green" if abs(v) < GOOD_M else "orange" if abs(v) < OK_M else "red"
        a.plot(*p, "o", ms=10, color=c, mec="k")
        a.annotate(f"{i}: {v:+.2f}m", p, xytext=(6, 6), textcoords="offset points", fontsize=11, weight="bold", color=c)
    a.plot([0, 16, 16, 0, 0], [0, 0, 11, 11, 0], "k-", lw=0.8)
    a.set_aspect("equal"); a.set_xlim(-0.8, 16.8); a.set_ylim(-0.8, 11.8); a.grid(alpha=0.3); a.legend(loc="center", fontsize=9)
    a.set_title(title, fontsize=12)
    plt.tight_layout(); plt.savefig(path, dpi=80); plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("map_dir")
    ap.add_argument("--realign", action="store_true", help="마커가 바깥 선 위에 오도록 지도 전체를 회전·이동해서 저장")
    ap.add_argument("--skip", type=int, nargs="*", default=[], help="바깥 선이 아닌 곳에 세운 마커 ID (맞춤에서 뺌)")
    a = ap.parse_args()
    d = os.path.abspath(os.path.expanduser(a.map_dir))
    P, Nr, line_off, t = lane_lines()
    rows, X, ids = read_markers(os.path.join(d, "markers.csv"))
    rep = {}
    if os.path.exists(os.path.join(d, "report.json")):
        import json
        rep = json.load(open(os.path.join(d, "report.json")))
    tp = os.path.join(d, "trajectory.csv")
    traj = np.genfromtxt(tp, delimiter=",", names=True) if os.path.exists(tp) else None
    # 마커마다 세운 선 (기본 = 2차선 바깥 경계선). 다른 선에 세운 마커: track.yaml markers.sets.<use>.marker_line_m {ID: 2차선 중심에서 가로 거리}
    #   예: 3: -1.25 (1차선 왼쪽 두꺼운 선 = 안쪽 경계선, S자에서 길 잃지 않게 — 2026-11-22 사용자)
    ms = t.get("markers", {}).get("sets", {}).get(os.environ.get("MARKERS") or t.get("markers", {}).get("use", ""), {}) or {}
    special = {int(k): float(v) for k, v in (ms.get("marker_line_m") or {}).items()}
    lo = np.array([special.get(i, line_off) for i in ids])
    e0 = offsets(X, P, Nr, lo)

    print(f"지도: {d}")
    if rep:
        print(f"  마커 {rep.get('n_markers')}개 · 크기 {rep.get('marker_size_m')} m · 재투영 {rep.get('reproj_rms_px')} px "
              f"(새 마커는 4 px 이하면 사용 가능) · 차선 {rep.get('lane_coverage', 0) * 100:.0f}% 돎 · 맞춤 {rep.get('aligned_to')} · "
              f"판정 {rep.get('verdict')}")
        for g in rep.get("covis_gaps", []):
            print(f"  ! 이어보기 끊김 x {g['x']:.1f} y {g['y']:.1f} ({g['gap_s']:.1f} s) — 그 근처 마커 추가 권장")
    for i in special:
        if i in ids:
            print(f"  (ID {i} 는 2차선 중심에서 {special[i]:+.2f} m 선 위에 세운 것으로 봄 — track.yaml marker_line_m)")
    print("  마커별 세운 선에서 거리 [m] (+ 바깥): " + "  ".join(f"{i}:{v:+.2f}" for i, v in zip(ids, e0)))
    print(f"  |거리| 중앙 {np.median(abs(e0)) * 100:.0f} cm · 최대 {abs(e0).max() * 100:.0f} cm (ID {ids[int(abs(e0).argmax())]})")

    st = t["start"].get("base_link") or {}
    ctr = np.array([float(st.get("x", 1.265)), float(st.get("y", 6.659))])
    use = np.array([i not in a.skip for i in ids])
    q, T = fit_rigid(X, use, P, Nr, lo, ctr)
    e1 = offsets(T(X), P, Nr, lo)
    bad = (abs(e1) > OUTLIER_M) & use
    if bad.any():                                      # 다른 선에 놓인 마커로 보이는 것 빼고 다시
        use &= ~bad
        q, T = fit_rigid(X, use, P, Nr, lo, ctr)
        e1 = offsets(T(X), P, Nr, lo)
    print(f"  마커를 선에 맞추려면: 회전 {np.degrees(q[0]):+.2f}° (출발점 기준) · 이동 ({q[1]:+.2f}, {q[2]:+.2f}) m → "
          f"|거리| 중앙 {np.median(abs(e1[use])) * 100:.0f} cm · 최대 {abs(e1[use]).max() * 100:.0f} cm")
    far = [(i, v) for i, v, u in zip(ids, e1, use) if not u]
    if far:
        print("  ! 맞춰도 선에서 먼 마커 (다른 선에 세웠거나 그 구간 지도가 틀림 — 실제 위치 확인): "
              + ", ".join(f"ID {i} {v:+.2f} m" for i, v in far))
    off_mat = [i for i, p in zip(ids, T(X) if a.realign else X) if not (-0.1 <= p[0] <= 16.1 and -0.1 <= p[1] <= 11.1)]
    if off_mat:
        print(f"  ! 매트 밖 마커: {off_mat}")

    if a.realign:
        for f in ("markers.csv", "trajectory.csv", "map.yaml"):
            src = os.path.join(d, f)
            if os.path.exists(src) and not os.path.exists(src + ".orig"):
                shutil.copy2(src, src + ".orig")
        Xn = T(X)
        dq = q[0]
        from scipy.spatial.transform import Rotation
        Rz = Rotation.from_euler("z", dq)
        for r, p in zip(rows, Xn):
            r["x_m"], r["y_m"] = f"{p[0]:.4f}", f"{p[1]:.4f}"
            rq = Rz * Rotation.from_quat([float(r["qx"]), float(r["qy"]), float(r["qz"]), float(r["qw"])])
            x_, y_, z_, w_ = rq.as_quat()
            r["qw"], r["qx"], r["qy"], r["qz"] = (f"{v:.6f}" for v in (w_, x_, y_, z_))
        with open(os.path.join(d, "markers.csv"), "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
        if traj is not None:
            xy = T(np.c_[traj["x_m"], traj["y_m"]])
            with open(tp, "w", newline="") as f:
                w = csv.writer(f); w.writerow(traj.dtype.names)
                for k, row in enumerate(traj):
                    w.writerow([int(row["frame"]), f"{row['t_s']:.3f}", f"{xy[k, 0]:.4f}", f"{xy[k, 1]:.4f}",
                                f"{(row['theta_deg'] + np.degrees(dq) + 180) % 360 - 180:.3f}"])
            traj = np.genfromtxt(tp, delimiter=",", names=True)
        my = yaml.safe_load(open(os.path.join(d, "map.yaml")))
        my["aligned_to"] = "markers_on_outer_line"
        my["realign"] = {"rot_deg": round(float(np.degrees(dq)), 3), "shift_m": [round(float(v), 4) for v in q[1:3]],
                         "about": [float(ctr[0]), float(ctr[1])], "skipped_ids": [int(i) for i, _ in far]}
        yaml.safe_dump(my, open(os.path.join(d, "map.yaml"), "w"), sort_keys=False, allow_unicode=True)
        X, e_show = Xn, e1
        print(f"  ✓ 지도를 {np.degrees(dq):+.2f}° 돌려 저장 (원본: *.orig)")
    else:
        e_show = e0
    png = os.path.join(d, "markers_vs_line.png")
    tr = np.c_[traj["x_m"], traj["y_m"]] if traj is not None else None
    plot(png, X, ids, e_show, P, Nr, line_off, tr,
         f"{os.path.basename(os.path.dirname(d))}/{os.path.basename(d)}  markers vs lane-2 outer line  "
         f"(|err| median {np.median(abs(e_show)) * 100:.0f} cm, max {abs(e_show).max() * 100:.0f} cm){'  [realigned]' if a.realign else ''}")
    print(f"  그림: {png}")


if __name__ == "__main__":
    main()
