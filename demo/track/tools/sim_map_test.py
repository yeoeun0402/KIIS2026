#!/usr/bin/env python3
"""[demo] build_map 합성 시험 — 정답을 아는 가짜 마커·주행으로 지도 정확도 확인.

  python3 -B sim_map_test.py [--noise 0.5] [--pitch 2.8] [--roll 1.5] [--lat 0.10] [--seed 0]

마커: 2차선 바깥(0.85 m)·안쪽(1.65 m) 도로 밖에 번갈아, 중심 높이 0.4 m, 다가오는 차 쪽을 비스듬히 보게, 각자 ±2° 기울어짐.
주행: 출발 자세에서 3초 정지 → 2차선 중심선 ± 횡오차(사인파)로 0.5 m/s 한 바퀴, 30 fps.
실제 장착 pitch·roll 을 mount.yaml 과 다르게 두고 추정이 맞는지 본다.
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import trackmap as tm  # noqa: E402
from build_map import build  # noqa: E402
from trackmap import Rotation  # noqa: E402


def parse(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--noise", type=float, default=0.5)
    ap.add_argument("--pitch", type=float, default=2.8)
    ap.add_argument("--roll", type=float, default=1.5)
    ap.add_argument("--yaw", type=float, default=1.0)
    ap.add_argument("--lat", type=float, default=0.10)
    ap.add_argument("--n-markers", type=int, default=16)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--fscale", type=float, default=1.0, help="실제 초점거리 / 캘리브레이션 초점거리 (관측 생성에만 사용)")
    ap.add_argument("--fix-focal", action="store_true")
    ap.add_argument("--align", default="start")
    ap.add_argument("--start-err", type=float, nargs=3, default=[0, 0, 0], help="차를 놓은 실제 오차 dx dy dθ(deg)")
    return ap.parse_args(argv)


def make_sim(a):
    """정답 마커·주행·관측 생성 → dict."""
    rng = np.random.default_rng(a.seed)
    track, start, cl = tm.load_track()
    mount = tm.load_mount()
    K, D = tm.load_intrinsics(1920)
    size = 0.20
    s, X, Y = cl[:, 0], cl[:, 1], cl[:, 2]; L = s[-1] + (s[1] - s[0])

    def at(q):
        q = np.mod(q, L)
        p = np.c_[np.interp(q, s, X, period=L), np.interp(q, s, Y, period=L)]
        p2 = np.c_[np.interp(q + 0.05, s, X, period=L), np.interp(q + 0.05, s, Y, period=L)]
        tg = p2 - p; tg /= np.linalg.norm(tg, axis=1, keepdims=True)
        return p, tg, np.c_[-tg[:, 1], tg[:, 0]]           # 위치, 접선, 왼쪽 법선 (=안쪽)

    # 마커 정답
    gt = {}
    for i, q in enumerate(np.linspace(0, L, a.n_markers, endpoint=False) + 1.5):
        p, tg, nl = at(np.array([q]))
        side = -1 if i % 2 == 0 else 1                        # 바깥 / 안쪽
        off = 0.85 if side < 0 else 1.65
        pos = np.r_[p[0] + side * off * nl[0], 0.4]
        face = -side * nl[0] * 0.6 - tg[0] * 0.8             # 도로 쪽 + 다가오는 차 쪽으로 비스듬히 (실제 배치처럼)
        yaw = np.arctan2(face[1], face[0]) + np.radians(rng.normal(0, 10))
        # 마커 좌표: z = 앞면 법선(face), y = 위, x = y × z
        zc = np.r_[np.cos(yaw), np.sin(yaw), 0]; yc = np.r_[0, 0, 1.0]; xc = np.cross(yc, zc)
        R = np.c_[xc, yc, zc] @ Rotation.from_euler("xy", rng.normal(0, 2, 2), degrees=True).as_matrix()
        gt[i] = (R, pos)

    # 주행 정답 (base_link). 출발: start (+ 놓기 오차), 3초 정지
    se = np.r_[a.start_err[:2], np.radians(a.start_err[2])]
    s0 = L - mount["rear_axle_to_front_bumper"]
    fps, v = 30.0, 0.5
    n_still = 90; n_move = int(L / v * fps)
    q = s0 + np.r_[np.zeros(n_still), np.arange(1, n_move + 1) * v / fps]
    p, tg, nl = at(q)
    lat = a.lat * np.sin(2 * np.pi * (q - s0) / 7.0) * (q > s0)
    P = p + lat[:, None] * nl
    hd = np.arctan2(np.gradient(P[:, 1]), np.gradient(P[:, 0])); hd[:n_still] = start[2]
    true = np.c_[P, hd]
    true[:n_still] = start + se
    true[n_still:] += se * np.exp(-np.arange(n_move) / 60.0)[:, None]   # 놓기 오차가 출발 후 점점 사라짐

    R_bo, t_bo = tm.T_base_opt(mount, a.pitch, a.roll, a.yaw)
    R_wc, t_wc = tm.cam_from_base(true, R_bo, t_bo)
    obj = tm.marker_obj(size)
    Kt = K.copy(); Kt[0, 0] *= a.fscale; Kt[1, 1] *= a.fscale
    ids_all, uv_all = [], []
    for f in range(len(true)):
        ids, uvs = [], []
        for k, (R, pos) in gt.items():
            Cw = (R @ obj.T).T + pos
            Xc = (Cw - t_wc[f]) @ R_wc[f]
            if (Xc[:, 2] < 0.3).any() or np.linalg.norm(Xc.mean(0)) > 8:
                continue
            n_c = R_wc[f].T @ R[:, 2]
            if n_c @ (-Xc.mean(0) / np.linalg.norm(Xc.mean(0))) < 0.3:   # 70° 넘게 비스듬하면 검출 안 됨
                continue
            uv = tm.project(Xc, Kt, D)
            if (uv < 0).any() or (uv[:, 0] > 1920).any() or (uv[:, 1] > 1080).any():
                continue
            if rng.random() < 0.1:
                continue
            ids.append(k); uvs.append(uv + rng.normal(0, a.noise, uv.shape))
        ids_all.append(np.array(ids, int)); uv_all.append(np.array(uvs).reshape(-1, 4, 2))
    times = np.arange(len(true)) / fps
    print(f"[sim] 프레임 {len(true)} · 마커 {len(gt)} · 프레임당 평균 {np.mean([len(i) for i in ids_all]):.1f}개 · "
          f"실제 pitch {a.pitch} roll {a.roll} (mount.yaml {mount['pitch_down_deg']} / {mount['roll_deg']})")
    return dict(gt=gt, true=true, ids=ids_all, uvs=uv_all, times=times, t_move=n_still / fps - 0.01, K=K, D=D,
                size=size, mount=mount, start=start, cl=cl)


def main():
    a = parse()
    S = make_sim(a)
    gt, true, ids_all, uv_all, times, K, D, size, mount, start, cl = (S[k] for k in
        ("gt", "true", "ids", "uvs", "times", "K", "D", "size", "mount", "start", "cl"))
    n_still_t = S["t_move"]
    r = build(ids_all, uv_all, times, n_still_t, K, D, size, mount, start, cl, stride=2, align=a.align,
              fix_focal=a.fix_focal)
    err = []
    for k, (rv, p) in r["markers"].items():
        e = np.linalg.norm(p - gt[k][1])
        ang = np.degrees((Rotation.from_rotvec(rv).inv() * Rotation.from_matrix(gt[k][0])).magnitude())
        err.append((k, e, np.linalg.norm(p[:2] - gt[k][1][:2]), ang))
    err = np.array(err)
    for k, e, exy, ang in err:
        if e > 0.10:
            print(f"[sim]   마커 {int(k)}: 오차 {e * 100:.0f} cm · 방향 {ang:.1f}° · 관측 {r['stats'][int(k)]['n_obs']} · rms {r['stats'][int(k)]['rms_px'] or -1:.1f}px · 최소거리 {r['stats'][int(k)]['min_dist_m'] or -1:.1f} m")
    tr = np.array([true[f] for f in r["kfs"]])
    te = np.hypot(*(r["cams"][:, :2] - tr[:, :2]).T)
    the = np.degrees(np.abs((r["cams"][:, 2] - tr[:, 2] + np.pi) % (2 * np.pi) - np.pi))
    print(f"[sim] 마커 위치 오차 3D 평균 {err[:, 1].mean() * 100:.1f} cm · 최대 {err[:, 1].max() * 100:.1f} cm · "
          f"평면(xy) 최대 {err[:, 2].max() * 100:.1f} cm · 방향 평균 {err[:, 3].mean():.2f}°")
    print(f"[sim] 차량 궤적 오차 평균 {te.mean() * 100:.1f} cm · 최대 {te.max() * 100:.1f} cm · θ 평균 {the.mean():.2f}°")
    print(f"[sim] 장착 추정 pitch {r['pr'][0]:.2f}° (참 {a.pitch}) · roll {r['pr'][1]:.2f}° (참 {a.roll}) · yaw {r['pr'][2]:.2f}° (참 {a.yaw}) · "
          f"f 배율 {r['pr'][3]:.4f} (참 {a.fscale}) · 재투영 RMS {np.sqrt((r['rms'] ** 2).mean()):.2f}px · 정합 {r['info']}")


if __name__ == "__main__":
    main()
