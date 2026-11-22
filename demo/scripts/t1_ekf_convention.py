"""T1. 원본 EKF(filters/extended_kalman_filter.py)의 회전 규약 일관성 검사 + 합성 주행 시뮬레이션.

  A) 단일 검사: 카메라가 yaw θ 만큼 돌아 있을 때 add_marker()로 넣은 랜드마크를 h()로 다시 투영하면
     측정값이 그대로 나와야 한다(일관된 규약이면 오차 0).
  B) 합성 주행: 직진 → 60° 선회 → 직진. 새 마커가 '회전한 뒤'에 처음 보이게 배치.
     원본 / add_marker 수정본 / (add_marker + 반각) 수정본을 같은 측정으로 돌려 오차 비교.

원본 파일은 import만 한다. 수정본은 이 스크립트 안의 서브클래스.
"""
import common  # noqa: F401  (경로·chdir)
import json
import os

import numpy as np
from scipy.spatial.transform import Rotation

from filters.extended_kalman_filter import CAM_DIMS, ERROR_DIMS, LM_DIMS, QUAT_DIMS, XYZ_DIMS, EKF


class EKFFixedInit(EKF):
    """add_marker 만 h()와 같은 규약(R = 카메라→맵)으로: t_ml = R p + t_mc."""

    def add_marker(self, idx, pose, uncertainity=None):
        R = Rotation.from_quat(self.state[QUAT_DIMS], scalar_first=True).as_matrix()
        # 원본은 inv(R) p + t → 이걸 R p + t 가 되게 측정값을 미리 돌려 넣는다
        p = np.array(pose, float).copy()
        p[XYZ_DIMS] = R @ R @ p[XYZ_DIMS]   # 원본 내부에서 R^T 가 곱해지므로 R·R 을 먼저 곱해 상쇄
        super().add_marker(idx, p, uncertainity)


class EKFFixedInitHalf(EKFFixedInit):
    """+ 오차 쿼터니언 스케일 일치: h()는 dq=(1,e) (회전각≈2|e|), 원본 update는 dq=(1,δ/2) (회전각≈|δ|)."""

    def update(self, ids, poses):
        from scipy import sparse
        import scipy.sparse.linalg  # noqa: F401
        z, h, dh = self.parse_poses(ids, poses)
        dh = sparse.csr_matrix(dh)
        P = sparse.csr_matrix(self.uncertainty)
        r = sparse.eye(dh.shape[0], format="csc") * 0.9
        s = dh @ P @ dh.T + r
        s_inv = sparse.linalg.spsolve(s, sparse.eye(s.shape[0], format="csc"))
        K = P @ dh.T @ s_inv
        inn = K @ (z - h)
        self.state[XYZ_DIMS] += inn[XYZ_DIMS]
        self.state[CAM_DIMS:] += inn[CAM_DIMS:]
        dq = np.array([1.0, *inn[ERROR_DIMS]])      # h() 와 같은 (1, e)
        dq /= np.linalg.norm(dq)
        q = Rotation.from_quat(dq, scalar_first=True) * Rotation.from_quat(self.state[QUAT_DIMS], scalar_first=True)
        self.state[QUAT_DIMS] = q.as_quat(scalar_first=True)
        self.state[ERROR_DIMS] = 0
        self.uncertainty = (np.eye(self.uncertainty.shape[0]) - K @ dh) @ self.uncertainty


def ry(deg):
    """카메라 광학 좌표계(x 오른쪽, y 아래, z 앞)에서 차의 yaw = y축 회전."""
    return Rotation.from_euler("y", deg, degrees=True)


def single_check():
    rows = []
    for yaw in (0, 10, 30, 60, 90):
        out = {"yaw_deg": yaw}
        for name, cls in (("original", EKF), ("fixed_init", EKFFixedInit)):
            f = cls(np.array([0, 0, 0, 1, 0, 0, 0, 0, 0, 0], float))
            R = ry(yaw)
            f.state[QUAT_DIMS] = R.as_quat(scalar_first=True)
            f.state[XYZ_DIMS] = [0.3, 0.0, 1.0]
            L = np.array([1.0, 0.1, 3.0])                    # 참 랜드마크(맵)
            p = R.as_matrix().T @ (L - f.state[XYZ_DIMS])    # h() 규약의 참 측정
            f.add_marker(7, np.hstack([p, 0, 0, 0]))
            L_est = f.state[CAM_DIMS:CAM_DIMS + LM_DIMS]
            out[name + "_landmark_err_m"] = round(float(np.linalg.norm(L_est - L)), 4)
        rows.append(out)
    return rows


TRACE = {}   # 그림용: 마지막 simulate() 의 궤적·마커


def simulate(cls, seed=0, fps=30):
    rng = np.random.default_rng(seed)
    # 궤적: 2 m 직진(3 s) → 반경 1.5 m 로 60° 선회 → 2 m 직진
    poses = []
    t, yaw = np.zeros(3), 0.0
    for _ in range(90):
        poses.append((t.copy(), yaw)); t = t + ry(yaw).apply([0, 0, 2 / 90])
    arc = np.radians(60) * 1.5
    for _ in range(60):
        poses.append((t.copy(), yaw)); yaw += 60 / 60; t = t + ry(yaw).apply([0, 0, arc / 60])
    for _ in range(90):
        poses.append((t.copy(), yaw)); t = t + ry(yaw).apply([0, 0, 2 / 90])
    # 마커: 처음 구간 3개 + 선회 후에야 보이는 3개 (높이 y=+0.1, 카메라보다 아래)
    end = poses[-1][0]
    marks = {0: [-0.6, 0.1, 1.5], 1: [0.6, 0.1, 2.5], 2: [0.0, 0.1, 3.2]}
    for i, d in enumerate((1.0, 2.0, 3.0)):
        c = end + ry(60).apply([(-0.6 if i % 2 else 0.6), 0.1, d - 1.5])
        marks[3 + i] = list(c)
    marks = {k: np.array(v) for k, v in marks.items()}

    f = cls(np.array([0, 0, 0, 1, 0, 0, 0, 0, 0, 0], float))
    est, seen = [], set()
    for tc, yw in poses:
        R = ry(yw).as_matrix()
        ids, meas = [], []
        for k, L in marks.items():
            p = R.T @ (L - tc)
            if p[2] < 0.3 or p[2] > 4.0 or abs(p[0] / p[2]) > np.tan(np.radians(35)):
                continue
            noise = rng.normal(0, 0.01 + 0.01 * p[2], 3)      # PnP 오차 ≈ 1 cm + 1 %·거리
            ids.append(k); meas.append(np.hstack([p + noise, 0, 0, 0]))
        if ids:
            seen.update(ids)
            f.observe(np.array(ids), np.array(meas))
        cam, _ = f.get_poses()
        est.append(cam[:3].copy())
    est = np.array(est); gt = np.array([p[0] for p in poses])
    err = np.linalg.norm(est - gt, axis=1)
    lm_err = {}
    for k, i in f.landmarks.items():
        L = f.state[CAM_DIMS + LM_DIMS * i: CAM_DIMS + LM_DIMS * (i + 1)]
        lm_err[int(k)] = round(float(np.linalg.norm(L - marks[k])), 3)
    TRACE.update(est=est, gt=gt, marks=marks, lm_est={int(k): f.state[CAM_DIMS + LM_DIMS * i: CAM_DIMS + LM_DIMS * (i + 1)].copy() for k, i in f.landmarks.items()})
    yaw_est = Rotation.from_quat(f.state[QUAT_DIMS], scalar_first=True).as_rotvec(degrees=True)[1]
    return {
        "cam_err_mean_m": round(float(err.mean()), 3),
        "cam_err_before_turn_m": round(float(err[:90].mean()), 3),
        "cam_err_after_turn_m": round(float(err[150:].mean()), 3),
        "cam_err_final_m": round(float(err[-1]), 3),
        "yaw_final_est_deg": round(float(yaw_est), 1), "yaw_final_true_deg": 60.0,
        "landmark_err_m": lm_err, "seen": sorted(int(s) for s in seen),
    }


if __name__ == "__main__":
    res = {"single_add_marker_check": single_check(), "simulation": {}}
    for name, cls in (("original", EKF), ("fixed_init", EKFFixedInit), ("fixed_init+half_angle", EKFFixedInitHalf)):
        runs = [simulate(cls, seed=s) for s in range(5)]
        agg = {k: round(float(np.mean([r[k] for r in runs])), 3) for k in runs[0] if k.startswith("cam_err") or k.startswith("yaw_final_est")}
        agg["landmark_err_m(seed0)"] = runs[0]["landmark_err_m"]
        res["simulation"][name] = agg
    print(json.dumps(res, ensure_ascii=False, indent=1))
    with open(os.path.join(common.OUT, "t1_ekf_convention.json"), "w") as fh:
        json.dump(res, fh, ensure_ascii=False, indent=1)
