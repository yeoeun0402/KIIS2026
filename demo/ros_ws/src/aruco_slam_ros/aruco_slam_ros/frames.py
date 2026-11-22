"""좌표계 변환 (PLAN_ROS.md §2.1). 순수 numpy — ROS 없이 시험 가능.

프레임
  camera_optical  OpenCV 기준: 원점 = 광학 중심, x 오른쪽 · y 아래 · z 앞
  camera_link     같은 원점, ROS 관례 x 앞 · y 왼쪽 · z 위
  base_link       뒷바퀴 축 중심의 바닥 투영, x 앞 · y 왼쪽 · z 위
  slam            EKF 내부 지도 = 첫 프레임의 camera_optical
  map             첫 프레임의 base_link (바닥이 xy 평면, x = 출발 시 차 정면)

T_a_b 는 b 좌표를 a 좌표로 옮기는 4x4 동차변환 (p_a = T_a_b · p_b). 쿼터니언은 EKF 와 같은 (w, x, y, z).
"""
import numpy as np
from scipy.spatial.transform import Rotation

# camera_link 에서 본 camera_optical 축: opt z(앞)=link x, opt x(오른쪽)=link −y, opt y(아래)=link −z
R_LINK_OPT = np.array([[0.0, 0.0, 1.0],
                       [-1.0, 0.0, 0.0],
                       [0.0, -1.0, 0.0]])


def make_T(R=None, t=None):
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    if t is not None:
        T[:3, 3] = t
    return T


def inv_T(T):
    R, t = T[:3, :3], T[:3, 3]
    return make_T(R.T, -R.T @ t)


def T_from_pose(p):
    """EKF 상태 앞 7개 [x y z qw qx qy qz] → T_slam_opt."""
    p = np.asarray(p, float)
    return make_T(Rotation.from_quat(p[3:7], scalar_first=True).as_matrix(), p[:3])


def pose_from_T(T):
    """T → (xyz, quat_wxyz)."""
    return T[:3, 3].copy(), Rotation.from_matrix(T[:3, :3]).as_quat(scalar_first=True)


def T_base_link(x=0.0, y=0.0, z=0.0, pitch_down_deg=0.0, yaw_deg=0.0, roll_deg=0.0):
    """장착 실측값 → T_base_link. pitch_down = 아래로 숙인 각(+), ROS 피치(+y 축 회전)와 같은 부호."""
    R = Rotation.from_euler("ZYX", [yaw_deg, pitch_down_deg, roll_deg], degrees=True).as_matrix()
    return make_T(R, [x, y, z])


def T_base_opt(mount):
    """mount = T_base_link(...) → T_base_opt."""
    return mount @ make_T(R_LINK_OPT)


class FrameChain:
    """EKF(slam 프레임) 결과를 map/base_link 로 옮긴다.

    map = 첫 프레임의 base_link 로 정의 → T_map_slam = T_base_opt (첫 프레임에서 opt = slam 원점)
      T_map_base(t) = T_base_opt · T_slam_opt(t) · T_base_opt⁻¹
      L_map        = T_base_opt · L_slam
    """

    def __init__(self, mount=None):
        self.T_bo = T_base_opt(mount if mount is not None else np.eye(4))
        self.T_ob = inv_T(self.T_bo)

    def base_in_map(self, cam_pose):
        return self.T_bo @ T_from_pose(cam_pose) @ self.T_ob

    def cam_in_map(self, cam_pose):
        return self.T_bo @ T_from_pose(cam_pose)

    def point_in_map(self, p_slam):
        p = np.atleast_2d(p_slam)
        return p @ self.T_bo[:3, :3].T + self.T_bo[:3, 3]


def yaw_deg(T):
    """map 에서의 진행 방향(°): x 축 기준 반시계 +."""
    return float(np.degrees(np.arctan2(T[1, 0], T[0, 0])))
