import aruco_slam_ros  # noqa: F401  (sys.path 먼저)
import numpy as np
from scipy.spatial.transform import Rotation

from aruco_slam_ros.frames import R_LINK_OPT, FrameChain, T_base_link, T_from_pose, inv_T, yaw_deg


def pose(t, R):
    return np.r_[t, Rotation.from_matrix(R).as_quat(scalar_first=True)]


def test_optical_axes():
    assert np.allclose(R_LINK_OPT @ [0, 0, 1], [1, 0, 0])    # 광학 앞 = 차 앞
    assert np.allclose(R_LINK_OPT @ [1, 0, 0], [0, -1, 0])   # 광학 오른쪽 = 차 −y
    assert np.allclose(R_LINK_OPT @ [0, 1, 0], [0, 0, -1])   # 광학 아래 = 차 −z
    assert np.isclose(np.linalg.det(R_LINK_OPT), 1)


def test_first_frame_is_origin():
    fc = FrameChain(T_base_link(0.25, 0.0, 0.12, 8.0))
    assert np.allclose(fc.base_in_map([0, 0, 0, 1, 0, 0, 0]), np.eye(4))


def test_forward_drive_moves_base_along_x():
    # 카메라가 광학 z(앞)로 1 m → 차가 map x 로 1 m (장착 숙임 0)
    fc = FrameChain(T_base_link(0.25, 0.0, 0.12))
    T = fc.base_in_map(pose([0, 0, 1.0], np.eye(3)))
    assert np.allclose(T[:3, 3], [1, 0, 0]) and np.isclose(yaw_deg(T), 0)


def test_left_turn_rotates_about_rear_axle():
    # 차가 뒷축 중심으로 제자리 좌회전 90° → base 위치는 그대로, 카메라는 원을 그림
    mount = T_base_link(0.30, 0.0, 0.15)
    fc = FrameChain(mount)
    T_map_base = np.eye(4); T_map_base[:3, :3] = Rotation.from_euler("z", 90, degrees=True).as_matrix()
    T_slam_opt = inv_T(fc.T_bo) @ T_map_base @ fc.T_bo           # 그때 EKF 가 낼 카메라 자세
    p = np.r_[T_slam_opt[:3, 3], Rotation.from_matrix(T_slam_opt[:3, :3]).as_quat(scalar_first=True)]
    T = fc.base_in_map(p)
    assert np.allclose(T[:3, 3], 0, atol=1e-9)
    assert np.isclose(yaw_deg(T), 90)
    cam = fc.cam_in_map(p)[:3, 3]
    assert np.allclose(cam, [0, 0.30, 0.15])                     # 카메라는 왼쪽 0.30 m 로 이동
    assert np.isclose(np.linalg.norm(T_slam_opt[:3, 3]), 0.30 * np.sqrt(2))


def test_pitch_down_marker_on_floor():
    # 15° 숙인 카메라가 정면 바닥점을 보면 map z ≈ 0
    mount = T_base_link(0.0, 0.0, 0.20, 15.0)
    fc = FrameChain(mount)
    d = 0.20 / np.tan(np.radians(15))                           # 광축이 바닥에 닿는 거리
    p_opt = np.array([0, 0, np.hypot(d, 0.20)])
    p_map = fc.point_in_map(p_opt)[0]
    assert np.allclose(p_map, [d, 0, 0], atol=1e-9)


def test_T_from_pose_roundtrip():
    R = Rotation.from_euler("xyz", [10, -20, 30], degrees=True).as_matrix()
    T = T_from_pose(pose([1, 2, 3], R))
    assert np.allclose(T[:3, :3], R) and np.allclose(T[:3, 3], [1, 2, 3])
    assert np.allclose(inv_T(T) @ T, np.eye(4))
