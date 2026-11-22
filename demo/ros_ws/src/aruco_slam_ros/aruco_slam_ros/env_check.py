"""P0 통과 확인: ros2 run aruco_slam_ros env_check
SLAM 노드가 쓸 파이썬 환경(OpenCV·numpy·scipy·EKF·rclpy·메시지)을 한 번에 점검하고 결과를 표로 낸다."""
import aruco_slam_ros  # noqa: F401  ← 반드시 맨 처음 (sys.path 설정)
import os
import sys


def main():
    rows, ok_all = [], True

    def check(name, fn):
        nonlocal ok_all
        try:
            v = fn()
            rows.append(("OK ", name, v))
        except Exception as e:  # noqa: BLE001
            ok_all = False
            rows.append(("FAIL", name, f"{type(e).__name__}: {e}"))

    import numpy as np
    check("numpy", lambda: f"{np.__version__}  ({os.path.dirname(np.__file__)})")

    def cv():
        import cv2
        major, minor = map(int, cv2.__version__.split(".")[:2])
        assert (major, minor) >= (4, 7), f"cv2 {cv2.__version__} < 4.7"
        cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50))
        return f"{cv2.__version__}  ArucoDetector OK  ({os.path.dirname(cv2.__file__)})"
    check("opencv", cv)

    def sp():
        import scipy
        from scipy.spatial.transform import Rotation
        Rotation.from_quat([1, 0, 0, 0], scalar_first=True)
        return scipy.__version__ + "  scalar_first OK"
    check("scipy", sp)

    def ekf():
        here = os.getcwd()
        os.chdir(aruco_slam_ros.SLAM_ROOT)          # calibration/*.npy 상대경로
        try:
            from filters.extended_kalman_filter import EKF
            tr = EKF(np.array([0, 0, 0, 1, 0, 0, 0, 0, 0, 0], float))
            K = tr.calib_matrix
        finally:
            os.chdir(here)
        return f"{aruco_slam_ros.SLAM_ROOT}  fx {K[0, 0]:.1f} fy {K[1, 1]:.1f} cx {K[0, 2]:.1f} cy {K[1, 2]:.1f}"
    check("EKF (aruco-slam-fixed)", ekf)

    def ros():
        import rclpy
        from geometry_msgs.msg import PoseStamped  # noqa: F401
        from nav_msgs.msg import Path  # noqa: F401
        from sensor_msgs.msg import CameraInfo, CompressedImage  # noqa: F401
        from visualization_msgs.msg import MarkerArray  # noqa: F401
        import tf2_ros  # noqa: F401
        rclpy.init()
        n = rclpy.create_node("aruco_env_check")
        n.destroy_node()
        rclpy.shutdown()
        return "rclpy·메시지·tf2_ros import + 노드 생성 OK"
    check("ROS 2", ros)

    def frames():
        from aruco_slam_ros.frames import FrameChain, T_base_link
        fc = FrameChain(T_base_link(0.2, 0, 0.15, 10))
        T = fc.base_in_map([0, 0, 0, 1, 0, 0, 0])
        assert np.allclose(T, np.eye(4)), T
        return "첫 프레임 base = map 원점 OK"
    check("frames", frames)

    print(f"python {sys.version.split()[0]}  ({sys.executable})")
    for s, n, v in rows:
        print(f"[{s}] {n:24s} {v}")
    print("P0 환경 점검:", "통과" if ok_all else "실패")
    sys.exit(0 if ok_all else 1)


if __name__ == "__main__":
    main()
