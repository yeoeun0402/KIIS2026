"""영상 기록 보조 함수 단위 테스트 (카메라·ffmpeg 없이)"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from car_drive.video_recorder import find_camera, frames_from_timestamps, input_start_epoch  # noqa: E402


def _cam(root, n, name, index):
    d = root / f"video{n}"
    d.mkdir()
    (d / "name").write_text(name + "\n")
    (d / "index").write_text(f"{index}\n")


def test_find_camera_picks_c920_capture_node_not_builtin(tmp_path):
    _cam(tmp_path, 0, "USB2.0 HD UVC WebCam: USB2.0 HD", 0)    # 노트북 내장 캠
    _cam(tmp_path, 1, "USB2.0 HD UVC WebCam: USB2.0 HD", 1)
    _cam(tmp_path, 2, "HD Pro Webcam C920", 0)
    _cam(tmp_path, 3, "HD Pro Webcam C920", 1)                  # 메타데이터 노드
    assert find_camera("c920", sys_root=str(tmp_path)) == ("/dev/video2", "HD Pro Webcam C920")


def test_find_camera_never_falls_back_to_builtin(tmp_path):
    _cam(tmp_path, 0, "USB2.0 HD UVC WebCam: USB2.0 HD", 0)
    assert find_camera("c920", sys_root=str(tmp_path)) == (None, None)


def test_find_camera_numeric_order(tmp_path):
    _cam(tmp_path, 10, "HD Pro Webcam C920", 0)
    _cam(tmp_path, 2, "HD Pro Webcam C920", 0)
    assert find_camera("c920", sys_root=str(tmp_path))[0] == "/dev/video2"


def test_frames_from_timestamps():
    # 영상은 0 ms부터, 첫 프레임 벽시계 = 1000002.5, 세션 t0 = 1000000.0
    text = "# timecode format v2\n0\n33\n\n67\n"
    assert frames_from_timestamps(text, 1000000.0, 1000002.5) == [(0, 2.5), (1, 2.533), (2, 2.567)]


def test_input_start_epoch_from_ffmpeg_log():
    log = ("Input #0, video4linux2,v4l2, from '/dev/video0':\n"
           "  Duration: N/A, start: 1795014797.758000, bitrate: N/A\n"
           "  Stream #0:0: Video: mjpeg (Baseline), yuvj422p(pc), 1280x720, 30 fps\n"
           "Output #0, matroska, to 'video.mkv':\n  Duration: N/A, start: 0.000000\n")
    assert input_start_epoch(log) == 1795014797.758
    assert input_start_epoch("[video4linux2] Cannot open video device") is None
