"""세션 영상 기록 — ffmpeg로 카메라 MJPEG를 재인코딩 없이 video.mkv에 저장 (CPU 거의 안 씀).

프레임마다 ffmpeg가 받은 순간의 벽시계(epoch ms)를 함께 남기고(-use_wallclock_as_timestamps),
끝날 때 세션 시간축으로 바꿔 video_frames.csv(frame,t_s)를 만든다 → drive_log.csv의 t_s와 바로 맞춰 볼 수 있다.
t_s에는 카메라·USB 지연(수십 ms)이 들어 있다.

카메라는 이름으로 고른다(기본 C920). 못 찾으면 녹화하지 않는다 — 노트북 내장 웹캠으로 몰래 바꾸지 않는다
(예전에 usbipd/인덱스 때문에 내장 캠이 C920 대신 잡힌 적이 있음).
"""
import glob
import os
import re
import signal
import subprocess
import time

FFMPEG_TS = "video_ts.txt"


def find_camera(name_filter="c920", sys_root="/sys/class/video4linux"):
    """이름에 name_filter가 들어간 카메라의 캡처 노드(/dev/videoN). 없으면 (None, None)."""
    for d in sorted(glob.glob(os.path.join(sys_root, "video*")), key=lambda p: int(p.rsplit("video", 1)[1] or 0)):
        try:
            name = open(os.path.join(d, "name")).read().strip()
            index = open(os.path.join(d, "index")).read().strip()
        except OSError:
            continue
        if name_filter.lower() in name.lower() and index == "0":   # index 0 = 캡처, 1+ = 메타데이터 노드
            return "/dev/" + os.path.basename(d), name
    return None, None


def camera_name(device, sys_root="/sys/class/video4linux"):
    try:
        return open(os.path.join(sys_root, os.path.basename(device), "name")).read().strip()
    except OSError:
        return None


_START_RE = re.compile(r"Input #0, video4linux2.*?start:\s*([0-9.]+)", re.S)


def input_start_epoch(ffmpeg_log):
    """ffmpeg info 로그의 'Input #0, video4linux2 … start: 1795014797.758000' → 첫 프레임 벽시계(초). 없으면 None."""
    m = _START_RE.search(ffmpeg_log)
    return float(m.group(1)) if m else None


def frames_from_timestamps(ts_text, t0_epoch, start_epoch=0.0):
    """mkvtimestamp_v2 본문(영상 시작부터 ms, 한 줄씩) → [(frame, t_s)].
    t_s = start_epoch + ms/1000 − t0_epoch (start_epoch = 첫 프레임 벽시계)."""
    out = []
    for line in ts_text.splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        try:
            out.append((len(out), round(start_epoch + float(line) / 1000.0 - t0_epoch, 3)))
        except ValueError:
            continue
    return out


class VideoRecorder:
    def __init__(self, out_dir, device, t0_epoch, size="1280x720", fps=30, input_format="mjpeg", live_jpeg=""):
        """out_dir = 세션 폴더 — 파일은 video.mkv · video_frames.csv · video_ffmpeg.log
        live_jpeg = [twin] 비우면 끔. 경로를 주면 같은 ffmpeg 가 15 fps · 폭 960 JPEG 한 장을 계속 덮어씀
                    (원자적 교체) → 웹 대시보드 카메라 화면 · 실시간 위치 추정용. 녹화(video.mkv)는 그대로 무손실 복사."""
        self.out_dir, self.device, self.t0_epoch = out_dir, device, t0_epoch
        self.info = {"device": device, "name": camera_name(device), "size": size, "fps_requested": fps,
                     "input_format": input_format, "file": os.path.basename(self._p("video.mkv")),
                     "frames_csv": os.path.basename(self._p("video_frames.csv"))}
        self._err = open(self._p("video_ffmpeg.log"), "w")
        # 영상 파일은 0초부터(-copyts를 쓰면 재생 시간이 epoch 기준 수십만 시간으로 나온다).
        # 첫 프레임의 벽시계는 ffmpeg가 info 로그에 "start: <epoch>"로 찍는다 → stop()에서 읽어 t_s로 바꾼다.
        cmd = ["ffmpeg", "-hide_banner", "-loglevel", "info", "-nostats", "-nostdin",
               "-f", "v4l2", "-input_format", input_format, "-framerate", str(fps), "-video_size", size,
               "-use_wallclock_as_timestamps", "1", "-i", device,
               "-map", "0:v", "-c:v", "copy", "-f", "matroska", self._p("video.mkv"),
               "-map", "0:v", "-c:v", "copy", "-f", "mkvtimestamp_v2", self._p(FFMPEG_TS)]
        base = list(cmd)                                         # 녹화만 (실시간 JPEG 실패 시 이걸로 다시)
        if live_jpeg:
            os.makedirs(os.path.dirname(live_jpeg) or ".", exist_ok=True)
            # 지난 실행의 파일이 남아 있으면 ffmpeg 가 "already exists" 로 녹화까지 통째로 멈춤 (2026-11-21 실제로 겪음)
            # → 미리 지우고, 이 출력에만 덮어쓰기를 허용 (-y 는 전역이라 쓰지 않음: video.mkv 를 실수로 덮지 않게)
            try:
                os.remove(live_jpeg)
            except FileNotFoundError:
                pass
            cmd += ["-map", "0:v", "-vf", "fps=15,scale=960:-2", "-q:v", "5", "-f", "image2", "-update", "1",
                    "-atomic_writing", "1", live_jpeg]
            self.info["live_jpeg"] = live_jpeg
        # start_new_session: 터미널 Ctrl+C(SIGINT)가 ffmpeg 에 직접 가지 않게 — stop()이 SIGINT 한 번만 보내야
        # mkv 트레일러·타임스탬프 파일을 정상으로 닫는다 (두 번 받으면 "Immediate exit requested")
        self._proc = subprocess.Popen(cmd, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=self._err,
                                      start_new_session=True)
        if live_jpeg:
            # 안전장치: 실시간 JPEG 출력 때문에 ffmpeg 가 바로 죽으면 녹화가 통째로 없어진다
            # → 1.5 초 안에 죽으면 JPEG 출력을 빼고 녹화만 다시 시작 (대시보드 카메라 화면만 포기)
            for _ in range(15):
                if self._proc.poll() is not None:
                    break
                time.sleep(0.1)
            if self._proc.poll() is not None:
                for f in ("video.mkv", FFMPEG_TS):
                    try:
                        os.remove(self._p(f))
                    except FileNotFoundError:
                        pass
                self._err.seek(0); self._err.truncate()          # 첫 프레임 시각을 새 실행 로그에서 읽도록
                self.info["live_jpeg"] = None
                self.info["live_jpeg_error"] = "ffmpeg 가 실시간 JPEG 출력과 함께 시작하지 못해 녹화만 다시 시작"
                self._proc = subprocess.Popen(base, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                                              stderr=self._err, start_new_session=True)

    def _p(self, suffix):
        return os.path.join(self.out_dir, suffix)

    def error_if_dead(self):
        """ffmpeg가 이미 끝났으면 오류 문자열(카메라 사용 중·형식 미지원 등), 녹화 중이면 None. 막지 않는다."""
        if self._proc.poll() is None:
            return None
        self._err.flush()
        return self._read_err() or f"ffmpeg 종료 코드 {self._proc.returncode}"

    def _read_err(self):
        try:
            return open(self._p("video_ffmpeg.log")).read().strip()
        except OSError:
            return ""

    def stop(self):
        """녹화를 끝내고 <세션>_video_frames.csv를 만든다. session.json에 넣을 정보를 돌려준다."""
        if self._proc.poll() is None:
            self._proc.send_signal(signal.SIGINT)        # mkv를 정상적으로 닫게
            try:
                self._proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._proc.kill()
                self._proc.wait()
        self._err.close()
        ts_path = self._p(FFMPEG_TS)
        frames = []
        start = input_start_epoch(self._read_err())
        self.info["first_frame_epoch"] = start
        if os.path.exists(ts_path) and start is not None:
            frames = frames_from_timestamps(open(ts_path).read(), self.t0_epoch, start)
            with open(self._p("video_frames.csv"), "w") as f:
                f.write("frame,t_s\n")
                f.writelines(f"{i},{t:.3f}\n" for i, t in frames)
            os.remove(ts_path)
        mkv = self._p("video.mkv")
        self.info["frames"] = len(frames)
        self.info["first_frame_t_s"] = frames[0][1] if frames else None
        self.info["last_frame_t_s"] = frames[-1][1] if frames else None
        span = (frames[-1][1] - frames[0][1]) if len(frames) > 1 else 0
        self.info["fps_measured"] = round((len(frames) - 1) / span, 2) if span > 0 else None
        self.info["size_mb"] = round(os.path.getsize(mkv) / 1e6, 1) if os.path.exists(mkv) else 0
        # 로그는 info 수준(스트림 정보 포함) — 오류처럼 보이는 줄만 고르고, 정지 때마다 나오는 무해한 줄은 뺀다
        err = [l for l in self._read_err().splitlines()
               if re.search(r"error|failed|invalid|busy|cannot|no such", l, re.I) and "VIDIOC_QBUF" not in l]
        if err:
            self.info["ffmpeg_errors"] = err[-3:]
        return self.info
