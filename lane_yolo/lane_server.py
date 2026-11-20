"""실시간 차선 판단 서버: Unity가 보낸 Car_Camera 사진 → YOLOv8-seg → 차선 3개 곡선/직선 → 제어기 판단 → Unity로 답장
(Unity는 받은 답장을 MATLAB switch_discrete.m으로 그대로 전달)

실행 순서: Unity Play → 이 서버 → MATLAB switch_discrete
  C:\\Users\\haepi\\venvs\\kiis_yolo\\Scripts\\python.exe lane_server.py
  (옵션) --port 55002 --weights runs/lane_seg/weights/best.pt --kappa-th 0.05 --conf 0.25

통신 (TCP, 이 서버가 기다리고 Unity CarCameraCapture가 접속):
  Unity → 서버:  "IMG,<k>,<t>,<run>,<w>,<h>,<nbytes>\\n" + RGB 바이트 (Unity 순서: 아래 행부터)
  서버 → Unity:  "LANE,<k>,<판단>,<보이는 차선 수>,<outer 상태>,<outer kappa>,<dashed 상태>,<dashed kappa>,
                  <inner 상태>,<inner kappa>,<추론 ms>\\n"
                 판단 = PP_PID | PID,  상태 = curve | straight | none,  kappa 없으면 nan
판단 규칙과 곡률 계산은 lane_state.py의 judge()와 같습니다 (보이는 차선이 모두 곡선이면 PP_PID, 아니면 PID).

feature 사진 저장 (MATLAB switching_2/switch2_discrete.m용, 2026-11-16 추가. 기존 switching은 이 포트를 안 씀):
  MATLAB이 --ctrl-port(기본 55003)로 직접 접속해서 한 줄씩 보냄
    "RUN,<저장 폴더>"                         이번 실행의 사진 폴더
    "SAVE,<k>,<이름>,<규칙>,<kappa_th>,<kappa_bc>"  사진 k에 detection(차선 마스크, 차선별 곡률·상태, 판단)을 그려
                                                     <저장 폴더>/k<k 6자리>_<이름>.jpg로 저장
  최근 사진 BUFFER장을 메모리에 들고 있다가 그림. --ctrl-port 0이면 이 기능을 끔
"""
import argparse
import json
import socket
import sys
import threading
import time
from collections import OrderedDict
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from lane_state import CLASS_BGR, NAMES, GroundProjector, judge, masks_from_result  # noqa: E402

BUFFER = 60                     # 메모리에 들고 있는 최근 사진 수 (6 s)


class FeatureSaver:
    """최근 사진·마스크·판단을 들고 있다가 MATLAB이 요청하면 detection을 그려 저장"""

    def __init__(self, proj):
        self.proj = proj
        self.lock = threading.Lock()
        self.frames = OrderedDict()
        self.out_dir = None
        self.saved = 0

    def put(self, k, bgr, masks, lines):
        with self.lock:
            self.frames[k] = (bgr, masks, lines)
            self.frames.move_to_end(k)
            while len(self.frames) > BUFFER:
                self.frames.popitem(last=False)

    def render(self, k, bgr, masks, lines, label, rule, kth, kbc):
        img = bgr.copy()
        over = img.copy()
        for c, m in masks.items():
            if m is not None:
                over[m] = CLASS_BGR[c]
        img = cv2.addWeighted(over, 0.45, img, 0.55, 0)
        for c, m in masks.items():
            if m is not None:
                cnts, _ = cv2.findContours(m.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(img, cnts, -1, CLASS_BGR[c], 1)
        # 곡률을 잰 구간 (차 앞 1 m, 5 m 바닥선)
        p = self.proj
        for z in (1.0, 5.0):
            v = int(round(p.cy + p.f * p.hgt / z))
            if 0 <= v < p.h:
                cv2.line(img, (0, v), (p.w - 1, v), (255, 255, 255), 1, cv2.LINE_AA)
                cv2.putText(img, f"{z:.0f} m", (4, v - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
        # 글자: 윗줄 판단, 아래 차선별 상태 (MATLAB의 kappa_th로 다시 판단한 값)
        cv2.rectangle(img, (0, 0), (p.w, 92), (0, 0, 0), -1)
        cv2.putText(img, f"k {k}   {label.replace('_to_', ' -> ')}   ({rule})", (8, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2, cv2.LINE_AA)
        y = 44
        for c, name in NAMES.items():
            kap = lines[name][0]
            if kap is None:
                txt = f"{name}: none"
            else:
                st = "curve" if abs(kap) > kth else "straight"
                txt = f"{name}: {st}  kappa {kap:+.3f} 1/m"
            cv2.putText(img, txt, (8 + 210 * c, y), cv2.FONT_HERSHEY_SIMPLEX, 0.48, CLASS_BGR[c], 1, cv2.LINE_AA)
        cv2.putText(img, f"kappa_th {kth:.3f}   kappa_BC {kbc:.3f}   (lines: outer red, dashed green, inner blue)",
                    (8, 72), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1, cv2.LINE_AA)
        return img

    def handle(self, line):
        f = line.strip().split(",")
        if f[0] == "RUN" and len(f) >= 2:
            self.out_dir = Path(",".join(f[1:]))
            self.out_dir.mkdir(parents=True, exist_ok=True)
            print(f"[lane_server] feature 사진 폴더: {self.out_dir}", flush=True)
        elif f[0] == "SAVE" and len(f) >= 6 and self.out_dir is not None:
            k, label, rule, kth, kbc = int(f[1]), f[2], ",".join(f[3:-2]), float(f[-2]), float(f[-1])
            with self.lock:
                fr = self.frames.get(k)
            if fr is None:
                print(f"[lane_server] feature 사진 k {k}가 메모리에 없어 저장하지 못함", flush=True)
                return
            img = self.render(k, *fr, label, rule, kth, kbc)
            cv2.imwrite(str(self.out_dir / f"k{k:06d}_{label}.jpg"), img, [cv2.IMWRITE_JPEG_QUALITY, 92])
            self.saved += 1

    def serve(self, port):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("127.0.0.1", port))
        srv.listen(1)
        while True:
            conn, _ = srv.accept()
            self.saved = 0
            try:
                for raw in conn.makefile("rb"):
                    try:
                        self.handle(raw.decode("utf-8"))
                    except Exception as e:      # 사진 하나 실패해도 판단 서버는 계속
                        print(f"[lane_server] feature 사진 저장 오류: {e}", flush=True)
            except OSError:
                pass
            finally:
                conn.close()
                print(f"[lane_server] MATLAB 사진 저장 연결 종료 (사진 {self.saved}장)", flush=True)


def recv_exact(f, n):
    buf = f.read(n)
    if buf is None or len(buf) < n:
        raise ConnectionError("연결 끊김")
    return buf


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=55002)
    ap.add_argument("--weights", default=None)
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--kappa-th", type=float, default=0.05)
    ap.add_argument("--zmin", type=float, default=1.0)
    ap.add_argument("--zmax", type=float, default=5.0)
    ap.add_argument("--min-span", type=float, default=1.0)
    ap.add_argument("--ctrl-port", type=int, default=55003, help="MATLAB feature 사진 저장 명령 포트 (0이면 끔)")
    a = ap.parse_args()

    from ultralytics import YOLO
    weights = Path(a.weights) if a.weights else sorted(HERE.glob("runs/*/weights/best.pt"), key=lambda p: p.stat().st_mtime)[-1]
    model = YOLO(str(weights))
    cam = json.loads((HERE / "camera.json").read_text(encoding="utf-8"))
    proj = GroundProjector(cam)
    model.predict(np.zeros((cam["height"], cam["width"], 3), np.uint8), verbose=False, device=0)   # GPU 예열
    print(f"[lane_server] 모델 {weights}")
    print(f"[lane_server] 127.0.0.1:{a.port} 에서 Unity를 기다리는 중 (kappa-th {a.kappa_th}, conf {a.conf})", flush=True)
    saver = None
    if a.ctrl_port:
        saver = FeatureSaver(proj)
        threading.Thread(target=saver.serve, args=(a.ctrl_port,), daemon=True).start()
        print(f"[lane_server] 127.0.0.1:{a.ctrl_port} 에서 MATLAB feature 사진 저장 명령을 받음", flush=True)

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", a.port))
    srv.listen(1)
    while True:
        conn, addr = srv.accept()
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        print(f"[lane_server] Unity 연결됨 {addr}", flush=True)
        f = conn.makefile("rb")
        n, last, ms_sum = 0, None, 0.0
        try:
            while True:
                header = f.readline()
                if not header:
                    break
                parts = header.decode("ascii").strip().split(",")
                if parts[0] != "IMG":
                    continue
                k, t, run, w, h, nb = int(parts[1]), float(parts[2]), int(parts[3]), int(parts[4]), int(parts[5]), int(parts[6])
                rgb = np.frombuffer(recv_exact(f, nb), np.uint8).reshape(h, w, 3)[::-1]   # Unity는 아래 행부터
                t0 = time.perf_counter()
                bgr = np.ascontiguousarray(rgb[:, :, ::-1])                                # YOLO는 BGR
                res = model.predict(bgr, conf=a.conf, retina_masks=True, verbose=False, device=0)[0]
                masks = masks_from_result(res)
                lines, decision = judge(masks, proj, a.kappa_th, a.zmin, a.zmax, a.min_span)
                ms = (time.perf_counter() - t0) * 1000
                if saver is not None:
                    saver.put(k, bgr, masks, lines)      # 답장보다 먼저 넣어 둠 (MATLAB의 SAVE,k가 바로 와도 있게)
                vis = sum(s != "none" for _, _, s in lines.values())
                fields = [f"{st},{('nan' if kap is None else f'{kap:.4f}')}" for kap, _, st in lines.values()]
                conn.sendall(f"LANE,{k},{decision},{vis},{','.join(fields)},{ms:.1f}\n".encode("ascii"))
                n += 1; ms_sum += ms
                if decision != last:
                    print(f"[lane_server] run {run} k {k:5d} t {t:7.2f}  → {decision:6s} "
                          f"({', '.join(f'{nm} {st}' for nm, (_, _, st) in lines.items())})", flush=True)
                    last = decision
        except (ConnectionError, OSError) as e:
            print(f"[lane_server] 연결 종료: {e}", flush=True)
        finally:
            conn.close()
            if n:
                print(f"[lane_server] 사진 {n}장 처리, 평균 판단 시간 {ms_sum / n:.1f} ms. 다시 기다리는 중", flush=True)


if __name__ == "__main__":
    main()
