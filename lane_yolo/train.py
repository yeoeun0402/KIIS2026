"""YOLOv8-seg 차선 모델 학습 (클래스: 0 outer_solid, 1 center_dashed, 2 inner_solid)

학습/검증은 Play(주행) 단위로 나눕니다. 0.1 s 간격 연속 사진은 서로 거의 같아서
사진을 무작위로 섞어 나누면 검증 점수가 실제보다 좋게 나옵니다.

사용 (가상환경 C:\\Users\\haepi\\venvs\\kiis_yolo):
  python train.py --train 20261115_204947 20261115_204754 --val 20261115_205703
  python train.py ... --epochs 80 --model yolov8s-seg.pt
결과: lane_yolo/runs/<name>/weights/best.pt, results.png, 검증 예측 그림
"""
import argparse
from pathlib import Path

from ultralytics import YOLO

HERE = Path(__file__).resolve().parent
YOLO_ROOT = HERE.parent / "yolo"
NAMES = {0: "outer_solid", 1: "center_dashed", 2: "inner_solid"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--train", nargs="+", required=True, help="학습용 Play 폴더 이름 (yolo/ 아래)")
    ap.add_argument("--val", nargs="+", required=True, help="검증용 Play 폴더 이름")
    ap.add_argument("--model", default="yolov8n-seg.pt")
    ap.add_argument("--epochs", type=int, default=50)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--batch", type=int, default=16)
    ap.add_argument("--name", default="lane_seg")
    a = ap.parse_args()

    for p in a.train + a.val:
        if not (YOLO_ROOT / p / "labels").is_dir():
            raise SystemExit(f"labels 없음: {YOLO_ROOT / p} (먼저 mask_to_yolo.py 실행)")

    lines = [f"path: {YOLO_ROOT.as_posix()}", "train:"]
    lines += [f"  - {p}/images" for p in a.train]
    lines += ["val:"] + [f"  - {p}/images" for p in a.val]
    lines += ["names:"] + [f"  {i}: {n}" for i, n in NAMES.items()]
    data_yaml = HERE / "dataset.yaml"
    data_yaml.write_text("\n".join(lines) + "\n", encoding="utf-8")

    model = YOLO(a.model)
    model.train(
        data=str(data_yaml), epochs=a.epochs, imgsz=a.imgsz, batch=a.batch, device=0, workers=2,
        project=str(HERE / "runs"), name=a.name, exist_ok=False,
        fliplr=0.0,   # 좌우 뒤집기 끔: 바깥 실선은 항상 오른쪽, 안쪽 실선은 왼쪽이라 뒤집으면 클래스 의미가 바뀜
        seed=0, deterministic=True,
    )


if __name__ == "__main__":
    main()
