"""맵 텍스처(image.png)에서 차선 3개만 골라 클래스별 색으로 칠한 라벨 텍스처를 만듭니다.

Unity(CarCameraCapture)는 사진을 찍을 때 맵 재질만 이 텍스처로 바꿔 같은 시점에서 한 번 더 렌더링하고,
그 결과(masks/*.png)를 mask_to_yolo.py가 YOLO 라벨(labels/*.txt)로 바꿉니다.

클래스 (색은 BGR이 아닌 RGB 기준):
  0 outer_solid   바깥 실선 (도로와 잔디 경계)   (255, 0, 0)
  1 center_dashed 중앙 점선                      (0, 255, 0)
  2 inner_solid   안쪽 실선 (도로와 안쪽 회색 경계) (0, 0, 255)
  그 외(주차선, 횡단보도, 정지선, 글자, 안쪽 회색 구역의 선 등)는 검정

방법:
  1) 아스팔트(어두운 회색)를 닫힘 연산으로 메워 도로 고리(ring)를 만듦 (점선·얇은 표시선이 메워짐)
  2) 고리의 바깥 경계 / 안쪽 경계(안쪽 회색 섬)와의 거리로 흰 페인트를 분류
     - 바깥 경계 근처 흰색 → 바깥 실선, 안쪽 경계 근처 흰색 → 안쪽 실선
     - 고리 안에서 f = d_out / (d_out + d_in) 이 0.5 근처이고, 가늘고(점선 두께), f 폭이 좁은 덩어리 → 중앙 점선
       (횡단보도 줄무늬·정지선은 두껍거나 도로를 가로질러 f 폭이 넓어서 빠짐)

사용:  python make_label_texture.py   →  lane_label_texture.png, lane_label_check.png (확인용 겹친 그림)
"""
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
MAP = HERE.parent / "KIIS2026_AutoCar" / "Assets" / "image.png"

COLORS_RGB = {0: (255, 0, 0), 1: (0, 255, 0), 2: (0, 0, 255)}
NAMES = {0: "outer_solid", 1: "center_dashed", 2: "inner_solid"}

EDGE_BAND = 14      # 경계에서 이 거리 [px] 안의 흰색 = 실선 (선 두께 약 6~8 px)
CENTER_BAND = 0.08  # |f - 0.5| < 이 값이면 중앙 후보
DASH_MAX_HALF = 6   # 점선 덩어리의 최대 반두께 [px] (거리변환 최댓값)
DASH_MAX_FSPAN = 0.15
DASH_MAX_AREA = 900 # 점선 조각 최대 넓이 [px] (조각 약 6 x 55 px, 횡단보도 줄무늬는 1000 px 이상)


def main():
    bgr = cv2.imread(str(MAP))
    h, w = bgr.shape[:2]
    b, g, r = [bgr[:, :, i].astype(int) for i in range(3)]
    mx, mn = np.maximum(np.maximum(r, g), b), np.minimum(np.minimum(r, g), b)

    asphalt = (mx < 100) & (mx - mn < 15)
    white = mn > 200

    # 1) 도로 고리: 아스팔트를 메우고 가장 큰 덩어리
    closed = cv2.morphologyEx(asphalt.astype(np.uint8), cv2.MORPH_CLOSE,
                              cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (25, 25)))
    n, lab, stats, _ = cv2.connectedComponentsWithStats(closed)
    ring = (lab == 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])).astype(np.uint8)

    contours, hier = cv2.findContours(ring, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    outer_idx = max(range(len(contours)), key=lambda i: cv2.contourArea(contours[i]) if hier[0][i][3] < 0 else -1)
    holes = [i for i in range(len(contours)) if hier[0][i][3] == outer_idx]
    inner_idx = max(holes, key=lambda i: cv2.contourArea(contours[i]))

    outer_fill = np.zeros((h, w), np.uint8); cv2.drawContours(outer_fill, contours, outer_idx, 1, -1)
    inner_fill = np.zeros((h, w), np.uint8); cv2.drawContours(inner_fill, contours, inner_idx, 1, -1)
    road = outer_fill & (1 - inner_fill)

    # 경계까지 거리 (경계 안팎 모두: 안쪽 픽셀은 앞 항, 바깥 픽셀은 뒤 항만 0이 아님)
    def dist_to_boundary(fill):
        return (cv2.distanceTransform(fill, cv2.DIST_L2, 5)
                + cv2.distanceTransform(1 - fill, cv2.DIST_L2, 5))

    d_outer_b = dist_to_boundary(outer_fill)
    d_inner_b = dist_to_boundary(inner_fill)

    # 실선은 아스팔트 밖(바깥 실선: 잔디 쪽, 안쪽 실선: 회색 섬 쪽)에 있음.
    # 도로 안의 횡단보도 줄무늬·정지선 끝은 고리 안이라 여기서 빠짐
    label = np.full((h, w), 255, np.uint8)   # 255 = 배경
    label[white & (outer_fill == 0) & (d_outer_b <= EDGE_BAND)] = 0
    label[white & (inner_fill == 1) & (d_inner_b <= EDGE_BAND)] = 2

    # 2) 중앙 점선
    d_out = cv2.distanceTransform(outer_fill, cv2.DIST_L2, 5)        # 바깥 경계까지 (안쪽에서)
    d_in = cv2.distanceTransform(1 - inner_fill, cv2.DIST_L2, 5)     # 안쪽 섬까지
    f = d_out / np.maximum(d_out + d_in, 1e-6)
    cand = (white & (road > 0) & (d_outer_b > EDGE_BAND) & (d_inner_b > EDGE_BAND)).astype(np.uint8)
    n, lab, stats, _ = cv2.connectedComponentsWithStats(cand)
    half = cv2.distanceTransform(cand, cv2.DIST_L2, 3)
    kept, areas = 0, []
    for i in range(1, n):
        m = lab == i
        if not (30 <= stats[i, cv2.CC_STAT_AREA] <= DASH_MAX_AREA):   # 횡단보도 줄무늬는 점선 조각보다 훨씬 큼
            continue
        fs = f[m]
        if abs(np.median(fs) - 0.5) > CENTER_BAND or fs.max() - fs.min() > DASH_MAX_FSPAN:
            continue
        if half[m].max() > DASH_MAX_HALF:
            continue
        label[m] = 1
        kept += 1
        areas.append(int(stats[i, cv2.CC_STAT_AREA]))

    out = np.zeros((h, w, 3), np.uint8)
    for c, rgb in COLORS_RGB.items():
        out[label == c] = rgb[::-1]   # cv2는 BGR로 저장
    cv2.imwrite(str(HERE / "lane_label_texture.png"), out)

    check = bgr.copy()
    sel = label != 255
    check[sel] = (0.2 * check[sel] + 0.8 * out[sel]).astype(np.uint8)
    cv2.imwrite(str(HERE / "lane_label_check.png"), check)

    for c in NAMES:
        print(f"{NAMES[c]:14s} {int((label == c).sum()):7d} px")
    print(f"center dashes kept: {kept}, area min/median/max = {min(areas)}/{int(np.median(areas))}/{max(areas)} px")
    print(f"saved: {HERE / 'lane_label_texture.png'}")


if __name__ == "__main__":
    main()
