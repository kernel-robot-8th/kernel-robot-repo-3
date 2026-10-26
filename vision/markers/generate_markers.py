"""인쇄용 ArUco 마커를 만든다.

실행: cd vision && python3 markers/generate_markers.py
결과:
- markers/out/marker_<id>.png  화면 확인용 (흰 여백 포함)
- markers/out/markers_a4.pdf   인쇄용. 검은 사각형 한 변이 MARKER_MM이 되도록 A4에 배치
인쇄할 때 "실제 크기(100%)"로 출력하고, 검은 사각형 한 변을 자로 재서 기록한다.
"""
import sys
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from vision_omx.detector import MarkerDetector  # noqa: E402

# 0~9 바닥 기준 마커, 10·11 물체 (ID 규칙은 PLAN.md)
IDS = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]
# Pinky 윗면용. 20 = robot1, 21 = robot2. 방향을 읽어야 하므로 앞쪽 화살표와 자를 선을 넣는다
PINKY_IDS = [20, 21]
PINKY_MARKER_MM = 70
SIZE_PX = 400
BORDER_PX = 60

DPI = 300
A4_MM = (210, 297)
MARKER_MM = 50       # 검은 사각형 한 변
CELL_MM = 80         # 마커 하나가 차지하는 칸 (여백·라벨 포함)
MARGIN_MM = 15


def mm_to_px(mm):
    return int(round(mm * DPI / 25.4))


def build_a4_sheets(detector):
    """마커를 A4 페이지에 격자로 배치한다. 페이지가 넘치면 다음 장으로."""
    page_w, page_h = mm_to_px(A4_MM[0]), mm_to_px(A4_MM[1])
    cell, margin, marker_px = mm_to_px(CELL_MM), mm_to_px(MARGIN_MM), mm_to_px(MARKER_MM)
    cols = (page_w - 2 * margin) // cell
    rows = (page_h - 2 * margin) // cell
    per_page = cols * rows

    pages = []
    for start in range(0, len(IDS), per_page):
        page = np.full((page_h, page_w), 255, dtype=np.uint8)
        for i, marker_id in enumerate(IDS[start:start + per_page]):
            r, c = divmod(i, cols)
            x0 = margin + c * cell + (cell - marker_px) // 2
            y0 = margin + r * cell + (cell - marker_px) // 2
            page[y0:y0 + marker_px, x0:x0 + marker_px] = detector.render(marker_id, marker_px)
            cv2.putText(page, f'ID {marker_id}  ({MARKER_MM} mm)', (x0, y0 + marker_px + mm_to_px(6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, 0, 2)
        pages.append(Image.fromarray(page))
    return pages


def build_pinky_sheet(detector):
    """Pinky 윗면용 A4 한 장. 마커 위쪽이 Pinky 정면을 향하게 붙인다."""
    page_w, page_h = mm_to_px(A4_MM[0]), mm_to_px(A4_MM[1])
    page = np.full((page_h, page_w), 255, dtype=np.uint8)
    marker_px = mm_to_px(PINKY_MARKER_MM)
    quiet = mm_to_px(10)        # 마커 둘레 흰 여백. 화살표·글자는 이 밖에 둔다
    arrow = mm_to_px(15)
    block = marker_px + 2 * quiet
    x0 = (page_w - block) // 2
    for i, marker_id in enumerate(PINKY_IDS):
        y0 = mm_to_px(MARGIN_MM) + arrow + i * (block + arrow + mm_to_px(25))
        mx, my = x0 + quiet, y0 + quiet
        page[my:my + marker_px, mx:mx + marker_px] = detector.render(marker_id, marker_px)
        cv2.rectangle(page, (x0, y0), (x0 + block, y0 + block), 0, 2)    # 자를 선
        cx = x0 + block // 2
        cv2.arrowedLine(page, (cx, y0 - mm_to_px(2)), (cx, y0 - arrow), 0, 6, tipLength=0.4)
        cv2.putText(page, 'FRONT', (cx + mm_to_px(4), y0 - mm_to_px(5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.4, 0, 3)
        cv2.putText(page, f'ID {marker_id}  robot{marker_id - 19}  ({PINKY_MARKER_MM} mm)',
                    (x0, y0 + block + mm_to_px(8)), cv2.FONT_HERSHEY_SIMPLEX, 1.2, 0, 3)
    return Image.fromarray(page)


def main():
    out = Path(__file__).resolve().parent / 'out'
    out.mkdir(exist_ok=True)
    detector = MarkerDetector('DICT_4X4_50')
    for marker_id in IDS:
        image = detector.render(marker_id, SIZE_PX)
        image = cv2.copyMakeBorder(image, BORDER_PX, BORDER_PX, BORDER_PX, BORDER_PX,
                                   cv2.BORDER_CONSTANT, value=255)
        cv2.putText(image, f'ID {marker_id}', (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, 0, 2)
        path = out / f'marker_{marker_id}.png'
        cv2.imwrite(str(path), image)
        print(path)

    pages = build_a4_sheets(detector)
    pdf_path = out / 'markers_a4.pdf'
    pages[0].save(pdf_path, resolution=DPI, save_all=True, append_images=pages[1:])
    print(pdf_path)

    pinky_path = out / 'pinky_markers_a4.pdf'
    build_pinky_sheet(detector).save(pinky_path, resolution=DPI)
    print(pinky_path)


if __name__ == '__main__':
    main()
