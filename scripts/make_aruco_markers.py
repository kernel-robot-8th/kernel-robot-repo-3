"""핑키 ArUco 마커를 실제 크기로 인쇄할 종이를 만든다 (A4 PDF + 마커마다 PNG 한 장).

마커 종류와 크기는 주차 판정에 등록한 값과 같아야 한다: DICT_4X4_50, 한 변 70 mm
(.control-station/parking-spots/*.json), 그리고 핑키마다 붙이는 마커 번호(config/pinky-markers.json).
종이에는 50 mm 잣대가 들어 있다. 100 %("실제 크기")로 인쇄하고 오리기 전에 잣대를 자로 재 본다.

    .venv/bin/python scripts/make_aruco_markers.py            # config/pinky-markers.json의 마커
    .venv/bin/python scripts/make_aruco_markers.py --ids 1 23 # 원하는 번호의 마커

파일(기본 docs/markers/)을 쓸 뿐이며 카메라·로봇과는 통신하지 않는다.
첫 줄은 argparse 설명(__doc__.splitlines()[0])으로 쓰이므로 한 줄 요약으로 둔다.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
# 인쇄 해상도. mm 단위 치수를 이 DPI의 픽셀로 바꿔 그린다.
DPI = 600
A4_MM = (210.0, 297.0)
QUIET_MM = 10.0  # 오릴 때 검은 사각형 둘레에 남기는 흰 여백. 검출기가 마커 경계를 찾으려면 필요하다
FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
FONT_REGULAR = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"


def px(mm: float) -> int:
    """mm를 DPI 기준 픽셀 수로 바꾼다 (1 inch = 25.4 mm)."""
    return round(mm / 25.4 * DPI)


def font(size_mm: float, bold: bool = True) -> ImageFont.FreeTypeFont:
    """글자 높이 size_mm의 Noto CJK 글꼴. 글꼴 파일이 없으면 기본 글꼴(한글이 깨질 수 있음)로 대신한다."""
    path = FONT if bold else FONT_REGULAR
    try:
        return ImageFont.truetype(path, px(size_mm), index=1)  # index 1 = Noto CJK 모음 파일 안의 한국어 글꼴
    except OSError:
        return ImageFont.load_default()


def marker_image(marker_id: int, side_mm: float) -> Image.Image:
    """검은 테두리 1칸을 포함해 한 변이 side_mm인 DICT_4X4_50 마커 그림을 만든다."""
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    square = cv2.aruco.generateImageMarker(dictionary, marker_id, px(side_mm), borderBits=1)
    return Image.fromarray(square).convert("L")


def pinky_markers() -> list[tuple[int, str, str]]:
    """config/pinky-markers.json에서 (마커 번호, 핑키 이름, 자리 ID)를 P0부터 순서대로 읽는다."""
    names = {"10": "P0", "11": "P1", "12": "P2"}
    document = json.loads((ROOT / "config" / "pinky-markers.json").read_text(encoding="utf-8"))
    return [(int(entry["marker_id"]), names.get(pinky_id, pinky_id), entry["spot_id"])
            for pinky_id, entry in sorted(document["pinkies"].items())]


def draw_sheet(markers: list[tuple[int, str, str]], side_mm: float) -> Image.Image:
    """A4 한 장에 안내 문구, 마커(최대 4칸, 실제로는 3개), 점선 자르는 선을 그리고 남는 칸에 잣대를 넣는다."""
    page = Image.new("L", (px(A4_MM[0]), px(A4_MM[1])), 255)
    draw = ImageDraw.Draw(page)
    draw.text((px(15), px(12)), f"핑키 ArUco 마커 · DICT_4X4_50 · 한 변 {side_mm:.0f} mm", fill=0, font=font(6))
    for line, text in enumerate((
        "실제 크기(배율 100%)로 인쇄하고, 오른쪽 아래 잣대가 정확히 50 mm인지 자로 확인하세요.",
        "점선을 따라 오려 흰 테두리를 남기세요. 마커 면이 위를 보게, 마커의 왼쪽 변이 핑키 앞쪽을 향하게 붙입니다.",
        "붙인 뒤 핑키마다 5번 정차 위치를 등록합니다(docs/implementation/runbook.md · 정답 위치 등록).",
    )):
        draw.text((px(15), px(22 + line * 6)), text, fill=60, font=font(3.4, bold=False))

    block = side_mm + 2 * QUIET_MM
    slots = [(15.0, 46.0), (15.0 + block + 10.0, 46.0), (15.0, 46.0 + block + 22.0), (15.0 + block + 10.0, 46.0 + block + 22.0)]
    for (marker_id, name, spot_id), (x_mm, y_mm) in zip(markers, slots, strict=False):
        page.paste(marker_image(marker_id, side_mm), (px(x_mm + QUIET_MM), px(y_mm + QUIET_MM)))
        dashed_square(draw, x_mm, y_mm, block)
        draw.text((px(x_mm), px(y_mm + block + 2)), f"{name} · {marker_id}번", fill=0, font=font(6))
        draw.text((px(x_mm + 36), px(y_mm + block + 3.6)), f"{spot_id}  ◀ 왼쪽 변이 핑키 앞쪽", fill=90, font=font(3, bold=False))
    if len(markers) < len(slots):
        ruler(draw, *slots[len(markers)])
    return page


def dashed_square(draw: ImageDraw.ImageDraw, x_mm: float, y_mm: float, size_mm: float) -> None:
    """오릴 자리를 3 mm 선, 2 mm 간격의 점선 사각형으로 그린다."""
    left, top, right, bottom = px(x_mm), px(y_mm), px(x_mm + size_mm), px(y_mm + size_mm)
    dash, gap = px(3), px(2)
    for start in range(left, right, dash + gap):
        draw.line([(start, top), (min(start + dash, right), top)], fill=150, width=3)
        draw.line([(start, bottom), (min(start + dash, right), bottom)], fill=150, width=3)
    for start in range(top, bottom, dash + gap):
        draw.line([(left, start), (left, min(start + dash, bottom))], fill=150, width=3)
        draw.line([(right, start), (right, min(start + dash, bottom))], fill=150, width=3)


def ruler(draw: ImageDraw.ImageDraw, x_mm: float, y_mm: float) -> None:
    """인쇄 배율을 확인하는 50 mm 막대와 10 mm 눈금을 그린다."""
    y = px(y_mm + 30)
    draw.line([(px(x_mm), y), (px(x_mm + 50), y)], fill=0, width=6)
    for tick in range(0, 51, 10):
        draw.line([(px(x_mm + tick), y - px(3 if tick % 50 else 5)), (px(x_mm + tick), y)], fill=0, width=5)
    draw.text((px(x_mm), y + px(3)), "잣대 50 mm", fill=0, font=font(4.5))
    draw.text((px(x_mm), y + px(10)), "자로 재서 50 mm가 아니면 배율을 고쳐 다시 인쇄", fill=90, font=font(3, bold=False))


def main() -> None:
    """A4 PDF 한 장과 마커별 PNG를 --out 폴더에 저장한다."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--ids", type=int, nargs="*", help="marker numbers (default: config/pinky-markers.json)")
    parser.add_argument("--side-mm", type=float, default=70.0)
    parser.add_argument("--out", type=Path, default=ROOT / "docs" / "markers")
    args = parser.parse_args()
    markers = [(marker_id, f"마커 {marker_id}", "") for marker_id in args.ids] if args.ids else pinky_markers()
    args.out.mkdir(parents=True, exist_ok=True)
    sheet = draw_sheet(markers[:3], args.side_mm)
    sheet_path = args.out / "pinky-aruco-markers-a4.pdf"
    sheet.convert("RGB").save(sheet_path, "PDF", resolution=float(DPI))
    print(f"A4 인쇄 파일: {sheet_path}")
    for marker_id, _, _ in markers:
        single = Image.new("L", (px(args.side_mm + 2 * QUIET_MM),) * 2, 255)
        single.paste(marker_image(marker_id, args.side_mm), (px(QUIET_MM), px(QUIET_MM)))
        path = args.out / f"aruco-4x4_50-id{marker_id:02d}-{args.side_mm:.0f}mm.png"
        single.save(path, dpi=(DPI, DPI))
        print(f"마커 {marker_id}번 PNG({DPI} dpi): {path}")


if __name__ == "__main__":
    main()
