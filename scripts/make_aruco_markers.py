"""Print sheet of the Pinky ArUco markers at true size (A4 PDF + one PNG per marker).

The markers and their size must match what the parking judgment was registered with: DICT_4X4_50, 70 mm side
(.control-station/parking-spots/*.json), and the marker numbers each Pinky carries (config/pinky-markers.json).
The sheet carries a 50 mm ruler: print at 100 % ("actual size") and measure it before cutting.

    .venv/bin/python scripts/make_aruco_markers.py            # markers from config/pinky-markers.json
    .venv/bin/python scripts/make_aruco_markers.py --ids 1 23 # any markers
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
DPI = 600
A4_MM = (210.0, 297.0)
QUIET_MM = 10.0  # white margin kept around the black square when cutting; the detector needs it
FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
FONT_REGULAR = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"


def px(mm: float) -> int:
    return round(mm / 25.4 * DPI)


def font(size_mm: float, bold: bool = True) -> ImageFont.FreeTypeFont:
    path = FONT if bold else FONT_REGULAR
    try:
        return ImageFont.truetype(path, px(size_mm), index=1)  # index 1 = Korean in the Noto CJK collection
    except OSError:
        return ImageFont.load_default()


def marker_image(marker_id: int, side_mm: float) -> Image.Image:
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    square = cv2.aruco.generateImageMarker(dictionary, marker_id, px(side_mm), borderBits=1)
    return Image.fromarray(square).convert("L")


def pinky_markers() -> list[tuple[int, str, str]]:
    """(marker id, Pinky name, spot id) from config/pinky-markers.json, P0 first."""
    names = {"10": "P0", "11": "P1", "12": "P2"}
    document = json.loads((ROOT / "config" / "pinky-markers.json").read_text(encoding="utf-8"))
    return [(int(entry["marker_id"]), names.get(pinky_id, pinky_id), entry["spot_id"])
            for pinky_id, entry in sorted(document["pinkies"].items())]


def draw_sheet(markers: list[tuple[int, str, str]], side_mm: float) -> Image.Image:
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
    left, top, right, bottom = px(x_mm), px(y_mm), px(x_mm + size_mm), px(y_mm + size_mm)
    dash, gap = px(3), px(2)
    for start in range(left, right, dash + gap):
        draw.line([(start, top), (min(start + dash, right), top)], fill=150, width=3)
        draw.line([(start, bottom), (min(start + dash, right), bottom)], fill=150, width=3)
    for start in range(top, bottom, dash + gap):
        draw.line([(left, start), (left, min(start + dash, bottom))], fill=150, width=3)
        draw.line([(right, start), (right, min(start + dash, bottom))], fill=150, width=3)


def ruler(draw: ImageDraw.ImageDraw, x_mm: float, y_mm: float) -> None:
    """A 50 mm bar with 10 mm ticks to check the print scale."""
    y = px(y_mm + 30)
    draw.line([(px(x_mm), y), (px(x_mm + 50), y)], fill=0, width=6)
    for tick in range(0, 51, 10):
        draw.line([(px(x_mm + tick), y - px(3 if tick % 50 else 5)), (px(x_mm + tick), y)], fill=0, width=5)
    draw.text((px(x_mm), y + px(3)), "잣대 50 mm", fill=0, font=font(4.5))
    draw.text((px(x_mm), y + px(10)), "자로 재서 50 mm가 아니면 배율을 고쳐 다시 인쇄", fill=90, font=font(3, bold=False))


def main() -> None:
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
