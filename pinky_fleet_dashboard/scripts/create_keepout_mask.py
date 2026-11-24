#!/usr/bin/env python3
"""핑키 map-proto 점유 지도에 맞춘 Nav2 keepout(진입 금지) 마스크를 만든다.

실행 위치: 오프라인 도구 (사람이 PC에서 직접 실행: python3 scripts/create_keepout_mask.py [지도 yaml]).
사용처: 결과 config/keepout/keepout_mask.pgm·.yaml은 runtime_bundle이 주행준비 때 로봇에 올리고,
map-proto 지도일 때 Nav2 전역·지역 costmap의 keepout filter가 읽는다. 원본 지도는 바꾸지 않는다.

핵심: 지도와 같은 크기·해상도·원점의 PGM을 만들고, 금지 사각형 안의 칸은 0(검정 = 막힘), 나머지는 255(흰색 =
지나갈 수 있음)로 쓴다. 금지 띠 안의 수령 구역은 작은 원만 열어 둔다. 범위를 바꾸면 다시 실행하고, 로봇 Nav2를
다음에 준비할 때 새 파일이 전송된다(OPERATIONS.md).
"""

from __future__ import annotations

from pathlib import Path
import re
import sys


# 사용자가 노란 사각형으로 표시한 영역을 map-proto 지도 좌표로 어림한 경계.
# 각 항목은 (min_x, max_x, min_y, max_y), 단위 m.
KEEP_OUT_RECTS_M = (
    (-0.20, 1.80, 1.86, 2.19),  # 위쪽 방 띠
    (-0.20, 1.36, 0.89, 1.00),  # 방을 가로지르는 띠
)
# 수령 구역은 위쪽 방 띠 안에 있다(2026-11-16에 웨이포인트 '수령 A' (0.56, 1.93)와 '수령 B' (1.08, 1.94)로 기록).
# 각 지점 둘레의 원만 열어서, 띠의 나머지는 금지로 둔 채 핑키가 들어가 거기 멈출 수 있게 한다.
# (x, y, 반지름), 단위는 지도 m.
OPENINGS_M = (
    (0.56, 1.93, 0.15),  # 수령 A
    (1.08, 1.94, 0.15),  # 수령 B
)


def _pgm_tokens(data: bytes) -> tuple[list[bytes], int]:
    """PGM 헤더의 토큰 4개(magic, 너비, 높이, 최댓값)와 픽셀 데이터 시작 위치를 돌려준다. '#' 주석 줄은 건너뛴다."""
    tokens: list[bytes] = []
    index = 0
    while len(tokens) < 4:
        while index < len(data) and data[index] in b" \t\r\n":
            index += 1
        if index < len(data) and data[index] == ord("#"):
            newline = data.find(b"\n", index)
            if newline < 0:
                raise ValueError("PGM header ends inside a comment")
            index = newline + 1
            continue
        start = index
        while index < len(data) and data[index] not in b" \t\r\n#":
            index += 1
        if start == index:
            raise ValueError("PGM header is incomplete")
        tokens.append(data[start:index])
    while index < len(data) and data[index] in b" \t\r\n":
        index += 1
    return tokens, index


def read_pgm_dimensions(path: Path) -> tuple[int, int]:
    """ASCII(P2) 또는 바이너리(P5) Netpbm 회색조 이미지에서 (너비, 높이)를 읽는다."""
    tokens, _ = _pgm_tokens(path.read_bytes())
    magic = tokens[0]
    if magic not in (b"P2", b"P5"):
        raise ValueError(f"Expected grayscale PGM P2/P5, got {magic.decode(errors='replace')}")
    width, height, maximum = (int(token) for token in tokens[1:])
    if width <= 0 or height <= 0 or not 0 < maximum <= 255:
        raise ValueError("Unsupported PGM dimensions or grayscale range")
    return width, height


def _map_metadata(path: Path) -> tuple[float, float, float, float]:
    """지도 YAML에서 (해상도 m/칸, 원점 x, 원점 y, 원점 yaw)를 정규식으로 읽는다. 회전된 원점은 거부한다."""
    text = path.read_text(encoding="utf-8")
    resolution_match = re.search(r"(?m)^resolution:\s*([0-9.]+)\s*$", text)
    origin_match = re.search(r"(?m)^origin:\s*\[\s*([-0-9.]+)\s*,\s*([-0-9.]+)\s*,\s*([-0-9.]+)\s*\]", text)
    if not resolution_match or not origin_match:
        raise ValueError(f"Could not read resolution/origin from {path}")
    resolution = float(resolution_match.group(1))
    origin_x = float(origin_match.group(1))
    origin_y = float(origin_match.group(2))
    origin_yaw = float(origin_match.group(3))
    if resolution <= 0 or abs(origin_yaw) > 1e-9:
        raise ValueError("Keepout mask requires positive resolution and an unrotated map origin")
    return resolution, origin_x, origin_y, origin_yaw


def create_mask(map_yaml: Path, map_pgm: Path, output_pgm: Path, output_yaml: Path) -> int:
    """표시한 사각형을 치명(lethal) 칸으로 넣은 trinary 마스크를 쓰고, 막힌 칸 수를 돌려준다."""
    # PGM은 맨 윗줄이 지도에서 y가 가장 큰 줄이므로 행 번호를 뒤집어 칸 중심의 지도 좌표를 구한다.
    width, height = read_pgm_dimensions(map_pgm)
    resolution, origin_x, origin_y, _ = _map_metadata(map_yaml)
    pixels: list[str] = []
    blocked_count = 0
    for row in range(height):
        world_y = origin_y + (height - row - 0.5) * resolution
        values: list[str] = []
        for column in range(width):
            world_x = origin_x + (column + 0.5) * resolution
            blocked = any(
                min_x <= world_x <= max_x and min_y <= world_y <= max_y
                for min_x, max_x, min_y, max_y in KEEP_OUT_RECTS_M
            ) and not any((world_x - x) ** 2 + (world_y - y) ** 2 <= r ** 2 for x, y, r in OPENINGS_M)
            values.append("0" if blocked else "255")
            blocked_count += int(blocked)
        pixels.append(" ".join(values))

    output_pgm.parent.mkdir(parents=True, exist_ok=True)
    output_pgm.write_text(
        f"P2\n# Dark cells are keepout; white cells are traversable.\n{width} {height}\n255\n"
        + "\n".join(pixels)
        + "\n",
        encoding="ascii",
    )
    # mode trinary, negate 0: 어두운 칸(0)이 점유(occupied_thresh 0.65 이상)로 읽혀 금지 구역이 된다.
    output_yaml.write_text(
        "image: keepout_mask.pgm\n"
        "mode: trinary\n"
        f"resolution: {resolution:.3f}\n"
        f"origin: [{origin_x:.3f}, {origin_y:.3f}, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n",
        encoding="utf-8",
    )
    return blocked_count


def main() -> int:
    """인자로 받은 지도 YAML(없으면 ~/Documents/Pinky_backup/map-proto.yaml)로 config/keepout/에 마스크를 쓴다."""
    root = Path(__file__).resolve().parents[1]
    map_yaml = Path(sys.argv[1]).expanduser() if len(sys.argv) > 1 else Path.home() / "Documents/Pinky_backup/map-proto.yaml"
    map_pgm = map_yaml.parent / "map-proto.pgm"
    output_dir = root / "config/keepout"
    count = create_mask(map_yaml, map_pgm, output_dir / "keepout_mask.pgm", output_dir / "keepout_mask.yaml")
    print(f"Wrote {output_dir / 'keepout_mask.pgm'}; blocked cells: {count}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
