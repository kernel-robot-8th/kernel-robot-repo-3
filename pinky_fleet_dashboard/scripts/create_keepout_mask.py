#!/usr/bin/env python3
"""Create a Nav2 keepout mask aligned to the Pinky map-proto occupancy map."""

from __future__ import annotations

from pathlib import Path
import re
import sys


# Bounds estimated from the user's yellow rectangles in map-proto map coordinates.
KEEP_OUT_RECTS_M = (
    (-0.20, 1.80, 1.86, 2.19),  # upper room band
    (-0.20, 1.36, 0.89, 1.00),  # cross-room strip
)


def _pgm_tokens(data: bytes) -> tuple[list[bytes], int]:
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
    """Read dimensions from an ASCII or binary Netpbm grayscale image."""
    tokens, _ = _pgm_tokens(path.read_bytes())
    magic = tokens[0]
    if magic not in (b"P2", b"P5"):
        raise ValueError(f"Expected grayscale PGM P2/P5, got {magic.decode(errors='replace')}")
    width, height, maximum = (int(token) for token in tokens[1:])
    if width <= 0 or height <= 0 or not 0 < maximum <= 255:
        raise ValueError("Unsupported PGM dimensions or grayscale range")
    return width, height


def _map_metadata(path: Path) -> tuple[float, float, float, float]:
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
    """Write a trinary mask with the marked rectangles encoded as lethal cells."""
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
            )
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
    root = Path(__file__).resolve().parents[1]
    map_yaml = Path(sys.argv[1]).expanduser() if len(sys.argv) > 1 else Path.home() / "Documents/Pinky_backup/map-proto.yaml"
    map_pgm = map_yaml.parent / "map-proto.pgm"
    output_dir = root / "config/keepout"
    count = create_mask(map_yaml, map_pgm, output_dir / "keepout_mask.pgm", output_dir / "keepout_mask.yaml")
    print(f"Wrote {output_dir / 'keepout_mask.pgm'}; blocked cells: {count}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
