#!/usr/bin/env python3
"""Derive the shelf slot map (config/shelf-slots.json) from the team's Blender glTF shelf models.

Each model holds one shelf node (e.g. `shelf_a`) with a mesh and child empties named
`slot_<shelf>_<level>_<index>` placed on the deck top surface. This script reads those empties and
the mesh's horizontal faces, and expresses every slot in one shelf frame shared by both shelves:

  origin  bottom of the seam between the two shelves, on their front plane
  x       into the shelf (away from the front camera)
  y       left as seen from the front
  z       up (heights are from the shelf base, not the floor or table)

Values come from the model only. They are not verified against the printed shelves.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import struct
import sys
from pathlib import Path
from typing import Any

import numpy as np


_SLOT_NAME = re.compile(r"^slot_([a-z])_(\d+)_(\d+)$")
_COMPONENTS = {5126: np.float32, 5123: np.uint16, 5125: np.uint32, 5121: np.uint8}
_WIDTHS = {"SCALAR": 1, "VEC2": 2, "VEC3": 3}
_UP = np.array([0.0, 1.0, 0.0])
MAX_MODEL_BYTES = 16 * 1024 * 1024


def _load_glb(path: Path) -> tuple[dict[str, Any], bytes, str]:
    raw = path.read_bytes()
    if len(raw) > MAX_MODEL_BYTES:
        raise ValueError(f"{path} is larger than {MAX_MODEL_BYTES} bytes")
    magic, version, length = struct.unpack_from("<4sII", raw, 0)
    if magic != b"glTF" or version != 2 or length != len(raw):
        raise ValueError(f"{path} is not a glTF 2.0 binary file")
    json_length, json_type = struct.unpack_from("<I4s", raw, 12)
    if json_type != b"JSON":
        raise ValueError(f"{path} has no JSON chunk")
    document = json.loads(raw[20 : 20 + json_length])
    offset = 20 + json_length
    bin_length, bin_type = struct.unpack_from("<I4s", raw, offset)
    if bin_type != b"BIN\x00":
        raise ValueError(f"{path} has no binary chunk")
    return document, raw[offset + 8 : offset + 8 + bin_length], hashlib.sha256(raw).hexdigest()


def _accessor(document: dict[str, Any], binary: bytes, index: int) -> np.ndarray:
    accessor = document["accessors"][index]
    view = document["bufferViews"][accessor["bufferView"]]
    if view.get("byteStride"):
        raise ValueError("interleaved buffer views are not supported")
    width = _WIDTHS[accessor["type"]]
    start = view.get("byteOffset", 0) + accessor.get("byteOffset", 0)
    values = np.frombuffer(
        binary, _COMPONENTS[accessor["componentType"]], accessor["count"] * width, start
    )
    return values.reshape(accessor["count"], width) if width > 1 else values


def _rotation(quaternion: list[float]) -> np.ndarray:
    x, y, z, w = quaternion
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _horizontal_faces(triangles: np.ndarray, facing_up: bool) -> dict[float, dict[str, Any]]:
    """Group horizontal triangles by height (mm resolution): area and x/z extent per height."""
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    areas = np.linalg.norm(normals, axis=1) / 2
    units = normals / np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)
    mask = units[:, 1] > 0.98 if facing_up else units[:, 1] < -0.98
    groups: dict[float, dict[str, Any]] = {}
    for triangle, area in zip(triangles[mask], areas[mask]):
        key = round(float(triangle[:, 1].mean()), 3)
        group = groups.setdefault(key, {"area": 0.0, "x": [], "z": []})
        group["area"] += float(area)
        group["x"].extend(triangle[:, 0])
        group["z"].extend(triangle[:, 2])
    return groups


def _side_profile(positions: np.ndarray) -> dict[str, float]:
    """Front corners of the stepped side frame (local z = front, y = up), read from its outer face."""
    outer = positions[np.abs(np.abs(positions[:, 0]) - np.abs(positions[:, 0]).max()) < 1e-4]
    low_front = float(outer[outer[:, 1] < 0.005][:, 2].max())
    step_height = float(outer[outer[:, 2] > low_front - 1e-4][:, 1].max())
    high_front = float(outer[outer[:, 1] > step_height + 1e-3][:, 2].max())
    return {
        "half_width": float(np.abs(positions[:, 0]).max()),
        "low_front_z": low_front,
        "step_height": step_height,
        "high_front_z": high_front,
        "top_height": float(outer[:, 1].max()),
    }


def read_shelf(path: Path) -> dict[str, Any]:
    document, binary, sha256 = _load_glb(path)
    nodes = document["nodes"]
    shelves = [node for node in nodes if "mesh" in node and node.get("name", "").startswith("shelf_")]
    if len(shelves) != 1:
        raise ValueError(f"{path} must contain exactly one shelf_* mesh node")
    shelf = shelves[0]
    if "scale" in shelf or "matrix" in shelf:
        raise ValueError(f"{path}: scaled or matrix shelf nodes are not supported")
    shelf_letter = shelf["name"].removeprefix("shelf_")
    rotation = _rotation(shelf.get("rotation", [0, 0, 0, 1]))
    translation = np.array(shelf.get("translation", [0, 0, 0]), dtype=float)
    if not np.allclose(rotation @ _UP, _UP, atol=1e-6):
        raise ValueError(f"{path}: the shelf must stand upright (rotation about the vertical axis only)")

    primitive = document["meshes"][shelf["mesh"]]["primitives"][0]
    positions = _accessor(document, binary, primitive["attributes"]["POSITION"]).astype(float)
    triangles = positions[_accessor(document, binary, primitive["indices"]).astype(int).reshape(-1, 3)]
    up_faces = _horizontal_faces(triangles, facing_up=True)
    down_faces = _horizontal_faces(triangles, facing_up=False)

    slots = []
    for child_index in shelf.get("children", []):
        child = nodes[child_index]
        match = _SLOT_NAME.fullmatch(child.get("name", ""))
        if not match or match.group(1) != shelf_letter:
            raise ValueError(f"{path}: unexpected child node {child.get('name')!r}")
        marker = np.array(child["translation"], dtype=float)
        deck = up_faces.get(round(float(marker[1]), 3))
        if deck is None:
            raise ValueError(f"{path}: {child['name']} is not on an upward deck surface")
        # The lowest downward surface above the deck that covers most of it bounds object height.
        ceilings = [
            height
            for height, group in down_faces.items()
            if height > marker[1] and group["area"] >= 0.5 * deck["area"]
        ]
        # The deck's front lip: the upward face just above the deck that reaches the deck front edge,
        # and the deck underside just below. These edges are what the front camera actually sees.
        deck_front = max(deck["z"])
        deck_height = round(float(marker[1]), 3)  # the grouping key of the deck surface itself
        lip_tops = [
            height
            for height, group in up_faces.items()
            if deck_height + 0.001 < height <= deck_height + 0.03 and max(group["z"]) >= deck_front - 0.001
        ]
        lip_bottoms = [height for height in down_faces if marker[1] - 0.01 <= height < marker[1]]
        if not lip_tops or not lip_bottoms:
            raise ValueError(f"{path}: no front lip found for {child['name']}")
        slots.append(
            {
                "slot_id": child["name"],
                "marker_local": marker,
                "deck_x": (min(deck["x"]), max(deck["x"])),
                "deck_z": (min(deck["z"]), deck_front),
                "ceiling_y": min(ceilings) if ceilings else None,
                "lip_top_y": min(lip_tops),
                "lip_front_z": max(up_faces[min(lip_tops)]["z"]),
                "lip_bottom_y": max(lip_bottoms),
            }
        )
    if not slots:
        raise ValueError(f"{path}: no slot_* empties under {shelf['name']}")
    return {
        "side_profile": _side_profile(positions),
        "file": path.name,
        "sha256": sha256,
        "generator": document.get("asset", {}).get("generator"),
        "shelf_id": shelf["name"],
        "rotation": rotation,
        "translation": translation,
        "local_min": positions.min(axis=0),
        "local_max": positions.max(axis=0),
        "slots": slots,
    }


def build_slot_map(
    shelves: list[dict[str, Any]],
    inactive: set[str],
    lower_deck_shift: float = 0.0,
    adjustment_note: str | None = None,
) -> dict[str, Any]:
    """Build the slot map. `lower_deck_shift` (m) moves the lower deck and its lip to a measured height."""
    if len(shelves) != 2:
        raise ValueError("exactly two shelf models are expected")
    fronts = [shelf["rotation"] @ np.array([0.0, 0.0, 1.0]) for shelf in shelves]
    if not np.allclose(fronts[0], fronts[1], atol=1e-6):
        raise ValueError("both shelves must face the same direction")
    front = fronts[0]
    left = np.cross(front, _UP)
    widths = [float(shelf["local_max"][0] - shelf["local_min"][0]) for shelf in shelves]
    gap = abs(float(np.dot(shelves[0]["translation"] - shelves[1]["translation"], left)))
    if abs(gap - (widths[0] + widths[1]) / 2) > 0.001:
        raise ValueError(f"shelves are not side by side (center gap {gap:.4f} m)")
    front_extent = float(shelves[0]["local_max"][2])
    origin = (shelves[0]["translation"] + shelves[1]["translation"]) / 2 + front * front_extent
    origin[1] = 0.0

    def to_frame(point_world: np.ndarray) -> dict[str, float]:
        offset = point_world - origin
        return {
            "x": round(float(np.dot(offset, -front)), 4),
            "y": round(float(np.dot(offset, left)), 4),
            "z": round(float(offset[1]), 4),
        }

    shelf_entries = []
    slot_entries = []
    lips: dict[tuple[str, str], dict[str, float]] = {}
    for shelf in sorted(shelves, key=lambda item: -float(np.dot(item["translation"], left))):
        side = "LEFT" if float(np.dot(shelf["translation"] - origin, left)) > 0 else "RIGHT"
        size = shelf["local_max"] - shelf["local_min"]
        shelf_entries.append(
            {
                "shelf_id": shelf["shelf_id"],
                "side": side,
                "width_m": round(float(size[0]), 4),
                "height_m": round(float(size[1]), 4),
                "depth_m": round(float(size[2]), 4),
            }
        )
        markers = [slot["marker_local"][1] for slot in shelf["slots"]]
        for slot in shelf["slots"]:
            to_world = lambda local: shelf["translation"] + shelf["rotation"] @ np.asarray(local, dtype=float)
            marker = to_frame(to_world(slot["marker_local"]))
            # Split the deck footprint at the shelf center line into this slot's half.
            corners = [
                to_frame(to_world([x, slot["marker_local"][1], z]))
                for x in (slot["deck_x"][0], 0.0, slot["deck_x"][1])
                for z in slot["deck_z"]
            ]
            center_y = to_frame(to_world([0.0, 0.0, 0.0]))["y"]
            own = [corner for corner in corners if (corner["y"] - center_y) * (marker["y"] - center_y) >= 0]
            level = "UPPER" if slot["marker_local"][1] == max(markers) else "LOWER"
            slot_side = "LEFT" if marker["y"] > center_y else "RIGHT"
            shift = lower_deck_shift if level == "LOWER" else 0.0
            marker["z"] = round(marker["z"] + shift, 4)
            lips[(shelf["shelf_id"], level)] = {
                "x": round(front_extent - slot["lip_front_z"], 4),
                "top": round(slot["lip_top_y"] + shift, 4),
                "bottom": round(slot["lip_bottom_y"] + shift, 4),
            }
            slot_entries.append(
                {
                    "slot_id": slot["slot_id"],
                    "shelf_id": shelf["shelf_id"],
                    "level": level,
                    "side": slot_side,
                    "label": f"{'왼쪽' if side == 'LEFT' else '오른쪽'} 선반 "
                    f"{'위' if level == 'UPPER' else '아래'} 단 {'좌' if slot_side == 'LEFT' else '우'}",
                    "active": slot["slot_id"] not in inactive,
                    "marker_m": marker,
                    "footprint_m": {
                        "x_min": min(corner["x"] for corner in own),
                        "x_max": max(corner["x"] for corner in own),
                        "y_min": min(corner["y"] for corner in own),
                        "y_max": max(corner["y"] for corner in own),
                    },
                    "deck_height_m": marker["z"],
                    "lip_top_height_m": lips[(shelf["shelf_id"], level)]["top"],
                    "clear_height_m": (
                        round(float(slot["ceiling_y"] - slot["marker_local"][1] - shift), 4)
                        if slot["ceiling_y"] is not None
                        else None
                    ),
                }
            )
    # Side-frame front corners the operator can click in the front camera image (for solvePnP).
    keypoints = []
    profile = shelves[0]["side_profile"]
    if shelves[1]["side_profile"] != profile:
        raise ValueError("both shelves must share the same side-frame profile")
    for place, label, offset_y in (
        ("left_end", "왼쪽 끝 옆판", 1.0),
        ("seam", "가운데 이음새", 0.0),
        ("right_end", "오른쪽 끝 옆판", -1.0),
    ):
        y = offset_y * 2 * profile["half_width"]
        # The side-frame step is left out: it lines up with the lower lip only in the model.
        for corner, corner_label, local_z, height in (
            ("bottom_front", "아래 앞 모서리", profile["low_front_z"], 0.0),
            ("top_front", "위 앞 모서리", profile["high_front_z"], profile["top_height"]),
        ):
            keypoints.append(
                {
                    "keypoint_id": f"{place}_{corner}",
                    "label": f"{label} {corner_label}",
                    "point_m": {
                        "x": round(front_extent - local_z, 4),
                        "y": round(y, 4),
                        "z": round(height, 4),
                    },
                }
            )
    # Corners of the front lip of each deck with a used slot. The deck surface itself hides behind the
    # lip, so the lip's top and bottom edges are the lines the front camera shows.
    for shelf in shelf_entries:
        for level, level_label in (("UPPER", "위"), ("LOWER", "아래")):
            deck = [
                slot for slot in slot_entries
                if slot["shelf_id"] == shelf["shelf_id"] and slot["level"] == level
            ]
            if not any(slot["active"] for slot in deck):
                continue
            lip = lips[(shelf["shelf_id"], level)]
            for edge, edge_label in (("top", "위"), ("bottom", "아래")):
                for corner, corner_label, y in (
                    ("left", "왼쪽", max(slot["footprint_m"]["y_max"] for slot in deck)),
                    ("right", "오른쪽", min(slot["footprint_m"]["y_min"] for slot in deck)),
                ):
                    keypoints.append(
                        {
                            "keypoint_id": f"{shelf['shelf_id']}_{level.lower()}_lip_{edge}_{corner}",
                            "label": f"{'왼쪽' if shelf['side'] == 'LEFT' else '오른쪽'} 선반 "
                            f"{level_label} 단 앞 턱 {edge_label} {corner_label} 모서리",
                            "point_m": {"x": lip["x"], "y": y, "z": lip[edge]},
                        }
                    )
    unknown = inactive - {slot["slot_id"] for slot in slot_entries}
    if unknown:
        raise ValueError("unknown --inactive slots: " + ", ".join(sorted(unknown)))
    slot_entries.sort(key=lambda slot: slot["slot_id"])
    adjustments = (
        [{"level": "LOWER", "shift_m": lower_deck_shift, "note": adjustment_note}] if lower_deck_shift else []
    )
    return {
        "schema_version": 1,
        "status": "MODEL_DERIVED_WITH_MEASURED_ADJUSTMENTS" if adjustments else "MODEL_DERIVED_UNVERIFIED",
        "note": "팀 3D 모델에서 계산한 값에 measured_adjustments의 실측 보정을 반영했습니다."
        if adjustments
        else "팀 3D 모델에서 계산한 값입니다. 출력한 실물 치수와 배치는 확인하지 않았습니다.",
        "measured_adjustments": adjustments,
        "frame": {
            "frame_id": "shelf_front",
            "unit": "m",
            "origin": "두 선반이 맞닿는 선의 밑면, 선반 정면 평면 위",
            "x": "선반 안쪽 (정면 카메라에서 멀어지는 방향)",
            "y": "정면에서 볼 때 왼쪽",
            "z": "위 (선반 밑면 기준, 바닥·책상 높이 미포함)",
        },
        "source_models": [
            {"file": shelf["file"], "sha256": shelf["sha256"], "generator": shelf["generator"], "shelf_id": shelf["shelf_id"]}
            for shelf in shelves
        ],
        "shelves": shelf_entries,
        "slots": slot_entries,
        "keypoints": keypoints,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("models", nargs=2, type=Path, help="the two shelf .glb files")
    parser.add_argument("--inactive", default="", help="comma-separated slot IDs that exist but are not used")
    parser.add_argument("--output", type=Path, default=Path("config/shelf-slots.json"))
    parser.add_argument("--force", action="store_true", help="overwrite an existing output file")
    parser.add_argument(
        "--lower-deck-shift",
        type=float,
        default=0.0,
        help="measured offset (m) of the printed lower deck and its lip from the model, e.g. -0.035",
    )
    parser.add_argument("--adjustment-note", help="how and when the lower deck shift was measured")
    args = parser.parse_args()
    if args.output.exists() and not args.force:
        print(f"{args.output} already exists; pass --force to replace it", file=sys.stderr)
        return 1
    if args.lower_deck_shift and not args.adjustment_note:
        print("--lower-deck-shift needs --adjustment-note describing the measurement", file=sys.stderr)
        return 1
    inactive = {item.strip() for item in args.inactive.split(",") if item.strip()}
    try:
        slot_map = build_slot_map(
            [read_shelf(path) for path in args.models], inactive, args.lower_deck_shift, args.adjustment_note
        )
    except (OSError, ValueError, KeyError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(slot_map, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    for slot in slot_map["slots"]:
        marker = slot["marker_m"]
        state = "" if slot["active"] else "  (사용 안 함)"
        print(
            f"{slot['slot_id']}  {slot['label']:14s} x={marker['x']*100:5.1f} y={marker['y']*100:6.1f} "
            f"z={marker['z']*100:5.1f} cm{state}"
        )
    print(f"wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
