"""Which arm serves which shelf (config/arms.json).

Two arms stand on either side of the single spot where Pinky stops. An object found on the left shelf is picked by
the left arm and one on the right shelf by the right arm (decided 2026-11-11), so the arm for each ordered unit is
chosen from the shelf where the stock check saw it, not from the object type.

`board_serial` (optional) is the arm's OpenRB-150 board serial. It names the physical arm: the device paths of each
arm (serial port, gripper and overhead cameras) are set per PC in arm_devices.py, and an older single-arm port
setting is matched to its arm by this serial.
"""

from __future__ import annotations

import os
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from control_station.json_files import load_json_file

DEFAULT_ARMS_PATH = Path(__file__).resolve().parents[2] / "config" / "arms.json"
MAX_ARMS_BYTES = 16 * 1024
ARM_ID = re.compile(r"^arm-[0-9]{2}$")
_NAME = re.compile(r"^[A-Za-z0-9_.-]{1,80}$")
_BOARD_SERIAL = re.compile(r"^[0-9A-F]{32}$")
_REQUIRED = {"arm_id", "label", "shelf_id", "stock_camera_id"}


class ArmConfigError(ValueError):
    """The arm file is missing or does not match schema version 1."""


@dataclass(frozen=True)
class ArmRegistry:
    arms: tuple[dict[str, Any], ...]

    @property
    def arm_ids(self) -> frozenset[str]:
        return frozenset(arm["arm_id"] for arm in self.arms)

    def get(self, arm_id: str) -> dict[str, Any] | None:
        return next((arm for arm in self.arms if arm["arm_id"] == arm_id), None)

    def for_shelf(self, shelf_id: str) -> dict[str, Any] | None:
        return next((arm for arm in self.arms if arm["shelf_id"] == shelf_id), None)


def _arm(raw: object) -> dict[str, Any]:
    if not isinstance(raw, dict) or not _REQUIRED <= set(raw) <= _REQUIRED | {"board_serial"}:
        raise ArmConfigError("each arm needs arm_id, label, shelf_id and stock_camera_id (board_serial optional)")
    arm_id = raw["arm_id"]
    if not isinstance(arm_id, str) or not ARM_ID.fullmatch(arm_id):
        raise ArmConfigError("arm_id must look like arm-01")
    label = raw["label"]
    if not isinstance(label, str) or not label.strip() or len(label) > 40:
        raise ArmConfigError(f"{arm_id}.label must be text of 1 to 40 characters")
    for key in ("shelf_id", "stock_camera_id"):
        if not isinstance(raw[key], str) or not _NAME.fullmatch(raw[key]):
            raise ArmConfigError(f"{arm_id}.{key} has an invalid format")
    board_serial = raw.get("board_serial")
    if board_serial is not None and (not isinstance(board_serial, str) or not _BOARD_SERIAL.fullmatch(board_serial)):
        raise ArmConfigError(f"{arm_id}.board_serial must be the 32 hex digits of the OpenRB-150 serial")
    return {"arm_id": arm_id, "label": label.strip(), "shelf_id": raw["shelf_id"],
            "stock_camera_id": raw["stock_camera_id"], "board_serial": board_serial}


def load_arms(path: str | Path | None = None) -> ArmRegistry:
    arms_path = Path(path or os.environ.get("CONTROL_STATION_ARMS") or DEFAULT_ARMS_PATH)
    _, document = load_json_file(arms_path, MAX_ARMS_BYTES, "팔 설정 파일", ArmConfigError)
    if not isinstance(document, dict) or document.get("schema_version") != 1 or isinstance(document.get("schema_version"), bool):
        raise ArmConfigError("팔 설정 파일의 schema_version은 1이어야 합니다")
    if not isinstance(document.get("arms"), list) or not document["arms"]:
        raise ArmConfigError("arms must be a non-empty list")
    arms = tuple(_arm(item) for item in document["arms"])
    for key in ("arm_id", "shelf_id", "board_serial"):
        values = [arm[key] for arm in arms if arm[key] is not None]
        if len(set(values)) != len(values):
            raise ArmConfigError(f"duplicate {key} in arms")
    return ArmRegistry(arms)
