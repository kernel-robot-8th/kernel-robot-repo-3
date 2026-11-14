"""Where each arm's devices are on this PC: follower serial port, LeRobot robot id, gripper and overhead cameras.

With two arms there are six cameras: each arm has a gripper (joint) camera and an overhead camera that its ACT
policies were trained with, plus the ceiling camera that checks Pinky's parking and the shelf camera that counts
stock. Device paths differ per PC, so they come from .control-station/control-station.env, one set per arm:

    CONTROL_STATION_ARM01_FOLLOWER_PORT     /dev/serial/by-id/usb-ROBOTIS_OpenRB-150_<board serial>-if00
    CONTROL_STATION_ARM01_ROBOT_ID          LeRobot calibration name (default CONTROL_STATION_POLICY_ROBOT_ID)
    CONTROL_STATION_ARM01_GRIPPER_CAMERA    /dev/v4l/by-path/...   (by-path: two cameras of one model share a by-id)
    CONTROL_STATION_ARM01_OVERHEAD_CAMERA   /dev/v4l/by-path/...
    (ARM02 the same)

The older single-arm settings (CONTROL_STATION_POLICY_ROBOT_PORT or CONTROL_STATION_FOLLOWER_PORT, the
CONTROL_STATION_POLICY_*_CAMERA, CONTROL_STATION_ACT_OVERHEAD_CAMERA and CONTROL_STATION_JOINT_CAMERA) still
work. They count for one arm only: the arm whose board serial is in that port path, and only while no ARMxx
setting of that arm is given. An arm-bound policy therefore never runs on another arm's port or cameras.
"""

from __future__ import annotations

import os
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

DEFAULT_ROBOT_ID = "omx_follower_arm"
CAMERA_ROLES = ("GRIPPER", "OVERHEAD")
ROLE_LABELS = {"GRIPPER": "그리퍼 캠", "OVERHEAD": "탑 캠"}
_FIELDS = ("FOLLOWER_PORT", "ROBOT_ID", "GRIPPER_CAMERA", "OVERHEAD_CAMERA")


def env_prefix(arm_id: str) -> str:
    """arm-01 -> CONTROL_STATION_ARM01_"""
    return "CONTROL_STATION_" + arm_id.replace("-", "").upper() + "_"


def _first(env: Mapping[str, str], *names: str) -> str | None:
    for name in names:
        value = env.get(name, "").strip()
        if value:
            return value
    return None


def legacy_port(env: Mapping[str, str] = os.environ) -> str | None:
    return _first(env, "CONTROL_STATION_POLICY_ROBOT_PORT", "CONTROL_STATION_FOLLOWER_PORT")


def legacy_cameras(env: Mapping[str, str] = os.environ) -> dict[str, str | None]:
    """The single-arm camera settings by role (policy runner names first, then the arm worker's)."""
    return {
        "OVERHEAD": _first(env, "CONTROL_STATION_POLICY_OVERHEAD_CAMERA", "CONTROL_STATION_POLICY_FRONT_CAMERA",
                           "CONTROL_STATION_ACT_OVERHEAD_CAMERA"),
        "GRIPPER": _first(env, "CONTROL_STATION_POLICY_GRIPPER_CAMERA", "CONTROL_STATION_POLICY_WRIST_CAMERA",
                          "CONTROL_STATION_JOINT_CAMERA"),
    }


def legacy_arm_id(arms: list[dict[str, Any]] | tuple[dict[str, Any], ...], env: Mapping[str, str] = os.environ) -> str | None:
    """The arm the single-arm port setting points at, found by its board serial in the path; None when unknown."""
    port = legacy_port(env)
    if not port:
        return None
    paths = {port, os.path.realpath(port)}
    found = [arm["arm_id"] for arm in arms
             if arm.get("board_serial") and any(arm["board_serial"] in path for path in paths)]
    return found[0] if len(found) == 1 else None


@dataclass(frozen=True)
class ArmDevices:
    arm_id: str
    label: str
    follower_port: str | None
    robot_id: str
    cameras: dict[str, str | None]  # GRIPPER / OVERHEAD -> device path
    source: str  # "arm": ARMxx settings · "legacy": the single-arm settings · "none": nothing set

    @property
    def missing(self) -> list[str]:
        """Settings still needed before a policy can run on this arm."""
        prefix = env_prefix(self.arm_id)
        names = [] if self.follower_port else [prefix + "FOLLOWER_PORT"]
        return names + [f"{prefix}{role}_CAMERA" for role in CAMERA_ROLES if not self.cameras.get(role)]

    @property
    def ready(self) -> bool:
        return not self.missing

    @property
    def devices(self) -> list[str]:
        """The cameras a policy run on this arm opens itself."""
        return [device for role in CAMERA_ROLES if (device := self.cameras.get(role))]

    def summary(self) -> dict[str, Any]:
        return {
            "arm_id": self.arm_id, "label": self.label, "source": self.source, "ready": self.ready,
            "missing": self.missing, "follower_port": self.follower_port, "robot_id": self.robot_id,
            "cameras": dict(self.cameras),
        }


def arm_devices(arms: list[dict[str, Any]] | tuple[dict[str, Any], ...],
                env: Mapping[str, str] = os.environ) -> dict[str, ArmDevices]:
    """arm_id -> its devices on this PC, in config order."""
    default_robot_id = _first(env, "CONTROL_STATION_POLICY_ROBOT_ID") or DEFAULT_ROBOT_ID
    legacy_arm = legacy_arm_id(arms, env)
    result: dict[str, ArmDevices] = {}
    for arm in arms:
        prefix = env_prefix(arm["arm_id"])
        own = {field: _first(env, prefix + field) for field in _FIELDS}
        if any(own.values()) or arm["arm_id"] != legacy_arm:
            port, cameras = own["FOLLOWER_PORT"], {role: own[f"{role}_CAMERA"] for role in CAMERA_ROLES}
            source = "arm" if any(own.values()) else "none"
        else:
            port, cameras, source = legacy_port(env), legacy_cameras(env), "legacy"
        result[arm["arm_id"]] = ArmDevices(
            arm_id=arm["arm_id"], label=arm["label"], follower_port=port,
            robot_id=own["ROBOT_ID"] or default_robot_id, cameras=cameras, source=source,
        )
    return result
