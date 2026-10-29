"""Offline validation for a single LeRobot OMX follower joint target."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping


OMX_POSITION_KEYS = (
    "shoulder_pan.pos",
    "shoulder_lift.pos",
    "elbow_flex.pos",
    "wrist_flex.pos",
    "wrist_roll.pos",
    "gripper.pos",
)


class PoseValidationError(ValueError):
    """Raised when a pose does not match the inspected OMX action contract."""


@dataclass(frozen=True)
class OmxJointPose:
    """A complete normalized OMX joint target; this object never sends commands."""

    positions: dict[str, float]

    @classmethod
    def from_mapping(cls, payload: Mapping[str, object]) -> "OmxJointPose":
        if payload.get("schema_version") != 1:
            raise PoseValidationError("schema_version must be 1")
        if payload.get("robot_type") != "omx_follower":
            raise PoseValidationError("robot_type must be 'omx_follower'")
        if payload.get("position_mode") != "lerobot_default_normalized":
            raise PoseValidationError(
                "position_mode must be 'lerobot_default_normalized'; degree targets are not supported"
            )

        raw_positions = payload.get("positions")
        if not isinstance(raw_positions, Mapping):
            raise PoseValidationError("positions must be an object")

        expected = set(OMX_POSITION_KEYS)
        actual = set(raw_positions)
        missing = sorted(expected - actual)
        extra = sorted(actual - expected)
        if missing or extra:
            details = []
            if missing:
                details.append(f"missing keys: {', '.join(missing)}")
            if extra:
                details.append(f"unexpected keys: {', '.join(extra)}")
            raise PoseValidationError("; ".join(details))

        positions: dict[str, float] = {}
        for key in OMX_POSITION_KEYS:
            value = raw_positions[key]
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise PoseValidationError(f"{key} must be a number")
            numeric_value = float(value)
            if not math.isfinite(numeric_value):
                raise PoseValidationError(f"{key} must be finite")

            minimum, maximum = (0.0, 100.0) if key == "gripper.pos" else (-100.0, 100.0)
            if not minimum <= numeric_value <= maximum:
                raise PoseValidationError(f"{key} must be between {minimum:g} and {maximum:g}")
            positions[key] = numeric_value

        return cls(positions=positions)

    def to_mapping(self) -> dict[str, object]:
        return {
            "schema_version": 1,
            "robot_type": "omx_follower",
            "position_mode": "lerobot_default_normalized",
            "positions": dict(self.positions),
        }
