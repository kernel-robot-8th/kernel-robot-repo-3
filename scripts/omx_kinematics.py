#!/usr/bin/env python3
"""OMX forward and inverse kinematics on Dynamixel ticks. No hardware imports unless --live is used.

    ~/lerobot-venv/bin/python scripts/omx_kinematics.py --pose slot_a_2_1        # a pose from omx-slot-poses.json
    sg dialout -c "~/lerobot-venv/bin/python scripts/omx_kinematics.py --live"    # the follower's present ticks (read only)
    ~/lerobot-venv/bin/python scripts/omx_kinematics.py --target 0.20,0,0.15      # IK only, prints the ticks

Frame `omx_link0` (the ROBOTIS model's base frame): origin at the bottom of the OMX base, 1.125 cm behind the
shoulder-pan axis; x forward (where the arm points at shoulder_pan tick 2048), y to the arm's left, z up, metres.

Link offsets and joint axes are copied from the ROBOTIS OMX MJCF in simulation/mujoco_omx/robotis_omx/omx.xml
(robotis_mujoco_menagerie). No MuJoCo is needed: every body there has an identity orientation, so the chain is
plain translations and single-axis rotations, and the IK is closed form: the pan turns toward the target, and the
three pitch joints form a planar two-link arm plus a wrist held at the requested pitch.

The tick-to-angle mapping is a hypothesis until it matches measurements: angle = (tick - 2048) * 2*pi/4096, the
ROBOTIS convention, same sign for every joint (Drive_Mode 0). The tool point is the jaw mount (link6 origin on the
wrist axis); the real fingertip is further out and not measured yet.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
FRAME_ID = "omx_link0"
ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
TICKS_PER_TURN = 4096
ZERO_TICK = 2048
# (joint, offset of its link from the parent link in metres, rotation axis) from omx.xml, link1..link5.
CHAIN = (
    ("shoulder_pan", (-0.01125, 0.0, 0.034), (0, 0, 1)),
    ("shoulder_lift", (0.0, 0.0, 0.0635), (0, 1, 0)),
    ("elbow_flex", (0.0415, 0.0, 0.11315), (0, 1, 0)),
    ("wrist_flex", (0.162, 0.0, 0.0), (0, 1, 0)),
    ("wrist_roll", (0.0287, 0.0, 0.0), (1, 0, 0)),
)
TOOL = (0.0295, 0.0, 0.0)  # jaw mount (link6 origin) on the wrist axis, without the jaw's 7.5 mm sideways offset
# Joint ranges of the MJCF (radians). IK answers outside them are refused; recorded poses are not checked.
LIMITS = {
    "shoulder_pan": (-4.712389, 6.283185),
    "shoulder_lift": (-2.094395, 1.570796),
    "elbow_flex": (-2.094395, 1.570796),
    "wrist_flex": (-1.745329, 1.745329),
    "wrist_roll": (-4.712389, 4.712389),
}
POINT_NAMES = ("어깨 회전 모터", "어깨 들기 축", "팔꿈치 축", "손목 꺾기 축", "손목 회전 시작", "그리퍼 턱 부착점")

# Planar geometry used by the IK, derived from CHAIN.
_PAN_X = CHAIN[0][1][0]
_SHOULDER_Z = CHAIN[0][1][2] + CHAIN[1][1][2]
_UPPER = (CHAIN[2][1][0], CHAIN[2][1][2])  # shoulder-lift axis to elbow, at shoulder_lift 0 (forward, up)
_UPPER_LENGTH = math.hypot(*_UPPER)
_UPPER_ANGLE = math.atan2(_UPPER[1], _UPPER[0])
_FOREARM = CHAIN[3][1][0]
_WRIST_TO_TOOL = CHAIN[4][1][0] + TOOL[0]


class Unreachable(Exception):
    """No joint solution for the requested point; `reason_code` says why."""

    def __init__(self, reason_code: str, message: str) -> None:
        super().__init__(message)
        self.reason_code = reason_code


def tick_to_radians(ticks: dict[str, int]) -> dict[str, float]:
    return {name: (ticks[name] - ZERO_TICK) * 2 * math.pi / TICKS_PER_TURN for name in ARM_JOINTS}


def radians_to_ticks(angles: dict[str, float]) -> dict[str, int]:
    return {name: ZERO_TICK + round(angles[name] * TICKS_PER_TURN / (2 * math.pi)) for name in ARM_JOINTS}


def rotation(axis: tuple[int, int, int], angle: float) -> np.ndarray:
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]], dtype=float)
    return np.eye(3) + s * cross + (1 - c) * cross @ cross


def forward(angles: dict[str, float]) -> list[np.ndarray]:
    """Positions (metres, omx_link0) of the six points named in POINT_NAMES; the last one is the tool point."""
    position, orientation = np.zeros(3), np.eye(3)
    points = []
    for name, offset, axis in CHAIN:
        position = position + orientation @ np.array(offset)
        orientation = orientation @ rotation(axis, angles[name])
        points.append(position)
    points.append(position + orientation @ np.array(TOOL))
    return points


def tool_pitch(angles: dict[str, float]) -> float:
    """How far the wrist axis points below horizontal (radians, positive = down)."""
    return angles["shoulder_lift"] + angles["elbow_flex"] + angles["wrist_flex"]


def inverse(target: tuple[float, float, float], pitch: float = 0.0, wrist_roll: float = 0.0,
            seed: dict[str, float] | None = None) -> dict[str, float]:
    """Joint angles that put the tool point at `target` (omx_link0, metres) with the wrist axis `pitch` radians
    below horizontal. Of the two elbow solutions the one nearer `seed` is returned (elbow up without a seed)."""
    x, y, z = (float(value) for value in target)
    reach = math.hypot(x - _PAN_X, y)
    if reach < 1e-6:
        raise Unreachable("OUT_OF_REACH", "target is on the shoulder-pan axis")
    pan = math.atan2(y, x - _PAN_X)
    # Work in the arm's vertical plane: forward distance from the pan axis and height above the shoulder.
    wrist_forward = reach - _WRIST_TO_TOOL * math.cos(pitch)
    wrist_up = z - _SHOULDER_Z + _WRIST_TO_TOOL * math.sin(pitch)
    distance = math.hypot(wrist_forward, wrist_up)
    cos_inner = (_UPPER_LENGTH**2 + distance**2 - _FOREARM**2) / (2 * _UPPER_LENGTH * distance)
    if not -1.0 <= cos_inner <= 1.0:
        raise Unreachable("OUT_OF_REACH", f"wrist point {distance * 100:.1f} cm from the shoulder cannot be reached")
    direction, inner = math.atan2(wrist_up, wrist_forward), math.acos(cos_inner)
    candidates = []
    for upper_angle in (direction + inner, direction - inner):  # elbow up first
        elbow_forward = wrist_forward - _UPPER_LENGTH * math.cos(upper_angle)
        elbow_up = wrist_up - _UPPER_LENGTH * math.sin(upper_angle)
        forearm_angle = math.atan2(elbow_up, elbow_forward)
        # Joint rotations about +y turn the arm downward, so planar angles are the negatives of joint sums.
        lift = _UPPER_ANGLE - upper_angle
        elbow = -forearm_angle - lift
        angles = {
            "shoulder_pan": pan,
            "shoulder_lift": lift,
            "elbow_flex": elbow,
            "wrist_flex": pitch - lift - elbow,
            "wrist_roll": wrist_roll,
        }
        angles = {name: math.remainder(value, 2 * math.pi) for name, value in angles.items()}
        if all(LIMITS[name][0] <= angles[name] <= LIMITS[name][1] for name in ARM_JOINTS):
            candidates.append(angles)
    if not candidates:
        raise Unreachable("JOINT_LIMIT", "every solution leaves the model's joint range")
    if seed is None:
        return candidates[0]
    return min(candidates, key=lambda angles: sum((angles[name] - seed[name]) ** 2 for name in ARM_JOINTS))


def read_live(port: str) -> dict[str, int]:
    from record_omx_pose import open_follower

    bus = open_follower(port)
    try:
        values = bus.sync_read("Present_Position", list(ARM_JOINTS), normalize=False, num_retry=3)
    finally:
        bus.disconnect(disable_torque=False)
    return {name: int(value) for name, value in values.items()}


def describe(ticks: dict[str, int]) -> None:
    angles = tick_to_radians(ticks)
    print("관절   " + "  ".join(f"{name} {ticks[name]} ({math.degrees(angles[name]):+.1f}°)" for name in ARM_JOINTS))
    for label, point in zip(POINT_NAMES, forward(angles)):
        x, y, z = point * 100
        print(f"  {label:<12} x {x:6.1f}  y {y:6.1f}  z {z:6.1f} cm")
    print(f"  손목 기울기 수평 아래로 {math.degrees(tool_pitch(angles)):+.1f}°")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--pose", help="pose name in --poses, e.g. home or slot_a_2_1")
    source.add_argument("--live", action="store_true", help="read the follower's present ticks (no writes)")
    source.add_argument("--target", help="x,y,z of the tool point in omx_link0, metres")
    parser.add_argument("--pitch-deg", type=float, default=0.0, help="wrist axis below horizontal for --target")
    parser.add_argument("--poses", type=Path, default=REPOSITORY_ROOT / "config" / "omx-slot-poses.json")
    parser.add_argument("--port", default="/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_F58AE68650323952582E3120FF05260E-if00")
    args = parser.parse_args()

    if args.target:
        target = tuple(float(value) for value in args.target.split(","))
        try:
            ticks = radians_to_ticks(inverse(target, math.radians(args.pitch_deg)))
        except Unreachable as error:
            print(f"{error.reason_code}: {error}")
            return 1
    else:
        ticks = read_live(args.port) if args.live else json.loads(args.poses.read_text(encoding="utf-8"))["poses"][args.pose]["ticks"]
    describe(ticks)
    print(f"좌표계 {FRAME_ID}. 가설 변환(틱 2048 = 0°, 모든 관절 부호 +)으로 계산했습니다. 실측 확인 전에는 참고용입니다.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
