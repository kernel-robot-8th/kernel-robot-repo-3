"""Pick an object at a given position in omx_link0 (from the camera) and put it down, without a learned policy.

PICK_AT_POSITION carries the object's centre in arm coordinates. The worker plans, from the jaw-mount IK in
omx_kinematics.py with the wrist pitched 45 degrees down and the fingertip 6.5 cm beyond the jaw mount:

  open gripper -> HOVER (fingertip 6 cm above, 2 cm short) -> GRASP (fingertip at the centre, 0.8 cm past it)
  -> close gripper (current-limited, stops at the object) -> LIFT (6 cm up) -> PLACE (taught pose "place_pinky")
  -> open gripper -> home

Every arm move is the slot worker's joint move (omx_slot_motion.SlotMotion.move: servo profile speed, timeout,
cancel, hold on error); the path between poses is not checked for collisions. Wrist roll stays where it is, so the
fingers close across the direction from the arm base to the object. Whether the object was really grasped and
placed is not observed: the gripper's stopping tick is reported as a hint only.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Callable

from omx_kinematics import FRAME_ID, Unreachable, inverse, radians_to_ticks, tick_to_radians
from omx_slot_motion import ARM_JOINTS, GRIPPER_OPEN, GRIPPER_SQUEEZE, SlotMotion

FINGER_M = 0.065  # jaw mount to fingertip along the wrist axis (ROBOTIS OMX gripper mesh)
PITCH_RAD = math.radians(45)
HOVER_ABOVE_M = 0.06
HOVER_BACK_M = 0.02
GRASP_PAST_M = 0.008  # the fingertip goes slightly past the centre so the object sits between the fingers
LIFT_M = 0.06
STOP_AFTER = ("grasp", "lift", "place")


@dataclass
class PickPlan:
    poses: dict[str, dict[str, int]]  # hover, grasp, lift: goal ticks for the five arm joints
    fingertips: dict[str, tuple[float, float, float]] = field(default_factory=dict)


def fingertip_to_jaw(tip: tuple[float, float, float], ux: float, uy: float) -> tuple[float, float, float]:
    return (
        tip[0] - FINGER_M * math.cos(PITCH_RAD) * ux,
        tip[1] - FINGER_M * math.cos(PITCH_RAD) * uy,
        tip[2] + FINGER_M * math.sin(PITCH_RAD),
    )


def plan_pick(center: tuple[float, float, float], present: dict[str, int]) -> PickPlan:
    """Joint goals for hover, grasp and lift; raises Unreachable."""
    x, y, z = center
    reach = math.hypot(x, y)
    if reach < 0.08:
        raise Unreachable("OUT_OF_REACH", "object is too close to the arm base")
    ux, uy = x / reach, y / reach
    tips = {
        "hover": (x - HOVER_BACK_M * ux, y - HOVER_BACK_M * uy, z + HOVER_ABOVE_M),
        "grasp": (x + GRASP_PAST_M * ux, y + GRASP_PAST_M * uy, z),
    }
    tips["lift"] = (tips["grasp"][0], tips["grasp"][1], z + LIFT_M)
    seed = tick_to_radians(present)
    poses = {}
    for name in ("hover", "grasp", "lift"):
        angles = inverse(fingertip_to_jaw(tips[name], ux, uy), PITCH_RAD, seed["wrist_roll"], seed=seed)
        poses[name] = radians_to_ticks(angles)
        seed = angles
    return PickPlan(poses, tips)


class PickAtPosition:
    """Runs one PICK_AT_POSITION command with the worker's SlotMotion. `send(event, **fields)` reports to the
    control station and raises when it refuses; `cancel_requested()` is polled during every move."""

    def __init__(self, motion: SlotMotion, stop_after: str, max_delta: int) -> None:
        self.motion = motion
        self.stop_after = stop_after
        self.max_delta = max_delta

    def check(self, command: dict) -> tuple[str | None, PickPlan | None]:
        """(reason to reject, plan). Reads the arm but writes nothing."""
        target = command["target"]
        if target.get("frame_id") != FRAME_ID:
            return "FRAME_UNSUPPORTED", None
        if target.get("approach") != "PITCH_DOWN_45":
            return "APPROACH_UNSUPPORTED", None
        if datetime.fromisoformat(command["valid_until"]) <= datetime.now(timezone.utc):
            return "EXPIRED", None
        if "home" not in self.motion.poses["poses"]:
            return "POSE_NOT_RECORDED", None
        if self.stop_after == "place" and "place_pinky" not in self.motion.poses["poses"]:
            print("  놓을 자세 place_pinky가 기록되지 않았습니다 (record_omx_pose.py place_pinky)", flush=True)
            return "POSE_NOT_RECORDED", None
        position = target["position"]
        present = self.motion.present()
        try:
            plan = plan_pick((position["x"], position["y"], position["z"]), present)
        except Unreachable as error:
            print(f"  IK 실패: {error}", flush=True)
            return error.reason_code, None
        # Each move must stay within the per-move joint change limit, starting from where the arm is now.
        sequence = [("hover", plan.poses["hover"]), ("grasp", plan.poses["grasp"]), ("lift", plan.poses["lift"])]
        if self.stop_after == "place":
            place = {name: int(self.motion.poses["poses"]["place_pinky"]["ticks"][name]) for name in ARM_JOINTS}
            home = {name: int(self.motion.poses["poses"]["home"]["ticks"][name]) for name in ARM_JOINTS}
            sequence += [("place", place), ("home", home)]
        previous = present
        for name, goal in sequence:
            delta = max(abs(goal[joint] - previous[joint]) for joint in ARM_JOINTS)
            print(f"  {name}: 목표 {goal} (이전 자세에서 최대 {delta} ticks)", flush=True)
            if delta > self.max_delta:
                return "TARGET_TOO_FAR", None
            previous = goal
        for name, tip in plan.fingertips.items():
            print(f"  손가락 끝 {name}: {tuple(round(v * 100, 1) for v in tip)} cm", flush=True)
        return None, plan

    def run(self, plan: PickPlan, send: Callable[..., dict], cancel_requested: Callable[[], bool]) -> None:
        motion = self.motion
        details: dict = {"fingertips_m": {k: [round(v, 4) for v in tip] for k, tip in plan.fingertips.items()}}

        def go(name: str, goal: dict[str, int], phase: str) -> bool:
            send("PROGRESS", phase=phase)
            outcome = motion.move(goal, cancel_requested, sync=True)
            details[name] = {"goal_ticks": goal, "final_ticks": outcome.final_ticks, "max_error_ticks": outcome.max_error_ticks}
            if outcome.outcome == "SUCCEEDED":
                return True
            if outcome.outcome == "CANCELED":
                send("CANCELED", outcome, phase=phase, details=details)
            else:
                send(outcome.outcome, outcome, reason_code=outcome.reason_code, message=outcome.message or None,
                     details=details)
            return False

        details["gripper_open_ticks"] = motion.gripper_to(GRIPPER_OPEN)
        if not go("hover", plan.poses["hover"], "APPROACH_TARGET"):
            return
        if not go("grasp", plan.poses["grasp"], "GRASP_TARGET"):
            return
        # Closing stops at the object in current-based position mode; where it stopped is only a hint.
        details["gripper_closed_ticks"] = motion.gripper_to(GRIPPER_SQUEEZE)
        if self.stop_after == "grasp":
            send("RESULT_UNKNOWN", reason_code="STOPPED_AFTER_GRASP", message="잡은 채로 멈췄습니다. 현장에서 확인하세요.",
                 details=details)
            return
        if not go("lift", plan.poses["lift"], "LIFT_TARGET"):
            return
        if self.stop_after == "lift":
            send("RESULT_UNKNOWN", reason_code="STOPPED_AFTER_LIFT", message="들어 올린 채로 멈췄습니다. 잡혔는지 현장에서 확인하세요.",
                 details=details)
            return
        place = {name: int(motion.poses["poses"]["place_pinky"]["ticks"][name]) for name in ARM_JOINTS}
        if not go("place", place, "MOVE_TO_DESTINATION"):
            return
        send("PROGRESS", phase="PLACE_TARGET")
        details["gripper_released_ticks"] = motion.gripper_to(GRIPPER_OPEN)
        home = {name: int(motion.poses["poses"]["home"]["ticks"][name]) for name in ARM_JOINTS}
        outcome = motion.move(home, cancel_requested, sync=True)
        details["home"] = {"goal_ticks": home, "final_ticks": outcome.final_ticks, "max_error_ticks": outcome.max_error_ticks}
        if outcome.outcome != "SUCCEEDED":
            send(outcome.outcome, outcome, reason_code=outcome.reason_code or "HOME_RETURN_FAILED",
                 message=outcome.message or None, details=details)
            return
        send("RESULT_UNKNOWN", outcome, reason_code="PLACE_NOT_VERIFIED", pose_reached=True,
             message="놓고 home으로 돌아왔습니다. 집기·적재는 확인하지 않았습니다.", details=details)
