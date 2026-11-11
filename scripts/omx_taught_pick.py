#!/usr/bin/env python3
"""Pick an object with joint poses taught by hand and put it on Pinky (manual test tool, no control station).

Teach the poses first with the arm's torque off (record_omx_pose.py --capture-s 30 holds the pose you keep still):

    pick_<object>_above   gripper right above the object
    pick_<object>_grasp   the object between the open fingers
    place_pinky           above Pinky's cart
    home                  rest pose (already recorded for this arm)

Then replay:

    sg dialout -c "~/lerobot-venv/bin/python scripts/omx_taught_pick.py --object apple --stop-after lift ..."

Order: open gripper -> above -> grasp -> close gripper -> above (lift) -> place_pinky -> open gripper -> home.
Each arm move is the slot worker's move (omx_slot_motion: torque enabled with the goal at the present position,
servo profile speed, timeout, hold where it is on error). Ctrl+C holds the arm where it is. Joint-space moves do
not check the path for collisions: watch the arm with the power switch at hand. Whether the object was grasped
and placed is not observed; the gripper's stopping tick is printed as a hint (it stops early on an object).
The worker must not be handling a command at the same time (the follower port is shared).
"""

from __future__ import annotations

import argparse
import sys

from omx_slot_motion import ARM_JOINTS, GRIPPER_OPEN, GRIPPER_SQUEEZE, SlotMotion, add_motion_arguments

STOP_AFTER = ("grasp", "lift", "place")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--object", required=True, help="taught pose name part, e.g. apple -> pick_apple_above")
    parser.add_argument("--place", default="place_pinky")
    parser.add_argument("--stop-after", choices=STOP_AFTER, default="place", help="stop holding the object here")
    add_motion_arguments(parser)
    args = parser.parse_args()
    args.frame_id = "omx_link0"

    motion = SlotMotion(args)
    poses = motion.poses["poses"]
    names = [f"pick_{args.object}_above", f"pick_{args.object}_grasp", "home"]
    if args.stop_after == "place":
        names.append(args.place)
    missing = [name for name in names if name not in poses]
    if missing:
        print(f"기록되지 않은 자세: {missing} (record_omx_pose.py로 먼저 기록)", file=sys.stderr)
        return 1
    goal = {name: {joint: int(poses[name]["ticks"][joint]) for joint in ARM_JOINTS} for name in names}
    above, grasp = goal[f"pick_{args.object}_above"], goal[f"pick_{args.object}_grasp"]
    steps = [("물체 위", above), ("집는 자세", grasp), ("들기", above)]
    if args.stop_after == "place":
        steps += [("Pinky 위", goal[args.place]), ("home", goal["home"])]

    with motion.follower():
        previous = motion.present()
        for label, target in steps:  # every move within the per-move limit, before anything moves
            delta = max(abs(target[joint] - previous[joint]) for joint in ARM_JOINTS)
            print(f"  {label}: 최대 {delta} ticks", flush=True)
            if delta > args.max_delta:
                print(f"{label}까지 관절 변화가 --max-delta {args.max_delta}보다 큽니다. 움직이지 않았습니다.", file=sys.stderr)
                return 1
            previous = target

        def move(label: str, target: dict[str, int]) -> bool:
            print(f"→ {label}", flush=True)
            outcome = motion.move(target, lambda: False, sync=True)
            if outcome.outcome != "SUCCEEDED":
                print(f"  멈춤: {outcome.outcome} {outcome.reason_code} {outcome.message}", flush=True)
                return False
            return True

        try:
            print(f"→ 그리퍼 열기: {motion.gripper_to(GRIPPER_OPEN)} ticks", flush=True)
            if not (move("물체 위", above) and move("집는 자세", grasp)):
                return 1
            closed = motion.gripper_to(GRIPPER_SQUEEZE)
            hint = "물체에 걸려 멈춘 듯함" if closed > GRIPPER_SQUEEZE + 60 else "끝까지 닫힘(물체를 못 잡았을 수 있음)"
            print(f"→ 그리퍼 닫기: {closed} ticks ({hint})", flush=True)
            if args.stop_after == "grasp":
                return 0
            if not move("들기", above):
                return 1
            if args.stop_after == "lift":
                print("들어 올린 채 멈췄습니다. 잡혔는지 확인하세요.", flush=True)
                return 0
            if not move("Pinky 위", goal[args.place]):
                return 1
            print(f"→ 그리퍼 열기(놓기): {motion.gripper_to(GRIPPER_OPEN)} ticks", flush=True)
            if not move("home", goal["home"]):
                return 1
            print("끝: 놓고 home으로 돌아왔습니다. 놓였는지는 눈으로 확인하세요.", flush=True)
            return 0
        except KeyboardInterrupt:
            motion.hold_here()
            print("\nCtrl+C: 그 자리에 멈췄습니다 (토크 유지).", flush=True)
            return 1


if __name__ == "__main__":
    raise SystemExit(main())
