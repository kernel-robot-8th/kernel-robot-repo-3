#!/usr/bin/env python3
"""손으로 가르친 관절 자세로 물체를 집어 핑키 위에 올린다(수동 시험 도구, 관제 없이 실행).

실물 팔과 그리퍼가 움직인다. 관제 API와는 통신하지 않고, 팔로워 포트(LeRobot Dynamixel 버스)에 직접 쓴다.

먼저 팔 토크를 끈 상태에서 자세를 가르친다(record_omx_pose.py --capture-s 30은 가만히 들고 있는 자세를 기록한다):

    pick_<object>_above   그리퍼가 물체 바로 위
    pick_<object>_grasp   물체가 벌린 손가락 사이에 있음
    place_pinky           핑키 카트 위
    home                  쉬는 자세 (이 팔에 이미 기록되어 있음)

그다음 재생한다:

    sg dialout -c "~/lerobot-venv/bin/python scripts/omx_taught_pick.py --object apple --stop-after lift ..."

순서: 그리퍼 열기 → 물체 위 → 집는 자세 → 그리퍼 닫기 → 물체 위(들기) → place_pinky → 그리퍼 열기 → home.
팔 이동은 모두 칸 워커와 같은 이동이다(omx_slot_motion: 목표를 현재 위치로 맞춘 뒤 토크를 켬, 서보 프로파일 속도,
시간 제한, 오류 시 그 자리에 멈춤). Ctrl+C를 누르면 팔을 그 자리에 세운다. 관절 공간 이동이라 경로의 충돌을 검사하지
않는다: 전원 스위치에 손을 둔 채 팔을 지켜본다. 물체를 잡았는지, 놓았는지는 관측하지 않는다. 그리퍼가 멈춘 틱을
힌트로 출력한다(물체에 걸리면 일찍 멈춘다).
워커가 동시에 명령을 처리하고 있으면 안 된다(팔로워 포트를 함께 쓰게 된다).
"""

from __future__ import annotations

import argparse
import sys

from omx_slot_motion import ARM_JOINTS, GRIPPER_OPEN, GRIPPER_SQUEEZE, SlotMotion, add_motion_arguments

# --stop-after로 고를 수 있는 멈춤 지점: 잡은 직후, 들어 올린 뒤, 핑키 위에 놓고 home까지(기본).
STOP_AFTER = ("grasp", "lift", "place")


def main() -> int:
    """가르친 자세를 확인하고, 모든 이동이 --max-delta 안인지 먼저 검사한 뒤 차례로 재생한다."""
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
        for label, target in steps:  # 무엇이든 움직이기 전에, 모든 이동이 한 번 이동 한도 안인지 확인한다
            delta = max(abs(target[joint] - previous[joint]) for joint in ARM_JOINTS)
            print(f"  {label}: 최대 {delta} ticks", flush=True)
            if delta > args.max_delta:
                print(f"{label}까지 관절 변화가 --max-delta {args.max_delta}보다 큽니다. 움직이지 않았습니다.", file=sys.stderr)
                return 1
            previous = target

        def move(label: str, target: dict[str, int]) -> bool:
            """한 자세로 옮긴다. sync=True라 모든 관절이 함께 도착하도록 관절마다 프로파일을 맞춘다."""
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
            # 그리퍼는 전류 기반 위치 모드라 물체에 닿으면 목표(GRIPPER_SQUEEZE)보다 벌어진 채 멈춘다.
            # 목표보다 60틱 넘게 벌어져 있으면 물체에 걸린 것으로 짐작한다(확인은 아니다).
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
            # 현재 위치를 목표로 써서 그 자리에 세운다. 토크는 끄지 않는다(팔이 떨어지지 않게).
            motion.hold_here()
            print("\nCtrl+C: 그 자리에 멈췄습니다 (토크 유지).", flush=True)
            return 1


if __name__ == "__main__":
    raise SystemExit(main())
