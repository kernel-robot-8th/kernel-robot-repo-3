#!/usr/bin/env python3
"""Send one /omx/move_to_slot goal, print its feedback and result, and cancel it on Ctrl+C.

`ros2 action send_goal` does not send a cancel when interrupted (Jazzy, 2026-11-04), so this client exists for
hands-on practice: Ctrl+C (or --cancel-after) asks the worker to stop, and the worker holds the arm where it is.
It also fills slot_map_version and valid_until, which are tedious to type by hand.

    source /opt/ros/jazzy/setup.bash && source cpp/install/setup.bash
    export ROS_DOMAIN_ID=87 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    python3 scripts/omx_move_to_slot_client.py slot_a_2_1
    python3 scripts/omx_move_to_slot_client.py home --cancel-after 1.5

Exit code 0 only when the outcome is SUCCEEDED with pose_reached; a simulated result says so on its last line.
This does not replace the hardware power cut.
"""

from __future__ import annotations

import argparse
import hashlib
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from omx_interfaces.action import MoveToSlot


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
STATUS_NAMES = {
    GoalStatus.STATUS_SUCCEEDED: "succeeded", GoalStatus.STATUS_CANCELED: "canceled", GoalStatus.STATUS_ABORTED: "aborted",
}


def build_goal(args: argparse.Namespace) -> MoveToSlot.Goal:
    goal = MoveToSlot.Goal()
    goal.command_id = args.command_id or f"PRACTICE-{time.strftime('%H%M%S')}"
    goal.slot_id = args.slot_id
    goal.slot_map_version = args.slot_map_version or hashlib.sha256(args.slots.read_bytes()).hexdigest()
    deadline = time.time() + args.valid_for
    goal.valid_until.sec = int(deadline)
    goal.valid_until.nanosec = int((deadline % 1) * 1e9)
    return goal


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("slot_id", help="a slot ID such as slot_a_2_1, or 'home' when it is recorded")
    parser.add_argument("--action", default="/omx/move_to_slot")
    parser.add_argument("--slots", type=Path, default=REPOSITORY_ROOT / "config" / "shelf-slots.json")
    parser.add_argument("--slot-map-version", help="send this instead of the SHA-256 of --slots (to practise a rejection)")
    parser.add_argument("--valid-for", type=float, default=30.0, help="seconds until the worker must not start (negative = expired)")
    parser.add_argument("--command-id")
    parser.add_argument("--cancel-after", type=float, help="request cancel this many seconds after the goal is accepted")
    parser.add_argument("--server-timeout", type=float, default=5.0)
    args = parser.parse_args()

    # rclpy's own SIGINT handler shuts the context down, after which no cancel can be sent; keep Python's.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node("omx_move_to_slot_client")
    client = ActionClient(node, MoveToSlot, args.action)
    try:
        if not client.wait_for_server(timeout_sec=args.server_timeout):
            print(f"{args.action} 서버를 {args.server_timeout:.0f}초 안에 찾지 못했습니다 (워커·ROS_DOMAIN_ID 확인). 보낸 목표 없음.")
            return 2
        goal = build_goal(args)
        print(f"[{goal.command_id}] {goal.slot_id} 목표 전송 (유효 {args.valid_for:+.0f}초)", flush=True)

        def on_feedback(message) -> None:
            feedback = message.feedback
            ticks = dict(zip(feedback.joint_names, feedback.present_ticks))
            print(f"  진행 {feedback.phase}: 남은 최대 오차 {feedback.max_error_ticks} ticks  {ticks}", flush=True)

        sent = client.send_goal_async(goal, feedback_callback=on_feedback)
        rclpy.spin_until_future_complete(node, sent)
        handle = sent.result()
        if not handle.accepted:
            print("  ROS 단계에서 받지 않음 (다른 목표 실행 중). 팔은 이 목표로 움직이지 않습니다.")
            return 1
        print("  수락됨 (수락은 이동 성공이 아닙니다)", flush=True)

        result_future = handle.get_result_async()
        cancel_at = None if args.cancel_after is None else time.monotonic() + args.cancel_after
        cancel_sent = False
        while not result_future.done():
            try:
                rclpy.spin_until_future_complete(node, result_future, timeout_sec=0.1)
                if cancel_at is not None and not cancel_sent and time.monotonic() >= cancel_at:
                    raise KeyboardInterrupt
            except KeyboardInterrupt:
                if cancel_sent:
                    print("  취소 요청은 이미 보냈습니다. 결과를 기다립니다 (팔이 이상하면 전원을 끊으세요).", flush=True)
                    continue
                print("  취소 요청 전송 → 워커는 그 자리에 멈춥니다", flush=True)
                handle.cancel_goal_async()
                cancel_sent = True

        response = result_future.result()
        result = response.result
        print(f"  결과 {result.outcome}  pose_reached={result.pose_reached}  ROS 상태 {STATUS_NAMES.get(response.status, response.status)}")
        if result.reason_code or result.message:
            print(f"  사유 {result.reason_code} {result.message}".rstrip())
        if result.joint_names:
            print(f"  최종 {dict(zip(result.joint_names, result.final_ticks))}  최대 오차 {result.max_error_ticks} ticks")
        print("  모의 결과 (실물 모터는 움직이지 않음)" if result.simulated else "  실물 모터 결과")
        return 0 if result.outcome == "SUCCEEDED" and result.pose_reached else 1
    finally:
        client.destroy()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
