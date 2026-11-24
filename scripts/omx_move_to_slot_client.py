#!/usr/bin/env python3
"""/omx/move_to_slot 목표 하나를 보내고 진행과 결과를 출력하며, Ctrl+C를 누르면 취소를 보낸다.

`ros2 action send_goal`은 중단해도 취소를 보내지 않으므로(Jazzy, 2026-11-04) 직접 해 보는 실습용으로 이 클라이언트를
만들었다. Ctrl+C(또는 --cancel-after)는 워커에 멈추라고 요청하고, 워커는 팔을 그 자리에 세운다.
손으로 치기 번거로운 slot_map_version과 valid_until도 채워 준다.

    source /opt/ros/jazzy/setup.bash && source cpp/install/setup.bash
    export ROS_DOMAIN_ID=87 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    python3 scripts/omx_move_to_slot_client.py slot_a_2_1
    python3 scripts/omx_move_to_slot_client.py home --cancel-after 1.5

종료 코드는 outcome이 SUCCEEDED이고 pose_reached일 때만 0이다. 모의 결과면 마지막 줄에 그렇게 표시한다.
워커(omx_ros_worker.py)가 실물 모드면 이 목표로 실물 팔이 움직인다. 하드웨어 전원 차단을 대신하지 않는다.
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
# action_msgs의 GoalStatus 번호 → 출력용 이름
STATUS_NAMES = {
    GoalStatus.STATUS_SUCCEEDED: "succeeded", GoalStatus.STATUS_CANCELED: "canceled", GoalStatus.STATUS_ABORTED: "aborted",
}


def build_goal(args: argparse.Namespace) -> MoveToSlot.Goal:
    """CLI 인자로 MoveToSlot 목표를 만든다.

    slot_map_version은 따로 주지 않으면 --slots 파일의 SHA-256이고, valid_until은 지금부터 --valid-for초 뒤다.
    """
    goal = MoveToSlot.Goal()
    goal.command_id = args.command_id or f"PRACTICE-{time.strftime('%H%M%S')}"
    goal.slot_id = args.slot_id
    goal.slot_map_version = args.slot_map_version or hashlib.sha256(args.slots.read_bytes()).hexdigest()
    deadline = time.time() + args.valid_for
    goal.valid_until.sec = int(deadline)
    goal.valid_until.nanosec = int((deadline % 1) * 1e9)
    return goal


def main() -> int:
    """목표 하나를 보내고 결과를 기다린다. 서버가 없으면 2, 거부·실패면 1, 성공이면 0을 돌려준다."""
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

    # rclpy 자체 SIGINT 처리기는 컨텍스트를 종료해 그 뒤로는 취소를 보낼 수 없다. Python 기본 처리기를 그대로 둔다.
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

        # 0.1초씩 돌며 결과를 기다린다. Ctrl+C(KeyboardInterrupt)나 --cancel-after 시각이 되면 취소를 한 번만 보낸다.
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
