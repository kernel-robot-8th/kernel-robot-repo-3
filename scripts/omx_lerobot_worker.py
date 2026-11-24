#!/usr/bin/env python3
"""실물 팔로워용 OMX 팔 워커다. 칸에 기록해 둔 관절 자세로 팔을 옮기거나(MOVE_TO_SLOT), 기록한 "home" 자세로
되돌리거나(RETURN_HOME), 물체별로 학습한 ACT 정책을 실행한다(PICK_WITH_POLICY). 운영자 조그(JOG_JOINT)도 받는다.

시리얼 포트 권한이 있는 LeRobot 환경에서 실행한다:

    sg dialout -c "~/lerobot-venv/bin/python scripts/omx_lerobot_worker.py --server http://127.0.0.1:8000"

`OMXFollower.send_action` 대신 LeRobot의 Dynamixel 버스를 직접 쓴다. LeRobot 로봇을 연결하면 모터 설정을 다시 쓰고,
목표 위치를 현재 위치에 먼저 맞추지 않은 채 토크를 켜며, 호출마다 거는 한 스텝 제한은 속도 제한이 아니기 때문이다.
여기서는 서보 자체의 사다리꼴 프로파일(Profile_Velocity/Profile_Acceleration)이 속도를 제한한다.

이동 자체와 안전 검사는 ROS 2 action 워커(omx_ros_worker.py)와 함께 쓰는 omx_slot_motion.py에 있다.
이 HTTP 워커가 더하는 것:
- 관제가 ACCEPTED에 HTTP 200으로 답한 뒤에만 움직인다
- 움직이는 동안 약 1초마다 명령의 취소 표시를 확인한다
- --release-torque는 팔 토크를 끄고 끝난다 (먼저 손으로 팔을 받친다)
PICK_WITH_POLICY는 먼저 이 팔에 기록된 home 자세로 돌아간 뒤, 이 PC의 정책 서버를 거쳐 그 물체용으로 학습한
LeRobot ACT 정책을 실행한다(omx_policy_pick.py, ROS 2 action 서버 omx_pick_action_server.py와 공유).
팔이 쉬는 자세로 돌아오면 정책을 멈추고 기록한 home 자세로 정렬한다. 물체를 집어 놓았는지는 확인하지 않으므로
결과는 RESULT_UNKNOWN / PLACE_NOT_VERIFIED다.
한 팔(arm_id)에서 녹화한 정책은 그 팔의 워커(--robot-id)가 그 팔의 보드(config/arms.json의 board_serial)에서만
실행한다. 팔이 두 대면 팔마다 워커 하나를 띄우고, 각 워커는 자기 --robot-id에 맞는
CONTROL_STATION_ARM01_* / ARM02_* (FOLLOWER_PORT, GRIPPER_CAMERA, OVERHEAD_CAMERA)에서 포트와 카메라를 가져온다.
그 밖의 명령 종류는 거부한다(NOT_IMPLEMENTED).
명령을 받으면 실물 팔이 움직인다. 하드웨어 전원 차단을 대신하지 않는다.
"""

from __future__ import annotations

import argparse
import time
from datetime import datetime
from pathlib import Path

from mock_omx_worker import Refused, Worker, request_json
from omx_policy_pick import PolicyPick, add_policy_arguments, arm_env, finish_policy_arguments
from omx_slot_motion import ARM_JOINTS, Outcome, Rejected, SlotMotion, add_motion_arguments

class LeRobotOmxWorker(Worker):
    """모의 워커(mock_omx_worker.Worker)의 long-poll·이벤트 보고 틀에 실물 이동과 정책 실행을 붙인 워커."""

    banner = "실물 OMX 워커 {worker_id}: {url} 대기 (명령을 받으면 팔이 움직입니다)"

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__(args)
        self.motion = SlotMotion(args)
        self.goal: dict[str, int] = {}
        self.pick = PolicyPick(self.motion, args)
        print(self.motion.describe(), flush=True)
        print(f"학습 정책: {sorted(self.pick.policies)} · 위 카메라 {args.overhead_camera or '미지정'} · 그리퍼 카메라 "
              f"{args.gripper_camera} · 정책 서버 {args.policy_server}", flush=True)

    # ---- 워커 훅 ----
    def check(self, command: dict) -> str | None:
        """모터에 쓰기 전 검사. 통과하면 self.goal(관절별 목표 틱)을 채우고 None, 아니면 reason code를 돌려준다.

        팔로워 포트를 연 상태(follower() 안)에서 불린다. 조그는 현재 관절값을 읽어 목표를 계산하기 때문이다.
        """
        target = command["target"]
        if command["operation"] == "JOG_JOINT":
            try:
                self.goal = self.motion.plan_jog(
                    target.get("joint"), target.get("direction"),
                    datetime.fromisoformat(command["valid_until"]), step_ticks=MANUAL_JOG_STEP_TICKS,
                )
            except Rejected as rejected:
                return rejected.reason_code
            return None
        if command["operation"] not in ("MOVE_TO_SLOT", "RETURN_HOME"):
            return "NOT_IMPLEMENTED"
        # RETURN_HOME은 이 워커의 자세 파일에 있는 "home" 자세를 재생한다. home은 워커가 읽어 들인 칸 정의 버전에 속한다.
        home = command["operation"] == "RETURN_HOME"
        try:
            self.goal = self.motion.plan(
                "home" if home else target.get("pick_slot_id"),
                self.motion.slot_map_version if home else target.get("slot_map_version"),
                datetime.fromisoformat(command["valid_until"]),
            )
        except Rejected as rejected:
            return rejected.reason_code
        return None

    # ---- PICK_WITH_POLICY (검사, 실행, home 정렬은 omx_policy_pick.PolicyPick이 한다) ----
    def handle_policy(self, command: dict) -> None:
        """정책 집기 한 건: 검사 → ACCEPTED → home 복귀 → 정책 준비 → 정책 실행 → home 정렬 → 최종 보고.

        각 단계 사이에 관제의 취소 요청을 확인한다. home 복귀나 준비가 실패·취소되면 정책을 시작하지 않는다.
        """
        policy_ref = command["target"].get("policy") or {}
        print(f"[{command['command_id']}] PICK_WITH_POLICY {policy_ref.get('label')} ({policy_ref.get('model')})", flush=True)
        try:
            reason, policy = self.pick.check(policy_ref, datetime.fromisoformat(command["valid_until"]))
        except (ConnectionError, OSError, RuntimeError) as error:
            print(f"  팔로워 포트를 열 수 없음: {error}", flush=True)
            reason, policy = "FOLLOWER_PORT_UNAVAILABLE", None
        try:
            if reason:
                self.send(command, "REJECTED", reason_code=reason)
                return
            command = self.send(command, "ACCEPTED")  # 여기서 HTTP 200을 받아야만 움직일 수 있다
        except Refused as refused:
            print(f"  중단: 관제가 거부함 ({refused}). 더 진행하지 않습니다.", flush=True)
            return

        command_id = command["command_id"]

        def cancel_requested() -> bool:
            # 정책 실행 중에는 취소 확인이 안 되면 멈추는 쪽이 안전하므로, 200이 아니면 예외로 정책 루프를 끝낸다.
            status, body = request_json("GET", f"{self.base}/api/arm/commands/{command_id}", timeout=5)
            if status != 200:
                raise RuntimeError(f"HTTP {status}")
            return body["command"]["cancel_requested"]

        try:
            command = self.send(command, "PROGRESS", phase="RETURNING_HOME",
                                message="정책 시작 전 저장된 초기자세로 복귀 중")
            home_end = self.pick.return_home_before_policy(cancel_requested)
            if home_end is not None:
                self.send(command, home_end.event, reason_code=home_end.reason_code, message=home_end.message,
                          pose_reached=home_end.pose_reached, phase=home_end.phase, details=home_end.details)
                return
            if cancel_requested():
                self.send(command, "CANCELED", reason_code="OPERATOR_CANCELED",
                          message="초기자세 복귀 후 정책 시작 전에 취소되었습니다.", pose_reached=True,
                          phase="RETURNING_HOME")
                return
            self.pick.prepare()
            if cancel_requested():
                self.send(command, "CANCELED", reason_code="OPERATOR_CANCELED",
                          message="정책 준비 중 취소되어 정책을 시작하지 않았습니다.", pose_reached=True,
                          phase="RETURNING_HOME")
                return
        except Refused as refused:
            print(f"  관제가 초기자세 복귀를 거부함 ({refused}). 정책은 시작하지 않았습니다.", flush=True)
            return
        except Exception as error:
            self.report_policy_error(command, f"초기자세 복귀 또는 정책 준비 실패: {error}")
            return

        try:
            result, policy_details = self.pick.run(
                policy, cancel_requested=cancel_requested,
                started=lambda: self.send(command, "PROGRESS", phase="APPROACH_TARGET"),
            )
        except Refused as refused:
            print(f"  관제가 진행 보고를 거부함 ({refused}). 정책은 멈췄고 팔은 그 자리에서 토크를 유지합니다.", flush=True)
            return
        except Exception as error:  # 팔은 정책이 멈춘 자리에서 토크를 유지한다. 사람이 현장을 봐야 한다
            self.report_policy_error(command, str(error))
            return
        end = self.pick.end(result, policy_details)
        try:
            self.send(command, end.event, reason_code=end.reason_code, message=end.message,
                      pose_reached=end.pose_reached, phase=end.phase, details=end.details)
        except Refused as refused:
            print(f"  관제가 결과 보고를 거부함 ({refused})", flush=True)

    def report_policy_error(self, command: dict, message: str, policy_details: dict | None = None) -> None:
        """정책 실행 중 오류를 RESULT_UNKNOWN / POLICY_RUN_ERROR로 보고한다(팔이 어디서 멈췄는지 모르므로 실패가 아니다)."""
        print(f"  정책 실행 오류: {message}", flush=True)
        try:
            self.send(command, "RESULT_UNKNOWN", reason_code="POLICY_RUN_ERROR", message=message[:300],
                      details={"policy_run": policy_details} if policy_details else None)
        except Refused:
            pass

    def handle(self, command: dict) -> None:
        """명령 한 건을 처리한다. 칸 이동·원상복귀·조그는 명령을 처리하는 동안만 팔로워 포트를 연다.

        포트를 계속 쥐고 있지 않아야 다음에 집기를 맡는 LeRobot 정책이 같은 포트를 열 수 있다.
        """
        if command["operation"] == "PICK_WITH_POLICY":
            self.handle_policy(command)
            return
        target = command["target"]
        if command["operation"] == "RETURN_HOME":
            where = "home"
        elif command["operation"] == "JOG_JOINT":
            where = f"{target.get('joint')} {target.get('direction')}"
        else:
            where = f"{target.get('pick_slot_id')} ({target.get('slot_label')})"
        print(f"[{command['command_id']}] {command['operation']} {where}", flush=True)
        try:
            with self.motion.follower():
                reason = self.check(command)
                try:
                    if reason:
                        self.send(command, "REJECTED", reason_code=reason)
                        return
                    command = self.send(command, "ACCEPTED")  # 여기서 HTTP 200을 받아야만 움직일 수 있다
                    self.move(command, self.goal)
                except Refused as refused:
                    # 관제가 거부했으면 이미 시작했을 수 있는 이동을 그 자리에서 멈춘다.
                    self.motion.hold_here()
                    print(f"  중단: 관제가 거부함 ({refused}). 더 진행하지 않습니다.", flush=True)
        except (ConnectionError, OSError, RuntimeError) as error:
            # 정책이나 텔레오퍼레이션이 포트를 쓰고 있을 수 있다. 모터에는 아무것도 보내지 않았다.
            print(f"  팔로워 포트를 열 수 없음: {error}", flush=True)
            try:
                self.send(command, "REJECTED", reason_code="FOLLOWER_PORT_UNAVAILABLE", message=str(error)[:300])
            except Refused:
                pass

    def move(self, command: dict, goal: dict[str, int]) -> None:
        """goal까지 움직이고 결과를 최종 이벤트로 보고한다. 취소 확인은 1초에 한 번만 관제에 묻는다."""
        command = self.send(command, "PROGRESS", phase="APPROACH_TARGET")
        next_check = time.monotonic() + 1.0

        def cancel_requested() -> bool:
            nonlocal next_check
            if time.monotonic() < next_check:
                return False
            next_check = time.monotonic() + 1.0
            status, body = request_json("GET", f"{self.base}/api/arm/commands/{command['command_id']}", timeout=5)
            return status == 200 and body["command"]["cancel_requested"]

        # 조그는 운영자가 바로 옆에서 누르는 한 단계 이동이라 더 느린 프로파일(속도 2, 가속 1)로 움직인다.
        is_jog = command["operation"] == "JOG_JOINT"
        result = self.motion.move(
            goal,
            cancel_requested,
            profile_velocity=2 if is_jog else None,
            profile_acceleration=1 if is_jog else None,
        )
        details = {"goal_ticks": goal, "final_ticks": result.final_ticks, "max_error_ticks": result.max_error_ticks}
        if result.outcome == "SUCCEEDED":
            self.send(command, "SUCCEEDED", result, pose_reached=True, details=details)
        elif result.outcome == "CANCELED":
            self.send(command, "CANCELED", result, phase="APPROACH_TARGET", details=details)
        else:
            self.send(command, result.outcome, result, reason_code=result.reason_code, message=result.message or None,
                      details=details)

    def send(self, command: dict, event: str, result: Outcome | None = None, **fields: object) -> dict:
        """모의 워커의 send에 실제 simulated 값을 붙이고, None인 필드는 빼서 보낸다.

        가짜 팔(--fake-follower)이나 --dry-run이면 simulated가 true가 되어 관제가 실물 결과로 오해하지 않는다.
        """
        fields["simulated"] = result.simulated if result else self.motion.simulated
        fields = {key: value for key, value in fields.items() if value is not None}
        return super().send(command, event, **fields)


def main() -> int:
    """CLI 진입점. --robot-id에 따라 팔별 환경 변수에서 포트·자세 파일·카메라 기본값을 정한다."""
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--server", default="http://127.0.0.1:8000")
    parser.add_argument("--robot-id", default="arm-01")
    robot_id = parser.parse_known_args()[0].robot_id  # 아래 팔별 기본값이 이 값에 따라 정해진다
    parser.add_argument("--worker-id", default="omx-lerobot-worker")
    parser.add_argument("--wait-s", type=float, default=20)
    parser.add_argument("--once", action="store_true", help="exit after one command")
    add_motion_arguments(parser)
    add_policy_arguments(parser, robot_id)
    parser.add_argument("--release-torque", action="store_true", help="turn arm torque off and exit (hold the arm first)")
    if arm_env(robot_id, "FOLLOWER_PORT"):
        parser.set_defaults(port=arm_env(robot_id, "FOLLOWER_PORT"))
    # 이 팔의 자세 파일(CONTROL_STATION_ARM01_OMX_POSES 또는 _LEFT_OMX_POSES). SlotMotion은 다른 보드에서 기록한
    # 자세 파일을 거부하므로, 기본값이 틀려도 엉뚱하게 움직이지 않고 멈춘다.
    if arm_env(robot_id, "OMX_POSES"):
        parser.set_defaults(poses=Path(arm_env(robot_id, "OMX_POSES")))
    args = parser.parse_args()
    finish_policy_arguments(args)
    # Worker 틀이 쓰는 모의용 값. 실물 워커에서는 의미가 없으므로 고정해 둔다.
    args.result = "succeeded"
    args.phase_delay = 0.0

    # 토크 끄기: 팔이 중력으로 떨어지므로 반드시 손으로 받친 뒤 실행한다. 그리퍼는 건드리지 않는다.
    if args.release_torque:
        from record_omx_pose import open_follower

        bus = open_follower(args.port)
        try:
            bus.sync_write("Torque_Enable", {name: 0 for name in ARM_JOINTS}, normalize=False, num_retry=3)
            print("팔 관절 토크를 껐습니다.")
        finally:
            bus.disconnect(disable_torque=False)
        return 0

    worker = LeRobotOmxWorker(args)
    try:
        return worker.run()
    except KeyboardInterrupt:
        print("\n워커 종료 (토크는 켜 둔 채로 둡니다)", flush=True)
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
