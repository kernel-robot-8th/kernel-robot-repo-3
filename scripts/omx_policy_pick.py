"""팔 하나에서 PICK_WITH_POLICY 한 번: 움직이기 전 검사, ACT 정책 실행, home으로 되돌아가기.

HTTP 워커(omx_lerobot_worker.py)와 ROS 2 action 서버(omx_pick_action_server.py)가 함께 쓰므로, 두 경로가 같은 규칙을
적용한다.
- 한 팔(arm_id)에서 녹화한 정책은 그 팔(--robot-id), 그리고 그 팔의 보드(config/arms.json의 board_serial)에서만
  돈다. 아니면 POLICY_WRONG_ARM
- 명령에 고정된 정책 파일 SHA-256과 모델이 이 PC의 정책 파일과 맞아야 한다. 아니면 POLICY_REGISTRY_MISMATCH,
  POLICY_UNKNOWN
- 수락한 뒤 정책을 시작하기 전에 팔은 자기의 기록된 home 자세로 돌아간다. 평소의 max-delta 가드를 넘는 이동은
  수락 전에 거부한다
- 취소하면 팔이 있는 자리에서 정책을 멈춘다. 토크는 켜 둔다(팔이 떨어지면 안 된다)
- 쉬는 자세로 돌아오면 팔을 기록된 home 자세에 맞춘다. 파지와 적재는 관측하지 않으므로, 가장 좋은 끝은
  pose_reached true인 RESULT_UNKNOWN / PLACE_NOT_VERIFIED다

여기서는 어느 전송 방식(HTTP/ROS 2)인지 모른다. 호출하는 쪽이 언제부터 움직여도 되는지(자기 ACCEPTED 이후),
취소를 어떻게 알아내는지, 진행을 어떻게 알릴지를 정해 준다. 결과는 HTTP 워커의 이벤트 이름(RESULT_UNKNOWN,
FAILED, CANCELED)을 쓰고, action 서버가 그것을 자기 outcome으로 바꾼다.

모의 실행(--fake-follower: 팔 없음, --dry-run: 실물 팔을 읽기만 하고 아무것도 쓰지 않음)은 정책을 돌리지 않고
카메라도 열지 않으며 정책 서버도 필요 없다. 정해진 시간만 기다린 뒤 home 정렬을 같은 방식으로 한다. 모의 결과에는
모두 모의라고 표시한다.

실물 하드웨어: 모의가 아니면 return_home_before_policy(), prepare(), run(), finish_at_home()이 팔로워 모터에 쓴다.
check()는 팔을 읽기만 한다.
"""

from __future__ import annotations

import argparse
import json
import os
import time
from dataclasses import asdict, dataclass
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any, Callable

from omx_act_policy_run import (
    DEFAULT_POLICIES, PolicyRunResult, RestWatcher, check_policy_server, load_policies, policy_cameras, run_policy,
    wait_devices_free,
)
from omx_slot_motion import ARM_JOINTS, REPOSITORY_ROOT, Rejected, SlotMotion, board_serial

# 그리퍼 카메라 기본값(Innomaker U20CAM). 관제 화면의 Joint Camera와 같은 장치다.
DEFAULT_GRIPPER_CAMERA = "/dev/v4l/by-id/usb-Innomaker_Innomaker-U20CAM-720P_SN0001-video-index0"


# 두 팔 설정 이름(arm-01 = 좌측 LEFT, arm-02 = 우측 RIGHT)과, 그 설정에서 쓰는 카메라 이름 대응.
SIDE_PREFIX = {"arm-01": "CONTROL_STATION_LEFT_", "arm-02": "CONTROL_STATION_RIGHT_"}
SIDE_NAME = {"GRIPPER_CAMERA": "JOINT_CAMERA", "OVERHEAD_CAMERA": "CEILING_CAMERA"}


def arm_env(robot_id: str, name: str) -> str:
    """arm-01이면 CONTROL_STATION_ARM01_<name>(arm-02는 ARM02_)을, 없으면 쪽 이름 CONTROL_STATION_LEFT_<name>
    (arm-02는 RIGHT_. 그리퍼 / 위 카메라는 JOINT_CAMERA / CEILING_CAMERA)을 읽는다. 둘 다 없으면 빈 문자열."""
    value = os.environ.get("CONTROL_STATION_" + robot_id.replace("-", "").upper() + "_" + name, "").strip()
    if not value and robot_id in SIDE_PREFIX:
        value = os.environ.get(SIDE_PREFIX[robot_id] + SIDE_NAME.get(name, name), "").strip()
    return value


def arm_board(robot_id: str) -> str | None:
    """config/arms.json에 기록된 이 팔의 OpenRB-150 보드 일련번호. 기록이 없으면 None."""
    try:
        document = json.loads((REPOSITORY_ROOT / "config" / "arms.json").read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None
    return next((arm.get("board_serial") for arm in document.get("arms", []) if arm.get("arm_id") == robot_id), None)


def add_policy_arguments(parser: argparse.ArgumentParser, robot_id: str) -> None:
    """PICK_WITH_POLICY 설정 인자를 더한다. 카메라 기본값은 이 팔의 CONTROL_STATION_ARMxx_* 설정에서 온다."""
    policy = parser.add_argument_group("PICK_WITH_POLICY (LeRobot ACT)")
    policy.add_argument("--policies", type=Path, default=DEFAULT_POLICIES, help="object -> trained policy file")
    policy.add_argument("--policy-server", default="127.0.0.1:8080", help="lerobot.async_inference.policy_server address")
    policy.add_argument("--overhead-camera",
                        default=arm_env(robot_id, "OVERHEAD_CAMERA") or os.environ.get("CONTROL_STATION_ACT_OVERHEAD_CAMERA", ""),
                        help="camera seeing the shelf from above, as in training "
                             "(env CONTROL_STATION_ARM01_OVERHEAD_CAMERA for arm-01, else CONTROL_STATION_ACT_OVERHEAD_CAMERA)")
    policy.add_argument("--gripper-camera",
                        default=arm_env(robot_id, "GRIPPER_CAMERA") or os.environ.get("CONTROL_STATION_JOINT_CAMERA")
                        or DEFAULT_GRIPPER_CAMERA,
                        help="camera on the gripper (env CONTROL_STATION_ARM01_GRIPPER_CAMERA, else CONTROL_STATION_JOINT_CAMERA)")
    # 300틱 ≈ 26°, 150틱 ≈ 13° (1회전 4096틱). 정지 판정은 omx_act_policy_run.RestWatcher가 한다.
    policy.add_argument("--policy-depart-ticks", type=int, default=300, help="a joint this far from home = policy started moving")
    policy.add_argument("--policy-rest-ticks", type=int, default=150, help="all joints this close to home = back at rest")
    policy.add_argument("--policy-settle-s", type=float, default=1.5, help="seconds at rest before stopping the policy")
    policy.add_argument("--policy-device", default="cuda",
                        help="where the policy server runs the model (cpu while another job fills the GPU)")
    policy.add_argument("--policy-any-start", action="store_true",
                        help="compatibility option: policy still starts from recorded home; enables a 40-tick action limit")
    policy.add_argument("--policy-max-step-ticks", type=int, default=None,
                        help="limit each policy goal to present ± this many ticks per 1/30 s step (default: none; "
                             "--policy-any-start sets 40 when omitted)")
    policy.add_argument("--simulated-run-s", type=float, default=3.0,
                        help="how long a simulated policy run (--fake-follower, --dry-run) pretends to take")


def finish_policy_arguments(args: argparse.Namespace) -> None:
    """인자를 읽은 뒤 서로 얽힌 기본값을 채운다: --policy-any-start면 한 스텝 40틱 제한을 켠다."""
    if args.policy_any_start and args.policy_max_step_ticks is None:
        args.policy_max_step_ticks = 40  # 30 Hz에서 약 1,200틱/s. 처음 보는 자세에서 나온 첫 action은 튈 수 있다


@dataclass
class PickEnd:
    """집기 한 번이 어떻게 끝났는지를 HTTP 워커의 이벤트 이름으로 나타낸다."""

    event: str                     # RESULT_UNKNOWN, FAILED, CANCELED
    reason_code: str | None = None
    message: str | None = None
    pose_reached: bool | None = None
    phase: str | None = None
    details: dict[str, Any] | None = None
    run_s: float = 0.0


class PolicyPick:
    """한 팔의 정책 집기. 검사(check) → home 복귀(return_home_before_policy) → 속도 프로파일 해제(prepare)
    → 정책 실행(run) → 결과 정리와 home 정렬(end) 순서로 호출한다. 각 단계 사이의 ACCEPTED·진행 보고는 호출하는 쪽 몫이다.
    """

    def __init__(self, motion: SlotMotion, settings: argparse.Namespace) -> None:
        self.motion = motion
        self.settings = settings
        self.policy_version, self.policies = load_policies(settings.policies)
        # 정책 파일의 카메라 역할 → 이 팔의 실제 장치 경로.
        self.camera_devices = {"OVERHEAD": settings.overhead_camera, "GRIPPER": settings.gripper_camera}

    @property
    def simulated(self) -> bool:
        """가짜 팔로워(--fake-follower)나 dry-run이면 True. 이때는 정책을 실제로 돌리지 않는다."""
        return self.motion.simulated

    def check(self, policy_ref: dict, valid_until: datetime, *, allow_simulation: bool = False,
              ) -> tuple[str | None, dict | None]:
        """(거부 이유, 정책)을 돌려준다. 팔을 읽기만 하고 아무것도 쓰지 않는다.

        `allow_simulation`이 없으면 모의 팔은 거부한다(POLICY_NEEDS_REAL_ARM). HTTP 워커가 늘 그렇게 해 왔다.
        검사는 싼 것부터 한다: 정책 파일 → 팔·보드 → home 기록 → 유효 시각 → 카메라 → 정책 서버 → home 도달 가능성.
        """
        settings = self.settings
        if self.simulated and not allow_simulation:
            return "POLICY_NEEDS_REAL_ARM", None
        if policy_ref.get("registry_version") != self.policy_version:
            return "POLICY_REGISTRY_MISMATCH", None
        policy = self.policies.get(policy_ref.get("policy_id"))
        if policy is None or policy["model"] != policy_ref.get("model"):
            return "POLICY_UNKNOWN", None
        # 한 팔에서 녹화한 정책은 그 팔에서만 돈다: 이 워커의 팔 id와, 이 워커 포트에 꽂힌 보드로 확인한다.
        if policy.get("arm_id") not in (None, settings.robot_id):
            return "POLICY_WRONG_ARM", None
        board = arm_board(settings.robot_id)
        if policy.get("arm_id") and board and not settings.fake_follower and board not in board_serial(settings.port):
            print(f"  {settings.robot_id}의 보드({board})가 아닌 포트입니다: {settings.port}", flush=True)
            return "POLICY_WRONG_ARM", None
        if "home" not in self.motion.poses["poses"]:
            return "POSE_NOT_RECORDED", None
        if valid_until <= datetime.now(timezone.utc):
            return "EXPIRED", None
        if self.simulated:
            return None, policy  # 정책을 돌리지 않으므로 카메라, 정책 서버, 시작 자세는 상관없다
        missing = [role for role, path in self.camera_devices.items() if not path or not os.path.exists(path)]
        if missing:
            print(f"  카메라 없음: {missing} {self.camera_devices}", flush=True)
            return "CAMERA_UNAVAILABLE", None
        # 관제는 이 명령을 넘겨줄 때(DISPATCHED) 이 카메라들을 닫는다. 그것을 기다린다(최대 5초).
        busy = wait_devices_free(list(self.camera_devices.values()))
        if busy:
            print(f"  다른 프로그램이 카메라를 쓰는 중: {busy}", flush=True)
            return "CAMERA_BUSY", None
        problem = check_policy_server(settings.policy_server)
        if problem:
            print(f"  {problem}", flush=True)
            return "POLICY_SERVER_UNAVAILABLE", None
        # 명령을 수락하기 전에 기억해 둔 home 자세가 도달 가능한지 미리 확인한다(예: TARGET_TOO_FAR). 워커는 수락 뒤에도
        # 다시 계획하고, 정책을 시작하기 전에 실제로 home으로 돌아간다.
        try:
            with self.motion.follower():
                self.motion.plan("home", self.motion.slot_map_version, valid_until)
        except Rejected as rejected:
            return rejected.reason_code, None
        return None, policy

    def return_home_before_policy(self, cancel_requested: Callable[[], bool]) -> PickEnd | None:
        """정책을 시작하기 전에 이 팔을 자기의 기록된 home으로 옮긴다. 도착하면 None, 도착하지 못하면 실패/취소 결과.

        ACCEPTED에 200을 받은 뒤에만 불러야 한다. 실물 팔이 움직인다(칸 이동과 같은 속도 프로파일·max-delta·시간 제한).
        """
        if self.simulated:
            return None
        if cancel_requested():
            return PickEnd("CANCELED", "OPERATOR_CANCELED", "초기자세 복귀 전에 취소되었습니다.",
                           phase="RETURNING_HOME")
        try:
            with self.motion.follower():
                goal = self.motion.plan("home", self.motion.slot_map_version,
                                        datetime.now(timezone.utc) + timedelta(seconds=60))
                outcome = self.motion.move(goal, cancel_requested)
        except Rejected as rejected:
            return PickEnd("FAILED", rejected.reason_code, str(rejected)[:300], phase="RETURNING_HOME")
        except (ConnectionError, OSError, RuntimeError) as error:
            # 모터 통신 오류: 팔이 어디까지 갔는지 모르므로 실패가 아니라 결과 불명으로 보고한다.
            return PickEnd("RESULT_UNKNOWN", "HOME_RETURN_ERROR", str(error)[:300], phase="RETURNING_HOME")

        details = {"goal_ticks": goal, "final_ticks": outcome.final_ticks,
                   "max_error_ticks": outcome.max_error_ticks, "outcome": outcome.outcome}
        if outcome.outcome == "SUCCEEDED":
            print("  정책 시작 전 초기자세 복귀 확인", flush=True)
            return None
        if outcome.outcome == "CANCELED":
            return PickEnd("CANCELED", "OPERATOR_CANCELED", "초기자세 복귀 중 취소되었습니다.",
                           phase="RETURNING_HOME", details={"home_return": details})
        return PickEnd("FAILED" if outcome.outcome == "FAILED" else "RESULT_UNKNOWN",
                       outcome.reason_code or "HOME_RETURN_FAILED", outcome.message or "초기자세 복귀 실패",
                       phase="RETURNING_HOME", details={"home_return": details})

    def prepare(self) -> None:
        """정책을 녹화 때와 같은 서보 속도(전원 기본값 0 = 프로파일 제한 없음)로 돌리게 한다. 칸/home 이동은 느린
        프로파일을 RAM에 남겨 두기 때문이다. 모의 팔이면 할 일이 없다."""
        if self.simulated:
            return
        with self.motion.follower():
            self.motion.write("Profile_Velocity", {name: 0 for name in ARM_JOINTS})
            self.motion.write("Profile_Acceleration", {name: 0 for name in ARM_JOINTS})

    def run(self, policy: dict, *, cancel_requested: Callable[[], bool],
            started: Callable[[], None] | None = None) -> tuple[PolicyRunResult, dict]:
        """팔이 쉬는 자세로 돌아오거나, 시간 제한, 또는 취소까지 정책을 돌린다. 예외는 호출한 쪽이 처리한다:
        그때 팔은 정책이 남긴 자리에서 토크를 유지하고, 사람이 가서 봐야 한다.

        (현재 run_policy에는 시간 제한이 없다. 멈추는 조건은 쉬는 자세 복귀, 취소, 실행 오류다.)"""
        settings = self.settings
        if self.simulated:
            return self._simulate(policy, cancel_requested, started)
        home = {name: int(self.motion.poses["poses"]["home"]["ticks"][name]) for name in ARM_JOINTS}
        watcher = RestWatcher(home, settings.policy_depart_ticks, settings.policy_rest_ticks, settings.policy_settle_s)
        cameras = policy_cameras(policy, self.camera_devices)
        print(f"  정책 실행: {policy['model']} · front={cameras['front']} · wrist={cameras['wrist']}", flush=True)
        result = run_policy(
            policy, port=settings.port, cameras=cameras, server_address=settings.policy_server, watcher=watcher,
            cancel_requested=cancel_requested, started=started,
            max_step_ticks=settings.policy_max_step_ticks, policy_device=settings.policy_device,
        )
        return result, self._report(policy, result)

    def _simulate(self, policy: dict, cancel_requested: Callable[[], bool],
                  started: Callable[[], None] | None) -> tuple[PolicyRunResult, dict]:
        """모의 정책 실행: 팔·카메라·정책 서버 없이 --simulated-run-s 동안 기다리며 취소만 확인한다."""
        print(f"  모의 정책 실행 {self.settings.simulated_run_s:g}초: {policy['model']} (팔·카메라·정책 서버 안 씀)", flush=True)
        begin = time.monotonic()
        if started:
            started()
        reason = "RETURNED_TO_REST"
        while time.monotonic() - begin < self.settings.simulated_run_s:
            if cancel_requested():
                reason = "CANCELED"
                break
            time.sleep(0.05)
        result = PolicyRunResult(reason, 0, message="simulated", run_s=time.monotonic() - begin)
        return result, {**self._report(policy, result), "simulated": True}

    @staticmethod
    def _report(policy: dict, result: PolicyRunResult) -> dict:
        """정책 종료 줄을 출력하고, 관제 이벤트 details에 넣을 정책 실행 기록을 만든다."""
        print(f"  정책 종료: {result.stop_reason}, {result.run_s:.1f}s, 행동 {result.actions_sent}개, "
              f"home에서 가장 멀리 {result.farthest_from_home_ticks} ticks", flush=True)
        return {"policy_id": policy["policy_id"], "model": policy["model"], **asdict(result)}

    def end(self, result: PolicyRunResult, policy_details: dict,
            returning_home: Callable[[], None] | None = None) -> PickEnd:
        """정책 실행이 무엇으로 끝났는지 정한다. 쉬는 자세로 돌아왔으면 먼저 팔을 home에 맞춘다(`returning_home`에
        그 시작을 알린다). 취소·오류면 팔을 그 자리에 둔다."""
        run_s = result.run_s
        if result.stop_reason == "RETURNED_TO_REST":
            if returning_home:
                returning_home()
            end = self.finish_at_home(policy_details)
        elif result.stop_reason == "CANCELED":
            end = PickEnd("CANCELED", phase="APPROACH_TARGET", details={"policy_run": policy_details})
        # 참고: 지금의 run_policy는 NEVER_DEPARTED와 TIMEOUT을 만들지 않는다(RETURNED_TO_REST, CANCELED, ERROR만).
        elif result.stop_reason == "NEVER_DEPARTED":
            end = PickEnd("FAILED", "POLICY_DID_NOT_MOVE", details={"policy_run": policy_details})
        elif result.stop_reason == "TIMEOUT":
            # 쉬는 자세가 아니다: 물체를 쥐고 있거나 선반 가까이 있을 수 있으므로 자동으로 home으로 옮기지 않는다.
            end = PickEnd("FAILED", "POLICY_TIMEOUT_NOT_AT_REST",
                          "정책을 멈췄고 팔은 그 자리에서 토크를 유지합니다. 현장 확인 뒤 원상복귀하세요.",
                          details={"policy_run": policy_details})
        else:
            end = PickEnd("RESULT_UNKNOWN", "POLICY_RUN_ERROR", result.message[:300] or None,
                          details={"policy_run": policy_details})
        end.run_s = run_s
        return end

    def finish_at_home(self, policy_details: dict) -> PickEnd:
        """정책이 쉬는 자세로 돌아온 뒤에는 항상 이 팔의 기록된 home 자세에 맞춘다. 실물 팔이 움직인다.

        쉬는 자세 허용 오차가 넓어서, 팔이 기록 자세에서 눈에 띄게 어긋난 채로 정책이 끝날 수 있다. 평소 이동 계획의
        max-delta와 시간 제한 가드를 그대로 두고, 적재는 확인되지 않았다고 보고한다.
        """
        time.sleep(0.5)  # 정책이 마지막으로 쓴 목표를 서보가 마저 끝내게 기다린다
        try:
            with self.motion.follower():
                goal = self.motion.plan("home", self.motion.slot_map_version,
                                        datetime.now(timezone.utc) + timedelta(seconds=60))
                outcome = self.motion.move(goal, lambda: False)
        except Rejected as rejected:
            return PickEnd("FAILED", rejected.reason_code, details={"policy_run": policy_details})
        except (ConnectionError, OSError, RuntimeError) as error:
            print(f"  정책 실행 오류: home alignment: {error}", flush=True)
            return PickEnd("RESULT_UNKNOWN", "POLICY_RUN_ERROR", f"home alignment: {error}"[:300],
                           details={"policy_run": policy_details})
        home_details = {"goal_ticks": goal, "final_ticks": outcome.final_ticks, "max_error_ticks": outcome.max_error_ticks}
        details = {"policy_run": policy_details, "home_return": home_details}
        measured_home_error = (
            max(abs(int(outcome.final_ticks[name]) - goal[name]) for name in ARM_JOINTS)
            if all(name in outcome.final_ticks for name in ARM_JOINTS) else None
        )
        # 이동이 SUCCEEDED가 아니어도(예: 이동 허용 오차 밖에서 시간 초과) 측정 오차가 --policy-rest-ticks 이내면
        # home에 돌아온 것으로 본다.
        if outcome.outcome == "SUCCEEDED" or (
            measured_home_error is not None and measured_home_error <= self.settings.policy_rest_ticks
        ):
            # 팔은 home에 있다. 파지와 적재는 관측하지 않았으므로 성공이라고 할 수 없다.
            home_details["max_error_ticks"] = measured_home_error
            home_details["tolerance_ticks"] = self.settings.policy_rest_ticks
            return PickEnd("RESULT_UNKNOWN", "PLACE_NOT_VERIFIED", "정책 동작 뒤 초기자세 허용범위 복귀. 집기·적재는 확인하지 않았습니다.",
                           pose_reached=True, details=details)
        return PickEnd("FAILED" if outcome.outcome == "FAILED" else "RESULT_UNKNOWN",
                       outcome.reason_code or "HOME_RETURN_FAILED", outcome.message or None, details=details)
