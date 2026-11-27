"""관제의 PICK_WITH_POLICY 명령 1건에 대해 실물 OMX 팔로워에서 LeRobot ACT 정책을 돌리고, 팔이 쉬는 자세로 돌아오면
정책을 멈춘다. omx_lerobot_worker.py가 쓴다(omx_policy_pick.py를 거치며, ROS 2 집기 action 서버도 같은 경로를 쓴다).

정책은 LeRobot 비동기 추론(async inference)으로 돈다. 정책 서버(`python -m lerobot.async_inference.policy_server`,
이 PC에서 따로 띄운다)와 LeRobot의 RobotClient로 이루어지며, RobotClient는 이 모듈이 워커 프로세스 안에서 실행한다.
ACT 정책에는 "끝났다" 신호가 없고 스스로 멈추지도 않으므로, 언제 멈출지는 이 모듈이 정한다.

- RETURNED_TO_REST: 측정한 관절값이 먼저 기록된 home 자세를 벗어났고(어느 한 관절이든 `depart_ticks` 초과),
  그 뒤 서로 다른 측정값 3개가 연속으로 home에서 `rest_ticks` 이내였다. 학습 에피소드는 쉬는 자세에서 시작해
  쉬는 자세에서 끝난다.
- CANCELED: 운영자가 명령을 취소했다.
- (그 밖에 제어 루프가 죽거나 취소 여부를 읽지 못하면 ERROR로 멈춘다.)

쉬는 자세로 돌아왔다는 것은 물체를 잡았는지, 놓았는지에 대해 아무것도 말해 주지 않는다.

멈출 때 토크는 켜 둔다(`disable_torque_on_disconnect=False`). LeRobot 기본값은 토크를 꺼서 팔이 떨어진다.
LeRobot은 정책을 실제로 돌릴 때만 import하므로, 테스트에는 LeRobot도 팔도 필요 없다.

실물 하드웨어: run_policy()만 팔을 움직인다(정책이 팔로워 모터에 목표 위치를 쓴다). 나머지 함수(정책 파일 읽기,
카메라 점유 확인, 정책 서버 Ready 확인)는 모터에 아무것도 쓰지 않는다.
"""

from __future__ import annotations

import hashlib
import json
import os
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable

# 그리퍼를 뺀 다섯 관절. 쉬는 자세 판정과 home 오차는 이 다섯 관절로만 계산한다.
ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_POLICIES = REPOSITORY_ROOT / "config" / "omx-act-policies.json"


def max_home_error_ticks(home: dict[str, int], current: dict[str, int]) -> int:
    """다섯 팔 관절 중 home과 가장 많이 차이 나는 관절의 절대 오차(틱). 시작 검사와 home 복귀 검사가 함께 쓴다."""
    return max(abs(int(current[name]) - int(home[name])) for name in ARM_JOINTS)


def load_policies(path: Path) -> tuple[str, dict[str, dict[str, Any]]]:
    """(파일의 SHA-256, policy_id별 정책)을 돌려준다. 관제는 같은 SHA-256을 명령마다 고정해 넣는다.

    워커는 명령의 SHA-256과 이 값을 비교해, 관제와 워커가 서로 다른 정책 파일을 보고 있으면 실행을 거부한다.
    """
    raw = path.read_bytes()
    document = json.loads(raw)
    if document.get("schema_version") != 1:
        raise SystemExit(f"{path}: schema_version must be 1")
    return hashlib.sha256(raw).hexdigest(), {policy["policy_id"]: policy for policy in document["policies"]}


def policy_cameras(policy: dict[str, Any], devices: dict[str, str]) -> dict[str, str]:
    """정책 입력 이름(front/wrist) → 장치 경로. 정책을 학습할 때 쓴 카메라 역할을 그대로 따른다."""
    return {name: devices[role] for name, role in policy["cameras"].items()}


@dataclass
class RestWatcher:
    """측정한 관절 틱으로 팔이 기록된 home 자세로 돌아왔는지 알아낸다.

    처음부터 home 근처에 있는 팔을 곧바로 "돌아왔다"고 보지 않도록, 먼저 `depart_ticks`보다 멀리 벗어난 적이
    있어야(departed) 한다. 벗어났다가 `rest_ticks` 이내로 3번 연속 들어오면 쉬는 자세로 돌아온 것으로 본다.
    """

    home: dict[str, int]
    depart_ticks: int = 300  # 약 26°. 이만큼 벗어나면 정책이 움직이기 시작한 것으로 본다
    rest_ticks: int = 150  # 약 13°. 다섯 관절 모두 이 안이면 쉬는 자세 근처
    departed: bool = False
    rest_samples: int = 0
    last_sample_id: int = 0
    farthest: int = 0

    def distance(self, ticks: dict[str, int]) -> int:
        """home까지의 최대 관절 오차(틱)."""
        return max_home_error_ticks(self.home, ticks)

    def update(self, sample_id: int, ticks: dict[str, int]) -> bool:
        """출발한 뒤, 서로 다른 측정값 3개가 연속으로 home 허용 오차 안에 들어오면 True.

        같은 측정값을 두 번 세지 않도록 `sample_id`가 늘어난 경우에만 센다.
        """
        if sample_id <= self.last_sample_id or any(name not in ticks for name in ARM_JOINTS):
            return False
        self.last_sample_id = sample_id
        distance = self.distance(ticks)
        self.farthest = max(self.farthest, distance)
        if distance > self.depart_ticks:
            self.departed = True
        if not self.departed or distance > self.rest_ticks:
            self.rest_samples = 0
            return False
        self.rest_samples += 1
        return self.rest_samples >= 3


@dataclass
class PolicyRunResult:
    """정책 실행 한 번의 결과. 워커가 관제에 보고하는 details에 그대로 들어간다."""

    stop_reason: str  # RETURNED_TO_REST, CANCELED, ERROR
    farthest_from_home_ticks: int
    last_commanded_ticks: dict[str, int] = field(default_factory=dict)
    last_observed_ticks: dict[str, int] = field(default_factory=dict)
    final_error_ticks: dict[str, int] = field(default_factory=dict)
    actions_sent: int = 0
    message: str = ""
    run_s: float = 0.0


def devices_in_use(paths: list[str]) -> list[str]:
    """`paths` 가운데 이 사용자의 다른 프로세스가 열고 있는 장치(V4L2는 한 프로그램만 스트리밍할 수 있다).
    다른 사용자의 프로세스는 root가 아니면 보이지 않으므로 보고되지 않는다.

    /proc/<pid>/fd의 심볼릭 링크를 모두 훑어 장치 실제 경로와 비교한다(Linux 전용).
    """
    # by-id 경로는 /dev/videoN을 가리키는 링크이므로, fd 링크와 비교하려면 실제 경로로 바꿔야 한다.
    wanted = {os.path.realpath(path) for path in paths}
    busy: set[str] = set()
    me = str(os.getpid())
    for entry in os.scandir("/proc"):
        if not entry.name.isdigit() or entry.name == me:
            continue
        try:
            for fd in os.scandir(f"/proc/{entry.name}/fd"):
                try:
                    target = os.readlink(fd.path)
                except OSError:
                    continue
                if target in wanted:
                    busy.add(target)
        except OSError:
            continue
    return sorted(busy)


def wait_devices_free(paths: list[str], timeout_s: float = 5.0) -> list[str]:
    """관제가 정책 카메라를 닫기를 기다린다. `timeout_s` 뒤에도 열려 있는 장치를 돌려준다.

    관제는 워커가 정책 명령을 가져가는 순간(DISPATCHED) 화면에 띄우던 같은 카메라를 닫는다(카메라 넘겨주기).
    """
    deadline = time.monotonic() + timeout_s
    while True:
        busy = devices_in_use(paths)
        if not busy or time.monotonic() >= deadline:
            return busy
        time.sleep(0.2)


def check_policy_server(address: str, timeout_s: float = 2.0, ready_timeout_s: float = 5.0) -> str | None:
    """정책 서버가 Ready 호출에 답하면 None, 아니면 짧은 오류 문구. 팔에는 아무것도 보내지 않는다.

    포트가 열려 있는 것만으로는 부족하다. 이전 클라이언트의 스트림이 남은 서버가 연결은 받으면서 Ready에는
    끝내 답하지 않은 적이 있고(2026-11-11), RobotClient.start()는 시간 제한 없이 Ready를 기다린다."""
    import grpc
    from lerobot.transport import services_pb2, services_pb2_grpc

    channel = grpc.insecure_channel(address)
    try:
        grpc.channel_ready_future(channel).result(timeout=timeout_s)
        services_pb2_grpc.AsyncInferenceStub(channel).Ready(services_pb2.Empty(), timeout=ready_timeout_s)
        return None
    except grpc.FutureTimeoutError:
        return f"no policy server at {address}"
    except grpc.RpcError as error:
        return f"policy server at {address} did not answer Ready: {error.code().name}"
    finally:
        channel.close()


def run_policy(
    policy: dict[str, Any],
    *,
    port: str,
    cameras: dict[str, str],
    server_address: str,
    watcher: RestWatcher,
    cancel_requested: Callable[[], bool],
    started: Callable[[], None] | None = None,
    max_step_ticks: int | None = None,
    policy_device: str = "cuda",
) -> PolicyRunResult:
    """측정으로 확인한 출발 뒤 home 복귀, 운영자 취소, 실행 오류 중 하나가 일어날 때까지 정책을 돌린다.
    끝난 뒤에도 토크는 켜져 있다. 실물 팔이 움직인다.

    `max_step_ticks`는 각 단계(1/30초)마다 관절 목표가 현재 위치에서 벗어날 수 있는 거리를 제한한다. 정책이
    학습하지 않은 자세에서 시작해 첫 action이 튈 수 있을 때 쓴다."""
    run_started_at = time.monotonic()

    from lerobot.async_inference.configs import RobotClientConfig
    from lerobot.async_inference.robot_client import RobotClient
    from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
    from lerobot.robots.omx_follower.config_omx_follower import OmxFollowerConfig

    class WatchedClient(RobotClient):
        """명령한 팔 위치와 측정한 팔 위치를 raw 틱으로 기록하는 RobotClient."""

        last_commanded_ticks: dict[str, int] = {}
        last_observed_ticks: dict[str, int] = {}
        observation_sequence = 0
        actions_sent = 0

        def control_loop_action(self, verbose: bool = False):
            action = super().control_loop_action(verbose)
            calibration = self.robot.bus.calibration
            ticks = {}
            for name in ARM_JOINTS:
                # LeRobot action은 보정 범위(range_min..range_max)를 -100..100으로 정규화한 값이라 raw 틱으로 되돌린다.
                low, high = calibration[name].range_min, calibration[name].range_max
                ticks[name] = round((float(action[f"{name}.pos"]) + 100) / 200 * (high - low) + low)
            self.last_commanded_ticks = ticks
            # 정지 판정은 명령값이 아니라 같은 팔로워 버스에서 읽은 실제 Present_Position으로 한다.
            measured = self.robot.bus.sync_read("Present_Position", normalize=False)
            self.last_observed_ticks = {name: int(measured[name]) for name in ARM_JOINTS}
            self.observation_sequence += 1
            self.actions_sent += 1
            return action

    robot = OmxFollowerConfig(
        port=port,
        id="omx_follower_arm",
        disable_torque_on_disconnect=False,
        # LeRobot은 각 목표를 현재 위치 ± 이 값으로 자른다. 단위는 LeRobot 정규화 단위다(0..4095 틱 범위 → -100..100).
        max_relative_target=None if max_step_ticks is None else max_step_ticks * 200 / 4095,
        cameras={
            name: OpenCVCameraConfig(index_or_path=Path(path), width=640, height=480, fps=30, fourcc="MJPG")
            for name, path in cameras.items()
        },
    )
    # 아래 값은 scripts/omx_act_policy_test.sh client와 같다: chunk 100개, 대기열이 70% 이하로 줄면 다음 chunk 요청,
    # 겹치는 구간은 가중 평균, 30 Hz 제어.
    config = RobotClientConfig(
        policy_type=policy["policy_type"],
        pretrained_name_or_path=policy["model"],
        robot=robot,
        actions_per_chunk=100,
        task=policy["task"],
        server_address=server_address,
        policy_device=policy_device,
        chunk_size_threshold=0.7,
        fps=30,
        aggregate_fn_name="weighted_average",
        debug_visualize_queue_size=False,
    )
    client = WatchedClient(config)  # 팔을 연결한다: LeRobot이 모터 설정을 쓰는 동안 토크가 잠깐 꺼진다
    receiver = control = None
    try:
        if not client.start():
            return PolicyRunResult("ERROR", 0, message="policy server refused the client")
        # 하나는 정책 서버에서 action chunk를 받고, 하나는 30 Hz로 관측을 보내며 action을 모터에 쓴다.
        receiver = threading.Thread(target=client.receive_actions, daemon=True)
        control = threading.Thread(target=client.control_loop, args=(policy["task"],), daemon=True)
        receiver.start()
        control.start()
        if started:
            started()
        next_cancel_check = 0.0
        reason, message = "", ""
        # 정지 감시 루프: 50 ms마다 정지 조건을 보고, 취소 여부(관제 HTTP 호출일 수 있음)는 1초에 한 번만 묻는다.
        while not reason:
            time.sleep(0.05)
            now = time.monotonic()
            if not control.is_alive():
                reason, message = "ERROR", "control loop stopped"
            elif client.last_observed_ticks and watcher.update(client.observation_sequence, client.last_observed_ticks):
                reason = "RETURNED_TO_REST"
            elif now >= next_cancel_check:
                next_cancel_check = now + 1.0
                try:
                    if cancel_requested():
                        reason = "CANCELED"
                except Exception as error:  # 취소 플래그를 읽을 수 없으면 눈먼 채로 돌리지 않고 정책을 멈춘다
                    reason, message = "ERROR", f"cancel check failed: {error}"[:300]
        final_observed = dict(client.last_observed_ticks)
        final_errors = {
            name: abs(final_observed[name] - int(watcher.home[name]))
            for name in ARM_JOINTS
            if name in final_observed and name in watcher.home
        }
        return PolicyRunResult(
            reason,
            watcher.farthest,
            last_commanded_ticks=dict(client.last_commanded_ticks),
            last_observed_ticks=final_observed,
            final_error_ticks=final_errors,
            actions_sent=client.actions_sent,
            message=message,
            run_s=time.monotonic() - run_started_at,
        )
    finally:
        client.shutdown_event.set()
        if control is not None:
            control.join(timeout=5)  # 제어 루프가 버스를 쥐고 있으므로, 연결을 끊기 전에 그 단계를 마치게 한다
        client.stop()  # robot.disconnect는 (위 설정대로) 토크를 유지하고, 카메라와 gRPC 채널은 닫힌다
        if receiver is not None:
            receiver.join(timeout=3)
