"""액션 수에 상한을 둔 LeRobot 비동기 정책 실행 한 번의 수명 관리 (POST /api/arm/policy/start|stop).

관제가 LeRobot의 async_inference 로봇 클라이언트를 자식 프로세스로 띄운다. 정책 추론 서버(policy server)는 따로 떠
있어야 하고(CONTROL_STATION_POLICY_SERVER_ADDRESS), 이 클라이언트가 카메라 영상과 관절 값을 보내고 받은 액션으로
팔을 움직인다. 기본으로는 꺼져 있고 CONTROL_STATION_POLICY_ENABLED=1이어야 켜진다.

정책은 팔 한 대에서 학습되므로(config/omx-act-policies.json의 `arm_id`), 그 팔의 팔로워 포트와 카메라(arms/config.py)
에서만 실행한다. `arm_id`가 없는 정책은 예전처럼 단일 팔 설정으로 실행한다. 한 번에 하나만 실행한다: 두 팔은 번갈아
움직인다(config/fleet-planning.json의 arms_overlap false).

주문 흐름의 집기(arms/pick.py)는 이 모듈을 쓰지 않는다. 그쪽은 팔 명령 큐를 거쳐 팔 쪽 워커가 정책을 실행한다.
"""

from __future__ import annotations

import math
import os
import signal
import subprocess
import threading
import uuid
from pathlib import Path
from typing import Any

from control_station import utc_now_iso
from control_station.arms.config import ArmConfigError, ArmRegistry, arm_devices, load_arms
from control_station.arms.policies import ActPolicyError, load_act_policies

# 정책 클라이언트가 max_action_steps(액션 상한)에 도달해 끝날 때 내는 종료 코드.
ACTION_LIMIT_EXIT_CODE = 75


def _env_int(name: str, default: int | None = None) -> int | None:
    """정수 환경 변수. 비어 있으면 default, 정수가 아니면 ValueError(서버 시작이 실패한다)."""
    raw = os.environ.get(name)
    if raw is None or not raw.strip():
        return default
    try:
        return int(raw)
    except ValueError as exc:
        raise ValueError(f"{name} must be an integer") from exc


def _policy_profiles(path: str | None) -> dict[str, dict[str, Any]]:
    """실행 프로필: config/omx-act-policies.json의 정책마다 하나. 주문 흐름이 쓰는 목록과 같다."""
    try:
        registry = load_act_policies(path)
    except ActPolicyError as exc:
        raise ValueError(str(exc)) from exc
    return {
        policy["policy_id"]: {
            "id": policy["policy_id"], "name": policy["label"], "policy_type": policy["policy_type"],
            "model": policy["model"], "task": policy["task"], "cameras": dict(policy["cameras"]),
            "arm_id": policy["arm_id"],
        }
        for policy in registry.policies.values()
    }


class PolicyRunSupervisor:
    """설정된 비동기 추론 클라이언트 하나를 엄격한 액션 상한과 함께 시작·중단하고 상태를 알려 준다.

    액션 상한에 도달해 프로세스가 끝나도 작업 성공으로 보지 않는다. LeRobot 클라이언트는 집기·적재 완료 신호를
    받지 않기 때문이다. 결과는 사람이나 카메라가 따로 확인해야 한다.

    상태(_state): DISABLED, STOPPED, STARTING, RUNNING, STOPPING, LIMIT_REACHED, INTERRUPTED, ERROR.
    프로세스 종료는 따로 감시하지 않고, snapshot()·start()·stop() 등을 부를 때 poll()로 확인한다(_refresh_locked).
    모든 상태 변경은 self._lock 안에서 한다(HTTP 요청 스레드 여러 개가 동시에 부를 수 있다).
    """

    def __init__(self, *, disabled_reason: str | None = None, arms: ArmRegistry | None = None) -> None:
        configured_enabled = os.environ.get("CONTROL_STATION_POLICY_ENABLED", "0").lower() in {
            "1",
            "true",
            "yes",
        }
        self.enabled = configured_enabled and disabled_reason is None
        self.disabled_reason = disabled_reason
        self.lerobot_dir = Path(
            os.environ.get("CONTROL_STATION_LEROBOT_DIR", str(Path.home() / "Desktop" / "lerobot"))
        ).expanduser()
        self.python = Path(
            os.environ.get(
                "CONTROL_STATION_LEROBOT_PYTHON",
                str(self.lerobot_dir / "venv" / "bin" / "python"),
            )
        ).expanduser()
        self.max_action_steps = _env_int("CONTROL_STATION_POLICY_MAX_ACTION_STEPS")
        self.server_address = os.environ.get("CONTROL_STATION_POLICY_SERVER_ADDRESS", "127.0.0.1:8080")
        self.policy_profiles = _policy_profiles(os.environ.get("CONTROL_STATION_ACT_POLICIES"))
        self.current_profile_id = next(iter(self.policy_profiles))
        if arms is None:
            try:
                arms = load_arms()
            except ArmConfigError:
                arms = None  # 그러면 팔에 묶인 정책은 "그 팔이 설정되지 않았다"고 알린다
        # 팔에 묶인 정책용 팔별 장치. 아래의 단일 팔 설정은 arm_id가 없는 정책용이다.
        self.arm_devices = arm_devices(arms.arms if arms is not None else ())
        self._run_arm: dict[str, str | None] | None = None  # 실행 중인(또는 마지막) 실행의 팔
        self._run_devices: list[str] = []
        self.robot_port = os.environ.get("CONTROL_STATION_POLICY_ROBOT_PORT", "/dev/omx_follower")
        self.robot_id = os.environ.get("CONTROL_STATION_POLICY_ROBOT_ID", "omx_follower_arm")
        # 카메라 역할별 장치. 정책마다 `front`·`wrist` 입력에 어느 역할을 넣을지 정해 두므로(일부는 둘을 바꿔
        # 녹화했다), 장치는 _command에서 정책별로 고른다.
        # FRONT/WRIST는 이 설정의 예전 이름이다: front는 탑(overhead) 카메라, wrist는 그리퍼 카메라였다.
        self.camera_devices = {
            "OVERHEAD": os.environ.get("CONTROL_STATION_POLICY_OVERHEAD_CAMERA")
            or os.environ.get("CONTROL_STATION_POLICY_FRONT_CAMERA", "/dev/video7"),
            "GRIPPER": os.environ.get("CONTROL_STATION_POLICY_GRIPPER_CAMERA")
            or os.environ.get("CONTROL_STATION_POLICY_WRIST_CAMERA", "/dev/video9"),
        }
        # 아래는 LeRobot async 클라이언트 인자 그대로다. chunk_size_threshold(0–1)는 남은 액션 큐가 묶음 크기의 이 비율
        # 아래로 줄면 새 관측을 보내 다음 묶음을 받는 기준이고, aggregate_fn_name은 겹치는 묶음의 액션을 합치는 방법이다.
        self.policy_device = os.environ.get("CONTROL_STATION_POLICY_DEVICE", "cuda")
        self.fps = _env_int("CONTROL_STATION_POLICY_FPS", 30)
        self.actions_per_chunk = _env_int("CONTROL_STATION_POLICY_ACTIONS_PER_CHUNK", 100)
        self.chunk_size_threshold = os.environ.get("CONTROL_STATION_POLICY_CHUNK_SIZE_THRESHOLD", "0.7")
        try:
            chunk_threshold = float(self.chunk_size_threshold)
        except ValueError as exc:
            raise ValueError("CONTROL_STATION_POLICY_CHUNK_SIZE_THRESHOLD must be a number") from exc
        if not math.isfinite(chunk_threshold) or not 0 <= chunk_threshold <= 1:
            raise ValueError("CONTROL_STATION_POLICY_CHUNK_SIZE_THRESHOLD must be between 0 and 1")
        if self.fps is None or self.fps <= 0:
            raise ValueError("CONTROL_STATION_POLICY_FPS must be positive")
        if self.actions_per_chunk is None or self.actions_per_chunk <= 0:
            raise ValueError("CONTROL_STATION_POLICY_ACTIONS_PER_CHUNK must be positive")
        self.aggregate_fn_name = os.environ.get(
            "CONTROL_STATION_POLICY_AGGREGATE_FN_NAME", "weighted_average"
        )
        self.log_path = Path(
            os.environ.get("CONTROL_STATION_POLICY_LOG", ".control-station/logs/policy-run.log")
        )
        self._active_log_path = self.log_path
        self._run_id: str | None = None
        self._lock = threading.RLock()
        self._process: subprocess.Popen[bytes] | None = None
        self._log_file: Any = None
        self._state = "DISABLED" if not self.enabled else "STOPPED"
        self._detail = self.disabled_reason or (
            "정책 실행이 비활성화되어 있습니다."
            if not self.enabled
            else "학습 정책 실행을 시작하지 않았습니다."
        )
        self._updated_at = utc_now_iso()
        self._exit_code: int | None = None

    def run_config(self, profile: dict[str, Any]) -> dict[str, Any]:
        """프로필 하나의 실행 설정: 팔, 팔로워 포트, LeRobot robot id, 역할별 카메라 장치.

        팔에 묶인 정책인데 그 팔이 config/arms.json에 없거나 장치 설정이 모자라면 RuntimeError다."""
        arm_id = profile.get("arm_id")
        if arm_id is None:
            return {"arm_id": None, "arm_label": None, "port": self.robot_port, "robot_id": self.robot_id,
                    "cameras": dict(self.camera_devices)}
        devices = self.arm_devices.get(arm_id)
        if devices is None:
            raise RuntimeError(f"{profile['name']} 정책은 {arm_id}에서 학습됐는데 config/arms.json에 그 팔이 없습니다.")
        if not devices.ready:
            raise RuntimeError(
                f"{profile['name']} 정책은 {devices.label}({arm_id}) 전용입니다. "
                f"이 팔의 장치 설정이 없습니다: {', '.join(devices.missing)}"
            )
        return {"arm_id": arm_id, "arm_label": devices.label, "port": devices.follower_port,
                "robot_id": devices.robot_id, "cameras": dict(devices.cameras)}

    def devices_for(self, profile_id: str | None = None) -> list[str]:
        """이 프로필로 실행하면 열게 될 카메라들. 이 PC에서 실행할 수 없는 프로필이면 []."""
        profile = self.policy_profiles.get(profile_id or self.current_profile_id)
        if profile is None:
            return []
        try:
            cameras = self.run_config(profile)["cameras"]
        except RuntimeError:
            return []
        return [cameras[role] for role in ("OVERHEAD", "GRIPPER") if cameras.get(role)]

    def active_devices(self) -> list[str]:
        """살아 있는 정책 프로세스가 쓰는 카메라들(그동안 관제는 이 카메라를 닫아 둔다). 카메라 관리자가 콜백으로 부른다."""
        with self._lock:
            self._refresh_locked()
            return list(self._run_devices) if self._process is not None else []

    def _command(self, profile: dict[str, Any]) -> list[str]:
        """자식 프로세스 명령줄. 액션 상한이 없으면 시작하지 않는다(정책이 끝없이 팔을 움직이지 않도록)."""
        if self.max_action_steps is None or self.max_action_steps <= 0:
            raise RuntimeError(
                "CONTROL_STATION_POLICY_MAX_ACTION_STEPS에 양의 정수 액션 상한을 설정해야 합니다."
            )
        if not self.python.is_file():
            raise RuntimeError(f"LeRobot Python 실행 파일을 찾을 수 없습니다: {self.python}")
        if not self.lerobot_dir.is_dir():
            raise RuntimeError(f"LeRobot 디렉터리를 찾을 수 없습니다: {self.lerobot_dir}")
        config = self.run_config(profile)
        front = config["cameras"][profile["cameras"]["front"]]
        wrist = config["cameras"][profile["cameras"]["wrist"]]
        cameras = (
            "{ front: {type: opencv, index_or_path: "
            f"{front}, width: 640, height: 480, fps: {self.fps}, fourcc: MJPG}}, "
            "wrist: {type: opencv, index_or_path: "
            f"{wrist}, width: 640, height: 480, fps: {self.fps}, fourcc: MJPG}} }}"
        )
        return [
            str(self.python),
            "-c",
            (
                "import lerobot.cameras.opencv.configuration_opencv; "
                "import lerobot.robots.omx_follower.config_omx_follower; "
                "from lerobot.async_inference.robot_client import async_client; async_client()"
            ),
            "--robot.type=omx_follower",
            f"--robot.port={config['port']}",
            f"--robot.id={config['robot_id']}",
            f"--robot.cameras={cameras}",
            # LeRobot 기본값은 연결을 끊을 때 토크를 끈다. 그러면 클라이언트가 끝날 때 팔이 힘을 잃고 떨어진다.
            "--robot.disable_torque_on_disconnect=false",
            f"--task={profile['task']}",
            f"--server_address={self.server_address}",
            f"--policy_type={profile['policy_type']}",
            f"--pretrained_name_or_path={profile['model']}",
            f"--policy_device={self.policy_device}",
            f"--fps={self.fps}",
            f"--actions_per_chunk={self.actions_per_chunk}",
            f"--max_action_steps={self.max_action_steps}",
            f"--chunk_size_threshold={self.chunk_size_threshold}",
            f"--aggregate_fn_name={self.aggregate_fn_name}",
            # True면 종료할 때 matplotlib 창이 떠서, 창을 닫을 때까지 프로세스가 끝나지 않는다.
            "--debug_visualize_queue_size=false",
        ]

    def _refresh_locked(self) -> None:
        """프로세스가 끝났는지 확인하고 종료 코드로 상태를 정한다. self._lock을 잡은 채로 불러야 한다.

        중단 요청(STOPPING) 뒤의 0·SIGINT 종료는 INTERRUPTED, 액션 상한 종료 코드는 LIMIT_REACHED, 그 밖은 ERROR다."""
        if self._process is None:
            return
        code = self._process.poll()
        if code is None:
            if self._state != "STOPPING":
                self._state = "RUNNING"
                self._detail = (
                    f"정책 클라이언트 실행 중 · 총 액션 상한 {self.max_action_steps} · "
                    "집기/배치 성공 여부는 확인되지 않았습니다."
                )
            return

        was_stopping = self._state == "STOPPING"
        self._exit_code = code
        profile = self.policy_profiles[self.current_profile_id]
        if code == ACTION_LIMIT_EXIT_CODE:
            self._state = "LIMIT_REACHED"
            self._detail = (
                f"{profile['name']} 정책이 설정된 액션 상한 {self.max_action_steps}에 도달했습니다. "
                "정책 프로세스는 종료됐으며 집기/배치 결과를 별도로 확인해야 합니다."
            )
        elif was_stopping and code in (0, -signal.SIGINT, 128 + signal.SIGINT):
            self._state = "INTERRUPTED"
            self._detail = "중단 요청으로 클라이언트가 종료됐습니다. 실제 동작 결과는 미확정이므로 로봇 상태를 확인하세요."
        else:
            self._state = "ERROR"
            self._detail = f"정책 클라이언트가 종료 코드 {code}로 끝났습니다. 로그와 실제 로봇 상태를 확인하세요."
        self._process = None
        if self._log_file is not None:
            self._log_file.close()
            self._log_file = None
        self._updated_at = utc_now_iso()

    def snapshot(self) -> dict[str, object]:
        """화면용 현재 상태. process_alive는 다른 곳(카메라 넘기기, 재고 확인 중지)에서도 쓴다."""
        with self._lock:
            self._refresh_locked()
            return {
                "state": self._state,
                "enabled": self.enabled,
                "process_alive": self._process is not None,
                "detail": self._detail,
                "updated_at": self._updated_at,
                "exit_code": self._exit_code,
                "max_action_steps": self.max_action_steps,
                "run_id": self._run_id,
                "current_profile_id": self.current_profile_id,
                # 실행 중이거나 마지막 실행의 팔(None: 단일 팔 설정), 그리고 팔마다 실행 준비가 됐는지.
                "current_arm": dict(self._run_arm) if self._run_arm else None,
                "arms": [devices.summary() for devices in self.arm_devices.values()],
                "current_profile": self.policy_profiles[self.current_profile_id],
                "policy_profiles": list(self.policy_profiles.values()),
                "server_address": self.server_address,
                "model": self.policy_profiles[self.current_profile_id]["model"],
                "task": self.policy_profiles[self.current_profile_id]["task"],
                "log_path": str(self._active_log_path),
            }

    def start(self, profile_id: str | None = None) -> dict[str, object]:
        """정책 클라이언트를 시작한다. 꺼져 있거나 이미 실행 중이거나 설정이 모자라면 RuntimeError(API는 409).

        로그는 실행마다 새 파일(policy-run-{run_id}.log)에 남긴다. start_new_session=True로 새 프로세스 그룹을 만들어,
        중단할 때 os.killpg로 클라이언트와 그 자식 프로세스에 함께 SIGINT를 보낸다."""
        with self._lock:
            self._refresh_locked()
            if not self.enabled:
                raise RuntimeError(
                    self.disabled_reason
                    or "정책 실행이 비활성화되어 있습니다. CONTROL_STATION_POLICY_ENABLED=1 설정이 필요합니다."
                )
            if self._process is not None:
                raise RuntimeError("정책 실행 프로세스가 이미 실행 중입니다.")
            selected_profile_id = profile_id or self.current_profile_id
            profile = self.policy_profiles.get(selected_profile_id)
            if profile is None:
                raise RuntimeError(f"등록되지 않은 정책 프로필입니다: {selected_profile_id}")
            command = self._command(profile)
            config = self.run_config(profile)
            self.current_profile_id = selected_profile_id
            self._run_arm = {"arm_id": config["arm_id"], "label": config["arm_label"]}
            self._run_devices = [config["cameras"][role] for role in ("OVERHEAD", "GRIPPER") if config["cameras"].get(role)]
            self._run_id = uuid.uuid4().hex
            self._active_log_path = self.log_path.with_name(
                f"{self.log_path.stem}-{self._run_id}{self.log_path.suffix}"
            )
            self._active_log_path.parent.mkdir(parents=True, exist_ok=True)
            self._log_file = self._active_log_path.open("ab", buffering=0)
            # LeRobot 소스 위치는 설치 방식에 따라 두 가지라서, lerobot 패키지가 있는 쪽을 PYTHONPATH 앞에 넣는다.
            source_root = self.lerobot_dir / "src" / "lerobot" / "src"
            if not (source_root / "lerobot").is_dir():
                source_root = self.lerobot_dir / "src"
            environment = os.environ.copy()
            existing_pythonpath = environment.get("PYTHONPATH")
            environment["PYTHONPATH"] = os.pathsep.join(
                item for item in (str(source_root), existing_pythonpath) if item
            )
            try:
                self._process = subprocess.Popen(
                    command,
                    cwd=self.lerobot_dir,
                    env=environment,
                    stdin=subprocess.DEVNULL,
                    stdout=self._log_file,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
            except (OSError, ValueError) as exc:
                self._log_file.close()
                self._log_file = None
                self._state = "ERROR"
                self._detail = f"정책 클라이언트를 시작하지 못했습니다: {exc}"
                self._updated_at = utc_now_iso()
                raise RuntimeError(self._detail) from exc
            self._state = "STARTING"
            on_arm = f"{config['arm_label']}({config['arm_id']})에서 " if config["arm_id"] else ""
            self._detail = (
                f"{on_arm}{profile['name']} 정책 시작 요청됨 · 최대 {self.max_action_steps}개 액션 · "
                "집기/배치 완료 여부는 별도 확인이 필요합니다."
            )
            self._exit_code = None
            self._updated_at = utc_now_iso()
            return self.snapshot()

    def stop(self) -> dict[str, object]:
        """중단 신호(SIGINT, Ctrl+C와 같음)를 보낸다. 바로 멈췄다는 뜻이 아니며, 실제 종료는 다음 상태 확인 때 반영된다."""
        with self._lock:
            self._refresh_locked()
            if self._process is None:
                return self.snapshot()
            self._state = "STOPPING"
            self._detail = "중단 신호를 보냈습니다. 실제 로봇 동작 결과는 확인 전까지 미확정입니다."
            self._updated_at = utc_now_iso()
            try:
                os.killpg(self._process.pid, signal.SIGINT)
            except ProcessLookupError:
                self._refresh_locked()
            return self.snapshot()

    def close(self) -> None:
        """서버 종료 시 살아 있는 정책 프로세스에 SIGINT를 보낸다(끝날 때까지 기다리지는 않는다)."""
        with self._lock:
            if self._process is not None and self._process.poll() is None:
                try:
                    os.killpg(self._process.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
            self._refresh_locked()
