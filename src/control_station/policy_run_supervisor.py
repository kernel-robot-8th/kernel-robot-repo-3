"""Lifecycle management for one bounded LeRobot async policy run.

A policy is trained on one arm (`arm_id` in config/omx-act-policies.json) and runs only on that arm's follower port
and cameras (arm_devices.py). A policy without `arm_id` runs on the single-arm settings, as before. One run at a time:
the two arms take turns (config/fleet-planning.json arms_overlap false)."""

from __future__ import annotations

import math
import os
import signal
import subprocess
import threading
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from control_station.act_policies import ActPolicyError, load_act_policies
from control_station.arm_devices import arm_devices
from control_station.arms import ArmConfigError, ArmRegistry, load_arms


ACTION_LIMIT_EXIT_CODE = 75


def _now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def _env_int(name: str, default: int | None = None) -> int | None:
    raw = os.environ.get(name)
    if raw is None or not raw.strip():
        return default
    try:
        return int(raw)
    except ValueError as exc:
        raise ValueError(f"{name} must be an integer") from exc


def _policy_profiles(path: str | None) -> dict[str, dict[str, Any]]:
    """The run profiles: one per policy in config/omx-act-policies.json, the same list the order flow uses."""
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
    """Start one configured async inference client with a strict action budget.

    The process exit at the action limit is not treated as task success. The
    LeRobot client does not receive a grasp/place completion signal.
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
                arms = None  # arm-bound policies then report that their arm is not configured
        # Per-arm devices for arm-bound policies; the single-arm settings below are for policies without arm_id.
        self.arm_devices = arm_devices(arms.arms if arms is not None else ())
        self._run_arm: dict[str, str | None] | None = None  # the arm of the running (or last) run
        self._run_devices: list[str] = []
        self.robot_port = os.environ.get("CONTROL_STATION_POLICY_ROBOT_PORT", "/dev/omx_follower")
        self.robot_id = os.environ.get("CONTROL_STATION_POLICY_ROBOT_ID", "omx_follower_arm")
        # Devices by camera role. Each policy says which role feeds its `front` and `wrist` inputs (some were
        # recorded with the two swapped), so the device is chosen per policy in _command.
        # FRONT/WRIST are the older names of these settings: front was the overhead camera, wrist the gripper one.
        self.camera_devices = {
            "OVERHEAD": os.environ.get("CONTROL_STATION_POLICY_OVERHEAD_CAMERA")
            or os.environ.get("CONTROL_STATION_POLICY_FRONT_CAMERA", "/dev/video7"),
            "GRIPPER": os.environ.get("CONTROL_STATION_POLICY_GRIPPER_CAMERA")
            or os.environ.get("CONTROL_STATION_POLICY_WRIST_CAMERA", "/dev/video9"),
        }
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
        self._updated_at = _now()
        self._exit_code: int | None = None

    def run_config(self, profile: dict[str, Any]) -> dict[str, Any]:
        """Arm, follower port, LeRobot robot id and camera device per role for one profile."""
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
        """Cameras the run of this profile would open; [] when the profile cannot run here."""
        profile = self.policy_profiles.get(profile_id or self.current_profile_id)
        if profile is None:
            return []
        try:
            cameras = self.run_config(profile)["cameras"]
        except RuntimeError:
            return []
        return [cameras[role] for role in ("OVERHEAD", "GRIPPER") if cameras.get(role)]

    def active_devices(self) -> list[str]:
        """Cameras held by the live policy process (the control station keeps them closed meanwhile)."""
        with self._lock:
            self._refresh_locked()
            return list(self._run_devices) if self._process is not None else []

    def _command(self, profile: dict[str, Any]) -> list[str]:
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
            # LeRobot's default turns torque off on disconnect, which drops the arm when the client ends.
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
            # True opens a matplotlib window at exit that keeps the process alive until it is closed.
            "--debug_visualize_queue_size=false",
        ]

    def _refresh_locked(self) -> None:
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
        self._updated_at = _now()

    def snapshot(self) -> dict[str, object]:
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
                # The arm of the running or last run (None: the single-arm settings), and each arm's readiness.
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
                self._updated_at = _now()
                raise RuntimeError(self._detail) from exc
            self._state = "STARTING"
            on_arm = f"{config['arm_label']}({config['arm_id']})에서 " if config["arm_id"] else ""
            self._detail = (
                f"{on_arm}{profile['name']} 정책 시작 요청됨 · 최대 {self.max_action_steps}개 액션 · "
                "집기/배치 완료 여부는 별도 확인이 필요합니다."
            )
            self._exit_code = None
            self._updated_at = _now()
            return self.snapshot()

    def stop(self) -> dict[str, object]:
        with self._lock:
            self._refresh_locked()
            if self._process is None:
                return self.snapshot()
            self._state = "STOPPING"
            self._detail = "중단 신호를 보냈습니다. 실제 로봇 동작 결과는 확인 전까지 미확정입니다."
            self._updated_at = _now()
            try:
                os.killpg(self._process.pid, signal.SIGINT)
            except ProcessLookupError:
                self._refresh_locked()
            return self.snapshot()

    def close(self) -> None:
        with self._lock:
            if self._process is not None and self._process.poll() is None:
                try:
                    os.killpg(self._process.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
            self._refresh_locked()
