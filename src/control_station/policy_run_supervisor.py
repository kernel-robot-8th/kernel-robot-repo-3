"""Lifecycle management for one bounded LeRobot async policy run."""

from __future__ import annotations

import math
import json
import os
import re
import signal
import stat
import subprocess
import threading
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


ACTION_LIMIT_EXIT_CODE = 75
MAX_POLICY_PROFILE_CONFIG_BYTES = 256 * 1024


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


def _unique_json_object(pairs: list[tuple[str, object]]) -> dict[str, object]:
    result: dict[str, object] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def _load_policy_profiles(path: Path) -> dict[str, dict[str, str]]:
    try:
        file_stat = path.lstat()
    except OSError as exc:
        raise ValueError(f"policy profile file is unavailable: {path}") from exc
    if stat.S_ISLNK(file_stat.st_mode) or not stat.S_ISREG(file_stat.st_mode):
        raise ValueError("policy profile path must be a regular non-symlink file")
    if file_stat.st_size > MAX_POLICY_PROFILE_CONFIG_BYTES:
        raise ValueError("policy profile file exceeds 256 KiB")
    try:
        payload = json.loads(path.read_text(encoding="utf-8"), object_pairs_hook=_unique_json_object)
    except (OSError, UnicodeDecodeError, json.JSONDecodeError, ValueError) as exc:
        raise ValueError(f"policy profile JSON is invalid: {exc}") from exc
    if not isinstance(payload, dict) or set(payload) != {"schema_version", "profiles"}:
        raise ValueError("policy profile JSON must contain schema_version and profiles only")
    if payload["schema_version"] != 1 or isinstance(payload["schema_version"], bool):
        raise ValueError("policy profile schema_version must be 1")
    entries = payload["profiles"]
    if not isinstance(entries, list) or not entries:
        raise ValueError("policy profiles must be a non-empty array")
    profiles: dict[str, dict[str, str]] = {}
    required_fields = {"id", "name", "policy_type", "model", "task"}
    for entry in entries:
        if not isinstance(entry, dict) or set(entry) != required_fields:
            raise ValueError("each policy profile must contain id, name, policy_type, model, and task only")
        profile_id = entry["id"]
        if not isinstance(profile_id, str) or not re.fullmatch(r"[a-z0-9][a-z0-9_-]{0,63}", profile_id):
            raise ValueError("policy profile id must use lowercase letters, digits, '_' or '-' (max 64 chars)")
        if profile_id in profiles:
            raise ValueError(f"duplicate policy profile id: {profile_id}")
        for field in ("name", "policy_type", "model", "task"):
            value = entry[field]
            if not isinstance(value, str) or not value.strip() or len(value) > 512:
                raise ValueError(f"policy profile {field} must be a non-empty string up to 512 characters")
        profiles[profile_id] = {key: str(value).strip() for key, value in entry.items()}
    return profiles


class PolicyRunSupervisor:
    """Start one configured async inference client with a strict action budget.

    The process exit at the action limit is not treated as task success. The
    LeRobot client does not receive a grasp/place completion signal.
    """

    def __init__(self, *, disabled_reason: str | None = None) -> None:
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
        profile_path = Path(
            os.environ.get(
                "CONTROL_STATION_POLICY_PROFILES",
                str(Path(__file__).resolve().parents[2] / "config" / "policy_profiles.json"),
            )
        ).expanduser()
        self.policy_profiles = _load_policy_profiles(profile_path)
        self.current_profile_id = next(iter(self.policy_profiles))
        self.robot_port = os.environ.get("CONTROL_STATION_POLICY_ROBOT_PORT", "/dev/omx_follower")
        self.robot_id = os.environ.get("CONTROL_STATION_POLICY_ROBOT_ID", "omx_follower_arm")
        self.front_camera = os.environ.get("CONTROL_STATION_POLICY_FRONT_CAMERA", "/dev/video7")
        self.wrist_camera = os.environ.get("CONTROL_STATION_POLICY_WRIST_CAMERA", "/dev/video9")
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

    def _command(self, profile: dict[str, str]) -> list[str]:
        if self.max_action_steps is None or self.max_action_steps <= 0:
            raise RuntimeError(
                "CONTROL_STATION_POLICY_MAX_ACTION_STEPS에 양의 정수 액션 상한을 설정해야 합니다."
            )
        if not self.python.is_file():
            raise RuntimeError(f"LeRobot Python 실행 파일을 찾을 수 없습니다: {self.python}")
        if not self.lerobot_dir.is_dir():
            raise RuntimeError(f"LeRobot 디렉터리를 찾을 수 없습니다: {self.lerobot_dir}")
        cameras = (
            "{ front: {type: opencv, index_or_path: "
            f"{self.front_camera}, width: 640, height: 480, fps: {self.fps}, fourcc: MJPG}}, "
            "wrist: {type: opencv, index_or_path: "
            f"{self.wrist_camera}, width: 640, height: 480, fps: {self.fps}, fourcc: MJPG}} }}"
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
            f"--robot.port={self.robot_port}",
            f"--robot.id={self.robot_id}",
            f"--robot.cameras={cameras}",
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
            "--debug_visualize_queue_size=True",
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
            self.current_profile_id = selected_profile_id
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
            self._detail = (
                f"{profile['name']} 정책 시작 요청됨 · 최대 {self.max_action_steps}개 액션 · "
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
