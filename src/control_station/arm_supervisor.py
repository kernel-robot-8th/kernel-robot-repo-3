"""Local lifecycle control for the configured LeRobot teleoperation process."""

from __future__ import annotations

import os
import signal
import subprocess
import threading
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


def _now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


class ArmTeleopSupervisor:
    """Start/stop a fixed LeRobot profile and report process-level state.

    A running process is not proof that the arm is ready or that an operator's
    task has completed. The current LeRobot CLI does not expose those events.
    """

    def __init__(self) -> None:
        self.enabled = os.environ.get("CONTROL_STATION_ARM_ENABLED", "0").lower() in {"1", "true", "yes"}
        self.program = os.environ.get("CONTROL_STATION_ARM_PROGRAM", "lerobot-teleoperate")
        self.workdir = os.environ.get("CONTROL_STATION_LEROBOT_DIR") or None
        self.follower_port = os.environ.get("CONTROL_STATION_FOLLOWER_PORT", "/dev/omx_follower")
        self.leader_port = os.environ.get("CONTROL_STATION_LEADER_PORT", "/dev/omx_leader")
        self.follower_id = os.environ.get("CONTROL_STATION_FOLLOWER_ID", "omx_folower_arm")
        self.leader_id = os.environ.get("CONTROL_STATION_LEADER_ID", "omx_leader_arm")
        self.log_path = Path(os.environ.get("CONTROL_STATION_ARM_LOG", "data/arm-teleop.log"))
        self._lock = threading.RLock()
        self._process: subprocess.Popen[bytes] | None = None
        self._log_file: Any = None
        self._state = "DISABLED" if not self.enabled else "STOPPED"
        self._detail = "실행 설정이 비활성화되어 있습니다." if not self.enabled else "텔레오퍼레이션이 실행되지 않았습니다."
        self._updated_at = _now()
        self._exit_code: int | None = None

    def _command(self) -> list[str]:
        return [
            self.program,
            "--robot.type=omx_follower",
            f"--robot.port={self.follower_port}",
            f"--robot.id={self.follower_id}",
            "--teleop.type=omx_leader",
            f"--teleop.port={self.leader_port}",
            f"--teleop.id={self.leader_id}",
            "--display_data=false",
        ]

    def _refresh_locked(self) -> None:
        if self._process is None:
            return
        code = self._process.poll()
        if code is None:
            if self._state != "STOPPING":
                self._state = "RUNNING"
                self._detail = "프로세스 실행 중 · 로봇 준비 완료나 작업 완료를 뜻하지 않습니다."
        else:
            was_stopping = self._state == "STOPPING"
            self._exit_code = code
            stopped_cleanly = code == 0 or (was_stopping and code == -signal.SIGINT)
            self._state = "STOPPED" if stopped_cleanly else "ERROR"
            self._detail = (
                "종료 요청으로 프로세스가 종료됨" if was_stopping and stopped_cleanly
                else "정상 종료" if code == 0
                else f"프로세스가 오류 코드 {code}로 종료됨 · 로그 확인"
            )
            self._process = None
            if self._log_file is not None:
                self._log_file.close()
                self._log_file = None
            self._updated_at = _now()

    def snapshot(self) -> dict[str, object]:
        with self._lock:
            self._refresh_locked()
            return {
                "robot_id": "arm-01",
                "state": self._state,
                "process_alive": self._process is not None,
                "detail": self._detail,
                "updated_at": self._updated_at,
                "exit_code": self._exit_code,
                "enabled": self.enabled,
                "log_path": str(self.log_path),
            }

    def start(self) -> dict[str, object]:
        with self._lock:
            self._refresh_locked()
            if not self.enabled:
                raise RuntimeError("팔 제어가 비활성화되어 있습니다. CONTROL_STATION_ARM_ENABLED=1 설정이 필요합니다.")
            if self._process is not None:
                raise RuntimeError("텔레오퍼레이션 프로세스가 이미 실행 중입니다.")
            self.log_path.parent.mkdir(parents=True, exist_ok=True)
            self._log_file = self.log_path.open("ab", buffering=0)
            try:
                self._process = subprocess.Popen(
                    self._command(),
                    cwd=self.workdir,
                    stdin=subprocess.DEVNULL,
                    stdout=self._log_file,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
            except (OSError, ValueError) as exc:
                self._log_file.close()
                self._log_file = None
                self._state = "ERROR"
                self._detail = f"프로세스를 시작하지 못함: {exc}"
                self._updated_at = _now()
                raise RuntimeError(self._detail) from exc
            self._state = "STARTING"
            self._detail = "프로세스 시작 요청됨 · 장치 연결과 준비 상태를 별도로 확인해야 합니다."
            self._exit_code = None
            self._updated_at = _now()
            return self.snapshot()

    def stop(self) -> dict[str, object]:
        with self._lock:
            self._refresh_locked()
            if self._process is None:
                return self.snapshot()
            self._state = "STOPPING"
            self._detail = "종료 신호를 보냈습니다. 프로세스가 안전하게 종료되는지 확인 중입니다."
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
