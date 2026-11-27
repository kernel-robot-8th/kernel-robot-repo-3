"""실행 구성요소 상태와 켜기: 핑키 관제(8765)·천장 캠 주차 launch·ACT 정책 서버·팔 워커.

run-control-station.sh(`python -m control_station.runtime_services ensure`)와 관제 화면 사이드바가 같은 코드를 씁니다.
핑키 관제와 주차 launch는 따로 떠서(새 세션) 관제를 꺼도 남고, 이미 켜져 있으면 다시 켜지 않습니다.
끄기·재시작은 일부러 두지 않습니다: 핑키 관제를 다시 켜면 주행 중인 핑키를 놓치므로 사람이 직접 합니다.
정책 서버와 팔 워커는 스크립트가 관제와 함께 켜므로 여기서는 상태만 봅니다.
"""

from __future__ import annotations

import json
import os
import socket
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping
from urllib.parse import urlsplit
from urllib.request import urlopen

REPO_DIR = Path(__file__).resolve().parents[2]
LOG_DIR = REPO_DIR / ".control-station" / "logs"
FLEET_RUN = REPO_DIR / "pinky_fleet_dashboard" / "run.sh"
# 관제 쪽 ROS 설정(도메인 87, LOCALHOST 탐색)을 물려받으면 핑키 관제가 와이파이 너머 핑키를 못 봅니다.
FLEET_ENV_DROP = ("ROS_AUTOMATIC_DISCOVERY_RANGE", "ROS_STATIC_PEERS", "ROS_LOCALHOST_ONLY", "ROS_DOMAIN_ID", "PYTHONPATH")
PARKING_READY_LINE = "카메라 프레임 수신 중"
WORKER_FRESH_S = 45
ARMS = (("arm-01", "왼팔", "LEFT"), ("arm-02", "오른팔", "RIGHT"))
STARTABLE = ("pinky_fleet", "parking")

_children: list[subprocess.Popen[bytes]] = []  # started here; polled so an exited one does not stay a zombie


class RuntimeStartError(Exception):
    def __init__(self, status: int, code: str, message: str) -> None:
        super().__init__(message)
        self.status, self.code, self.message = status, code, message


def _row(service_id: str, label: str, state: str, detail: str, can_start: bool = False) -> dict[str, Any]:
    """state: on(켜짐) · off(꺼짐) · warn(켜졌지만 확인 필요) · skip(이 설정에선 안 씀)"""
    return {"id": service_id, "label": label, "state": state, "detail": detail, "can_start": can_start}


def fleet_url(env: Mapping[str, str]) -> str:
    return env.get("CONTROL_STATION_PINKY_FLEET_URL", "").strip().rstrip("/")


def _fleet_is_local(url: str) -> bool:
    parts = urlsplit(url)
    return parts.hostname in {"127.0.0.1", "localhost"} and parts.port == 8765


def read_fleet_robots(url: str, timeout_s: float = 0.8) -> list[dict[str, Any]] | None:
    """핑키 관제 /api/status의 핑키 목록, 응답이 없으면 None"""
    try:
        with urlopen(f"{url}/api/status", timeout=timeout_s) as response:
            data = json.load(response)
    except (OSError, ValueError):
        return None
    robots = data.get("robots", data) if isinstance(data, dict) else data
    if isinstance(robots, dict):
        robots = list(robots.values())
    return [robot for robot in robots if isinstance(robot, dict)] if isinstance(robots, list) else []


def fleet_row(env: Mapping[str, str], robots: list[dict[str, Any]] | None | bool = False) -> dict[str, Any]:
    url = fleet_url(env)
    if not url:
        return _row("pinky_fleet", "핑키 관제", "skip", "모의 핑키 사용 중")
    if robots is False:
        robots = read_fleet_robots(url)
    if robots is None:
        where = "8765" if _fleet_is_local(url) else url
        return _row("pinky_fleet", "핑키 관제", "off", f"{where} 응답 없음", can_start=_fleet_is_local(url))
    def ids(*stages: str) -> list[str]:
        return [str(robot.get("id", "?")) for robot in robots if robot.get("stage") in stages]

    faults, idle, prepared = ids("fault"), ids("idle"), ids("prepared")
    if faults:
        return _row("pinky_fleet", "핑키 관제", "warn", f"오류 {', '.join(faults)}번 · 주행준비를 다시 누르세요")
    if idle:
        return _row("pinky_fleet", "핑키 관제", "warn", f"주행준비 전 {', '.join(idle)}번 · 준비 완료 {len(prepared)}대")
    if len(prepared) < len(robots):
        return _row("pinky_fleet", "핑키 관제", "on", f"준비 완료 {len(prepared)}/{len(robots)}대 · 나머지 준비 중")
    return _row("pinky_fleet", "핑키 관제", "on", f"핑키 {len(robots)}대 준비 완료")


def parking_running() -> bool:
    try:
        return subprocess.run(["pgrep", "-f", "ceiling_parking.launch.py"], capture_output=True, timeout=2).returncode == 0
    except (OSError, subprocess.TimeoutExpired):
        return False


def parking_row(env: Mapping[str, str], running: bool | None = None) -> dict[str, Any]:
    spot = env.get("CONTROL_STATION_PARKING_SPOT", "").strip()
    if not spot:
        return _row("parking", "천장 캠 주차", "skip", "설정 안 됨")
    if running is None:
        running = parking_running()
    return _row("parking", "천장 캠 주차", "on", spot) if running else _row("parking", "천장 캠 주차", "off", "꺼짐", can_start=True)


def _port_open(host: str, port: int, timeout_s: float = 0.3) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout_s):
            return True
    except OSError:
        return False


def _arm_configured(env: Mapping[str, str], side: str) -> bool:
    return all(env.get(f"CONTROL_STATION_{side}_{key}", "").strip()
               for key in ("FOLLOWER_PORT", "CEILING_CAMERA", "JOINT_CAMERA"))


def policy_row(env: Mapping[str, str], is_open: bool | None = None) -> dict[str, Any]:
    if not any(_arm_configured(env, side) for _, _, side in ARMS):
        return _row("policy_server", "ACT 정책 서버", "skip", "팔 설정 없음")
    host = env.get("CONTROL_STATION_POLICY_SERVER_HOST", "127.0.0.1")
    port = int(env.get("CONTROL_STATION_POLICY_SERVER_PORT", "8080"))
    if is_open is None:
        is_open = _port_open(host, port)
    return _row("policy_server", "ACT 정책 서버", "on" if is_open else "off",
                str(port) if is_open else f"{port} 응답 없음 (관제 스크립트로 다시 켜기)")


def _age_s(iso: object, now: datetime) -> float | None:
    try:
        return (now - datetime.fromisoformat(str(iso))).total_seconds()
    except ValueError:
        return None


def arm_rows(env: Mapping[str, str], workers: list[dict[str, Any]], now: datetime | None = None) -> list[dict[str, Any]]:
    now = now or datetime.now(timezone.utc)
    by_robot = {worker.get("robot_id"): worker for worker in workers}
    rows = []
    for robot_id, label, side in ARMS:
        service_id = f"worker_{robot_id}"
        worker = by_robot.get(robot_id)
        age = _age_s(worker.get("last_poll_at"), now) if worker else None
        alive = worker is not None and (worker.get("waiting_polls", 0) > 0 or (age is not None and -2 <= age <= WORKER_FRESH_S))
        if alive:
            rows.append(_row(service_id, f"{label} 워커", "on", str(worker.get("worker_id", robot_id))))
        elif not _arm_configured(env, side):
            rows.append(_row(service_id, f"{label} 워커", "skip", "꺼 둠 (포트·카메라 설정 없음)"))
        else:
            rows.append(_row(service_id, f"{label} 워커", "off", "연결 없음 (관제 스크립트로 다시 켜기)"))
    return rows


def status(env: Mapping[str, str], workers: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """화면 사이드바에 보이는 순서대로"""
    _reap()
    return [fleet_row(env), parking_row(env), policy_row(env), *arm_rows(env, workers)]


def _reap() -> None:
    _children[:] = [child for child in _children if child.poll() is None]


def _spawn(argv: list[str], log_path: Path, env: Mapping[str, str], cwd: Path) -> None:
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    with open(log_path, "ab") as log:
        child = subprocess.Popen(argv, cwd=cwd, env=dict(env), stdin=subprocess.DEVNULL, stdout=log,
                                 stderr=subprocess.STDOUT, start_new_session=True)
    _children.append(child)


def _stamp() -> str:
    return datetime.now().strftime("%Y%m%d-%H%M%S")


def start(service_id: str, env: Mapping[str, str] | None = None) -> dict[str, Any]:
    """켜기 요청만 보냄 (떠서 응답하기까지는 상태로 확인). 이미 켜져 있으면 409."""
    env = os.environ if env is None else env
    if service_id == "pinky_fleet":
        url = fleet_url(env)
        if not url or not _fleet_is_local(url):
            raise RuntimeStartError(400, "PINKY_FLEET_NOT_LOCAL", "이 PC의 핑키 관제(127.0.0.1:8765)만 켤 수 있습니다")
        if read_fleet_robots(url) is not None or _port_open("127.0.0.1", 8765):
            raise RuntimeStartError(409, "ALREADY_RUNNING", "핑키 관제가 이미 켜져 있습니다")
        log_path = LOG_DIR / f"pinky-fleet-{_stamp()}.log"
        clean = {key: value for key, value in env.items() if key not in FLEET_ENV_DROP}
        _spawn(["bash", str(FLEET_RUN)], log_path, clean, FLEET_RUN.parent)
        return {"service": service_id, "log": str(log_path), "message": "핑키 관제를 켜는 중입니다"}
    if service_id == "parking":
        spot = env.get("CONTROL_STATION_PARKING_SPOT", "").strip()
        if not spot:
            raise RuntimeStartError(400, "PARKING_NOT_CONFIGURED", "CONTROL_STATION_PARKING_SPOT 설정이 없습니다")
        if parking_running():
            raise RuntimeStartError(409, "ALREADY_RUNNING", "천장 캠 주차 launch가 이미 켜져 있습니다")
        log_path = LOG_DIR / f"parking-launch-{_stamp()}.log"
        script = ('source /opt/ros/jazzy/setup.bash && source "$1/cpp/install/setup.bash" && '
                  'exec ros2 launch parking_vision_ros ceiling_parking.launch.py repository_root:="$1" spot_id:="$2"')
        _spawn(["bash", "-c", script, "_", str(REPO_DIR), spot], log_path, env, REPO_DIR)
        return {"service": service_id, "log": str(log_path), "message": "천장 캠 주차 launch를 켜는 중입니다"}
    raise RuntimeStartError(400, "NOT_STARTABLE", f"여기서 켤 수 없는 구성요소입니다: {service_id}")


def ensure(env: Mapping[str, str] | None = None, wait_s: float = 20.0) -> int:
    """스크립트용: 꺼진 핑키 관제·주차 launch를 켜고 뜰 때까지 기다리며 상태를 한 줄씩 출력"""
    env = os.environ if env is None else env
    failed = False
    for service_id, row in (("pinky_fleet", fleet_row(env)), ("parking", parking_row(env))):
        label = row["label"]
        if row["state"] == "skip":
            continue
        if row["state"] != "off":
            print(f"{label} 재사용: {row['detail']}", flush=True)
            continue
        try:
            started = start(service_id, env)
        except RuntimeStartError as error:
            print(f"{label}: {error.message}", flush=True)
            continue
        print(f"{label} 시작 (로그: {started['log']})", flush=True)
        log_path = Path(started["log"])
        deadline = time.monotonic() + wait_s
        while time.monotonic() < deadline:
            if service_id == "pinky_fleet" and read_fleet_robots(fleet_url(env)) is not None:
                break
            if service_id == "parking" and PARKING_READY_LINE in log_path.read_text(errors="replace"):
                break
            _reap()
            if not _children or _children[-1].poll() is not None:
                break
            time.sleep(1)
        ready = (read_fleet_robots(fleet_url(env)) is not None if service_id == "pinky_fleet"
                 else PARKING_READY_LINE in log_path.read_text(errors="replace"))
        if ready:
            print(f"{label} 켜짐", flush=True)
        else:
            failed = True
            print(f"{label}이(가) 아직 준비되지 않았습니다. 로그: {log_path}", file=sys.stderr, flush=True)
            print("\n".join(log_path.read_text(errors="replace").splitlines()[-20:]), file=sys.stderr, flush=True)
    return 1 if failed else 0


if __name__ == "__main__":
    if sys.argv[1:] != ["ensure"]:
        print("사용법: python -m control_station.runtime_services ensure", file=sys.stderr)
        raise SystemExit(2)
    raise SystemExit(ensure())
