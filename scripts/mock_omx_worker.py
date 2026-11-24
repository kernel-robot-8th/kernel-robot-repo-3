#!/usr/bin/env python3
"""관제의 팔 명령 API에 붙는 모의 OMX 팔 워커다. 팔을 절대 움직이지 않는다.

워커 쪽 계약을 표준 라이브러리만으로 보여 주므로, 같은 루프를 OMX 노트북의 LeRobot 환경 안에서도
그대로 돌릴 수 있다.

1. long-poll   GET  /api/arm/{robot_id}/commands/next?worker_id=...&wait_s=20
2. 검사        칸 정의 버전, 유효 시각, 취소 표시 -> 문제가 있으면 reason code와 함께 REJECTED, 아니면
3. 핸드셰이크  POST /api/arm/commands/{id}/events  {"event": "ACCEPTED"}  -> HTTP 200일 때만 움직인다
4. 보고        단계마다 PROGRESS(단계 사이에 취소 확인), 그다음 최종 이벤트 하나

여기서는 모든 이벤트에 "simulated": true를 붙인다. 실물 워커(omx_lerobot_worker.py)는 기록해 둔 관절 자세를
재생하거나 학습한 ACT 정책을 실행하고, "simulated": false를 붙이며, 결과가 실제로 확인됐을 때만
SUCCEEDED를 보낸다(그 밖에는 RESULT_UNKNOWN 또는 FAILED).

사용법:
  .venv/bin/python scripts/mock_omx_worker.py --server http://127.0.0.1:8000 [--robot-id arm-01]
      [--result succeeded|failed|unknown|reject|silent] [--once]
  --result silent는 명령만 가져가고 ACCEPTED를 보내지 않아, 관제가 ACK 제한 시간 뒤 UNKNOWN으로 두는지 확인할 때 쓴다.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


# 모의로 차례대로 보고할 집기 단계 이름. 관제의 집기 단계 이름과 같다.
PHASES = ("APPROACH_TARGET", "GRASP_TARGET", "LIFT_TARGET", "MOVE_TO_DESTINATION", "PLACE_TARGET")
# JOG_JOINT로 움직일 수 있는 관절 5개. 그리퍼는 조그 대상이 아니다.
JOG_JOINTS = {"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"}


class Refused(Exception):
    """관제가 이벤트에 200이 아닌 상태로 답했다: 이 명령을 더 진행하지 않는다."""


def request_json(method: str, url: str, body: dict | None = None, timeout: float = 40) -> tuple[int, dict]:
    """JSON 요청을 보내고 (HTTP 상태, JSON 본문)을 돌려준다.

    4xx/5xx도 예외로 던지지 않고 상태 코드로 돌려준다(관제의 409 등을 호출자가 판단한다).
    연결 자체가 안 되면 URLError가 그대로 올라간다.
    """
    data = json.dumps(body).encode("utf-8") if body is not None else None
    headers = {"Content-Type": "application/json"} if body is not None else {}
    try:
        with urlopen(Request(url, data=data, headers=headers, method=method), timeout=timeout) as response:
            raw = response.read()
            return response.status, (json.loads(raw) if raw else {})
    except HTTPError as error:
        raw = error.read()
        return error.code, (json.loads(raw) if raw else {})


class Worker:
    """long-poll로 명령을 가져와 검사·수락·단계 보고·최종 보고를 하는 워커 틀.

    실물 워커(omx_lerobot_worker.py)는 이 클래스를 이어받아 check()와 handle()의 실행 부분을 바꾼다.
    """

    banner = "모의 OMX 워커 {worker_id}: {url} 대기 (팔을 움직이지 않음)"

    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.base = args.server.rstrip("/")
        # 관제와 같은 칸 정의 파일을 읽어 SHA-256을 버전으로 쓴다. 명령의 slot_map_version과 다르면 거부한다.
        raw = args.slots.read_bytes()
        self.slot_map_version = hashlib.sha256(raw).hexdigest()
        self.known_slots = {slot["slot_id"] for slot in json.loads(raw)["slots"] if slot["active"]}

    def send(self, command: dict, event: str, **fields: object) -> dict:
        """이벤트 하나를 보고하고 관제가 돌려준 최신 command를 돌려준다(cancel_requested 확인용).

        event_id는 워커가 정한다. 200이 아니면 Refused를 던져 이 명령의 진행을 멈춘다.
        """
        body = {
            "event_id": f"{event}-{uuid.uuid4().hex[:8]}",
            "robot_id": command["robot_id"],
            "worker_id": self.args.worker_id,
            "event": event,
            "simulated": True,
            "occurred_at": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
            **fields,
        }
        status, result = request_json("POST", f"{self.base}/api/arm/commands/{command['command_id']}/events", body)
        print(f"  -> {event} {fields.get('phase') or fields.get('reason_code') or ''} : HTTP {status}", flush=True)
        if status != 200:
            raise Refused(result.get("error_code") or f"HTTP {status}")
        return result["command"]

    def check(self, command: dict) -> str | None:
        """수락하기 전 워커 쪽 검사. 거부할 reason code를 돌려주거나, 통과하면 None을 돌려준다."""
        target = command["target"]
        if command["operation"] == "RETURN_HOME":
            pass  # 실물 워커는 자기가 기록한 "home" 자세를 재생한다. 검사할 목표가 없다
        elif command["operation"] == "PICK_WITH_POLICY":
            pass  # 정책이 물체를 스스로 찾는다. 카메라가 본 칸은 근거로만 쓴다
        elif command["operation"] == "MOVE_TO_SLOT":
            # 실물 워커는 이 칸 정의 버전에서 그 칸에 기록해 둔 관절 자세를 찾는다.
            if target["slot_map_version"] != self.slot_map_version:
                return "SLOT_MAP_MISMATCH"
            if target["pick_slot_id"] not in self.known_slots:
                return "SLOT_UNKNOWN"
        elif command["operation"] == "JOG_JOINT":
            if target.get("joint") not in JOG_JOINTS:
                return "JOINT_NOT_SUPPORTED"
            if target.get("direction") not in {"POSITIVE", "NEGATIVE"}:
                return "DIRECTION_NOT_SUPPORTED"
        else:
            return "NOT_IMPLEMENTED"
        if datetime.fromisoformat(command["valid_until"]) <= datetime.now(timezone.utc):
            return "EXPIRED"
        return None

    def execute_phase(self, command: dict, phase: str) -> None:
        """단계 하나를 실행한다. 모의 워커는 --phase-delay초 기다리기만 한다."""
        # 실물 워커: 기록한 자세를 재생하거나 ACT 정책을 실행한다.
        time.sleep(self.args.phase_delay)

    def handle(self, command: dict) -> None:
        """명령 한 건을 처리한다: 검사 → REJECTED 또는 ACCEPTED → 단계별 PROGRESS → 최종 이벤트."""
        target = command["target"]
        if command["operation"] == "RETURN_HOME":
            where = "home"
        elif command["operation"] == "JOG_JOINT":
            where = f"{target.get('joint')} {target.get('direction')}"
        elif target.get("pick_slot_id") is not None:
            where = f"칸 {target['pick_slot_id']} ({target['slot_label']})"
        else:
            where = "정책이 스스로 찾음"
        place = f" -> {command['place']['slot_id']}" if command.get("place") else ""
        print(f"[{command['command_id']}] {command['operation']} {target.get('object_class') or ''} @ {where}{place}", flush=True)
        if self.args.result == "silent":
            print("  (silent: ACCEPTED를 보내지 않음 -> 관제가 ACK_TIMEOUT 후 UNKNOWN 처리)", flush=True)
            return
        reason = self.check(command) or ("MOCK_REJECT" if self.args.result == "reject" else None)
        try:
            if reason:
                self.send(command, "REJECTED", reason_code=reason)
                return
            command = self.send(command, "ACCEPTED")  # 여기서 HTTP 200을 받아야만 움직일 수 있다
            if command["operation"] == "JOG_JOINT":
                if command["cancel_requested"]:
                    self.send(command, "CANCELED", phase="JOINT_JOG")
                    return
                self.execute_phase(command, "JOINT_JOG")
                command = self.send(command, "PROGRESS", phase="JOINT_JOG")
                if self.args.result == "succeeded":
                    self.send(
                        command,
                        "SUCCEEDED",
                        pose_reached=True,
                        details={
                            "simulated": True,
                            "joint": target["joint"],
                            "direction": target["direction"],
                            "step_ticks": 20,
                        },
                    )
                elif self.args.result == "failed":
                    self.send(command, "FAILED", reason_code="MOCK_JOG_FAILED")
                else:
                    self.send(command, "RESULT_UNKNOWN", reason_code="MOCK_RESULT_UNCONFIRMED")
                return
            for phase in PHASES:
                if command["cancel_requested"]:
                    self.send(command, "CANCELED", phase=phase)
                    return
                self.execute_phase(command, phase)
                command = self.send(command, "PROGRESS", phase=phase)
            if self.args.result == "succeeded" and command["operation"] == "PICK_WITH_POLICY":
                # 실물 워커와 같이: home으로 돌아왔지만 파지와 적재는 관측하지 못했다.
                # 관제는 grasp_confirmed·place_confirmed 없이는 SUCCEEDED를 받지 않는다.
                self.send(command, "RESULT_UNKNOWN", reason_code="PLACE_NOT_VERIFIED", pose_reached=True)
            elif self.args.result == "succeeded":
                self.send(command, "SUCCEEDED", pose_reached=True)
            elif self.args.result == "failed":
                self.send(command, "FAILED", reason_code="MOCK_GRASP_FAILED")
            else:
                self.send(command, "RESULT_UNKNOWN", reason_code="MOCK_RESULT_UNCONFIRMED")
        except Refused as refused:
            print(f"  중단: 관제가 거부함 ({refused}). 이 명령은 더 진행하지 않습니다.", flush=True)

    def run(self) -> int:
        """long-poll 루프. 200이면 명령 처리, 204면(대기 시간 동안 명령 없음) 바로 다시 요청한다.

        이 요청 자체가 관제 화면의 "워커 대기 중" 접속 신호가 된다. 연결이 끊기면 2초 뒤 재시도한다.
        """
        url = (
            f"{self.base}/api/arm/{self.args.robot_id}/commands/next"
            f"?worker_id={self.args.worker_id}&wait_s={self.args.wait_s:g}"
        )
        print(self.banner.format(worker_id=self.args.worker_id, url=url), flush=True)
        while True:
            try:
                # 서버가 wait_s까지 붙잡고 있으므로 소켓 제한 시간은 그보다 10초 길게 준다.
                status, body = request_json("GET", url, timeout=self.args.wait_s + 10)
            except (URLError, TimeoutError, ConnectionError) as error:
                print(f"관제 연결 실패: {error}; 2초 뒤 재시도", flush=True)
                time.sleep(2)
                continue
            if status == 200:
                self.handle(body["command"])
                if self.args.once:
                    return 0
            elif status != 204:
                print(f"폴링 오류 HTTP {status}: {body}", file=sys.stderr, flush=True)
                time.sleep(2)


def main() -> int:
    """CLI 진입점."""
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--server", default="http://127.0.0.1:8000")
    parser.add_argument("--robot-id", default="arm-01")
    parser.add_argument("--worker-id", default="mock-omx-worker")
    parser.add_argument(
        "--slots",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "config" / "shelf-slots.json",
        help="slot map whose slots this worker can pick (its SHA-256 must match the command)",
    )
    parser.add_argument(
        "--result",
        choices=("succeeded", "failed", "unknown", "reject", "silent"),
        default="succeeded",
        help="final outcome to simulate",
    )
    parser.add_argument("--phase-delay", type=float, default=0.8)
    parser.add_argument("--wait-s", type=float, default=20)
    parser.add_argument("--once", action="store_true", help="exit after one command")
    return Worker(parser.parse_args()).run()


if __name__ == "__main__":
    raise SystemExit(main())
