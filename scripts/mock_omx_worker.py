#!/usr/bin/env python3
"""Mock OMX arm worker for the control-station arm command API. It never moves an arm.

It shows the worker side of the contract with the standard library only, so the same loop can
run inside the LeRobot environment on the OMX laptop:

1. long-poll   GET  /api/arm/{robot_id}/commands/next?worker_id=...&wait_s=20
2. check       frame, validity time, cancel flag -> REJECTED with a reason code, or
3. handshake   POST /api/arm/commands/{id}/events  {"event": "ACCEPTED"}  -> move ONLY on HTTP 200
4. report      PROGRESS per phase (check cancel between phases), then one final event

Every event sets "simulated": true here. A real worker replaces `execute_phase` with IK and
LeRobot `send_action`, sets "simulated": false, and sends SUCCEEDED only when grasp and placement
were actually confirmed (otherwise RESULT_UNKNOWN or FAILED).
"""

from __future__ import annotations

import argparse
import json
import sys
import time
import uuid
from datetime import datetime, timezone
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


PHASES = ("APPROACH_TARGET", "GRASP_TARGET", "LIFT_TARGET", "MOVE_TO_DESTINATION", "PLACE_TARGET")


class Refused(Exception):
    """The control station answered an event with a non-200 status: stop this command."""


def request_json(method: str, url: str, body: dict | None = None, timeout: float = 40) -> tuple[int, dict]:
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
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.base = args.server.rstrip("/")

    def send(self, command: dict, event: str, **fields: object) -> dict:
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
        """Worker-side checks before accepting. Return a reason code to reject, or None."""
        target = command["target"]
        if target["frame_id"] != self.args.frame_id:
            return "FRAME_UNSUPPORTED"
        if datetime.fromisoformat(command["valid_until"]) <= datetime.now(timezone.utc):
            return "EXPIRED"
        # A real worker also runs IK here and rejects with IK_FAILED / OUT_OF_WORKSPACE.
        return None

    def execute_phase(self, command: dict, phase: str) -> None:
        # Real worker: IK for command["target"]["position"] in its base frame, then LeRobot send_action.
        time.sleep(self.args.phase_delay)

    def handle(self, command: dict) -> None:
        target = command["target"]
        print(
            f"[{command['command_id']}] {target['object_class']} @ {target['frame_id']} "
            f"{target['position']} m -> slot {command['place']['slot_id']}",
            flush=True,
        )
        if self.args.result == "silent":
            print("  (silent: ACCEPTED를 보내지 않음 -> 관제가 ACK_TIMEOUT 후 UNKNOWN 처리)", flush=True)
            return
        reason = self.check(command) or ("MOCK_REJECT" if self.args.result == "reject" else None)
        try:
            if reason:
                self.send(command, "REJECTED", reason_code=reason)
                return
            command = self.send(command, "ACCEPTED")  # only an HTTP 200 here allows motion
            for phase in PHASES:
                if command["cancel_requested"]:
                    self.send(command, "CANCELED", phase=phase)
                    return
                self.execute_phase(command, phase)
                command = self.send(command, "PROGRESS", phase=phase)
            if self.args.result == "succeeded":
                self.send(command, "SUCCEEDED", grasp_confirmed=True, place_confirmed=True)
            elif self.args.result == "failed":
                self.send(command, "FAILED", reason_code="MOCK_GRASP_FAILED")
            else:
                self.send(command, "RESULT_UNKNOWN", reason_code="MOCK_RESULT_UNCONFIRMED")
        except Refused as refused:
            print(f"  중단: 관제가 거부함 ({refused}). 이 명령은 더 진행하지 않습니다.", flush=True)

    def run(self) -> int:
        url = (
            f"{self.base}/api/arm/{self.args.robot_id}/commands/next"
            f"?worker_id={self.args.worker_id}&wait_s={self.args.wait_s:g}"
        )
        print(f"모의 OMX 워커 {self.args.worker_id}: {url} 대기 (팔을 움직이지 않음)", flush=True)
        while True:
            try:
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
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--server", default="http://127.0.0.1:8000")
    parser.add_argument("--robot-id", default="arm-01")
    parser.add_argument("--worker-id", default="mock-omx-worker")
    parser.add_argument("--frame-id", default="omx_link0", help="the only target frame this worker accepts")
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
