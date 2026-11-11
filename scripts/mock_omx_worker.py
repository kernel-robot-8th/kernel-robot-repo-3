#!/usr/bin/env python3
"""Mock OMX arm worker for the control-station arm command API. It never moves an arm.

It shows the worker side of the contract with the standard library only, so the same loop can
run inside the LeRobot environment on the OMX laptop:

1. long-poll   GET  /api/arm/{robot_id}/commands/next?worker_id=...&wait_s=20
2. check       slot map, validity time, cancel flag -> REJECTED with a reason code, or
3. handshake   POST /api/arm/commands/{id}/events  {"event": "ACCEPTED"}  -> move ONLY on HTTP 200
4. report      PROGRESS per phase (check cancel between phases), then one final event

Every event sets "simulated": true here. A real worker (omx_lerobot_worker.py) replays taught joint poses or
runs a trained ACT policy, sets "simulated": false, and sends SUCCEEDED only when its result was actually
confirmed (otherwise RESULT_UNKNOWN or FAILED).
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
    banner = "모의 OMX 워커 {worker_id}: {url} 대기 (팔을 움직이지 않음)"

    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.base = args.server.rstrip("/")
        raw = args.slots.read_bytes()
        self.slot_map_version = hashlib.sha256(raw).hexdigest()
        self.known_slots = {slot["slot_id"] for slot in json.loads(raw)["slots"] if slot["active"]}

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
        if command["operation"] == "RETURN_HOME":
            pass  # a real worker replays its recorded "home" pose; there is no target to check
        elif command["operation"] == "PICK_WITH_POLICY":
            pass  # the policy finds the object itself; a camera-observed slot is evidence only
        elif command["operation"] == "MOVE_TO_SLOT":
            # A real worker looks up the joint pose it taught for this slot under this slot map version.
            if target["slot_map_version"] != self.slot_map_version:
                return "SLOT_MAP_MISMATCH"
            if target["pick_slot_id"] not in self.known_slots:
                return "SLOT_UNKNOWN"
        else:
            return "NOT_IMPLEMENTED"
        if datetime.fromisoformat(command["valid_until"]) <= datetime.now(timezone.utc):
            return "EXPIRED"
        return None

    def execute_phase(self, command: dict, phase: str) -> None:
        # Real worker: replays the taught pose or runs the ACT policy.
        time.sleep(self.args.phase_delay)

    def handle(self, command: dict) -> None:
        target = command["target"]
        if command["operation"] == "RETURN_HOME":
            where = "home"
        elif target.get("pick_slot_id") is not None:
            where = f"칸 {target['pick_slot_id']} ({target['slot_label']})"
        else:
            where = "정책이 스스로 찾음"
        place = f" -> {command['place']['slot_id']}" if command.get("place") else ""
        print(f"[{command['command_id']}] {command['operation']} {target['object_class'] or ''} @ {where}{place}", flush=True)
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
            if self.args.result == "succeeded" and command["operation"] == "PICK_WITH_POLICY":
                # Like the real worker: back home, but grasp and placement are not observed.
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
        url = (
            f"{self.base}/api/arm/{self.args.robot_id}/commands/next"
            f"?worker_id={self.args.worker_id}&wait_s={self.args.wait_s:g}"
        )
        print(self.banner.format(worker_id=self.args.worker_id, url=url), flush=True)
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
