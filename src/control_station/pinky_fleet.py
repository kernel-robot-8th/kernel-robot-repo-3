"""HTTP client for the Pinky team's fleet dashboard (origin/pinky_fleet, pinky_fleet_dashboard/app.py).

The dashboard owns each Pinky's mission: it starts routes, follows the robot's mission phase and issues a fresh
mission token per waiting phase. At station 5 the Pinky stops in `docking_manual` and waits for `dock_complete`,
re-anchors its map pose at station 5 in `docking_anchor` (up to 18 s, no token; failing it stops the route), then
holds in `docked_manual` until `received`, after which it backs out (`dock_exit`) and drives on. Those two operator signals are the
dashboard's POST /api/robots/<id>/operator-confirm {"action", "token"}; the dashboard writes them to the robot
over SSH, and the robot has no ROS interface for them, so the control station talks to the dashboard, not to the
robot (decided 2026-11-11).

What the answers mean here:
- 202 from operator-confirm: the dashboard accepted the signal and wrote it to the robot. It does not mean the
  Pinky moved; the order flow confirms that from the next phase in GET /api/status.
- 409: the phase or token did not match (late, duplicate, or for another Pinky) -> PINKY_SIGNAL_REJECTED.
- 502: the dashboard could not reach the robot -> PINKY_SIGNAL_NOT_DELIVERED.
- no answer: OSError -> the flow reports PINKY_FLEET_UNREACHABLE and does not resend by itself.
The dashboard listens on 127.0.0.1:8765 by default; reaching it from another PC needs the Pinky team to open it.
"""

from __future__ import annotations

import json
import threading
import time
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

# Route states of the dashboard that mean the Pinky is not on a mission.
_IDLE_ROUTE_STATES = {None, "complete", "cancelled", "failed"}
# Mission phases in which the Pinky stands on the loading spot at station 5.
STATION_PHASES = frozenset({"docking_manual", "docking_anchor", "docked_manual"})


def _fresh_topic(robot: dict[str, Any], key: str, field: str) -> float | None:
    topic = (robot.get("topics") or {}).get(key) or {}
    value = (topic.get("value") or {}).get(field) if topic.get("fresh") else None
    return float(value) if isinstance(value, (int, float)) and not isinstance(value, bool) else None


def _fresh_battery(robot: dict[str, Any]) -> tuple[float | None, float | None]:
    """(percent, volts) from the dashboard's live topics; None when missing or stale.

    The percentage is the Pinky's own (`/battery/percent`, computed from the voltage) and is used as is, including
    0 %: on 2026-11-14 Pinky C76C read 0 % at a 6.59 V dip and 64-66 % at 7.37 V a few minutes later, so a low
    reading is the robot saying its battery is low, not a missing value."""
    return _fresh_topic(robot, "battery_percent", "percent"), _fresh_topic(robot, "battery_voltage", "voltage_v")


class PinkyFleetError(RuntimeError):
    """The Pinky fleet dashboard refused a signal or answered something unusable."""

    def __init__(self, error_code: str, message: str) -> None:
        super().__init__(message)
        self.error_code = error_code


class PinkyFleetClient:
    def __init__(self, base_url: str, timeout_s: float = 3.0, status_max_age_s: float = 0.5) -> None:
        self.base_url = base_url.rstrip("/")
        self.timeout_s = timeout_s
        # Three Pinky cards and the waiting order poll the same status; share one answer for a moment.
        self.status_max_age_s = status_max_age_s
        self._cache: tuple[float, dict[str, Any]] | None = None
        self._lock = threading.Lock()

    def _request(self, method: str, path: str, body: dict[str, Any] | None = None) -> tuple[int, dict[str, Any]]:
        data = None if body is None else json.dumps(body).encode("utf-8")
        request = Request(f"{self.base_url}{path}", data=data, method=method,
                          headers={"Content-Type": "application/json"} if data is not None else {})
        try:
            with urlopen(request, timeout=self.timeout_s) as response:  # noqa: S310 (configured local URL)
                status, raw = response.status, response.read()
        except HTTPError as error:
            status, raw = error.code, error.read()
        except (URLError, TimeoutError) as error:
            raise OSError(f"핑키 관제({self.base_url})에 연결하지 못했습니다: {getattr(error, 'reason', error)}") from error
        try:
            answer = json.loads(raw.decode("utf-8") or "{}")
        except (UnicodeDecodeError, json.JSONDecodeError) as error:
            raise PinkyFleetError("PINKY_FLEET_BAD_ANSWER", f"핑키 관제 응답이 JSON이 아닙니다 (HTTP {status})") from error
        return status, answer if isinstance(answer, dict) else {}

    def _robots(self) -> dict[str, Any]:
        with self._lock:
            if self._cache and time.monotonic() - self._cache[0] < self.status_max_age_s:
                return self._cache[1]
        status, answer = self._request("GET", "/api/status")
        if status != 200 or not isinstance(answer.get("robots"), list):
            raise PinkyFleetError("PINKY_FLEET_BAD_ANSWER", f"핑키 관제 상태를 읽지 못했습니다 (HTTP {status})")
        robots = {str(robot.get("id")): robot for robot in answer["robots"] if isinstance(robot, dict)}
        with self._lock:
            self._cache = (time.monotonic(), robots)
        return robots

    def fleet_status(self, pinky_id: str) -> dict[str, Any]:
        """The Pinky's mission phase in the order flow's terms (parked, driving, docking_manual, docked_manual…)."""
        robot = self._robots().get(pinky_id)
        if robot is None:
            raise PinkyFleetError("PINKY_UNKNOWN", f"핑키 관제에 핑키 {pinky_id}가 없습니다")
        route = robot.get("route") if isinstance(robot.get("route"), dict) else {}
        route_status = route.get("status")
        if route.get("phase") and route_status not in _IDLE_ROUTE_STATES:
            phase = route["phase"]
        elif route_status in _IDLE_ROUTE_STATES:
            phase = "parked"
        else:
            phase = "driving"  # starting / running before the robot reported a mission phase
        operational = robot.get("operational") if isinstance(robot.get("operational"), dict) else {}
        battery_percent, battery_volts = _fresh_battery(robot)
        return {
            "source": "pinky_fleet", "simulated": False, "pinky_id": pinky_id, "phase": phase,
            "mission_token": route.get("mission_token"), "route_status": route_status,
            "message": route.get("message") or robot.get("message"),
            "location": route.get("current_label") or robot.get("stage"),
            "connected": operational.get("state") != "disconnected",
            "ready_for_motion": bool(operational.get("ready_for_motion")),
            "readiness_label": operational.get("label"),
            "battery_percent": battery_percent,
            "battery_volts": battery_volts,
        }

    def fleet_confirm(self, pinky_id: str, action: str, token: str | None) -> dict[str, Any]:
        status, answer = self._request(
            "POST", f"/api/robots/{pinky_id}/operator-confirm", {"action": action, "token": token}
        )
        with self._lock:
            self._cache = None  # the phase is about to change
        if status == 202 and answer.get("ok"):
            return {"accepted": True, "simulated": False, "http_status": status, "message": answer.get("message")}
        if status == 409:
            raise PinkyFleetError("PINKY_SIGNAL_REJECTED", answer.get("error") or "현재 단계·토큰과 신호가 맞지 않습니다")
        if status == 502 or (status == 202 and not answer.get("ok")):
            raise PinkyFleetError("PINKY_SIGNAL_NOT_DELIVERED", answer.get("message") or "핑키에 신호를 전달하지 못했습니다")
        raise PinkyFleetError("PINKY_FLEET_BAD_ANSWER", f"핑키 관제가 HTTP {status}로 답했습니다: {answer}")
