"""핑키 팀 관제(핑키 관제: origin/pinky_fleet의 pinky_fleet_dashboard/app.py)에 붙는 HTTP 클라이언트.

핑키 관제가 핑키마다 임무를 갖는다. 경로를 시작하고, 로봇의 임무 단계(phase)를 따라가며, 기다리는 단계마다 새
임무 토큰(mission token)을 발급한다. 5번(적재 자리)에서 핑키는 `docking_manual`로 서서 `dock_complete`를 기다리고,
`docking_anchor`에서 5번 위치로 지도 위 자세를 다시 맞춘 뒤(최대 18초, 토큰 없음, 실패하면 경로가 멈춘다)
`docked_manual`로 서서 `received`를 기다린다. 그 뒤 뒤로 빠져나와(`dock_exit`) 다음 곳으로 간다. 이 두 운영자 신호는
핑키 관제의 POST /api/robots/<id>/operator-confirm {"action", "token"}이다. 핑키 관제가 이를 SSH로 로봇에 써서
전하고, 로봇에는 이 신호를 받는 ROS 인터페이스가 없다. 그래서 관제는 로봇이 아니라 핑키 관제와 이야기한다
(2026-11-11 결정).

응답의 뜻:
- operator-confirm의 202: 핑키 관제가 신호를 받아 로봇에 썼다. 핑키가 움직였다는 뜻은 아니다. 주문 흐름은
  GET /api/status의 다음 단계로 그것을 확인한다.
- 409: 단계나 토큰이 맞지 않음(늦었거나, 중복이거나, 다른 핑키에게 갈 신호) -> PINKY_SIGNAL_REJECTED.
- 502: 핑키 관제가 로봇에 닿지 못함 -> PINKY_SIGNAL_NOT_DELIVERED.
- 응답 없음: OSError -> 주문 흐름이 PINKY_FLEET_UNREACHABLE로 알리고 스스로 다시 보내지 않는다.
핑키 관제는 기본으로 127.0.0.1:8765에서만 열린다. 다른 PC에서 닿으려면 핑키 팀이 접속을 열어 줘야 한다.

위치: app.py가 CONTROL_STATION_PINKY_FLEET_URL이 있을 때 하나 만들고, 주문 흐름(fleet_status, fleet_confirm,
dispatch, mission_complete), 핑키 화면(pinky/gateway.py), 적재 자리 안내·자동 처리(pinky/docking.py)가 함께 쓴다.
상태(GET /api/status)는 `status_max_age_s`(기본 0.5초) 동안 캐시해 여러 호출자가 한 응답을 나눠 쓰고, 상태를 바꾸는
요청을 보내면 캐시를 비운다. 여러 스레드에서 불리므로 캐시와 부가 정보는 `_lock` 아래에서 읽고 쓴다.
"""

from __future__ import annotations

import json
import statistics
import threading
import time
from collections import deque
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

# 핑키 관제의 경로 상태 가운데 핑키가 임무 중이 아님을 뜻하는 것들(None은 경로가 아예 없음).
_IDLE_ROUTE_STATES = {None, "complete", "cancelled", "failed"}
# 핑키가 5번 적재 자리에 서 있는 임무 단계들.
STATION_PHASES = frozenset({"docking_manual", "docking_anchor", "docked_manual"})


# 핑키는 배터리를 4~5초마다 발행하는데 핑키 관제는 약 4초가 지나면 토픽을 오래됐다(stale)고 본다. 그래서 배터리 값은
# 이만큼(초) 더 오래 인정한다. 배터리는 그보다 빨리 변하지 않는다.
BATTERY_MAX_AGE_S = 15.0
# 핑키가 직접 계산한 퍼센트는 읽을 때마다 튄다(2026-11-14, 7.49 V로 일정한데 77 %였다가 99 %). 그래서 이 시간 창(초)
# 안에서 본 값들의 중앙값을 알린다.
BATTERY_WINDOW_S = 60.0


def _fresh_topic(robot: dict[str, Any], key: str, field: str, max_age_s: float | None = None) -> float | None:
    """살아 있는(fresh) 토픽 값. `max_age_s`를 주면 핑키 관제가 오래됐다고 한 값이라도 그보다 젊으면 쓴다.
    숫자가 아니면(bool 포함) None."""
    topic = (robot.get("topics") or {}).get(key) or {}
    age = topic.get("age_s")
    recent = max_age_s is not None and isinstance(age, (int, float)) and age <= max_age_s
    value = (topic.get("value") or {}).get(field) if topic.get("fresh") or recent else None
    return float(value) if isinstance(value, (int, float)) and not isinstance(value, bool) else None


def _fresh_battery(robot: dict[str, Any]) -> tuple[float | None, float | None]:
    """핑키 관제 토픽에서 읽은 (퍼센트, 볼트). 없거나 BATTERY_MAX_AGE_S보다 오래됐으면 None.

    퍼센트는 핑키 자신의 값(`/battery/percent`, 전압으로 계산)이고 0 %까지 그대로 쓴다. 2026-11-14에 핑키 C76C가
    6.59 V로 떨어졌을 때 0 %를 읽었다. 그러니 낮은 값은 빠진 값이 아니라 로봇이 배터리가 낮다고 말하는 것이다."""
    return (_fresh_topic(robot, "battery_percent", "percent", BATTERY_MAX_AGE_S),
            _fresh_topic(robot, "battery_voltage", "voltage_v", BATTERY_MAX_AGE_S))


class BatteryHistory:
    """핑키별 최근 배터리 값. 중앙값이 튀는 퍼센트를 가라앉히되, 실제로 떨어지는 것은 가리지 않는다.
    주문 흐름과 화면이 여러 스레드에서 부르므로 잠금 아래에서 다룬다."""

    def __init__(self, window_s: float = BATTERY_WINDOW_S) -> None:
        self.window_s = window_s
        self._readings: dict[str, deque[tuple[float, float]]] = {}
        self._lock = threading.Lock()

    def add(self, pinky_id: str, at: float, percent: float | None) -> float | None:
        """값을 기록하고(None이면 넣지 않음) 창 안 값의 중앙값을 돌려준다. 창이 비면 None.
        `at`은 time.monotonic() 초이고, 창보다 오래된 값은 여기서 버린다."""
        with self._lock:
            readings = self._readings.setdefault(pinky_id, deque())
            if percent is not None:
                readings.append((at, percent))
            while readings and at - readings[0][0] > self.window_s:
                readings.popleft()
            return statistics.median(value for _, value in readings) if readings else None


class PinkyFleetError(RuntimeError):
    """핑키 관제가 신호를 거절했거나 쓸 수 없는 답을 했다. error_code는 주문 흐름과 API가 그대로 쓰는 코드다."""

    def __init__(self, error_code: str, message: str) -> None:
        super().__init__(message)
        self.error_code = error_code


class PinkyFleetClient:
    """핑키 관제 하나에 붙는 클라이언트. 읽기(상태)는 잠깐 캐시하고, 명령은 (HTTP 상태, 응답 JSON)으로 돌려준다."""

    def __init__(self, base_url: str, timeout_s: float = 3.0, status_max_age_s: float = 0.5) -> None:
        self.base_url = base_url.rstrip("/")
        self.timeout_s = timeout_s  # 요청 하나의 기본 시간 제한(초)
        self._battery = BatteryHistory()
        self._extras: dict[str, Any] = {}
        # 핑키 카드 세 장과 기다리는 주문이 같은 상태를 계속 묻는다. 한 응답을 잠깐(초) 나눠 쓴다.
        self.status_max_age_s = status_max_age_s
        self._cache: tuple[float, dict[str, Any]] | None = None  # (받은 monotonic 시각, 핑키 id -> 레코드)
        self._lock = threading.Lock()
        # 가상 함대(simulation/virtual_pinky_fleet)는 모든 응답에 "simulated": true를 싣는다. 실제 핑키 관제는
        # 아무 말도 하지 않으므로 그 핑키들은 실제로 남는다.
        self.simulated = False

    def _request(self, method: str, path: str, body: dict[str, Any] | None = None,
                 timeout_s: float | None = None) -> tuple[int, dict[str, Any]]:
        """HTTP 요청 하나를 보내 (상태 코드, JSON 본문 dict)를 돌려준다.

        4xx/5xx도 예외가 아니라 상태 코드로 돌려준다(HTTPError의 본문을 읽는다). 연결 실패·시간 초과는 OSError,
        JSON이 아닌 응답은 PinkyFleetError(PINKY_FLEET_BAD_ANSWER)로 올린다. 본문이 dict가 아니면 빈 dict.
        """
        data = None if body is None else json.dumps(body).encode("utf-8")
        request = Request(f"{self.base_url}{path}", data=data, method=method,
                          headers={"Content-Type": "application/json"} if data is not None else {})
        try:
            with urlopen(request, timeout=timeout_s or self.timeout_s) as response:  # noqa: S310 (configured local URL)
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
        """GET /api/status를 {핑키 id: 레코드}로 돌려준다. status_max_age_s 안의 캐시가 있으면 그것을 준다.

        요청은 잠금 밖에서 보내 다른 스레드를 막지 않는다(그래서 캐시가 빌 때 두 스레드가 함께 요청할 수는 있다).
        """
        with self._lock:
            if self._cache and time.monotonic() - self._cache[0] < self.status_max_age_s:
                return self._cache[1]
        status, answer = self._request("GET", "/api/status")
        if status != 200 or not isinstance(answer.get("robots"), list):
            raise PinkyFleetError("PINKY_FLEET_BAD_ANSWER", f"핑키 관제 상태를 읽지 못했습니다 (HTTP {status})")
        robots = {str(robot.get("id")): robot for robot in answer["robots"] if isinstance(robot, dict)}
        with self._lock:
            self._cache = (time.monotonic(), robots)
            self.simulated = answer.get("simulated") is True
            self._extras = {"keepout": answer.get("keepout"), "traffic": answer.get("traffic"),
                            "password_set": answer.get("password_set"), "busy_robot": answer.get("busy_robot")}
        return robots

    def extras(self) -> dict[str, Any]:
        """마지막 상태 응답의 함대 전체 부분: keepout 마스크와 공유 구역 예약(traffic). 핑키 관제가 SSH 비밀번호를
        갖고 있는지(password_set)와 바쁜 로봇(busy_robot)도 함께 준다."""
        self._robots()
        with self._lock:
            return dict(self._extras)

    def robots(self) -> dict[str, Any]:
        """핑키 id -> 핑키 관제 자신의 레코드(경로, 운행 상태, 실시간 토픽). 잠깐 동안 공유된다."""
        return self._robots()

    def stop(self, pinky_id: str) -> tuple[int, dict[str, Any]]:
        """핑키 관제에 이 핑키를 멈춰 달라고 한다. 핑키 관제는 로봇에 정지를 걸어 두고(latch) Nav2 목표를 취소한다.
        202 = 받음(아직 적용 전), 401 = 아직 SSH 비밀번호 없음, 409 = 앞선 정지를 아직 적용하는 중."""
        return self._send("POST", f"/api/robots/{pinky_id}/route/cancel", {})

    def prepare(self, pinky_id: str, map_name: str) -> tuple[int, dict[str, Any]]:
        """주행준비: 고른 지도로 bringup, Nav2, 로봇 쪽 안전 감시기를 띄운 뒤, AMCL에 그 핑키 자신의 주차 자세를
        준다(핑키가 거기 서 있어야 한다)."""
        return self._send("POST", f"/api/robots/{pinky_id}/ready", {"map": map_name})

    def prepare_all(self, map_name: str) -> tuple[int, dict[str, Any]]:
        """아직 준비되지 않은 모든 핑키를 차례로 주행준비한다. 각각 마지막에 자기 주차 자리에서 위치를 잡는다.
        202 = 시작됨(응답에 `robots`와 `skipped` 목록), 401 = SSH 비밀번호 없음, 409 = 바쁘거나 할 일 없음."""
        return self._send("POST", "/api/prepare-all", {"map": map_name})

    def initial_pose(self, pinky_id: str) -> tuple[int, dict[str, Any]]:
        """AMCL 자세를 그 핑키 자신의 주차 자리로 정한다. 핑키가 거기 서 있어야 한다."""
        return self._send("POST", f"/api/robots/{pinky_id}/initial-pose", {})

    def localize(self, pinky_id: str, pose: dict[str, float] | None = None) -> tuple[int, dict[str, Any]]:
        """라이다로 지도 위에서 핑키를 찾아 그 자세를 AMCL에 준다(SSH 비밀번호 필요 없음, Nav2가 돌고 있어야 함).
        `pose`(locate가 준 후보)를 주면 그 자리를 쓴다. 202 = 시작됨. 결과는 로봇 레코드의 `localization`과
        message에 나온다."""
        return self._send("POST", f"/api/robots/{pinky_id}/localize", {} if pose is None else {"pose": pose})

    def set_pose(self, pinky_id: str, pose: dict[str, float]) -> tuple[int, dict[str, Any]]:
        """운영자가 정한 핑키 자세(RViz의 2D Pose Estimate와 같은 것). 라이다는 30 cm·30도 안에서만 다듬고, 그 뒤
        AMCL에 준다. 경로가 도는 동안은 거절(409). 202 = 시작됨."""
        return self._send("POST", f"/api/robots/{pinky_id}/set-pose", {"pose": pose})

    def locate(self, pinky_id: str) -> tuple[int, dict[str, Any]]:
        """읽기 전용: 지금 라이다가 핑키를 놓는 자리와, 들어맞는 다른 자리들(AMCL은 건드리지 않음)."""
        # 새 스캔을 기다린 뒤 스캔을 지도 전체와 맞추는 데 1초쯤 걸린다. 그래서 시간 제한을 15초로 넉넉히 둔다.
        return self._request("GET", f"/api/robots/{pinky_id}/locate", timeout_s=15.0)

    def start_test_mission(self, pinky_id: str, dropoff: str, via: list[str] | None = None) -> tuple[int, dict[str, Any]]:
        """주문 없는 시험 주행: 핑키가 선 곳에서 5번(도킹, 적재)으로, 내려 줄 웨이포인트(전달)로, 자기 주차 자리로
        돌아온다. 정류장 사이는 Nav2가 계획한다. 202 = 시작됨(움직였다는 뜻은 아님)."""
        return self._send("POST", f"/api/robots/{pinky_id}/mission", {"dropoff": dropoff, "via": list(via or [])})

    def nav_log(self, pinky_id: str) -> tuple[int, dict[str, Any]]:
        """읽기 전용, 개발자용: 핑키 Nav2 로그의 오류 줄과 끝부분(SSH로 읽으므로 시간 제한 60초)."""
        return self._request("GET", f"/api/robots/{pinky_id}/nav-log", timeout_s=60.0)

    def clear_route(self, pinky_id: str) -> tuple[int, dict[str, Any]]:
        """기존 경로 날리기: 도는 경로가 있으면 먼저 멈추고, 경로와 그 줄들, 집으로 갈 대기까지 버린다."""
        return self._send("POST", f"/api/robots/{pinky_id}/route/clear", {})

    def return_home(self, pinky_id: str, via: list[str] | None = None) -> tuple[int, dict[str, Any]]:
        """핑키가 선 곳에서 `via`를 거쳐 자기 주차 자리로 간다(Nav2). 202 = 시작됨."""
        return self._send("POST", f"/api/robots/{pinky_id}/return-home", {"via": list(via or [])})

    def reset(self, pinky_id: str, map_name: str) -> tuple[int, dict[str, Any]]:
        """초기 세팅: 정지를 걸어 두고, 경로를 버린 뒤, 이미 준비됐더라도 주행준비를 처음부터 한다. 202 = 시작됨."""
        return self._send("POST", f"/api/robots/{pinky_id}/reset", {"map": map_name})

    def nav_restart(self, pinky_id: str) -> tuple[int, dict[str, Any]]:
        """Nav2 액션 서버가 없을 때 Nav2를 다시 띄운다. 경로가 도는 동안은 거절된다."""
        return self._send("POST", f"/api/robots/{pinky_id}/nav/restart", {})

    def record_waypoint(self, pinky_id: str, label: str) -> tuple[int, dict[str, Any]]:
        """핑키의 지금 지도 자세를 이름 붙인 웨이포인트로 저장한다(새 지도와 자세가 필요)."""
        return self._send("POST", f"/api/robots/{pinky_id}/waypoints", {"label": label})

    def delete_waypoint(self, pinky_id: str, waypoint_id: str) -> tuple[int, dict[str, Any]]:
        """핑키 관제에 저장된 웨이포인트 하나를 지운다."""
        return self._send("DELETE", f"/api/robots/{pinky_id}/waypoints/{waypoint_id}", None)

    def send_password(self, password: str) -> tuple[int, dict[str, Any]]:
        """SSH 비밀번호를 핑키 관제에 넘긴다. 핑키 관제는 자기 메모리에만 둔다."""
        return self._send("POST", "/api/credentials", {"password": password})

    def _send(self, method: str, path: str, body: dict[str, Any] | None) -> tuple[int, dict[str, Any]]:
        """상태를 바꾸는 요청. 보낸 뒤 상태 캐시를 비워 다음 읽기가 새 상태를 받게 한다."""
        status, answer = self._request(method, path, body)
        with self._lock:
            self._cache = None  # 상태가 곧 바뀐다
        return status, answer

    def fleet_status(self, pinky_id: str) -> dict[str, Any]:
        """주문 흐름의 말로 바꾼 핑키 임무 단계(parked, driving, docking_manual, docked_manual…)와 연결·준비·배터리.

        경로가 없거나 끝났으면 parked, 경로가 돌지만 로봇이 아직 단계를 알리지 않았으면 driving이다. 배터리 퍼센트는
        BATTERY_WINDOW_S 창의 중앙값이다."""
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
            phase = "driving"  # 로봇이 임무 단계를 알리기 전의 starting / running
        operational = robot.get("operational") if isinstance(robot.get("operational"), dict) else {}
        battery_percent, battery_volts = _fresh_battery(robot)
        battery_percent = self._battery.add(pinky_id, time.monotonic(), battery_percent)
        return {
            "source": "pinky_fleet", "simulated": self.simulated, "pinky_id": pinky_id, "phase": phase,
            "mission_token": route.get("mission_token"), "route_status": route_status,
            "message": route.get("message") or robot.get("message"),
            "location": route.get("current_label") or robot.get("stage"),
            "connected": operational.get("state") != "disconnected",
            "ready_for_motion": bool(operational.get("ready_for_motion")),
            "readiness_label": operational.get("label"),
            "battery_percent": battery_percent,
            "battery_volts": battery_volts,
        }

    def dispatch(self, pinky_id: str, route_labels: list[str] | None) -> dict[str, Any]:
        """핑키의 경로를 시작한다(핑키 관제가 맨 앞에 그 핑키의 주차 자리를 붙인다). 202는 핑키 관제가 출발을
        받았다는 뜻이지 핑키가 움직였다는 뜻은 아니다. 주문 흐름이 그 뒤 단계를 따라간다.

        경로에 5번이 있으면 POST /api/robots/<id>/mission(Nav2 임무), 없으면 예전의 POST /api/robots/<id>/route
        (라벨 순서대로)로 보낸다."""
        if not route_labels:
            raise PinkyFleetError("RECEIVING_ZONE_MISSING", "수령 구역 경로가 없는 주문은 실제 핑키를 출발시킬 수 없습니다")
        labels = list(route_labels)
        if "5" in labels:
            # Nav2 주행(2026-11-17): 5번(정면 맞추기, 들어가기, 천장 카메라, 다시 맞추기) -> 수령 구역(마지막 라벨)
            # -> 자기 주차 자리. 5 앞의 라벨들은 가는 길에 지나는 경유지다.
            status, answer = self._send("POST", f"/api/robots/{pinky_id}/mission",
                                        {"dropoff": labels[-1], "via": labels[:labels.index("5")]})
        else:
            status, answer = self._send("POST", f"/api/robots/{pinky_id}/route", {"labels": labels})
        if status == 202 and answer.get("ok"):
            return {"accepted": True, "simulated": answer.get("simulated") is True, "http_status": status,
                    "labels": answer.get("labels")}
        raise PinkyFleetError("PINKY_DISPATCH_REJECTED", answer.get("error") or answer.get("message") or f"HTTP {status}")

    def mission_complete(self, pinky_id: str, token: str | None) -> dict[str, Any]:
        """수령 구역에서 수령이 확인됨: POST /api/robots/<id>/mission-complete {token}. 핑키는 자기 주차 자리로
        돌아간다. 409는 PINKY_SIGNAL_REJECTED, 그 밖의 실패는 PINKY_SIGNAL_NOT_DELIVERED."""
        status, answer = self._send("POST", f"/api/robots/{pinky_id}/mission-complete", {"token": token})
        if status == 202 and answer.get("ok"):
            return {"accepted": True, "simulated": answer.get("simulated") is True, "http_status": status,
                    "message": answer.get("message")}
        if status == 409:
            raise PinkyFleetError("PINKY_SIGNAL_REJECTED", answer.get("error") or "코스 완주 단계·토큰과 맞지 않습니다")
        raise PinkyFleetError("PINKY_SIGNAL_NOT_DELIVERED", answer.get("message") or answer.get("error") or f"HTTP {status}")

    def virtual_parking(self, pinky_id: str) -> dict[str, Any] | None:
        """가상 함대의 천장 카메라가 적재 자리에서 이 핑키를 판정한 결과. 실제 핑키 관제에는 카메라가 없으므로
        (거기서는 주차 노드가 판정한다) None."""
        if not self.simulated:
            return None
        status, answer = self._request("GET", f"/api/robots/{pinky_id}/parking-check")
        return answer if status == 200 and answer.get("simulated") is True and answer.get("status") else None

    def fleet_confirm(self, pinky_id: str, action: str, token: str | None,
                      offset: dict[str, Any] | None = None) -> dict[str, Any]:
        """operator-confirm으로 운영자 신호(dock_complete, received, redock)를 그 단계의 토큰과 함께 보낸다.

        `offset`은 redock과 함께 간다: 천장 카메라가 잰, 적재 자리 대비 핑키의 forward_mm, left_mm, dyaw_deg. 핑키가
        그만큼 고쳐서 다시 들어온다(응답의 `guided`가 그것을 썼는지 알려 준다).
        202+ok는 받음, 409는 PINKY_SIGNAL_REJECTED, 502 또는 ok 없는 202는 PINKY_SIGNAL_NOT_DELIVERED, 그 밖은
        PINKY_FLEET_BAD_ANSWER로 올린다."""
        body: dict[str, Any] = {"action": action, "token": token}
        if offset is not None:
            body["offset"] = offset
        status, answer = self._request("POST", f"/api/robots/{pinky_id}/operator-confirm", body)
        with self._lock:
            self._cache = None  # 단계가 곧 바뀐다
        if status == 202 and answer.get("ok"):
            return {"accepted": True, "simulated": answer.get("simulated") is True, "http_status": status,
                    "message": answer.get("message"), "guided": answer.get("guided") is True}
        if status == 409:
            raise PinkyFleetError("PINKY_SIGNAL_REJECTED", answer.get("error") or "현재 단계·토큰과 신호가 맞지 않습니다")
        if status == 502 or (status == 202 and not answer.get("ok")):
            raise PinkyFleetError("PINKY_SIGNAL_NOT_DELIVERED", answer.get("message") or "핑키에 신호를 전달하지 못했습니다")
        raise PinkyFleetError("PINKY_FLEET_BAD_ANSWER", f"핑키 관제가 HTTP {status}로 답했습니다: {answer}")
