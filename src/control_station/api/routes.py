"""GET/POST 경로 표 (위에서부터 순서대로 판별).

app.py의 ControlStationHandler.do_GET/do_POST가 find()로 요청에 맞는 첫 항목을 찾고, 그 처리 함수를
(handler, route, parts, body)로 부른다. route는 쿼리 문자열을 뺀 경로, parts는 그 경로를 "/"로 나눈 조각이다
(예: "/api/tasks/T-1/start" -> ["api", "tasks", "T-1", "start"]).

순서가 중요하다. 판별 조건 중 일부는 앞부분(startswith)만 보므로, 더 구체적인 경로를 앞에 둔다
(예: /api/arm/commands는 /api/arm/commands/... 보다 먼저 정확히 일치로 판별한다).

POST 항목의 cross_origin_message가 있으면 다른 출처(브라우저) 요청을 403으로 거부한다. 상태를 바꾸는 POST는
모두 이 값을 갖고, 다른 출처 검사가 없는 것은 POST /api/tasks 하나뿐이다.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Callable

from control_station.api import arm, cameras, flow, orders, pinky, status, tasks

if TYPE_CHECKING:
    from control_station.app import ControlStationHandler

# 처리 함수: (handler, route, parts, body) -> None. 응답은 handler._json / _json_error로 직접 보낸다.
Handler = Callable[["ControlStationHandler", str, list[str], Any], None]
# 판별 조건: (route, parts) -> 이 경로가 맞는지.
Matcher = Callable[[str, list[str]], bool]


@dataclass(frozen=True)
class Route:
    """경로 하나: 판별 조건, 처리 함수, 다른 출처 거부, 본문 읽기 여부"""
    matches: Matcher
    handle: Handler
    cross_origin_message: str | None = None  # 값이 있으면 다른 출처 요청을 이 메시지로 거부한다
    cross_origin_error_code: bool = True  # True: error_code 형식의 오류 본문, False: 단순 {"error"} 본문
    reads_body: bool = True  # POST 전용: 처리 함수를 부르기 전에 JSON 객체 본문을 읽는다(False면 body=None)


def _path(expected: str) -> Matcher:
    """경로가 정확히 같은지"""
    return lambda route, parts: route == expected


def _prefix_suffix(prefix: str, suffix: str, contains: str = "") -> Matcher:
    """경로의 앞·뒤(와 중간 포함 문자열)로 판별"""
    return lambda route, parts: route.startswith(prefix) and contains in route and route.endswith(suffix)


def _task_action(*actions: str) -> Matcher:
    """작업 동작 경로 /api/tasks/{task_id}/{동작}"""
    return lambda route, parts: len(parts) == 4 and parts[:2] == ["api", "tasks"] and parts[3] in actions


# 팔 워커의 long-poll 경로: /api/arm/{robot_id}/commands/next
_NEXT_COMMAND = re.compile(r"/api/arm/[^/]+/commands/next")


GET_ROUTES: tuple[Route, ...] = (
    Route(_prefix_suffix("/streams/", ".mjpg"), cameras.get_stream),
    Route(_path("/api/health"), status.get_health),
    Route(_path("/api/robots"), status.get_robots),
    Route(_path("/api/cameras"), cameras.get_cameras),
    Route(_path("/api/arm/status"), arm.get_status),
    Route(_path("/api/arm/commands"), arm.get_commands),
    Route(_path("/api/arm/shelf-slots"), arm.get_shelf_slots),
    Route(_path("/api/arm/policies"), arm.get_policies),
    Route(_path("/api/arm/sequences"), arm.get_sequences),
    Route(lambda route, parts: route.startswith("/api/arm/sequences/"), arm.get_sequence),
    Route(lambda route, parts: route.startswith("/api/arm/commands/"), arm.get_command),
    Route(lambda route, parts: bool(_NEXT_COMMAND.fullmatch(route)), arm.get_next_command),
    Route(_prefix_suffix("/api/parking-spots/", "/live"), status.get_parking_live),
    Route(_prefix_suffix("/api/parking-spots/", "/check"), status.get_parking_check),
    Route(_path("/api/flow"), flow.get_flow),
    Route(_path("/api/pinky"), pinky.get_pinky),
    Route(_path("/api/pinky/map"), pinky.get_map),
    Route(lambda route, parts: len(parts) == 4 and parts[:2] == ["api", "pinky"] and parts[3] == "locate", pinky.get_locate),
    Route(lambda route, parts: len(parts) == 4 and parts[:2] == ["api", "pinky"] and parts[3] == "ros", pinky.get_ros),
    Route(lambda route, parts: len(parts) == 4 and parts[:2] == ["api", "pinky"] and parts[3] == "nav-log", pinky.get_nav_log),
    Route(_path("/api/catalog"), orders.get_catalog),
    Route(_path("/api/inventory"), orders.get_inventory),
    Route(_path("/api/orders"), orders.get_orders),
    Route(_path("/api/dispatch"), orders.get_dispatch),
    Route(lambda route, parts: len(parts) == 4 and parts[:2] == ["api", "orders"] and parts[3] == "assignment",
          orders.get_assignment),
    Route(lambda route, parts: len(parts) == 4 and parts[:2] == ["api", "orders"] and parts[3] == "track",
          orders.get_track),
    Route(_path("/api/tasks"), tasks.get_tasks),
    Route(lambda route, parts: route.startswith("/api/tasks/"), tasks.get_task),
)

POST_ROUTES: tuple[Route, ...] = (
    Route(_task_action("start", "cancel"), tasks.post_task_control,
          "Cross-origin task-control request rejected", reads_body=False),
    Route(lambda route, parts: len(parts) == 5 and parts[:3] == ["api", "flow", "alerts"] and parts[4] == "ack",
          flow.post_alert_ack, "Cross-origin alert request rejected"),
    Route(_path("/api/mock-world"), flow.post_mock_world, "Cross-origin mock-world request rejected"),
    Route(lambda route, parts: route in {"/api/arm/policy/start", "/api/arm/policy/stop"},
          arm.post_policy_run,
          "Cross-origin arm-control request rejected", cross_origin_error_code=False, reads_body=False),
    Route(_path("/api/tasks/parse-command"), tasks.post_parse_command,
          "Cross-origin command-preview request rejected", cross_origin_error_code=False),
    Route(_path("/api/arm/commands/from-camera"), arm.post_command_from_camera,
          "Cross-origin arm-command request rejected"),
    Route(_path("/api/arm/sequences"), arm.post_sequence,
          "Cross-origin arm-sequence request rejected"),
    Route(lambda route, parts: len(parts) == 5 and parts[:3] == ["api", "arm", "sequences"] and parts[4] == "cancel",
          arm.post_sequence, "Cross-origin arm-sequence request rejected"),
    # 팔 워커는 Origin을 보내지 않는 일반 HTTP 클라이언트라서, 다른 출처로 걸리는 것은 브라우저 요청뿐이다.
    Route(
        lambda route, parts: route == "/api/arm/commands"
        or (len(parts) == 5 and parts[:3] == ["api", "arm", "commands"] and parts[4] in {"events", "cancel", "clear-unknown"}),
        arm.post_command,
        "Cross-origin arm-command request rejected",
    ),
    Route(_path("/api/tasks"), tasks.post_task),
    # 큰 빨간 버튼: 팔과 모든 핑키에 한 번의 요청으로 정지 요청 (stop_all.py).
    Route(_path("/api/stop-all"), status.post_stop_everything, "Cross-origin stop request rejected", reads_body=False),
    Route(_path("/api/pinky/stop-all"), pinky.post_stop_all, "Cross-origin Pinky-stop request rejected",
          reads_body=False),
    Route(lambda route, parts: len(parts) == 4 and parts[:2] == ["api", "pinky"] and parts[3] == "stop",
          pinky.post_stop, "Cross-origin Pinky-stop request rejected", reads_body=False),
    Route(_path("/api/pinky/credentials"), pinky.post_credentials, "Cross-origin credentials request rejected"),
    Route(_path("/api/pinky/prepare-all"), pinky.post_prepare_all, "Cross-origin Pinky request rejected"),
    Route(_path("/api/pinky/dock-auto"), pinky.post_dock_auto, "Cross-origin Pinky request rejected"),
    Route(lambda route, parts: len(parts) == 4 and parts[:2] == ["api", "pinky"] and parts[3] == "dock-handshake",
          pinky.post_dock_handshake, "Cross-origin Pinky request rejected"),
    Route(lambda route, parts: parts[:2] == ["api", "pinky"] and (
              (len(parts) == 4 and parts[3] in {"ready", "initial-pose", "localize", "set-pose", "nav-restart", "waypoints", "test-mission", "signal", "route-clear", "return-home", "reset"})
              or (len(parts) == 6 and parts[3] == "waypoints" and parts[5] == "delete")),
          pinky.post_action, "Cross-origin Pinky request rejected"),
    Route(lambda route, parts: len(parts) == 3 and parts[:2] == ["api", "inventory"],
          orders.post_inventory, "Cross-origin inventory request rejected"),
    # 주문 창(/order)도 이 서버가 내주는 페이지라서, 브라우저 주문은 관제 화면처럼 같은 출처 요청이다.
    Route(_path("/api/orders"), orders.post_order, "Cross-origin order request rejected"),
    Route(_path("/api/dispatch"), orders.post_dispatch, "Cross-origin order-control request rejected"),
    Route(lambda route, parts: len(parts) == 4 and parts[:2] == ["api", "orders"] and parts[3] in {"start", "reject", "pickup-confirm", "arrival-confirm"},
          orders.post_order_action, "Cross-origin order-control request rejected", reads_body=False),
)


def find(routes: tuple[Route, ...], route: str, parts: list[str]) -> Route | None:
    """요청에 맞는 첫 경로를 찾는다. 없으면 None (GET은 정적 파일로, POST는 404로 이어진다)."""
    return next((entry for entry in routes if entry.matches(route, parts)), None)
