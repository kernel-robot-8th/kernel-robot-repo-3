"""GET/POST 경로 표 (위에서부터 순서대로 판별)"""

from __future__ import annotations

import re
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Callable

from control_station.api import arm, cameras, flow, orders, pinky, status, tasks

if TYPE_CHECKING:
    from control_station.app import ControlStationHandler

Handler = Callable[["ControlStationHandler", str, list[str], Any], None]
Matcher = Callable[[str, list[str]], bool]


@dataclass(frozen=True)
class Route:
    """경로 하나: 판별 조건, 처리 함수, 다른 출처 거부, 본문 읽기 여부"""
    matches: Matcher
    handle: Handler
    cross_origin_message: str | None = None  # reject requests from another origin with this message
    cross_origin_error_code: bool = True  # error_code envelope, or a plain {"error"} body
    reads_body: bool = True  # POST only: parse a JSON object body before calling the handler


def _path(expected: str) -> Matcher:
    """경로가 정확히 같은지"""
    return lambda route, parts: route == expected


def _prefix_suffix(prefix: str, suffix: str, contains: str = "") -> Matcher:
    """경로의 앞·뒤(와 중간 포함 문자열)로 판별"""
    return lambda route, parts: route.startswith(prefix) and contains in route and route.endswith(suffix)


def _task_action(*actions: str) -> Matcher:
    """작업 동작 경로 /api/tasks/{task_id}/{동작}"""
    return lambda route, parts: len(parts) == 4 and parts[:2] == ["api", "tasks"] and parts[3] in actions


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
    Route(lambda route, parts: route.startswith("/api/arm/commands/"), arm.get_command),
    Route(lambda route, parts: bool(_NEXT_COMMAND.fullmatch(route)), arm.get_next_command),
    Route(_prefix_suffix("/api/parking-spots/", "/live"), status.get_parking_live),
    Route(_prefix_suffix("/api/parking-spots/", "/check"), status.get_parking_check),
    Route(_path("/api/flow"), flow.get_flow),
    Route(_path("/api/pinky"), pinky.get_pinky),
    Route(_path("/api/pinky/map"), pinky.get_map),
    Route(_path("/api/catalog"), orders.get_catalog),
    Route(_path("/api/inventory"), orders.get_inventory),
    Route(_path("/api/orders"), orders.get_orders),
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
    Route(lambda route, parts: route in {"/api/arm/start", "/api/arm/stop", "/api/arm/policy/start", "/api/arm/policy/stop"},
          arm.post_teleop,
          "Cross-origin arm-control request rejected", cross_origin_error_code=False, reads_body=False),
    Route(_path("/api/tasks/parse-command"), tasks.post_parse_command,
          "Cross-origin command-preview request rejected", cross_origin_error_code=False),
    Route(_path("/api/arm/commands/from-camera"), arm.post_command_from_camera,
          "Cross-origin arm-command request rejected"),
    # The arm worker is a plain HTTP client without Origin, so only browser requests can be cross-origin.
    Route(
        lambda route, parts: route == "/api/arm/commands"
        or (len(parts) == 5 and parts[:3] == ["api", "arm", "commands"] and parts[4] in {"events", "cancel", "clear-unknown"}),
        arm.post_command,
        "Cross-origin arm-command request rejected",
    ),
    Route(_path("/api/tasks"), tasks.post_task),
    Route(_path("/api/pinky/stop-all"), pinky.post_stop_all, "Cross-origin Pinky-stop request rejected",
          reads_body=False),
    Route(lambda route, parts: len(parts) == 4 and parts[:2] == ["api", "pinky"] and parts[3] == "stop",
          pinky.post_stop, "Cross-origin Pinky-stop request rejected", reads_body=False),
    Route(_path("/api/pinky/credentials"), pinky.post_credentials, "Cross-origin credentials request rejected"),
    Route(lambda route, parts: len(parts) == 3 and parts[:2] == ["api", "inventory"],
          orders.post_inventory, "Cross-origin inventory request rejected"),
    # The order window is a page of this server, so a browser order is same-origin like the operator screen.
    Route(_path("/api/orders"), orders.post_order, "Cross-origin order request rejected"),
    Route(lambda route, parts: len(parts) == 4 and parts[:2] == ["api", "orders"] and parts[3] in {"start", "reject", "pickup-confirm"},
          orders.post_order_action, "Cross-origin order-control request rejected", reads_body=False),
)


def find(routes: tuple[Route, ...], route: str, parts: list[str]) -> Route | None:
    """요청에 맞는 경로 찾기"""
    return next((entry for entry in routes if entry.matches(route, parts)), None)
