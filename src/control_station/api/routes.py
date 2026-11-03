"""Route tables: which handler answers each request, in match order.

The matchers keep the path tests the routes have always used (prefix/suffix checks, not templates), so the same
paths reach the same handlers. POST routes that browsers can trigger reject cross-origin requests; some answer with
the error_code envelope and older ones with a plain {"error"} body, as before.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Callable

from control_station.api import arm, cameras, observations, status, tasks

if TYPE_CHECKING:
    from control_station.app import ControlStationHandler

Handler = Callable[["ControlStationHandler", str, list[str], Any], None]
Matcher = Callable[[str, list[str]], bool]


@dataclass(frozen=True)
class Route:
    matches: Matcher
    handle: Handler
    cross_origin_message: str | None = None  # reject requests from another origin with this message
    cross_origin_error_code: bool = True  # error_code envelope, or a plain {"error"} body
    reads_body: bool = True  # POST only: parse a JSON object body before calling the handler


def _path(expected: str) -> Matcher:
    return lambda route, parts: route == expected


def _prefix_suffix(prefix: str, suffix: str, contains: str = "") -> Matcher:
    return lambda route, parts: route.startswith(prefix) and contains in route and route.endswith(suffix)


def _task_action(*actions: str) -> Matcher:
    return lambda route, parts: len(parts) == 4 and parts[:2] == ["api", "tasks"] and parts[3] in actions


_NEXT_COMMAND = re.compile(r"/api/arm/[^/]+/commands/next")


GET_ROUTES: tuple[Route, ...] = (
    Route(_prefix_suffix("/streams/", ".mjpg"), cameras.get_stream),
    Route(_path("/api/health"), status.get_health),
    Route(_path("/api/robots"), status.get_robots),
    Route(_path("/api/cameras"), cameras.get_cameras),
    Route(_prefix_suffix("/api/cameras/", "/target-observation"), cameras.get_camera_target_observation),
    Route(_prefix_suffix("/api/tasks/", "/image", contains="/observations/"), observations.get_observation_image),
    Route(_prefix_suffix("/api/tasks/", "/artifact", contains="/projection-drafts/"),
          observations.get_projection_artifact),
    Route(_path("/api/arm/status"), arm.get_status),
    Route(_path("/api/arm/commands"), arm.get_commands),
    Route(_path("/api/arm/shelf-slots"), arm.get_shelf_slots),
    Route(lambda route, parts: route.startswith("/api/arm/commands/"), arm.get_command),
    Route(lambda route, parts: bool(_NEXT_COMMAND.fullmatch(route)), arm.get_next_command),
    Route(_prefix_suffix("/api/parking-spots/", "/live"), status.get_parking_live),
    Route(_prefix_suffix("/api/parking-spots/", "/check"), status.get_parking_check),
    Route(_path("/api/tasks"), tasks.get_tasks),
    Route(lambda route, parts: route.startswith("/api/tasks/"), tasks.get_task),
)

POST_ROUTES: tuple[Route, ...] = (
    Route(_task_action("start", "cancel"), tasks.post_task_control,
          "Cross-origin task-control request rejected", reads_body=False),
    Route(lambda route, parts: route in {"/api/arm/start", "/api/arm/stop"}, arm.post_teleop,
          "Cross-origin arm-control request rejected", cross_origin_error_code=False, reads_body=False),
    Route(_task_action("target-observation"), observations.post_target_observation,
          "Cross-origin task-observation request rejected"),
    Route(_task_action("target-selection"), observations.post_target_selection,
          "Cross-origin target-selection request rejected"),
    Route(_task_action("projection-draft"), observations.post_projection_draft,
          "Cross-origin projection-draft request rejected"),
    Route(_path("/api/tasks/parse-command"), tasks.post_parse_command,
          "Cross-origin command-preview request rejected", cross_origin_error_code=False),
    # The arm worker is a plain HTTP client without Origin, so only browser requests can be cross-origin.
    Route(
        lambda route, parts: route == "/api/arm/commands"
        or (len(parts) == 5 and parts[:3] == ["api", "arm", "commands"] and parts[4] in {"events", "cancel", "clear-unknown"}),
        arm.post_command,
        "Cross-origin arm-command request rejected",
    ),
    Route(_path("/api/arm/coordinate-target-drafts"), arm.post_coordinate_draft,
          "Cross-origin coordinate draft request rejected", cross_origin_error_code=False),
    Route(_path("/api/tasks"), tasks.post_task),
)


def find(routes: tuple[Route, ...], route: str, parts: list[str]) -> Route | None:
    return next((entry for entry in routes if entry.matches(route, parts)), None)
