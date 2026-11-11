"""Load check after the arm step: did everything the task asked for end up on Pinky's cart?

This is the B-zone "목표 수량 적재 성공 확인" branch of docs/spec/data-flow.md. An observer reports how many items
it sees on the cart; judge_load compares that count with the task's expected count. The arm reporting that it
placed an item is a separate result and does not count as seeing the item on the cart.

Only MockLoadObserver exists so far. It opens no camera and reports a configured count, so its observations say
simulated. A ceiling-camera YOLO observer is meant to plug in behind the same LoadObserver signature.
"""

from __future__ import annotations

import os
from typing import Any, Callable

# One object per task while the flow is being verified (decided 2026-11-03). Orders can hold several items (e.g. 3);
# those are planned as a check after every placement (the count before and after must differ by exactly one) plus
# this total check before Pinky leaves, once tasks carry a quantity.
EXPECTED_LOAD_COUNT = 1
LOAD_OBSERVATION_SCHEMA_VERSION = 1

# Returns {"source", "observed_count": int | None, "reason", ...}; raises with an error_code attribute when the
# observation could not start.
LoadObserver = Callable[[], dict[str, Any]]

LOAD_ERROR_CODES = {
    "INCOMPLETE": "LOAD_INCOMPLETE",
    "EXCESS": "LOAD_EXCESS",
    "NOT_VERIFIED": "LOAD_NOT_VERIFIED",
}


def judge_load(observation: dict[str, Any], expected_count: int) -> str:
    """LOADED only when the observed count equals the expected count; a missing count is NOT_VERIFIED."""
    count = observation.get("observed_count")
    if not isinstance(count, int) or isinstance(count, bool) or count < 0:
        return "NOT_VERIFIED"
    if count == expected_count:
        return "LOADED"
    return "INCOMPLETE" if count < expected_count else "EXCESS"


class MockLoadObserver:
    """Reports a fixed count without looking at anything, to exercise the VERIFY_LOAD flow in mock mode."""

    def __init__(self, observed_count: int) -> None:
        self.observed_count = observed_count

    def __call__(self) -> dict[str, Any]:
        return {
            "observation_schema_version": LOAD_OBSERVATION_SCHEMA_VERSION,
            "source": "mock",
            "simulated": True,
            "observed_count": self.observed_count,
            "reason": "모의 관측: 카메라를 열지 않고 CONTROL_STATION_MOCK_LOAD_OBSERVED 값을 보고함",
            "usable_for_robot_control": False,
        }


def mock_load_observer_from_env() -> MockLoadObserver | None:
    raw = os.environ.get("CONTROL_STATION_MOCK_LOAD_OBSERVED", "").strip()
    if not raw:
        return None
    try:
        count = int(raw)
    except ValueError:
        count = -1
    if count < 0:
        raise ValueError("CONTROL_STATION_MOCK_LOAD_OBSERVED must be a non-negative integer")
    return MockLoadObserver(count)
