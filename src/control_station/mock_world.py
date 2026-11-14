"""A simulated loading station for running orders end to end without robots or cameras.

It keeps shelf stock per shelf, what lies on each Pinky, and each Pinky's mission phase the way the Pinky fleet
dashboard (origin/pinky_fleet) reports it: parked -> driving -> docking_manual (stopped at station 5, waiting for
dock_complete) -> docked_manual (holding, waiting for received) -> driving -> parked. A mock pick moves one object
from a shelf onto the Pinky at the station. Every observation it returns says `simulated: true`.

Faults can be injected to check the operator screens (CONTROL_STATION_MOCK_FAULTS or POST /api/mock-world):
  STOCK_SHORT:<class>        the shelves report none of that object
  PICK_FAILED:<class>        the next pick of that object leaves it on the shelf
  ITEM_LOST:<class>          the next pick of that object takes it off the shelf but it never reaches the Pinky
  EXTRA_ON_PINKY:<class>     one unexpected object lies on the next Pinky that docks
  PINKY_MISALIGNED:<n>       the next n parking checks report the Pinky off its spot
  FLEET_REJECT:<action>      the next dock_complete or received signal is refused (HTTP 409 in the real dashboard)
Each fault fires once and is then removed, so the next order runs normally.
"""

from __future__ import annotations

import os
import threading
import time
import uuid
from collections import Counter
from datetime import datetime, timezone
from typing import Any, Callable

from control_station.pinky_fleet import PinkyFleetError

FAULT_KINDS = frozenset({"STOCK_SHORT", "PICK_FAILED", "ITEM_LOST", "EXTRA_ON_PINKY", "PINKY_MISALIGNED", "FLEET_REJECT"})
DEFAULT_STOCK = "shelf_a:carrot=3,tissue=2,apple=1;shelf_b:apple=3,carrot=1,tissue=1"
PINKY_HOMES = {"10": "주차장 P0", "11": "주차장 P1", "12": "주차장 P2"}
# Simulated battery levels, different on purpose so the Pinky assignment has something to rank.
MOCK_BATTERY = {"10": 92.0, "11": 78.0, "12": 64.0}


def _now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def parse_stock(text: str) -> dict[str, Counter]:
    """'shelf_a:carrot=3,apple=1;shelf_b:apple=2' -> {shelf_id: Counter}."""
    shelves: dict[str, Counter] = {}
    for part in filter(None, (piece.strip() for piece in text.split(";"))):
        shelf_id, _, items = part.partition(":")
        counts: Counter = Counter()
        for item in filter(None, (piece.strip() for piece in items.split(","))):
            name, _, count = item.partition("=")
            counts[name.strip()] += int(count or 1)
        shelves[shelf_id.strip()] = counts
    return shelves


def parse_faults(text: str) -> list[tuple[str, str]]:
    faults = []
    for item in filter(None, (piece.strip() for piece in text.split(","))):
        kind, _, argument = item.partition(":")
        if kind not in FAULT_KINDS:
            raise ValueError(f"unknown mock fault {kind!r}; one of {sorted(FAULT_KINDS)}")
        faults.append((kind, argument.strip()))
    return faults


class MockWorld:
    def __init__(
        self,
        stock: dict[str, Counter] | None = None,
        *,
        drive_s: float = 8.0,
        pick_s: float = 4.0,
        faults: list[tuple[str, str]] | None = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._lock = threading.RLock()
        self._initial_stock = {shelf: Counter(counts) for shelf, counts in (stock or parse_stock(DEFAULT_STOCK)).items()}
        self.shelves = {shelf: Counter(counts) for shelf, counts in self._initial_stock.items()}
        self.loads: dict[str, Counter] = {pinky: Counter() for pinky in PINKY_HOMES}
        self.pinkies: dict[str, dict[str, Any]] = {
            pinky: {"phase": "parked", "token": None, "message": "주차장 대기", "location": home, "until": None,
                    "battery": MOCK_BATTERY[pinky]}
            for pinky, home in PINKY_HOMES.items()
        }
        self.drive_s = drive_s
        self.pick_s = pick_s
        self.faults: list[tuple[str, str]] = list(faults or [])
        self.clock = clock

    @classmethod
    def from_env(cls) -> MockWorld:
        def seconds(name: str, default: float) -> float:
            try:
                return min(120.0, max(0.0, float(os.environ.get(name, default))))
            except ValueError:
                return default

        return cls(
            parse_stock(os.environ.get("CONTROL_STATION_MOCK_SHELF_STOCK", DEFAULT_STOCK)),
            drive_s=seconds("CONTROL_STATION_MOCK_PINKY_DRIVE_S", 8.0),
            pick_s=seconds("CONTROL_STATION_MOCK_PICK_S", 4.0),
            faults=parse_faults(os.environ.get("CONTROL_STATION_MOCK_FAULTS", "")),
        )

    # ---- operator controls for the screen check ----
    def inject(self, kind: str, argument: str = "") -> None:
        if kind not in FAULT_KINDS:
            raise ValueError(f"unknown mock fault {kind!r}")
        with self._lock:
            self.faults.append((kind, argument))

    def restock(self) -> None:
        with self._lock:
            self.shelves = {shelf: Counter(counts) for shelf, counts in self._initial_stock.items()}

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {
                "simulated": True,
                "shelves": {shelf: dict(counts) for shelf, counts in self.shelves.items()},
                "pinky_loads": {pinky: dict(counts) for pinky, counts in self.loads.items() if counts},
                "pending_faults": [f"{kind}:{argument}" if argument else kind for kind, argument in self.faults],
            }

    def _take_fault(self, kind: str, argument: str | None = None) -> str | None:
        """Remove and return the argument of the first matching fault, or None."""
        for index, (fault_kind, fault_argument) in enumerate(self.faults):
            if fault_kind == kind and (argument is None or fault_argument in ("", argument)):
                del self.faults[index]
                return fault_argument
        return None

    # ---- shelf and Pinky load observations ----
    def shelf_stock(self) -> dict[str, Any]:
        with self._lock:
            shelves = {shelf: dict(counts) for shelf, counts in self.shelves.items()}
            short = self._take_fault("STOCK_SHORT")
            if short:
                for counts in shelves.values():
                    counts.pop(short, None)
            return {"source": "mock_world", "simulated": True, "captured_at": _now(), "shelves": shelves}

    def pinky_load(self, pinky_id: str) -> dict[str, Any]:
        with self._lock:
            return {
                "source": "mock_world", "simulated": True, "captured_at": _now(),
                "pinky_id": pinky_id, "counts": dict(self.loads[pinky_id]),
            }

    # ---- Pinky fleet dashboard (mission phases and operator signals) ----
    def dispatch(self, pinky_id: str, route_labels: list[str] | None = None) -> dict[str, Any]:
        """The Pinky leaves its parking spot for station 5. With a route (an order with a receiving zone) it goes
        on to the end of that route after loading and waits there in course_complete, like the real fleet."""
        with self._lock:
            pinky = self.pinkies[pinky_id]
            if pinky["phase"] == "parked":
                pinky.update(phase="driving", token=None, message="5번 정류장으로 이동 중", location="이동 중",
                             until=self.clock() + self.drive_s, leg="to_station", route=list(route_labels or []))
            return {"accepted": True, "simulated": True, "message": "모의: 핑키가 적재 위치로 출발"}

    def mission_complete(self, pinky_id: str, token: str | None) -> dict[str, Any]:
        """The receiving zone confirmed pickup: the Pinky drives back to its own parking spot."""
        with self._lock:
            pinky = self.pinkies[pinky_id]
            if pinky["phase"] != "course_complete" or not token or token != pinky["token"]:
                raise PinkyFleetError("PINKY_SIGNAL_REJECTED", "코스 완주 후 현재 임무의 주행완료 신호만 처리할 수 있습니다.")
            pinky.update(phase="driving", token=None, message="주행완료 · 주차장으로 복귀", location="이동 중",
                         until=self.clock() + self.drive_s, leg="to_home")
            return {"accepted": True, "simulated": True, "message": "주행완료 신호를 보냈습니다. (모의)"}

    def fleet_status(self, pinky_id: str) -> dict[str, Any]:
        with self._lock:
            pinky = self.pinkies[pinky_id]
            if pinky["phase"] == "driving" and pinky["until"] is not None and self.clock() >= pinky["until"]:
                if pinky.get("leg") == "to_station":
                    pinky.update(phase="docking_manual", token=uuid.uuid4().hex[:12], until=None,
                                 message="5번 도착 · 정지 · 도킹완료 확인 대기", location="5번 정류장")
                    extra = self._take_fault("EXTRA_ON_PINKY")
                    if extra:
                        self.loads[pinky_id][extra] += 1
                elif pinky.get("leg") == "to_zone":
                    pinky.update(phase="course_complete", token=uuid.uuid4().hex[:12], until=None,
                                 message="코스 완주 · 정지 · 주행완료 신호 대기", location="수령 구역")
                else:
                    self.loads[pinky_id].clear()  # unloaded at the destination in this simulation
                    pinky.update(phase="parked", token=None, until=None, message="주차장 대기",
                                 location=PINKY_HOMES[pinky_id], route=[])
            return {
                "source": "mock_world", "simulated": True, "pinky_id": pinky_id, "phase": pinky["phase"],
                "mission_token": pinky["token"], "message": pinky["message"], "location": pinky["location"],
                "ready_for_motion": True, "readiness_label": "모의 핑키", "battery_percent": pinky["battery"],
            }

    def fleet_confirm(self, pinky_id: str, action: str, token: str | None) -> dict[str, Any]:
        with self._lock:
            pinky = self.pinkies[pinky_id]
            wanted_phase = {"dock_complete": "docking_manual", "received": "docked_manual"}.get(action)
            if wanted_phase is None:
                raise PinkyFleetError("PINKY_SIGNAL_INVALID", f"알 수 없는 신호 {action}")
            if self._take_fault("FLEET_REJECT", action) is not None:
                raise PinkyFleetError("PINKY_SIGNAL_REJECTED", "현재 임무의 확인 단계와 신호가 일치하지 않습니다. (모의 거부)")
            if pinky["phase"] != wanted_phase or not token or token != pinky["token"]:
                raise PinkyFleetError("PINKY_SIGNAL_REJECTED", "현재 임무의 확인 단계와 신호가 일치하지 않습니다.")
            if action == "dock_complete":
                pinky.update(phase="docked_manual", token=uuid.uuid4().hex[:12],
                             message="도킹완료 · 정지 유지 · 물건 수령 완료 대기")
            else:
                # With a route the Pinky carries the load on to the receiving zone; without one (older orders) it
                # goes home and is unloaded there, as before.
                leg = "to_zone" if pinky.get("route") else "to_home"
                pinky.update(phase="driving", token=None, message="물건 수령 완료 · 경로 재개", location="이동 중",
                             until=self.clock() + self.drive_s, leg=leg)
            return {"accepted": True, "simulated": True, "message": "현재 단계 확인 신호를 보냈습니다. (모의)"}

    # ---- parking judgment (ceiling camera) ----
    def parking(self, pinky_id: str) -> dict[str, Any]:
        with self._lock:
            remaining = self._take_fault("PINKY_MISALIGNED")
            if remaining is not None:
                count = int(remaining or 1) - 1
                if count > 0:
                    self.faults.insert(0, ("PINKY_MISALIGNED", str(count)))
                return {
                    "status": "MISALIGNED", "spot_id": "mock_station", "simulated": True, "pinky_id": pinky_id,
                    "reason": "모의: 정차 위치에서 벗어남", "captured_at": _now(),
                    "offset": {"dx_mm": 48.0, "dy_mm": -21.0, "distance_mm": 52.4, "dyaw_deg": 6.5},
                    "tolerance": {"position_mm": 30.0, "yaw_deg": 5.0},
                }
            return {
                "status": "ARRIVED", "spot_id": "mock_station", "simulated": True, "pinky_id": pinky_id,
                "reason": "모의: 정차 위치 안", "captured_at": _now(),
                "offset": {"dx_mm": 4.0, "dy_mm": 2.0, "distance_mm": 4.5, "dyaw_deg": 0.8},
                "tolerance": {"position_mm": 30.0, "yaw_deg": 5.0},
            }

    # ---- arm pick (policy run) ----
    def pick(
        self, arm_id: str, object_class: str, shelf_id: str, pinky_id: str, cancel_event: threading.Event,
        progress: Callable[[str], None],
    ) -> dict[str, Any] | None:
        progress(f"{arm_id}: 정책 실행 중 (모의)")
        if cancel_event.wait(self.pick_s * 0.6):
            return None
        progress(f"{arm_id}: 핑키 위에 놓고 home으로 복귀 중 (모의)")
        if cancel_event.wait(self.pick_s * 0.4):
            return None
        with self._lock:
            if self._take_fault("PICK_FAILED", object_class) is not None:
                pass  # the object stays on the shelf
            elif self._take_fault("ITEM_LOST", object_class) is not None:
                self.shelves[shelf_id][object_class] -= 1
            else:
                self.shelves[shelf_id][object_class] -= 1
                self.loads[pinky_id][object_class] += 1
            self.shelves[shelf_id] += Counter()  # drop zero and negative counts
        # Like the real worker: back home, but whether the object was grasped and placed is not observed by the arm.
        return {"source": "mock_world", "simulated": True, "outcome": "ARM_HOME_PLACEMENT_NOT_VERIFIED", "arm_id": arm_id}
