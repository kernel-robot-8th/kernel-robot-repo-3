"""로봇과 카메라 없이 주문을 처음부터 끝까지 돌려 보는 모의 적재 정류장.

선반별 재고, 핑키마다 실린 물건, 핑키마다 임무 단계를 갖고 있다. 임무 단계는 핑키 관제 대시보드
(origin/pinky_fleet)가 알려 주는 방식 그대로다: parked -> driving -> docking_manual (5번 정류장에 정지,
dock_complete 대기) -> docked_manual (정지 유지, received 대기) -> driving -> parked. 수령 구역 경로가 있는 주문은
received 뒤에 수령 구역으로 가서 course_complete(주행완료 신호 대기)로 서 있다가 주차장으로 돌아간다.
모의 집기는 선반의 물체 하나를 정류장의 핑키 위로 옮긴다. 돌려주는 관측은 모두 `simulated: true`다.

app.py가 이 객체의 메서드(shelf_stock, pinky_load, fleet_status, fleet_confirm, dispatch, mission_complete,
parking, pick)를 주문 흐름의 장치 자리에 끼운다. 실제 장치가 설정된 부분은 실제 것으로 바뀐다.
시간은 따로 흐르지 않는다: 주행 같은 시간 경과는 fleet_status를 부를 때 clock()과 비교해 반영한다.
모든 상태는 self._lock(RLock) 안에서 바꾼다(주문 흐름 스레드와 HTTP 요청 스레드가 함께 부른다).

운영자 화면을 확인하려고 사고를 넣을 수 있다(CONTROL_STATION_MOCK_FAULTS 또는 POST /api/mock-world):
  STOCK_SHORT:<class>        선반에 그 물체가 하나도 없다고 알림
  PICK_FAILED:<class>        그 물체의 다음 집기가 물체를 선반에 그대로 둠
  ITEM_LOST:<class>          그 물체의 다음 집기가 선반에서 가져가지만 핑키에 닿지 않음
  EXTRA_ON_PINKY:<class>     다음에 정류장에 들어오는 핑키 위에 예상하지 않은 물체가 하나 있음
  PINKY_MISALIGNED:<n>       다음 n번의 주차 판정이 핑키가 자리를 벗어났다고 알림
  FLEET_REJECT:<action>      다음 dock_complete 또는 received 신호를 거절함 (실제 대시보드에서는 HTTP 409)
사고는 한 번 일어나면 사라지므로, 그다음 주문은 정상으로 돈다.
"""

from __future__ import annotations

import os
import threading
import time
import uuid
from collections import Counter
from typing import Any, Callable

from control_station import utc_now_iso
from control_station.pinky.fleet_client import PinkyFleetError

FAULT_KINDS = frozenset({"STOCK_SHORT", "PICK_FAILED", "ITEM_LOST", "EXTRA_ON_PINKY", "PINKY_MISALIGNED", "FLEET_REJECT"})
# 기본 선반 재고 ("선반:물체=개수,...;..." 형식). 사과 정책은 보드 F58AE686에서 녹화했고, 이 보드는 2026-11-16부터
# 오른쪽 팔(arm-02)이므로 사과는 오른쪽 선반(shelf_b)에만 둔다.
DEFAULT_STOCK = "shelf_a:carrot=3,tissue=1;shelf_b:apple=4,carrot=1,tissue=2"
# 핑키 ID(10, 11, 12) -> 자기 주차장 자리.
PINKY_HOMES = {"10": "주차장 P0", "11": "주차장 P1", "12": "주차장 P2"}
# 모의 배터리 잔량(%). 핑키 배정이 순위를 매길 거리가 있도록 일부러 서로 다르게 둔다.
MOCK_BATTERY = {"10": 92.0, "11": 78.0, "12": 64.0}


def parse_stock(text: str) -> dict[str, Counter]:
    """'shelf_a:carrot=3,apple=1;shelf_b:apple=2' -> {shelf_id: Counter}. 개수를 빼면 1개로 본다."""
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
    """'PICK_FAILED:carrot,PINKY_MISALIGNED:2' -> [(종류, 인자)]. 모르는 종류면 ValueError(서버 시작 실패)."""
    faults = []
    for item in filter(None, (piece.strip() for piece in text.split(","))):
        kind, _, argument = item.partition(":")
        if kind not in FAULT_KINDS:
            raise ValueError(f"unknown mock fault {kind!r}; one of {sorted(FAULT_KINDS)}")
        faults.append((kind, argument.strip()))
    return faults


class MockWorld:
    """모의 정류장 하나. drive_s: 한 구간 주행 시간(초), pick_s: 집기 한 번의 시간(초)."""

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
        """환경 변수로 만든다: CONTROL_STATION_MOCK_SHELF_STOCK, _PINKY_DRIVE_S, _PICK_S, _FAULTS."""
        def seconds(name: str, default: float) -> float:
            try:
                # 0–120초로 자른다. 숫자가 아니면 기본값.
                return min(120.0, max(0.0, float(os.environ.get(name, default))))
            except ValueError:
                return default

        return cls(
            parse_stock(os.environ.get("CONTROL_STATION_MOCK_SHELF_STOCK", DEFAULT_STOCK)),
            drive_s=seconds("CONTROL_STATION_MOCK_PINKY_DRIVE_S", 8.0),
            pick_s=seconds("CONTROL_STATION_MOCK_PICK_S", 4.0),
            faults=parse_faults(os.environ.get("CONTROL_STATION_MOCK_FAULTS", "")),
        )

    # ---- 화면 확인용 운영자 조작 ----
    def inject(self, kind: str, argument: str = "") -> None:
        """사고 하나를 대기열 끝에 넣는다."""
        if kind not in FAULT_KINDS:
            raise ValueError(f"unknown mock fault {kind!r}")
        with self._lock:
            self.faults.append((kind, argument))

    def restock(self) -> None:
        """선반 재고를 처음 값으로 되돌린다(핑키 위 물건은 그대로)."""
        with self._lock:
            self.shelves = {shelf: Counter(counts) for shelf, counts in self._initial_stock.items()}

    def snapshot(self) -> dict[str, Any]:
        """화면용 상태: 선반 재고, 물건이 실린 핑키, 아직 일어나지 않은 사고."""
        with self._lock:
            return {
                "simulated": True,
                "shelves": {shelf: dict(counts) for shelf, counts in self.shelves.items()},
                "pinky_loads": {pinky: dict(counts) for pinky, counts in self.loads.items() if counts},
                "pending_faults": [f"{kind}:{argument}" if argument else kind for kind, argument in self.faults],
            }

    def _take_fault(self, kind: str, argument: str | None = None) -> str | None:
        """맞는 첫 사고를 꺼내(지우고) 그 인자를 돌려준다. 없으면 None. self._lock을 잡은 채로 부른다.

        argument를 주면 인자가 같거나 비어 있는(모든 대상에 해당하는) 사고만 맞는다. 인자가 빈 사고는 ""를 돌려주므로,
        호출하는 쪽은 `is not None`으로 확인해야 한다."""
        for index, (fault_kind, fault_argument) in enumerate(self.faults):
            if fault_kind == kind and (argument is None or fault_argument in ("", argument)):
                del self.faults[index]
                return fault_argument
        return None

    # ---- 선반 재고와 핑키 적재 관측 ----
    def shelf_stock(self) -> dict[str, Any]:
        """선반별 재고 관측. STOCK_SHORT 사고가 있으면 그 물체를 모든 선반에서 뺀 값으로 알린다(실제 재고는 그대로)."""
        with self._lock:
            shelves = {shelf: dict(counts) for shelf, counts in self.shelves.items()}
            short = self._take_fault("STOCK_SHORT")
            if short:
                for counts in shelves.values():
                    counts.pop(short, None)
            return {"source": "mock_world", "simulated": True, "captured_at": utc_now_iso("milliseconds"), "shelves": shelves}

    def pinky_load(self, pinky_id: str) -> dict[str, Any]:
        """그 핑키 위 물건 개수 관측."""
        with self._lock:
            return {
                "source": "mock_world", "simulated": True, "captured_at": utc_now_iso("milliseconds"),
                "pinky_id": pinky_id, "counts": dict(self.loads[pinky_id]),
            }

    # ---- 핑키 관제 대시보드 흉내 (임무 단계와 운영자 신호) ----
    def dispatch(self, pinky_id: str, route_labels: list[str] | None = None) -> dict[str, Any]:
        """핑키가 주차장에서 5번 정류장으로 출발한다. 경로가 있으면(수령 구역이 있는 주문) 적재 뒤 그 경로 끝까지 가서
        실제 대시보드처럼 course_complete로 기다린다. 주차 중이 아니면 아무것도 바꾸지 않고 접수만 알린다."""
        with self._lock:
            pinky = self.pinkies[pinky_id]
            if pinky["phase"] == "parked":
                pinky.update(phase="driving", token=None, message="5번 정류장으로 이동 중", location="이동 중",
                             until=self.clock() + self.drive_s, leg="to_station", route=list(route_labels or []))
            return {"accepted": True, "simulated": True, "message": "모의: 핑키가 적재 위치로 출발"}

    def mission_complete(self, pinky_id: str, token: str | None) -> dict[str, Any]:
        """수령 구역에서 수령이 확인됨: 핑키가 자기 주차장으로 돌아간다. course_complete이고 토큰이 맞을 때만 받는다."""
        with self._lock:
            pinky = self.pinkies[pinky_id]
            if pinky["phase"] != "course_complete" or not token or token != pinky["token"]:
                raise PinkyFleetError("PINKY_SIGNAL_REJECTED", "코스 완주 후 현재 임무의 주행완료 신호만 처리할 수 있습니다.")
            pinky.update(phase="driving", token=None, message="주행완료 · 주차장으로 복귀", location="이동 중",
                         until=self.clock() + self.drive_s, leg="to_home")
            return {"accepted": True, "simulated": True, "message": "주행완료 신호를 보냈습니다. (모의)"}

    def unload(self, pinky_id: str) -> None:
        """고객이 이 핑키에서 주문을 꺼내 갔다. 모의 핑키는 수령 구역에서 스스로 이렇게 한다. 실제(또는 가상) 핑키에
        모의 개수를 쓸 때는 주문 흐름의 수령 확인이 이것을 부른다(app.py의 mission_complete 감싸기)."""
        with self._lock:
            self.loads[pinky_id].clear()

    def fleet_status(self, pinky_id: str) -> dict[str, Any]:
        """핑키 상태(대시보드 응답 모양). 주행 시간이 다 됐으면 여기서 다음 단계로 넘긴다.

        도착할 때마다 새 mission_token을 만든다. 신호는 현재 토큰과 함께 와야 해서, 지난 단계의 신호가 늦게 와도
        잘못 적용되지 않는다(실제 대시보드와 같은 규칙)."""
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
                    self.loads[pinky_id].clear()  # 이 모의에서는 도착지에서 짐을 내린 것으로 본다
                    pinky.update(phase="parked", token=None, until=None, message="주차장 대기",
                                 location=PINKY_HOMES[pinky_id], route=[])
            return {
                "source": "mock_world", "simulated": True, "pinky_id": pinky_id, "phase": pinky["phase"],
                "mission_token": pinky["token"], "message": pinky["message"], "location": pinky["location"],
                "ready_for_motion": True, "readiness_label": "모의 핑키", "battery_percent": pinky["battery"],
            }

    def fleet_confirm(self, pinky_id: str, action: str, token: str | None,
                      offset: dict[str, Any] | None = None) -> dict[str, Any]:
        """운영자 신호(dock_complete, redock, received)를 받는다. 단계·토큰이 맞지 않으면 PinkyFleetError.

        redock은 핑키를 잠깐(drive_s의 1/3) 다시 주행시켜 5번에 다시 세운다. offset은 받았다는 표시(guided)만 한다."""
        with self._lock:
            pinky = self.pinkies[pinky_id]
            wanted_phase = {"dock_complete": "docking_manual", "redock": "docking_manual", "received": "docked_manual"}.get(action)
            if wanted_phase is None:
                raise PinkyFleetError("PINKY_SIGNAL_INVALID", f"알 수 없는 신호 {action}")
            if self._take_fault("FLEET_REJECT", action) is not None:
                raise PinkyFleetError("PINKY_SIGNAL_REJECTED", "현재 임무의 확인 단계와 신호가 일치하지 않습니다. (모의 거부)")
            # redock은 dock_complete 뒤(docked_manual)에도 받는다.
            allowed = {wanted_phase, "docked_manual"} if action == "redock" else {wanted_phase}
            if pinky["phase"] not in allowed or not token or token != pinky["token"]:
                raise PinkyFleetError("PINKY_SIGNAL_REJECTED", "현재 임무의 확인 단계와 신호가 일치하지 않습니다.")
            if action == "redock":
                pinky.update(phase="driving", token=None, message="5번 다시 맞추는 중 (모의)", location="5번 정류장",
                             until=self.clock() + self.drive_s / 3, leg="to_station")
                return {"accepted": True, "simulated": True, "guided": offset is not None,
                        "message": "다시 맞추기 신호를 보냈습니다. (모의)"}
            elif action == "dock_complete":
                pinky.update(phase="docked_manual", token=uuid.uuid4().hex[:12],
                             message="도킹완료 · 정지 유지 · 물건 수령 완료 대기")
            else:
                # 경로가 있으면 핑키가 짐을 싣고 수령 구역까지 간다. 경로가 없으면(예전 주문) 예전처럼 주차장으로 가서
                # 그곳에서 짐을 내린다.
                leg = "to_zone" if pinky.get("route") else "to_home"
                pinky.update(phase="driving", token=None, message="물건 수령 완료 · 경로 재개", location="이동 중",
                             until=self.clock() + self.drive_s, leg=leg)
            return {"accepted": True, "simulated": True, "message": "현재 단계 확인 신호를 보냈습니다. (모의)"}

    # ---- 주차 판정 (천장 카메라) ----
    def parking(self, pinky_id: str) -> dict[str, Any]:
        """주차 판정. PINKY_MISALIGNED:n 사고가 있으면 n번 동안 자리 벗어남(MISALIGNED)을, 아니면 도착(ARRIVED)을 알린다.

        허용 오차(위치 30 mm, 각도 5도)와 offset 값은 화면 확인용 고정값이다."""
        with self._lock:
            remaining = self._take_fault("PINKY_MISALIGNED")
            if remaining is not None:
                # 남은 횟수를 하나 줄여 다시 맨 앞에 넣는다(인자가 비어 있으면 1번).
                count = int(remaining or 1) - 1
                if count > 0:
                    self.faults.insert(0, ("PINKY_MISALIGNED", str(count)))
                return {
                    "status": "MISALIGNED", "spot_id": "mock_station", "simulated": True, "pinky_id": pinky_id,
                    "reason": "모의: 정차 위치에서 벗어남", "captured_at": utc_now_iso("milliseconds"),
                    "offset": {"dx_mm": 48.0, "dy_mm": -21.0, "distance_mm": 52.4, "dyaw_deg": 6.5,
                               "forward_mm": 48.0, "left_mm": -21.0},
                    "tolerance": {"position_mm": 30.0, "yaw_deg": 5.0},
                }
            return {
                "status": "ARRIVED", "spot_id": "mock_station", "simulated": True, "pinky_id": pinky_id,
                "reason": "모의: 정차 위치 안", "captured_at": utc_now_iso("milliseconds"),
                "offset": {"dx_mm": 4.0, "dy_mm": 2.0, "distance_mm": 4.5, "dyaw_deg": 0.8},
                "tolerance": {"position_mm": 30.0, "yaw_deg": 5.0},
            }

    # ---- 팔 집기 (정책 실행) ----
    def pick(
        self, arm_id: str, object_class: str, shelf_id: str, pinky_id: str, cancel_event: threading.Event,
        progress: Callable[[str], None],
    ) -> dict[str, Any] | None:
        """모의 집기: pick_s초 걸려 선반의 물체 하나를 핑키 위로 옮긴다. 주문이 취소되면 None(아무것도 옮기지 않음).

        arms/pick.py의 실제 집기와 같은 호출 모양이다. 시간의 60%는 정책 실행, 40%는 놓고 home 복귀로 나눠 진행을 알린다."""
        progress(f"{arm_id}: 정책 실행 중 (모의)")
        if cancel_event.wait(self.pick_s * 0.6):
            return None
        progress(f"{arm_id}: 핑키 위에 놓고 home으로 복귀 중 (모의)")
        if cancel_event.wait(self.pick_s * 0.4):
            return None
        with self._lock:
            if self._take_fault("PICK_FAILED", object_class) is not None:
                pass  # 물체는 선반에 그대로 남는다
            elif self._take_fault("ITEM_LOST", object_class) is not None:
                self.shelves[shelf_id][object_class] -= 1
            else:
                self.shelves[shelf_id][object_class] -= 1
                self.loads[pinky_id][object_class] += 1
            self.shelves[shelf_id] += Counter()  # 0 이하가 된 개수를 지운다(Counter 덧셈은 양수만 남긴다)
        # 실제 워커처럼: home에는 돌아왔지만, 물체를 집어 놓았는지는 팔이 관측하지 않는다(확인은 카메라 개수로).
        return {"source": "mock_world", "simulated": True, "outcome": "ARM_HOME_PLACEMENT_NOT_VERIFIED", "arm_id": arm_id}
