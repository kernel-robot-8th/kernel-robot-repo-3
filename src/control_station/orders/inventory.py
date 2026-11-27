"""카메라로 본 선반 재고: 선반에 무엇이 있는지, 열린 주문이 얼마나 잡아 두었는지, 아직 팔 수 있는 것이 얼마인지.

선반 개수 세기(cameras/counts.py의 ShelfStockCounter 또는 모의 세계)는 이미 몇 프레임의 중앙값을 쓰지만, YOLO 개수는
읽을 때마다 여전히 흔들리고 선반에 손을 뻗은 팔은 뒤의 물체를 가린다. 그래서:

- 바뀐 개수는 같은 값이 `confirm_readings`번 연속으로 읽혀야 받아들인다.
- 팔이 일하는 동안(적재 자리가 잡혀 있거나 정책이 돌고 있음)은 읽은 값을 쓰지 않는다. 장부는 마지막으로 확정한
  개수를 유지하고, 집고 있는 물체는 여전히 그 주문의 예약으로 덮여 있다.
- `stale_after_s` 동안 쓸 만한 읽기가 없으면(카메라 없음, 검출기 준비 안 됨) 재고는 0이 아니라 '모름'이다.

우리가 집은 것은 일어나는 즉시 장부에 적는다: 주문 흐름이 선반에서 핑키 위로 옮겼다고 확인하면 그 선반 개수를
바로 하나 줄이고(장부), 카메라는 나중에 그것을 확인만 한다. 우리가 하지 않은 변화를 카메라가 확정하면(누가 물체를
가져갔거나 채워 넣음) 그렇게 기록한다.

팔 수 있는 수량 = 선반 개수 − 열린 주문의 예약(주문했고 아직 핑키 위로 옮기지 않은 것). 주문 창이 팔 수 있는 만큼이다.
재고를 모르는 물체는 주문을 닫고, 주문 창과 관제 화면 둘 다 그 이유를 보인다.

운영자는 물체마다 덮어쓸 수 있다(InventoryOverrides, 작은 JSON 파일에 저장): `closed`는 판매 중지, `manual`은 카메라
대신 운영자가 입력한 개수로 판다(예: 카메라가 꺼진 동안), `auto`는 카메라로 돌아간다.

연결: app.py가 InventoryMonitor를 만들어 2초마다 선반을 읽고, OrderDesk가 snapshot()으로 판매 가능 수량을 본다.
"""

from __future__ import annotations

import json
import threading
import time
from collections import deque
from collections.abc import Callable, Iterable, Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from control_station import utc_now_iso

CONFIRM_READINGS = 3  # 바뀐 개수를 받아들이기 전에 같은 값이 연속으로 읽혀야 하는 횟수
STALE_AFTER_S = 10.0  # 쓸 만한 읽기가 이만큼(초) 없으면 재고를 '모름'으로 본다
READ_PERIOD_S = 2.0  # 선반을 읽는 주기(초)
EVENT_LOG_SIZE = 40  # 변화 기록을 최근 몇 건까지 들고 있을지


@dataclass(frozen=True)
class StockReading:
    """선반을 한 번 본 결과: 선반별 개수, 또는 개수가 없는 이유. 팔이 일하는 중이면 `busy`."""

    at: float  # time.monotonic() 초
    shelves: Mapping[str, Mapping[str, int]] | None = None
    error: str | None = None
    busy: bool = False


class InventoryBook:
    """읽은 값으로 쌓은 확정 선반 개수. 입출력이 없어서 규칙을 평범한 읽기 값만으로 시험할 수 있다.

    스레드 안전하지 않다. InventoryMonitor가 자기 락 안에서만 부른다."""

    def __init__(self, *, confirm_readings: int = CONFIRM_READINGS, stale_after_s: float = STALE_AFTER_S) -> None:
        self.confirm_readings = confirm_readings
        self.stale_after_s = stale_after_s
        self._confirmed: dict[tuple[str, str], int] = {}
        self._pending: dict[tuple[str, str], tuple[int, int]] = {}  # 키 → (본 값, 연속으로 읽힌 횟수)
        self._has_confirmed = False  # 첫 전체 그림이 확정됐는지. 그 전에는 재고 상태가 'unknown'이다
        self._last_usable_at: float | None = None
        self._last_error: str | None = None
        self._busy = False

    def observe(self, reading: StockReading) -> list[dict[str, Any]]:
        """읽기 하나를 받는다. 이 읽기로 확정된 변화(채움 또는 바깥에서 가져감)를 순서대로 돌려준다.

        오류 읽기는 이유만 기억하고, busy 읽기는 장부를 건드리지 않는다. 읽기에 없는 키는 0개로 본다(선반에서
        사라진 물체도 같은 규칙으로 0이 확정된다)."""
        if reading.error is not None:
            self._last_error = reading.error
            return []
        self._busy = reading.busy
        if reading.busy or reading.shelves is None:
            return []
        self._last_error = None
        self._last_usable_at = reading.at
        seen = {(shelf, name): int(count) for shelf, counts in reading.shelves.items() for name, count in counts.items()}
        changes = []
        for key in sorted(set(seen) | set(self._confirmed) | set(self._pending)):
            value = seen.get(key, 0)
            # 첫 전체 그림 전에도, 이미 확정된 키가 같은 값으로 읽히면 확정된 채로 둔다. 예전에는 첫 그림 전이면
            # 확정된 키도 다시 대기로 돌아가, 한 번 흔들린 키와 차례가 어긋나 모든 키가 한꺼번에 확정되는
            # 읽기가 오지 않았다(재고가 계속 'unknown', 2026-11-25 수정).
            confirmed = self._confirmed.get(key, 0) if self._has_confirmed else self._confirmed.get(key)
            if value == confirmed:
                self._pending.pop(key, None)
                continue
            previous, streak = self._pending.get(key, (None, 0))
            streak = streak + 1 if previous == value else 1
            if streak < self.confirm_readings:
                self._pending[key] = (value, streak)
                continue
            self._pending.pop(key, None)
            before = self._confirmed.get(key, 0)
            self._confirmed[key] = value
            if self._has_confirmed and value != before:
                shelf, name = key
                # 우리가 집은 것은 이미 장부에 있으므로, 여기서 확정된 변화는 주문 흐름 밖에서 온 것이다.
                changes.append({"shelf_id": shelf, "object_class": name, "from": before, "to": value,
                                "kind": "RESTOCKED" if value > before else "REMOVED_OUTSIDE"})
        if not self._has_confirmed and not self._pending:
            self._has_confirmed = True  # 모든 키가 같은 값으로 `confirm_readings`번 읽힘: 첫 전체 그림
        return changes

    def book_moves(self, moves: Iterable[tuple[str, str]]) -> None:
        """확인된 우리 옮김들((선반, 물체) 하나씩): 카메라가 보기 전에 지금 선반에서 하나씩 뺀다.

        그 키의 대기 중인 읽기도 버린다: 옮기기 전 개수를 다시 확정하지 않게 하려는 것이다."""
        for shelf, name in moves:
            key = (shelf, name)
            self._confirmed[key] = max(0, self._confirmed.get(key, 0) - 1)
            self._pending.pop(key, None)

    def state(self, now: float) -> tuple[str, str | None]:
        """(상태, 이유): live(살아 있음), held(팔이 일하는 중이라 마지막 확정값), unknown(확정된 그림이 없거나
        최근 읽기가 없음)."""
        if not self._has_confirmed:
            return "unknown", self._last_error or "재고를 확인하는 중입니다"
        if self._busy:
            return "held", "팔이 작업 중이라 마지막 확인값을 씁니다"
        if self._last_usable_at is None or now - self._last_usable_at > self.stale_after_s:
            return "unknown", self._last_error or "선반 카메라의 새 값이 없습니다"
        return "live", None

    def on_shelves(self) -> dict[str, dict[str, int]]:
        """확정된 개수 중 0보다 큰 것만, 선반 → 물체 → 개수."""
        shelves: dict[str, dict[str, int]] = {}
        for (shelf, name), count in sorted(self._confirmed.items()):
            if count > 0:
                shelves.setdefault(shelf, {})[name] = count
        return shelves

    def snapshot(self, products: Iterable[Mapping[str, str]], reserved: Mapping[str, int], now: float,
                 overrides: Mapping[str, Mapping[str, Any]] | None = None) -> dict[str, Any]:
        """상품마다: 개수의 출처, 선반별 개수, 예약, 팔 수 있는 수량, 상태. 상태는 available(남음), sold_out(품절),
        unknown(카메라 개수 없음, 주문 닫힘), closed(운영자가 판매 중지)다. 운영자 덮어쓰기가 카메라보다 먼저다."""
        state, reason = self.state(now)
        shelves = self.on_shelves()
        overrides = overrides or {}
        rows = []
        for product in products:
            name = product["object_class"]
            override = overrides.get(name) or {}
            held = int(reserved.get(name, 0))
            per_shelf = {shelf: counts[name] for shelf, counts in shelves.items() if counts.get(name)}
            if override.get("mode") == "closed":
                source, total, available, status = "closed", sum(per_shelf.values()), 0, "closed"
            elif override.get("mode") == "manual":
                source, total = "manual", int(override.get("count", 0))
                available = max(0, total - held)
                status = "available" if available else "sold_out"
            elif state == "unknown":
                source, total, available, status = "camera", None, None, "unknown"
            else:
                source, total = "camera", sum(per_shelf.values())
                available = max(0, total - held)
                status = "available" if available else "sold_out"
            rows.append({"object_class": name, "label": product.get("label", name), "source": source,
                         "on_shelves": total, "per_shelf": per_shelf, "reserved": held, "available": available,
                         "status": status, "override": dict(override) or None})
        return {"state": state, "reason": reason, "products": rows}


class InventoryOverrides:
    """운영자의 물체별 설정. 서버를 다시 켜도 남도록 JSON 파일에 둔다. closed/manual만 저장하고 auto는 항목을 지운다."""

    MODES = frozenset({"auto", "closed", "manual"})

    def __init__(self, path: Path) -> None:
        self.path = path
        self._lock = threading.Lock()
        # 파일이 없거나 깨졌으면 빈 설정(모두 auto)으로 시작한다. 모르는 모드는 버린다.
        try:
            loaded = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            loaded = {}
        self._values: dict[str, dict[str, Any]] = {
            name: value for name, value in (loaded.items() if isinstance(loaded, dict) else [])
            if isinstance(value, dict) and value.get("mode") in {"closed", "manual"}
        }

    def all(self) -> dict[str, dict[str, Any]]:
        """저장된 덮어쓰기 전부의 복사본."""
        with self._lock:
            return {name: dict(value) for name, value in self._values.items()}

    def set(self, object_class: str, mode: str, count: object = None) -> dict[str, Any] | None:
        """물체 하나의 모드를 정하고 저장한 덮어쓰기를 돌려준다(auto면 None). 잘못된 입력은 ValueError.

        수동 수량은 0~99 정수다."""
        if mode not in self.MODES:
            raise ValueError("mode는 auto, closed, manual 중 하나여야 합니다")
        if mode == "manual" and (isinstance(count, bool) or not isinstance(count, int) or not 0 <= count <= 99):
            raise ValueError("수동 수량은 0에서 99 사이의 정수여야 합니다")
        value = None if mode == "auto" else ({"mode": "manual", "count": count} if mode == "manual" else {"mode": "closed"})
        with self._lock:
            if value is None:
                self._values.pop(object_class, None)
            else:
                self._values[object_class] = {**value, "set_at": utc_now_iso()}
            self.path.parent.mkdir(parents=True, exist_ok=True)
            # 임시 파일에 쓰고 replace로 바꿔 끼운다: 쓰다가 죽어도 반쯤 쓰인 파일이 남지 않는다.
            temporary = self.path.with_suffix(".tmp")
            temporary.write_text(json.dumps(self._values, ensure_ascii=False, indent=2), encoding="utf-8")
            temporary.replace(self.path)
            return dict(self._values[object_class]) if value is not None else None


class InventoryMonitor:
    """백그라운드 스레드에서 `period_s`마다 선반을 읽어 장부와 짧은 변화 기록을 유지한다.

    입력은 모두 함수로 받는다: read_shelves(선반 개수), busy(팔이 일하는 중인지), products(주문할 수 있는 물체),
    reserved(열린 주문의 예약, TaskStore.reserved_units), transfers(확인된 옮김, TaskStore.confirmed_transfers)."""

    def __init__(self, read_shelves: Callable[[], Mapping[str, Any]], busy: Callable[[], bool],
                 products: Callable[[], list[dict[str, str]]], reserved: Callable[[], Mapping[str, int]],
                 transfers: Callable[[], list[tuple[str, str, str]]], book: InventoryBook | None = None,
                 overrides: InventoryOverrides | None = None, period_s: float = READ_PERIOD_S,
                 clock: Callable[[], float] = time.monotonic) -> None:
        self.read_shelves = read_shelves
        self.busy = busy
        self.products = products
        self.reserved = reserved
        self.transfers = transfers
        # 이 모니터를 시작하기 전에 있었던 옮김은 이미 카메라가 보는 개수에 들어가 있으므로 다시 빼지 않는다.
        self._booked = {step_id for step_id, _, _ in transfers()}
        self.book = book or InventoryBook()
        self.overrides = overrides
        self.period_s = period_s
        self.clock = clock
        self._events: deque[dict[str, Any]] = deque(maxlen=EVENT_LOG_SIZE)
        self._simulated = False
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def read_once(self) -> None:
        """한 번 읽는다: 새로 확인된 옮김을 장부에 적고, 팔이 쉬는 중이면 선반을 읽어 장부에 넣는다."""
        moves = [(step_id, shelf, name) for step_id, shelf, name in self.transfers() if step_id not in self._booked]
        busy = bool(self.busy())
        try:
            answer = None if busy else self.read_shelves()
            if answer is not None:
                self._simulated = bool(answer.get("simulated"))
            reading = StockReading(self.clock(), None if answer is None else answer["shelves"], busy=busy)
        except Exception as exc:  # 카메라 없음, 검출기 준비 안 됨, 새 프레임 없음: 재고가 '모름'이 된다
            reading = StockReading(self.clock(), error=f"선반 재고를 읽지 못했습니다: {exc}")
        with self._lock:
            self.book.book_moves((shelf, name) for _, shelf, name in moves)
            self._booked.update(step_id for step_id, _, _ in moves)
            for change in self.book.observe(reading):
                self._events.append({"at": utc_now_iso(), **change})

    def snapshot(self) -> dict[str, Any]:
        """화면과 주문 창이 보는 재고: 상품별 행, 상태, 최근 변화(최신부터)."""
        reserved = self.reserved()
        overrides = self.overrides.all() if self.overrides is not None else {}
        with self._lock:
            view = self.book.snapshot(self.products(), reserved, self.clock(), overrides)
            view["events"] = list(self._events)[::-1]
        # 모의 세계의 개수는 카메라 개수로 보이지 않게 그렇다고 표시한다.
        view["simulated"] = self._simulated
        if self._simulated:
            for row in view["products"]:
                if row["source"] == "camera":
                    row["source"] = "mock_world"
        view["checked_at"] = utc_now_iso()
        return view

    def set_override(self, object_class: str, mode: str, count: object = None) -> dict[str, Any] | None:
        """운영자가 물체를 판매 중지하거나, 손으로 센 개수를 넣거나, 카메라로 되돌린다. 변화 기록에도 남긴다."""
        if self.overrides is None:
            raise ValueError("재고 수정 기능이 켜져 있지 않습니다")
        if object_class not in {product["object_class"] for product in self.products()}:
            raise ValueError(f"주문할 수 있는 물건이 아닙니다: {object_class}")
        stored = self.overrides.set(object_class, mode, count)
        with self._lock:
            self._events.append({"at": utc_now_iso(), "kind": "OVERRIDE", "object_class": object_class, "mode": mode,
                                 "count": count if mode == "manual" else None})
        return stored

    def start(self) -> InventoryMonitor:
        """읽기 루프 스레드를 띄운다."""
        def loop() -> None:
            while not self._stop.is_set():
                self.read_once()
                self._stop.wait(self.period_s)
        self._thread = threading.Thread(target=loop, name="inventory-monitor", daemon=True)
        self._thread.start()
        return self

    def close(self) -> None:
        """루프를 멈추고 스레드를 최대 5초 기다린다."""
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=5)

