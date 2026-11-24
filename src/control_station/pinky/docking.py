"""적재 자리(5번)에서 천장 카메라와 핑키가 주고받는 일: 선 뒤의 "도착" 판정과, 들어오는 동안의 방향 안내.

1) 도착 판정 (`DockHandshake`, `DockSupervisor`, `DockAutoLoop`)
   핑키는 5번에 서서 기다린다(단계 `docking_manual`). 천장 카메라가 핑키의 마커를 그 핑키 자신의 등록 자리와
   비교하고, 판정을 차례로 받아 핑키에게 무엇을 돌려보낼지 정한다.

     ARRIVED가 `arrived_checks`번 연속             -> COMPLETE  호출한 쪽이 dock_complete를 보낸다
     MISALIGNED가 `misaligned_checks`번 연속      -> REDOCK    카메라가 잰 어긋남을 같이 보내, 그만큼 고쳐 다시 들어오게 한다
     redock을 보낸 뒤                              -> WAIT      핑키가 자리를 떠났다가 다시 설 때까지 기다린다.
                                                                redock 전 화면을 두 번 판정하면 곧바로 redock을 또 보내기 때문이다
     핑키가 다시 섬                                -> redock 결과(전 -> 후)를 한 번 기록한다
     이 핑키의 마커 등록이 없음                    -> OPERATOR  사람이 보고 확인한다
     카메라는 되는데 핑키 마커가 `blind_s` 동안
     보이지 않음(NOT_FOUND)                        -> REDOCK    어긋남 없이 "다시 들어와". 주문이 이어져야 하므로 사람을
                                                                기다리지 않고 다시 부른다(2026-11-19). 최대 `max_redocks`번
     다른 이유로 `blind_s` 동안 판정하지 못함      -> OPERATOR  노드가 꺼졌거나 화면이 움직였다. 다시 불러도 소용없다
     `max_redocks`를 다 쓰고도 어긋남              -> OPERATOR

   OPERATOR는 스스로 아무것도 움직이지 않는다. 사람이 확인할 때까지 핑키는 서 있고 팔도 움직이지 않는다.
   이 부분은 판단만 한다. 신호를 보내고 기록을 남기는 것은 호출한 쪽의 일이다(주문이면 주문 흐름,
   테스트 화면에서 몬 핑키면 DockSupervisor).

2) 들어오는 동안 방향 안내 (`DockGuideRelay`, 2026-11-18)
   주차 노드는 등록 자리 하나의 마커를 실시간으로 따라간다(/pinky_parking/state). 핑키가 등록 자리에서 얼마나
   벗어났는지를 프레임마다 알려 준다. 그 마커를 단 핑키가 임무 중이고 화면에 보이는 동안, 새 어긋남을 핑키 관제
   (POST /api/robots/{id}/dock-guide)로 보내고 핑키 관제가 핑키에게 넘긴다. 핑키는 5번으로 들어가는 동안 이 값을
   보며 방향을 잡고(pinky_fleet_dashboard의 mission_behaviors.DockEntry), 카메라가 "자리에 왔다"고 보는 곳에서 멈춘다.
   그래서 지도가 말한 곳에 섰다가 다시 불려 들어가는 대신 한 번에 자리에 들어온다.

   실시간 상태는 팔 단계의 조건이 되지 않는다는 계약은 그대로다. 도착은 여전히 새 프레임으로 확정한 판정(1번)이
   정한다. 여기서는 같은 숫자로 마지막 25 cm 주행만 안내하고, 안내가 끊긴 핑키는 예전처럼 자기 오도메트리로 들어온다.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable


# ──── 1) 적재 자리 도착 판정 ──────────────────────────────────────────────────────────────────────────────────────

WAIT, REDOCK, COMPLETE, OPERATOR = "WAIT", "REDOCK", "COMPLETE", "OPERATOR"
NOT_SEEN = frozenset({"NOT_FOUND", "NOT_IN_VIEW"})  # the camera works, the marker is not there (the node's, the virtual camera's word)
REPEAT_MAX = 20  # entries of the repeat test at one stop: well under the Pinky's own limit of redocks per stop (30)
DOCKING_PHASE = "docking_manual"


def redock_offset(offset: dict[str, Any] | None) -> dict[str, float] | None:
    """The camera's offset in the Pinky's own axes for a guided redock: forward_mm, left_mm, dyaw_deg (+ = turned
    left). None when the judgment has no Pinky-frame offset (the spot's marker_forward_axis is not registered)."""
    try:
        values = {key: float((offset or {})[key]) for key in ("forward_mm", "left_mm", "dyaw_deg")}
    except (KeyError, TypeError, ValueError):
        return None
    return values if all(math.isfinite(value) for value in values.values()) else None


def _distance(observation: dict[str, Any] | None) -> float | None:
    try:
        value = float(((observation or {}).get("offset") or {})["distance_mm"])
    except (KeyError, TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


@dataclass
class Decision:
    action: str  # WAIT | REDOCK | COMPLETE | OPERATOR
    reason: str  # NOT_DOCKING, SETTLING, ARRIVED, MISALIGNED, NOT_SEEN, REDOCK_PENDING, NOT_REGISTERED, CAMERA_BLIND, REDOCKS_USED
    message: str
    observation: dict[str, Any] | None = None
    guide: dict[str, float] | None = None  # REDOCK: what to send along (None: the Pinky comes in again as before)
    attempt: int = 0  # redocks sent so far, this one included for REDOCK
    # Once, on the first judgment after a redock: {"attempt", "guided", "before_mm", "after_mm", "status"}
    result: dict[str, Any] | None = None
    not_applied: bool = False  # once: a redock was sent but the Pinky never left the spot

    def as_dict(self) -> dict[str, Any]:
        return {"action": self.action, "reason": self.reason, "message": self.message, "attempt": self.attempt,
                "guide": self.guide, "result": self.result, "not_applied": self.not_applied,
                "status": (self.observation or {}).get("status"), "offset": (self.observation or {}).get("offset")}


class DockHandshake:
    """One Pinky's one stop at the loading spot."""

    def __init__(self, *, arrived_checks: int = 2, misaligned_checks: int = 3, max_redocks: int = 5,
                 blind_s: float = 20.0, redock_wait_s: float = 15.0,
                 clock: Callable[[], float] = time.monotonic) -> None:
        self.arrived_checks, self.misaligned_checks, self.max_redocks = arrived_checks, misaligned_checks, max_redocks
        self.blind_s, self.redock_wait_s, self.clock = blind_s, redock_wait_s, clock
        self.redocks = 0
        self._arrived = self._misaligned = 0
        self._blind_since: float | None = None
        # The redock in flight: {"attempt", "guided", "before_mm", "sent_at", "left"}; "left" once the Pinky was seen
        # outside docking_manual (redocking) after it was sent.
        self._pending: dict[str, Any] | None = None

    def redock_sent(self, guided: bool, before_mm: float | None = None) -> None:
        """The caller sent the redock this handshake asked for (REDOCK) and the dashboard accepted it."""
        self._pending = {"attempt": self.redocks, "guided": bool(guided), "before_mm": before_mm,
                         "sent_at": self.clock(), "left": False}
        self._arrived = self._misaligned = 0
        self._blind_since = None

    def step(self, phase: str | None, observation: dict[str, Any] | None) -> Decision:
        """`observation`: the camera's judgment of this Pinky ({"status", "offset", ...}), asked only while the
        Pinky stands in docking_manual (None otherwise, or when asking failed without a status)."""
        if phase != DOCKING_PHASE:
            if self._pending is not None:
                self._pending["left"] = True
            self._arrived = self._misaligned = 0
            self._blind_since = None
            return Decision(WAIT, "NOT_DOCKING", "핑키가 적재 자리에 서기를 기다려요", attempt=self.redocks)
        result, not_applied = None, False
        if self._pending is not None and not self._pending["left"]:
            if self.clock() - self._pending["sent_at"] < self.redock_wait_s:
                return Decision(WAIT, "REDOCK_PENDING", "다시 맞추기 신호를 보냈어요 · 핑키가 물러났다 들어오기를 기다려요",
                                attempt=self.redocks)
            self._pending, not_applied = None, True  # it never moved: judge what stands there now
        observation = observation or {"status": "PARKING_CHECK_FAILED", "reason": "카메라 판정을 받지 못했어요"}
        status = observation.get("status")
        distance = _distance(observation)
        if self._pending is not None and status in {"ARRIVED", "MISALIGNED"}:
            result = {"attempt": self._pending["attempt"], "guided": self._pending["guided"],
                      "before_mm": self._pending["before_mm"], "after_mm": distance, "status": status}
            self._pending = None
        made = dict(observation=observation, attempt=self.redocks, result=result, not_applied=not_applied)
        if status == "NOT_REGISTERED":
            self._arrived = self._misaligned = 0
            return Decision(OPERATOR, "NOT_REGISTERED", "이 핑키의 마커는 카메라에 등록돼 있지 않아요 · 눈으로 보고 확인해 주세요", **made)
        if status == "ARRIVED":
            self._arrived, self._misaligned, self._blind_since = self._arrived + 1, 0, None
            if self._arrived >= self.arrived_checks:
                return Decision(COMPLETE, "ARRIVED", "정상 위치 · 다음 단계로", **made)
            return Decision(WAIT, "SETTLING", f"정상 위치로 보여요 · 한 번 더 확인해요 ({self._arrived}/{self.arrived_checks})", **made)
        if status == "MISALIGNED":
            self._arrived, self._misaligned, self._blind_since = 0, self._misaligned + 1, None
            if self._misaligned < self.misaligned_checks:
                return Decision(WAIT, "MISALIGNED", f"위치가 어긋나 보여요 · 계속 그런지 확인해요 ({self._misaligned}/{self.misaligned_checks})", **made)
            if self.redocks >= self.max_redocks:
                return Decision(OPERATOR, "REDOCKS_USED",
                                f"다시 맞추기를 {self.max_redocks}번 했는데도 어긋나 있어요 · 눈으로 보고 확인하거나 핑키를 바로잡아 주세요", **made)
            self.redocks += 1
            self._misaligned = 0
            made["attempt"] = self.redocks
            return Decision(REDOCK, "MISALIGNED", f"위치 어긋남 · 다시 맞춰 들어가기 {self.redocks}/{self.max_redocks}",
                            guide=redock_offset(observation.get("offset")), **made)
        # Not judged: marker hidden, node down, view moved, still moving.
        self._arrived = self._misaligned = 0
        now = self.clock()
        if self._blind_since is None:
            self._blind_since = now
        if now - self._blind_since >= self.blind_s and status in NOT_SEEN and self.redocks < self.max_redocks:
            # The camera looks at the spot and the Pinky is not on it: call it in again rather than wait for a person.
            self.redocks += 1
            self._blind_since = None
            made["attempt"] = self.redocks
            return Decision(REDOCK, "NOT_SEEN", f"카메라에 핑키가 안 보여요 · 다시 들어오라고 해요 {self.redocks}/{self.max_redocks}",
                            guide=None, **made)
        if now - self._blind_since >= self.blind_s:
            return Decision(OPERATOR, "CAMERA_BLIND",
                            f"카메라가 {int(self.blind_s)}초 넘게 판정하지 못했어요 ({status}) · 눈으로 보고 확인해 주세요", **made)
        return Decision(WAIT, "CAMERA_BLIND", f"카메라 판정을 기다려요 ({status})", **made)


class DockSupervisor:
    """The same handshake for a Pinky driven without an order (the test screen): one step per call.

    status(pinky_id) -> {"phase", "mission_token", "route_status"}; judge(pinky_id) -> the camera's judgment at that
    Pinky's own registration; signal(pinky_id, action, offset) sends dock_complete / redock and raises on refusal;
    in_order(pinky_id) is True while an order's own flow talks to that Pinky (this then only watches).

    The repeat test of the entry (step(..., repeat=N), 2026-11-19): each time the camera confirms the Pinky on its
    spot, that entry is written down as one success and, until N are collected, the Pinky is sent out and in again
    (a redock without an offset) instead of dock_complete. Entries that came in off the spot are written down too
    (and corrected by the handshake as always). `record(trial)` keeps every trial (a JSON-lines file in the app)."""

    def __init__(self, status: Callable[[str], dict[str, Any]], judge: Callable[[str], dict[str, Any]],
                 signal: Callable[[str, str, dict[str, float] | None], dict[str, Any]],
                 in_order: Callable[[str], bool] = lambda pinky_id: False, history: int = 20,
                 record: Callable[[dict[str, Any]], None] | None = None, **limits: Any) -> None:
        self.status, self.judge, self.signal, self.in_order = status, judge, signal, in_order
        self.record = record
        self.limits, self.history = limits, history
        self._lock = threading.Lock()
        self._runs: dict[str, dict[str, Any]] = {}  # pinky id -> {"key", "handshake", "log", "done"}

    def _trial(self, run: dict[str, Any], pinky_id: str, outcome: str, decision: Decision) -> None:
        """One entry of the repeat test, as the camera judged it first."""
        offset = (decision.observation or {}).get("offset") or {}
        trial = {"at": time.time(), "pinky_id": pinky_id, "n": len(run["trials"]) + 1, "outcome": outcome,
                 **{key: offset.get(key) for key in ("forward_mm", "left_mm", "dyaw_deg", "distance_mm")}}
        run["trials"].append(trial)
        if self.record is not None:
            try:
                self.record(trial)
            except Exception:  # the record is evidence, not a condition for the Pinky
                pass

    def step(self, pinky_id: str, auto: bool, repeat: int | None = 0) -> dict[str, Any]:
        """Judge once and, with `auto`, act on it. Returns the decision, what was sent, and this stop's log.
        repeat: how many confirmed entries to collect at this stop before dock_complete (0: the first one ends it);
        None keeps what this stop already has (the server's own loop, which must not end a repeat test the test
        screen asked for)."""
        with self._lock:
            status = self.status(pinky_id)
            phase = status.get("phase")
            if phase not in {DOCKING_PHASE, "redocking"}:
                self._runs.pop(pinky_id, None)
                return {"pinky_id": pinky_id, "active": False, "phase": phase}
            if self.in_order(pinky_id):
                self._runs.pop(pinky_id, None)
                return {"pinky_id": pinky_id, "active": False, "phase": phase, "in_order": True,
                        "message": "주문 흐름이 이 핑키의 도착을 확인하고 있어요"}
            run = self._runs.get(pinky_id)
            if run is None:
                run = self._runs[pinky_id] = {"handshake": DockHandshake(**self.limits), "log": [], "done": False,
                                              "trials": [], "judged": False, "repeat": 0}
            if repeat is not None:
                run["repeat"] = max(0, min(REPEAT_MAX, int(repeat or 0)))
            handshake: DockHandshake = run["handshake"]
            observation = None
            if phase == DOCKING_PHASE and not run["done"]:
                try:
                    observation = self.judge(pinky_id)
                except Exception as exc:  # node down, view moved, no frame
                    observation = {"status": getattr(exc, "error_code", "PARKING_CHECK_FAILED"), "reason": str(exc)}
            if run["done"]:
                return {"pinky_id": pinky_id, "active": True, "phase": phase, "auto": bool(auto), "sent": None,
                        "done": True, "log": list(run["log"]), "repeat": self._repeat(run),
                        "decision": {"action": WAIT, "reason": "COMPLETE_SENT", "message": "도킹완료를 보냈어요"}}
            decision = handshake.step(phase, observation)
            sent = None
            if phase != DOCKING_PHASE:
                run["judged"] = False  # it is on its way in again: the next judgment is a new entry
            elif run["repeat"] and auto and not run["judged"] and decision.action in {REDOCK, COMPLETE}:
                run["judged"] = True
                self._trial(run, pinky_id, "SUCCESS" if decision.action == COMPLETE else decision.reason, decision)
            successes = sum(1 for trial in run["trials"] if trial["outcome"] == "SUCCESS")
            if decision.action == COMPLETE and auto and run["repeat"] and successes < run["repeat"]:
                # Collected, not finished: out and in again. Not one of the handshake's own correcting redocks.
                try:
                    self.signal(pinky_id, "redock", None)
                    handshake.redocks = 0
                    handshake.redock_sent(False, _distance(decision.observation))
                    self._note(run, "REPEAT", f"반복 테스트 {successes}/{run['repeat']}번 성공 · 다시 나갔다 들어오라고 해요",
                               {"successes": successes, "target": run["repeat"]})
                    sent = "redock"
                except Exception as exc:
                    self._note(run, "REDOCK_FAILED", f"반복 테스트의 다시 들어오기 요청 실패: {exc}", None)
                return {"pinky_id": pinky_id, "active": True, "phase": phase, "auto": True, "sent": sent,
                        "decision": decision.as_dict(), "log": list(run["log"]), "repeat": self._repeat(run)}
            if decision.result is not None:
                self._note(run, "REDOCK_RESULT", _result_text(decision.result), decision.result)
            if decision.not_applied:
                self._note(run, "REDOCK_NOT_APPLIED", "다시 맞추기 신호를 보냈지만 핑키가 움직이지 않았어요", None)
            if decision.action == REDOCK:
                if auto:
                    try:
                        answer = self.signal(pinky_id, "redock", decision.guide)
                        guided = bool(decision.guide) and answer.get("guided") is not False
                        handshake.redock_sent(guided, _distance(decision.observation))
                        sent = "redock"
                        self._note(run, "REDOCK", decision.message + (" · 카메라가 본 만큼 고쳐서" if guided else " · 보정 없이"),
                                   {"attempt": decision.attempt, "guide": decision.guide, "guided": guided})
                    except Exception as exc:
                        handshake.redocks -= 1  # not sent: it does not use up one of the redocks
                        self._note(run, "REDOCK_FAILED", f"다시 맞추기 요청 실패: {exc}", {"attempt": decision.attempt})
                else:
                    # Not sent: the operator presses it. The count follows what was sent, and the next judgment
                    # that is still off says REDOCK again instead of counting from one.
                    handshake.redocks -= 1
                    if decision.reason == "MISALIGNED":
                        handshake._misaligned = handshake.misaligned_checks
            elif decision.action == COMPLETE and auto:
                try:
                    self.signal(pinky_id, "dock_complete", None)
                    run["done"], sent = True, "dock_complete"
                    self._note(run, "COMPLETE", "카메라 확인 · 도킹완료를 보냈어요", {"offset": (decision.observation or {}).get("offset")})
                except Exception as exc:
                    self._note(run, "COMPLETE_FAILED", f"도킹완료 요청 실패: {exc}", None)
            elif decision.action == OPERATOR and (not run["log"] or run["log"][-1]["event"] != decision.reason):
                self._note(run, decision.reason, decision.message, None)
            return {"pinky_id": pinky_id, "active": True, "phase": phase, "auto": bool(auto), "sent": sent,
                    "decision": decision.as_dict(), "log": list(run["log"]), "repeat": self._repeat(run)}

    @staticmethod
    def _repeat(run: dict[str, Any]) -> dict[str, Any] | None:
        if not run.get("repeat") and not run.get("trials"):
            return None
        trials = run["trials"]
        return {"target": run.get("repeat", 0), "trials": len(trials),
                "successes": sum(1 for trial in trials if trial["outcome"] == "SUCCESS"), "last": trials[-5:]}

    def _note(self, run: dict[str, Any], event: str, message: str, detail: dict[str, Any] | None) -> None:
        run["log"].append({"at": time.time(), "event": event, "message": message, "detail": detail})
        del run["log"][:-self.history]


class DockAutoLoop:
    """The handshake run by the control station itself (2026-11-24): a Pinky that stops at the loading spot is judged
    by the camera and told the result (dock_complete, or a redock when it stands off the spot) without a test screen
    open in a browser and without anybody pressing 도킹완료. Before, the steps came only from the test screen's poll,
    and only with its box ticked.

    step(pinky_id) is DockSupervisor.step with auto and repeat=None; phase(pinky_id) is the Pinky's mission phase,
    read first so a Pinky elsewhere costs one cached status and no camera check; enabled() is the switch (the
    operator's box, CONTROL_STATION_DOCK_AUTO=0 starts with it off). An order's own flow still does its own handshake
    (the supervisor leaves a Pinky in an order alone)."""

    PHASES = frozenset({DOCKING_PHASE, "redocking"})

    def __init__(self, pinky_ids: Any, phase: Callable[[str], str | None], step: Callable[[str], dict[str, Any]],
                 enabled: Callable[[], bool] = lambda: True, period_s: float = 1.0) -> None:
        self.pinky_ids, self.phase, self.step, self.enabled, self.period_s = tuple(pinky_ids), phase, step, enabled, period_s
        self.last: dict[str, dict[str, Any]] = {}  # pinky id -> the last answer of step, or {"error"}
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def tick(self) -> list[str]:
        """One round; returns the Pinkys stepped."""
        if not self.enabled():
            return []
        stepped = []
        for pinky_id in self.pinky_ids:
            try:
                if self.phase(pinky_id) not in self.PHASES:
                    self.last.pop(pinky_id, None)
                    continue
                self.last[pinky_id] = self.step(pinky_id)
                stepped.append(pinky_id)
            except Exception as exc:  # the fleet dashboard is away: the next round tries again
                self.last[pinky_id] = {"error": str(exc)}
        return stepped

    def start(self) -> "DockAutoLoop":
        def loop() -> None:
            while not self._stop.wait(self.period_s):
                self.tick()
        self._thread = threading.Thread(target=loop, name="dock-auto", daemon=True)
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()


def _result_text(result: dict[str, Any]) -> str:
    before, after = result.get("before_mm"), result.get("after_mm")
    numbers = f"{before:.0f} mm → {after:.0f} mm" if before is not None and after is not None else "오차 값 없음"
    how = "카메라 보정" if result.get("guided") else "보정 없이"
    verdict = "정상 위치" if result.get("status") == "ARRIVED" else "아직 어긋남"
    return f"다시 맞추기 {result.get('attempt')}번째({how}) 결과: {numbers} · {verdict}"


# ──── 2) 들어오는 동안 방향 안내 ────────────────────────────────────────────────────────────────────────────────────

TRACKED_STATES = frozenset({"MOVING", "IN_POSITION", "OFF_POSITION"})
PARKED_PHASES = frozenset({"parked"})


def guide_from(live: dict[str, Any] | None) -> dict[str, float] | None:
    """forward_mm, left_mm, dyaw_deg from a live snapshot while the marker is tracked, else None."""
    if not isinstance(live, dict) or live.get("state") not in TRACKED_STATES:
        return None
    offset = live.get("offset") or {}
    try:
        guide = {key: float(offset[key]) for key in ("forward_mm", "left_mm", "dyaw_deg")}
    except (KeyError, TypeError, ValueError):
        return None
    return guide if all(math.isfinite(value) for value in guide.values()) else None


class DockGuideRelay:
    """snapshot() -> the live state; phase(pinky_id) -> the Pinky's mission phase; send(pinky_id, guide) -> True when
    the dashboard took it. One frame is sent once; after a refusal or an error it stays quiet for `retry_s`."""

    def __init__(self, snapshot: Callable[[], dict[str, Any]], pinky_id: str,
                 phase: Callable[[str], str | None], send: Callable[[str, dict[str, float]], bool], *,
                 period_s: float = 0.1, retry_s: float = 3.0, clock: Callable[[], float] = time.monotonic) -> None:
        self.snapshot, self.pinky_id, self.phase, self.send = snapshot, pinky_id, phase, send
        self.period_s, self.retry_s, self.clock = period_s, retry_s, clock
        self.sent = self.failed = 0
        self.last: dict[str, Any] | None = None  # the last guide sent, with when
        self._frame: Any = None
        self._quiet_until = 0.0
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def tick(self) -> str:
        """One round; returns what happened (for the status line and the tests)."""
        now = self.clock()
        if now < self._quiet_until:
            return "QUIET"
        live = self.snapshot()
        guide = guide_from(live)
        if guide is None:
            return "NOT_TRACKED"
        frame = live.get("frame_sequence")
        if frame is not None and frame == self._frame:
            return "SAME_FRAME"  # a stalled camera must not look fresh to the Pinky
        try:
            if self.phase(self.pinky_id) in PARKED_PHASES:
                return "PARKED"
            self._frame = frame
            if self.send(self.pinky_id, guide):
                self.sent += 1
                self.last = {**guide, "at": time.time()}
                return "SENT"
        except Exception:  # the dashboard is away: try again later, the Pinky drives in without the guide
            pass
        self.failed += 1
        self._quiet_until = now + self.retry_s
        return "REFUSED"

    def start(self) -> None:
        def loop() -> None:
            while not self._stop.wait(self.period_s):
                self.tick()
        self._thread = threading.Thread(target=loop, name="dock-guide-relay", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()

    def status(self) -> dict[str, Any]:
        return {"pinky_id": self.pinky_id, "sent": self.sent, "failed": self.failed, "last": self.last}
