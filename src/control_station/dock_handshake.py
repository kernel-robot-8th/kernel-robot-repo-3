"""The exchange between the ceiling camera and a Pinky standing at the loading spot, up to "arrived".

The Pinky stops at station 5 and waits (phase docking_manual). The camera judges its marker against that Pinky's own
registration. From the judgments, one after another, this decides what is said back to the Pinky:

  ARRIVED, seen `arrived_checks` times in a row          -> COMPLETE  (the caller sends dock_complete)
  MISALIGNED, seen `misaligned_checks` times in a row     -> REDOCK    (the caller sends redock with the camera's
                                                             offset, so the Pinky comes in corrected by that much)
  after a redock was sent                                 -> WAIT      until the Pinky has left the spot and stands
                                                             there again: the picture from before the redock is not
                                                             judged twice (it would send a second redock at once)
  the Pinky is back                                       -> the redock's result is reported once: before -> after
  no registration for this Pinky's marker                 -> OPERATOR  (a person looks and confirms)
  the camera works but does not see the Pinky's marker
  (NOT_FOUND) for `blind_s`                               -> REDOCK    without an offset: "come in again" (2026-11-19:
                                                             the order has to go on, so it is called in again
                                                             instead of waiting for a person), up to `max_redocks`
  the camera cannot judge for `blind_s` for another reason -> OPERATOR  (node down, view moved: calling the Pinky in
                                                             again would change nothing)
  `max_redocks` used and still misaligned                 -> OPERATOR

OPERATOR never moves anything by itself: the Pinky keeps standing and the arms stay still until a person confirms.
This module only decides; sending the signals and keeping the records is the caller's (the order flow for an order,
DockSupervisor for a Pinky driven from the test screen)."""
from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable

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

    def step(self, pinky_id: str, auto: bool, repeat: int = 0) -> dict[str, Any]:
        """Judge once and, with `auto`, act on it. Returns the decision, what was sent, and this stop's log.
        repeat: how many confirmed entries to collect at this stop before dock_complete (0: the first one ends it)."""
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
                                              "trials": [], "judged": False}
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


def _result_text(result: dict[str, Any]) -> str:
    before, after = result.get("before_mm"), result.get("after_mm")
    numbers = f"{before:.0f} mm → {after:.0f} mm" if before is not None and after is not None else "오차 값 없음"
    how = "카메라 보정" if result.get("guided") else "보정 없이"
    verdict = "정상 위치" if result.get("status") == "ARRIVED" else "아직 어긋남"
    return f"다시 맞추기 {result.get('attempt')}번째({how}) 결과: {numbers} · {verdict}"
