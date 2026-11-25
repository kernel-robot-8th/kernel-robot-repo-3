"""Loading at station 5 for a Pinky driven without an order (전체 주행 테스트), 2026-11-25.

The chain the operator asked for, the same one an order runs (order_flow: PICK_ITEM -> VERIFY_LOAD_TOTAL ->
NOTIFY_PINKY_RECEIVED), here without an order:

  docked (docked_manual) -> the arm picks (OMX; with no arm attached a person puts the object on the tray)
  -> the ceiling camera's YOLO counts the tray -> loaded -> `received` to the Pinky, which drives to its receiving zone

`step(pinky_id)` is one round, called by the control station's own loop (DockAutoLoop) while the box
"카메라 판정대로 자동 처리" is on. It never touches a Pinky an order's flow is talking to.

status(pinky_id) -> {"phase", ...}; count(pinky_id) -> {"counts": {class: n}, ...} (camera_counts.PinkyLoadCounter);
signal(pinky_id, action, offset) sends the Pinky signal; pick(pinky_id) asks an arm to load one object and returns
when it is back (None: no arm, a person loads).
"""

from __future__ import annotations

import threading
import time
from collections.abc import Callable
from typing import Any

LOADING_PHASE = "docked_manual"


class LoadRelay:
    def __init__(self, status: Callable[[str], dict[str, Any]], count: Callable[[str], dict[str, Any]],
                 signal: Callable[[str, str, dict[str, float] | None], dict[str, Any]],
                 in_order: Callable[[str], bool] = lambda pinky_id: False,
                 pick: Callable[[str], dict[str, Any]] | None = None, target: int = 1, confirm_checks: int = 2,
                 blind_s: float = 20.0, history: int = 20) -> None:
        self.status, self.count, self.signal, self.in_order, self.pick = status, count, signal, in_order, pick
        self.target, self.confirm_checks, self.blind_s, self.history = target, confirm_checks, blind_s, history
        self._lock = threading.Lock()
        self._runs: dict[str, dict[str, Any]] = {}
        self.last: dict[str, dict[str, Any]] = {}  # pinky id -> the last answer, for the test screen

    def view(self) -> dict[str, dict[str, Any]]:
        return dict(self.last)

    def step(self, pinky_id: str) -> dict[str, Any]:
        with self._lock:
            answer = self._step(pinky_id)
            if answer.get("active"):
                self.last[pinky_id] = answer
            else:
                self.last.pop(pinky_id, None)
            return answer

    def _step(self, pinky_id: str) -> dict[str, Any]:
        phase = self.status(pinky_id).get("phase")
        if phase != LOADING_PHASE:
            self._runs.pop(pinky_id, None)
            return {"pinky_id": pinky_id, "active": False, "phase": phase}
        if self.in_order(pinky_id):
            self._runs.pop(pinky_id, None)
            return {"pinky_id": pinky_id, "active": False, "phase": phase, "in_order": True}
        run = self._runs.get(pinky_id)
        if run is None:
            run = self._runs[pinky_id] = {"stage": "pick", "seen": 0, "log": [], "since": time.monotonic(),
                                          "counts": {}, "blind_since": None}
        if run["stage"] == "pick":
            if self.pick is None:
                self._note(run, "PICK_BY_HAND", "팔이 연결되지 않았어요 · 쟁반에 물건을 올려 주세요")
            else:
                try:
                    self._note(run, "PICK", "팔에 집기를 요청했어요")
                    self.pick(pinky_id)
                    self._note(run, "PICKED", "팔이 집어 올렸어요 · 카메라로 확인해요")
                except Exception as exc:
                    run["stage"] = "problem"
                    self._note(run, "PICK_FAILED", f"팔 집기 실패: {exc} · 쟁반을 확인해 주세요")
                    return self._answer(pinky_id, run, "attention")
            run["stage"] = "verify"
        if run["stage"] == "verify":
            try:
                observation = self.count(pinky_id)
                counts = {name: int(n) for name, n in (observation.get("counts") or {}).items() if int(n) > 0}
                run["counts"], run["blind_since"] = counts, None
            except Exception as exc:  # no fresh frame, detector down, no tray region: never counted as empty
                now = time.monotonic()
                run["blind_since"] = run["blind_since"] or now
                if now - run["blind_since"] >= self.blind_s:
                    message = f"카메라가 {int(self.blind_s)}초 넘게 쟁반을 세지 못했어요 ({exc}) · 눈으로 보고 [적재 완료]를 눌러 주세요"
                    if not run["log"] or run["log"][-1]["event"] != "CAMERA_BLIND":
                        self._note(run, "CAMERA_BLIND", message)
                    return self._answer(pinky_id, run, "attention", message)
                return self._answer(pinky_id, run, "", f"쟁반을 세는 중 ({exc})")
            total = sum(run["counts"].values())
            run["seen"] = run["seen"] + 1 if total >= self.target else 0
            if run["seen"] < self.confirm_checks:
                text = ", ".join(f"{name} {n}" for name, n in run["counts"].items()) or "없음"
                return self._answer(pinky_id, run, "", f"YOLO 쟁반 확인 · {text} ({total}/{self.target}개)")
            try:
                self.signal(pinky_id, "received", None)
            except Exception as exc:
                self._note(run, "RECEIVED_FAILED", f"출발 신호 실패: {exc}")
                return self._answer(pinky_id, run, "attention")
            run["stage"] = "sent"
            self._note(run, "RECEIVED", "YOLO가 다 실린 것을 확인 · 수령지로 출발 신호를 보냈어요",
                       {"counts": run["counts"]})
        if run["stage"] == "sent":
            return self._answer(pinky_id, run, "good", "다 실렸어요 · 수령지로 출발 신호를 보냈어요")
        return self._answer(pinky_id, run, "attention")

    def _answer(self, pinky_id: str, run: dict[str, Any], tone: str, message: str | None = None) -> dict[str, Any]:
        return {"pinky_id": pinky_id, "active": True, "phase": LOADING_PHASE, "stage": run["stage"], "tone": tone,
                "message": message or (run["log"][-1]["message"] if run["log"] else ""), "counts": run["counts"],
                "target": self.target, "log": list(run["log"])}

    def _note(self, run: dict[str, Any], event: str, message: str, detail: dict[str, Any] | None = None) -> None:
        run["log"].append({"at": time.time(), "event": event, "message": message, "detail": detail})
        del run["log"][:-self.history]
