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

위치: app.py가 DockSupervisor(테스트 화면), DockAutoLoop(관제가 스스로 1초마다), 핑키마다 DockGuideRelay(0.1초마다)를
만든다. 주문 흐름(orders/flow.py)은 DockHandshake를 직접 만들어 같은 판단 규칙을 쓴다. DockAutoLoop와 DockGuideRelay는
각자 데몬 스레드에서 돌고, DockSupervisor는 HTTP 스레드와 자동 루프가 함께 부르므로 `_lock`으로 한 번에 하나씩 처리한다.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable

# ──── 1) 적재 자리 도착 판정 ──────────────────────────────────────────────────────────────────────────────────────

# 판단 결과(Decision.action): 기다림, 다시 맞추기, 도착 완료, 사람 확인.
WAIT, REDOCK, COMPLETE, OPERATOR = "WAIT", "REDOCK", "COMPLETE", "OPERATOR"
# 카메라는 되는데 마커가 거기 없다는 상태(NOT_FOUND는 주차 노드의 말, NOT_IN_VIEW는 가상 카메라의 말).
NOT_SEEN = frozenset({"NOT_FOUND", "NOT_IN_VIEW"})
REPEAT_MAX = 20  # 한 번 섰을 때 반복 테스트 진입 횟수 상한: 핑키 자신의 정차당 redock 한도(30)보다 넉넉히 작다
DOCKING_PHASE = "docking_manual"


def redock_offset(offset: dict[str, Any] | None) -> dict[str, float] | None:
    """보정 redock에 실을, 핑키 자신의 축 기준 카메라 어긋남: forward_mm, left_mm, dyaw_deg(+ = 왼쪽으로 돌아 있음).
    판정에 핑키 축 어긋남이 없으면(자리의 marker_forward_axis가 등록되지 않음) None."""
    try:
        values = {key: float((offset or {})[key]) for key in ("forward_mm", "left_mm", "dyaw_deg")}
    except (KeyError, TypeError, ValueError):
        return None
    return values if all(math.isfinite(value) for value in values.values()) else None


def _distance(observation: dict[str, Any] | None) -> float | None:
    """판정의 등록 자리와의 거리(distance_mm, mm). 없거나 유한한 숫자가 아니면 None."""
    try:
        value = float(((observation or {}).get("offset") or {})["distance_mm"])
    except (KeyError, TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


@dataclass
class Decision:
    """DockHandshake.step 한 번의 판단: 무엇을 할지(action), 왜(reason), 화면에 보일 말(message)."""

    action: str  # WAIT | REDOCK | COMPLETE | OPERATOR
    # NOT_DOCKING, SETTLING, ARRIVED, MISALIGNED, NOT_SEEN, REDOCK_PENDING, NOT_REGISTERED, CAMERA_BLIND, REDOCKS_USED
    reason: str
    message: str
    observation: dict[str, Any] | None = None  # 이 판단의 근거가 된 카메라 판정
    guide: dict[str, float] | None = None  # REDOCK일 때 함께 보낼 것(None: 핑키가 예전처럼 보정 없이 다시 들어옴)
    attempt: int = 0  # 지금까지 보낸 redock 수. REDOCK이면 이번 것도 포함
    # 한 번만, redock 뒤 첫 판정에서: {"attempt", "guided", "before_mm", "after_mm", "status"}
    result: dict[str, Any] | None = None
    not_applied: bool = False  # 한 번만: redock을 보냈는데 핑키가 자리를 떠나지 않았다

    def as_dict(self) -> dict[str, Any]:
        """API 응답과 화면용 dict(판정의 status와 offset을 함께 펼친다)."""
        return {"action": self.action, "reason": self.reason, "message": self.message, "attempt": self.attempt,
                "guide": self.guide, "result": self.result, "not_applied": self.not_applied,
                "status": (self.observation or {}).get("status"), "offset": (self.observation or {}).get("offset")}


class DockHandshake:
    """핑키 한 대가 적재 자리에 한 번 서는 동안의 판단 상태(연속 횟수, 보낸 redock 수, 진행 중인 redock).

    순수한 상태 기계다: 신호를 보내지 않고, 시계는 `clock`(기본 time.monotonic, 초)으로 주입받아 테스트할 수 있다.
    기본값: 정상 2번 연속이면 완료, 어긋남 3번 연속이면 redock, redock은 최대 5번, 판정 못 함 20초면 사람/재호출,
    redock을 보낸 뒤 15초 안에 핑키가 움직이지 않으면 "신호가 먹지 않음"으로 보고 다시 판정한다.
    잠금이 없으므로 한 스레드(또는 호출한 쪽의 잠금) 안에서만 쓴다.
    """

    def __init__(self, *, arrived_checks: int = 2, misaligned_checks: int = 3, max_redocks: int = 5,
                 blind_s: float = 20.0, redock_wait_s: float = 15.0,
                 clock: Callable[[], float] = time.monotonic) -> None:
        self.arrived_checks, self.misaligned_checks, self.max_redocks = arrived_checks, misaligned_checks, max_redocks
        self.blind_s, self.redock_wait_s, self.clock = blind_s, redock_wait_s, clock
        self.redocks = 0
        self._arrived = self._misaligned = 0  # ARRIVED / MISALIGNED 연속 횟수
        self._blind_since: float | None = None  # 판정을 못 받기 시작한 clock 시각
        # 날아가는 중인 redock: {"attempt", "guided", "before_mm", "sent_at", "left"}. 보낸 뒤 핑키가
        # docking_manual 밖(redocking)에서 보이면 "left"가 된다.
        self._pending: dict[str, Any] | None = None

    def redock_sent(self, guided: bool, before_mm: float | None = None) -> None:
        """이 핸드셰이크가 요청한 redock(REDOCK)을 호출한 쪽이 보냈고 핑키 관제가 받았다. 연속 횟수를 새로 센다."""
        self._pending = {"attempt": self.redocks, "guided": bool(guided), "before_mm": before_mm,
                         "sent_at": self.clock(), "left": False}
        self._arrived = self._misaligned = 0
        self._blind_since = None

    def step(self, phase: str | None, observation: dict[str, Any] | None) -> Decision:
        """판정 하나를 받아 다음 판단을 돌려준다(모듈 설명의 표 참고).

        `observation`: 이 핑키에 대한 카메라 판정({"status", "offset", ...}). 핑키가 docking_manual에 서 있을 때만
        묻는다(그 밖에는 None, 또는 묻다가 status 없이 실패했을 때 None).
        """
        if phase != DOCKING_PHASE:
            # 자리를 떠났다: 진행 중인 redock은 "핑키가 움직였다"로 표시하고, 연속 횟수는 처음부터.
            if self._pending is not None:
                self._pending["left"] = True
            self._arrived = self._misaligned = 0
            self._blind_since = None
            return Decision(WAIT, "NOT_DOCKING", "핑키가 적재 자리에 서기를 기다려요", attempt=self.redocks)
        result, not_applied = None, False
        if self._pending is not None and not self._pending["left"]:
            # redock을 보냈는데 아직 떠나지 않았다: 움직이기 전 화면을 판정해 redock을 또 보내지 않게 기다린다.
            if self.clock() - self._pending["sent_at"] < self.redock_wait_s:
                return Decision(WAIT, "REDOCK_PENDING", "다시 맞추기 신호를 보냈어요 · 핑키가 물러났다 들어오기를 기다려요",
                                attempt=self.redocks)
            self._pending, not_applied = None, True  # 끝내 움직이지 않았다: 지금 서 있는 것을 판정한다
        observation = observation or {"status": "PARKING_CHECK_FAILED", "reason": "카메라 판정을 받지 못했어요"}
        status = observation.get("status")
        distance = _distance(observation)
        # 다시 선 뒤 첫 확정 판정: redock 결과(전 -> 후 거리)를 한 번 기록한다.
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
            self.redocks += 1  # 보내지 못하면 호출한 쪽이 되돌린다(DockSupervisor.step)
            self._misaligned = 0
            made["attempt"] = self.redocks
            return Decision(REDOCK, "MISALIGNED", f"위치 어긋남 · 다시 맞춰 들어가기 {self.redocks}/{self.max_redocks}",
                            guide=redock_offset(observation.get("offset")), **made)
        # 판정 못 함: 마커가 가려짐, 노드가 꺼짐, 화면이 움직임, 아직 움직이는 중.
        self._arrived = self._misaligned = 0
        now = self.clock()
        if self._blind_since is None:
            self._blind_since = now
        if now - self._blind_since >= self.blind_s and status in NOT_SEEN and self.redocks < self.max_redocks:
            # 카메라는 자리를 보고 있는데 핑키가 거기 없다: 사람을 기다리지 않고 다시 불러들인다(보정 없이).
            self.redocks += 1
            self._blind_since = None
            made["attempt"] = self.redocks
            return Decision(REDOCK, "NOT_SEEN", f"카메라에 핑키가 안 보여요 · 다시 들어오라고 해요 {self.redocks}/{self.max_redocks}",
                            guide=None, **made)
        if now - self._blind_since >= self.blind_s:
            # 노드가 꺼졌거나 화면이 움직였다(또는 NOT_SEEN으로 redock을 다 썼다): 다시 불러도 소용없으니 사람에게.
            return Decision(OPERATOR, "CAMERA_BLIND",
                            f"카메라가 {int(self.blind_s)}초 넘게 판정하지 못했어요 ({status}) · 눈으로 보고 확인해 주세요", **made)
        return Decision(WAIT, "CAMERA_BLIND", f"카메라 판정을 기다려요 ({status})", **made)


class DockSupervisor:
    """주문 없이 몬 핑키(테스트 화면)를 위한 같은 핸드셰이크. 호출 한 번에 한 단계.

    status(pinky_id) -> {"phase", "mission_token", "route_status"}; judge(pinky_id) -> 그 핑키 자신의 등록 자리에서
    본 카메라 판정; signal(pinky_id, action, offset)은 dock_complete / redock을 보내고 거절되면 예외를 올린다;
    in_order(pinky_id)는 주문 흐름이 그 핑키와 직접 주고받는 동안 True다(그때 이쪽은 지켜보기만 한다).

    진입 반복 테스트(step(..., repeat=N), 2026-11-19): 카메라가 핑키를 자리에서 확인할 때마다 그 진입을 성공 한 번으로
    적고, N번이 모일 때까지 dock_complete 대신 핑키를 내보냈다가 다시 들이게 한다(보정 없는 redock). 자리를 벗어나
    들어온 진입도 적는다(그리고 늘 그렇듯 핸드셰이크가 고친다). `record(trial)`은 모든 시도를 남긴다(app의 JSON-lines 파일).

    HTTP 스레드(테스트 화면)와 DockAutoLoop 스레드가 함께 부르므로 step 전체를 `_lock` 아래에서 한다.
    """

    def __init__(self, status: Callable[[str], dict[str, Any]], judge: Callable[[str], dict[str, Any]],
                 signal: Callable[[str, str, dict[str, float] | None], dict[str, Any]],
                 in_order: Callable[[str], bool] = lambda pinky_id: False, history: int = 20,
                 record: Callable[[dict[str, Any]], None] | None = None, **limits: Any) -> None:
        self.status, self.judge, self.signal, self.in_order = status, judge, signal, in_order
        self.record = record
        self.limits, self.history = limits, history  # limits: DockHandshake 인자, history: 정차당 남길 기록 수
        self._lock = threading.Lock()
        # 핑키 id -> 이번 정차의 상태 {"handshake", "log", "done", "trials", "judged", "repeat"}
        self._runs: dict[str, dict[str, Any]] = {}

    def _trial(self, run: dict[str, Any], pinky_id: str, outcome: str, decision: Decision) -> None:
        """반복 테스트의 진입 한 번을, 카메라가 처음 판정한 대로 적는다."""
        offset = (decision.observation or {}).get("offset") or {}
        trial = {"at": time.time(), "pinky_id": pinky_id, "n": len(run["trials"]) + 1, "outcome": outcome,
                 **{key: offset.get(key) for key in ("forward_mm", "left_mm", "dyaw_deg", "distance_mm")}}
        run["trials"].append(trial)
        if self.record is not None:
            try:
                self.record(trial)
            except Exception:  # 기록은 근거일 뿐 핑키를 움직이는 조건이 아니다
                pass

    def step(self, pinky_id: str, auto: bool, repeat: int | None = 0) -> dict[str, Any]:
        """한 번 판정하고, `auto`면 그대로 행동한다. 판단, 보낸 것, 이번 정차의 기록을 돌려준다.
        repeat: dock_complete 전에 이번 정차에서 모을 확인된 진입 수(0: 첫 확인으로 끝). None이면 이번 정차의 값을
        그대로 둔다(서버 자체 루프용. 테스트 화면이 시킨 반복 테스트를 끝내 버리면 안 되므로)."""
        with self._lock:
            status = self.status(pinky_id)
            phase = status.get("phase")
            # 적재 자리에 서 있지도, 다시 들어오는 중이지도 않으면 이번 정차는 끝났다: 상태를 버린다.
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
                except Exception as exc:  # 노드 꺼짐, 화면 움직임, 프레임 없음
                    observation = {"status": getattr(exc, "error_code", "PARKING_CHECK_FAILED"), "reason": str(exc)}
            if run["done"]:
                return {"pinky_id": pinky_id, "active": True, "phase": phase, "auto": bool(auto), "sent": None,
                        "done": True, "log": list(run["log"]), "repeat": self._repeat(run),
                        "decision": {"action": WAIT, "reason": "COMPLETE_SENT", "message": "도킹완료를 보냈어요"}}
            decision = handshake.step(phase, observation)
            sent = None
            if phase != DOCKING_PHASE:
                run["judged"] = False  # 다시 들어오는 중: 다음 판정은 새 진입이다
            elif run["repeat"] and auto and not run["judged"] and decision.action in {REDOCK, COMPLETE}:
                run["judged"] = True
                self._trial(run, pinky_id, "SUCCESS" if decision.action == COMPLETE else decision.reason, decision)
            successes = sum(1 for trial in run["trials"] if trial["outcome"] == "SUCCESS")
            if decision.action == COMPLETE and auto and run["repeat"] and successes < run["repeat"]:
                # 하나 모았지만 아직 끝이 아니다: 나갔다 다시 들어오게 한다. 핸드셰이크 자신의 보정 redock이 아니므로
                # redock 횟수는 0으로 되돌린다.
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
                        handshake.redocks -= 1  # 보내지 못함: redock 한 번을 쓴 것으로 치지 않는다
                        self._note(run, "REDOCK_FAILED", f"다시 맞추기 요청 실패: {exc}", {"attempt": decision.attempt})
                else:
                    # 보내지 않음: 운영자가 버튼을 누른다. 횟수는 실제로 보낸 것을 따르고, 여전히 어긋난 다음 판정은
                    # 하나부터 다시 세지 않고 곧바로 REDOCK을 말한다(연속 횟수를 기준값에 맞춰 둔다).
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
            # 같은 OPERATOR 사유를 판정마다 쌓지 않는다(바로 앞 기록과 같으면 건너뜀).
            elif decision.action == OPERATOR and (not run["log"] or run["log"][-1]["event"] != decision.reason):
                self._note(run, decision.reason, decision.message, None)
            return {"pinky_id": pinky_id, "active": True, "phase": phase, "auto": bool(auto), "sent": sent,
                    "decision": decision.as_dict(), "log": list(run["log"]), "repeat": self._repeat(run)}

    @staticmethod
    def _repeat(run: dict[str, Any]) -> dict[str, Any] | None:
        """반복 테스트 요약(목표, 시도, 성공, 최근 5번). 반복 테스트가 아니면 None."""
        if not run.get("repeat") and not run.get("trials"):
            return None
        trials = run["trials"]
        return {"target": run.get("repeat", 0), "trials": len(trials),
                "successes": sum(1 for trial in trials if trial["outcome"] == "SUCCESS"), "last": trials[-5:]}

    def _note(self, run: dict[str, Any], event: str, message: str, detail: dict[str, Any] | None) -> None:
        """이번 정차의 기록에 한 줄 더하고, 최근 `history`줄만 남긴다."""
        run["log"].append({"at": time.time(), "event": event, "message": message, "detail": detail})
        del run["log"][:-self.history]


class DockAutoLoop:
    """관제가 스스로 돌리는 핸드셰이크(2026-11-24): 적재 자리에 선 핑키를 카메라가 판정하고 결과(dock_complete,
    또는 자리를 벗어나 있으면 redock)를 알려 준다. 브라우저에 테스트 화면을 열어 두지 않아도, 누가 도킹완료를 누르지
    않아도 된다. 전에는 테스트 화면의 주기 조회에서만, 그것도 체크 상자를 켰을 때만 단계가 돌았다.

    step(pinky_id)는 auto=True, repeat=None인 DockSupervisor.step이다. phase(pinky_id)는 핑키의 임무 단계로, 먼저
    읽어서 다른 곳에 있는 핑키는 캐시된 상태 한 번만 쓰고 카메라 확인은 하지 않는다. enabled()는 스위치다(운영자의
    체크 상자, CONTROL_STATION_DOCK_AUTO=0이면 꺼진 채 시작). 주문의 흐름은 여전히 자기 핸드셰이크를 한다
    (supervisor는 주문 중인 핑키를 건드리지 않는다).
    """

    PHASES = frozenset({DOCKING_PHASE, "redocking"})  # 이 단계일 때만 step을 부른다

    def __init__(self, pinky_ids: Any, phase: Callable[[str], str | None], step: Callable[[str], dict[str, Any]],
                 enabled: Callable[[], bool] = lambda: True, period_s: float = 1.0) -> None:
        self.pinky_ids, self.phase, self.step, self.enabled, self.period_s = tuple(pinky_ids), phase, step, enabled, period_s
        self.last: dict[str, dict[str, Any]] = {}  # 핑키 id -> step의 마지막 답, 또는 {"error"}
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def tick(self) -> list[str]:
        """한 바퀴. step을 부른 핑키들을 돌려준다."""
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
            except Exception as exc:  # 핑키 관제가 응답하지 않음: 다음 바퀴에 다시 해 본다
                self.last[pinky_id] = {"error": str(exc)}
        return stepped

    def start(self) -> "DockAutoLoop":
        """period_s(초)마다 tick을 부르는 데몬 스레드("dock-auto")를 띄운다. 서버 종료를 막지 않는다."""
        def loop() -> None:
            while not self._stop.wait(self.period_s):
                self.tick()
        self._thread = threading.Thread(target=loop, name="dock-auto", daemon=True)
        self._thread.start()
        return self

    def stop(self) -> None:
        """다음 대기에서 루프를 끝낸다(기다리지 않음)."""
        self._stop.set()


def _result_text(result: dict[str, Any]) -> str:
    """redock 결과 기록을 사람이 읽는 한 줄로(주문 흐름의 PINKY_REDOCK_RESULT도 이 문장을 쓴다)."""
    before, after = result.get("before_mm"), result.get("after_mm")
    numbers = f"{before:.0f} mm → {after:.0f} mm" if before is not None and after is not None else "오차 값 없음"
    how = "카메라 보정" if result.get("guided") else "보정 없이"
    verdict = "정상 위치" if result.get("status") == "ARRIVED" else "아직 어긋남"
    return f"다시 맞추기 {result.get('attempt')}번째({how}) 결과: {numbers} · {verdict}"


# ──── 2) 들어오는 동안 방향 안내 ────────────────────────────────────────────────────────────────────────────────────

# 주차 노드의 실시간 상태 가운데 마커를 따라가고 있는(어긋남 값이 믿을 만한) 것들.
TRACKED_STATES = frozenset({"MOVING", "IN_POSITION", "OFF_POSITION"})
# 이 단계의 핑키에게는 안내를 보내지 않는다(주차장에 서 있음).
PARKED_PHASES = frozenset({"parked"})


def guide_from(live: dict[str, Any] | None) -> dict[str, float] | None:
    """마커를 따라가는 동안의 실시간 상태에서 꺼낸 forward_mm, left_mm(mm), dyaw_deg(도). 아니면 None."""
    if not isinstance(live, dict) or live.get("state") not in TRACKED_STATES:
        return None
    offset = live.get("offset") or {}
    try:
        guide = {key: float(offset[key]) for key in ("forward_mm", "left_mm", "dyaw_deg")}
    except (KeyError, TypeError, ValueError):
        return None
    return guide if all(math.isfinite(value) for value in guide.values()) else None


class DockGuideRelay:
    """핑키 한 대의 진입 안내 전달기. snapshot() -> 그 핑키 자리의 실시간 상태; phase(pinky_id) -> 핑키의 임무 단계;
    send(pinky_id, guide) -> 핑키 관제가 받으면 True. 프레임 하나는 한 번만 보내고, 거절되거나 오류가 나면
    `retry_s`(초) 동안 조용히 있는다. 데몬 스레드에서 period_s(기본 0.1초)마다 tick을 돈다.

    핑키 쪽은 0.4초 안의 값만 쓰므로, 보내지 못하면 핑키는 예전처럼 바퀴 거리로 들어온다(안내는 있으면 좋은 것).
    """

    def __init__(self, snapshot: Callable[[], dict[str, Any]], pinky_id: str,
                 phase: Callable[[str], str | None], send: Callable[[str, dict[str, float]], bool], *,
                 period_s: float = 0.1, retry_s: float = 3.0, clock: Callable[[], float] = time.monotonic) -> None:
        self.snapshot, self.pinky_id, self.phase, self.send = snapshot, pinky_id, phase, send
        self.period_s, self.retry_s, self.clock = period_s, retry_s, clock
        self.sent = self.failed = 0
        self.last: dict[str, Any] | None = None  # 마지막으로 보낸 안내와 그 시각
        self._frame: Any = None  # 마지막으로 보낸 프레임 번호
        self._quiet_until = 0.0  # 이 clock 시각까지 보내지 않는다
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def tick(self) -> str:
        """한 바퀴. 무슨 일이 있었는지 돌려준다(상태 줄과 테스트용):
        QUIET(쉬는 중), NOT_TRACKED(마커를 못 따라감), SAME_FRAME, PARKED, SENT, REFUSED."""
        now = self.clock()
        if now < self._quiet_until:
            return "QUIET"
        live = self.snapshot()
        guide = guide_from(live)
        if guide is None:
            return "NOT_TRACKED"
        frame = live.get("frame_sequence")
        if frame is not None and frame == self._frame:
            return "SAME_FRAME"  # 멈춘 카메라의 값이 핑키에게 새 값처럼 보이면 안 된다
        try:
            if self.phase(self.pinky_id) in PARKED_PHASES:
                return "PARKED"
            self._frame = frame
            if self.send(self.pinky_id, guide):
                self.sent += 1
                self.last = {**guide, "at": time.time()}
                return "SENT"
        except Exception:  # 핑키 관제가 응답하지 않음: 나중에 다시 해 본다. 핑키는 안내 없이 들어온다
            pass
        # 거절(False)이든 오류든 retry_s 동안 쉰다. 보내려 한 프레임 번호는 _frame에 남아 있어 그 프레임을 다시
        # 보내지 않는다(phase 조회에서 예외가 난 경우는 아직 기록 전이다).
        self.failed += 1
        self._quiet_until = now + self.retry_s
        return "REFUSED"

    def start(self) -> None:
        """period_s(초)마다 tick을 부르는 데몬 스레드("dock-guide-relay")를 띄운다."""
        def loop() -> None:
            while not self._stop.wait(self.period_s):
                self.tick()
        self._thread = threading.Thread(target=loop, name="dock-guide-relay", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """다음 대기에서 루프를 끝낸다."""
        self._stop.set()

    def status(self) -> dict[str, Any]:
        """화면·health용 요약: 보낸 수, 실패 수, 마지막 안내."""
        return {"pinky_id": self.pinky_id, "sent": self.sent, "failed": self.failed, "last": self.last}
