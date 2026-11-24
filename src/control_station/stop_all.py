"""움직이는 모든 것(팔과 핑키)에 한 번의 요청으로 정지를 요청한다 (관제 화면의 큰 빨간 버튼).

호출 위치: POST /api/stop-all -> api/status.py의 post_stop_everything -> stop_everything(app).

비상 정지(E-stop)가 아니라 소프트웨어 정지 요청이다. 개별 정지와 같은 경로를 그대로 거치고, 그 의미도 그대로
이어받는다.
- 핑키 관제 대시보드의 정지: 로봇에 정지가 걸린 채로 유지(latch)되고 Nav2 목표가 취소된다.
- 팔 명령의 취소 플래그: 워커가 팔을 움직이는 동안 약 1초마다 확인한다.
- 팔 연속 실행(시퀀스)의 취소, 학습 정책 실행 프로세스의 중단(SIGINT).
모든 응답은 "요청이 접수됐다"는 뜻일 뿐, 장치가 실제로 멈췄다는 뜻이 아니다.

한 부분이 실패해도 나머지 부분에는 모두 요청을 보낸다. 실패한 부분은 `errors`에 남는다. 여기서는 아무것도
잠그지 않는다(latch 없음). 아직 진행 중인 주문은 다음 명령을 보낼 수 있으므로, 멈춘 주문의 처리는 운영자가
이어서 해야 한다.
"""

from __future__ import annotations

from typing import Any

# 취소할 수 있는 시퀀스 상태. ArmSequenceSupervisor.cancel이 받는 상태와 같다(CANCELING은 이미 취소 중이라 뺀다).
SEQUENCE_ACTIVE = frozenset({"QUEUED", "DISPATCHED", "RUNNING"})


def stop_everything(app: Any) -> dict[str, Any]:
    """팔과 핑키에 정지를 요청한다. `app`은 arm_sequences, arm_commands, policy_run_supervisor, pinky_gateway를
    가진 서버 객체이고, 없는 부분은 건너뛴다.

    반환값은 {"ok", "pinkies", "arm_sequence", "arm_commands", "policy_run", "errors"}다. ok는 핑키 요청이 모두
    접수되고 어느 부분도 예외를 내지 않았을 때만 True다(모의 모드에서는 핑키 요청을 보내지 않으므로 ok는 False다).
    """
    result: dict[str, Any] = {"ok": False, "pinkies": [], "arm_sequence": None, "arm_commands": [],
                              "policy_run": None, "errors": []}

    def attempt(part: str, call: Any) -> Any:
        # 한 부분의 예외가 나머지 정지 요청을 막지 않도록, 예외는 errors에 적고 None을 돌려준다.
        try:
            return call()
        except Exception as exc:  # noqa: BLE001 - one part failing must not keep the others from being asked
            result["errors"].append({"part": part, "message": str(exc) or type(exc).__name__})
            return None

    # 팔 먼저 처리한다. 팔 쪽은 이 프로세스 안의 일이라 몇 ms면 끝나므로, 뒤의 핑키 요청(네트워크)을 늦추지 않는다.
    sequences = getattr(app, "arm_sequences", None)
    if sequences is not None:
        def cancel_sequence() -> dict[str, Any] | None:
            latest = sequences.latest()
            if latest is None or latest.get("state") not in SEQUENCE_ACTIVE:
                return None
            # 팔 명령보다 시퀀스를 먼저 취소한다. 그래야 그 명령이 끝날 때 시퀀스가 다음 단계 명령을 큐에 넣지 않는다.
            return sequences.cancel(latest["sequence_run_id"])

        result["arm_sequence"] = attempt("arm_sequence", cancel_sequence)

    # 대기 중(QUEUED) 명령은 바로 취소되고, 실행 중 명령에는 워커가 확인하는 취소 플래그가 켜진다.
    commands = getattr(app, "arm_commands", None)
    if commands is not None:
        result["arm_commands"] = attempt("arm_commands", commands.cancel_active) or []

    # 관제에서 직접 띄운 정책 프로세스는 살아 있을 때만 중단 신호를 보낸다(없으면 None).
    policy = getattr(app, "policy_run_supervisor", None)
    if policy is not None:
        def stop_policy() -> dict[str, Any] | None:
            return policy.stop() if policy.snapshot().get("process_alive") else None

        result["policy_run"] = attempt("policy_run", stop_policy)

    gateway = getattr(app, "pinky_gateway", None)
    if gateway is not None:
        result["pinkies"] = attempt("pinkies", gateway.stop_all) or []

    # 핑키 결과가 비었거나(게이트웨이 예외) 한 대라도 접수되지 않았으면 ok가 아니다. 모의 모드의 게이트웨이는
    # 보내지 않고 ok False로 기록하므로 여기서도 False가 된다.
    result["ok"] = not result["errors"] and bool(result["pinkies"]) and all(entry["ok"] for entry in result["pinkies"])
    return result
