"""OMX 팔 명령 큐, 선반 칸, 학습 정책 API (계약 전체는 docs/implementation/arm-command-api.md).

GET  /api/arm/status                          팔 프로세스 상태(텔레오퍼레이션은 제거되어 항상 비활성)와 정책 실행 상태
GET  /api/arm/commands                        최근 명령 20건, 워커 접속 상태, ACK 제한 시간
GET  /api/arm/commands/{command_id}           명령 하나와 이벤트 이력
GET  /api/arm/shelf-slots                     칸 정의 파일 내용과 버전(SHA-256)
GET  /api/arm/policies                        물체별 학습 정책 목록과 정책 파일 버전
GET  /api/arm/sequences, /sequences/{run_id}  정책 연속 실행 정의와 최근 실행
GET  /api/arm/{robot_id}/commands/next        워커의 long-poll (200 명령 / 204 없음)
POST /api/arm/commands                        명령 등록
POST /api/arm/commands/{id}/events|cancel|clear-unknown   워커 이벤트, 취소, UNKNOWN 현장 확인
POST /api/arm/commands/from-camera            정면 카메라가 물체를 본 칸으로 명령 등록
POST /api/arm/sequences, /sequences/{run_id}/cancel        연속 실행 시작·취소
POST /api/arm/policy/start|stop               관제에서 LeRobot 정책 프로세스를 직접 1회 실행·중단

관제는 팔을 직접 움직이지 않는다. 명령을 arms/commands.py의 ArmCommandStore에 기록하면, 팔 쪽 워커가 long-poll로
가져가 실행하고 이벤트로 결과를 알린다.

카메라 공유: V4L2 카메라는 한 프로그램만 열 수 있다. 정책이 쓰는 카메라를 관제 화면도 띄우고 있으면, 정책 실행
동안 관제가 그 카메라를 닫아 두고(빌려주고) 끝나면 다시 연다(sync_policy_cameras, post_policy_run).
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any
from urllib.parse import parse_qs, urlsplit

from control_station.arms.commands import ARM_ROBOT_IDS, ArmCommandConflict, ArmCommandError, ArmCommandNotFound
from control_station.arms.sequences import ArmSequenceError
from control_station.cameras.shelf_slots import SlotSelectionError, select_slot_for_class

if TYPE_CHECKING:
    from control_station.app import ControlStationHandler


def post_policy_run(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """학습 정책 실행 시작·중단 (?profile_id=정책 ID, 하나만). 성공하면 202, 거부되면 409와 현재 상태."""
    response_key = "policy_run"
    try:
        # 시작·중단 요청이 동시에 들어와도 카메라 넘기기와 프로세스 시작이 섞이지 않도록 한 번에 하나씩 처리한다.
        with h.app.arm_control_lock:
            if route == "/api/arm/policy/start":
                profile_ids = parse_qs(urlsplit(h.path).query).get("profile_id", [])
                if len(profile_ids) > 1:
                    raise RuntimeError("profile_id는 하나만 지정할 수 있습니다.")
                profile_id = profile_ids[0] if profile_ids else None
                # 시작하기 전에, 이 실행이 열 카메라(그 팔의 그리퍼·탑 캠 한 쌍)를 관제 화면 쪽에서 닫는다.
                devices = h.app.policy_run_supervisor.devices_for(profile_id)
                manager = h.app.camera_manager
                hold = getattr(manager, "hold_for_policy", None)
                # hold_for_policy가 없는 예전 관리자는 정면 카메라만 닫을 수 있다. 같은 (devices, on) 모양으로 맞춘다.
                if hold is None:
                    hold_front = getattr(manager, "hold_front_for_policy", None)
                    hold = (lambda _devices, on: hold_front(on)) if hold_front is not None else None
                # 정해진 시간 안에 닫혔는지 확인하지 못하면 시작하지 않는다. 두 프로그램이 같은 장치를 열려고 다투면
                # 정책 쪽 카메라 열기가 실패하기 때문이다. 이때는 붙잡아 둔 것을 풀고 409로 답한다.
                if hold is not None and not hold(devices, True):
                    hold(devices, False)
                    raise RuntimeError(
                        "관제 화면이 정책 카메라를 놓았는지 확인하지 못해 정책을 시작하지 않았습니다. 잠시 뒤 다시 시도하세요."
                    )
                try:
                    state = h.app.policy_run_supervisor.start(profile_id)
                finally:
                    # 여기서 붙잡은 것은 바로 푼다. 정책 프로세스가 살아 있는 동안에는 카메라 관리자가 policy_devices /
                    # policy_active 콜백으로 그 카메라를 계속 닫아 둔다.
                    if hold is not None:
                        hold(devices, False)
            else:
                state = h.app.policy_run_supervisor.stop()
    except RuntimeError as exc:
        h._json(409, {"error": str(exc), response_key: h.app.policy_run_supervisor.snapshot()})
        return
    h._json(202, {response_key: state})


# 워커가 정책을 실행 중이거나 곧 실행할 명령 상태. 이 동안 그 팔의 정책 카메라를 관제가 닫아 둔다.
# QUEUED는 넣지 않는다: 워커가 가져가기(DISPATCHED) 전까지는 화면에 카메라를 계속 띄운다.
POLICY_IN_FLIGHT = frozenset({"DISPATCHED", "ACCEPTED", "RUNNING"})


def _policy_devices_for(app: Any, robot_id: str) -> list[str]:
    """이 팔의 워커가 여는 카메라 장치 경로. 설정은 {arm_id: [...]}(예전 단일 팔 목록은 키 "*") 또는 목록 하나다."""
    devices = app.policy_camera_devices
    if isinstance(devices, dict):
        return list(devices.get(robot_id) or devices.get("*") or [])
    return list(devices or [])


def sync_policy_cameras(app: Any) -> None:
    """정책 집기 명령이 워커에 넘어가 끝나기 전까지, 그 팔의 정책이 쓰는 카메라를 관제가 닫아 둔다.

    V4L2 카메라는 한 프로그램만 열 수 있다. 팔마다 그리퍼 캠·탑 캠(CONTROL_STATION_ARM01_*_CAMERA, ARM02_*; 예전 단일 팔
    설정은 CONTROL_STATION_ACT_OVERHEAD_CAMERA, CONTROL_STATION_JOINT_CAMERA)을 관제도 화면에 띄우는 경우, 명령이
    DISPATCHED·ACCEPTED·RUNNING인 동안만 그 팔의 카메라를 닫고 끝나면 다시 연다. 워커는 열기 전에 장치가 비었는지 확인한다.

    호출 시점: 명령 목록 조회(화면이 1초마다 부름), 워커가 정책 명령을 가져갈 때, 워커 이벤트가 들어올 때.
    매번 최근 명령으로 "지금 닫아 둬야 할 장치"를 다시 계산하고, 지난번과 같으면 아무것도 하지 않는다."""
    manager = getattr(app, "camera_manager", None)
    if not getattr(app, "policy_camera_devices", None) or manager is None or not hasattr(manager, "lend_devices"):
        return
    commands = app.arm_commands.list_recent()["commands"]
    # 정책 명령이 진행 중인 팔의 카메라를 중복 없이 순서대로 모은다(dict.fromkeys로 순서 유지 중복 제거).
    wanted = tuple(dict.fromkeys(
        device
        for command in commands
        if command["operation"] == "PICK_WITH_POLICY" and command["state"] in POLICY_IN_FLIGHT
        for device in _policy_devices_for(app, command["robot_id"])
    ))
    # 여러 요청 스레드가 동시에 부를 수 있으므로, 빌려준 상태(policy_cameras_lent*)는 이 잠금 안에서만 바꾼다.
    with app.policy_camera_lock:
        lent = getattr(app, "policy_cameras_lent_devices", None)
        lent = lent if isinstance(lent, tuple) else ()
        if app.policy_cameras_lent and lent == wanted:
            return
        # 빌려줄 장치가 바뀌었으면 일단 모두 돌려받은 뒤, 지금 필요한 장치만 다시 빌려준다.
        if app.policy_cameras_lent:
            print(f"정책 종료: 관제가 카메라를 다시 엶 {manager.take_back_devices()}", flush=True)
            app.policy_cameras_lent = False
        if wanted:
            closed = manager.lend_devices(list(wanted), "학습 정책 실행 중: 이 카메라는 정책이 쓰고 있습니다")
            print(f"정책 실행: 관제가 카메라를 닫음 {closed}", flush=True)
            app.policy_cameras_lent = True
        app.policy_cameras_lent_devices = wanted



def get_status(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """팔 프로세스 상태와 정책 실행 상태. 관제의 텔레오퍼레이션 기능은 제거되어 비활성으로 응답한다."""
    h._json(200, {
        "arm": {"state": "DISABLED", "enabled": False, "process_alive": False,
                "detail": "관제 텔레오퍼레이션 기능이 비활성화되었습니다."},
        "policy_run": h.app.policy_run_supervisor.snapshot(),
    })


def get_commands(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """OMX 명령 목록"""
    sync_policy_cameras(h.app)
    h._json(200, h.app.arm_commands.list_recent())


def get_command(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """OMX 명령 하나 조회"""
    command_id = route.removeprefix("/api/arm/commands/")
    try:
        h._json(200, {"command": h.app.arm_commands.get(command_id)})
    except ArmCommandNotFound:
        h._json_error(404, "ARM_COMMAND_NOT_FOUND", "Arm command not found")


def get_policies(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """물체별 학습 정책 목록 (PICK_WITH_POLICY)"""
    policies = h.app.arm_commands.act_policies
    if policies is None:
        h._json_error(404, "ACT_POLICIES_NOT_CONFIGURED", getattr(h.app, "act_policies_error", None) or "정책 정의 파일이 없습니다")
    else:
        h._json(200, {"registry_version": policies.version, "policies": policies.summary()})


def get_sequences(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """연속 실행 정의(config/omx-act-sequences.json)와 가장 최근 실행. 정의 파일이 잘못됐으면 409."""
    try:
        h._json(200, {"sequences": h.app.arm_sequences.definitions(), "latest": h.app.arm_sequences.latest()})
    except ArmSequenceError as exc:
        h._json_error(409, "ARM_SEQUENCE_CONFIG_INVALID", str(exc))


def get_sequence(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """연속 실행 하나 (/api/arm/sequences/{run_id})"""
    run_id = parts[3]
    try:
        h._json(200, {"sequence": h.app.arm_sequences.get(run_id)})
    except KeyError:
        h._json_error(404, "ARM_SEQUENCE_NOT_FOUND", "Arm sequence not found")


def post_sequence(h: ControlStationHandler, route: str, parts: list[str], body: dict[str, Any]) -> None:
    """연속 실행 시작·취소.

    시작 본문은 {sequence_id}(정의 파일의 시퀀스) 또는 {object_classes}(운영자가 고른 물체 순서)다.
    취소(/api/arm/sequences/{run_id}/cancel)는 빈 객체를 받는다. 첫 명령을 만들 때 팔 명령 저장소가 거부하면
    그 오류 코드로 409를 돌려준다."""
    try:
        if route == "/api/arm/sequences":
            if set(body) == {"sequence_id"} and isinstance(body.get("sequence_id"), str):
                sequence = h.app.arm_sequences.start(body["sequence_id"])
            elif set(body) == {"object_classes"}:
                sequence = h.app.arm_sequences.start_custom(body["object_classes"])
            else:
                raise ArmSequenceError("body에는 sequence_id 또는 object_classes가 필요합니다")
            h._json(201, {"sequence": sequence})
        else:
            if body:
                raise ArmSequenceError("cancel body는 빈 객체여야 합니다")
            h._json(200, {"sequence": h.app.arm_sequences.cancel(parts[3])})
    except KeyError:
        h._json_error(404, "ARM_SEQUENCE_NOT_FOUND", "Arm sequence not found")
    except ArmSequenceError as exc:
        h._json_error(409, "ARM_SEQUENCE_CONFLICT", str(exc))
    except (ArmCommandError, ArmCommandConflict) as exc:
        details = {"command": exc.command} if isinstance(exc, ArmCommandConflict) and exc.command else {}
        h._json_error(409, getattr(exc, "error_code", "ARM_COMMAND_CONFLICT"), str(exc), **details)


def get_shelf_slots(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """선반 칸 정의"""
    slot_map = h.app.arm_commands.slot_map
    if slot_map is None:
        h._json_error(
            404,
            "SHELF_SLOTS_NOT_CONFIGURED",
            h.app.arm_commands.slot_map_error or "칸 정의 파일이 없습니다",
        )
    else:
        h._json(200, {"slot_map_version": slot_map.version, "slot_map": slot_map.document})


def get_next_command(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """워커가 다음 명령 받기 (기다리다 없으면 204).

    ?worker_id=...&wait_s=20. 명령이 올 때까지 최대 wait_s초(0–30) 이 요청을 붙잡아 둔다(long-poll). 이 요청 자체가
    워커의 접속 신호로 기록된다."""
    robot_id = route.split("/")[3]
    query = parse_qs(urlsplit(h.path).query)
    try:
        wait_s = float(query.get("wait_s", ["20"])[0])
        command = h.app.arm_commands.claim_next(robot_id, query.get("worker_id", [None])[0], wait_s)
    except ArmCommandNotFound:
        h._json_error(404, "ARM_NOT_FOUND", "Unknown arm robot_id")
        return
    except (ArmCommandError, ValueError) as exc:
        h._json_error(400, "ARM_COMMAND_INVALID", str(exc))
        return
    if command is None:
        h.send_response(204)
        h.send_header("Cache-Control", "no-store")
        h.end_headers()
        return
    if command["operation"] == "PICK_WITH_POLICY":
        sync_policy_cameras(h.app)  # 워커가 열기 전에 정책 카메라를 관제 쪽에서 닫는다
    h._json(200, {"command": command})


def post_command(h: ControlStationHandler, route: str, parts: list[str], body: dict[str, Any]) -> None:
    """OMX 명령 등록·진행 이벤트·취소·불명 해제.

    parts 길이 3이면 /api/arm/commands(등록), 5이면 /api/arm/commands/{id}/{events|cancel|clear-unknown}이다.
    계약 위반은 400, 지금 상태에서 허용되지 않는 요청은 409(현재 command를 함께)다."""
    store = h.app.arm_commands
    try:
        if len(parts) == 3:
            h._json(201, {"command": store.create(body)})
        elif parts[4] == "events":
            command, applied = store.apply_worker_event(parts[3], body)
            # 같은 event_id의 재전송(applied False)도 시퀀스에 알린다. 앞선 응답을 잃어 다음 단계로 못 넘어간 경우를
            # 여기서 복구하기 위해서다(sequences.py의 on_worker_event).
            h.app.arm_sequences.on_worker_event(command, applied)
            if command["operation"] == "PICK_WITH_POLICY":
                sync_policy_cameras(h.app)  # 명령이 끝났으면 카메라를 다시 연다
            h._json(200, {"command": command, "applied": applied})
        elif parts[4] == "cancel":
            if body:
                raise ArmCommandError("cancel takes an empty JSON object")
            h._json(200, {"command": store.cancel(parts[3])})
        else:
            if set(body) != {"note"}:
                raise ArmCommandError("clear-unknown takes exactly note")
            h._json(200, {"command": store.clear_unknown(parts[3], body["note"])})
    except ArmCommandNotFound:
        h._json_error(404, "ARM_COMMAND_NOT_FOUND", "Arm command not found")
    except ArmCommandError as exc:
        h._json_error(400, "ARM_COMMAND_INVALID", str(exc))
    except ArmCommandConflict as exc:
        h._json_error(409, exc.error_code, str(exc), command=exc.command)


def post_command_from_camera(h: ControlStationHandler, route: str, parts: list[str], body: dict[str, Any]) -> None:
    """정면 카메라가 물건을 찾은 칸으로 OMX 명령 등록 (한 칸에만 보일 때만)

    operation MOVE_TO_SLOT(기본)은 그 칸 자세로 이동, PICK_WITH_POLICY는 그 물체용 학습 정책 실행.
    정책은 위치를 입력으로 받지 않으므로, 정책 명령에서 칸은 물체가 실제로 있다는 근거로만 남는다.
    robot_id를 주지 않으면 물체가 보인 칸을 맡은 팔이 받고, 주면 그 팔이 칸 담당이어야 한다(ARM_SHELF_MISMATCH).
    정책은 그 팔에서 녹화한 것(또는 팔이 정해지지 않은 것)만 고른다(POLICY_NOT_FOUND)."""
    allowed = {"object_class", "camera_id", "valid_for_s", "operation", "robot_id"}
    object_class = body.get("object_class")
    operation = body.get("operation", "MOVE_TO_SLOT")
    requested_robot_id = body.get("robot_id")
    if (
        set(body) - allowed
        or not isinstance(object_class, str)
        or not object_class.strip()
        or operation not in {"MOVE_TO_SLOT", "PICK_WITH_POLICY"}
        or (requested_robot_id is not None and (
            not isinstance(requested_robot_id, str) or requested_robot_id not in ARM_ROBOT_IDS
        ))
    ):
        h._json_error(
            400,
            "ARM_COMMAND_INVALID",
            f"body needs object_class and may have robot_id ({', '.join(sorted(ARM_ROBOT_IDS))}), camera_id, "
            "valid_for_s, operation (MOVE_TO_SLOT or PICK_WITH_POLICY)",
        )
        return
    class_name = object_class.strip()
    policies = h.app.arm_commands.act_policies

    def no_policy(arm_id: str | None) -> None:
        """409 POLICY_NOT_FOUND와 함께, 그 팔(None이면 모든 팔)에서 쓸 수 있는 정책 목록을 알려 준다."""
        h._json_error(409, "POLICY_NOT_FOUND", f"{arm_id or '어느 팔'}에도 {class_name}에 학습된 정책이 없습니다",
                      details={"policies": [f"{item['label']}({item['object_class']})" for item in policies.policies.values()
                                            if arm_id is None or item["arm_id"] in (None, arm_id)]})

    # 팔이 정해지기 전에 이름만 먼저 바꾼다: 당근 -> carrot(검출기의 클래스 이름). 모든 operation에 적용한다.
    named = policies.resolve(class_name, requested_robot_id) if policies is not None else None
    if operation == "PICK_WITH_POLICY":
        if policies is None:
            h._json_error(409, "ACT_POLICIES_NOT_CONFIGURED", getattr(h.app, "act_policies_error", None) or "정책 정의 파일이 없습니다")
            return
        if named is None:
            no_policy(requested_robot_id)
            return
    if named is not None:
        class_name = named["object_class"]
    camera_id = body.get("camera_id", "shelf_camera")
    snapshot = next((camera for camera in h.app.camera_manager.snapshots() if camera.id == camera_id), None)
    if snapshot is None:
        h._json_error(404, "CAMERA_NOT_FOUND", f"camera {camera_id!r} is not configured")
        return
    try:
        selection = select_slot_for_class(h.app.shelf_slot_zones.annotate(snapshot.to_dict()), class_name)
        # 정책 집기는 물체(정책)에 묶인 팔을, 자세만 옮기는 칸 이동은 그 선반을 맡은 팔을 기대값으로 쓴다.
        # 주의: 아래 줄의 `policy`는 이 함수 어디에서도 정의되지 않은 이름이다(위의 `named`를 뜻한 것으로 보인다).
        # PICK_WITH_POLICY 요청은 여기서 NameError가 난다. 알려진 버그로 코드는 그대로 둔다.
        arms = getattr(h.app, "arms", None)
        slot_map = h.app.arm_commands.slot_map
        slot = slot_map.active_slots.get(selection["slot_id"]) if slot_map is not None else None
        shelf_arm = arms.for_shelf(slot["shelf_id"]) if arms is not None and slot is not None else None
        if operation == "PICK_WITH_POLICY" and policy is not None and policy["arm_id"] is not None:
            assigned_arm = arms.get(policy["arm_id"]) if arms is not None else None
            expected_robot_id = policy["arm_id"] if assigned_arm is not None else None
            mismatch_code = "ARM_POLICY_MISMATCH"
            mismatch_message = f"{policy['label']} 정책은 {assigned_arm['label'] if assigned_arm else policy['arm_id']}에 고정되어 있습니다."
        else:
            expected_robot_id = shelf_arm["arm_id"] if shelf_arm else None
            mismatch_code = "ARM_SHELF_MISMATCH"
            mismatch_message = (
                f"{selection['slot_label']}은 {shelf_arm['label']} 담당 선반입니다. 팔 선택을 확인하세요."
                if shelf_arm else "검출된 선반 칸을 담당할 팔 설정이 없습니다"
            )
        if expected_robot_id is None:
            h._json_error(409, "SHELF_ARM_NOT_CONFIGURED", "검출된 선반 칸을 담당할 팔 설정이 없습니다")
            return
        if requested_robot_id is not None and requested_robot_id != expected_robot_id:
            h._json_error(
                409,
                mismatch_code,
                mismatch_message,
                details={"expected_robot_id": expected_robot_id},
            )
            return
        robot_id = shelf_arm["arm_id"]
        # 정책은 이 팔에서 녹화한 것(또는 팔에 묶이지 않은 것)이어야 한다.
        if operation == "PICK_WITH_POLICY" and policies.resolve(class_name, robot_id) is None:
            no_policy(robot_id)
            return
        request: dict[str, Any] = {
            "robot_id": robot_id,
            "operation": operation,
            "valid_for_s": body.get("valid_for_s", 60),
            "target": {"object_class": class_name, "pick_slot_id": selection["slot_id"]},
            "place": {"slot_id": "CART_1"},
            "source": {"kind": "OPERATOR_INPUT"},  # 아래 create의 camera_source(카메라 근거)로 바뀐다
        }
        command = h.app.arm_commands.create(request, camera_source=selection)
    except SlotSelectionError as exc:
        h._json_error(409, exc.error_code, str(exc), details=exc.details)
        return
    except ArmCommandError as exc:
        h._json_error(400, "ARM_COMMAND_INVALID", str(exc))
        return
    except ArmCommandConflict as exc:
        h._json_error(409, exc.error_code, str(exc), command=exc.command)
        return
    h._json(201, {"command": command, "selection": selection})

