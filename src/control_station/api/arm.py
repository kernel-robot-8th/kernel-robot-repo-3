"""OMX 명령 큐와 선반 칸 API"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any
from urllib.parse import parse_qs, urlsplit

from control_station.arm_commands import ARM_ROBOT_IDS, ArmCommandConflict, ArmCommandError, ArmCommandNotFound
from control_station.arm_sequences import ArmSequenceError
from control_station.shelf_slot_zones import SlotSelectionError, select_slot_for_class

if TYPE_CHECKING:
    from control_station.app import ControlStationHandler


def post_policy_run(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """학습 정책 실행 시작·중단"""
    response_key = "policy_run"
    try:
        with h.app.arm_control_lock:
            if route == "/api/arm/policy/start":
                profile_ids = parse_qs(urlsplit(h.path).query).get("profile_id", [])
                if len(profile_ids) > 1:
                    raise RuntimeError("profile_id는 하나만 지정할 수 있습니다.")
                profile_id = profile_ids[0] if profile_ids else None
                # Close the control station's feeds on the cameras this run opens (its arm's pair) before starting.
                devices = h.app.policy_run_supervisor.devices_for(profile_id)
                manager = h.app.camera_manager
                hold = getattr(manager, "hold_for_policy", None)
                if hold is None:
                    hold_front = getattr(manager, "hold_front_for_policy", None)
                    hold = (lambda _devices, on: hold_front(on)) if hold_front is not None else None
                if hold is not None and not hold(devices, True):
                    hold(devices, False)
                    raise RuntimeError(
                        "관제 화면이 정책 카메라를 놓았는지 확인하지 못해 정책을 시작하지 않았습니다. 잠시 뒤 다시 시도하세요."
                    )
                try:
                    state = h.app.policy_run_supervisor.start(profile_id)
                finally:
                    # A live policy process keeps its cameras released through policy_devices / policy_active.
                    if hold is not None:
                        hold(devices, False)
            else:
                state = h.app.policy_run_supervisor.stop()
    except RuntimeError as exc:
        h._json(409, {"error": str(exc), response_key: h.app.policy_run_supervisor.snapshot()})
        return
    h._json(202, {response_key: state})


POLICY_IN_FLIGHT = frozenset({"DISPATCHED", "ACCEPTED", "RUNNING"})


def _policy_devices_for(app: Any, robot_id: str) -> list[str]:
    """Cameras the worker of this arm opens: {arm_id: [...]} with "*" for the older single-arm list, or a list."""
    devices = app.policy_camera_devices
    if isinstance(devices, dict):
        return list(devices.get(robot_id) or devices.get("*") or [])
    return list(devices or [])


def sync_policy_cameras(app: Any) -> None:
    """정책 집기 명령이 워커에 넘어가 끝나기 전까지, 그 팔의 정책이 쓰는 카메라를 관제가 닫아 둔다.

    V4L2 카메라는 한 프로그램만 열 수 있다. 팔마다 그리퍼 캠·탑 캠(CONTROL_STATION_ARM01_*_CAMERA, ARM02_*; 예전 단일 팔
    설정은 CONTROL_STATION_ACT_OVERHEAD_CAMERA, CONTROL_STATION_JOINT_CAMERA)을 관제도 화면에 띄우는 경우, 명령이
    DISPATCHED·ACCEPTED·RUNNING인 동안만 그 팔의 카메라를 닫고 끝나면 다시 연다. 워커는 열기 전에 장치가 비었는지 확인한다."""
    manager = getattr(app, "camera_manager", None)
    if not getattr(app, "policy_camera_devices", None) or manager is None or not hasattr(manager, "lend_devices"):
        return
    commands = app.arm_commands.list_recent()["commands"]
    wanted = tuple(dict.fromkeys(
        device
        for command in commands
        if command["operation"] == "PICK_WITH_POLICY" and command["state"] in POLICY_IN_FLIGHT
        for device in _policy_devices_for(app, command["robot_id"])
    ))
    with app.policy_camera_lock:
        lent = getattr(app, "policy_cameras_lent_devices", None)
        lent = lent if isinstance(lent, tuple) else ()
        if app.policy_cameras_lent and lent == wanted:
            return
        if app.policy_cameras_lent:
            print(f"정책 종료: 관제가 카메라를 다시 엶 {manager.take_back_devices()}", flush=True)
            app.policy_cameras_lent = False
        if wanted:
            closed = manager.lend_devices(list(wanted), "학습 정책 실행 중: 이 카메라는 정책이 쓰고 있습니다")
            print(f"정책 실행: 관제가 카메라를 닫음 {closed}", flush=True)
            app.policy_cameras_lent = True
        app.policy_cameras_lent_devices = wanted


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
    try:
        h._json(200, {"sequences": h.app.arm_sequences.definitions(), "latest": h.app.arm_sequences.latest()})
    except ArmSequenceError as exc:
        h._json_error(409, "ARM_SEQUENCE_CONFIG_INVALID", str(exc))


def get_sequence(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    run_id = parts[3]
    try:
        h._json(200, {"sequence": h.app.arm_sequences.get(run_id)})
    except KeyError:
        h._json_error(404, "ARM_SEQUENCE_NOT_FOUND", "Arm sequence not found")


def post_sequence(h: ControlStationHandler, route: str, parts: list[str], body: dict[str, Any]) -> None:
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
        h._json_error(409, getattr(exc, "error_code", "ARM_COMMAND_CONFLICT"), str(exc))


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
    """워커가 다음 명령 받기 (기다리다 없으면 204)"""
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
        sync_policy_cameras(h.app)  # close the policy's cameras before the worker tries to open them
    h._json(200, {"command": command})


def post_command(h: ControlStationHandler, route: str, parts: list[str], body: dict[str, Any]) -> None:
    """OMX 명령 등록·진행 이벤트·취소·불명 해제"""
    store = h.app.arm_commands
    try:
        if len(parts) == 3:
            h._json(201, {"command": store.create(body)})
        elif parts[4] == "events":
            command, applied = store.apply_worker_event(parts[3], body)
            h.app.arm_sequences.on_worker_event(command, applied)
            if command["operation"] == "PICK_WITH_POLICY":
                sync_policy_cameras(h.app)  # reopen them once the command has finished
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
        h._json_error(409, "POLICY_NOT_FOUND", f"{arm_id or '어느 팔'}에도 {class_name}에 학습된 정책이 없습니다",
                      details={"policies": [f"{item['label']}({item['object_class']})" for item in policies.policies.values()
                                            if arm_id is None or item["arm_id"] in (None, arm_id)]})

    # The name only, before the arm is known: 당근 -> carrot, the detector's class name, for every operation.
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
        # Policy picks use the object-to-arm assignment; pose-only slot moves use the arm serving the shelf.
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
        # The policy must be one recorded on this arm (or bound to no arm).
        if operation == "PICK_WITH_POLICY" and policies.resolve(class_name, robot_id) is None:
            no_policy(robot_id)
            return
        request: dict[str, Any] = {
            "robot_id": robot_id,
            "operation": operation,
            "valid_for_s": body.get("valid_for_s", 60),
            "target": {"object_class": class_name, "pick_slot_id": selection["slot_id"]},
            "place": {"slot_id": "CART_1"},
            "source": {"kind": "OPERATOR_INPUT"},  # replaced by the camera evidence below
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

