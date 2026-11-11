"""팔 텔레오퍼레이션, OMX 명령 큐, 선반 칸, 좌표 초안"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any
from urllib.parse import parse_qs, urlsplit

from control_station.arm_commands import ArmCommandConflict, ArmCommandError, ArmCommandNotFound
from control_station.camera_to_arm import CameraToArmError
from control_station.coordinate_target_drafts import CoordinateDraftError
from control_station.shelf_slot_zones import SlotSelectionError, select_slot_for_class

if TYPE_CHECKING:
    from control_station.app import ControlStationHandler


def get_status(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """팔 텔레오퍼레이션·학습 정책 한 번 실행 상태"""
    h._json(200, {"arm": h.app.arm_supervisor.snapshot(), "policy_run": h.app.policy_run_supervisor.snapshot()})


def post_teleop(h: ControlStationHandler, route: str, parts: list[str], body: None) -> None:
    """팔 텔레오퍼레이션 시작·정지, 학습 정책 한 번 실행 시작·중단 (둘은 동시에 실행하지 않음)"""
    policy_route = route.startswith("/api/arm/policy/")
    response_key = "policy_run" if policy_route else "arm"
    try:
        with h.app.arm_control_lock:
            if route == "/api/arm/start":
                if h.app.policy_run_supervisor.snapshot()["process_alive"]:
                    raise RuntimeError("학습 정책 실행이 끝날 때까지 텔레오퍼레이션을 시작할 수 없습니다.")
                state = h.app.arm_supervisor.start()
            elif route == "/api/arm/stop":
                state = h.app.arm_supervisor.stop()
            elif route == "/api/arm/policy/start":
                if h.app.arm_supervisor.snapshot()["process_alive"]:
                    raise RuntimeError("텔레오퍼레이션을 종료한 뒤 학습 정책을 실행하세요.")
                profile_ids = parse_qs(urlsplit(h.path).query).get("profile_id", [])
                if len(profile_ids) > 1:
                    raise RuntimeError("profile_id는 하나만 지정할 수 있습니다.")
                hold_front = getattr(h.app.camera_manager, "hold_front_for_policy", None)
                if hold_front is not None and not hold_front(True):
                    hold_front(False)
                    raise RuntimeError(
                        "관제 화면이 Front Camera를 놓았는지 확인하지 못해 정책을 시작하지 않았습니다. 잠시 뒤 다시 시도하세요."
                    )
                try:
                    state = h.app.policy_run_supervisor.start(profile_ids[0] if profile_ids else None)
                finally:
                    # A live policy process keeps the camera released through policy_active.
                    if hold_front is not None:
                        hold_front(False)
            else:
                state = h.app.policy_run_supervisor.stop()
    except RuntimeError as exc:
        snapshot = h.app.policy_run_supervisor.snapshot() if policy_route else h.app.arm_supervisor.snapshot()
        h._json(409, {"error": str(exc), response_key: snapshot})
        return
    h._json(202, {response_key: state})


POLICY_IN_FLIGHT = frozenset({"DISPATCHED", "ACCEPTED", "RUNNING"})


def sync_policy_cameras(app: Any) -> None:
    """정책 집기 명령이 워커에 넘어가 끝나기 전까지, 정책이 쓰는 카메라를 관제가 닫아 둔다.

    V4L2 카메라는 한 프로그램만 열 수 있다. 정책이 쓰는 카메라(CONTROL_STATION_ACT_OVERHEAD_CAMERA,
    CONTROL_STATION_JOINT_CAMERA)를 관제도 화면에 띄우는 경우, 명령이 DISPATCHED·ACCEPTED·RUNNING인 동안만
    관제가 닫고 끝나면 다시 연다. 워커는 열기 전에 장치가 비었는지 확인한다."""
    devices = getattr(app, "policy_camera_devices", None)
    manager = getattr(app, "camera_manager", None)
    if not devices or manager is None or not hasattr(manager, "lend_devices"):
        return
    commands = app.arm_commands.list_recent()["commands"]
    in_flight = any(
        command["operation"] == "PICK_WITH_POLICY" and command["state"] in POLICY_IN_FLIGHT for command in commands
    )
    with app.policy_camera_lock:
        if in_flight and not app.policy_cameras_lent:
            closed = manager.lend_devices(devices, "학습 정책 실행 중: 이 카메라는 정책이 쓰고 있습니다")
            print(f"정책 실행: 관제가 카메라를 닫음 {closed}", flush=True)
            app.policy_cameras_lent = True
        elif not in_flight and app.policy_cameras_lent:
            print(f"정책 종료: 관제가 카메라를 다시 엶 {manager.take_back_devices()}", flush=True)
            app.policy_cameras_lent = False


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

    operation MOVE_TO_SLOT(기본)은 그 칸 자세로 이동, PICK_WITH_POLICY는 그 물체용 학습 정책 실행,
    MOVE_TO_POSITION은 카메라가 잰 물체 위치를 팔 좌표로 바꿔 물체 바로 위(손목 45° 아래)로 이동.
    정책은 위치를 입력으로 받지 않으므로, 정책 명령에서 칸은 물체가 실제로 있다는 근거로만 남는다."""
    allowed = {"object_class", "camera_id", "valid_for_s", "operation"}
    object_class = body.get("object_class")
    operation = body.get("operation", "MOVE_TO_SLOT")
    if (
        set(body) - allowed
        or not isinstance(object_class, str)
        or not object_class.strip()
        or operation not in {"MOVE_TO_SLOT", "PICK_WITH_POLICY", "MOVE_TO_POSITION", "PICK_AT_POSITION"}
    ):
        h._json_error(
            400,
            "ARM_COMMAND_INVALID",
            "body needs object_class and may have camera_id, valid_for_s, "
            "operation (MOVE_TO_SLOT, PICK_WITH_POLICY, MOVE_TO_POSITION or PICK_AT_POSITION)",
        )
        return
    class_name = object_class.strip()
    policies = h.app.arm_commands.act_policies
    if operation != "PICK_WITH_POLICY" and policies is not None and policies.resolve(class_name):
        class_name = policies.resolve(class_name)["object_class"]  # 당근 -> carrot for the other operations too
    camera_to_arm = getattr(h.app, "camera_to_arm", None)
    if operation in {"MOVE_TO_POSITION", "PICK_AT_POSITION"} and camera_to_arm is None:
        h._json_error(409, "CAMERA_TO_ARM_NOT_CONFIGURED",
                      getattr(h.app, "camera_to_arm_error", None) or "카메라→팔 변환(CONTROL_STATION_CAMERA_TO_ARM)이 없습니다")
        return
    if operation == "PICK_WITH_POLICY":
        policies = h.app.arm_commands.act_policies
        if policies is None:
            h._json_error(409, "ACT_POLICIES_NOT_CONFIGURED", getattr(h.app, "act_policies_error", None) or "정책 정의 파일이 없습니다")
            return
        policy = policies.resolve(class_name)
        if policy is None:
            h._json_error(409, "POLICY_NOT_FOUND", f"{class_name}에 학습된 정책이 없습니다",
                          details={"policies": [f"{item['label']}({item['object_class']})" for item in policies.policies.values()]})
            return
        class_name = policy["object_class"]  # 당근 -> carrot: the detector's class name
    camera_id = body.get("camera_id", "shelf_camera")
    snapshot = next((camera for camera in h.app.camera_manager.snapshots() if camera.id == camera_id), None)
    if snapshot is None:
        h._json_error(404, "CAMERA_NOT_FOUND", f"camera {camera_id!r} is not configured")
        return
    hover = None
    try:
        selection = select_slot_for_class(h.app.shelf_slot_zones.annotate(snapshot.to_dict()), class_name)
        request: dict[str, Any] = {
            "robot_id": "arm-01",
            "operation": operation,
            "valid_for_s": body.get("valid_for_s", 60),
            "target": {"object_class": class_name, "pick_slot_id": selection["slot_id"]},
            "place": {"slot_id": "CART_1"},
            "source": {"kind": "OPERATOR_INPUT"},  # replaced by the camera evidence below
        }
        if operation in {"MOVE_TO_POSITION", "PICK_AT_POSITION"}:
            if not selection.get("position_shelf_m"):
                h._json_error(409, "OBJECT_POSITION_UNKNOWN", "정면 카메라가 물체의 선반 위치를 계산하지 못했습니다",
                              details={"slot_id": selection.get("slot_id")})
                return
            hover = camera_to_arm.hover_target(selection["position_shelf_m"])
            arm_target = (
                camera_to_arm.object_center_target(selection["position_shelf_m"])
                if operation == "PICK_AT_POSITION" else hover
            )
            request["target"] = {"object_class": class_name,
                                 **{key: arm_target[key] for key in ("frame_id", "unit", "position", "approach")}}
            if operation == "MOVE_TO_POSITION":
                del request["place"]  # PICK_AT_POSITION keeps place CART_1 (Pinky)
        command = h.app.arm_commands.create(request, camera_source=selection)
    except CameraToArmError as exc:
        h._json_error(409, "CAMERA_TO_ARM_FAILED", str(exc))
        return
    except SlotSelectionError as exc:
        h._json_error(409, exc.error_code, str(exc), details=exc.details)
        return
    except ArmCommandError as exc:
        h._json_error(400, "ARM_COMMAND_INVALID", str(exc))
        return
    except ArmCommandConflict as exc:
        h._json_error(409, exc.error_code, str(exc), command=exc.command)
        return
    h._json(201, {"command": command, "selection": selection, "hover": hover})


def post_coordinate_draft(h: ControlStationHandler, route: str, parts: list[str], body: dict[str, Any]) -> None:
    """좌표 목표 초안 저장"""
    try:
        draft = h.app.coordinate_target_drafts.create(body)
    except CoordinateDraftError as exc:
        h._json(400, {"error": str(exc)})
        return
    h._json(201, {"draft": draft})
