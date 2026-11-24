"""팔 설정: 어느 팔이 어느 선반을 맡는지(config/arms.json)와, 이 PC에서 각 팔의 장치가 어디 있는지(환경 변수).

1) 팔과 담당 선반 (`load_arms`, `ArmRegistry`)
   핑키가 서는 자리 하나의 양옆에 팔 두 대가 있다. 왼쪽 선반에서 찾은 물체는 왼쪽 팔이, 오른쪽 선반의 물체는
   오른쪽 팔이 집는다(2026-11-11 결정). 그래서 주문한 물체마다 팔은 물체 종류가 아니라, 재고 확인에서 그 물체를
   본 선반으로 정한다.

   `board_serial`(선택)은 팔의 OpenRB-150 보드 시리얼로, 실물 팔을 가리킨다. 아래 2)의 예전 단일 팔 포트 설정은
   이 시리얼로 어느 팔의 것인지 맞춘다.

2) 팔마다 장치 경로 (`arm_devices`, `ArmDevices`)
   팔로워 시리얼 포트, LeRobot robot id, 그리퍼·탑 카메라. 팔이 두 대면 카메라는 여섯 대다. 팔마다 ACT 정책을
   학습할 때 쓴 그리퍼(관절) 카메라와 탑 카메라가 있고, 핑키 주차를 판정하는 천장 카메라와 재고를 세는 선반
   카메라가 따로 있다. 장치 경로는 PC마다 달라서 .control-station/control-station.env에서 팔마다 한 벌씩 읽는다.

       CONTROL_STATION_ARM01_FOLLOWER_PORT     /dev/serial/by-id/usb-ROBOTIS_OpenRB-150_<보드 시리얼>-if00
       CONTROL_STATION_ARM01_ROBOT_ID          LeRobot 보정 이름 (기본 CONTROL_STATION_POLICY_ROBOT_ID)
       CONTROL_STATION_ARM01_GRIPPER_CAMERA    /dev/v4l/by-path/...   (같은 모델 카메라 두 대는 by-id가 같아서 by-path를 쓴다)
       CONTROL_STATION_ARM01_OVERHEAD_CAMERA   /dev/v4l/by-path/...
       (ARM02도 같다)

   쪽 이름도 쓸 수 있다(2026-11-16, main에서). arm-01은 CONTROL_STATION_LEFT_*, arm-02는 RIGHT_*이고, 그리퍼 카메라는
   JOINT_CAMERA, 탑 카메라는 CEILING_CAMERA다. ARMxx 설정이 쪽 이름 설정보다 우선한다.

   예전 단일 팔 설정(CONTROL_STATION_POLICY_ROBOT_PORT 또는 CONTROL_STATION_FOLLOWER_PORT,
   CONTROL_STATION_POLICY_*_CAMERA, CONTROL_STATION_ACT_OVERHEAD_CAMERA, CONTROL_STATION_JOINT_CAMERA)도 계속 된다.
   다만 한 팔에만 해당한다. 그 포트 경로에 보드 시리얼이 들어 있는 팔이고, 그 팔의 ARMxx 설정이 없을 때만이다.
   그래서 팔에 묶인 정책이 다른 팔의 포트나 카메라로 실행되는 일은 없다.
"""

from __future__ import annotations

import os
import re
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from control_station import REPO_ROOT
from control_station.storage.json_files import load_json_file

# ──── 1) 팔과 담당 선반 (config/arms.json) ──────────────────────────────────────────────────────────────────────

DEFAULT_ARMS_PATH = REPO_ROOT / "config" / "arms.json"
MAX_ARMS_BYTES = 16 * 1024
ARM_ID = re.compile(r"^arm-[0-9]{2}$")
_NAME = re.compile(r"^[A-Za-z0-9_.-]{1,80}$")
_BOARD_SERIAL = re.compile(r"^[0-9A-F]{32}$")
_REQUIRED = {"arm_id", "label", "shelf_id", "stock_camera_id"}


class ArmConfigError(ValueError):
    """팔 설정 파일이 없거나 schema_version 1 형식에 맞지 않는다."""


@dataclass(frozen=True)
class ArmRegistry:
    """config/arms.json에서 읽은 팔 목록(파일 순서). 팔마다 arm_id, label, shelf_id, stock_camera_id, board_serial."""
    arms: tuple[dict[str, Any], ...]

    @property
    def arm_ids(self) -> frozenset[str]:
        """설정된 팔 ID 전부."""
        return frozenset(arm["arm_id"] for arm in self.arms)

    def get(self, arm_id: str) -> dict[str, Any] | None:
        """arm_id의 팔 설정, 없으면 None."""
        return next((arm for arm in self.arms if arm["arm_id"] == arm_id), None)

    def for_shelf(self, shelf_id: str) -> dict[str, Any] | None:
        """그 선반을 맡은 팔, 없으면 None. 선반마다 팔은 하나다(load_arms가 shelf_id 중복을 거부한다)."""
        return next((arm for arm in self.arms if arm["shelf_id"] == shelf_id), None)


def _arm(raw: object) -> dict[str, Any]:
    """팔 항목 하나를 검증하고 정리한 사본을 돌려준다. board_serial이 없으면 None으로 채운다."""
    if not isinstance(raw, dict) or not _REQUIRED <= set(raw) <= _REQUIRED | {"board_serial"}:
        raise ArmConfigError("each arm needs arm_id, label, shelf_id and stock_camera_id (board_serial optional)")
    arm_id = raw["arm_id"]
    if not isinstance(arm_id, str) or not ARM_ID.fullmatch(arm_id):
        raise ArmConfigError("arm_id must look like arm-01")
    label = raw["label"]
    if not isinstance(label, str) or not label.strip() or len(label) > 40:
        raise ArmConfigError(f"{arm_id}.label must be text of 1 to 40 characters")
    for key in ("shelf_id", "stock_camera_id"):
        if not isinstance(raw[key], str) or not _NAME.fullmatch(raw[key]):
            raise ArmConfigError(f"{arm_id}.{key} has an invalid format")
    board_serial = raw.get("board_serial")
    if board_serial is not None and (not isinstance(board_serial, str) or not _BOARD_SERIAL.fullmatch(board_serial)):
        raise ArmConfigError(f"{arm_id}.board_serial must be the 32 hex digits of the OpenRB-150 serial")
    return {"arm_id": arm_id, "label": label.strip(), "shelf_id": raw["shelf_id"],
            "stock_camera_id": raw["stock_camera_id"], "board_serial": board_serial}


def load_arms(path: str | Path | None = None) -> ArmRegistry:
    """팔 설정을 읽는다. 경로는 인자, CONTROL_STATION_ARMS, config/arms.json 순으로 정한다.

    arm_id·shelf_id·board_serial이 팔 사이에서 겹치면 거부한다. 겹치면 "이 선반은 어느 팔?"이나 "이 보드는 어느 팔?"에
    답이 둘이 되기 때문이다. 틀리면 ArmConfigError다."""
    arms_path = Path(path or os.environ.get("CONTROL_STATION_ARMS") or DEFAULT_ARMS_PATH)
    _, document = load_json_file(arms_path, MAX_ARMS_BYTES, "팔 설정 파일", ArmConfigError)
    # True == 1이므로 bool을 따로 거른다.
    if not isinstance(document, dict) or document.get("schema_version") != 1 or isinstance(document.get("schema_version"), bool):
        raise ArmConfigError("팔 설정 파일의 schema_version은 1이어야 합니다")
    if not isinstance(document.get("arms"), list) or not document["arms"]:
        raise ArmConfigError("arms must be a non-empty list")
    arms = tuple(_arm(item) for item in document["arms"])
    for key in ("arm_id", "shelf_id", "board_serial"):
        values = [arm[key] for arm in arms if arm[key] is not None]
        if len(set(values)) != len(values):
            raise ArmConfigError(f"duplicate {key} in arms")
    return ArmRegistry(arms)


# ──── 2) 팔마다 장치 경로 (환경 변수) ────────────────────────────────────────────────────────────────────────────────

# LeRobot 보정 파일 이름의 기본값. CONTROL_STATION_POLICY_ROBOT_ID나 팔별 ROBOT_ID로 바꾼다.
DEFAULT_ROBOT_ID = "omx_follower_arm"
CAMERA_ROLES = ("GRIPPER", "OVERHEAD")
ROLE_LABELS = {"GRIPPER": "그리퍼 캠", "OVERHEAD": "탑 캠"}
_FIELDS = ("FOLLOWER_PORT", "ROBOT_ID", "GRIPPER_CAMERA", "OVERHEAD_CAMERA")


def env_prefix(arm_id: str) -> str:
    """arm-01 -> CONTROL_STATION_ARM01_"""
    return "CONTROL_STATION_" + arm_id.replace("-", "").upper() + "_"


# 쪽 이름 설정(2026-11-16): arm-01 = 왼쪽(LEFT), arm-02 = 오른쪽(RIGHT). 카메라 필드 이름도 쪽 이름 쪽 표기로 바꾼다.
SIDE_PREFIX = {"arm-01": "CONTROL_STATION_LEFT_", "arm-02": "CONTROL_STATION_RIGHT_"}
SIDE_FIELD = {"GRIPPER_CAMERA": "JOINT_CAMERA", "OVERHEAD_CAMERA": "CEILING_CAMERA"}


def env_names(arm_id: str, field: str) -> tuple[str, ...]:
    """이 팔의 `field` 값을 줄 수 있는 환경 변수 이름들. 앞의 것이 우선한다: ARMxx_<field>, 그다음 쪽 이름.

    ROBOT_ID는 쪽 이름 설정이 없다."""
    names = [env_prefix(arm_id) + field]
    if arm_id in SIDE_PREFIX and field != "ROBOT_ID":
        names.append(SIDE_PREFIX[arm_id] + SIDE_FIELD.get(field, field))
    return tuple(names)


def _first(env: Mapping[str, str], *names: str) -> str | None:
    """names 중 값이 비어 있지 않은 첫 환경 변수의 값(앞뒤 공백 제거), 없으면 None."""
    for name in names:
        value = env.get(name, "").strip()
        if value:
            return value
    return None


def legacy_port(env: Mapping[str, str] = os.environ) -> str | None:
    """예전 단일 팔 팔로워 포트 설정(정책 실행기 이름 먼저, 그다음 팔 워커 이름)."""
    return _first(env, "CONTROL_STATION_POLICY_ROBOT_PORT", "CONTROL_STATION_FOLLOWER_PORT")


def legacy_cameras(env: Mapping[str, str] = os.environ) -> dict[str, str | None]:
    """예전 단일 팔 카메라 설정을 역할별로 돌려준다 (정책 실행기 쪽 이름 먼저, 그다음 팔 워커 쪽 이름).

    FRONT/WRIST는 예전 이름이다: front는 탑(OVERHEAD) 카메라, wrist는 그리퍼 카메라였다."""
    return {
        "OVERHEAD": _first(env, "CONTROL_STATION_POLICY_OVERHEAD_CAMERA", "CONTROL_STATION_POLICY_FRONT_CAMERA",
                           "CONTROL_STATION_ACT_OVERHEAD_CAMERA"),
        "GRIPPER": _first(env, "CONTROL_STATION_POLICY_GRIPPER_CAMERA", "CONTROL_STATION_POLICY_WRIST_CAMERA",
                          "CONTROL_STATION_JOINT_CAMERA"),
    }


def legacy_arm_id(arms: list[dict[str, Any]] | tuple[dict[str, Any], ...], env: Mapping[str, str] = os.environ) -> str | None:
    """예전 단일 팔 포트 설정이 가리키는 팔. 포트 경로(또는 심볼릭 링크를 따라간 실제 경로)에 보드 시리얼이 들어 있는
    팔을 찾는다. 하나로 정해지지 않으면(없거나 둘 이상) None이고, 그러면 예전 설정은 어느 팔에도 쓰이지 않는다."""
    port = legacy_port(env)
    if not port:
        return None
    paths = {port, os.path.realpath(port)}
    found = [arm["arm_id"] for arm in arms
             if arm.get("board_serial") and any(arm["board_serial"] in path for path in paths)]
    return found[0] if len(found) == 1 else None


@dataclass(frozen=True)
class ArmDevices:
    """한 팔의 이 PC 장치 설정: 팔로워 포트, LeRobot robot id, 역할별 카메라, 설정 출처."""
    arm_id: str
    label: str
    follower_port: str | None
    robot_id: str
    cameras: dict[str, str | None]  # GRIPPER / OVERHEAD -> 장치 경로
    source: str  # "arm": ARMxx(또는 쪽 이름) 설정 · "legacy": 예전 단일 팔 설정 · "none": 설정 없음

    @property
    def missing(self) -> list[str]:
        """이 팔에서 정책을 실행하려면 아직 필요한 설정 이름들(ARMxx 이름으로 안내한다)."""
        prefix = env_prefix(self.arm_id)
        names = [] if self.follower_port else [prefix + "FOLLOWER_PORT"]
        return names + [f"{prefix}{role}_CAMERA" for role in CAMERA_ROLES if not self.cameras.get(role)]

    @property
    def ready(self) -> bool:
        """포트와 두 카메라가 모두 설정됐는지."""
        return not self.missing

    @property
    def devices(self) -> list[str]:
        """이 팔에서 정책을 실행하면 정책 프로세스가 직접 여는 카메라 장치들(관제는 그동안 닫아 둔다)."""
        return [device for role in CAMERA_ROLES if (device := self.cameras.get(role))]

    def summary(self) -> dict[str, Any]:
        """화면 표시용 요약(정책 실행 상태의 arms 항목)."""
        return {
            "arm_id": self.arm_id, "label": self.label, "source": self.source, "ready": self.ready,
            "missing": self.missing, "follower_port": self.follower_port, "robot_id": self.robot_id,
            "cameras": dict(self.cameras),
        }


def arm_devices(arms: list[dict[str, Any]] | tuple[dict[str, Any], ...],
                env: Mapping[str, str] = os.environ) -> dict[str, ArmDevices]:
    """arm_id -> 이 PC에서의 장치 설정, 설정 파일 순서대로.

    팔마다 자기 설정(ARMxx 또는 쪽 이름)이 하나라도 있으면 그것만 쓴다. 자기 설정이 하나도 없고 예전 단일 팔 포트가
    이 팔을 가리킬 때만 예전 설정을 쓴다. 그래서 두 설정이 한 팔에서 섞이지 않는다."""
    default_robot_id = _first(env, "CONTROL_STATION_POLICY_ROBOT_ID") or DEFAULT_ROBOT_ID
    legacy_arm = legacy_arm_id(arms, env)
    result: dict[str, ArmDevices] = {}
    for arm in arms:
        own = {field: _first(env, *env_names(arm["arm_id"], field)) for field in _FIELDS}
        if any(own.values()) or arm["arm_id"] != legacy_arm:
            port, cameras = own["FOLLOWER_PORT"], {role: own[f"{role}_CAMERA"] for role in CAMERA_ROLES}
            source = "arm" if any(own.values()) else "none"
        else:
            port, cameras, source = legacy_port(env), legacy_cameras(env), "legacy"
        result[arm["arm_id"]] = ArmDevices(
            arm_id=arm["arm_id"], label=arm["label"], follower_port=port,
            robot_id=own["ROBOT_ID"] or default_robot_id, cameras=cameras, source=source,
        )
    return result
