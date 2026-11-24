"""물체 이름 -> 그 물체를 집도록 학습한 LeRobot ACT 정책 (config/omx-act-policies.json).

ACT 정책은 카메라 영상과 관절 위치만 입력으로 받는다. 물체가 어디 있는지 알려 줄 수 없다. 그래서 관제는 물체
이름으로 정책을 고르고, 카메라가 본 칸은 근거로만 남긴다. 팔 워커도 같은 파일을 읽고, 명령의 정책 파일 버전
(SHA-256)이 자기 것과 다르면 그 명령을 거부한다(POLICY_REGISTRY_MISMATCH).

정책은 팔 한 대에서 학습하므로 그 팔을 적어 둘 수 있다(`arm_id`, 선택). 그러면 같은 물체에 팔마다 정책이 하나씩
있을 수 있다. `arm_id`가 없는 정책은 어느 팔에서나 쓴다.

쓰는 곳: 팔 명령 검증(arms/commands.py), 주문 물체 검증(api/common.py), 주문 흐름의 팔 배정, 정책 실행 프로필
(arms/policy_runner.py), 연속 실행(arms/sequences.py).
"""

from __future__ import annotations

import hashlib
import os
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from control_station import REPO_ROOT
from control_station.storage.json_files import load_json_file

DEFAULT_POLICIES_PATH = REPO_ROOT / "config" / "omx-act-policies.json"
MAX_POLICIES_BYTES = 64 * 1024
# 정책의 카메라 입력 이름(front, wrist)마다 어느 카메라 역할(탑 캠 OVERHEAD, 그리퍼 캠 GRIPPER)을 넣을지 정책 파일에
# 적는다. 일부 정책은 둘을 바꿔 녹화했기 때문에 정책마다 따로 정한다.
CAMERA_ROLES = frozenset({"OVERHEAD", "GRIPPER"})
POLICY_INPUTS = frozenset({"front", "wrist"})
_POLICY_ID = re.compile(r"^[a-z0-9_]{1,40}$")
_MODEL = re.compile(r"^[A-Za-z0-9_.-]{1,96}/[A-Za-z0-9_.-]{1,96}$")
_ARM_ID = re.compile(r"^arm-[0-9]{2}$")


class ActPolicyError(ValueError):
    """정책 정의 파일이 없거나 schema_version 1 형식에 맞지 않는다."""


@dataclass(frozen=True)
class ActPolicyRegistry:
    """읽어 들인 정책 목록. version은 파일 바이트의 SHA-256이라 내용이 한 글자만 바뀌어도 달라진다."""
    path: Path
    version: str
    policies: dict[str, dict[str, Any]] = field(repr=False)

    def matches(self, name: str) -> list[dict[str, Any]]:
        """물체 클래스, 표시 이름(label), 별칭 중 하나가 `name`과 같은 정책 전부 (대소문자 무시)."""
        wanted = name.strip().casefold()
        return [
            policy
            for policy in self.policies.values()
            if wanted in {item.casefold() for item in (policy["object_class"], policy["label"], *policy["aliases"])}
        ]

    def resolve(self, name: str, arm_id: str | None = None) -> dict[str, Any] | None:
        """`name`의 정책 하나. `arm_id`를 주면 그 팔에서 학습한 정책, 없으면 팔에 묶이지 않은 정책이다.

        `arm_id`를 주지 않으면 팔에 묶이지 않은 정책을 먼저, 없으면 찾은 것 중 첫 번째를 돌려준다."""
        found = self.matches(name)
        if arm_id is not None:
            return next((policy for policy in found if policy["arm_id"] == arm_id), None) or next(
                (policy for policy in found if policy["arm_id"] is None), None
            )
        return next((policy for policy in found if policy["arm_id"] is None), None) or (found[0] if found else None)

    def summary(self) -> list[dict[str, Any]]:
        """화면용 정책 목록 (GET /api/arm/policies)."""
        return [
            {key: policy[key] for key in ("policy_id", "object_class", "label", "aliases", "model", "arm_id")}
            for policy in self.policies.values()
        ]


def _text(policy: dict[str, Any], key: str, maximum: int = 80) -> str:
    """정책 항목의 글자 필드 하나를 검증하고 앞뒤 공백을 뺀다."""
    value = policy.get(key)
    if not isinstance(value, str) or not value.strip() or len(value) > maximum:
        raise ActPolicyError(f"{policy.get('policy_id')!r}.{key} must be text of 1 to {maximum} characters")
    return value.strip()


def _policy(raw: object) -> dict[str, Any]:
    """정책 항목 하나를 검증하고 정리한 사본을 돌려준다. model은 Hugging Face 저장소 ID(owner/name)다."""
    if not isinstance(raw, dict):
        raise ActPolicyError("each policy must be an object")
    policy_id = raw.get("policy_id")
    if not isinstance(policy_id, str) or not _POLICY_ID.fullmatch(policy_id):
        raise ActPolicyError("policy_id must be 1 to 40 lowercase letters, digits or _")
    model = _text(raw, "model", 200)
    if not _MODEL.fullmatch(model):
        raise ActPolicyError(f"{policy_id!r}.model must be a Hugging Face repo id like owner/name")
    if raw.get("policy_type") != "act":
        raise ActPolicyError(f"{policy_id!r}.policy_type must be 'act'")
    aliases = raw.get("aliases")
    if not isinstance(aliases, list) or not all(isinstance(alias, str) and alias.strip() for alias in aliases):
        raise ActPolicyError(f"{policy_id!r}.aliases must be a list of names")
    cameras = raw.get("cameras")
    if not isinstance(cameras, dict) or set(cameras) != POLICY_INPUTS or set(cameras.values()) != CAMERA_ROLES:
        raise ActPolicyError(f"{policy_id!r}.cameras must map front and wrist to OVERHEAD and GRIPPER, one each")
    arm_id = raw.get("arm_id")
    if arm_id is not None and (not isinstance(arm_id, str) or not _ARM_ID.fullmatch(arm_id)):
        raise ActPolicyError(f"{policy_id!r}.arm_id must look like arm-01 when given")
    return {
        "policy_id": policy_id,
        "object_class": _text(raw, "object_class"),
        "label": _text(raw, "label"),
        "aliases": [alias.strip() for alias in aliases],
        "model": model,
        "policy_type": "act",
        "task": _text(raw, "task", 200),
        "cameras": dict(cameras),
        "arm_id": arm_id,
    }


def load_act_policies(path: str | Path | None = None) -> ActPolicyRegistry:
    """정책 파일을 읽는다. 경로는 인자, CONTROL_STATION_ACT_POLICIES, config/omx-act-policies.json 순으로 정한다."""
    policy_path = Path(path or os.environ.get("CONTROL_STATION_ACT_POLICIES") or DEFAULT_POLICIES_PATH)
    raw, document = load_json_file(policy_path, MAX_POLICIES_BYTES, "정책 정의 파일", ActPolicyError)
    # True == 1이므로 bool을 따로 거른다.
    if not isinstance(document, dict) or document.get("schema_version") != 1 or isinstance(document.get("schema_version"), bool):
        raise ActPolicyError("정책 정의 파일의 schema_version은 1이어야 합니다")
    if not isinstance(document.get("policies"), list) or not document["policies"]:
        raise ActPolicyError("policies must be a non-empty list")
    policies: dict[str, dict[str, Any]] = {}
    # 이름 -> [(policy_id, arm_id)]. 같은 이름은 서로 다른 팔에 묶인 정책끼리만 겹칠 수 있다. 한쪽이라도 팔에 묶이지
    # 않았으면(None) 어느 팔에서 그 이름을 찾든 답이 둘이 되므로 거부한다.
    names: dict[str, list[tuple[str, str | None]]] = {}
    for raw_policy in document["policies"]:
        policy = _policy(raw_policy)
        if policy["policy_id"] in policies:
            raise ActPolicyError(f"duplicate policy_id: {policy['policy_id']}")
        for name in {policy["object_class"], policy["label"], *policy["aliases"]}:
            owners = names.setdefault(name.casefold(), [])
            if any(arm is None or policy["arm_id"] is None or arm == policy["arm_id"] for _, arm in owners):
                raise ActPolicyError(f"name {name!r} belongs to two policies for the same arm")
            owners.append((policy["policy_id"], policy["arm_id"]))
        policies[policy["policy_id"]] = policy
    return ActPolicyRegistry(policy_path, hashlib.sha256(raw).hexdigest(), policies)
