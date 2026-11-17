"""Object name -> the LeRobot ACT policy trained to pick that object (config/omx-act-policies.json).

An ACT policy takes only camera images and joint positions; it cannot be told where the object is. The control
station therefore uses the object name to choose a policy and keeps the camera-observed slot as evidence only.
The arm worker loads the same file and refuses a command whose registry version (SHA-256) differs from its own.

A policy is trained on one arm, so it may name that arm (`arm_id`, optional). The same object can then have one
policy per arm; a policy without `arm_id` is used on any arm.
"""

from __future__ import annotations

import hashlib
import os
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from control_station.json_files import load_json_file

DEFAULT_POLICIES_PATH = Path(__file__).resolve().parents[2] / "config" / "omx-act-policies.json"
MAX_POLICIES_BYTES = 64 * 1024
CAMERA_ROLES = frozenset({"OVERHEAD", "GRIPPER"})
POLICY_INPUTS = frozenset({"front", "wrist"})
_POLICY_ID = re.compile(r"^[a-z0-9_]{1,40}$")
_MODEL = re.compile(r"^[A-Za-z0-9_.-]{1,96}/[A-Za-z0-9_.-]{1,96}$")
_ARM_ID = re.compile(r"^arm-[0-9]{2}$")


class ActPolicyError(ValueError):
    """The policy file is missing or does not match schema version 1."""


@dataclass(frozen=True)
class ActPolicyRegistry:
    path: Path
    version: str
    policies: dict[str, dict[str, Any]] = field(repr=False)

    def matches(self, name: str) -> list[dict[str, Any]]:
        """Every policy whose object class, label or alias equals `name` (case-insensitive)."""
        wanted = name.strip().casefold()
        return [
            policy
            for policy in self.policies.values()
            if wanted in {item.casefold() for item in (policy["object_class"], policy["label"], *policy["aliases"])}
        ]

    def resolve(self, name: str, arm_id: str | None = None) -> dict[str, Any] | None:
        """The policy for `name`; with `arm_id`, the one trained on that arm, else one usable on any arm."""
        found = self.matches(name)
        if arm_id is not None:
            return next((policy for policy in found if policy["arm_id"] == arm_id), None) or next(
                (policy for policy in found if policy["arm_id"] is None), None
            )
        return next((policy for policy in found if policy["arm_id"] is None), None) or (found[0] if found else None)

    def summary(self) -> list[dict[str, Any]]:
        return [
            {key: policy[key] for key in ("policy_id", "object_class", "label", "aliases", "model", "max_run_s", "arm_id")}
            for policy in self.policies.values()
        ]


def _text(policy: dict[str, Any], key: str, maximum: int = 80) -> str:
    value = policy.get(key)
    if not isinstance(value, str) or not value.strip() or len(value) > maximum:
        raise ActPolicyError(f"{policy.get('policy_id')!r}.{key} must be text of 1 to {maximum} characters")
    return value.strip()


def _policy(raw: object) -> dict[str, Any]:
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
    max_run_s = raw.get("max_run_s")
    if isinstance(max_run_s, bool) or not isinstance(max_run_s, int) or not 5 <= max_run_s <= 300:
        raise ActPolicyError(f"{policy_id!r}.max_run_s must be an integer from 5 to 300")
    return {
        "policy_id": policy_id,
        "object_class": _text(raw, "object_class"),
        "label": _text(raw, "label"),
        "aliases": [alias.strip() for alias in aliases],
        "model": model,
        "policy_type": "act",
        "task": _text(raw, "task", 200),
        "cameras": dict(cameras),
        "max_run_s": max_run_s,
        "arm_id": arm_id,
    }


def load_act_policies(path: str | Path | None = None) -> ActPolicyRegistry:
    policy_path = Path(path or os.environ.get("CONTROL_STATION_ACT_POLICIES") or DEFAULT_POLICIES_PATH)
    raw, document = load_json_file(policy_path, MAX_POLICIES_BYTES, "정책 정의 파일", ActPolicyError)
    if not isinstance(document, dict) or document.get("schema_version") != 1 or isinstance(document.get("schema_version"), bool):
        raise ActPolicyError("정책 정의 파일의 schema_version은 1이어야 합니다")
    if not isinstance(document.get("policies"), list) or not document["policies"]:
        raise ActPolicyError("policies must be a non-empty list")
    policies: dict[str, dict[str, Any]] = {}
    # name -> [(policy_id, arm_id)]: a name may repeat only across policies pinned to different arms.
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
