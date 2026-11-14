"""경로 처리 공통: 다른 출처 확인, JSON 본문, 집기 조건 검증"""

from __future__ import annotations

import json
from typing import TYPE_CHECKING, Any
from urllib.parse import urlsplit

from control_station.command_parser import MAX_SHELF_LEVEL
from control_station.json_files import unique_keys

if TYPE_CHECKING:
    from control_station.app import ControlStationHandler


# Errors that say "not now" rather than "not this": the same request can succeed once a Pinky or a receiving zone is
# free, or the Pinky has reached the zone. Everything else needs a different request (or a person) to change.
RETRYABLE_ERROR_CODES = frozenset({"ZONE_BUSY", "PINKY_UNAVAILABLE", "PICKUP_NOT_READY"})


def is_retryable(status: int, error_code: str) -> bool:
    """Whether sending the same request again later can succeed (the `retryable` field of every error answer)."""
    return status in {408, 429} or status >= 500 or error_code in RETRYABLE_ERROR_CODES


def is_cross_origin(handler: ControlStationHandler) -> bool:
    """다른 출처(브라우저) 요청인지"""
    origin = handler.headers.get("Origin")
    return bool(origin) and urlsplit(origin).netloc != handler.headers.get("Host")


def read_json_object(handler: ControlStationHandler) -> dict[str, Any] | None:
    """요청 본문을 JSON 객체로 읽기, 실패하면 오류 응답 후 None"""
    if handler.headers.get("Content-Type", "").split(";", 1)[0].strip() != "application/json":
        handler._json(415, {"error": "Content-Type must be application/json"})
        return None
    try:
        length = int(handler.headers.get("Content-Length", "0"))
    except ValueError:
        handler._json(400, {"error": "Invalid Content-Length"})
        return None
    if not 0 < length <= 4096:
        handler._json(400, {"error": "Request body must be between 1 and 4096 bytes"})
        return None
    try:
        body = json.loads(handler.rfile.read(length), object_pairs_hook=unique_keys)
    except (ValueError, UnicodeDecodeError, RecursionError):
        handler._json(400, {"error": "Request body must be valid JSON with unique object keys"})
        return None
    if not isinstance(body, dict):
        handler._json(400, {"error": "Request body must be a JSON object"})
        return None
    return body


def validate_pick_request(value: object) -> dict[str, object] | None:
    """집기 조건(종류, 색, 선반 층) 검증"""
    if value is None:
        return None
    if not isinstance(value, dict):
        raise ValueError("pick_request must be an object")
    allowed = {"category", "color", "shelf_level"}
    if set(value) - allowed:
        raise ValueError("pick_request supports only category, color, and shelf_level")
    category = value.get("category")
    if (
        not isinstance(category, str)
        or not category.strip()
        or len(category.strip()) > 80
        or any(ord(character) < 32 or ord(character) == 127 for character in category.strip())
    ):
        raise ValueError(
            "pick_request.category must contain 1 to 80 characters without control characters"
        )
    result: dict[str, object] = {"category": category.strip()}
    if "color" in value and value["color"] is not None:
        color = value["color"]
        if (
            not isinstance(color, str)
            or not color.strip()
            or len(color.strip()) > 40
            or any(ord(character) < 32 or ord(character) == 127 for character in color.strip())
        ):
            raise ValueError(
                "pick_request.color must contain 1 to 40 characters without control characters"
            )
        result["color"] = color.strip()
    if "shelf_level" in value and value["shelf_level"] is not None:
        shelf_level = value["shelf_level"]
        if (
            isinstance(shelf_level, bool)
            or not isinstance(shelf_level, int)
            or not 1 <= shelf_level <= MAX_SHELF_LEVEL
        ):
            raise ValueError(
                f"pick_request.shelf_level must be an integer from 1 to {MAX_SHELF_LEVEL}"
            )
        result["shelf_level"] = shelf_level
    return result


MAX_ORDER_KINDS = 5
MAX_UNITS_PER_KIND = 5


def validate_order_items(value: object, policies: Any) -> list[dict[str, object]]:
    """주문 물체 검증: [{object_class, quantity}] 1–5종, 종류별 1–5개, 학습 정책이 있는 물체만.

    이름(당근, carrot)은 정책 파일의 물체 클래스로 바꾸고, 같은 물체가 두 번 오면 수량을 합친다."""
    if policies is None:
        raise ValueError("정책 정의 파일이 없어 주문을 받을 수 없습니다")
    if not isinstance(value, list) or not 1 <= len(value) <= MAX_ORDER_KINDS:
        raise ValueError(f"order_items must be a list of 1 to {MAX_ORDER_KINDS} items")
    merged: dict[str, int] = {}
    for item in value:
        if not isinstance(item, dict) or set(item) != {"object_class", "quantity"}:
            raise ValueError("each order item must have exactly object_class and quantity")
        name, quantity = item["object_class"], item["quantity"]
        if not isinstance(name, str) or not name.strip() or len(name.strip()) > 80:
            raise ValueError("order_items[].object_class must contain 1 to 80 characters")
        if isinstance(quantity, bool) or not isinstance(quantity, int) or not 1 <= quantity <= MAX_UNITS_PER_KIND:
            raise ValueError(f"order_items[].quantity must be an integer from 1 to {MAX_UNITS_PER_KIND}")
        found = policies.matches(name)
        if not found:
            known = ", ".join(sorted({f"{policy['label']}({policy['object_class']})" for policy in policies.policies.values()}))
            raise ValueError(f"{name.strip()}에 학습된 정책이 없습니다. 주문할 수 있는 물체: {known}")
        object_class = found[0]["object_class"]
        merged[object_class] = merged.get(object_class, 0) + quantity
        if merged[object_class] > MAX_UNITS_PER_KIND:
            raise ValueError(f"{object_class}는 한 주문에 {MAX_UNITS_PER_KIND}개까지입니다")
    return [{"object_class": name, "quantity": quantity} for name, quantity in merged.items()]

