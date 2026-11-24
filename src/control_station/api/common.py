"""경로 처리 공통: 다른 출처 확인, JSON 본문 읽기, 오류의 재시도 가능 여부, 집기 조건·주문 물체 검증.

app.py의 ControlStationHandler(do_POST, _json_error)와 api/ 아래 처리 함수들이 함께 쓴다.

관제에는 인증이 없다. 대신 브라우저가 다른 사이트에서 이 서버로 요청을 보내는 일(CSRF)을 막으려고, 상태를
바꾸는 POST는 Origin 머리글이 이 서버와 다르면 거부한다(is_cross_origin). Origin을 보내지 않는 팔 워커 같은
일반 HTTP 클라이언트는 이 검사에 걸리지 않는다.
"""

from __future__ import annotations

import json
from typing import TYPE_CHECKING, Any
from urllib.parse import urlsplit

from control_station.orders.command_parser import MAX_SHELF_LEVEL
from control_station.storage.json_files import unique_keys

if TYPE_CHECKING:
    from control_station.app import ControlStationHandler


# "이 요청은 안 된다"가 아니라 "지금은 안 된다"는 오류들이다. 핑키나 수령 구역이 비거나 핑키가 수령 구역에
# 도착하면 같은 요청이 성공할 수 있다. 그 밖의 오류는 요청을 바꾸거나 사람이 무언가를 고쳐야 한다.
RETRYABLE_ERROR_CODES = frozenset({"ZONE_BUSY", "PINKY_UNAVAILABLE", "PICKUP_NOT_READY"})


def is_retryable(status: int, error_code: str) -> bool:
    """같은 요청을 나중에 다시 보내면 성공할 수 있는지 판단한다 (모든 오류 응답의 `retryable` 필드).

    408(시간 초과)·429(요청 과다)·5xx(서버 쪽 문제)는 상태 코드만으로, 그 밖에는 위 오류 코드 목록으로 정한다.
    """
    return status in {408, 429} or status >= 500 or error_code in RETRYABLE_ERROR_CODES


def is_cross_origin(handler: ControlStationHandler) -> bool:
    """다른 출처(브라우저) 요청인지 판단한다.

    Origin 머리글의 host:port가 Host 머리글과 다를 때만 True다. Origin이 없으면(팔 워커, curl 같은 일반 HTTP
    클라이언트, 같은 출처 GET) 다른 출처로 보지 않는다.
    """
    origin = handler.headers.get("Origin")
    return bool(origin) and urlsplit(origin).netloc != handler.headers.get("Host")


def read_json_object(handler: ControlStationHandler) -> dict[str, Any] | None:
    """요청 본문을 JSON 객체로 읽는다. 실패하면 오류 응답(415/400)을 보낸 뒤 None을 돌려준다.

    Content-Type이 application/json이어야 한다. 브라우저의 단순 form 전송(text/plain 등)은 CORS 사전 확인 없이
    다른 사이트에서도 보낼 수 있으므로, 이 검사가 다른 출처 차단을 한 번 더 받쳐 준다.
    """
    if handler.headers.get("Content-Type", "").split(";", 1)[0].strip() != "application/json":
        handler._json(415, {"error": "Content-Type must be application/json"})
        return None
    try:
        length = int(handler.headers.get("Content-Length", "0"))
    except ValueError:
        handler._json(400, {"error": "Invalid Content-Length"})
        return None
    # 이 API의 본문은 모두 작다(명령 한 건, 주문 한 건). 4 KiB 상한으로 큰 본문이 요청 스레드를 붙잡지 못하게 한다.
    if not 0 < length <= 4096:
        handler._json(400, {"error": "Request body must be between 1 and 4096 bytes"})
        return None
    try:
        # unique_keys: 같은 키가 두 번 나오면 거부한다. 뒤의 값이 조용히 앞의 값을 덮는 일을 막는다.
        # RecursionError: 아주 깊게 중첩된 JSON도 형식 오류로 처리한다.
        body = json.loads(handler.rfile.read(length), object_pairs_hook=unique_keys)
    except (ValueError, UnicodeDecodeError, RecursionError):
        handler._json(400, {"error": "Request body must be valid JSON with unique object keys"})
        return None
    if not isinstance(body, dict):
        handler._json(400, {"error": "Request body must be a JSON object"})
        return None
    return body


def validate_pick_request(value: object) -> dict[str, object] | None:
    """이전 형식 작업의 집기 조건(종류, 색, 선반 층)을 검증하고 앞뒤 공백을 뺀 사본을 돌려준다.

    없으면(None) None이다. category는 1–80자, color는 1–40자이고 제어 문자는 받지 않는다. shelf_level은
    1–MAX_SHELF_LEVEL 정수다(bool은 int의 하위 타입이라 따로 거른다). 틀리면 ValueError를 낸다.
    """
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


# 한 주문의 상한: 물체 종류 5가지, 종류마다 5개까지.
MAX_ORDER_KINDS = 5
MAX_UNITS_PER_KIND = 5


def validate_order_items(value: object, policies: Any) -> list[dict[str, object]]:
    """주문 물체 검증: [{object_class, quantity}] 1–5종, 종류별 1–5개, 학습 정책이 있는 물체만 받는다.

    이름(당근, carrot)은 정책 파일의 물체 클래스로 바꾸고, 같은 물체가 두 번 오면 수량을 합친다.
    합친 뒤에도 종류별 상한(5개)을 넘으면 거부한다. 정책 파일이 없으면 주문 자체를 받지 않는다."""
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

