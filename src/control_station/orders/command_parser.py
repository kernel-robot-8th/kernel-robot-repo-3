"""사람이 검토할 초안을 만드는 보수적인 한국어/영어 요청 문장 해석기.

POST로 들어온 문장 하나(api/tasks.py의 post_parse_command)를 두 가지로 해석한다.
- parse_pick_command: 이전 형식 작업 폼의 초안(대상 종류, 색, 선반 층, 목적지).
- parse_order_text: 주문 폼의 초안(학습 정책이 있는 물체와 수량).

원칙: 작은 문법만 받고, 값이 둘 이상 맞서면 하나를 고르지 않고 비워 두며 경고를 단다. 부정·제외 표현("사과 말고")
이 붙은 값도 채우지 않는다. 결과는 언제나 초안이라 작업을 만들지도 팔 명령을 보내지도 않는다
(task_created=False, arm_command_sent=False). 운영자가 확인한 뒤 폼으로 보낸다.
"""

from __future__ import annotations

import re
from typing import Any

MAX_SHELF_LEVEL = 999_999_999  # 선반 층 번호 상한(api/common.validate_pick_request와 같은 입력 검증 범위)


# 표준 색 이름 → 문장에서 찾을 별칭.
_COLORS = {
    "red": ("빨간색", "빨간", "빨강", "red"),
    "blue": ("파란색", "파란", "파랑", "blue"),
    "green": ("초록색", "초록", "green"),
    "yellow": ("노란색", "노란", "yellow"),
    "black": ("검은색", "검정색", "검정", "black"),
    "white": ("흰색", "하얀색", "white"),
}
# 표준 대상 종류 → 별칭. detector 클래스 목록이 없을 때 쓰는 고정 어휘다.
_CATEGORIES = {
    "box": ("상자", "박스", "box", "carton"),
    "bottle": ("병", "물병", "bottle"),
}
# 공백을 뺀 한국어 서수 → 숫자(선반·층 번호용).
_KOREAN_ORDINALS = {
    "첫번째": 1,
    "첫째": 1,
    "두번째": 2,
    "둘째": 2,
    "두째": 2,
    "세번째": 3,
    "셋째": 3,
    "네번째": 4,
    "넷째": 4,
    "다섯번째": 5,
    "다섯째": 5,
    "여섯번째": 6,
    "여섯째": 6,
    "일곱번째": 7,
    "일곱째": 7,
    "여덟번째": 8,
    "여덟째": 8,
    "아홉번째": 9,
    "아홉째": 9,
    "열번째": 10,
    "열째": 10,
}
# 서수 정규식: 첫 번째/첫째 … 열 번째/열째, 사이 띄어쓰기 허용.
_KOREAN_ORDINAL_PATTERN = (
    r"(?:첫(?:\s*번째|\s*째)|(?:두|둘)(?:\s*번째|\s*째)|"
    r"(?:세|셋)(?:\s*번째|\s*째)|(?:네|넷)(?:\s*번째|\s*째)|"
    r"다섯(?:\s*번째|\s*째)|여섯(?:\s*번째|\s*째)|"
    r"일곱(?:\s*번째|\s*째)|여덟(?:\s*번째|\s*째)|"
    r"아홉(?:\s*번째|\s*째)|열(?:\s*번째|\s*째))"
)
# 동작 뒤 목적지: "집어서 <목적지>에 넣어", "가져와서 <목적지>로 옮겨".
_DESTINATION_PATTERN = re.compile(
    r"(?:집어(?:서)?|찾아(?:서)?|가져(?:와서|서)?|옮겨(?:서)?|집은\s*뒤)\s*"
    r"(?P<destination>.+?)\s*(?:안에|으로|에|로)\s*(?:넣|놓|배치|옮기)",
    re.IGNORECASE,
)
# 물체가 먼저 오는 꼴: "<물체>를 <목적지>에 놓아". 위 꼴로 못 찾았을 때만 쓴다.
_OBJECT_FIRST_DESTINATION_PATTERN = re.compile(
    r".+?(?:을|를)\s*(?P<destination>.+?)\s*(?:안에|으로|에|로)\s*"
    r"(?:넣|놓|배치|옮기|이동)",
    re.IGNORECASE,
)
# 대조·선택 표현(아닌, 말고, 또는, or…). 목적지·대상 표현 안에 있으면 확정하지 않는다.
_DESTINATION_UNCERTAINTY_PATTERN = re.compile(
    r"(?:아닌|아니고|아니라|말고|제외(?:하고)?|빼고|또는|혹은|아니면|"
    r"(?<![A-Za-z0-9])(?:or|either)(?![A-Za-z0-9]))",
    re.IGNORECASE,
)
# 열린 어휘(YOLO-World) 대상: "<대상>을/를 집어/찾아/가져/옮겨"의 <대상>.
_OPEN_VOCABULARY_TARGET_PATTERN = re.compile(
    r"(?P<target>[^,;.!?]+?)(?:을|를)\s*(?=(?:집어|찾아|가져|옮겨))"
)
# 대상이 여러 개라는 표현(2개, 두 개, 여러, 모든, -들 …).
_OPEN_VOCABULARY_MULTIPLE_PATTERN = re.compile(
    r"(?:[2-9]\d*|1\d+|두|세|네|다섯|여섯|일곱|여덟|아홉|열|몇)\s*개(?:의)?|"
    r"여러(?:\s*개)?|여럿|복수(?:의)?|모든|전부|각각|들$",
    re.IGNORECASE,
)
# 대상 여럿을 잇는 표현(쉼표, &, 그리고, 또는, 와/과 …).
_OPEN_VOCABULARY_CONJUNCTION_PATTERN = re.compile(
    r"(?:,|&|(?:^|\s)(?:그리고|또는|혹은|및|or)(?:\s|$)|[가-힣]+(?:와|과)\s+[가-힣])",
    re.IGNORECASE,
)
# 대상이 될 수 없는 막연한 말(물건, 그거 …). 이것만 남으면 대상을 뽑지 않는다.
_GENERIC_TARGET_PHRASES = {"물건", "물체", "대상", "것", "그것", "이것", "저것", "거", "그거", "이거", "저거"}
_GENERIC_TARGET_HEAD_PATTERN = re.compile(
    r"(?:물건|물체|대상|것|그것|이것|저것|거|그거|이거|저거)(?:들)?$"
)


def _contains_term(text: str, term: str) -> bool:
    """문장에 term이 낱말로 들어 있는지. 영숫자 term은 앞뒤가 영숫자가 아니어야 하고, 한글 term은 뒤에 조사 하나가
    붙어도 되지만 다른 한글 글자에 붙어 있으면 안 된다(예: '병'이 '물병' 안에서 잡히지 않게)."""
    if term.isascii() and term.replace(" ", "").isalnum():
        return re.search(rf"(?<![A-Za-z0-9]){re.escape(term)}(?![A-Za-z0-9])", text, re.I) is not None
    particles = r"(?:에서|으로|에게|부터|까지|이|가|은|는|을|를|의|에|로|와|과|도|들)?"
    return re.search(
        rf"(?<![가-힣]){re.escape(term)}{particles}(?![가-힣])", text
    ) is not None


def _term_is_negated(text: str, term: str) -> bool:
    """term에 부정·제외 표현이 붙었는지: 영어는 "not/no/without/except X"나 "X 아닌…", 한글은 "X(은/는/…) 말고/빼고/
    아닌…"."""
    markers = r"(?:아닌|아니고|아니라|말고|제외(?:하고)?|빼고)"
    if term.isascii() and term.replace(" ", "").isalnum():
        escaped = re.escape(term)
        return re.search(
            rf"(?<![A-Za-z0-9])(?:not|no|without|except)\s+{escaped}(?![A-Za-z0-9])",
            text,
            re.I,
        ) is not None or re.search(
            rf"(?<![A-Za-z0-9]){escaped}(?![A-Za-z0-9])\s+{markers}(?![가-힣])",
            text,
            re.I,
        ) is not None
    return re.search(
        rf"(?<![가-힣]){re.escape(term)}(?:은|는|이|가|을|를)?\s*{markers}(?![가-힣])",
        text,
    ) is not None


def _has_multiple_target_marker(text: str, terms: list[str]) -> bool:
    """대상 앞(32자)이나 뒤(24자)에 수량(2개, 두 개)이나 복수 표현(여러, 모든, -들)이 있는지. 지금 초안은 물체
    한 개만 지원하므로 있으면 대상을 비운다. "두 번째"처럼 서수인 것은 수량으로 보지 않는다."""
    quantity = r"(?:[2-9]\d*|1\d+|두|세|네|다섯|여섯|일곱|여덟|아홉|열|몇)\s*개(?:의)?"
    bare_korean_count = r"(?:두|세|네|다섯|여섯|일곱|여덟|아홉|열)(?!\s*번째)"
    multiple_word = r"(?:여러(?:\s*개)?|여럿|복수(?:의)?|모든|전부|각각)"
    count_before_target = re.compile(
        rf"(?:{quantity}|{bare_korean_count}|{multiple_word})\s*(?:[가-힣]+\s*){{0,3}}$"
    )
    count_after_target = re.compile(
        rf"^\s*(?:은|는|이|가|을|를)?\s*(?:{quantity}|{multiple_word})"
        rf"(?:씩|은|는|이|가|을|를)?"
    )
    for term in terms:
        if not term:
            continue
        expression = (
            rf"(?<![A-Za-z0-9]){re.escape(term)}(?![A-Za-z0-9])"
            if term.isascii() and term.replace(" ", "").isalnum()
            else re.escape(term)
        )
        for match in re.finditer(expression, text, re.IGNORECASE):
            before = text[max(0, match.start() - 32) : match.start()]
            after = text[match.end() : match.end() + 24]
            plural_suffix = re.match(r"들(?:은|는|이|가|을|를|에|로|도|만)?", after)
            if (
                count_before_target.search(before)
                or count_after_target.match(after)
                or plural_suffix
            ):
                return True
    return False


def _open_vocabulary_target_phrase(text: str) -> str | None:
    """사람이 검토할 YOLO-World 프롬프트용으로, 문장에서 분명한 한국어 대상 표현 하나를 뽑는다.

    "X를 집어"가 정확히 한 번 나와야 하고, 앞의 선반·층 표현과 색 이름은 떼어 낸다. 남은 말이 비었거나, 80자를 넘거나,
    막연한 말이거나, 대조·복수·접속 표현이나 제어 문자가 있으면 None이다."""
    matches = list(_OPEN_VOCABULARY_TARGET_PATTERN.finditer(text))
    if len(matches) != 1:
        return None
    phrase = " ".join(matches[0].group("target").split())
    shelf_prefix = re.compile(
        rf"^(?:(?:\d+\s*(?:번째\s*)?선반)|(?:{_KOREAN_ORDINAL_PATTERN}\s*선반)|"
        rf"(?:\d+\s*층)|(?:{_KOREAN_ORDINAL_PATTERN}\s*층))"
        r"\s*(?:에\s*있는|에\s*놓인|의|에서|에)?\s*"
    )
    phrase = shelf_prefix.sub("", phrase, count=1)
    phrase = re.sub(r"^.+?에\s*있는\s+", "", phrase)
    for aliases in _COLORS.values():
        for alias in sorted(aliases, key=len, reverse=True):
            if alias.isascii():
                phrase = re.sub(
                    rf"(?<![A-Za-z0-9]){re.escape(alias)}(?![A-Za-z0-9])",
                    " ",
                    phrase,
                    flags=re.IGNORECASE,
                )
            else:
                phrase = re.sub(re.escape(alias), " ", phrase)
    phrase = " ".join(phrase.strip(" .,:;!?\"'“”‘’").split())
    if (
        not phrase
        or len(phrase) > 80
        or phrase.casefold() in _GENERIC_TARGET_PHRASES
        or _GENERIC_TARGET_HEAD_PATTERN.search(phrase)
        or _DESTINATION_UNCERTAINTY_PATTERN.search(phrase)
        or _OPEN_VOCABULARY_MULTIPLE_PATTERN.search(phrase)
        or _OPEN_VOCABULARY_CONJUNCTION_PATTERN.search(phrase)
        or any(ord(character) < 32 or ord(character) == 127 for character in phrase)
    ):
        return None
    return phrase


def parse_pick_command(
    text: str,
    known_classes: list[str] | None = None,
    *,
    allow_open_vocabulary: bool = False,
) -> dict[str, Any]:
    """일부러 작게 잡은 문법으로 해석한다. 맞서는 값 중 하나를 고르는 일은 절대 하지 않는다.

    known_classes는 지금 검출기가 아는 클래스 이름이고, allow_open_vocabulary는 검출기가 YOLO-World(열린 어휘)라
    고정 어휘 밖의 대상도 프롬프트 초안으로 받을 수 있을 때 True다. 돌려주는 dict의 parsed.pick_request는
    대상 종류가 정해졌을 때만 채우고, field_validation.category가 그 판단 근거(MATCHED_DETECTOR_CLASS,
    AMBIGUOUS_DETECTOR_CLASS, NEGATED_CATEGORY, MULTIPLE_TARGETS_UNSUPPORTED …)를 말한다."""
    normalized = " ".join(text.split())
    normalized_classes: dict[str, str] = {}
    for label in known_classes or []:
        if isinstance(label, str) and label.strip():
            normalized_classes.setdefault(label.strip().casefold(), label.strip())
    # 대소문자만 다른 클래스 이름은 하나로 본다.
    known_classes = list(normalized_classes.values())
    warnings: list[str] = []
    ambiguous_fields: list[str] = []
    category_negated = any(
        _term_is_negated(normalized, alias)
        for aliases in _CATEGORIES.values()
        for alias in aliases
    ) or any(_term_is_negated(normalized, label) for label in known_classes)

    # ---- 1) 대상 종류: 검출기 클래스에 맞추고, 안 되면 고정 어휘나 열린 어휘 초안으로 ----
    category_matches = [
        canonical
        for canonical, aliases in _CATEGORIES.items()
        if any(_contains_term(normalized, alias) for alias in aliases)
    ]
    if known_classes:
        available = {label.casefold(): label for label in known_classes}
        matched_classes = [
            label for label in available.values() if _contains_term(normalized, label)
        ]
        if not matched_classes:
            # 클래스 이름이 문장에 직접 없으면, 문장의 종류 별칭("상자")이 들어 있는 클래스 이름을 찾는다.
            for canonical in category_matches:
                aliases = _CATEGORIES[canonical]
                candidates = [
                    label for folded, label in available.items()
                    if any(_contains_term(folded, alias.casefold()) for alias in aliases if alias.isascii())
                    or any(_contains_term(label, alias) for alias in aliases if not alias.isascii())
                ]
                matched_classes.extend(candidates)
        matched_classes = list(dict.fromkeys(matched_classes))
        if len(matched_classes) == 1:
            if len(matched_classes[0]) <= 80:
                category: str | None = matched_classes[0]
                category_validation = "MATCHED_DETECTOR_CLASS"
            else:
                category = None
                category_validation = "DETECTOR_CLASS_LABEL_TOO_LONG"
                warnings.append("detector 클래스 이름이 작업 입력 허용 길이를 초과합니다.")
        elif len(matched_classes) > 1:
            category = None
            category_validation = "AMBIGUOUS_DETECTOR_CLASS"
            ambiguous_fields.append("category")
        elif allow_open_vocabulary and len(category_matches) == 1:
            category = category_matches[0]
            category_validation = "REQUEST_PROMPT_REVIEW_REQUIRED"
            warnings.append(
                "요청 대상이 YOLO-World 고정 vocabulary에 없어 one-shot class prompt 초안을 만들었습니다. "
                "대상 종류를 확인·수정한 뒤 작업을 등록하세요."
            )
        elif allow_open_vocabulary and not category_matches:
            category = _open_vocabulary_target_phrase(normalized)
            if category is not None:
                category_validation = "REQUEST_PROMPT_REVIEW_REQUIRED"
                warnings.append(
                    "문장의 단일 대상 표현을 YOLO-World one-shot class prompt 초안으로 추출했습니다. "
                    "대상 종류를 확인·수정한 뒤 작업을 등록하세요."
                )
            elif any(_contains_term(normalized, label) for label in available.values()):
                category = None
                category_validation = "NO_DETECTOR_CLASS_MATCH"
                warnings.append("대상 표현과 일치하는 detector 클래스가 없습니다.")
            else:
                category = None
                category_validation = "NOT_RECOGNIZED"
        elif category_matches or any(_contains_term(normalized, label) for label in available.values()):
            category = None
            category_validation = "NO_DETECTOR_CLASS_MATCH"
            warnings.append("대상 표현과 일치하는 detector 클래스가 없습니다.")
        else:
            category = None
            category_validation = "NOT_RECOGNIZED"
    elif len(category_matches) == 1:
        category = category_matches[0]
        category_validation = "MODEL_CLASSES_UNAVAILABLE"
        warnings.append("detector 클래스 목록을 확인하지 못해 대상 클래스 일치는 미검증입니다.")
    elif len(category_matches) > 1:
        category = None
        category_validation = "AMBIGUOUS_CATEGORY"
        ambiguous_fields.append("category")
    else:
        category = None
        category_validation = "NOT_RECOGNIZED"

    if category_negated:
        category = None
        category_validation = "NEGATED_CATEGORY"
        if "category" not in ambiguous_fields:
            ambiguous_fields.append("category")
        warnings.append("대상에 부정·제외 표현이 있어 종류를 자동으로 채우지 않았습니다.")

    # ---- 2) 대상이 여러 개인지: 지금은 한 개만 지원한다 ----
    target_terms = [
        alias
        for canonical in category_matches
        for alias in _CATEGORIES[canonical]
    ]
    if category is not None:
        target_terms.append(category)
    if category is not None and _has_multiple_target_marker(normalized, target_terms):
        category = None
        category_validation = "MULTIPLE_TARGETS_UNSUPPORTED"
        if "category" not in ambiguous_fields:
            ambiguous_fields.append("category")
        warnings.append(
            "현재 작업 초안은 물체 한 개만 지원합니다. 복수·수량 표현을 제거하거나 한 개씩 지정하세요."
        )

    # ---- 3) 색: 검토용 힌트일 뿐 후보 필터·선택에는 쓰지 않는다 ----
    color_matches = [
        canonical
        for canonical, aliases in _COLORS.items()
        if any(_contains_term(normalized, alias) for alias in aliases)
    ]
    if len(color_matches) == 1:
        color: str | None = color_matches[0]
    elif len(color_matches) > 1:
        color = None
        ambiguous_fields.append("color")
    else:
        color = None
    color_negated = any(
        _term_is_negated(normalized, alias)
        for aliases in _COLORS.values()
        for alias in aliases
    )
    if color_negated:
        color = None
        if "color" not in ambiguous_fields:
            ambiguous_fields.append("color")
        warnings.append("색상에 부정·제외 표현이 있어 색상을 자동으로 채우지 않았습니다.")

    # ---- 4) 선반 층: "2번째 선반", "셋째 층" 등. 이것도 검토용 비교에만 쓴다 ----
    level_matches: list[int] = []
    shelf_level_negated = False
    shelf_pattern = re.compile(
        rf"(?<!\d)(?P<shelf>\d+)\s*(?:번째\s*)?선반|"
        rf"(?P<shelf_word>{_KOREAN_ORDINAL_PATTERN})\s*선반|"
        rf"(?<!\d)(?P<floor>\d+)\s*층|"
        rf"(?P<floor_word>{_KOREAN_ORDINAL_PATTERN})\s*층"
    )
    negation_suffix = r"(?:은|는|이|가|을|를)?\s*(?:아닌|아니고|아니라|말고|제외(?:하고)?|빼고)(?![가-힣])"
    for match in shelf_pattern.finditer(normalized):
        numeric_level = match.group("shelf") or match.group("floor")
        if numeric_level is not None:
            level = int(numeric_level)
        else:
            ordinal = (match.group("shelf_word") or match.group("floor_word") or "").replace(" ", "")
            level = _KOREAN_ORDINALS.get(ordinal)
            if level is None:
                continue
        level_matches.append(level)
        if re.match(negation_suffix, normalized[match.end() :]):
            shelf_level_negated = True
    level_matches = list(dict.fromkeys(level_matches))
    if len(level_matches) == 1 and 1 <= level_matches[0] <= MAX_SHELF_LEVEL:
        shelf_level: int | None = level_matches[0]
    elif len(level_matches) > 1:
        shelf_level = None
        ambiguous_fields.append("shelf_level")
    else:
        shelf_level = None
        if level_matches:
            warnings.append(
                f"선반 층은 1부터 {MAX_SHELF_LEVEL}까지의 정수여야 합니다."
            )
    if shelf_level_negated:
        shelf_level = None
        if "shelf_level" not in ambiguous_fields:
            ambiguous_fields.append("shelf_level")
        warnings.append("선반 층에 부정·제외 표현이 있어 층을 자동으로 채우지 않았습니다.")

    # ---- 5) 목적지: 하나만 나와야 하고 40자 이하. "집어서"의 "서"가 딸려 오면 뗀다 ----
    destination_matches = [
        " ".join(match.group("destination").strip().split()).removeprefix("서 ").strip()
        for match in _DESTINATION_PATTERN.finditer(normalized)
    ]
    if not destination_matches:
        destination_matches = [
            " ".join(match.group("destination").strip().split()).removeprefix("서 ").strip()
            for match in _OBJECT_FIRST_DESTINATION_PATTERN.finditer(normalized)
        ]
    destination_matches = [value for value in destination_matches if value]
    uncertain_destination = any(
        _DESTINATION_UNCERTAINTY_PATTERN.search(value) for value in destination_matches
    )
    if uncertain_destination:
        destination = None
        ambiguous_fields.append("destination")
        warnings.append("목적지에 대조·선택 표현이 있어 목적지를 자동으로 채우지 않았습니다.")
    elif len(destination_matches) == 1:
        destination: str | None = destination_matches[0]
        if len(destination) > 40:
            destination = None
            warnings.append("목적지 표현이 작업 입력 허용 길이(40자)를 초과합니다.")
    elif len(destination_matches) > 1:
        destination = None
        ambiguous_fields.append("destination")
    else:
        destination = None

    # ---- 6) 빠진 값과 경고를 모아 초안으로 돌려준다 ----
    missing_fields = [
        name for name, value in (("category", category), ("destination", destination)) if value is None
    ]
    if category is None and category_validation == "NOT_RECOGNIZED":
        warnings.append(
            "단일 한국어 대상 표현을 추출하지 못했습니다. 대상 종류를 폼에서 확인하세요."
            if allow_open_vocabulary
            else "지원하는 상자/병 표현으로 대상을 특정하지 못했습니다."
        )
    if destination is None and "destination" not in ambiguous_fields:
        warnings.append("목적지를 특정하지 못했습니다.")
    if ambiguous_fields:
        warnings.append("여러 후보가 있어 해당 필드는 비워 두었습니다.")
    if color is not None or shelf_level is not None:
        warnings.append(
            "색상은 미보정 힌트 비교만 검토용으로 표시하며 후보 필터·선택에는 쓰지 않습니다. "
            "선반 층은 활성 ROI 초안이 일치할 때만 검토용 비교를 표시하며 후보 필터·선택에는 쓰지 않습니다."
        )

    pick_request: dict[str, object] | None = None
    if category is not None:
        pick_request = {"category": category}
        if color is not None:
            pick_request["color"] = color
        if shelf_level is not None:
            pick_request["shelf_level"] = shelf_level

    return {
        "schema_version": 1,
        "parser": "RULE_BASED_DRAFT",
        "review_required": True,
        "task_created": False,
        "arm_command_sent": False,
        "parsed": {
            "pick_request": pick_request,
            "destination": destination,
            "arm_job": f"집기 후 {destination}에 배치" if category and destination else None,
        },
        "field_validation": {"category": category_validation},
        "ambiguous_fields": ambiguous_fields,
        "missing_fields": missing_fields,
        "warnings": warnings,
    }


# 수량을 말로 할 때: 한/하나 … 다섯(한 주문에 종류별 5개가 상한이다).
_ORDER_COUNT_WORDS = {"한": 1, "하나": 1, "두": 2, "둘": 2, "세": 3, "셋": 3, "석": 3, "네": 4, "넷": 4, "다섯": 5}
# 물체 이름 바로 뒤의 수량: "당근 2개", "당근 두 개", "사과 하나", "carrot x2", "당근2".
# 수량은 단위(개)로 끝나거나, 아니면 문장 끝·문장 부호·잇는 조사(랑, 하고, 와…)로 끝나야 한다. 그래서
# "한테"나 "두고"를 수량으로 읽지 않는다.
_ORDER_COUNT_PATTERN = re.compile(
    r"\s*(?:x\s*)?(?:(?P<digits>\d{1,2})|(?P<word>다섯|하나|한|두|둘|세|셋|석|네|넷))\s*"
    r"(?:개|ea|pcs?|(?=$|[\s,.!?·]|이랑|랑|하고|와|과|도|만|씩|을|를|이|가|은|는|으로|로|에))",
    re.I,
)
ORDER_PARSER = "RULE_BASED_ORDER_DRAFT"  # 결과의 parser 값: 규칙 기반 주문 초안


def parse_order_text(text: str, policies: Any) -> dict[str, Any]:
    """주문 문장 → 주문 물체 초안. 학습 정책의 이름(당근, 물티슈, 사과…)을 어휘로 쓴다.

    "당근 1개랑 사과 두 개" -> [{carrot, 1}, {apple, 2}]. 수량 없는 이름은 한 개다. 이름은 긴 것부터 맞춰서
    물티슈 안의 티슈를 두 번 세지 않는다. 부정된 이름(사과 말고)은 경고와 함께 뺀다. 같은 물체가 여러 번 나오면
    수량을 더한다. 종류별 5개를 넘으면 경고만 하고 넣어 둔다(접수할 때 validate_order_items가 거절한다).
    결과는 운영자가 검토할 초안이고 작업을 만들지 않는다. policies는 정책 레지스트리(없으면 빈 초안).
    """
    normalized = " ".join(text.split())
    warnings: list[str] = []
    totals: dict[str, int] = {}
    if policies is None:
        return {"parser": ORDER_PARSER, "review_required": True, "task_created": False, "order_items": [],
                "warnings": ["정책 정의 파일이 없어 주문 물체를 알 수 없습니다."]}
    vocabulary = sorted(
        {
            (name, policy["object_class"])
            for policy in policies.policies.values()
            for name in (policy["label"], policy["object_class"], *policy["aliases"])
        },
        key=lambda pair: -len(pair[0]),
    )
    taken: list[tuple[int, int]] = []  # 이미 어떤 이름이 차지한 문장 구간(부정된 이름도 차지한다)
    hits: list[tuple[int, str, int]] = []  # (문장 속 위치, 물체 클래스, 수량)
    for name, object_class in vocabulary:
        pattern = (
            rf"(?<![A-Za-z0-9]){re.escape(name)}(?![A-Za-z0-9])"
            if name.isascii()
            else rf"(?<![가-힣]){re.escape(name)}"
        )
        for match in re.finditer(pattern, normalized, re.I):
            start, end = match.span()
            # 더 긴 이름이 이미 차지한 자리면 건너뛴다(물티슈 안의 티슈).
            if any(start < other_end and other_start < end for other_start, other_end in taken):
                continue
            taken.append((start, end))
            if _term_is_negated(normalized[start:], name):
                warnings.append(f"'{name}'에 부정·제외 표현이 있어 주문에 넣지 않았습니다.")
                continue
            rest = normalized[end:]
            # "당근을 2개"처럼 이름과 수량 사이에 조사 하나는 허용한다.
            particle = re.match(r"(?:을|를|이|가|은|는|도)?", rest)
            count = _ORDER_COUNT_PATTERN.match(rest[particle.end() if particle else 0:])
            quantity = 1
            if count is not None:
                quantity = int(count.group("digits")) if count.group("digits") else _ORDER_COUNT_WORDS[count.group("word")]
            if quantity < 1:
                warnings.append(f"'{name}' 수량이 0이라 넣지 않았습니다.")
                continue
            hits.append((start, object_class, quantity))
    for _, object_class, quantity in sorted(hits):  # 운영자가 말한 순서를 지킨다
        totals[object_class] = totals.get(object_class, 0) + quantity
    if not totals and not warnings:
        warnings.append("주문할 수 있는 물체 이름을 찾지 못했습니다.")
    for object_class, quantity in totals.items():
        if quantity > 5:
            warnings.append(f"{object_class} {quantity}개: 한 주문에 종류별 5개까지만 받습니다.")
    return {
        "parser": ORDER_PARSER,
        "review_required": True,
        "task_created": False,
        "order_items": [{"object_class": name, "quantity": quantity} for name, quantity in totals.items()],
        "warnings": warnings,
    }

