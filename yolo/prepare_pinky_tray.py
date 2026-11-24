"""촬영한 사진에 박스를 그리고 핑키 적재 전용 데이터셋을 만든다.

`capture_pinky_tray.py`로 배치(물건 구성)별 폴더에 찍어 둔 천장 카메라 사진을 골라, 사진마다 그 배치에 있는
물건과 핑키를 박스로 그려 YOLO 데이터셋(`images/`, `labels/`, `data.yaml`)을 만든다. 배치 이름이 곧 정답
구성이므로, 사진마다 각 클래스 박스가 정확히 하나씩 있어야 저장된다.

    python yolo/prepare_pinky_tray.py                      # 기본: 당근·물티슈·핑키·수박 데이터셋
    python yolo/prepare_pinky_tray.py --cargo-only         # 수박·당근·키위·사과·핑키 전용 데이터셋
    python yolo/prepare_pinky_tray.py --list               # 고를 사진만 보여 주고 라벨링하지 않음
    python yolo/prepare_pinky_tray.py --relabel carrot1 --split val   # 저장된 조합의 박스 다시 그리기
    python yolo/prepare_pinky_tray.py --cargo-only --append-layout apple1 --since <첫 사진 파일명>
    python yolo/prepare_pinky_tray.py --cargo-only --append-layout apple1 --since <첫 사진 파일명> --auto-weights <모델.pt>

`--append-layout`은 기존 분할을 그대로 두고 한 조합의 새 사진(`--since` 이후)을 학습용에 추가한다.
`--auto-weights`를 주면 기존 모델이 그린 박스 초안을 검토·수정하는 방식으로 라벨링한다.

조합마다 사진을 촬영 시각순으로 앞 3/4은 학습용, 뒤 1/4은 검증용으로 나누고, 각 구간에서 시간 간격이 고르게
`--per-layout`장(검증은 그 1/4)을 고른다. 이미 만든 라벨은 건너뛰므로 중간에 멈춰도 같은 명령으로 이어서 한다.
사진 파일만 다루며 카메라나 로봇과는 통신하지 않는다.
"""

import argparse
from datetime import datetime
from itertools import combinations
from pathlib import Path
import shutil

import cv2
import yaml


YOLO_DIR = Path(__file__).resolve().parent
# 기본 데이터셋의 클래스와 배치 이름 → 그 사진에 있는 클래스. 클래스 번호는 이 순서이므로 바꾸지 않는다.
CLASSES = ("carrot", "tissue", "pinky", "watermelon")
LAYOUTS = {
    "carrot1": ("carrot", "pinky"),
    "tissue1": ("tissue", "pinky"),
    "carrot1_tissue1": ("carrot", "tissue", "pinky"),
    "watermelon1_tissue1": ("watermelon", "tissue", "pinky"),
}

# --cargo-only 데이터셋: 물건 4종의 가능한 모든 조합(각 1개)과 빈 쟁반("empty")을 배치로 둔다.
# 예: "watermelon1_kiwi1" → ("watermelon", "kiwi", "pinky"). 핑키는 모든 사진에 있다.
CARGO = ("watermelon", "carrot", "kiwi", "apple")
CARGO_LAYOUTS = {"empty": ("pinky",)}
for size in range(1, len(CARGO) + 1):
    for cargo in combinations(CARGO, size):
        CARGO_LAYOUTS["_".join(f"{name}1" for name in cargo)] = (*cargo, "pinky")


def sample(images, limit):
    """촬영 시각 범위에서 고르게 최대 `limit`장을 고른다. 파일 이름은 `capture_pinky_tray.py`의 시각 형식이다.

    목표 시각마다 가장 가까운 사진을 고르므로, 같은 사진이 겹치면 `limit`보다 적게 돌려줄 수 있다.
    """
    if len(images) <= limit:
        return images
    if limit == 1:
        return [images[len(images) // 2]]
    # 촬영 시간 범위에서 균등하게 골라 연사 구간의 비중을 줄인다.
    times = [datetime.strptime(path.stem, "%Y%m%d_%H%M%S_%f") for path in images]
    duration = times[-1] - times[0]
    return sorted({
        images[min(range(len(times)), key=lambda i: abs(times[i] - target))]
        for target in (times[0] + duration * i / (limit - 1) for i in range(limit))
    })


def select_box(window, image, initial=None):
    """창에서 박스 하나를 드래그로 받는다. Enter/Space는 확정, C/Esc나 창 닫기는 None(중단)이다.

    박스는 (x, y, 너비, 높이) 픽셀 값이다.
    """
    start = None
    box = initial

    def drag(event, x, y, flags, _):
        nonlocal start, box
        if event == cv2.EVENT_LBUTTONDOWN:
            start = (x, y)
            box = None
        elif start is not None and event in (cv2.EVENT_MOUSEMOVE, cv2.EVENT_LBUTTONUP):
            x0, y0 = start
            box = (min(x, x0), min(y, y0), abs(x - x0), abs(y - y0))
            if event == cv2.EVENT_LBUTTONUP:
                start = None

    cv2.namedWindow(window, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(window, drag)
    try:
        while True:
            view = image.copy()
            if box:
                x, y, width, height = box
                cv2.rectangle(view, (x, y), (x + width, y + height), (0, 255, 255), 2)
            cv2.imshow(window, view)
            key = cv2.waitKey(30) & 0xFF
            if key in (ord("c"), ord("C"), 27) or cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                return None
            if key in (10, 13, 32):  # Enter(LF/CR)나 Space
                if box and box[2] and box[3]:
                    return box
                print("박스를 먼저 드래그하세요.", flush=True)
    finally:
        cv2.destroyAllWindows()


def propose_boxes(model, image, classes):
    """기존 모델로 자동 라벨 초안을 만든다. 클래스마다 신뢰도가 가장 높은 박스 하나만 남긴다.

    사람이 `review_proposals`에서 검토하므로 신뢰도 기준을 0.15로 낮게 두어 놓치는 박스를 줄인다.
    배치에 없는 클래스로 검출된 박스는 버린다.
    """
    result = model.predict(image, conf=0.15, imgsz=640, verbose=False)[0]
    height, width = image.shape[:2]
    proposals = {}
    for box in sorted(result.boxes, key=lambda item: float(item.conf.item()), reverse=True):
        name = result.names[int(box.cls.item())]
        if name not in classes or name in proposals:
            continue
        x1, y1, x2, y2 = (round(value) for value in box.xyxy[0].tolist())
        # 화면 밖으로 나간 좌표를 사진 안으로 자르고, (x, y, 너비, 높이) 형식으로 바꾼다.
        x1, x2 = max(0, x1), min(width, x2)
        y1, y2 = max(0, y1), min(height, y2)
        if x1 < x2 and y1 < y2:
            proposals[name] = (x1, y1, x2 - x1, y2 - y1)
    return proposals


def review_proposals(image, proposals, classes):
    """자동 라벨 초안을 보여 주고 고치게 한다. 모든 클래스에 박스가 있어야 Enter로 확정된다.

    숫자키로 클래스를 고르고 드래그로 그 클래스 박스를 다시 그리며, D/Backspace/Delete로 지운다.
    C/Esc나 창 닫기는 None(중단)이다. 노란 박스가 지금 고른 클래스, 아래 상태 줄의 `?`는 박스가 없는 클래스다.
    """
    window = "Auto labels | 1-5: select | D: delete | Enter: save | C: quit"
    selected = classes[0]
    height, width = image.shape[:2]
    start = None
    cursor = None

    def drag(event, x, y, flags, _):
        nonlocal start, cursor
        point = (min(max(x, 0), width - 1), min(max(y, 0), height - 1))  # 창 밖으로 끌어도 사진 안으로 자른다.
        if event == cv2.EVENT_LBUTTONDOWN:
            start = cursor = point
        elif start is not None and event == cv2.EVENT_MOUSEMOVE:
            cursor = point
        elif start is not None and event == cv2.EVENT_LBUTTONUP:
            x0, y0 = start
            x1, y1 = point
            if x0 != x1 and y0 != y1:
                proposals[selected] = (min(x0, x1), min(y0, y1), abs(x1 - x0), abs(y1 - y0))
            start = cursor = None

    cv2.namedWindow(window, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(window, drag)
    try:
        while True:
            view = image.copy()
            for number, name in enumerate(classes, 1):
                color = (0, 255, 255) if name == selected else (0, 220, 0)
                if name in proposals:
                    x, y, w, h = proposals[name]
                    cv2.rectangle(view, (x, y), (x + w, y + h), color, 2)
                    anchor = (x, max(16, y - 5))
                    caption = f"{number} {name}"
                    cv2.putText(view, caption, anchor, cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 0, 0), 3)
                    cv2.putText(view, caption, anchor, cv2.FONT_HERSHEY_SIMPLEX, .5, color, 1)
            if start is not None and cursor is not None:
                cv2.rectangle(view, start, cursor, (0, 255, 255), 2)
            status = "  ".join(
                f"{i}:{name}{'*' if name == selected else ''}{'?' if name not in proposals else ''}"
                for i, name in enumerate(classes, 1)
            )
            cv2.rectangle(view, (0, height - 24), (width, height), (0, 0, 0), -1)
            cv2.putText(view, status, (6, height - 7), cv2.FONT_HERSHEY_SIMPLEX, .42, (255, 255, 255), 1)
            cv2.imshow(window, view)
            key = cv2.waitKey(30) & 0xFF
            if key in (ord("c"), ord("C"), 27) or cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                return None
            if ord("1") <= key < ord("1") + len(classes):
                selected = classes[key - ord("1")]
                start = cursor = None
            if key in (ord("d"), ord("D"), 8, 127):  # D, Backspace, Delete
                proposals.pop(selected, None)
                start = cursor = None
            if key in (10, 13, 32):
                missing = [name for name in classes if name not in proposals]
                if not missing:
                    return proposals
                print(f"아직 박스가 없는 클래스: {', '.join(missing)}", flush=True)
    finally:
        cv2.destroyAllWindows()


def label_image(path, classes, names, model=None):
    """사진 한 장의 YOLO 라벨 문자열을 만든다. 사용자가 중단하면 None을 돌려준다.

    `model`이 있으면 자동 초안을 검토하고, 없으면 클래스마다 차례로 박스를 그린다. 클래스 번호는 데이터셋 전체의
    `names` 순서에서 찾는다.
    """
    image = cv2.imread(str(path))
    if image is None:
        raise SystemExit(f"사진을 읽지 못했습니다: {path}")
    height, width = image.shape[:2]
    # 큰 사진은 화면에 맞게 줄여 보여 준다. 라벨은 0~1 비율이라 축소한 사진 기준으로 계산해도 원본에 맞는다.
    scale = min(1.0, 1200 / width, 800 / height)
    preview = cv2.resize(image, (round(width * scale), round(height * scale)))
    height, width = preview.shape[:2]
    boxes = None
    if model is not None:
        boxes = review_proposals(preview, propose_boxes(model, preview, classes), classes)
        if boxes is None:
            return None
    rows = []
    for name in classes:
        window = f"Select {name} | {path.parent.name}"
        if boxes is None:
            print(f"  {name} 박스 확인. 맞으면 Enter, 틀리면 다시 드래그. C: 종료", flush=True)
            box = select_box(window, preview)
            if box is None:
                return None
        else:
            box = boxes[name]
        x, y, w, h = box
        rows.append(f"{names.index(name)} {(x + w / 2) / width:.6f} "
                    f"{(y + h / 2) / height:.6f} {w / width:.6f} {h / height:.6f}\n")
        cv2.rectangle(preview, (x, y), (x + w, y + h), (0, 220, 0), 2)  # 다음 클래스를 그릴 때 앞 박스를 보여 준다.
    return "".join(rows)


def review_labels(output, selection, split, layouts, names):
    """이미 저장한 조합(`all`이면 전부)의 사진을 다시 띄워 박스를 새로 그리고 라벨 파일을 덮어쓴다.

    분할을 지정하지 않으면 검증용(val)부터 본다. 중단하면 그 사진의 기존 라벨은 그대로 둔다.
    """
    selected_layouts = layouts if selection == "all" else (selection,)
    for folder in ((split,) if split else ("val", "train")):
        for layout in selected_layouts:
            # 파일 이름은 `<조합>_<시각>`이고 시각은 2로 시작한다. `_2`까지 맞춰야 "carrot1"이
            # "carrot1_tissue1_…" 사진까지 잡지 않는다.
            for path in sorted((output / "images" / folder).glob(f"{layout}_2*.jpg")):
                label = output / "labels" / folder / f"{path.stem}.txt"
                if not label.is_file():
                    raise SystemExit(f"기존 라벨을 찾을 수 없습니다: {label}")
                print(f"{folder}: {path.name}", flush=True)
                rows = label_image(path, layouts[layout], names)
                if rows is None:
                    print("현재 사진의 라벨은 유지됩니다.")
                    return
                label.write_text(rows, encoding="utf-8")
                print(f"수정 저장: {label.name}", flush=True)


def main():
    """인자를 검사하고 사진을 골라 차례로 라벨링한 뒤 `data.yaml`을 쓴다."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=None)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--cargo-only", action="store_true", help="수박·당근·키위·사과·핑키 전용 데이터셋")
    parser.add_argument("--per-layout", type=int, default=16, help="조합당 첫 라벨링 목표 장수")
    parser.add_argument("--list", action="store_true", help="사진 선택 결과만 표시")
    parser.add_argument("--relabel", help="저장된 조합의 박스를 다시 그리기")
    parser.add_argument("--split", choices=("train", "val"), help="다시 라벨링할 분할")
    parser.add_argument("--append-layout", help="기존 분할을 유지하고 한 조합의 새 사진을 학습용에 추가")
    parser.add_argument("--since", help="추가할 첫 사진의 파일명(확장자 제외)")
    parser.add_argument("--auto-weights", type=Path, help="검토할 자동 라벨 초안 가중치")
    args = parser.parse_args()
    if args.auto_weights and (not args.append_layout or not args.auto_weights.is_file()):
        parser.error("--auto-weights는 기존 가중치 파일과 --append-layout이 필요합니다.")
    if bool(args.append_layout) != bool(args.since) or (args.append_layout and args.relabel):
        parser.error("--append-layout과 --since를 함께 사용하세요. --relabel과 동시에 쓸 수 없습니다.")
    if args.per_layout < 4:  # 검증용이 per_layout // 4장이므로 4 이상이어야 검증 사진이 1장 이상 생긴다.
        parser.error("--per-layout은 4 이상이어야 합니다.")
    base = YOLO_DIR / "experiments"
    names = (*CARGO, "pinky") if args.cargo_only else CLASSES
    layouts = CARGO_LAYOUTS if args.cargo_only else LAYOUTS
    source = args.source or base / ("pinky_tray_watermelon_carrot_kiwi_apple" if args.cargo_only else "pinky_tray_eval")
    output = (args.output or base / ("pinky_cargo_dataset" if args.cargo_only else "pinky_tray_dataset")).resolve()
    config_path = output / "data.yaml"
    # 기존 데이터셋에 이어서 쓸 때 클래스 번호가 다르면 라벨 의미가 섞이므로 멈춘다.
    if config_path.is_file():
        current_names = yaml.safe_load(config_path.read_text())["names"]
        if current_names != dict(enumerate(names)):
            parser.error("기존 데이터셋의 클래스 번호가 다릅니다. 별도 출력 폴더를 사용하세요.")

    if args.relabel:
        if args.relabel != "all" and args.relabel not in layouts:
            parser.error(f"지원하지 않는 조합: {args.relabel}")
        review_labels(output, args.relabel, args.split, layouts, names)
        return
    if args.split:
        parser.error("--split은 --relabel과 함께 사용하세요.")

    selected = []
    if args.append_layout:
        if not config_path.is_file():
            parser.error("추가할 기존 데이터셋이 없습니다.")
        layout = args.append_layout
        if layout not in layouts:
            parser.error(f"지원하지 않는 조합: {layout}")
        images = sorted((source / layout).glob("*.jpg"))
        if not any(path.stem == args.since for path in images):
            parser.error(f"첫 사진을 찾을 수 없습니다: {args.since}")
        # 파일 이름이 촬영 시각이라 문자열 비교가 곧 시각 비교다. --since 사진부터 전부 학습용에 넣는다.
        selected = [("train", path, layouts[layout]) for path in images if path.stem >= args.since]
        print(f"{layout} 추가 대상: {len(selected)}장", flush=True)
    else:
        for layout, classes in layouts.items():
            images = sorted((source / layout).glob("*.jpg"))
            if args.cargo_only and not images:  # 조합이 많아 아직 찍지 않은 조합은 건너뛴다.
                continue
            if len(images) < 4:
                parser.error(f"{layout} 사진이 4장 이상 필요합니다.")
            # 촬영 시각순 앞 3/4은 학습용, 뒤 1/4은 검증용이다. 연속 촬영한 비슷한 사진이 양쪽에 섞이지 않는다.
            boundary = len(images) * 3 // 4
            val_limit = args.per_layout // 4
            for split, group, limit in (
                ("train", images[:boundary], args.per_layout - val_limit),
                ("val", images[boundary:], val_limit),
            ):
                paths = sample(group, limit)
                print(f"{layout} {split}: {len(paths)}장", flush=True)
                selected.extend((split, path, classes) for path in paths)

    if not selected:
        raise SystemExit(f"촬영한 사진이 없습니다: {source}")
    if args.list:
        return
    model = None
    if args.auto_weights:
        from ultralytics import YOLO

        model = YOLO(args.auto_weights)
        # 클래스 이름 집합만 비교한다. 번호는 다를 수 있으나 라벨 번호는 `names`에서 다시 찾으므로 문제없다.
        if set(model.names.values()) != set(names):
            parser.error("모델 클래스가 데이터셋 클래스와 다릅니다.")
    selected.sort(key=lambda item: item[0] != "val")  # 검증용 사진부터 라벨링한다.
    for split, path, classes in selected:
        name = f"{path.parent.name}_{path.stem}"
        image_path = output / "images" / split / f"{name}.jpg"
        label_path = output / "labels" / split / f"{name}.txt"
        # 같은 사진이 학습용과 검증용에 모두 들어가면 검증 점수가 부풀려지므로 막는다.
        if any((output / "labels" / other / label_path.name).exists()
               for other in ("train", "val") if other != split):
            raise SystemExit(f"이미 반대 분할에 라벨링된 사진입니다: {path.name}")
        if label_path.is_file() and image_path.is_file():  # 이미 라벨링한 사진은 건너뛰어 이어서 작업한다.
            continue
        print(f"{split}: {name}", flush=True)
        rows = label_image(path, classes, names, model)
        if rows is None:
            print("저장된 사진은 유지됩니다. 같은 명령으로 이어서 라벨링하세요.")
            return
        label_path.parent.mkdir(parents=True, exist_ok=True)
        image_path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(path, image_path)
        label_path.write_text(rows, encoding="utf-8")

    config = {"path": str(output), "train": "images/train", "val": "images/val",
              "names": dict(enumerate(names))}
    (output / "data.yaml").write_text(yaml.safe_dump(config), encoding="utf-8")
    print(f"데이터셋 준비 완료: {output / 'data.yaml'}")


if __name__ == "__main__":
    main()
