"""촬영한 사진에 박스를 그리고 핑키 적재 전용 데이터셋을 만든다."""

import argparse
from datetime import datetime
from itertools import combinations
from pathlib import Path
import shutil

import cv2
import yaml


YOLO_DIR = Path(__file__).resolve().parent
CLASSES = ("carrot", "tissue", "pinky", "watermelon")
LAYOUTS = {
    "carrot1": ("carrot", "pinky"),
    "tissue1": ("tissue", "pinky"),
    "carrot1_tissue1": ("carrot", "tissue", "pinky"),
    "watermelon1_tissue1": ("watermelon", "tissue", "pinky"),
}

CARGO = ("watermelon", "carrot", "kiwi", "apple")
CARGO_LAYOUTS = {"empty": ("pinky",)}
for size in range(1, len(CARGO) + 1):
    for cargo in combinations(CARGO, size):
        CARGO_LAYOUTS["_".join(f"{name}1" for name in cargo)] = (*cargo, "pinky")


def sample(images, limit):
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
            if key in (10, 13, 32):
                if box and box[2] and box[3]:
                    return box
                print("박스를 먼저 드래그하세요.", flush=True)
    finally:
        cv2.destroyAllWindows()


def propose_boxes(model, image, classes):
    result = model.predict(image, conf=0.15, imgsz=640, verbose=False)[0]
    height, width = image.shape[:2]
    proposals = {}
    for box in sorted(result.boxes, key=lambda item: float(item.conf.item()), reverse=True):
        name = result.names[int(box.cls.item())]
        if name not in classes or name in proposals:
            continue
        x1, y1, x2, y2 = (round(value) for value in box.xyxy[0].tolist())
        x1, x2 = max(0, x1), min(width, x2)
        y1, y2 = max(0, y1), min(height, y2)
        if x1 < x2 and y1 < y2:
            proposals[name] = (x1, y1, x2 - x1, y2 - y1)
    return proposals


def review_proposals(image, proposals, classes):
    window = "Auto labels | 1-5: select | D: delete | Enter: save | C: quit"
    selected = classes[0]
    height, width = image.shape[:2]
    start = None
    cursor = None

    def drag(event, x, y, flags, _):
        nonlocal start, cursor
        point = (min(max(x, 0), width - 1), min(max(y, 0), height - 1))
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
            if key in (ord("d"), ord("D"), 8, 127):
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
    image = cv2.imread(str(path))
    if image is None:
        raise SystemExit(f"사진을 읽지 못했습니다: {path}")
    height, width = image.shape[:2]
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
        cv2.rectangle(preview, (x, y), (x + w, y + h), (0, 220, 0), 2)
    return "".join(rows)


def review_labels(output, selection, split, layouts, names):
    selected_layouts = layouts if selection == "all" else (selection,)
    for folder in ((split,) if split else ("val", "train")):
        for layout in selected_layouts:
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
    if args.per_layout < 4:
        parser.error("--per-layout은 4 이상이어야 합니다.")
    base = YOLO_DIR / "experiments"
    names = (*CARGO, "pinky") if args.cargo_only else CLASSES
    layouts = CARGO_LAYOUTS if args.cargo_only else LAYOUTS
    source = args.source or base / ("pinky_tray_watermelon_carrot_kiwi_apple" if args.cargo_only else "pinky_tray_eval")
    output = (args.output or base / ("pinky_cargo_dataset" if args.cargo_only else "pinky_tray_dataset")).resolve()
    config_path = output / "data.yaml"
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
        selected = [("train", path, layouts[layout]) for path in images if path.stem >= args.since]
        print(f"{layout} 추가 대상: {len(selected)}장", flush=True)
    else:
        for layout, classes in layouts.items():
            images = sorted((source / layout).glob("*.jpg"))
            if args.cargo_only and not images:
                continue
            if len(images) < 4:
                parser.error(f"{layout} 사진이 4장 이상 필요합니다.")
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
        if set(model.names.values()) != set(names):
            parser.error("모델 클래스가 데이터셋 클래스와 다릅니다.")
    selected.sort(key=lambda item: item[0] != "val")
    for split, path, classes in selected:
        name = f"{path.parent.name}_{path.stem}"
        image_path = output / "images" / split / f"{name}.jpg"
        label_path = output / "labels" / split / f"{name}.txt"
        if any((output / "labels" / other / label_path.name).exists()
               for other in ("train", "val") if other != split):
            raise SystemExit(f"이미 반대 분할에 라벨링된 사진입니다: {path.name}")
        if label_path.is_file() and image_path.is_file():
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
