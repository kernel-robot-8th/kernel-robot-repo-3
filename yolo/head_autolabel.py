import argparse  # 입력 폴더·제외 사진·출력 이름을 지정한다.
from pathlib import Path  # 사진과 라벨 경로를 다룬다.

import cv2  # 사진을 읽고 색 영역을 찾는 OpenCV를 가져온다.
import numpy as np  # 마스크와 검수 시트를 만든다.
from ultralytics import YOLO  # COCO 사전학습 모델로 공·자동차를 찾는다.


classes = {0: 'strawberry', 1: 'car', 2: 'ball'}  # train.py·data.yaml과 같은 번호다.
colors = {0: (60, 60, 255), 1: (255, 160, 0), 2: (0, 200, 255)}  # 검수 시트의 클래스별 박스 색이다.
# COCO 클래스 → (우리 클래스, 최소 신뢰도). 장난감 자동차는 cell phone·truck으로 잡히곤 한다.
coco_map = {'sports ball': (2, 0.2), 'orange': (2, 0.2), 'apple': (0, 0.2),
            'car': (1, 0.1), 'truck': (1, 0.1), 'bus': (1, 0.1), 'cell phone': (1, 0.3)}


def overlap(a, b):  # 박스 a 면적 중 b와 겹치는 비율이다.
    x1, y1, x2, y2 = max(a[0], b[0]), max(a[1], b[1]), min(a[2], b[2]), min(a[3], b[3])
    return max(0, x2 - x1) * max(0, y2 - y1) / max((a[2] - a[0]) * (a[3] - a[1]), 1)


def red_minus_orange(image, box):  # 박스 안 빨강(딸기) 화소 수 - 주황(공) 화소 수다.
    x1, y1, x2, y2 = (int(v) for v in box)
    hsv = cv2.cvtColor(image[y1:y2, x1:x2], cv2.COLOR_BGR2HSV)
    red = cv2.inRange(hsv, (0, 120, 60), (6, 255, 255)) | cv2.inRange(hsv, (170, 120, 60), (180, 255, 255))
    orange = cv2.inRange(hsv, (8, 120, 60), (22, 255, 255))
    return int(np.count_nonzero(red)) - int(np.count_nonzero(orange))


def strawberry_boxes(image, others, scale):  # COCO에 딸기가 없어서 흰 선반 위의 진한 빨강 덩어리로 찾는다.
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (0, 140, 70), (6, 255, 255)) | cv2.inRange(hsv, (170, 140, 70), (180, 255, 255))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))  # 작은 점을 지운다.
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))  # 씨앗 구멍을 메운다.
    boxes = []
    for x, y, w, h, area in cv2.connectedComponentsWithStats(mask)[2][1:]:  # 덩어리마다 반복한다.
        # 크기·가로세로비·채움 비율로 딸기 모양만 남긴다. 면적 기준은 1280x720에서 정했다.
        if not (600 * scale < area < 30000 * scale) or not (0.5 < w / h < 2.0) or area / (w * h) < 0.45:
            continue
        box = (x, max(y - int(h * 0.25), 0), x + w, y + h)  # 위쪽 초록 꼭지를 조금 포함한다.
        if all(overlap(box, o) <= 0.3 for o in others):  # COCO가 이미 잡은 물체와 겹치면 버린다.
            boxes.append(box)
    return boxes


def label(model, image):  # 한 장의 자동 라벨이다. [(클래스, (x1, y1, x2, y2), 출처, 점수)]
    scale = image.shape[0] * image.shape[1] / (1280 * 720)  # 면적 기준을 해상도에 맞춘다.
    result = model.predict(image, conf=0.05, imgsz=1280, verbose=False)[0]  # 작은 물체도 보이게 크게 넣는다.
    found = []
    for box, cls, conf in zip(result.boxes.xyxy.tolist(), result.boxes.cls.tolist(), result.boxes.conf.tolist()):
        name = result.names[int(cls)]
        if name not in coco_map or (box[2] - box[0]) * (box[3] - box[1]) > 20000 * scale:  # 큰 박스는 물체가 아니다.
            continue
        ours, min_conf = coco_map[name]
        if conf < min_conf or box[0] < 5:  # 화면 왼쪽 끝에 붙은 박스(노트북 등)는 버린다.
            continue
        if ours in (0, 2):  # COCO는 딸기도 sports ball로 잡는다. 색으로 다시 나눈다.
            ours = 0 if red_minus_orange(image, box) > 0 else 2
        found.append((ours, tuple(box), f'{name} {conf:.2f}', conf))
    labels = []  # 같은 물체를 두 번 잡은 박스는 신뢰도 높은 것만 남긴다.
    for item in sorted(found, key=lambda l: -l[3]):
        if all(overlap(item[1], k[1]) < 0.5 and overlap(k[1], item[1]) < 0.5 for k in labels):
            labels.append(item)
    labels += [(0, box, 'hsv', 0.5) for box in strawberry_boxes(image, [l[1] for l in labels], scale)]
    return labels


def borrow_car(index, images, labels, max_gap=8, radius=40):  # 자동차를 놓친 사진은 가까운 사진의 자동차를 같은 자리에서 찾는다.
    gray = cv2.cvtColor(images[index], cv2.COLOR_BGR2GRAY)
    for gap in range(1, max_gap + 1):  # 가까운 사진부터 본다.
        for other in (index - gap, index + gap):
            cars = [l for l in labels.get(other, []) if l[0] == 1 and l[2] != 'borrowed']
            if not cars:
                continue
            x1, y1, x2, y2 = (int(v) for v in cars[0][1])
            template = cv2.cvtColor(images[other][y1:y2, x1:x2], cv2.COLOR_BGR2GRAY)  # 그 사진의 자동차 모습이다.
            sx, sy = max(x1 - radius, 0), max(y1 - radius, 0)  # 같은 자리 주변만 찾는다.
            window = gray[sy:min(y2 + radius, gray.shape[0]), sx:min(x2 + radius, gray.shape[1])]
            _, score, _, (x, y) = cv2.minMaxLoc(cv2.matchTemplate(window, template, cv2.TM_CCOEFF_NORMED))
            if score >= 0.6:  # 자동차가 돌아갔으면 점수가 낮다. 그때는 억지로 붙이지 않는다.
                return (1, (sx + x, sy + y, sx + x + x2 - x1, sy + y + y2 - y1), 'borrowed', score)
            return None
    return None


if __name__ == '__main__':  # 직접 실행할 때만 라벨링을 시작한다.
    folder = Path(__file__).resolve().parent  # yolo 폴더다.
    parser = argparse.ArgumentParser()  # 실행 옵션을 준비한다.
    parser.add_argument('--frames', type=Path, default=folder / 'head_frames')  # head_capture.py가 찍은 사진 폴더다.
    parser.add_argument('--name', default='dataset_head')  # 만들 데이터셋 폴더 이름이다 (yolo/ 아래).
    parser.add_argument('--skip', default='')  # 검수 후 뺄 사진 번호다. 예: 5,6,8 (손에 든 물체, 라벨 누락)
    parser.add_argument('--coco-weights', type=Path, default=folder / 'yolo11x.pt')  # COCO 사전학습 YOLO11x다.
    args = parser.parse_args()  # 옵션을 읽는다.
    if not args.coco_weights.is_file():  # 임의 모델 다운로드를 막고 경로를 안내한다.
        raise SystemExit(f'COCO 가중치가 없습니다: {args.coco_weights} '
                         '(https://github.com/ultralytics/assets/releases 의 yolo11x.pt, --coco-weights로 지정)')
    skip = {int(s) for s in args.skip.split(',') if s.strip()}  # 뺄 사진 번호다.
    model = YOLO(str(args.coco_weights))  # 공·자동차를 찾을 COCO 모델이다.

    images, labels = {}, {}  # 사진 번호별 사진과 라벨이다.
    for path in sorted(args.frames.glob('*.jpg')):  # 찍은 순서대로 읽는다.
        image = cv2.imread(str(path))
        if image is None:  # MJPG가 깨진 사진은 건너뛴다.
            print(f'읽을 수 없음: {path.name}')
            continue
        index = int(path.stem.split('_')[-1])  # head_012.jpg → 12
        images[index] = image
        labels[index] = label(model, image)
    for index, items in labels.items():  # 자동차를 놓친 사진을 보충한다.
        if not any(l[0] == 1 for l in items):
            car = borrow_car(index, images, labels)
            if car and all(overlap(car[1], l[1]) < 0.3 for l in items):
                items.append(car)

    out = folder / args.name  # 데이터셋 폴더다. yolo/.gitignore의 dataset*/ 대상이다.
    review = out / 'review'  # 사람이 확인할 검수 시트 폴더다.
    for sub in ('images/train', 'images/val', 'labels/train', 'labels/val', 'review'):
        (out / sub).mkdir(parents=True, exist_ok=True)
    thumbs = []  # 검수 시트에 넣을 축소 사진이다.
    counts = {'train': 0, 'val': 0, 'boxes': [0, 0, 0]}  # 결과 요약이다.
    for index, image in sorted(images.items()):
        if index in skip:  # 검수에서 뺀 사진이다.
            continue
        split = 'val' if (index // 5) % 5 == 4 else 'train'  # 연속 사진이 섞이지 않게 5장 블록 단위로 25%를 검증용으로 둔다.
        name = f'head_{index:03d}'
        height, width = image.shape[:2]
        cv2.imwrite(str(out / 'images' / split / f'{name}.jpg'), image, [cv2.IMWRITE_JPEG_QUALITY, 95])
        lines = [f'{c} {(x1 + x2) / 2 / width:.6f} {(y1 + y2) / 2 / height:.6f} {(x2 - x1) / width:.6f} {(y2 - y1) / height:.6f}'
                 for c, (x1, y1, x2, y2), *_ in labels[index]]  # YOLO 형식: 클래스 중심x 중심y 너비 높이 (0~1)
        (out / 'labels' / split / f'{name}.txt').write_text('\n'.join(lines) + ('\n' if lines else ''))
        counts[split] += 1
        for c, *_ in labels[index]:
            counts['boxes'][c] += 1
        view = image.copy()  # 검수용 그림이다.
        for c, (x1, y1, x2, y2), source, _ in labels[index]:
            cv2.rectangle(view, (int(x1), int(y1)), (int(x2), int(y2)), colors[c], 2)
            cv2.putText(view, f'{classes[c]} {source}', (int(x1), max(int(y1) - 6, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, colors[c], 2)
        view = cv2.resize(view, (640, 360))
        cv2.putText(view, f'{index} {split}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        thumbs.append(view)
    for start in range(0, len(thumbs), 12):  # 12장씩 3x4 검수 시트로 묶는다.
        group = thumbs[start:start + 12] + [np.zeros_like(thumbs[0])] * (12 - len(thumbs[start:start + 12]))
        sheet = np.vstack([np.hstack(group[r * 3:r * 3 + 3]) for r in range(4)])
        cv2.imwrite(str(review / f'sheet_{start // 12:02d}.jpg'), sheet, [cv2.IMWRITE_JPEG_QUALITY, 80])
    (out / 'data.yaml').write_text(f'path: {out}\ntrain: images/train\nval: images/val\n'
                                   'names:\n  0: strawberry\n  1: car\n  2: ball\n')  # 학습 설정이다.
    print(f'train {counts["train"]}장, val {counts["val"]}장, 박스 {dict(zip(classes.values(), counts["boxes"]))}')
    print(f'검수: {review} 의 시트를 보고 잘못된 사진 번호를 --skip 으로 빼서 다시 실행한다.')
