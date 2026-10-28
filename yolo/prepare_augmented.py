import itertools  # 밝기·크기·회전의 모든 조합을 만든다.
import json  # 각 사진의 원본과 변형 조건을 기록한다.
from pathlib import Path  # 데이터 경로를 다룬다.

import cv2  # 밝기와 기하 변형을 적용한다.
import numpy as np  # 변환 행렬과 좌표를 계산한다.
import yaml  # 원본 data.yaml의 클래스 이름을 읽는다.


def floor_background(image, label_path, scale):  # 물체가 없는 바닥 부분으로 640 화면을 채운다.
    h, w = image.shape[:2]  # 축소한 사진 크기다.
    x1, y1, x2, y2 = w, h, 0, 0  # 모든 물체를 감싸는 범위를 구한다.
    for row in label_path.read_text().splitlines():  # 정답 박스를 읽는다.
        _, x, y, bw, bh = map(float, row.split())  # 정규화 좌표다.
        x1, x2 = min(x1, int((x - bw / 2) * w)), max(x2, int((x + bw / 2) * w) + 1)
        y1, y2 = min(y1, int((y - bh / 2) * h)), max(y2, int((y + bh / 2) * h) + 1)
    strips = [image[:, :max(x1, 0)], image[:, x2:], image[:max(y1, 0)], image[y2:]]  # 물체 바깥
    floor = max(strips, key=lambda strip: strip.shape[0] * strip.shape[1])  # 가장 넓은 바닥이다.
    if floor.shape[0] < 4 or floor.shape[1] < 4:  # 바닥이 거의 없으면 회색으로 채운다.
        return np.full((640, 640, 3), 114, dtype=np.uint8)
    size = (max(1, round(floor.shape[1] * scale)), max(1, round(floor.shape[0] * scale)))
    floor = cv2.resize(floor, size, interpolation=cv2.INTER_AREA)  # 물체와 같은 배율로 줄인다.
    block = np.vstack([np.hstack([floor, floor[:, ::-1]]),  # 좌우 반전을 붙여 이음새를 줄인다.
                       np.hstack([floor[::-1], floor[::-1, ::-1]])])  # 상하 반전도 붙인다.
    repeat = (640 // block.shape[0] + 1, 640 // block.shape[1] + 1, 1)  # 640을 덮을 반복 수다.
    return np.ascontiguousarray(np.tile(block, repeat)[:640, :640])  # 바닥 무늬를 이어 붙인다.


def generate(source, output, split):  # 한 분할(train 또는 val)의 변형 파일을 생성한다.
    image_folder = output / 'images' / split  # 변형 사진을 저장할 경로다.
    label_folder = output / 'labels' / split  # 변형 정답을 저장할 경로다.
    for directory in (image_folder, label_folder):  # 사진과 정답 폴더를 준비한다.
        directory.mkdir(parents=True, exist_ok=True)
    manifest = []  # 생성한 모든 사진의 출처를 기록한다.
    levels = itertools.product([0.4, 0.7, 1.0, 1.3, 1.6],  # 픽셀 밝기 배율이다.
                               [0.05, 0.1, 0.2, 0.3,  # 아주 먼 거리에서 작게 보이는 크기다.
                                0.5, 0.75, 1.0, 1.25, 1.5], [-20, 0, 20])  # 크기와 회전이다.
    for brightness, scale, angle in levels:  # 135가지 조건을 반복한다.
        condition = f'b{brightness}_s{scale}_r{angle}'  # 조건 식별자를 만든다.
        for path in sorted((source / 'images' / split).glob('*.jpg')):  # 해당 분할 원본만 읽는다.
            image = cv2.imread(str(path))  # 원본 사진을 읽는다.
            if image is None:  # 손상 사진을 확인한다.
                raise SystemExit(f'읽기 실패: {path}')  # 오류를 안내한다.
            height, width = image.shape[:2]  # 원본 크기를 확인한다.
            ratio = min(640 / width, 640 / height)  # 긴 변을 640으로 줄인다.
            w, h = round(width * ratio), round(height * ratio)  # 축소 크기를 계산한다.
            image = cv2.resize(image, (w, h), interpolation=cv2.INTER_AREA)  # 크기를 통일한다.
            left, top = (640 - w) // 2, (640 - h) // 2  # 640 화면 중앙 배치 위치를 구한다.
            label_path = source / 'labels' / split / f'{path.stem}.txt'  # 원본 정답이다.
            background = floor_background(image, label_path, scale)  # 바닥으로 채운 배경이다.
            shift = np.array([[1, 0, left], [0, 1, top], [0, 0, 1]])  # 사진을 중앙으로 옮긴다.
            matrix = cv2.getRotationMatrix2D((320, 320), angle, scale)  # 기하 변환이다.
            transformed = cv2.warpAffine(image, matrix @ shift, (640, 640), dst=background,
                                         borderMode=cv2.BORDER_TRANSPARENT)  # 배경 위에 사진을 얹는다.
            transformed = np.clip(transformed.astype(np.float32) * brightness, 0, 255)
            labels, dropped = [], 0  # 변환한 박스와 잘린 박스 개수를 기록한다.
            for row in label_path.read_text().splitlines():  # 정답 박스를 처리한다.
                cls, x, y, bw, bh = map(float, row.split())  # 정규화 좌표를 읽는다.
                x1, x2 = (x - bw / 2) * w + left, (x + bw / 2) * w + left
                y1, y2 = (y - bh / 2) * h + top, (y + bh / 2) * h + top
                corners = np.array([[x1, y1, 1], [x2, y1, 1],  # 네 모서리를 만든다.
                                    [x2, y2, 1], [x1, y2, 1]])
                warped = corners @ matrix.T  # 사진과 같은 변환을 박스에도 적용한다.
                low, high = warped.min(axis=0), warped.max(axis=0)  # 외접 박스를 계산한다.
                clipped_low, clipped_high = np.clip(low, 0, 640), np.clip(high, 0, 640)
                size = np.maximum(0, clipped_high - clipped_low)  # 화면에 남은 박스다.
                visibility = np.prod(size) / max(np.prod(high - low), 1e-9)
                if visibility < 0.5 or min(size) < 2:  # 절반 이상 잘린 물체는 제외한다.
                    dropped += 1  # 잘린 박스를 센다.
                    continue  # 남은 박스만 정답으로 저장한다.
                center = (clipped_low + clipped_high) / 2 / 640  # 정규화 중심이다.
                bw, bh = size / 640  # 정규화 크기다.
                labels.append(f'{int(cls)} {center[0]:.6f} {center[1]:.6f} {bw:.6f} {bh:.6f}')
            if dropped:  # 물체가 잘린 사진은 저장하지 않는다.
                continue  # 다음 원본으로 넘어간다.
            name = f'{condition}_{path.stem}'  # 원본까지 식별할 이름을 만든다.
            destination = image_folder / f'{name}.jpg'  # 결과 사진 경로다.
            if not cv2.imwrite(str(destination), transformed.astype(np.uint8)):
                raise SystemExit(f'저장 실패: {destination}')  # 저장 오류를 안내한다.
            (label_folder / f'{name}.txt').write_text('\n'.join(labels) + '\n')
            manifest.append({'image': destination.name, 'source': path.name,  # 출처를 기록한다.
                             'condition': condition, 'brightness': brightness,
                             'scale': scale, 'angle': angle})  # 변형 조건을 기록한다.
    print(f'{split}: 이미지·라벨 {len(manifest)}쌍 생성 완료', flush=True)
    return manifest  # 생성한 파일 목록을 반환한다.


def generate_crops(source, output, split):  # 물체 일부가 화면 가장자리에서 잘린 사진을 생성한다.
    image_folder = output / 'images' / split  # 크롭 사진을 저장할 경로다.
    label_folder = output / 'labels' / split  # 크롭 정답을 저장할 경로다.
    for directory in (image_folder, label_folder):  # 사진과 정답 폴더를 준비한다.
        directory.mkdir(parents=True, exist_ok=True)
        for old in directory.glob('crop_*'):  # 이전 방식으로 만든 크롭 파일을 지운다.
            old.unlink()
    manifest = []  # 생성한 크롭 사진의 출처를 기록한다.
    for path in sorted((source / 'images' / split).glob('*.jpg')):  # 해당 분할 원본만 읽는다.
        image = cv2.imread(str(path))  # 원본 사진을 읽는다.
        if image is None:  # 손상 사진을 확인한다.
            raise SystemExit(f'읽기 실패: {path}')  # 오류를 안내한다.
        height, width = image.shape[:2]  # 원본 크기를 확인한다.
        boxes = []  # 픽셀 단위 정답 박스를 모은다.
        label_path = source / 'labels' / split / f'{path.stem}.txt'  # 원본 정답이다.
        for row in label_path.read_text().splitlines():  # 정규화 좌표를 픽셀 좌표로 바꾼다.
            cls, x, y, bw, bh = map(float, row.split())  # 클래스와 중심·크기를 읽는다.
            boxes.append((int(cls), (x - bw / 2) * width, (y - bh / 2) * height,
                          (x + bw / 2) * width, (y + bh / 2) * height))  # 좌상단·우하단이다.
        crops = itertools.product(range(len(boxes)), ['left', 'right', 'top', 'bottom'], [0.4, 0.7])
        for index, side, visible in crops:  # 물체마다 각각 기준으로 잘라 본다.
            _, x1, y1, x2, y2 = boxes[index]  # 이번에 자르는 기준 물체다.
            bw, bh = x2 - x1, y2 - y1  # 기준 물체의 픽셀 크기다.
            left, top = max(0, round(x1 - bw)), max(0, round(y1 - bh))  # 물체 주변 배경도 남긴다.
            right, bottom = min(width, round(x2 + bw)), min(height, round(y2 + bh))
            if side == 'left':  # 물체의 왼쪽 부분만 남긴다.
                right = round(x1 + bw * visible)
            elif side == 'right':  # 물체의 오른쪽 부분만 남긴다.
                left = round(x2 - bw * visible)
            elif side == 'top':  # 물체의 위쪽 부분만 남긴다.
                bottom = round(y1 + bh * visible)
            else:  # 물체의 아래쪽 부분만 남긴다.
                top = round(y2 - bh * visible)
            crop = image[top:bottom, left:right]  # 사진을 잘라낸다. 크기 조절은 YOLO에 맡긴다.
            crop_w, crop_h = right - left, bottom - top  # 잘라낸 사진 크기다.
            labels = []  # 크롭 안에 남은 박스를 기록한다.
            for cls, bx1, by1, bx2, by2 in boxes:  # 모든 박스를 크롭 범위로 자른다.
                cx1, cy1 = max(bx1, left), max(by1, top)  # 크롭 안에 남은 좌상단이다.
                cx2, cy2 = min(bx2, right), min(by2, bottom)  # 크롭 안에 남은 우하단이다.
                if cx2 - cx1 < 2 or cy2 - cy1 < 2:  # 거의 남지 않은 박스는 정답에서 뺀다.
                    continue  # 다음 박스로 넘어간다.
                labels.append(f'{cls} {((cx1 + cx2) / 2 - left) / crop_w:.6f} '  # 중심 x다.
                              f'{((cy1 + cy2) / 2 - top) / crop_h:.6f} '  # 중심 y다.
                              f'{(cx2 - cx1) / crop_w:.6f} {(cy2 - cy1) / crop_h:.6f}')  # 크기다.
            name = f'crop_{side}{visible}_b{index}_{path.stem}'  # 방향·남은 비율·기준 물체 번호다.
            destination = image_folder / f'{name}.jpg'  # 결과 사진 경로다.
            if not cv2.imwrite(str(destination), crop):  # 크롭 사진을 저장한다.
                raise SystemExit(f'저장 실패: {destination}')  # 저장 오류를 안내한다.
            (label_folder / f'{name}.txt').write_text('\n'.join(labels) + '\n')
            manifest.append({'image': destination.name, 'source': path.name,  # 출처를 기록한다.
                             'condition': f'crop_{side}{visible}'})  # 크롭 조건을 기록한다.
    print(f'{split}: 크롭 이미지·라벨 {len(manifest)}쌍 생성 완료', flush=True)
    return manifest  # 생성한 파일 목록을 반환한다.


if __name__ == '__main__':  # 직접 실행할 때만 실제 이미지와 라벨 파일을 생성한다.
    root = Path(__file__).resolve().parent  # yolo 폴더를 찾는다.
    source = root / 'dataset_strawberry_car_ball'  # prepare_data.py가 분리한 원본 데이터다.
    output = root / 'dataset_strawberry_car_ball_expanded'  # 원본을 보존할 별도 데이터 폴더다.
    records = {}  # 각 분할의 원본 출처를 기록한다.
    for split in ('train', 'val'):  # 원본 분할을 유지하며 각각 변형한다.
        records[split] = generate(source, output, split) + generate_crops(source, output, split)
        (output / f'manifest_{split}.json').write_text(json.dumps(records[split], indent=2))
    train_sources = {row['source'] for row in records['train']}  # 학습 원본 목록이다.
    val_sources = {row['source'] for row in records['val']}  # 검증 원본 목록이다.
    if train_sources & val_sources:  # 같은 원본의 파생본이 양쪽에 섞였는지 확인한다.
        raise SystemExit('학습·검증 원본이 겹칩니다.')  # 누수 시 설정 생성을 중단한다.
    names = yaml.safe_load((source / 'data.yaml').read_text(encoding='utf-8'))['names']  # 클래스
    settings = (f'path: {output}\n'  # YOLO 데이터 기준 경로다.
                'train: images/train\nval: images/val\n'  # 실제 이미지 경로다.
                'names:\n')  # 원본과 같은 클래스를 등록한다.
    settings += ''.join(f'  {index}: {name}\n' for index, name in names.items())
    (output / 'data.yaml').write_text(settings, encoding='utf-8')  # 학습 설정을 저장한다.
    print(f'학습 가능한 데이터 설정: {output / "data.yaml"}')  # 설정 위치를 안내한다.
