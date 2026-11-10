import argparse  # 집중할 클래스와 저장 위치를 지정한다.
from pathlib import Path  # 사진과 라벨 경로를 다룬다.
import random  # 자르는 위치를 조금씩 바꾼다.
import shutil  # 이전에 만든 결과 폴더를 지운다.

import cv2  # 사진을 자르고 저장한다.


folder = Path(__file__).resolve().parent  # yolo 폴더다.
parser = argparse.ArgumentParser()  # 실행 옵션을 준비한다.
parser.add_argument('--classes', type=int, nargs='+', default=[5, 11])  # 집중할 클래스다. 기본은 carrot 5, pepper 11이다.
parser.add_argument('--min-boxes', type=int, default=10)  # 박스가 이보다 적은 사진은 라벨이 덜 됐다고 보고 쓰지 않는다.
parser.add_argument('--output', type=Path, default=folder / 'cam_focus_frames')  # 자른 사진을 둘 폴더다. 매번 새로 만든다.
args = parser.parse_args()  # 옵션을 읽는다.

random.seed(0)  # 다시 실행해도 같은 결과가 나오게 한다.
shutil.rmtree(args.output, ignore_errors=True)  # 이 스크립트가 만든 폴더라 지우고 다시 만든다. 원본은 건드리지 않는다.
args.output.mkdir()
made = skipped = 0  # 만든 사진과 버린 자르기 수다.
for source in ('cam_front_frames', 'cam_top_frames'):  # 실제 카메라 사진만 쓴다.
    for image_path in sorted((folder / source).glob('*.jpg')):  # 사진을 하나씩 본다.
        label_path = image_path.with_suffix('.txt')  # 같은 이름의 라벨이다.
        if not label_path.exists():  # 라벨이 없는 사진은 쓰지 않는다.
            continue
        rows = [line.split() for line in label_path.read_text(encoding='utf-8').splitlines()]  # 박스 목록이다.
        if len(rows) < args.min_boxes:  # 일부만 라벨링한 사진은 빠진 물체가 배경으로 학습되므로 쓰지 않는다.
            continue
        image = cv2.imread(str(image_path))  # 원본 사진이다.
        height, width = image.shape[:2]  # 사진 크기다.
        boxes = []  # (클래스, x1, y1, x2, y2) 픽셀 좌표다.
        for cls, cx, cy, w, h in rows:
            cx, cy, w, h = float(cx) * width, float(cy) * height, float(w) * width, float(h) * height
            boxes.append((int(cls), cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2))
        targets = [b for b in boxes if b[0] in args.classes]  # 집중할 물체다.
        if not targets:  # 집중할 물체가 없는 사진은 넘어간다.
            continue
        ux1, uy1 = min(b[1] for b in targets), min(b[2] for b in targets)  # 집중할 물체를 모두 감싸는 영역이다.
        ux2, uy2 = max(b[3] for b in targets), max(b[4] for b in targets)
        for k, scale in enumerate((1.6, 1.6, 2.4, 2.4, 3.5)):  # 확대 정도를 바꿔 여러 장을 만든다.
            size = min(max(ux2 - ux1, uy2 - uy1, 48) * scale, width, height)  # 정사각형 한 변이다.
            cx = (ux1 + ux2) / 2 + random.uniform(-0.15, 0.15) * size  # 물체가 늘 가운데 오지 않게 조금 옮긴다.
            cy = (uy1 + uy2) / 2 + random.uniform(-0.15, 0.15) * size
            x1 = int(min(max(cx - size / 2, 0), width - size))  # 사진 밖으로 나가지 않게 맞춘다.
            y1 = int(min(max(cy - size / 2, 0), height - size))
            x2, y2 = x1 + int(size), y1 + int(size)
            labels, ambiguous = [], False  # 자른 사진의 라벨과, 애매하게 걸친 물체가 있는지다.
            for cls, bx1, by1, bx2, by2 in boxes:  # 모든 물체를 자른 영역 기준으로 옮긴다.
                ix1, iy1, ix2, iy2 = max(bx1, x1), max(by1, y1), min(bx2, x2), min(by2, y2)  # 겹치는 부분이다.
                if ix2 <= ix1 or iy2 <= iy1:  # 영역 밖이면 상관없다.
                    continue
                visible = (ix2 - ix1) * (iy2 - iy1) / ((bx2 - bx1) * (by2 - by1))  # 물체가 보이는 비율이다.
                if visible < 0.4:  # 조금만 걸친 물체는 라벨을 붙이기도 빼기도 애매하므로 이 자르기를 버린다.
                    ambiguous = visible > 0.05
                    if ambiguous:
                        break
                    continue
                w_, h_ = x2 - x1, y2 - y1  # 자른 사진 크기다.
                labels.append(f'{cls} {((ix1 + ix2) / 2 - x1) / w_:.6f} {((iy1 + iy2) / 2 - y1) / h_:.6f} '
                              f'{(ix2 - ix1) / w_:.6f} {(iy2 - iy1) / h_:.6f}')
            if ambiguous or not labels:  # 쓸 수 없는 자르기는 버린다.
                skipped += 1
                continue
            name = f'{source}_{image_path.stem}_c{k}'  # 원본을 알 수 있는 이름이다.
            cv2.imwrite(str(args.output / f'{name}.jpg'), image[y1:y2, x1:x2])  # 자른 사진을 저장한다.
            (args.output / f'{name}.txt').write_text('\n'.join(labels) + '\n', encoding='utf-8')
            made += 1

print(f'만든 사진: {made}장, 버린 자르기: {skipped}개, 저장 폴더: {args.output}')
