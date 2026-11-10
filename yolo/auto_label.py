import argparse  # 실험 이름과 모델 경로를 지정한다.
from collections import Counter  # 클래스별 검출 수를 센다.
from pathlib import Path  # 사진과 라벨 파일의 경로를 다룬다.
import shutil  # 원본을 유지하면서 파일을 복사한다.

import cv2  # 박스가 그려진 사진을 저장한다.
from ultralytics import YOLO  # 학습한 YOLO 모델을 불러온다.


folder = Path(__file__).resolve().parent  # yolo 폴더다.
parser = argparse.ArgumentParser()  # 실행 옵션을 준비한다.
parser.add_argument('--name', required=True)  # 실험 이름이다. 예: labels10
parser.add_argument('--weights', type=Path, required=True)  # 이번 단계에서 학습한 모델이다.
parser.add_argument('--conf', type=float, default=0.25)  # 이 신뢰도 이상만 자동 라벨로 저장한다.
parser.add_argument('--source', type=Path, default=folder / 'total_frames')  # 자동 라벨을 만들 사진 폴더다.
args = parser.parse_args()  # 옵션을 읽는다.
if not args.weights.is_file():  # 모델 경로를 확인한다.
    raise SystemExit(f'모델이 없습니다: {args.weights}')

source = args.source  # 원본 사진 폴더다. 여기에는 아무것도 쓰지 않는다.
output = folder / 'experiments' / args.name  # 이번 단계의 기록을 모으는 폴더다.
manual = output / 'manual'  # 이번 단계에서 직접 라벨링한 사진과 라벨이다.
auto_labels = output / 'auto_labels'  # 모델이 만든 라벨이다.
previews = output / 'previews'  # 박스가 그려진 사진이다.
if auto_labels.exists():  # 기록을 덮어쓰지 않는다.
    raise SystemExit(f'이미 결과가 있습니다: {auto_labels}')

if not manual.exists():  # 처음이면 지금 라벨이 있는 사진을 이번 단계의 직접 라벨로 기록한다.
    manual.mkdir(parents=True)
    for label_path in source.glob('*.txt'):  # 라벨과 사진을 함께 복사한다.
        shutil.copy2(label_path, manual / label_path.name)
        shutil.copy2(label_path.with_suffix('.jpg'), manual / f'{label_path.stem}.jpg')
trained = {path.stem for path in manual.glob('*.txt')}  # 학습에 쓴 사진이다.
targets = [path for path in sorted(source.glob('*.jpg')) if path.stem not in trained]  # 나머지 사진만 평가한다.
if not targets:  # 자동 라벨을 만들 사진이 있어야 한다. 직접 라벨은 없어도 된다.
    raise SystemExit(f'자동 라벨 대상이 없습니다: 직접 라벨 {len(trained)}장, 전체 {len(trained) + len(targets)}장')

auto_labels.mkdir()  # 자동 라벨 폴더를 만든다.
previews.mkdir()  # 확인용 사진 폴더를 만든다.
shutil.copy2(args.weights, output / 'model.pt')  # 이번 단계의 모델도 함께 보관한다.
model = YOLO(str(args.weights))  # 모델을 불러온다.
counts, confidences, per_image = Counter(), Counter(), []  # 클래스별 개수, 신뢰도 합, 사진별 박스 수다.
for image_path in targets:  # 사진을 한 장씩 인식한다.
    result = model.predict(source=str(image_path), conf=args.conf, imgsz=640, device=0, verbose=False)[0]
    rows = []  # 이 사진의 라벨이다.
    for box in result.boxes:  # 검출한 물체마다 반복한다.
        cls, confidence = int(box.cls[0]), float(box.conf[0])  # 클래스와 신뢰도다.
        x, y, w, h = box.xywhn[0].tolist()  # 사진 대비 비율 좌표다. 직접 라벨과 같은 형식이다.
        rows.append(f'{cls} {x:.6f} {y:.6f} {w:.6f} {h:.6f}')
        counts[cls] += 1
        confidences[cls] += confidence
    (auto_labels / f'{image_path.stem}.txt').write_text(''.join(f'{row}\n' for row in rows), encoding='utf-8')
    if not cv2.imwrite(str(previews / image_path.name), result.plot()):  # 확인용 사진을 저장한다.
        raise SystemExit(f'결과 저장 실패: {image_path.name}')
    per_image.append(len(rows))

lines = [  # 나중에 단계끼리 비교할 요약이다.
    f'실험: {args.name}',
    f'모델: {args.weights}',
    f'신뢰도 기준: {args.conf}',
    f'직접 라벨: {len(trained)}장',
    f'자동 라벨 대상: {len(targets)}장',
    f'사진당 평균 검출: {sum(per_image) / len(per_image):.1f}개 (최소 {min(per_image)}, 최대 {max(per_image)})',
    '클래스별 검출 수 / 평균 신뢰도:',
]
for cls, name in model.names.items():  # 한 번도 검출되지 않은 클래스도 0으로 남긴다.
    average = confidences[cls] / counts[cls] if counts[cls] else 0  # 평균 신뢰도다.
    lines.append(f'  {name}: {counts[cls]}개 / {average:.2f}')
summary = '\n'.join(lines) + '\n'  # 요약 글이다.
(output / 'summary.txt').write_text(summary, encoding='utf-8')  # 파일로 남긴다.
print(summary + f'저장 폴더: {output}')  # 결과를 출력한다.
