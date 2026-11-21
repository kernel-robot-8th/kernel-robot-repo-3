import argparse  # 미리보기와 실제 적용을 고른다.
from pathlib import Path  # 사진과 라벨 파일의 경로를 다룬다.
import shutil  # 고치기 전에 원본 라벨을 백업한다.

import cv2  # 확인용 사진에 박스를 그린다.


# 위 카메라 라벨에서 Pinky 로봇 전체를 car(1번)로 잡은 박스를 지운다.
# 장난감 자동차는 사진 면적의 1.5% 미만, Pinky 박스는 3~6%라서 면적 2%를 기준으로 나눈다.
folder = Path(__file__).resolve().parent  # yolo 폴더다.
parser = argparse.ArgumentParser()  # 실행 옵션을 준비한다.
parser.add_argument('--source', type=Path, default=folder / 'cam_top_frames')  # 고칠 라벨 폴더다.
parser.add_argument('--min-area', type=float, default=0.02)  # car 박스 면적(w×h)이 이보다 크면 Pinky로 본다.
parser.add_argument('--apply', action='store_true')  # 붙이면 실제로 라벨을 고친다. 없으면 미리보기만 한다.
args = parser.parse_args()  # 옵션을 읽는다.

check = folder / 'experiments' / 'pinky_check'  # 확인용 사진과 백업을 모으는 폴더다.
previews = check / 'previews'  # 지울 박스를 빨간색으로 그린 사진이다.
backup = check / 'backup_labels'  # 고치기 전 원본 라벨이다.

changes = []  # (라벨 경로, 남길 줄, 지울 줄)
for label_path in sorted(args.source.glob('*.txt')):  # 라벨 파일을 하나씩 확인한다.
    keep, remove = [], []  # 이 파일에서 남길 줄과 지울 줄이다.
    for row in label_path.read_text().splitlines():  # 박스 한 줄씩 읽는다.
        values = row.split()  # 클래스와 좌표 다섯 항목이다.
        if len(values) == 5 and values[0] == '1' and float(values[3]) * float(values[4]) > args.min_area:
            remove.append(row)  # 큰 car 박스는 Pinky다.
        elif row.strip():
            keep.append(row)  # 나머지는 그대로 둔다.
    if remove:  # 지울 박스가 있는 파일만 기록한다.
        changes.append((label_path, keep, remove))

empty = [path.name for path, keep, _ in changes if not keep]  # 지우고 나면 빈 라벨이 되는 파일이다.
print(f'대상 폴더: {args.source}')
print(f'고칠 파일: {len(changes)}개 · 지울 박스: {sum(len(r) for _, _, r in changes)}개 (기준 면적 {args.min_area})')
if empty:  # prepare_data.py는 빈 라벨에서 멈추므로 먼저 알린다.
    print(f'지우면 빈 라벨이 되는 파일 {len(empty)}개: {", ".join(empty[:10])}')

previews.mkdir(parents=True, exist_ok=True)  # 확인용 사진 폴더를 만든다.
for label_path, keep, remove in changes:  # 지울 박스는 빨강, 남길 박스는 초록으로 그린다.
    image = cv2.imread(str(label_path.with_suffix('.jpg')))  # 같은 이름의 사진이다.
    if image is None:
        continue
    height, width = image.shape[:2]  # 비율 좌표를 픽셀로 바꿀 크기다.
    for rows, color in [(keep, (0, 200, 0)), (remove, (0, 0, 255))]:
        for row in rows:
            _, x, y, w, h = map(float, row.split())  # 중심 좌표와 크기다.
            x1, y1 = int((x - w / 2) * width), int((y - h / 2) * height)
            x2, y2 = int((x + w / 2) * width), int((y + h / 2) * height)
            cv2.rectangle(image, (x1, y1), (x2, y2), color, 3 if color == (0, 0, 255) else 1)
    cv2.imwrite(str(previews / label_path.with_suffix('.jpg').name), image)
print(f'확인용 사진: {previews} (빨강 = 지울 박스, 초록 = 남길 박스)')

if not args.apply:  # 미리보기만 하고 끝낸다.
    print('라벨은 바꾸지 않았습니다. 확인 후 --apply를 붙여 실행하세요.')
    raise SystemExit
if empty:  # 빈 라벨을 만들지 않는다.
    raise SystemExit('빈 라벨이 생기는 파일이 있어 적용하지 않았습니다.')
if backup.exists():  # 이전 백업을 덮어쓰지 않는다.
    raise SystemExit(f'이미 백업이 있습니다: {backup}')
backup.mkdir(parents=True)  # 백업 폴더를 만든다.
for label_path in args.source.glob('*.txt'):  # 폴더의 모든 라벨을 원본 그대로 백업한다.
    shutil.copy2(label_path, backup / label_path.name)
for label_path, keep, _ in changes:  # 큰 car 박스를 뺀 라벨로 다시 쓴다.
    label_path.write_text(''.join(f'{row}\n' for row in keep), encoding='utf-8')
print(f'적용 완료: {len(changes)}개 파일 수정 · 원본 백업: {backup}')
