import argparse  # 미리보기와 실제 저장을 고른다.
from pathlib import Path  # 사진과 라벨 파일의 경로를 다룬다.
import shutil  # 미리보기 폴더를 새로 만든다.

import cv2  # 물체를 지우고 확인용 사진을 저장한다.
import numpy as np  # 지울 영역 마스크를 만든다.


# 카메라 사진에서 라벨 박스 안의 물체를 지우고 주변 픽셀로 채워(인페인팅) 배경 사진(물체 없음, 빈 라벨)을 만든다.
# 사진 전체 구도는 그대로라서 선반·수레·Pinky·로봇 팔이 물체 없이 보이는 장면이 된다.
folder = Path(__file__).resolve().parent  # yolo 폴더다.
parser = argparse.ArgumentParser()  # 실행 옵션을 준비한다.
parser.add_argument('--sources', nargs='+', default=['cam_top_frames', 'cam_front_frames'])  # 물체를 지울 사진 폴더다.
parser.add_argument('--count', type=int, default=120)  # 만들 배경 사진 수다. 전체 학습 사진의 약 7%다.
parser.add_argument('--margin', type=float, default=0.01)  # 박스를 이만큼 넓혀 지워서 물체 가장자리가 남지 않게 한다.
parser.add_argument('--radius', type=int, default=7)  # 지운 곳을 채울 때 참고하는 주변 픽셀 거리다.
parser.add_argument('--apply', action='store_true')  # 붙이면 background_frames에 저장한다. 없으면 미리보기만 한다.
parser.add_argument('--review', action='store_true')  # 붙이면 미리보기 사진을 한 장씩 보며 뺄 사진을 고른다.
args = parser.parse_args()  # 옵션을 읽는다.

output = folder / 'background_frames'  # 배경 사진과 빈 라벨을 저장할 폴더다.
previews = folder / 'experiments' / 'background_check'  # 확인용 사진 폴더다.

if args.review:  # 미리보기를 다시 만들지 않고 지금 있는 사진을 확인한다.
    crops_dir, removed = previews / 'crops', previews / 'removed'  # 남길 사진과 뺀 사진 폴더다.
    removed.mkdir(parents=True, exist_ok=True)  # 뺀 사진은 지우지 않고 여기로 옮긴다.
    names = sorted(path.name for path in crops_dir.glob('*.jpg')) + sorted(path.name for path in removed.glob('*.jpg'))
    if not names:
        raise SystemExit('먼저 미리보기를 만드세요: python yolo/make_background.py')
    print('Space/Enter/→: 남기고 다음 · D: 빼기 · K: 다시 남기기 · ←/B: 이전 · Q/Esc: 종료')
    index = 0  # 지금 보는 사진 번호다.
    while 0 <= index < len(names):
        name = names[index]
        is_removed = (removed / name).is_file()  # 이미 뺀 사진인지 확인한다.
        crop = cv2.imread(str((removed if is_removed else crops_dir) / name))  # 잘라낸 배경이다.
        where = cv2.imread(str(previews / 'where' / name))  # 원본에서 자른 위치다.
        crop = cv2.resize(crop, (where.shape[1], where.shape[0])) if where is not None else crop  # 원본과 같은 크기로 맞춘다.
        view = cv2.hconcat([crop, where]) if where is not None else crop
        state = 'REMOVED' if is_removed else 'KEEP'  # 지금 상태다.
        cv2.putText(view, f'{index + 1}/{len(names)}  {state}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                    (0, 0, 255) if is_removed else (0, 200, 0), 2)
        cv2.imshow('background review (left: erased, right: original)', view)
        key = cv2.waitKeyEx(0)  # 화살표까지 받는다.
        if key in (ord('q'), ord('Q'), 27):  # 종료한다.
            break
        if key in (ord('d'), ord('D')) and not is_removed:  # 물체가 보이면 뺀다.
            shutil.move(str(crops_dir / name), str(removed / name))
            print(f'뺌: {name}')
        elif key in (ord('k'), ord('K')) and is_removed:  # 잘못 뺀 사진을 되돌린다.
            shutil.move(str(removed / name), str(crops_dir / name))
            print(f'되돌림: {name}')
        if key in (ord('b'), ord('B'), 65361, 2424832):  # ← 또는 B는 이전 사진이다.
            index = max(index - 1, 0)
        elif key not in (ord('d'), ord('D'), ord('k'), ord('K')) or index < len(names) - 1:
            index += 1  # 나머지 키는 다음 사진이다.
    cv2.destroyAllWindows()
    print(f'남김 {len(list(crops_dir.glob("*.jpg")))}장 · 뺌 {len(list(removed.glob("*.jpg")))}장')
    print('저장하려면: python yolo/make_background.py --apply')
    raise SystemExit

images = []  # 라벨이 있는 카메라 사진이다.
for name in args.sources:  # 폴더별로 라벨이 있는 사진만 모은다.
    images += [path for path in sorted((folder / name).glob('*.jpg')) if path.with_suffix('.txt').is_file()]


def boxes_of(image_path, width, height):
    """라벨 박스를 margin만큼 넓힌 픽셀 좌표 (x1, y1, x2, y2) 목록이다."""
    boxes = []
    for row in image_path.with_suffix('.txt').read_text().splitlines():
        if row.strip():
            _, x, y, w, h = map(float, row.split())  # 중심 좌표와 크기 비율이다.
            boxes.append(((x - w / 2 - args.margin) * width, (y - h / 2 - args.margin) * height,
                          (x + w / 2 + args.margin) * width, (y + h / 2 + args.margin) * height))
    return boxes


if args.apply:  # 미리보기를 다시 만들지 않고, 확인하며 남긴 사진만 저장한다.
    kept = sorted((previews / 'crops').glob('*.jpg'))  # 미리보기에서 지우지 않은 사진이다.
    if not kept:
        raise SystemExit(f'먼저 미리보기를 만드세요: python yolo/make_background.py')
    if output.exists():  # 이전 배경 사진을 덮어쓰지 않는다.
        raise SystemExit(f'이미 배경 폴더가 있습니다: {output}')
    output.mkdir()  # 배경 폴더를 만든다.
    for crop in kept:  # 확인한 사진 그대로 옮긴다.
        shutil.copy2(crop, output / crop.name)
        (output / crop.with_suffix('.txt').name).write_text('', encoding='utf-8')  # 물체가 없다는 뜻의 빈 라벨이다.
    print(f'저장 완료: {len(kept)}장 → {output}')
    raise SystemExit

if previews.exists():  # 이전 미리보기를 지운다. 확인용 폴더라 원본과 무관하다.
    shutil.rmtree(previews)
(previews / 'crops').mkdir(parents=True)  # 물체를 지운 배경 사진이다. --review와 --apply가 이 폴더를 쓴다.
(previews / 'where').mkdir()  # 원본에서 어디를 지웠는지 보여주는 사진이다.
step = max(1, len(images) // args.count)  # 연속 프레임이 몰리지 않게 일정 간격으로 고른다.
chosen = sorted(images)[::step][:args.count]  # 이름순으로 고르게 뽑은 사진이다.
for image_path in chosen:  # 사진마다 라벨 박스 안을 지운다.
    image = cv2.imread(str(image_path))  # 원본 사진이다.
    height, width = image.shape[:2]  # 사진 크기다.
    mask = np.zeros((height, width), np.uint8)  # 지울 곳만 흰색인 마스크다.
    boxes = boxes_of(image_path, width, height)  # 지울 라벨 영역이다.
    for x1, y1, x2, y2 in boxes:
        cv2.rectangle(mask, (max(int(x1), 0), max(int(y1), 0)), (min(int(x2), width - 1), min(int(y2), height - 1)), 255, -1)
    erased = cv2.inpaint(image, mask, args.radius, cv2.INPAINT_TELEA)  # 지운 곳을 주변 색으로 채운다.
    name = f'{image_path.parent.name}_{image_path.stem}.jpg'  # 폴더 이름을 붙여 겹치지 않게 한다.
    cv2.imwrite(str(previews / 'crops' / name), erased)
    for x1, y1, x2, y2 in boxes:  # 지운 라벨 영역은 빨강이다.
        cv2.rectangle(image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)
    cv2.imwrite(str(previews / 'where' / name), image)
print(f'후보 사진: {len(images)}장 · 만든 배경: {len(chosen)}장 (목표 {args.count})')
print(f'확인용 사진: {previews} (crops = 물체를 지운 사진, where = 빨강이 지운 라벨)')
print('저장하지 않았습니다. --review로 확인하며 물체가 남은 사진을 뺀 뒤 --apply를 붙여 실행하세요.')
