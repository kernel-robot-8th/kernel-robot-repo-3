import argparse  # 라벨링할 폴더를 지정한다.
from pathlib import Path  # 사진과 라벨 파일의 경로를 다룬다.

import cv2  # 사진을 띄우고 마우스로 박스를 그리는 OpenCV를 가져온다.


classes = {0: 'strawberry', 1: 'car', 2: 'ball', 3: 'pumpkin', 4: 'apple', 5: 'carrot', 6: 'fruits',
           7: 'potato', 8: 'watermelon', 9: 'tissue', 10: 'kiwi', 11: 'pepper'}  # data.yaml과 같게 둔다.
menu = {'1': 1, '2': 10, '3': 8, '4': 0, '5': 3, '6': 4, '7': 11, '8': 2, '9': 5, '0': 7, '-': 9}  # 키와 라벨 번호다.
colors = {0: (0, 0, 255), 1: (255, 128, 0), 2: (0, 200, 255), 3: (0, 140, 255), 4: (80, 80, 255),
          5: (0, 100, 200), 6: (120, 0, 200), 7: (0, 230, 230), 8: (0, 200, 0), 9: (255, 0, 255),
          10: (40, 160, 120), 11: (200, 0, 120)}  # 클래스별 박스 색이다.
parser = argparse.ArgumentParser()  # 실행 옵션을 준비한다.
parser.add_argument('--folder', type=Path, default=Path(__file__).resolve().parent / 'total_frames')  # 사진 폴더다.
parser.add_argument('--start', type=int, default=1)  # 몇 번째 사진부터 볼지 정한다.
parser.add_argument('--every', type=int, default=1)  # 몇 장 간격으로 볼지 정한다.
parser.add_argument('--auto', type=Path)  # 라벨이 없는 사진에 먼저 띄울 자동 라벨 폴더다. 원본은 고치지 않는다.
args = parser.parse_args()  # 옵션을 읽는다.

images = sorted(args.folder.glob('*.jpg'))[args.start - 1::args.every]  # 고른 사진을 이름순으로 가져온다.
if not images:  # 경로가 잘못되었거나 사진이 없는지 확인한다.
    raise SystemExit(f'사진이 없습니다: {args.folder}')

boxes = []  # 현재 사진의 확정한 박스다. (클래스, x1, y1, x2, y2) 표시 화면 기준 좌표다.
drag = {'start': None, 'end': None, 'pending': None, 'selected': None}  # 그리는 박스, 종류를 기다리는 박스, 고른 박스다.
size = {'w': 1, 'h': 1}  # 표시 중인 사진의 너비와 높이다.


def on_mouse(event, x, y, flags, param):  # 마우스 드래그로 박스를 그린다.
    if drag['pending']:  # 이전 박스의 종류를 고르기 전에는 새 박스를 그리지 않는다.
        return
    x, y = min(max(x, 0), size['w'] - 1), min(max(y, 0), size['h'] - 1)  # 안내 줄이나 창 밖으로 나가지 않게 한다.
    if event == cv2.EVENT_LBUTTONDOWN:  # 왼쪽 버튼을 누른 곳이 시작점이다.
        drag['start'] = drag['end'] = (x, y)
        drag['selected'] = None  # 고른 박스를 해제한다.
    elif event == cv2.EVENT_MOUSEMOVE and drag['start']:  # 끄는 동안 끝점을 따라간다.
        drag['end'] = (x, y)
    elif event == cv2.EVENT_LBUTTONUP and drag['start']:  # 버튼을 떼면 박스가 완성된다.
        (sx, sy), (ex, ey) = drag['start'], (x, y)  # 시작점과 끝점이다.
        drag['start'] = drag['end'] = None  # 다음 드래그를 준비한다.
        if abs(ex - sx) > 4 and abs(ey - sy) > 4:  # 충분히 끌었으면 새 박스다.
            drag['pending'] = (min(sx, ex), min(sy, ey), max(sx, ex), max(sy, ey))
        else:  # 클릭이면 그 점을 포함한 박스 중 가장 작은 것을 고른다. 겹친 박스도 작은 것부터 고를 수 있다.
            hits = [i for i, b in enumerate(boxes) if b[1] <= x <= b[3] and b[2] <= y <= b[4]]
            drag['selected'] = min(hits, key=lambda i: (boxes[i][3] - boxes[i][1]) * (boxes[i][4] - boxes[i][2]),
                                   default=None)


def load(image_path):  # 사진을 표시 크기로 줄이고 저장된 라벨이 있으면 박스로 불러온다.
    image = cv2.imread(str(image_path))  # 원본 사진을 읽는다.
    if image is None:  # 사진이 손상되어 읽지 못했는지 확인한다.
        raise SystemExit(f'사진을 읽지 못했습니다: {image_path}')
    height, width = image.shape[:2]  # 원본 사진의 크기다.
    scale = min(800 / height, 1400 / width)  # 화면에 맞는 비율이다. 작은 사진은 물체를 그리기 쉽게 키운다.
    preview = cv2.resize(image, (round(width * scale), round(height * scale)))  # 표시용 크기로 바꾼다.
    size['h'], size['w'] = preview.shape[:2]  # 마우스 좌표를 제한할 크기다.
    boxes.clear()  # 이전 사진의 박스를 지운다.
    drag.update(start=None, end=None, pending=None, selected=None)  # 이전 사진의 드래그 상태를 지운다.
    label_path = image_path.with_suffix('.txt')  # 사진과 이름이 같은 라벨 파일이다.
    if not label_path.exists() and args.auto:  # 직접 저장한 라벨이 없으면 자동 라벨을 띄운다.
        label_path = args.auto / label_path.name
    if label_path.exists():  # 이미 저장한 라벨은 다시 표시해서 고칠 수 있게 한다.
        for row in label_path.read_text(encoding='utf-8').splitlines():  # 박스를 한 줄씩 읽는다.
            cls, cx, cy, w, h = row.split()  # 클래스와 비율 좌표다.
            cx, cy, w, h = float(cx) * size['w'], float(cy) * size['h'], float(w) * size['w'], float(h) * size['h']
            boxes.append((int(cls), round(cx - w / 2), round(cy - h / 2), round(cx + w / 2), round(cy + h / 2)))
    return preview


def save(image_path):  # 현재 박스를 사진 대비 비율로 바꿔 txt로 저장한다.
    labels = [f"{cls} {(x1 + x2) / 2 / size['w']:.6f} {(y1 + y2) / 2 / size['h']:.6f} "
              f"{(x2 - x1) / size['w']:.6f} {(y2 - y1) / size['h']:.6f}"
              for cls, x1, y1, x2, y2 in boxes]  # 사진과 박스를 함께 줄였으므로 비율은 원본에도 그대로 맞는다.
    image_path.with_suffix('.txt').write_text('\n'.join(labels) + '\n', encoding='utf-8')
    print(f'저장 완료: {image_path.name} 박스 {len(boxes)}개')


index = next((i for i, p in enumerate(images) if not p.with_suffix('.txt').exists()), 0)  # 라벨이 없는 첫 사진이다.
done = sum(p.with_suffix('.txt').exists() for p in images)  # 이미 라벨이 있는 사진 수다.
print(f'라벨링 대상: {len(images)}장 / 완료: {done}장 / {images[index].name}부터 시작')
print('드래그: 박스 / 숫자·- 키: 종류 / 클릭: 박스 고르기(숫자·- 키: 종류 변경, D: 삭제) / Z: 마지막 박스 삭제')
print('N·Enter: 저장 후 다음 / P: 저장 후 이전 / X: 저장 없이 다음 / Q: 종료')
cv2.namedWindow('label')  # 라벨링 창을 만든다.
cv2.setMouseCallback('label', on_mouse)  # 마우스 동작을 박스 그리기에 연결한다.
preview = load(images[index])  # 첫 사진을 불러온다.
entries = [f'{key}:{classes[cls]}' for key, cls in menu.items()]  # 키 안내 글자다.

while True:  # 종료하거나 마지막 사진을 끝낼 때까지 반복한다.
    view = cv2.copyMakeBorder(preview, 0, 90, 0, 0, cv2.BORDER_CONSTANT, value=(40, 40, 40))  # 아래에 안내 줄을 붙인다.
    for cls, x1, y1, x2, y2 in boxes:  # 확정한 박스를 클래스 색으로 그린다.
        cv2.rectangle(view, (x1, y1), (x2, y2), colors[cls], 2)
        cv2.putText(view, classes[cls], (x1, max(y1 - 5, 15)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, colors[cls], 2)
    if drag['selected'] is not None:  # 고른 박스를 두꺼운 흰색으로 그린다.
        cv2.rectangle(view, boxes[drag['selected']][1:3], boxes[drag['selected']][3:], (255, 255, 255), 4)
    if drag['start']:  # 드래그 중인 박스를 흰색으로 그린다.
        cv2.rectangle(view, drag['start'], drag['end'], (255, 255, 255), 1)
    if drag['pending']:  # 종류를 기다리는 박스를 초록색으로 그린다.
        cv2.rectangle(view, drag['pending'][:2], drag['pending'][2:], (0, 255, 0), 2)
        guide = 'press class key (Z: cancel)'
    elif drag['selected'] is not None:  # 고른 박스로 할 수 있는 일을 안내한다.
        guide = f"selected: {classes[boxes[drag['selected']][0]]}  class key:change  D:delete  click empty:cancel"
    else:  # 다음 행동을 안내한다.
        guide = (f'{index + 1}/{len(images)}  boxes:{len(boxes)}  drag:new  click:select  Z:undo  '
                 'N:save+next  P:save+prev  X:skip  Q:quit')
    cv2.putText(view, guide, (10, size['h'] + 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    for row, part in enumerate((entries[:6], entries[6:])):  # 키 안내를 두 줄로 나눠 쓴다.
        for col, text in enumerate(part):
            cv2.putText(view, text, (10 + col * 165, size['h'] + 50 + row * 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 255, 255), 1)
    cv2.imshow('label', view)  # 화면을 갱신한다.
    key = cv2.waitKey(20) & 0xFF  # 키 입력을 확인한다.
    if cv2.getWindowProperty('label', cv2.WND_PROP_VISIBLE) < 1:  # 창을 닫으면 끝낸다.
        print('창을 닫아 종료합니다. 현재 사진은 저장하지 않았습니다.')
        break
    if key in (ord('q'), ord('Q'), 27):  # Q나 Esc를 누르면 현재 사진을 저장하지 않고 끝낸다.
        print('종료합니다. 현재 사진은 저장하지 않았습니다.')
        break
    if drag['pending']:  # 종류를 기다리는 동안에는 종류와 취소만 받는다.
        if chr(key) in menu:  # 키로 박스 종류를 정한다.
            boxes.append((menu[chr(key)], *drag['pending']))
            drag['pending'] = None  # 다음 박스를 그릴 수 있다.
        elif key in (ord('z'), ord('Z')):  # 종류를 기다리는 박스를 지운다.
            drag['pending'] = None
        continue
    if drag['selected'] is not None and (chr(key) in menu or key in (ord('d'), ord('D'))):  # 고른 박스를 고친다.
        if chr(key) in menu:  # 종류를 바꾼다.
            boxes[drag['selected']] = (menu[chr(key)], *boxes[drag['selected']][1:])
        else:  # 박스를 지운다.
            boxes.pop(drag['selected'])
        drag['selected'] = None  # 다음 박스를 고를 수 있다.
    elif key in (ord('z'), ord('Z')) and boxes:  # 마지막 박스를 지운다.
        boxes.pop()
        drag['selected'] = None  # 번호가 바뀔 수 있으므로 해제한다.
    elif key in (ord('n'), ord('N'), 13, ord('p'), ord('P'), ord('x'), ord('X')):  # 다음이나 이전 사진으로 간다.
        if boxes and key not in (ord('x'), ord('X')):  # 빈 라벨은 prepare_data.py가 거부하므로 저장하지 않는다.
            save(images[index])
        else:  # 라벨을 만들지 않고 넘어간다. 라벨이 없는 사진은 학습에서 빠진다.
            print(f'건너뜀: {images[index].name}')
        step =-1 if key in (ord('p'), ord('P')) else 1  # 이동 방향이다.
        if index + step >= len(images):  # 마지막 사진까지 끝났다.
            print('마지막 사진까지 저장했습니다.')
            break
        index = max(index + step, 0)  # 첫 사진 앞으로는 가지 않는다.
        preview = load(images[index])  # 다음 사진을 불러온다.

cv2.destroyAllWindows()  # 창을 닫는다.
completed = sum(p.with_suffix('.txt').exists() for p in images)  # 저장된 라벨 개수를 센다.
print(f'라벨 파일: {completed}/{len(images)}장')  # 현재까지의 작업 진행 상황을 출력한다.
