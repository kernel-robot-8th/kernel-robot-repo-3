import argparse  # 카메라 번호를 지정한다.
from pathlib import Path  # 사진과 라벨을 저장할 경로를 다룬다.

import cv2  # 카메라 영상을 띄우고 마우스로 박스를 그리는 OpenCV를 가져온다.


classes = {0: 'strawberry', 1: 'car', 2: 'ball'}  # 라벨 번호와 이름이다. data.yaml과 같게 둔다.
colors = {0: (0, 0, 255), 1: (255, 128, 0), 2: (0, 200, 255)}  # 클래스별 박스 색이다.
parser = argparse.ArgumentParser()  # 실행 옵션을 준비한다.
parser.add_argument('--camera', type=int, default=0)  # /dev/video 번호다. 0은 외장 USB 카메라다.
args = parser.parse_args()  # 옵션을 읽는다.

folder = Path(__file__).resolve().parent / 'camera_frames'  # yolo/camera_frames에 모은다.
folder.mkdir(exist_ok=True)  # prepare_data.py가 기존 3종류 사진과 합쳐 학습 데이터로 만든다.
count = len(list(folder.glob('*.jpg')))  # 이미 저장한 사진 다음 번호부터 이어서 저장한다.

boxes = []  # 확정한 박스다. (클래스, x1, y1, x2, y2)
drag = {'start': None, 'end': None, 'pending': None}  # 그리는 중인 박스와 종류를 기다리는 박스다.


def on_mouse(event, x, y, flags, param):  # 정지 화면에서 마우스 드래그로 박스를 그린다.
    if drag['pending']:  # 이전 박스의 종류를 고르기 전에는 새 박스를 그리지 않는다.
        return
    if event == cv2.EVENT_LBUTTONDOWN:  # 왼쪽 버튼을 누른 곳이 시작점이다.
        drag['start'] = drag['end'] = (x, y)
    elif event == cv2.EVENT_MOUSEMOVE and drag['start']:  # 끄는 동안 끝점을 따라간다.
        drag['end'] = (x, y)
    elif event == cv2.EVENT_LBUTTONUP and drag['start']:  # 버튼을 떼면 박스가 완성된다.
        (sx, sy), (ex, ey) = drag['start'], (x, y)  # 시작점과 끝점이다.
        drag['start'] = drag['end'] = None  # 다음 드래그를 준비한다.
        if abs(ex - sx) > 4 and abs(ey - sy) > 4:  # 클릭만 한 아주 작은 박스는 무시한다.
            drag['pending'] = (min(sx, ex), min(sy, ey), max(sx, ex), max(sy, ey))


camera = cv2.VideoCapture(args.camera)  # 카메라를 연다.
if not camera.isOpened():  # 다른 프로그램이 카메라를 쓰고 있는지 확인한다.
    raise SystemExit(f'카메라 {args.camera}번을 열 수 없습니다. 다른 프로그램을 닫아 주세요.')
cv2.namedWindow('camera')  # 실시간 화면과 라벨링을 한 창에서 한다.
cv2.setMouseCallback('camera', on_mouse)  # 마우스 동작을 박스 그리기에 연결한다.
print(f'저장 폴더: {folder} (현재 {count}장)')  # 저장 위치를 안내한다.
print('Space: 멈추기 / 드래그: 박스 / 0·1·2: 종류 / Z: 되돌리기 / S: 저장 / C: 취소 / Q: 종료')

still = None  # 멈춘 화면이다. None이면 실시간 화면이다.
while True:  # 종료할 때까지 반복한다.
    if still is None:  # 실시간 모드에서는 카메라 영상을 계속 읽는다.
        ok, frame = camera.read()  # 카메라에서 한 장을 읽는다.
        if not ok:  # 영상이 들어오지 않으면 멈춘다.
            raise SystemExit('카메라 영상을 읽지 못했습니다.')
        view = frame.copy()  # 화면에 표시할 사진이다.
        cv2.putText(view, 'LIVE - Space: freeze', (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (0, 255, 0), 2)  # 실시간 상태를 표시한다.
    else:  # 멈춘 화면에 지금까지 그린 박스를 표시한다.
        view = still.copy()  # 원본은 박스 없이 저장하도록 복사본에 그린다.
        for cls, x1, y1, x2, y2 in boxes:  # 확정한 박스를 클래스 색으로 그린다.
            cv2.rectangle(view, (x1, y1), (x2, y2), colors[cls], 2)
            cv2.putText(view, classes[cls], (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        colors[cls], 2)  # 박스 위에 이름을 쓴다.
        if drag['start']:  # 드래그 중인 박스를 흰색으로 그린다.
            cv2.rectangle(view, drag['start'], drag['end'], (255, 255, 255), 1)
        if drag['pending']:  # 종류를 기다리는 박스를 초록색으로 그린다.
            x1, y1, x2, y2 = drag['pending']
            cv2.rectangle(view, (x1, y1), (x2, y2), (0, 255, 0), 2)
            guide = 'press ' + '  '.join(f'{i}:{name}' for i, name in classes.items())
        else:  # 다음 행동을 안내한다.
            guide = f'drag box ({len(boxes)})  Z:undo  S:save  C:cancel'
        cv2.putText(view, guide, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.imshow('camera', view)  # 화면을 갱신한다.
    key = cv2.waitKey(20) & 0xFF  # 키 입력을 확인한다.

    if key in (ord('q'), ord('Q'), 27):  # Q나 Esc를 누르면 끝낸다.
        break
    if still is None:  # 실시간 모드에서는 Space만 처리한다.
        if key == ord(' '):  # Space를 누르면 화면을 멈추고 라벨링을 시작한다.
            still, boxes = frame.copy(), []
        continue
    if drag['pending'] and key - ord('0') in classes:  # 숫자키로 박스 종류를 정한다.
        boxes.append((key - ord('0'), *drag['pending']))
        drag['pending'] = None  # 다음 박스를 그릴 수 있다.
    elif key in (ord('z'), ord('Z')):  # 마지막 박스나 종류를 기다리는 박스를 지운다.
        if drag['pending']:
            drag['pending'] = None
        elif boxes:
            boxes.pop()
    elif key in (ord('c'), ord('C')):  # 저장하지 않고 실시간 화면으로 돌아간다.
        still, boxes, drag['pending'] = None, [], None
        print('취소했습니다.')
    elif key in (ord('s'), ord('S')) and boxes and not drag['pending']:  # 사진과 라벨을 저장한다.
        height, width = still.shape[:2]  # 사진 크기다.
        labels = [f'{cls} {(x1 + x2) / 2 / width:.6f} {(y1 + y2) / 2 / height:.6f} '
                  f'{(x2 - x1) / width:.6f} {(y2 - y1) / height:.6f}'
                  for cls, x1, y1, x2, y2 in boxes]  # 박스를 사진 대비 비율로 바꾼다.
        count += 1  # 다음 사진 번호다.
        image_path = folder / f'frame_{count:06d}.jpg'  # 저장할 사진 경로다.
        if not cv2.imwrite(str(image_path), still):  # 박스 없는 원본 사진을 저장한다.
            raise SystemExit(f'저장 실패: {image_path}')
        image_path.with_suffix('.txt').write_text('\n'.join(labels) + '\n', encoding='utf-8')
        print(f'저장 완료: {image_path.name} 박스 {len(boxes)}개 (총 {count}장)')
        still, boxes = None, []  # 실시간 화면으로 돌아간다.

camera.release()  # 카메라를 놓아준다.
cv2.destroyAllWindows()  # 창을 닫는다.
