import argparse  # 카메라 경로와 저장 위치를 지정한다.
from pathlib import Path  # 저장 폴더와 카메라 경로를 다룬다.
import re  # 파일 이름에서 번호를 읽는다.

import cv2  # 카메라 영상을 읽고 화면에 표시한다.


parser = argparse.ArgumentParser()  # 실행 옵션을 준비한다.
by_path = '/dev/v4l/by-path/pci-0000:00:14.0-usb-0:{}:1.0-video-index0'  # USB 포트 기준 경로라 번호가 바뀌어도 같다.
parser.add_argument('--front', default=by_path.format(2))  # 앞에서 보는 카메라다.
parser.add_argument('--top', default=by_path.format(1))  # 위에서 내려다보는 카메라다.
parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parent)  # 저장 폴더를 만들 위치다.
parser.add_argument('--scene', type=int)  # 배치 번호다. 생략하면 지난 실행 다음 번호를 쓴다.
args = parser.parse_args()  # 옵션을 읽는다.
used = [int(m.group(1)) for name in ('cam_front_frames', 'cam_top_frames')
        for p in (args.root / name).glob('scene*_*.jpg') if (m := re.fullmatch(r'scene(\d+)_\d+', p.stem))]
scene = args.scene or max(used, default=0) + 1  # 실행할 때마다 새 배치 번호를 쓴다. 두 카메라가 같은 번호를 쓴다.
print(f'배치 번호: scene{scene:02d}')  # 이번 배치를 알린다.
print('Space: 현재 카메라 저장 / N: 다음 카메라 / Q 또는 Esc: 종료')  # 조작 방법을 안내한다.

finished = False  # Q를 누르면 다음 카메라로 넘어가지 않는다.
for label, name, path in (('front', 'cam_front_frames', args.front), ('top', 'cam_top_frames', args.top)):  # 차례로 찍는다.
    if finished:  # 이미 종료했으면 멈춘다.
        break
    if not Path(path).exists():  # USB가 빠져 있으면 장치 경로 자체가 없다.
        raise SystemExit(f'카메라가 연결되어 있지 않습니다: {path} (ls /dev/v4l/by-path/)')
    camera = cv2.VideoCapture(path)  # 이번 카메라만 연다.
    if not camera.isOpened():  # 다른 프로그램이 쓰고 있는지 확인한다.
        raise SystemExit(f'카메라를 열 수 없습니다: {path} 다른 프로그램을 닫아 주세요.')
    print(f'{label} 카메라 영상을 기다리는 중...')  # 영상이 안 오면 여기서 멈춘 것처럼 보이므로 알린다.
    for _ in range(30):  # 켠 직후에는 색이 초록빛이므로 몇 장을 버린다.
        if not camera.read()[0]:  # 영상이 안 오면 장마다 오래 기다리므로 첫 실패에서 끝낸다.
            raise SystemExit(f'{label} 카메라에서 영상이 오지 않습니다: {path} USB를 다시 꽂아 주세요.')
    folder = args.root / name  # 카메라별 저장 폴더다.
    folder.mkdir(exist_ok=True)  # 폴더가 없으면 만든다.
    numbers = [int(m.group(1)) for p in folder.glob(f'scene{scene:02d}_*.jpg')
               if (m := re.fullmatch(rf'scene{scene:02d}_(\d+)', p.stem))]
    count = max(numbers, default=0)  # 같은 배치를 이어 찍어도 덮어쓰지 않도록 가장 큰 번호 다음부터 저장한다.
    print(f'{label} 카메라: {folder} (scene{scene:02d} 마지막 번호 {count})')

    while True:  # 다음 카메라로 넘어가거나 종료할 때까지 표시한다.
        ok, frame = camera.read()  # 한 장을 읽는다.
        if not ok:  # 영상이 들어오지 않으면 멈춘다.
            raise SystemExit('카메라 영상을 읽지 못했습니다.')
        view = frame.copy()  # 글자는 표시용 사진에만 쓴다.
        cv2.putText(view, f'scene{scene:02d} {label}  saved:{count}  Space:save  N:next  Q:quit', (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)  # 배치, 카메라, 조작 방법을 표시한다.
        cv2.imshow('camera capture', view)  # 화면을 갱신한다.
        key = cv2.waitKey(1) & 0xFF  # 키 입력을 확인한다.
        if key in (ord('q'), ord('Q'), 27):  # Q나 Esc를 누르면 끝낸다.
            finished = True
            break
        if key in (ord('n'), ord('N')):  # N을 누르면 다음 카메라로 넘어간다.
            break
        if key == ord(' '):  # Space를 누르면 글자 없는 원본을 저장한다.
            count += 1  # 다음 사진 번호다.
            file_name = f'scene{scene:02d}_{count:03d}.jpg'  # 배치 번호와 그 배치 안의 순번이다.
            if not cv2.imwrite(str(folder / file_name), frame):  # 저장 성공 여부를 확인한다.
                raise SystemExit(f'저장 실패: {folder}')
            print(f'저장: {name}/{file_name}')
    camera.release()  # 다음 카메라를 열기 전에 놓아준다.

cv2.destroyAllWindows()  # 창을 닫는다.
