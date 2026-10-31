import argparse  # 카메라·간격·장수를 지정한다.
from pathlib import Path  # 저장 폴더를 다룬다.
import time  # 촬영 간격을 잰다.

import cv2  # 카메라 영상을 읽고 저장하는 OpenCV를 가져온다.


if __name__ == '__main__':  # 직접 실행할 때만 촬영을 시작한다.
    folder = Path(__file__).resolve().parent  # yolo 폴더다.
    parser = argparse.ArgumentParser()  # 실행 옵션을 준비한다.
    head_camera = '/dev/v4l/by-id/usb-Jieli_Technology_USB_Composite_Device-video-index0'  # HCAM0, 관제 Head Camera 기본값과 같다.
    parser.add_argument('--camera', default=head_camera)  # 카메라 경로나 /dev/video 번호다.
    parser.add_argument('--out', type=Path, default=folder / 'head_frames')  # 사진을 저장할 폴더다.
    parser.add_argument('--count', type=int, default=60)  # 찍을 장수다.
    parser.add_argument('--interval', type=float, default=2.0)  # 몇 초마다 찍을지다. 그 사이에 물체를 옮긴다.
    parser.add_argument('--warmup', type=float, default=10.0)  # 첫 장까지 기다리는 시간이다.
    parser.add_argument('--width', type=int, default=640)  # 관제 서버 캡처와 같은 640x480이 기본이다.
    parser.add_argument('--height', type=int, default=480)
    args = parser.parse_args()  # 옵션을 읽는다.

    source = int(args.camera) if args.camera.isdigit() else args.camera  # 숫자면 /dev/video 번호로 쓴다.
    camera = cv2.VideoCapture(source, cv2.CAP_V4L2)  # USB 카메라는 V4L2로 연다.
    camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))  # 큰 해상도는 MJPG로만 나온다.
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if not camera.isOpened():  # 다른 프로그램이 카메라를 쓰고 있는지 확인한다.
        raise SystemExit(f'카메라 {args.camera}를 열 수 없습니다. 다른 프로그램을 닫아 주세요 (fuser -v 로 확인).')
    args.out.mkdir(parents=True, exist_ok=True)  # 폴더가 없으면 만든다.
    saved = len(list(args.out.glob('head_*.jpg')))  # 이미 찍은 사진 다음 번호부터 이어서 저장한다.
    print(f'저장 폴더: {args.out} (현재 {saved}장). {args.warmup:.0f}초 뒤부터 {args.interval:.0f}초마다 찍는다. Q: 중단')

    start = time.monotonic()  # 시작 시각이다.
    next_shot = start + args.warmup  # 첫 장을 찍을 시각이다.
    taken = 0  # 이번에 찍은 장수다.
    flash = 0  # 찍은 직후 화면 테두리를 몇 프레임 하얗게 보여줄지다.
    while taken < args.count:  # 정한 장수를 다 찍을 때까지 반복한다.
        ok, frame = camera.read()  # 카메라에서 한 장을 읽는다.
        if not ok:  # MJPG 프레임이 깨지면 한 장 건너뛴다.
            continue
        now = time.monotonic()  # 현재 시각이다.
        if now >= next_shot:  # 찍을 시각이 되면 그림 없는 원본을 저장한다.
            cv2.imwrite(str(args.out / f'head_{saved + taken:03d}.jpg'), frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
            taken += 1
            next_shot = now + args.interval  # 다음 촬영 시각이다.
            flash = 3
        view = frame.copy()  # 안내 문구는 화면에만 그린다.
        if now < start + args.warmup:  # 첫 장 전에는 남은 시간을 보여준다.
            message = f'START IN {start + args.warmup - now:4.1f}s'
        else:  # 촬영 중에는 장수와 다음 촬영까지 남은 시간을 보여준다.
            message = f'SHOT {taken}/{args.count}  next {max(next_shot - now, 0):3.1f}s'
        if flash:  # 방금 찍었으면 테두리를 하얗게 그려 알린다.
            cv2.rectangle(view, (0, 0), (view.shape[1] - 1, view.shape[0] - 1), (255, 255, 255), 20)
            flash -= 1
        cv2.putText(view, message, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 0), 6)
        cv2.putText(view, message, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 2)
        cv2.imshow('head capture - move objects', view)  # 물체를 옮기면서 볼 화면이다.
        if (cv2.waitKey(1) & 0xFF) in (ord('q'), ord('Q'), 27):  # Q나 Esc로 중단한다.
            break

    print(f'{taken}장 저장 -> {args.out}')  # 다음 단계 head_autolabel.py의 입력이다.
    camera.release()  # 카메라를 놓아준다.
    cv2.destroyAllWindows()  # 창을 닫는다.
