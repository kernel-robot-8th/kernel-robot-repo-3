import argparse  # 카메라 번호와 모델을 지정한다.
from pathlib import Path  # 모델 경로를 찾는다.

import cv2  # 카메라 영상을 읽고 화면에 표시하는 OpenCV를 가져온다.
from ultralytics import YOLO  # 학습한 YOLO 모델을 불러온다.


if __name__ == '__main__':  # 직접 실행할 때만 카메라 인식을 시작한다.
    folder = Path(__file__).resolve().parent  # yolo 폴더다.
    parser = argparse.ArgumentParser()  # 실행 옵션을 준비한다.
    parser.add_argument('--camera', type=int, default=0)  # /dev/video 번호다. 0은 외장 USB 카메라다.
    parser.add_argument('--weights', type=Path)  # 모델 경로다. 생략하면 최근 추가 학습 모델이다.
    parser.add_argument('--conf', type=float, default=0.25)  # 이 신뢰도 이상만 표시한다.
    args = parser.parse_args()  # 옵션을 읽는다.
    candidates = list((folder / 'runs').glob('strawberry_car_ball_expanded*/weights/best.pt'))
    weights = args.weights or (max(candidates, key=lambda p: p.stat().st_mtime) if candidates else None)
    if weights is None or not weights.is_file():  # 학습 완료 여부를 확인한다.
        raise SystemExit('먼저 python yolo/train_expanded.py로 학습하세요.')
    print(f'모델: {weights}')  # 어떤 모델을 쓰는지 확인한다.
    model = YOLO(str(weights))  # 모델을 불러온다.

    camera = cv2.VideoCapture(args.camera)  # 카메라를 연다.
    if not camera.isOpened():  # 다른 프로그램이 카메라를 쓰고 있는지 확인한다.
        raise SystemExit(f'카메라 {args.camera}번을 열 수 없습니다. 다른 프로그램을 닫아 주세요.')
    output = folder / 'test_results'  # S로 저장한 화면을 둘 폴더다.
    output.mkdir(exist_ok=True)  # 폴더가 없으면 만든다.
    saved = len(list(output.glob('camera_*.jpg')))  # 기존 저장 화면 다음 번호부터 저장한다.
    print('S: 현재 화면 저장 / Q 또는 Esc: 종료')  # 조작 방법을 안내한다.

    while True:  # 종료할 때까지 카메라 영상을 인식한다.
        ok, frame = camera.read()  # 카메라에서 한 장을 읽는다.
        if not ok:  # 영상이 들어오지 않으면 멈춘다.
            raise SystemExit('카메라 영상을 읽지 못했습니다.')
        result = model.predict(source=frame, conf=args.conf, imgsz=640,  # 학습 때와 같은 입력 크기다.
                               device=0, verbose=False)[0]  # 첫 번째 NVIDIA GPU로 인식한다.
        view = result.plot()  # 검출 박스·클래스·신뢰도를 그린다.
        found = [f'{result.names[int(box.cls[0])]} {float(box.conf[0]):.0%}' for box in result.boxes]
        speed = sum(result.speed.values())  # 한 장 처리 시간(ms)이다.
        cv2.putText(view, f'{len(found)} found  {speed:.0f} ms', (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)  # 검출 개수와 속도를 표시한다.
        cv2.imshow('camera test', view)  # 결과 화면을 표시한다.
        key = cv2.waitKey(1) & 0xFF  # 키 입력을 확인한다.
        if key in (ord('q'), ord('Q'), 27):  # Q나 Esc를 누르면 끝낸다.
            break
        if key in (ord('s'), ord('S')):  # S를 누르면 현재 결과 화면을 저장한다.
            saved += 1  # 저장 번호다.
            path = output / f'camera_{saved:03d}.jpg'  # 저장 경로다.
            cv2.imwrite(str(path), view)  # 박스가 그려진 화면을 저장한다.
            summary = ', '.join(found) or '검출 없음'  # 저장한 화면의 검출 목록이다.
            print(f'저장: {path.name} -> {summary}')

    camera.release()  # 카메라를 놓아준다.
    cv2.destroyAllWindows()  # 창을 닫는다.
