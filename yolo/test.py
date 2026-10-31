from pathlib import Path  # 사진과 모델의 파일 경로를 다루는 도구를 가져온다.
import argparse  # 화면 없이 테스트할 실행 옵션을 읽는다.

import cv2  # 검출 결과 사진을 저장하고 화면에 표시하는 OpenCV를 가져온다.
from ultralytics import YOLO  # 학습한 YOLO 모델을 불러오는 도구를 가져온다.


if __name__ == '__main__':  # 파일을 직접 실행할 때만 테스트한다.
    folder = Path(__file__).resolve().parent  # 현재 test.py가 있는 yolo 폴더를 찾는다.
    parser = argparse.ArgumentParser()  # 원하는 모델과 사진을 지정할 옵션을 만든다.
    parser.add_argument('--weights', type=Path)  # 학습 모델 경로를 받는다.
    parser.add_argument('--source', type=Path, default=folder / 'dataset_strawberry_car_ball_expanded' / 'images' / 'val')
    parser.add_argument('--no-show', action='store_true')  # 화면 없이 결과를 저장한다.
    args = parser.parse_args()  # 터미널 옵션을 읽는다.
    candidates = list((folder / 'runs').glob('strawberry_car_ball*/weights/best.pt'))  # 학습 결과 전체다.
    weights = args.weights or (max(candidates, key=lambda p: p.stat().st_mtime) if candidates else None)
    if weights is None or not weights.is_file():  # 학습 완료 여부를 확인한다.
        raise SystemExit('먼저 python yolo/train.py로 학습하세요.')
    print(f'테스트 모델: {weights}')  # 어떤 학습 결과를 사용하는지 실행할 때 확인한다.
    model = YOLO(str(weights))  # 지정한 3클래스 모델을 불러온다.
    source = args.source  # 지정한 사진 또는 폴더를 사용한다.
    output = folder / 'test_results'  # 박스가 그려진 결과 사진을 저장할 폴더를 지정한다.
    output.mkdir(exist_ok=True)  # 결과 폴더가 없으면 만든다.
    show = not args.no_show  # --no-show를 붙이면 창 없이 저장만 한다.
    if not source.exists():  # 입력 경로가 존재하는지 확인한다.
        raise SystemExit(f'입력 경로가 없습니다: {source}')  # 경로 오류를 안내한다.
    extensions = ('.jpg', '.jpeg', '.png', '.webp')  # 지원할 사진 확장자다.
    images = [source] if source.is_file() else sorted(  # 사진 한 장 또는 폴더를 읽는다.
        p for p in source.iterdir() if p.is_file() and p.suffix.lower() in extensions
    )  # 폴더에서는 사진 파일만 이름순으로 처리한다.
    if not images:  # 지정한 폴더에 사진이 있는지 확인한다.
        raise SystemExit(f'테스트 사진이 없습니다: {source}')  # 사진이 없으면 경로를 알리고 종료한다.

    for image_path in images:  # 사진을 한 장씩 인식한다.
        result = model.predict(  # 모델에 사진을 넣어 딸기·자동차·공을 검출한다.
            source=str(image_path),  # 이번에 인식할 사진 경로를 지정한다.
            conf=0.25,  # 신뢰도가 25% 이상인 검출만 표시한다.
            imgsz=640,  # 학습 때와 같은 입력 크기를 사용한다.
            device=0,  # 첫 번째 NVIDIA GPU로 인식한다.
            verbose=False,  # 라이브러리의 사진별 기본 로그를 생략한다.
        )[0]  # 사진 한 장의 인식 결과를 가져온다.
        print(f'\n{image_path.name}: 객체 {len(result.boxes)}개 검출')  # 사진별 검출 개수를 출력한다.
        for box in result.boxes:  # 검출된 물체의 정보를 하나씩 출력한다.
            confidence = float(box.conf[0])  # 검출 신뢰도를 일반 숫자로 가져온다.
            x, y, w, h = box.xywh[0].tolist()  # 원본 사진 기준 중심 좌표와 크기를 픽셀로 가져온다.
            print(f'  {result.names[int(box.cls[0])]} 신뢰도 {confidence:.1%}, 중심 ({x:.1f}, {y:.1f}) px')  # 신뢰도와 위치를 출력한다.
        annotated = result.plot()  # 원본 사진 위에 검출 박스·클래스·신뢰도를 그린다.
        saved = cv2.imwrite(str(output / image_path.name), annotated)  # 표시된 결과를 사진으로 저장한다.
        if not saved:  # 파일 저장 성공 여부를 확인한다.
            raise SystemExit(f'결과 저장 실패: {image_path.name}')  # 저장 실패를 알리고 종료한다.
        if show:  # 화면 표시 옵션이 켜져 있을 때만 창을 연다.
            cv2.namedWindow('Strawberry car ball test', cv2.WINDOW_NORMAL)  # 크기를 조절할 수 있는 창을 만든다.
            cv2.resizeWindow('Strawberry car ball test', 540, 800)  # 세로 사진을 보기 편한 창 크기로 정한다.
            cv2.imshow('Strawberry car ball test', annotated)  # 검출 결과 사진을 화면에 표시한다.
            print('아무 키: 다음 사진 / Q 또는 Esc: 종료')  # 사진을 넘기거나 종료하는 방법을 안내한다.
            key = cv2.waitKey(0) & 0xFF  # 사용자의 키 입력을 기다린다.
            if key in (ord('q'), ord('Q'), 27):  # Q나 Esc 키를 눌렀는지 확인한다.
                break  # 나머지 사진 처리를 멈춘다.

    if show:  # 화면을 사용한 경우에만 창 정리를 실행한다.
        cv2.destroyAllWindows()  # 열려 있는 결과 창을 닫는다.
    print(f'\n결과 저장 폴더: {output}')  # 저장된 사진을 확인할 경로를 출력한다.
