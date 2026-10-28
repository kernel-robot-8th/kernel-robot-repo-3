import argparse  # 학습 횟수와 배치 크기를 지정한다.
from pathlib import Path  # 모델과 데이터 경로를 찾는다.

from ultralytics import YOLO  # 학습과 모델 저장 기능을 가져온다.


if __name__ == '__main__':  # 직접 실행할 때만 학습한다.
    root = Path(__file__).resolve().parent  # yolo 폴더다.
    parser = argparse.ArgumentParser()  # 실행 옵션을 준비한다.
    parser.add_argument('--epochs', type=int, default=20)  # 20회 추가 학습한다.
    parser.add_argument('--batch', type=int, default=16)  # GPU에서 16장씩 학습한다.
    parser.add_argument('--weights', type=Path)  # 시작 가중치다. 생략하면 최근 train.py 결과다.
    args = parser.parse_args()  # 옵션을 읽는다.
    candidates = [p for p in (root / 'runs').glob('strawberry_car_ball*/weights/best.pt')
                  if 'expanded' not in p.parts[-3]]  # train.py가 만든 기본 학습 결과만 고른다.
    weights = args.weights or (max(candidates, key=lambda p: p.stat().st_mtime) if candidates else None)
    if weights is None or not weights.is_file():  # 임의 모델 다운로드를 방지한다.
        raise SystemExit('먼저 python yolo/train.py로 기본 학습을 하세요.')  # 필요한 단계를 알린다.
    print(f'시작 모델: {weights}')  # 어떤 모델에서 추가 학습하는지 확인한다.
    model = YOLO(str(weights))  # 기존에 학습한 모델을 시작점으로 사용한다.
    model.train(  # 파일로 생성한 변형 학습 사진으로 추가 학습한다.
        data=str(root / 'dataset_strawberry_car_ball_expanded' / 'data.yaml'),  # 확장 train과 val
        epochs=args.epochs, batch=args.batch, imgsz=640,  # 학습 횟수와 입력 크기다.
        device=0, workers=2,  # NVIDIA GPU와 읽기 작업자 두 개를 사용한다.
        project=str(root / 'runs'), name='strawberry_car_ball_expanded',  # 별도 결과 폴더다.
        hsv_h=0.0, hsv_s=0.0, hsv_v=0.0,  # 이미 변형된 사진에 색상 증강을 겹치지 않는다.
        degrees=0.0, translate=0.0, scale=0.0, shear=0.0, perspective=0.0,  # 기하 증강도 끈다.
        fliplr=0.0, flipud=0.0, mosaic=0.0, mixup=0.0, cutmix=0.0, copy_paste=0.0,
        augmentations=[],  # 추가 Albumentations 변형도 끈다.
        seed=0, deterministic=True,  # 실험 설정을 재현할 수 있게 고정한다.
    )  # 학습 중 변형 val의 지표로 최적 가중치를 선택한다.
    print(f'최적 모델 저장 위치: {model.trainer.best}')  # 테스트할 가중치 경로를 출력한다.
