import argparse  # 시작 모델·데이터·장치를 지정한다.
from pathlib import Path  # 실행 위치와 관계없이 파일 경로를 찾는다.

from ultralytics import YOLO  # 학습한 모델에서 이어서 학습한다.


if __name__ == '__main__':  # 직접 실행할 때만 학습을 시작한다.
    folder = Path(__file__).resolve().parent  # yolo 폴더다.
    parser = argparse.ArgumentParser()  # 실행 옵션을 준비한다.
    parser.add_argument('--weights', type=Path, required=True)  # 이어서 학습할 모델이다. 보통 train.py 결과 best.pt다.
    parser.add_argument('--data', type=Path, default=folder / 'dataset_head' / 'data.yaml')  # head_autolabel.py 결과다.
    parser.add_argument('--epochs', type=int, default=100)  # 새 시점에 맞추는 추가 학습이라 처음 학습보다 짧다.
    parser.add_argument('--device', default='0')  # 첫 번째 NVIDIA GPU다. GPU가 없으면 cpu.
    args = parser.parse_args()  # 옵션을 읽는다.
    if not args.weights.is_file():  # 시작 모델이 있는지 확인한다.
        raise SystemExit(f'시작 모델이 없습니다: {args.weights}')
    model = YOLO(str(args.weights))  # 기존 학습 결과에서 시작해 이미 배운 물체 모양을 이어 쓴다.
    model.train(  # 상단 카메라 사진으로 추가 학습한다.
        data=str(args.data),  # 3클래스 데이터 설정이다.
        epochs=args.epochs,
        patience=30,  # 30회 동안 나아지지 않으면 멈춘다.
        imgsz=640,  # train.py와 같은 입력 크기다.
        batch=16,  # 사진이 적어 한 번에 16장씩 처리한다.
        device=args.device,
        workers=4,
        project=str(folder / 'runs'),  # 결과를 yolo/runs 아래에 저장한다.
        name='head_camera_finetune',  # camera_test.py가 자동으로 고르는 strawberry_car_ball* 와 섞이지 않는 이름이다.
        # 상단 카메라는 선반을 정면에서 보고 물체가 선반 위에 서 있다. 회전·상하 뒤집기는 거의 쓰지 않는다.
        degrees=5.0,  # 카메라가 살짝 기운 정도만 회전한다.
        fliplr=0.5,  # 좌우는 뒤집어도 같은 장면이다.
        scale=0.5,  # 크기를 0.5~1.5배로 바꾼다.
        translate=0.1,  # 위치를 조금 옮긴다.
        mosaic=1.0,  # 사진 4장을 이어 붙여 여러 물체 장면을 만든다.
        close_mosaic=10,  # 마지막 10회는 실제 화면과 같은 한 장 사진으로 학습한다.
        hsv_h=0.015, hsv_s=0.7, hsv_v=0.4,  # 조명·색감 차이에 대응한다 (ultralytics 기본값).
    )
    print(f'최적 모델 저장 위치: {model.trainer.best}')  # camera_test.py --weights 로 확인한다.
