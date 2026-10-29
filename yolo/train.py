import argparse  # 학습 횟수를 지정한다.
from pathlib import Path  # 실행 위치와 관계없이 파일 경로를 찾는 도구를 가져온다.

from ultralytics import YOLO  # 사전학습 모델 불러오기와 학습을 지원하는 YOLO를 가져온다.


if __name__ == '__main__':  # 이 파일을 직접 실행할 때만 학습을 시작한다.
    folder = Path(__file__).resolve().parent  # 현재 train.py가 있는 yolo 폴더를 찾는다.
    parser = argparse.ArgumentParser()  # 실행 옵션을 준비한다.
    parser.add_argument('--epochs', type=int, default=100)  # 전체 데이터를 반복 학습할 횟수다.
    parser.add_argument('--pretrained', type=Path,  # 시작할 사전학습 가중치다.
                        default=folder.parent / 'omx_ws' / 'yolo26n.pt')
    args = parser.parse_args()  # 옵션을 읽는다.
    if not args.pretrained.is_file():  # 임의 모델 다운로드를 막고 경로를 안내한다.
        raise SystemExit(f'사전학습 가중치가 없습니다: {args.pretrained} (--pretrained로 지정)')
    model = YOLO(str(args.pretrained))  # 사전학습 가중치를 불러온다.
    model.train(  # 딸기·자동차·공 사진과 정답 라벨로 모델을 추가 학습한다.
        data=str(folder / 'dataset_strawberry_car_ball' / 'data.yaml'),  # 3클래스 데이터 설정이다.
        epochs=args.epochs,  # 전체 학습 데이터를 지정한 횟수만큼 반복해서 학습한다.
        imgsz=640,  # YOLO가 처리할 입력 이미지 크기의 기준을 640픽셀로 정한다.
        batch=4,  # 한 번에 사진 4장씩 처리한다.
        device=0,  # 첫 번째 NVIDIA GPU로 학습한다.
        workers=0,  # 데이터를 읽을 별도 작업 프로세스를 만들지 않아 실행을 단순하게 한다.
        project=str(folder / 'runs'),  # 학습 결과를 yolo/runs 아래에 저장한다.
        name='strawberry_car_ball',  # 이미 있으면 YOLO가 -2, -3처럼 번호를 붙인다.
        hsv_v=0.5,  # 밝기를 무작위로 변화시켜 어둡거나 밝은 환경을 학습한다.
        degrees=25.0,  # 사진을 최대 좌우 25도 회전하며 박스도 함께 변환한다.
        scale=0.6,  # 크기를 약 0.4~1.6배로 바꿔 거리 변화에 대응한다.
        perspective=0.0003,  # 비스듬히 보는 상황을 작은 원근 변형으로 반영한다.
    )  # 학습 종료 시 가장 좋은 모델과 마지막 모델이 결과 폴더에 저장된다.
    print(f'최적 모델 저장 위치: {model.trainer.best}')  # 다음 단계에서 사용할 best.pt 경로를 출력한다.
