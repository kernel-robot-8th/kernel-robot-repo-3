import argparse  # 학습 횟수를 지정한다.
from pathlib import Path  # 실행 위치와 관계없이 파일 경로를 찾는 도구를 가져온다.

import albumentations as A  # 흐림·노이즈·노출 같은 픽셀 단위 증강을 만든다.
from ultralytics import YOLO  # 사전학습 모델 불러오기와 학습을 지원하는 YOLO를 가져온다.


if __name__ == '__main__':  # 이 파일을 직접 실행할 때만 학습을 시작한다.
    folder = Path(__file__).resolve().parent  # 현재 train.py가 있는 yolo 폴더를 찾는다.
    parser = argparse.ArgumentParser()  # 실행 옵션을 준비한다.
    parser.add_argument('--epochs', type=int, default=150)  # 증강이 강해 기존 100회보다 늘린다.
    parser.add_argument('--pretrained', type=Path,  # 시작할 사전학습 가중치다.
                        default=folder.parent / 'omx_ws' / 'yolo26n.pt')
    args = parser.parse_args()  # 옵션을 읽는다.
    if not args.pretrained.is_file():  # 임의 모델 다운로드를 막고 경로를 안내한다.
        raise SystemExit(f'사전학습 가중치가 없습니다: {args.pretrained} (--pretrained로 지정)')
    pixel_augmentations = [  # 박스 위치는 그대로 두고 화질만 바꾸는 변형이다. 매번 무작위로 적용된다.
        A.OneOf([A.MotionBlur(blur_limit=(3, 9)),  # 팔이 움직일 때 생기는 흔들림이다.
                 A.GaussianBlur(blur_limit=(3, 7)),  # 초점이 맞지 않은 화면이다.
                 A.MedianBlur(blur_limit=5)], p=0.3),  # 셋 중 하나를 30% 확률로 적용한다.
        A.GaussNoise(std_range=(0.02, 0.08), p=0.25),  # 어두운 곳의 카메라 잡음이다.
        A.RandomBrightnessContrast(brightness_limit=0.35, contrast_limit=0.35, p=0.5),  # 대비 변화다.
        A.RandomGamma(gamma_limit=(50, 160), p=0.3),  # 하얗게 날아가거나 어둡게 눌린 노출이다.
        A.CLAHE(p=0.05),  # 국소 대비를 강조한 화면이다.
        A.ToGray(p=0.05),  # 색 정보가 약한 화면에서도 모양으로 찾게 한다.
        A.ImageCompression(quality_range=(40, 95), p=0.3),  # MJPEG 스트림의 압축 손상이다.
    ]
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
        # 기하 변형: 사진을 미리 만들지 않고 매 학습마다 무작위로 바꿔서 같은 사진도 매번 다르게 본다.
        degrees=180.0,  # 어느 방향으로 누워 있어도 찾도록 -180~180도 전체를 회전한다.
        flipud=0.5,  # 절반 확률로 상하를 뒤집어 거꾸로 된 물체를 학습한다.
        fliplr=0.5,  # 절반 확률로 좌우를 뒤집는다.
        perspective=0.001,  # 비스듬히 내려다보는 모습을 원근 변형으로 반영한다.
        shear=10.0,  # 사선으로 기울어진 모습을 최대 10도 비튼다.
        scale=0.8,  # 크기를 약 0.2~1.8배로 바꿔 멀리 있는 작은 물체도 학습한다.
        translate=0.2,  # 위치를 옮겨 화면 가장자리에서 잘린 물체도 학습한다.
        mosaic=1.0,  # 사진 4장을 이어 붙여 여러 물체·겹침·작은 물체 장면을 만든다.
        mixup=0.1,  # 10% 확률로 두 사진을 겹쳐 가려지고 애매한 장면을 만든다.
        # 색 변형: 조명과 카메라 색감 차이에 대응한다.
        hsv_h=0.02,  # 색조를 조금 바꾼다. 딸기 빨강이 다른 색이 되지 않을 정도다.
        hsv_s=0.7,  # 채도를 크게 바꿔 색이 바랜 화면을 학습한다.
        hsv_v=0.6,  # 밝기를 크게 바꿔 어둡거나 과노출된 화면을 학습한다.
        augmentations=pixel_augmentations,  # 위에서 만든 흐림·노이즈·노출·압축 변형을 쓴다.
    )  # 학습 종료 시 가장 좋은 모델과 마지막 모델이 결과 폴더에 저장된다.
    print(f'최적 모델 저장 위치: {model.trainer.best}')  # 다음 단계에서 사용할 best.pt 경로를 출력한다.
