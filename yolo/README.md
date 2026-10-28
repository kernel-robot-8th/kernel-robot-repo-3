# 딸기·자동차·공 YOLO 검출 모델

로봇팔 카메라 화면에서 **딸기(0)·자동차(1)·공(2)** 을 물체마다 각각 박스로 찾는 YOLO 모델의 라벨링·학습·테스트 코드다.
관제 서버와는 독립된 Python 스크립트이며, 결과물은 `best.pt` 모델 파일 하나다.

## 준비

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -r yolo/requirements.txt
```

- NVIDIA GPU 기준이다. 스크립트는 `device=0`(첫 번째 GPU)을 사용한다.
- 사전학습 가중치 `yolo26n.pt`(Ultralytics YOLO26 nano)를 미리 받아 두고 `train.py --pretrained`로 지정한다. 기본값은 `../omx_ws/yolo26n.pt`다.
- 사진·데이터셋·모델(`.pt`)은 용량이 커서 git에 넣지 않는다 (`yolo/.gitignore`).

## 폴더 구조

```
video/                                  원본 사진 (저장소 밖에서 따로 공유)
  KakaoTalk_20261022_201351859_frames/  딸기 47장 + 라벨 txt
  KakaoTalk_20261022_201218532_frames/  자동차 44장 + 라벨 txt
  ball_frames/                          공 39장 + 라벨 txt
yolo/
  camera_frames/                        camera_label.py로 찍은 여러 물체 사진 + 라벨
  dataset_strawberry_car_ball/          prepare_data.py 결과 (train/val)
  dataset_strawberry_car_ball_expanded/ prepare_augmented.py 결과
  runs/                                 학습 결과 (weights/best.pt)
  test_results/                         테스트 결과 사진
```

라벨은 YOLO 형식이다. 사진과 같은 이름의 txt에 박스마다 한 줄씩 `클래스 중심x 중심y 너비 높이`(0~1 비율)를 적는다.

## 전체 순서

```bash
python yolo/camera_label.py        # 1) 카메라로 여러 물체 사진 찍고 라벨링
python yolo/prepare_data.py        # 2) 원본 + 카메라 사진을 폴더별 8:2로 train/val 분할
python yolo/prepare_augmented.py   # 3) 밝기·크기·회전·크롭 변형 사진 생성
python yolo/train.py               # 4) 사전학습 모델에서 기본 학습 100회
python yolo/train_expanded.py      # 5) 4)의 결과에서 변형 사진으로 추가 학습 20회
python yolo/camera_test.py         # 6) 카메라 실시간 검출로 확인
```

| 파일 | 하는 일 |
|---|---|
| `label.py` | 폴더의 사진을 한 장씩 띄워 물체 1개를 드래그로 라벨링한다. 파일 위쪽의 `class_id`·폴더를 바꿔 쓴다. |
| `camera_label.py` | 카메라 실시간 화면에서 Space로 멈추고, 물체마다 드래그한 뒤 숫자키로 종류를 정해 S로 저장한다. |
| `prepare_data.py` | 원본 폴더별로 마지막 약 20%를 검증용으로 나누고 `data.yaml`을 만든다. 라벨 형식과 클래스 번호를 검사한다. |
| `prepare_augmented.py` | 사진마다 밝기 5 × 크기 9 × 회전 3 = 135가지 변형과, 물체마다 일부가 화면 끝에서 잘린 크롭 8가지를 만든다. 박스도 같이 변환한다. |
| `train.py` | 사전학습 가중치에서 학습한다. 학습 중 밝기·회전·크기·원근 증강을 무작위로 적용한다. |
| `train_expanded.py` | 가장 최근 `train.py` 결과에서 이어서 변형 사진으로 학습한다. 이미 변형된 사진이라 학습 중 증강은 끈다. |
| `test.py` | 사진 한 장이나 폴더를 검출해 결과 사진을 `test_results/`에 저장하고, 중심 좌표를 출력한다. |
| `camera_test.py` | 카메라 화면에서 매 프레임 검출해 박스·종류·신뢰도를 실시간으로 표시한다. S로 화면을 저장한다. |

- `train_expanded.py`, `test.py`, `camera_test.py`는 가장 최근 모델을 자동으로 고른다. 직접 지정하려면 `--weights 경로/best.pt`를 붙인다.
- 같은 이름의 결과 폴더가 있으면 YOLO가 `-2`, `-3`처럼 번호를 붙인다.
- 학습 횟수는 `--epochs`로 바꾼다. 변형 사진이 약 1.5만 장이라 `train_expanded.py`는 1회에 약 8~12분 걸린다 (RTX 4060 Laptop 기준).

## 카메라 라벨링 (`camera_label.py`)

```bash
python yolo/camera_label.py             # 외장 USB 카메라 (/dev/video0)
python yolo/camera_label.py --camera 2  # 다른 카메라 번호
```

| 키 | 동작 |
|---|---|
| Space | 실시간 화면 멈추기 |
| 마우스 드래그 | 박스 그리기 |
| 0 / 1 / 2 | 방금 그린 박스의 종류 (딸기 / 자동차 / 공) |
| Z | 마지막 박스 지우기 |
| S | 사진과 라벨 저장 후 실시간 화면으로 |
| C | 저장하지 않고 실시간 화면으로 |
| Q / Esc | 종료 |

여러 물체가 함께 나온 카메라 사진이 실제 사용 화면과 가장 비슷하므로 주 학습 데이터로 삼는다.
물체 위치·간격·겹침·카메라 거리를 바꿔 가며 50장 이상 찍는 것을 목표로 한다.

## 변형 사진 (`prepare_augmented.py`)

- **밝기** 0.4 ~ 1.6배, **회전** -20° / 0° / 20°
- **크기** 0.05 ~ 1.5배. 0.05·0.1·0.2·0.3은 아주 먼 거리로, 640 화면에서 물체가 약 10~60px로 보인다. 빈 공간은 원본의 바닥 부분을 반복해 채운다.
- **크롭**: 물체마다 상·하·좌·우 중 한쪽을 잘라 40% 또는 70%만 보이게 하고, 나머지 방향의 배경은 남긴다. 화면 안의 다른 물체도 보이는 만큼 정답에 들어간다. 파일 이름은 `crop_<방향><남은 비율>_b<물체 번호>_원본이름.jpg`다.
- 원본과 그 변형이 train과 val에 섞이지 않도록 분할한 뒤에 변형한다.

## 현재 결과 (2026-10-28)

`strawberry_car_ball_expanded-6` (기존 사진 130장 + 카메라 사진 5장, 추가 학습 10회)

- 변형 val 검증: mAP50 0.969, mAP50-95 0.742
- 카메라 사진 5장에서 딸기·자동차·공 3개를 모두 각각 검출했다 (신뢰도 0.88~0.94). 다만 5장 중 4장은 학습에 쓴 사진이다.

검증 사진은 학습 사진과 같은 영상·장면에서 나왔으므로 이 점수를 일반 성능으로 해석하지 않는다.
실제 성능은 새로 배치한 물체를 `camera_test.py`로 확인한다.

## 알려진 한계

- 원본 딸기 라벨이 몸통 아래의 초록 끈까지 포함한다. 몸통 기준으로 다시 라벨링해야 박스가 정확해진다.
- 카메라 사진이 아직 5장이라 선반 배경·실제 거리에 대한 학습이 부족하다.
- 10px 정도로 작은 물체는 YOLO26n·입력 640의 한계에 가깝다. 필요하면 `imgsz`를 키우는 것을 검토한다.
- 클래스를 늘릴 때는 `camera_label.py`의 `classes`, `prepare_data.py`의 `class_ids`·`data.yaml` 이름을 같이 고친다. 기존 번호는 바꾸지 않고 뒤에 붙인다.

## 다른 곳에서 모델 쓰기

`runs/<최신 폴더>/weights/best.pt` 파일 하나만 넘기면 된다. 클래스 이름이 파일 안에 들어 있다.
받는 쪽은 `ultralytics>=8.4`가 필요하다 (YOLO26 지원).
