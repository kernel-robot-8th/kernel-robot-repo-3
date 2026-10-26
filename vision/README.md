# vision_omx PoC: 카메라로 위치 보고 OMX에 좌표 넘기기

ROS 없이 OpenCV만으로 도는 실험 폴더. 관제 서버(`src/control_station`)와 연결하지 않는다. 비전은 관제 MVP 범위 밖이라([시스템 명세](../docs/spec/system-spec.md) 1절) 여기서 따로 검증한다.

```
상단 카메라 → ArUco 검출(픽셀) → homography → 작업대(mm) → 오프셋·회전 → OMX base(m) → [Space] 전송
```

## 흐름과 현황

```
상단 카메라 (/dev/video4, 1280x720)
   │  camera.open_camera()
   ▼
렌즈 왜곡 펴기              ← calib/top_a_intrinsic_*.yaml      완료
   │  calibration.Undistorter
   ▼
물체 검출                   ← models/*.pt (YOLO)                 동작, 정확도는 아래 "YOLO 모델"
   │  YoloDetector.detect() → Detection(이름, 중심 픽셀, bbox, 신뢰도)
   ▼
어느 칸 + 칸 안 좌우 위치  ← calib/top_a_baskets_<시각>.yaml    완료 (6칸 등록)
   │  baskets.locate(), lateral_position() → 0.0 왼쪽 ~ 1.0 오른쪽
   ▼
OMX 접근 자세               ← 칸별 왼쪽·가운데·오른쪽 티칭 값      TODO (OMX 연결 필요)
   │  좌우 위치로 이웃한 두 자세 사이를 보간
   ▼
손목캠 미세 보정 → 집기                                          TODO (PLAN 5단계)
```

좌표는 칸 단위로 정한다 (PLAN "바구니 등록"). 상단캠은 선반을 거의 수평으로 보고 물체가 높이가 다른 여러 칸에 놓여서, 바닥 평면 하나를 가정하는 homography로는 mm 좌표 오차가 크다. 한 칸에 물체가 2~3개(왼쪽·가운데·오른쪽) 들어가므로 상단캠은 칸과 함께 칸 안 좌우 위치(0~1)를 낸다. 정면 시점이라 좌우는 잘 보이지만 앞뒤 깊이는 거의 안 보여서 좌우만 구한다. OMX는 칸마다 왼쪽·가운데·오른쪽 3개 자세를 티칭해 두고, 좌우 위치로 이웃한 두 자세 사이를 보간해 물체 앞으로 간다. 깊이와 미세 위치는 손목캠으로 맞춘다. 물체가 늘 세 자리 중 하나에만 놓이면 보간 대신 가장 가까운 자세(`position_label`)를 쓴다. homography 경로(`poc_debug.py`, `geometry.py`)는 바닥에 놓인 물체용으로 남겨 둔다.

남은 연결:

- 칸별 OMX 자세(왼쪽·가운데·오른쪽) 티칭과 저장. OMX 명령이 관절 각도인지 (x, y, z)인지, 관절값을 어떻게 읽는지 정해야 한다.
- `Detection` → 칸·좌우 위치 → 보간한 자세 → OMX 어댑터 호출.
- 손목캠 그리퍼 중심 등록과 미세 보정.

## 구성

| 경로 | 내용 |
|---|---|
| `config/poc.yaml` | 카메라 번호, 모서리 마커 위치, OMX 원점·방향·도달 거리, 물체별 집기 높이 |
| `vision_omx/detector.py` | ArUco 마커 검출 (OpenCV 4.6 / 4.7+ 모두 지원) |
| `vision_omx/calibration.py` | 체크보드 → 렌즈 왜곡(intrinsic), 기준 마커 4개 이상 → homography. 카메라별 저장·불러오기 |
| `vision_omx/yolo_detector.py` | YOLO 물체 검출 → 공통 출력 `Detection`(이름, 중심 픽셀, bbox, 신뢰도) |
| `vision_omx/baskets.py` | 칸(바구니) 영역 저장·불러오기, 점이 어느 칸에 있는지와 칸 안 좌우 위치(0~1, left/center/right) |
| `vision_omx/geometry.py` | 픽셀 → 작업대 → OMX 좌표 변환, 도달 범위 검사 |
| `vision_omx/types.py` | `PickTarget`, `Failure`, 실패 코드 |
| `calib_intrinsics.py` | 체크보드로 카메라별 렌즈 왜곡 보정 |
| `poc_debug.py` | OpenCV 디버그 창. 왜곡을 편 프레임에서 검출·homography 보정 |
| `yolo_debug.py` | YOLO 인식 디버그 창. 왜곡을 편 프레임에서 검출, homography가 있으면 작업대 좌표(mm)도 표시 |
| `register_baskets.py` | 상단캠 화면에 칸 영역을 마우스로 등록 → `calib/<카메라>_baskets_<시각>.yaml` |
| `markers/generate_markers.py` | 마커 PNG + 인쇄용 PDF 생성. `markers_a4.pdf`(기준 0~9, 물체 10·11, 50 mm), `pinky_markers_a4.pdf`(Pinky 20·21, 70 mm, 앞쪽 화살표) |
| `patterns/checkerboard_9x7_25mm_A4.pdf` | 내부 캘리브레이션용 체크보드 (A4 가로, 9×7칸, 한 칸 25 mm → `--cols 8 --rows 6`). "페이지에 맞춤"을 끄고 100%로 인쇄한 뒤 아래쪽 50 mm 막대를 자로 재서 배율을 확인한다 |
| `models/` | YOLO 가중치. 아래 "YOLO 모델" |
| `requirements.txt` | 필요한 Python 패키지 |
| `tests/` | 카메라 없이 도는 pytest |

`calib/`, `recordings/`, `markers/out/`은 실행하면 생기는 결과물이고 `datasets/`는 학습 사진이라 git에 올리지 않는다. YOLO 가중치 `models/*.pt`는 git에 함께 올린다.

## 실행

```bash
cd ~/kernel-robot-repo-3/vision
pip install -r requirements.txt          # 시스템에 이미 있으면 생략
python3 -m pytest -q tests              # 단위 테스트
python3 markers/generate_markers.py     # markers/out/ 에 PNG와 인쇄용 PDF 2개
python3 calib_intrinsics.py --camera top_a --cols 8 --rows 6 --square-mm 25   # 렌즈 왜곡 보정
python3 poc_debug.py --camera top_a     # 디버그 창
python3 register_baskets.py --camera top_a --names L_top L_bottom C_top C_bottom R_top R_bottom   # 칸 등록
python3 yolo_debug.py --camera top_a    # YOLO 인식 창 (기본 모델 models/best_top_a.pt)
python3 yolo_debug.py --camera top_a --model models/best.pt   # 다른 가중치로
```

`register_baskets.py` 키: 클릭으로 꼭짓점 추가, `Enter` 이 칸 완료, `z` 마지막 점 취소, `r` 이 칸 다시, `s` 저장, `q` 저장 없이 종료. `--names`에 적은 순서대로 그린다. 칸이 겹치면 먼저 등록한 칸으로 판정한다.

`yolo_debug.py` 키: `u` 왜곡 펴기 on/off, `+`/`-` 신뢰도 기준, `s` 원본 프레임 저장(박스 없음, 학습 데이터용), `q` 종료. 칸 영역이 있으면 bbox 중심이 든 칸과 칸 안 좌우 위치(예: `C_bottom 0.62 (center)`)를, homography가 있으면 작업대 좌표(mm)도 표시한다. 칸 영역과 homography는 만들 때와 같은 프레임(왜곡 폄 여부, intrinsic 버전)일 때만 쓴다. 다르면 화면에 경고가 뜨니 다시 등록한다.

시스템 Python에 pip이 없어 team10-ai PC에서는 YOLO용 가상환경 `~/yolo-venv`(ultralytics, CPU torch)를 쓴다. 셸에 잡힌 ROS `PYTHONPATH`가 pytest 플러그인과 충돌하므로 빼고 실행한다.

```bash
env -u PYTHONPATH ~/yolo-venv/bin/python yolo_debug.py --camera top_a
```

카메라는 한 프로그램만 열 수 있다. `device 4를 열 수 없습니다`가 나오면 `fuser -v /dev/video4`로 붙잡고 있는 프로세스를 확인한다.

## YOLO 모델

클래스는 `0 strawberry`, `1 car`, `2 ball`. 모델: YOLO26n, 입력 640.

| 파일 | 학습 데이터 | 상태 |
|---|---|---|
| `models/best.pt` | YOLO 담당자(devwoong) 원본 데이터셋 `dataset_strawberry_car_ball_expanded`. 2026-10-24, 10 epoch | top_a 시점에서는 거의 검출 못 함 (신뢰도 0.3 미만) |
| `models/best_top_a.pt` | `best.pt`에서 이어서 top_a 촬영 74장으로 추가 학습. 2026-10-26, 증강 켬 | top_a 시점용. 다른 시점(로봇팔 카메라 등)에서는 원본보다 못할 수 있다 |

- `best.pt`가 top_a에서 안 되는 이유: 원본 학습에서 증강(mosaic, fliplr, scale, hsv)이 모두 꺼져 있어 학습 장면만 외웠다. 다시 학습할 때는 ultralytics 기본 증강을 켜 둔다.
- 추가 학습 데이터(`datasets/top_a_20261026/`)는 2초 간격으로 60장을 찍었다. 공·자동차는 COCO YOLO11x로, 딸기는 HSV 빨강 영역으로 자동 라벨을 붙인 뒤 사람이 검수했다. 손에 든 물체나 라벨이 빠진 프레임을 빼고 37장을 남겼고, 원본과 왜곡 편 사본을 함께 넣었다. 5장 단위 블록으로 train 58 / val 16장으로 나눴다.
- 검증 사진도 같은 날 같은 선반에서 찍어서 val 수치가 실제 성능보다 높게 나온다. 물체를 새로 배치해 `yolo_debug.py`로 직접 확인한다.

**TODO**: 원본 데이터셋을 받아 top_a 데이터와 합쳐 다시 학습한다. 그래야 원본 시점과 top_a 시점을 한 모델로 둘 다 볼 수 있다. 합친 모델이 나오면 `best.pt`를 교체한다. 촬영·자동 라벨·데이터셋 생성 스크립트는 아직 레포에 없다.

## 상단캠 보정 (카메라마다)

1. **렌즈 왜곡**: `calib_intrinsics.py`. `--cols/--rows`는 칸 수가 아니라 안쪽 모서리 수(10x7칸 보드 → 9 6), `--square-mm`은 칸 한 변을 자로 잰 값. 보드를 가운데·네 모서리·가장자리에 두루, 기울여 가며 15장 이상 `Space`로 담고 `c`. RMS 0.5 px 이하면 양호. `u`로 직선이 곧게 펴졌는지 본다. → `calib/<카메라>_intrinsic_<시각>.yaml`
2. **바닥 homography**: 기준 마커(ID 0~9) 4개 이상을 화면 가장자리 쪽에 넓게 붙이고 중심 좌표를 실측해 `reference_markers_mm`에 적는다. 보정에 안 쓸 검증 마커도 1개 이상 실측해 `check_markers_mm`에 적는다. `poc_debug.py`에서 `c`. → `calib/<카메라>_<시각>.yaml`
3. **확인**: 화면의 `check #ID` 오차가 ±5 mm 안인지 본다. `ref err max`가 `drift_warn_mm`를 넘으면 카메라가 밀린 것이니 2를 다시 한다.

intrinsic을 다시 보정하면 픽셀 좌표가 바뀌므로 homography도 다시 만들어야 한다(화면에 빨간 경고가 뜬다).

## 실험 순서

1. 위 "상단캠 보정"을 마친다. 원점은 ID0 중심.
2. OMX 바닥 중심의 작업대 좌표와 정면 방향을 재서 `omx.origin_on_table_mm`, `omx.yaw_deg`에 적는다.
3. 물체 마커(ID 10, 11)를 작업대에 놓고 화면의 작업대 좌표와 자로 잰 값을 비교한다. 오차를 기록한다.
4. OMX 연결 후 `send_to_omx()`를 어댑터 호출로 바꾸고, `Space`로 OMX가 물체 바로 위로 가는지 확인한다.

## 좌표계

- 작업대: 원점 ID0 중심, x는 ID0→ID1, y는 ID0→ID3, 단위 mm
- OMX base: 원점 OMX 바닥 중심, +X 정면, +Z 위, 단위 m. 작업대 x축에서 정면까지 `yaw_deg`
- 깊이 카메라가 없어 높이(z)는 물체별 상수 `grasp_z_mm`를 쓴다. 카메라나 작업대가 움직이면 다시 보정한다.
