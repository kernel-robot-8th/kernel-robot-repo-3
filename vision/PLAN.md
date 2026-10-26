# vision_omx 작업 플랜

Pinky 도착 확인 → 물체 인식 → 좌표 전달 → 손목캠 2차 검증 → 집기.

담당 범위: 카메라 보정, Pinky 도착 판정, 검출 결과 → OMX 좌표 변환, 2차 검증과 재시도 흐름.
YOLO 학습은 YOLO 담당자가 맡는다. 이 작업은 YOLO 결과를 받아서 쓴다.

## 현장 구조

- 바닥에 고정한 판넬에 바구니를 달고, 물체는 바구니 안에 둔다.
- 바구니 자리는 고정이지만, 바구니 **안에서** 물체 위치는 바뀔 수 있다.
- Pinky는 정차 구역에 서고, OMX가 바구니에서 물체를 집어 Pinky 수레에 싣는다.

## 흐름

```
① Pinky 도착 확인 (상단캠 중 보이는 것, ArUco ID 20·21)
   ├─ 어긋남 → 보정량 전달 → 다시 확인 (최대 N회)
   └─ OK
② 물체 인식 (상단캠, YOLO) → 어느 바구니에 있는지
③ 좌표 전달 → OMX가 그 바구니의 등록 hover 자세로 이동
④ 2차 검증 (손목캠, YOLO)
   ├─ 맞음 → 집기
   ├─ 조금 어긋남 → 미세 보정 → ④ 다시
   └─ 아님 → ②로 돌아가 다시 찾기 (최대 M회, 넘으면 실패 보고)
```

## 인식 방법

| 대상 | 방법 | 이유 |
|---|---|---|
| 기준 마커 (0~9) | ArUco | 카메라 보정용. 고정돼 있어 가장 정확하다 |
| Pinky (20·21) | ArUco | 위치와 방향(yaw)을 정밀하게 재야 한다 |
| 물체 | YOLO | 바구니 안에서 위치·자세가 바뀐다 |

물체 마커(10·11)는 YOLO 모델을 받기 전 흐름 개발용 임시 수단이다. 검출기 출력 형식을 하나로 맞춰 두고, 나중에 `MarkerDetector`를 `YoloDetector`로 바꾼다.

```
검출기 공통 출력: 물체 이름, 중심 픽셀, bbox, 신뢰도, (각도)
   ├─ MarkerDetector  ← 지금
   └─ YoloDetector    ← 모델 받은 뒤
```

## 카메라 구성

| | 상단캠 3대 (고정) | 손목캠 1대 (OMX 손목 관절) |
|---|---|---|
| Pinky | ArUco로 위치·yaw → 도착 판정 | 사용 안 함 |
| 물체 | YOLO로 어느 바구니인지 판단 | YOLO bbox 중심 vs 그리퍼 중심 → 미세 보정 |
| 보정 | 정차 구역 바닥 기준 homography (기준 마커 4개 이상) | 그리퍼 중심 픽셀 + mm/px 1회 등록 |

- 상단캠 셋은 같은 역할이다. 도착 확인과 물체 인식은 카메라가 아니라 요청 종류로 나뉜다.
- 세 카메라 모두 같은 바닥 좌표계를 쓴다. 겹치는 구역에서 결과가 `agree_mm` 이상 다르면 `CAMERA_DISAGREE`로 보고한다.
- homography는 한 평면에서만 정확하다. Pinky 마커 높이에 맞춘 보정을 쓴다.

## 바구니 등록

- 바구니마다 OMX hover 자세를 OMX를 직접 움직여 한 번 저장한다(티칭). 바구니마다 높이가 달라도 각자 저장한 값을 쓴다.
- 상단캠 화면에서 바구니마다 영역(다각형)을 한 번 그려 둔다. YOLO bbox 중심이 어느 영역에 있는지로 바구니를 정한다.
- 설정 예: `baskets: {A: {hover_pose: [...], image_region: {top_a: [[u,v], ...]}}}`

## YOLO 담당자와 맞출 것

| 항목 | 제안 |
|---|---|
| 학습 시점 | 상단캠과 손목캠 **둘 다** 포함 |
| 넘겨받는 형태 | 가중치(`.pt`) + 클래스 목록. 추론은 `YoloDetector`에서 돌린다 (또는 담당자가 ROS 노드로 `vision_msgs/Detection2DArray` 발행) |
| 출력 | 클래스, 신뢰도, bbox(픽셀). 잡는 방향이 중요한 물체가 있으면 OBB 또는 segmentation |
| 클래스 이름 | 업무 서버의 `productId`와 같은 이름 |
| 신뢰도 기준 | 담당자 추천값을 `min_confidence`에 넣는다. 미만이면 `LOW_CONFIDENCE` → 다시 찾기 |
| 학습 데이터 | 카메라 설치 후 이쪽에서 실제 시점 영상을 찍어 넘긴다 (`poc_debug.py`의 `s` 키) |
| 일정 | 모델을 받기 전까지 마커로 흐름을 완성해 둔다 |

## 마커 ID 규칙

| ID | 용도 |
|---|---|
| 0~9 | 기준 마커. 정차 구역 바닥에 고정, 좌표 실측 (`reference_markers_mm`) |
| 10~19 | 물체 (YOLO 전 임시) |
| 20~29 | Pinky 윗면. 20 = robot1, 21 = robot2. 70 mm, 화살표가 Pinky 정면을 향하게 붙인다 |

## 단계

### 1. 상단캠 3대 장착·보정
- [x] 카메라별 설정(`cameras: {top_a, top_b, top_c}`)과 `--camera` 선택
- [x] 체크보드 렌즈 왜곡 보정 `calib_intrinsics.py` → `calib/<카메라>_intrinsic_<시각>.yaml`
- [x] `table_corners_mm` → `reference_markers_mm`(0~9 중 4개 이상) + 검증 마커 `check_markers_mm`
- [x] 카메라별 homography 파일 `calib/<카메라>_<시각>.yaml`, `load_latest(dir, camera)`
- [ ] top_a 실측 보정 (체크보드 → 기준 마커 배치·실측 → homography)
- [ ] top_b·top_c 연결 후 같은 절차
- [ ] 겹침 구역 카메라 간 차이 비교 (세 카메라 동시 실행 필요)
- 완료 기준: 아는 위치 5곳에서 오차 ±5 mm. 겹침 구역에서 카메라끼리 차이 ±10 mm.

### 2. 손목캠 준비
- 연결, 해상도 확인 (`wrist_camera.device`).
- hover 자세에서 그리퍼 중심 픽셀을 등록한다 (`calib/wrist_<시각>.yaml`).
- 마커 한 변 50 mm의 픽셀 수로 mm/px를 구한다.

### 3. Pinky 도착 확인 (`arrival.py`)
- `detector.detect()`가 중심과 함께 yaw도 돌려주게 한다.
- `station.dock_pose_mm: [x, y, yaw]`, `tol_xy_mm`, `tol_yaw_deg`.
- 결과: `ARRIVED` / `MISALIGNED(dx, dy, dyaw, Pinky 기준)` / `NOT_FOUND`.
- 완료 기준: 일부러 2~3 cm, 10° 어긋나게 세웠을 때 보정량이 맞는다.

### 4. 바구니 등록과 좌표 전달
- 검출기 공통 출력 타입(`Detection`)을 정하고 `MarkerDetector`를 거기에 맞춘다.
- 바구니 hover 자세 티칭, 상단캠 화면에 바구니 영역 등록.
- 검출 결과 → 바구니 → hover 자세를 OMX에 보낸다.
- OMX 어댑터와 맞출 명령: `move_to(hover pose)`, `grasp(pose)`.

### 5. 2차 검증 (`wrist_verify.py`)

| 검사 | 결과 |
|---|---|
| 물체가 안 보임 | `NOT_FOUND` → 다시 찾기 |
| 다른 물체 | `WRONG_OBJECT` → 다시 찾기 |
| 신뢰도 미달 | `LOW_CONFIDENCE` → 다시 찾기 |
| 그리퍼 중심과 거리 ≤ `tol_mm` | `OK` → 집기 |
| `tol_mm` 초과, `correct_max_mm` 이하 | `OFFSET` → 미세 보정 후 재검증 |
| `correct_max_mm` 초과 | `OFFSET_TOO_LARGE` → 다시 찾기 |

여러 프레임에서 안정적일 때만 판정한다 (예: 5프레임 중 4번).

### 6. 상태 머신 (`pick_flow.py`)
`CHECK_ARRIVAL → DETECT → MOVE_HOVER → VERIFY → (CORRECT ↺ VERIFY) → GRASP → DONE`
실패하면 `DETECT`로 돌아간다. 재시도 횟수를 넘으면 `RETRY_EXCEEDED`. OMX 없이 키보드로 "이동 완료"를 흉내 내서 먼저 테스트한다.

### 7. YOLO 연결
- 받은 가중치로 `YoloDetector`를 만들고 `MarkerDetector` 자리에 끼운다.
- 상단캠·손목캠 실제 영상에서 검출률과 신뢰도를 확인하고 `min_confidence`를 정한다.

### 8. ROS 이전
순수 계산 모듈을 ROS 패키지로 옮긴다(위치 미정). 노드는 연결만 맡는다. 같은 상단캠 노드를 3개 인스턴스로 띄우고(네임스페이스로 구분), 결과를 합치는 노드를 따로 둔다.

### 9. 통합 시연
10회 연속 실행. 성공률과 실패 사유를 기록한다.

## 진행 순서

1 → 2 → 3 → 4 → 5 → 6 → 7 → 8 → 9

- OMX 없이 먼저 할 수 있는 것: 1, 2(연결), 3, 5(손으로 손목캠 들고)
- OMX가 있어야 하는 것: 2(그리퍼 중심 등록), 4(티칭·이동), 6, 9
- YOLO 모델이 있어야 하는 것: 7. 그 전까지는 마커로 진행

## 결정 대기

- 판넬이 세워져 있는지 눕혀져 있는지, 바구니 개수, OMX가 모든 바구니에 닿는지
- 상단캠 3대의 정확한 배치와 겹침 범위
- Pinky 보정량을 받는 쪽: `pinky_fleet` Nav2 목표 재전송 / `cmd_vel` 미세 이동
- OMX 어댑터 메시지 형식
- YOLO 담당자와 위 표의 항목 합의
- 비전 책임 범위 확정: 비전이 Pinky 위치를 측정하고 보정량을 내는 것까지 담당하는지
- 관제 쪽 카메라(Head·Joint, 로봇팔 장착)와 이 플랜의 상단캠·손목캠이 어떻게 대응하는지
