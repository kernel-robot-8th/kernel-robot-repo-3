# 카메라 내부 파라미터 보정 초안

`scripts/calibrate_camera_intrinsics.py`는 저장된 체커보드 영상으로 OpenCV 카메라 내부 파라미터와 왜곡 계수를 추정합니다. OpenCV는 기존 `requirements.txt`에 포함되어 있으므로 새 외부 의존성을 추가하지 않습니다. 이 도구는 파일만 읽고 결과 JSON을 쓸 뿐 카메라 장치를 열거나 로봇에 연결하지 않습니다.

## 입력 준비

- 보정할 카메라 한 대에서 동일한 해상도로 촬영한 체커보드 이미지를 준비합니다.
- `--columns`, `--rows`는 체커보드 칸 수가 아니라 **내부 교차점 수**입니다.
- `--square-size`와 `--unit`에는 인쇄물에서 잰 실제 한 칸 크기와 단위를 넣습니다.
- 스크립트는 서로 다른 각도·거리·화면 위치에서 코너를 찾은 이미지가 최소 6장 있어야 계산을 진행합니다. 이 최소 조건은 보정 품질을 보증하지 않으며, 화면을 넓게 덮는 다양한 자세로 더 많이 촬영해야 합니다.
- 해상도가 다르거나 체커보드 코너를 찾지 못한 입력은 결과의 `rejected_images`에 이유와 함께 남깁니다.
- 각 입력 이미지는 regular non-symlink file이어야 하며, 파일당 최대 크기는 64 MiB입니다. 더 큰 파일과 symlink는 보정 view로 쓰지 않고 `rejected_images`에 남깁니다.

## 실행

```bash
source .venv/bin/activate
python scripts/calibrate_camera_intrinsics.py ./calibration-images/head \
  --camera-id head_camera \
  --pixel-transform-id opencv-v4l2-capture-no-app-geometry-v1 \
  --columns 9 --rows 6 --square-size 25 --unit mm
```

예시의 보드 크기는 설명용입니다. 실제 인쇄 보드의 내부 코너 수와 한 칸 크기로 바꿔야 합니다. 기본 출력은 `.control-station/camera-calibrations/{camera_id}.json`이며 `--output`으로 바꿀 수 있습니다. 지정한 출력 파일이 이미 있으면 어떤 calibration/projection CLI도 기존 산출물을 교체하지 않습니다. 새 경로를 지정해야 합니다.

## 결과와 제한

내부 보정 JSON schema v2에는 camera ID와 `pixel_transform_id`, 원본 이미지 크기, checkerboard 내부 코너 수·실측 칸 크기·단위, camera matrix, distortion coefficients, OpenCV 버전, RMS 및 이미지별 재투영 오차가 기록됩니다. 각 승인 이미지의 SHA-256은 OpenCV에 decode한 바로 그 파일 bytes에서 계산해 기록하며, byte-identical 복제 이미지는 추가 view로 세지 않습니다. calibration/observation CLI에서 `--output`을 해당 실행의 입력 이미지나 JSON과 같은 파일(경로 alias 및 기존 hard link 포함)로 지정하면 덮어쓰지 않고 거부합니다. 모든 calibration 산출물도 출력 경로에 파일이 이미 있으면 원자적으로 덮어쓰지 않고 거부합니다. 산출물은 `DRAFT_ONLY`, review-required, `usable_for_robot_control: false`이고 후보 API나 로봇 제어 경로에서 읽지 않습니다.

이 계산은 렌즈 내부 파라미터만 추정합니다. 카메라 장착 위치(extrinsic), 카메라와 OMX 기준 frame 간 변환, 선반 평면/깊이, 좌표 허용 오차, IK 또는 안전한 집기 목표를 제공하지 않습니다. 카메라 해상도·crop·회전 설정이 바뀌면 결과를 그대로 재사용할 수 있다고 가정하지 않습니다. RMS 숫자만으로 보정을 승인하지 말고, 원본 이미지와 코너 분포를 검토한 뒤 알려진 기준점으로 오차를 확인해야 합니다.

이 결과는 천장 카메라 주차 판정(`cpp/parking_vision`)과 정면 카메라 선반 위치·칸 영역 보정([shelf-pose-calibration.md](shelf-pose-calibration.md))에서 읽습니다. 선반 평면 투영과 관측 후보 2D 좌표 초안 도구는 좌표 기반 팔 제어를 뺀 2026-11-11에 지웠습니다.
