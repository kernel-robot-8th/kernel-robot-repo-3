# 캘리브레이션 파일 지도

카메라마다 어떤 스크립트로 어떤 보정 파일을 만들고, 그 파일이 어디에 저장되는지 정리합니다. 각 도구의 상세 절차는 표에 링크한 문서를 따릅니다.

보정 결과와 원본 사진은 모두 `.control-station/` 아래에 저장됩니다. 2026-11-12부터 다른 PC에서도 같은 판정을 하도록, 지금 쓰는 결과 파일은 Git에 올립니다.

| Git에 올림 | Git에서 제외 |
|---|---|
| `camera-calibrations/ceiling_camera.json`, `camera-calibrations/shelf_camera.json` (내부 보정) | 원본 사진(`calibration-images/`, 약 30 MB) |
| `shelf-pose-calibrations/shelf_camera.json` (지금 쓰는 선반 위치·칸 영역) | 이전·후보 결과(`camera-calibrations/*-shelf-pose-*`, `*-25views.json`, `*.before-*.json`, `rejected/`) |
| `parking-spots/*.json` (핑키 주차 위치) | YOLO 모델(`models/`, `scripts/check_setup.sh --vision`이 Git 커밋에서 받음) |
| `omx-poses-*.json` (follower 보드별 팔 자세) | PC별 설정 `control-station.env` (예시: `control-station.env.example`), 로그·관측·녹화 |

보정을 다시 하면 새 결과를 위 경로에 두고 커밋합니다. 카메라나 거치 위치가 바뀌면 Git의 결과도 맞지 않으므로 다시 보정합니다.

## 저장 위치

```
.control-station/
  calibration-images/
    <camera_id>/                         체커보드 원본 사진 (내부 보정 입력)
  camera-calibrations/
    <camera_id>.json                     내부 보정 (렌즈 왜곡)
    <camera_id>-shelf-pose-<id>.json     선반 기준 카메라 위치와 6칸 영역 (+ .overlay.png)
  shelf-pose-calibrations/
    <camera_id>.json                     관제가 쓰는 선반 위치·칸 영역 (위 결과 중 하나를 복사)
  parking-spots/
    <spot_id>.json                       Pinky 주차 정답 위치
    rejected/                            쓰지 않는 이전 등록 파일
```

## 단계별 도구

| 단계 | 스크립트 | 입력 | 출력 | 상세 |
|---|---|---|---|---|
| 내부 보정 | `scripts/calibrate_camera_intrinsics.py` | 체커보드 사진 폴더 | `camera-calibrations/<camera_id>.json` | [카메라 내부 보정](camera-intrinsic-calibration.md) |
| 선반 위치·칸 영역 | `scripts/calibrate_shelf_pose.py` (`scripts/shelf_pose_editor.html`) | 내부 보정, 칸 정의(`config/shelf-slots.json`), 원본 프레임, 기준점 JSON(6개 이상) | `camera-calibrations/<camera_id>-shelf-pose-<id>.json` | [선반 위치 보정](shelf-pose-calibration.md) |
| Pinky 주차 위치 | `ros2 run parking_vision pinky_parking register` | 천장 카메라 내부 보정, Pinky 마커, (선택) 바닥 고정 마커 | `parking-spots/<spot_id>.json` | [실행 방법](runbook.md#천장-카메라-pinky-주차-판정) |

공용 검증 코드는 `scripts/calibration_artifacts.py`에 있습니다.

## 카메라별 현황 (2026-11-02)

| camera_id | 실물 | 해상도 | 내부 보정 | 다음 단계 |
|---|---|---|---|---|
| `ceiling_camera` | HCAM0 (관제의 Head Camera와 같은 카메라) | 640×480 | 2026-10-31, 35장, RMS 0.17 px | 주차 위치 `pinky_dock_1` 등록 완료. 허용 오차는 Pinky 주행 완료 후 다시 측정 |
| `shelf_camera` | 정면 카메라 | 640×480 | 2026-10-31, 23장, RMS 0.19 px | 선반 위치·칸 영역 보정 완료(`calibrate_shelf_pose.py`). 칸 표시와 물체 칸 확인에 사용 |

`ceiling_camera-25views.json`은 같은 사진 중 25장만 쓴 이전 계산입니다. 판정에는 `ceiling_camera.json`을 씁니다.

## 체커보드

내부 보정용 팀 공용 보드는 [checkerboard_9x7_25mm_A4.pdf](calibration-patterns/checkerboard_9x7_25mm_A4.pdf)입니다. A4 가로, 9×7칸, 한 칸 25 mm입니다. 스크립트에는 칸 수가 아니라 내부 교차점 수를 넣으므로 `--columns 8 --rows 6 --square-size 25 --unit mm`입니다. "페이지에 맞춤"을 끄고 100% 크기로 인쇄한 뒤, 아래쪽 50 mm 막대를 자로 재서 배율을 확인합니다.

## 다시 보정할 때

- 출력 파일이 이미 있으면 어떤 보정 스크립트도 덮어쓰지 않습니다. 새 경로를 지정하거나 기존 파일을 옮깁니다.
- 내부 보정을 다시 하면 그 내부 보정으로 만든 결과도 모두 다시 만들어야 합니다. 주차 위치 파일은 내부 보정 파일의 SHA-256을 기록하므로, 내부 보정이 바뀌면 판정을 거부하고 재등록을 요구합니다.
- 카메라 위치나 각도가 바뀌면 선반 위치·칸 영역과 주차 위치를 다시 등록합니다. 내부 보정은 해상도와 초점이 같으면 유지합니다.
- 해상도를 바꾸면 내부 보정부터 다시 합니다.
