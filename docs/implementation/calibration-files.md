# 캘리브레이션 파일 지도

카메라마다 어떤 스크립트로 어떤 보정 파일을 만들고, 그 파일이 어디에 저장되는지 정리합니다. 각 도구의 상세 절차는 표에 링크한 문서를 따릅니다.

보정 결과와 원본 사진은 모두 `.control-station/` 아래에 저장되며 Git에서 제외됩니다. 저장소에는 스크립트, 문서, 인쇄용 패턴만 있습니다. 다른 PC에서 쓰거나 다시 계산하려면 결과 파일과 원본 사진을 따로 옮기거나 백업해야 합니다.

## 저장 위치

```
.control-station/
  calibration-images/
    <camera_id>/                         체커보드 원본 사진 (내부 보정 입력)
  camera-calibrations/
    <camera_id>.json                     내부 보정 (렌즈 왜곡)
    <camera_id>-plane-<id>.json          픽셀 → 선반 평면 homography
    <camera_id>-shelf-regions-<id>.json  선반 층 영상 영역
    <camera_id>-projection-<id>.json     관측 후보의 평면 좌표 초안
  parking-spots/
    <spot_id>.json                       Pinky 주차 정답 위치
    rejected/                            쓰지 않는 이전 등록 파일
```

## 단계별 도구

| 단계 | 스크립트 | 입력 | 출력 | 상세 |
|---|---|---|---|---|
| 내부 보정 | `scripts/calibrate_camera_intrinsics.py` | 체커보드 사진 폴더 | `camera-calibrations/<camera_id>.json` | [카메라 좌표 초안](camera-intrinsic-calibration.md) |
| 선반 평면 | `scripts/calibrate_shelf_plane.py` | 내부 보정, 대응점 JSON(fit 4개 + validation 2개 이상) | `camera-calibrations/<camera_id>-plane-<id>.json` | [카메라 좌표 초안](camera-intrinsic-calibration.md#고정-선반-평면-변환-초안) |
| 선반 층 영역 | `scripts/calibrate_shelf_regions.py` (`scripts/shelf_region_editor.html`) | 관측 frame, 층별 polygon | `camera-calibrations/<camera_id>-shelf-regions-<id>.json` | [선반 층 영상 영역](shelf-image-regions.md) |
| 후보 평면 좌표 | `scripts/project_observation_to_shelf_plane.py` | 관측 event, 내부 보정, 선반 평면 | `camera-calibrations/<camera_id>-projection-<id>.json` | [카메라 좌표 초안](camera-intrinsic-calibration.md#task-관측-후보의-2d-평면-좌표-초안) |
| Pinky 주차 위치 | `scripts/pinky_parking.py register` | 천장 카메라 내부 보정, Pinky 마커 | `parking-spots/<spot_id>.json` | [실행 방법](runbook.md#천장-카메라-pinky-주차-판정) |

공용 검증 코드는 `scripts/calibration_artifacts.py`에 있으며 MuJoCo·그리퍼·TCP 스크립트도 함께 씁니다.

## 카메라별 현황 (2026-11-02)

| camera_id | 실물 | 해상도 | 내부 보정 | 다음 단계 |
|---|---|---|---|---|
| `ceiling_camera` | HCAM0 (관제의 Head Camera와 같은 카메라) | 640×480 | 2026-10-31, 35장, RMS 0.17 px | 주차 위치 `pinky_dock_1` 등록 완료. 허용 오차는 Pinky 주행 완료 후 다시 측정 |
| `shelf_camera` | 정면 카메라 | 640×480 | 2026-10-31, 23장, RMS 0.19 px | 선반 평면 보정. 기준점은 ArUco 마커로 잡을 예정 |

`ceiling_camera-25views.json`은 같은 사진 중 25장만 쓴 이전 계산입니다. 판정에는 `ceiling_camera.json`을 씁니다.

## 체커보드

내부 보정용 팀 공용 보드는 [checkerboard_9x7_25mm_A4.pdf](calibration-patterns/checkerboard_9x7_25mm_A4.pdf)입니다. A4 가로, 9×7칸, 한 칸 25 mm입니다. 스크립트에는 칸 수가 아니라 내부 교차점 수를 넣으므로 `--columns 8 --rows 6 --square-size 25 --unit mm`입니다. "페이지에 맞춤"을 끄고 100% 크기로 인쇄한 뒤, 아래쪽 50 mm 막대를 자로 재서 배율을 확인합니다.

## 다시 보정할 때

- 출력 파일이 이미 있으면 어떤 보정 스크립트도 덮어쓰지 않습니다. 새 경로를 지정하거나 기존 파일을 옮깁니다.
- 내부 보정을 다시 하면 그 내부 보정으로 만든 결과도 모두 다시 만들어야 합니다. 주차 위치 파일은 내부 보정 파일의 SHA-256을 기록하므로, 내부 보정이 바뀌면 판정을 거부하고 재등록을 요구합니다.
- 카메라 위치나 각도가 바뀌면 선반 평면, 층 영역, 주차 위치를 다시 등록합니다. 내부 보정은 해상도와 초점이 같으면 유지합니다.
- 해상도를 바꾸면 내부 보정부터 다시 합니다.
