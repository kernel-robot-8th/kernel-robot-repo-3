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

내부 보정 JSON schema v2에는 camera ID와 `pixel_transform_id`, 원본 이미지 크기, checkerboard 내부 코너 수·실측 칸 크기·단위, camera matrix, distortion coefficients, OpenCV 버전, RMS 및 이미지별 재투영 오차가 기록됩니다. 각 승인 이미지의 SHA-256은 OpenCV에 decode한 바로 그 파일 bytes에서 계산해 기록하며, byte-identical 복제 이미지는 추가 view로 세지 않습니다. calibration/observation CLI에서 `--output`을 해당 실행의 입력 이미지나 JSON과 같은 파일(경로 alias 및 기존 hard link 포함)로 지정하면 덮어쓰지 않고 거부합니다. 모든 calibration/projection 산출물도 출력 경로에 파일이 이미 있으면 원자적으로 덮어쓰지 않고 거부합니다. Shelf-plane 보정은 correspondence JSON과 intrinsic JSON을 각각 최대 1 MB, 원본 이미지를 각각 최대 64 MiB로 읽고 symlink file과 source-image 경로 구성요소의 symlink를 거부합니다. 각 대응 source image를 실제 decode해 대응점 JSON 및 intrinsic 초안의 선언 해상도와 같은지도 확인합니다. 선반 대응점 입력은 schema v1이며 알려진 필드만 허용합니다. 각 `source_image`는 대응점 JSON과 같은 폴더(또는 하위 폴더)에 실제로 있어야 합니다. 평면 보정 결과 schema v4에는 대응점 JSON basename/hash와 각 source image의 상대 경로/hash를 보존합니다. projection CLI는 provenance가 빠진 평면 schema v2/v3 초안을 거부하므로 새 입력으로 다시 계산해야 합니다. 두 산출물 모두 `DRAFT_ONLY`, review-required, `usable_for_robot_control: false`이고 후보 API나 로봇 제어 경로에서 읽지 않습니다.

이 계산은 렌즈 내부 파라미터만 추정합니다. 카메라 장착 위치(extrinsic), 카메라와 OMX 기준 frame 간 변환, 선반 평면/깊이, 좌표 허용 오차, IK 또는 안전한 집기 목표를 제공하지 않습니다. 카메라 해상도·crop·회전 설정이 바뀌면 결과를 그대로 재사용할 수 있다고 가정하지 않습니다. RMS 숫자만으로 보정을 승인하지 말고, 원본 이미지와 코너 분포를 검토한 뒤 알려진 기준점으로 오차를 확인해야 합니다.

## 고정 선반 평면 변환 초안

`pixel-transform-id`는 관제 camera API에서 읽은 `pixel_transform_id`와 정확히 같아야 합니다. 현재 값은 앱의 capture 이후 geometry transform 버전을 가리키며, 앱에서 crop/resize/rotation을 바꾸면 ID도 갱신해야 합니다. 이 ID는 physical camera/driver의 회전·crop·광학 설정이 일치함을 증명하지 않으므로 해상도와 frame 출처도 별도로 확인합니다.

`scripts/calibrate_shelf_plane.py`는 내부 보정 초안과 사용자가 만든 이미지↔선반 평면 대응점을 받아 평면 homography 초안을 만듭니다. 각 대응점은 원본(왜곡된) 이미지 픽셀 `(u,v)`와 측정한 선반 평면 `(x,y)`로 입력합니다. 대응점 JSON의 `pixel_transform_id`는 intrinsic 초안과 동일해야 합니다. 두 calibration 소비 CLI는 공통 `scripts/calibration_artifacts.py` validator로 내부 보정 초안의 검토/사용 가능 상태, camera matrix·distortion 배열과 checkerboard 설명의 구조, 최소 유효 view 수, 이미지별 원본·hash·오차 기록과 전체 RMS 일관성을 확인한 뒤 distortion을 적용해 픽셀을 보정하고 homography를 계산합니다. projection summary에는 checkerboard 실측 설명도 보존합니다. 이 확인은 calibration artifact 내부 데이터의 구조와 일관성을 검사하며 실제 보정 품질을 승인하지 않습니다.

최소 4개는 계산용 `fit`, 최소 2개는 계산에 넣지 않는 독립 `validation` 점으로 표시해야 합니다. 각 점에는 원본 이미지 식별자를 기록합니다. 같은 원본 이미지 안의 같은 pixel은 중복 대응점으로 거부하지만, 다른 이미지에서 우연히 같은 pixel 좌표를 쓰는 것은 별개 관측으로 허용합니다. 동일한 plane 좌표는 서로 다른 픽셀 대응을 중복시키지 않도록 전체 입력에서 한 번만 허용합니다. 두 그룹은 선반 평면에 넓게 퍼지도록 고르고, validation 잔차와 원본 위치를 사람이 검토합니다. 작은 잔차만으로 보정이 승인되지는 않습니다.

대응점 파일 예시:

```json
{
  "schema_version": 1,
  "camera_id": "head_camera",
  "pixel_transform_id": "opencv-v4l2-capture-no-app-geometry-v1",
  "image_size_px": {"width": 640, "height": 480},
  "plane_frame": {
    "frame_id": "shelf_A_plane",
    "unit": "mm",
    "axis_definition": "원점은 좌하단, +X는 오른쪽, +Y는 선반 안쪽"
  },
  "samples": [
    {"source_image": "point-1.jpg", "image_px": [80, 90], "plane_xy": [0, 0], "role": "fit"},
    {"source_image": "point-2.jpg", "image_px": [550, 90], "plane_xy": [400, 0], "role": "fit"},
    {"source_image": "point-3.jpg", "image_px": [80, 390], "plane_xy": [0, 300], "role": "fit"},
    {"source_image": "point-4.jpg", "image_px": [550, 390], "plane_xy": [400, 300], "role": "fit"},
    {"source_image": "point-5.jpg", "image_px": [300, 90], "plane_xy": [200, 0], "role": "validation"},
    {"source_image": "point-6.jpg", "image_px": [300, 390], "plane_xy": [200, 300], "role": "validation"}
  ]
}
```

숫자는 형식 예시일 뿐 실제 측정값이 아닙니다. 실행 예:

```bash
python scripts/calibrate_shelf_plane.py ./shelf-points.json \
  ./.control-station/camera-calibrations/head_camera.json
```

입력 JSON에는 `schema_version: 1`과 문서 예시의 필드를 정확히 지정합니다. `source_image` 파일은 그 입력 JSON 폴더 안의 상대 경로이며 파일 bytes의 SHA-256이 각 대응점에 기록됩니다. 평면 보정 출력은 schema v4 `DRAFT_ONLY`/`CAMERA_TO_FIXED_PLANE_ONLY`이며 내부 보정 ID, source correspondence basename/SHA-256, 원본 사진 경로/hash, frame/unit/축 설명, homography, 대응점별 residual과 validation RMS를 저장합니다. projection 요약에도 대응점 입력과 고유 원본 사진의 hash를 남깁니다. 선반 평면 좌표는 OMX 좌표가 아니므로, 이 결과는 팔 명령·작업 영역 검증·IK에 사용할 수 없습니다. 선반이 기울거나 물체가 평면에서 뜨거나 다른 깊이에 있으면 이 단일 평면 변환은 해당 점에 유효하지 않습니다.

### Task 관측 후보의 2D 평면 좌표 초안

`scripts/project_observation_to_shelf_plane.py`는 task 상세 API 응답의 `TARGET_OBSERVED` event와 위 두 보정 초안을 사용해 해당 후보의 detection bbox 하단 중앙을 선반 평면 `(x,y)` 초안으로 투영합니다. 후보는 `--candidate-index`로 직접 지정하거나, `--selection-event-id`로 같은 관측 event를 참조하는 운영자 `TARGET_SELECTED` event를 지정할 수 있습니다. 후자는 task 이력에 정확히 한 번 존재해야 하고 task/frame/request/candidate snapshot이 관측 event와 일치해야 하며, 연결된 원본 PNG 파일 hash와 복원 BGR pixel hash도 확인합니다. 두 방식은 상호 배타적입니다. 영상 bbox 중심은 물체가 표면 위에 있을 때 선반 평면에 놓인 점이 아니므로 투영 anchor로 쓰지 않습니다. bbox 하단 중앙은 물체의 지지점이 아닐 수 있는 **proxy**이며, bbox 가림·그림자·카메라 시점 때문에 실제 접촉점과 다를 수 있습니다. 출력은 anchor 종류와 미검증 가정, `physical_contact_point_verified: false`, `usable_for_pick_target: false`를 명시하므로 이 값은 집기 좌표가 아닙니다. task가 아직 `PENDING`이고 캡처 시점에 fresh였을 때만 처리합니다. 직접 index 방식은 기본 2초(최대 60초로 명시 조정 가능) freshness 제한도 적용합니다. 선택 event 방식은 보관된 정확한 PNG의 hash와 픽셀 hash를 검증하는 오프라인 검토 스냅샷이므로 캡처 후 경과 시간 제한은 없으며, 나이와 `SAVED_REVIEW_SNAPSHOT_NOT_FOR_CURRENT_CONTROL` 정책을 출력에 남깁니다. 이는 현재 장면이나 로봇 상태를 뜻하지 않고 어떤 제어에도 쓸 수 없습니다. 지정한 event ID는 task 이력에 정확히 한 번 있어야 하고 event type과 `camera_observation` 출처가 맞아야 합니다. event 안의 작업 조건은 task에 저장된 `pick_request`와 같아야 합니다. observation schema v2, camera ID, 원본 BGR frame의 SHA-256, `pixel_transform_id`, 해상도, frame sequence/capture timestamp, target class, intrinsic/plane calibration 참조도 모두 일치해야 합니다. frame hash는 픽셀 데이터가 보관되었다는 뜻이 아닙니다. transform ID는 앱 내부 geometry 경로를 식별할 뿐 driver-side 픽셀 변환을 증명하지 않습니다. bbox는 양의 면적이어야 하며 `center_px`를 포함하고 영상 가장자리에 닿지 않아야 합니다. 가장자리 bbox는 일부가 잘렸을 수 있어 투영을 거부합니다. bbox-bottom-center proxy는 calibration correspondence convex hull 안에 있어야 하고, 범위 밖 homography 외삽은 거부합니다. 두 calibration draft에 operator review 표시와 최소 fit/validation point 및 intrinsic view 수, 유한한 잔차 값이 있어야 합니다. projection 도구는 plane calibration의 각 fit/validation 대응점에 대해 homography projection과 residual/RMS를 다시 계산해 파일에 적힌 값과 일치하는지 확인하고, projection 결과는 관련 잔차·점 수·단위를 함께 보존합니다. 자동 합격 잔차 기준은 설정하지 않았으므로 숫자와 원본 자료는 여전히 사람이 검토해야 합니다.

`--arm` 모드에서 Joint Camera weights가 별도 설정되면 Joint Camera observation도 같은 후보/API 경로를 사용할 수 있습니다. 실제 가중치가 없어 해당 경로는 아직 관측하지 않았습니다. 유효한 observation event와 실제 유효한 calibration 자료가 없으면 이 투영 도구를 사용할 수 없습니다. 직접 `--candidate-index`를 지정할 때는 기본 2초 이내 observation만 허용합니다(`--max-age-seconds`로 최대 60초까지 조정 가능). `--selection-event-id` 경로는 캡처 당시 freshness, `PENDING` task, 저장 PNG의 file SHA-256과 복원 BGR pixel hash를 검증해 과거의 정확한 검토 snapshot을 투영하므로 경과 시간 제한은 없습니다. `--preview-output result.png`를 추가하면 검증된 사본에 bbox와 하단 중앙 proxy, 평면 `(x,y)`, `DRAFT ONLY` 표기를 덧그린 별도 PNG를 만들고 그 파일 hash와 source frame hash를 JSON 초안에 기록합니다. 원본 PNG는 바꾸지 않으며 미리보기 역시 로봇 목표가 아닙니다. 이 값은 현재 장면이나 로봇 상태가 아닌 `DRAFT_ONLY` 기록입니다. 입력 intrinsic 초안에서 유효 view 수, 고유한 source image 목록, 이미지별 오차 목록이 서로 일치하는지 검사하고, 각 view의 코너 수가 동일한 체커보드 설정이므로 이미지별 RMS로 전체 RMS를 다시 계산해 파일의 RMS와 수치적으로 일치하는지도 확인합니다. 이는 초안 데이터의 손상·불일치를 거르는 내부 일관성 검사이며, 허용 오차 기준이나 보정 품질 합격 판정은 아닙니다. 투영 결과에는 원본 이미지 목록과 view별 재투영 오차를 남겨 보정 지표를 추적할 수 있게 합니다.

```bash
curl -sS http://127.0.0.1:8000/api/tasks/TASK-ID -o /tmp/task-detail.json
python scripts/project_observation_to_shelf_plane.py \
  /tmp/task-detail.json \
  ./.control-station/camera-calibrations/head_camera-plane-CALIBRATION-ID.json \
  ./.control-station/camera-calibrations/head_camera.json \
  --event-id OBSERVATION-EVT-ID --selection-event-id SELECTION-EVT-ID \
  --preview-output /tmp/object-coordinate-review.png
```

event와 보정 파일 이름은 실제 출력값으로 바꿉니다. 이전 동작처럼 수동 index를 지정하려면 `--selection-event-id` 대신 `--candidate-index 0`을 사용합니다. 결과는 `.control-station/camera-calibrations/`에 `DRAFT_ONLY`/`CAMERA_PIXEL_TO_FIXED_SHELF_PLANE_2D`로 저장되며 source observation의 `observation_step_id`, 선택 event(있으면)의 `selection_step_id`, camera/frame, candidate와 calibration ID를 포함합니다. `target_selection_complete`는 원본 task에 사람의 `TARGET_SELECTED` event가 연결되고 미적용 color/shelf constraint가 없을 때만 `true`입니다. CLI index만 직접 지정한 경우는 선택 이력이 없으므로 `false`입니다. color/shelf_level 요청은 현재 후보 선택에 적용하지 않아 미적용 조건으로 남깁니다. 오래된 event는 다시 관측해야 합니다. 출력에는 z, 선반→OMX transform, IK, 작업영역 검증 또는 집기 목표가 없고 `usable_for_robot_control: false`, `arm_command_sent: false`입니다. 승인된 실제 calibration이 아직 없으므로 지금은 placeholder/실측 점으로 계산한 초안만 가능합니다.

검토 이벤트 연결을 원할 때만 `--record-review-event`를 추가합니다. 이 옵션은 기본 동작이 아니며, 지정한 localhost control-station에 새 JSON artifact의 상대 경로와 SHA-256만 제출합니다. 출력·입력 calibration JSON은 모두 저장소의 `.control-station/camera-calibrations/` 바로 아래의 일반 파일이어야 합니다. projection 초안에는 원본 plane/intrinsic calibration 경로와 SHA-256을 담고, 서버가 파일 hash와 camera/frame/transform/calibration ID 연결을 다시 확인한 뒤 `TARGET_PLANE_PROJECTION_RECORDED`를 pending `ESTIMATE_TARGET_POSE`에 추가합니다. task와 step은 계속 pending이며 로봇 명령은 보내지 않습니다. 네트워크 오류나 응답 유실이면 서버에 이벤트가 저장됐는지 CLI가 확정할 수 없습니다. 초안 파일을 보존하고 task 이력을 먼저 확인합니다. 동일 draft ID와 hash의 재요청은 DB 트랜잭션 안에서 기존 event를 반환해 중복을 막습니다.
