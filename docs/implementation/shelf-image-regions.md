# 선반 층 영상 영역 초안

`scripts/calibrate_shelf_regions.py`는 저장된 `TARGET_OBSERVED` 원본 PNG와 운영자가 기록한 층별 polygon을 검증해 `DRAFT_ONLY` artifact를 만듭니다. 입력 좌표를 측정해 주지는 않으며, 자동 대상 선택이나 팔 제어 허가도 만들지 않습니다.

## 원본 프레임 준비

작업 이력에서 관측할 대상을 기록하고 **원본 RGB 프레임 PNG도 저장**을 선택합니다. 같은 작업이 `PENDING`인 동안 아래처럼 task 상세 JSON을 저장합니다. CLI는 task JSON을 최대 64 MiB, 비-symlink regular file로 제한해 읽습니다.

```bash
curl -sS http://127.0.0.1:8000/api/tasks/TASK-ID -o /tmp/task-detail.json
```

저장 PNG는 관측 event의 `frame_artifact`와 `frame_sha256`에 연결돼야 합니다. 관측 캡처 sequence는 첫 프레임이 1이므로 CLI는 양의 정수만 받습니다. 관제 저장기는 PNG 파일을 최대 64 MiB로 제한하고 managed observation directory의 symbolic link를 거부합니다. CLI는 task/event 상태와 event에 존재하는 `DETECT_TARGETS` step link, camera ID, 해상도, `pixel_transform_id`, 최대 64 MiB PNG 파일 SHA-256을 확인한 뒤 PNG를 다시 decode해 BGR 픽셀 SHA-256이 원본 관측과 일치하는지 확인합니다. 이미지 파일은 task별 `.control-station/observations/{task_id}/` 안에서만 읽습니다.

## 층 영역 입력

작업 이력의 해당 관측 event에서 **프레임 보기**를 눌러 PNG를 내려받습니다. 저장소 파일 브라우저에서 `scripts/shelf_region_editor.html`을 열고 PNG를 선택한 다음, 층 번호를 지정해 각 칸의 안쪽 경계를 클릭합니다. 도구는 PNG 파일 SHA-256과 원본 해상도, polygon을 포함한 `regions.json`을 다운로드합니다. task provenance나 calibration artifact는 만들지 않으므로 아래 CLI 검증을 계속 거쳐야 합니다.

브라우저에서 Web Crypto를 사용할 수 없어 fingerprint 계산이 실패하면 저장소 루트에서 로컬 전용 정적 서버를 실행하고 `http://127.0.0.1:8765/scripts/shelf_region_editor.html`을 여세요.

```bash
python3 -m http.server 8765 --bind 127.0.0.1
```

작업이 끝나면 정적 서버를 `Ctrl+C`로 종료합니다.

좌표는 원본 이미지의 좌상단 원점 pixel `(x,y)`이며, 점은 테두리를 따라 한 방향으로 순서대로 기록합니다. 수치는 실제 저장된 영상에서 확인합니다. 도구는 클릭 좌표와 JSON 구조를 편하게 입력할 뿐, 층의 물리적 위치·경계나 camera calibration을 측정하지 않습니다.

`regions.json` 입력 schema v2는 다음과 같습니다. `source_image_sha256`와 `source_image_size_px`는 선택한 PNG의 파일 hash와 해상도입니다. CLI는 둘 다 저장 task observation artifact와 대조하므로 같은 크기의 다른 프레임에서 그린 polygon을 연결하지 않습니다. 영역 입력은 최대 1 MB의 비-symlink regular file이어야 합니다. task 응답, 영역 입력 및 runtime calibration artifact의 JSON 객체에서 중복 키가 발견되면 마지막 값으로 덮어쓰지 않고 거부합니다.

```json
{
  "schema_version": 2,
  "source_image_sha256": "<64 lowercase hex characters>",
  "source_image_size_px": {"width": 640, "height": 480},
  "regions": [
    {"shelf_level": 1, "polygon_px": [["X1_PX", "Y1_PX"], ["X2_PX", "Y2_PX"], ["X3_PX", "Y3_PX"]]}
  ]
}
```

`regions.json`은 1 MB 이하, 영역은 1~64개, 각 polygon 정점은 3~512개여야 합니다. 각 층 번호는 유일한 `1–999,999,999` 정수여야 하며 작업 명세의 `shelf_level` 입력 범위와 같습니다. polygon은 서로 다른 정점 3개 이상, 양의 면적을 가져야 하고 모든 점이 프레임 안에 있어야 합니다. 중복 정점과 자기 교차 polygon은 거부합니다. 생성 artifact도 관측 비교기가 읽을 수 있도록 1 MB 이하로 제한합니다. 서로 다른 층 polygon이 겹치면 관측 비교에서 해당 후보는 모호로 남습니다.

## 실행과 산출물

`EVT-ID`는 저장된 task 상세 응답 안의 원본 `TARGET_OBSERVED` event ID로 바꿉니다.

```bash
python3 scripts/calibrate_shelf_regions.py \
  /tmp/task-detail.json /tmp/regions.json \
  --event-id EVT-ID
```

출력 기본 위치는 `.control-station/camera-calibrations/{camera_id}-shelf-regions-{calibration_id}.json`입니다. artifact 최상위의 `camera_id`, `pixel_transform_id`, `image_size_px`는 `source`의 같은 값과 일치하며 runtime 비교기가 모두 교차 확인합니다. artifact는 region 입력 JSON의 SHA-256, 원본 event/task/step/frame, 이미지 파일 및 pixel-buffer hash, 해상도, `pixel_transform_id`, polygon을 함께 보존합니다. 완성한 임시 파일을 같은 디렉터리에서 원자적으로 새 이름에 연결하며, 출력 경로가 이미 있으면 덮어쓰지 않고 실패합니다.

모든 산출물은 `DRAFT_ONLY`, `review_required: true`, `selection_enabled: false`, `usable_for_robot_control: false`, `arm_command_sent: false`입니다. 후보 관측에서 층 비교 힌트를 보려면 사람이 선택한 파일을 명시적 활성 디렉터리에 camera ID 이름으로 둡니다. 기본 활성 디렉터리는 `.control-station/shelf-region-calibrations`이며 환경 변수 `CONTROL_STATION_SHELF_REGION_DIR`로 바꿀 수 있습니다.

```bash
mkdir -p .control-station/shelf-region-calibrations
cp --no-clobber \
  .control-station/camera-calibrations/CAMERA-SHELF-REGIONS-ID.json \
  .control-station/shelf-region-calibrations/CAMERA_ID.json
```

`CAMERA_ID`는 관측 API의 camera ID, 예를 들어 `head_camera`여야 합니다. API는 `{camera_id}.json` 한 개만 확인합니다. 비교기는 artifact를 최대 1 MB까지만 읽고 calibration 디렉터리/artifact의 symbolic link와 경로 구성요소의 symlink 및 regular file이 아닌 입력을 거부합니다. 파일을 제한 이내에서 전부 읽으면 JSON/출처 검증 결과와 관계없이 그 내용의 SHA-256을 관측 결과와 task event에 표시합니다. 파일 읽기 실패 또는 크기 초과 시에는 hash를 제공하지 않습니다. camera ID·해상도·`pixel_transform_id`가 source와 현재 관측에 일치할 때만 detection `center_px`를 polygon과 대조합니다. `CALIBRATION_NOT_CONFIGURED`, source mismatch, polygon 밖, 경계/겹침 모호, 요청 층 일치 상태를 관측 결과와 task event에 표시합니다. 후보 배열은 그대로 두며 `used_for_selection: false`입니다. 활성 디렉터리에 draft를 배치하는 행위는 review-only 비교를 허용할 뿐 물리 측정의 검토 완료나 로봇 사용 승인이 아닙니다.

활성화 전에 별도의 검토 프레임에서 층별 중심점 결과를 사람이 확인하고 검토 근거를 기록합니다. 현재 artifact schema는 이 holdout 검토 결과를 저장하지 않으며 API는 이를 자동 승인하지 않습니다.

이 2D 영역은 해당 카메라 위치·회전/crop·해상도에만 적용할 수 있습니다. 원근, 가림, 한 물체가 여러 영역에 걸치는 경우를 해결하지 않으며, 층 번호·polygon의 실제 의미나 로봇 작업공간을 증명하지 않습니다. 카메라 구성이 바뀌면 다시 기록하고 검토합니다.
