# 정면 카메라 선반 위치 보정

정면(선반) 카메라가 선반 기준으로 어디에 있는지 구하고, 6칸 영역을 카메라 화면에 투영합니다. 칸 영역을 손으로 그리지 않고 [칸 정의](arm-command-api.md#선반-칸-정의)의 3D 좌표에서 만듭니다. 결과는 `DRAFT_ONLY` 검토용이며 칸 판정이나 팔 명령에는 아직 연결하지 않았습니다.

## 원리

[config/shelf-slots.json](../../config/shelf-slots.json)의 `keypoints`에는 정면에서 찍을 선반 모서리 18개의 3D 좌표(`shelf_front` 기준, m)가 들어 있습니다. 3D 모델에서 계산하고 아래 단 실측 보정(−3.5 cm)을 반영한 값입니다.

- **앞 턱 모서리 12개 (주로 이것을 씀):** 쓰는 단(왼쪽 선반 위·아래 단, 오른쪽 선반 아래 단)마다 앞 턱의 위·아래 선 양 끝. 바닥판 윗면은 턱에 가려 보이지 않으므로, 정면 카메라에 실제로 보이는 선은 턱의 위·아래 경계입니다. 위 단 턱은 24.9–26.9 cm, 아래 단 턱은 8.1–10.0 cm입니다.
- 옆판 모서리 6개: 왼쪽 끝·가운데 이음새·오른쪽 끝 옆판의 아래 앞(0 cm)과 위 앞(26.9 cm). 현장 정면 카메라에서는 선반 밑면이 책상 앞 모서리에 가려 아래 앞 모서리가 보이지 않습니다. 모델의 옆판 계단(13.5 cm)은 실물 아래 단 위치와 맞지 않아 기준점에서 뺐습니다.

사진에서 이 중 보이는 점을 6개 이상 찍으면, 내부 보정과 함께 solvePnP(SQPnP 후 Levenberg-Marquardt 보정)로 카메라 위치를 구합니다. 그다음 칸마다 두 영역을 화면에 투영합니다.

- **판정 영역 (`anchor_polygon_px`):** 바닥판부터 앞 턱 위 + `--anchor-margin`(기본 0.02 m)까지의 얇은 상자입니다. 검출 bbox의 아래 가운데 점(물건이 바닥에 닿는 곳)이 보일 수 있는 범위이고, 칸 판정은 이 영역으로 합니다. 아래 단은 8.3–12.0 cm, 위 단은 25.1–28.9 cm입니다. 칸 전체 공간으로 판정하면 아래 칸 영역이 위 단 바로 밑까지 올라가서, 위 단에 놓인 물건의 바닥점이 아래 칸 경계에 걸립니다. 여유 0.02 m는 bbox 오차를 고려한 가정값입니다.
- **칸 전체 공간 (`polygon_px`):** 바닥판부터 위 단까지의 상자입니다. 화면에 참고로만 그립니다. 위가 열린 칸(왼쪽 위 단)은 높이를 `--open-slot-height`(기본 0.10 m)로 잡는데, 이것도 가정값이며 측정값이 아닙니다.

## 절차

1. **프레임 준비:** OMX 팔이 선반을 가리지 않게 홈 자세로 둔 상태에서 정면 카메라 원본 프레임을 저장합니다. 관제 MJPEG 스트림에는 YOLO 박스가 그려질 수 있으니, 검출을 끈 상태나 task 관측 저장 PNG처럼 원본 프레임을 씁니다. 해상도는 내부 보정과 같은 640×480이어야 합니다.
2. **기준점 찍기:** 브라우저에서 `scripts/shelf_pose_editor.html`을 엽니다(Web Crypto 때문에 `python3 -m http.server 8765 --bind 127.0.0.1` 뒤 `http://127.0.0.1:8765/scripts/shelf_pose_editor.html` 권장). 프레임과 `config/shelf-slots.json`을 고르고, 목록에서 점을 골라 사진의 그 모서리를 클릭합니다. 안 보이는 점은 건너뜁니다. 서로 다른 높이·깊이의 점(예: 이음새 위 앞, 이음새 계단 앞, 바닥판 앞 모서리, 양 끝 위 앞)을 섞어 찍어야 위치가 잘 정해집니다. **기준점 JSON 다운로드**로 `shelf-pose-points.json`을 받습니다.
3. **계산:**

   ```bash
   .venv/bin/python scripts/calibrate_shelf_pose.py shelf-pose-points.json frame.png
   ```

   기본 입력은 `.control-station/camera-calibrations/shelf_camera.json`(내부 보정)과 `config/shelf-slots.json`입니다. 출력은 `.control-station/camera-calibrations/shelf_camera-shelf-pose-<id>.json`과 같은 이름의 `.overlay.png`이며, 같은 이름이 있으면 덮어쓰지 않습니다.
4. **검토:** 오버레이에서 초록 십자(찍은 점)와 빨간 원(계산 위치)이 겹치는지, 칸 영역이 실제 칸과 맞는지 눈으로 확인합니다. 터미널에는 점마다 재투영 오차와 "그 점을 빼고 계산했을 때의 예측 오차"가 나옵니다. 빼고 예측한 오차만 큰 점은 잘못 찍었을 가능성이 높으니 다시 찍습니다. 카메라 위치(선반 기준 앞·왼쪽·높이)가 실제와 비슷한지도 확인합니다. 자동 합격 기준은 없습니다.

## 2026-11-03 현장 결과

- 프레임: `.control-station/shelf-pose/shelf_camera-20261103-205505.png` (팔 접은 상태, 관제 서버 없이 OpenCV로 직접 캡처, YOLO 박스 없음)
- 기준점: 앞 턱 모서리 9개. 턱 선은 밝기 경계로 재고, 끝 위치는 옆판 바깥면·이음새 선에서 옆판 두께만큼 안쪽으로 잡음 (`points-20261103-205505-claude-v2.json`)
- 결과: `shelf_camera-shelf-pose-f581e381.json`, 재투영 RMS 1.3 px, 점 하나를 빼고 예측한 오차 최대 3.1 px. 판정 영역을 추가한 뒤 같은 점으로 다시 계산한 `shelf_camera-shelf-pose-12a67aa7.json`이 활성 보정입니다(칸 정의에 앞 턱 높이가 추가돼 칸 정의 hash가 바뀌었기 때문).
- 카메라 위치(선반 기준): 앞으로 116.5 cm, 이음새에서 오른쪽 2.5 cm, 선반 밑면에서 38.3 cm 위
- 오버레이에서 6칸 영역이 실제 칸과 물건 위치에 맞는 것을 눈으로 확인

처음에는 아래 단 높이를 모델 값(13.5 cm)으로 두고, 바닥판 윗면 모서리를 찍어 RMS 25 px로 실패했습니다. 그 결과는 `camera-calibrations/rejected/`에 있습니다. 실측으로 아래 단 위치를 고치고 보이는 턱 선을 기준점으로 바꾼 뒤 위 결과를 얻었습니다.

## 관제 화면에 연결

검토한 결과 파일을 카메라 ID 이름으로 활성 폴더에 두면, 관제가 그 카메라 영상 위에 칸 영역을 그리고 검출마다 칸을 표시합니다. 활성 폴더 기본값은 `.control-station/shelf-pose-calibrations/`이고, `CONTROL_STATION_SHELF_POSE_DIR`로 바꿀 수 있습니다.

```bash
mkdir -p .control-station/shelf-pose-calibrations
cp --update=none .control-station/camera-calibrations/shelf_camera-shelf-pose-<id>.json \
  .control-station/shelf-pose-calibrations/shelf_camera.json
```

- 파일이 바뀌면 다음 `/api/cameras` 요청부터 반영되므로 관제를 다시 시작하지 않아도 됩니다. 다만 칸 정의 파일(`config/shelf-slots.json`)이 바뀌면 관제를 다시 시작해야 합니다.
- `/api/cameras`의 해당 카메라에 `slot_zones`가 붙습니다. 값은 `status`, 칸별 `polygon_px`, 재투영 RMS, 카메라 위치입니다. `status`가 `READY`가 아니면 이유를 함께 담고 칸은 쓰지 않습니다. 이유는 `SLOT_MAP_MISMATCH`(칸 정의 버전이 다름), `IMAGE_MISMATCH`(해상도·pixel transform이 다름), `INVALID`, `SLOT_MAP_UNAVAILABLE` 중 하나입니다.
- 검출마다 `slot_assignment`가 붙습니다. bbox 아래 가운데 점(물건이 바닥판에 닿는 곳)이 한 칸 안에만 있으면 `IN_SLOT`과 칸 ID·이름, 칸 경계 위나 여러 칸에 걸치면 `AMBIGUOUS`, 어느 칸에도 없으면 `OUTSIDE`입니다.
- 모든 결과는 `used_for_selection: false`인 표시용입니다. 대상을 자동으로 고르거나 팔 명령을 만들지 않습니다.
- 화면의 정면 카메라 카드는 영상 위에 칸 경계를 그립니다. 검출이 있는 칸은 초록, 검출의 기준점은 점으로 표시하고, 카드 아래에 "물건 → 칸" 목록을 보여 줍니다.
- 카메라나 선반이 움직이면 이 영역은 맞지 않으니 다시 보정해야 합니다. 관제는 카메라가 움직였는지 감지하지 못합니다.

## 입력·출력 계약

- 기준점 JSON(schema 1): `camera_id`, `source_image_sha256`, `source_image_size_px`, `slot_map_sha256`, `points[]`(`keypoint_id`, `image_px`). CLI는 프레임 hash, 칸 정의 hash, 해상도, 카메라 ID가 모두 맞아야 계산합니다. 점은 6개 이상, 중복 없이 칸 정의에 있는 ID여야 하고 한 직선 위에 있으면 거부합니다.
- 결과 JSON(schema 1): 카메라 위치·`rvec`/`tvec`, 점별 오차와 빼고 예측한 오차, 재투영 RMS, 칸별 `polygon_px`(상자 투영의 볼록 껍질)와 `deck_polygon_px`(바닥판 투영), 입력 hash 4개. `status: DRAFT_ONLY`, `usable_for_robot_control: false`.

## 제한

- 3D 좌표는 모델 값입니다. 출력한 선반이 모델과 다르면 그만큼 어긋납니다. 실물 치수 확인 전입니다.
- 카메라나 선반이 움직이면 다시 찍어야 합니다. 내부 보정을 다시 하면 이 결과도 다시 만듭니다.
- 사진에서 오른쪽 선반은 위 단 바닥판이 보이지 않습니다. 그 단은 쓰지 않으므로 기준점에도 넣지 않았습니다.
- 칸 polygon으로 검출 결과를 칸에 배정하는 단계(관측 → 칸 판정)는 다음 단위입니다.
