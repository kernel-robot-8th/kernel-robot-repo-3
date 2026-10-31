# Pick & Place 현장 입력표

이 문서는 저장소만으로 알 수 없는 첫 고정 pick 시나리오와 보정 입력을 현장에서 기록하기 위한 표입니다. 추정값이나 예시 숫자를 실제 값으로 복사하지 않습니다. 각 값에는 측정 도구, 측정 날짜, 확인자와 원본 자료를 함께 남깁니다. 이 표를 채워도 자체로 로봇 이동 허가가 되지는 않습니다.

## 1. 첫 MVP 작업 범위

처음에는 한 번의 시도에서 바꾸는 물체·선반 칸·놓을 위치를 각각 하나로 제한합니다.

| 항목 | 현장 기록 | 확인 근거 |
|---|---|---|
| 대상 물체의 실제 이름/종류 | 미확인 | 물체 사진 및 사람이 확인한 이름 |
| 대상 물체 색상 또는 구별 조건 | 미확인 | 조명 조건을 포함한 사진 |
| 선반 ID와 층 번호 표기 규칙 | 미확인 | 현장 기준 원점과 층 번호 사진 |
| 놓을 위치 ID와 내부 기준 | 미확인 | 빈 목적지의 사진 및 측정 |
| 물체 외형 크기 범위와 질량 출처 | 미확인 | 캘리퍼스/저울 측정 또는 제조사 사양 |
| 시험 중 허용되는 물체 배치와 조명 | 미확인 | 운영자가 정한 고정 시나리오 |

## 2. 카메라와 영상 입력

장치 노드명이나 `/dev/video*` 번호만으로 물리 장착 위치를 확정하지 않습니다. 화면 영상과 실제 장착 위치를 사람이 대조해 기록합니다.

| 확인 항목 | Head Camera | Joint/Wrist Camera |
|---|---|---|
| 관제에서 선택한 camera ID | 미확인 | 미확인 |
| 확인한 물리 장착 위치 | 미확인 | 미확인 |
| 선반을 향하는 방향/시야 가림 | 미확인 | 미확인 |
| RGB 해상도·프레임률·픽셀 회전/crop | 미확인 | 미확인 |
| 관제 API의 `pixel_transform_id` | 구현 기본값 `opencv-v4l2-capture-no-app-geometry-v1`; `/api/cameras`에서 관측 후 기록 | 미확인 |
| 현재 관제 구현 / 물리 카메라의 Depth·CameraInfo·정렬 계약 | RGB OpenCV color frame만 제공 / 물리 센서 기능 미확인 | RGB OpenCV color frame만 제공 / 물리 센서 기능 미확인 |
| camera capture timestamp 출처와 ROS/장치 clock 기준 | 미확인 | 미확인 |
| 영상 시각과 관절 상태 수신 시각 간 최대 허용 skew 및 측정 방법 | 미확인 | 미확인 |
| camera image와 같은 시점의 joint positions를 얻는 source/interface | 해당 없음 | 미확인 |
| 카메라를 다른 프로그램이 점유하는지 | 미확인 | 미확인 |
| detector 상태 및 사용 가중치/class 목록 | 미확인 | 미확인 |

캘리브레이션 이미지는 실제 서비스 입력과 같은 camera ID, `pixel_transform_id`, 해상도 및 driver-side 회전/crop 설정으로 저장합니다. `pixel_transform_id`는 관제 코드의 앱 내부 geometry transform 버전만 식별하며 camera firmware/driver 또는 lens 설정을 증명하지 않습니다. 설정이 바뀌면 해당 이미지와 보정 결과의 적용 가능성을 다시 검토합니다. 물체를 집으려 접근할 때 손목 카메라가 쓰일지는 장착 시야와 제어 구조를 확인한 후 정합니다.

2026-10-31 호스트의 read-only V4L2 관측에서는 HCAM0가 `/dev/video4`, Innomaker U20CAM-720P가 `/dev/video6`이었습니다. 각 by-id 경로는 `/dev/video4`·`/dev/video6`으로 해석됐습니다. Innomaker의 1280×720 단일 MJPG frame은 가까운 표면 위주라 선반과 역할을 식별하지 못했고, HCAM0에서 읽은 단일 stream artifact는 JPEG frame으로 decode되지 않았습니다. 파일은 `.control-station/physical-camera-readonly/`에 보존했으며 이 관측만으로 장치 고장, 물리 장착 또는 Head/Joint 역할을 판정하지 않습니다.

현재 MuJoCo 시야 탐색은 측정되지 않은 링크6 부착 위치 60개를 가정한 합성 비교입니다([탐색 결과와 제한](mujoco-omx.md)). 시뮬레이터의 상위 camera pose를 장착 설계값으로 복사하지 말고, 실물을 확인한 뒤 다음 항목을 기록합니다.

| Wrist camera / 장착 입력 | 현장 기록 | 확인 근거 |
|---|---|---|
| camera 본체 모델, 렌즈, bracket 및 추가 부품 질량 | 미확인 | 제조사 문서, 저울, 사진 |
| bracket을 포함한 외곽 치수와 로봇 링크별 최소 여유 | 미확인 | 캘리퍼스/깊이 게이지 및 관절별 사진 |
| 광학 중심에서 실물 tool/TCP 및 확인된 링크 frame까지의 translation | 미확인 | 측정점 정의, 기준자세, 측정 반복값 |
| optical frame 축 방향과 link frame 대비 quaternion 또는 회전행렬 | 미확인 | 축 표식이 보이는 사진, hand-eye/fixture 측정 artifact |
| RGB intrinsics, distortion, 실제 FOV, 해상도, crop/rotate 설정 | 미확인 | 해당 camera ID의 저장 frame과 intrinsic calibration source |
| 지원되는 RGB/Depth/CameraInfo encoding 및 stream timestamp | 미확인 | driver topic/장치 API 관측 기록; 현재 관제 RGB-only 경로와 구분 |
| bracket/카메라/케이블의 joint별 self-collision·선반 간섭 | 미확인 | 정지 상태 육안 확인과 승인된 저속 clearance 점검 기록 |
| 재관측에 쓸 접근 keyframe별 관절 상태와 target pixel bbox | 미확인 | 동기화한 image/frame sequence, capture time, joint-state source |
| 측정 도구·오차/불확실도·측정자·날짜·원본 file hash | 미확인 | 원본 artifact와 측정 기록 |

치수나 camera frame 관계가 불명확하면 해당 값을 `미확인`으로 둡니다. 시야가 좋아 보이는 영상만으로 optical center, transform, collision clearance 또는 재인식 성공을 판정하지 않습니다.

## 3. 카메라 영상의 선반 층 영역 기록

선반 층 선택은 실제 카메라 장착·회전/crop 설정에 고정된 **영상 영역 초안**으로만 검토합니다. 이미지만으로 깊이, 물체의 실제 선반 접촉 위치, 가려진 층을 알아낼 수 있다고 가정하지 않습니다. 영역을 기록해도 자동 대상 선택이나 팔 제어 허가가 되지 않습니다.

1. 관제 화면에서 물리 장착 위치를 확인한 카메라를 선택하고, 카메라를 움직이지 않은 채 각 선반 층이 모두 보이는 프레임을 저장합니다. 작업 이력의 원본 PNG 저장 옵션을 켜면 이벤트와 연결된 입력 이미지를 보존할 수 있습니다.
2. 원본 PNG 위에 각 층의 보이는 내부 영역을 영상 좌상단 원점의 pixel 좌표로 다각형 표시합니다. 정점은 테두리 방향으로 일관된 순서로 기록하고, 선반 경계·가림·겹침 영역을 임의로 포함하지 않습니다.
3. 모든 영역에 동일한 camera ID, frame width/height, `pixel_transform_id` 및 입력 프레임 출처를 연결합니다. 카메라 위치·회전·crop·해상도 또는 소프트웨어 geometry 경로가 바뀌면 이전 영역 초안을 다시 검토합니다.
4. ROI 정의에 쓰지 않은 별도 프레임에서 층별 객체 중심이 어느 영역에 들어가는지 사람이 검토해 기록합니다. 두 영역에 들어가거나 어떤 영역에도 들어가지 않는 점은 모호/미분류로 남기며 가장 가까운 층으로 보정하지 않습니다.

| 확인 항목 | 현장 기록 |
|---|---|
| camera ID / 물리 장착 위치·방향 확인자 | 미확인 |
| 기준 PNG 상대 경로와 파일 SHA-256 | 미확인 |
| 원본 frame SHA-256 / sequence / capture time | 미확인 |
| frame 해상도 / `pixel_transform_id` | 미확인 |
| 층 번호 표기와 polygon pixel vertices (좌상단 원점) | 미확인 |
| 별도 검토 프레임과 층별 중심점 판정 | 미확인 |
| 겹침·가림·화면 밖 영역 및 모호 사례 | 미확인 |
| 측정 도구 / 기록자 / 날짜 / 검토 상태 | 미확인 |

영역의 pixel center 비교는 2D advisory 분류에만 사용할 수 있습니다. 사선 시점, 선반 면 깊이, 큰 물체가 여러 층 영역에 걸치는 경우를 해결하지 못합니다. 실제 적용 전 고정 카메라와 층별 검토 프레임에서 사람이 결과를 확인하고, calibration ID 및 원본 hash를 보존해야 합니다.

저장된 원본 PNG에 polygon 좌표를 입력하는 방법과 artifact 생성 절차는 [선반 층 영상 영역 초안](shelf-image-regions.md)을 따릅니다. API는 사람이 `{camera_id}.json`으로 명시적으로 활성화한 ROI draft에 한해 후보 층 비교 상태를 반환합니다. 결과는 검토용이며 후보 선택, 3D 위치 추정 또는 팔 제어로 연결되지 않습니다. 실제 카메라 장치가 depth를 지원하더라도 현재 관제 경로는 이를 수집하지 않습니다.

## 4. 선반 평면 대응점 기록

RGB 전용 고정 카메라 MVP를 선택했을 때만 이 절차를 사용합니다. 대상이 놓인 표면이 하나의 고정 평면으로 표현되는지 먼저 확인합니다. 병·상자 높이의 실제 파지점은 선반 평면의 `(x,y)`만으로 얻을 수 없습니다.

1. 같은 고정 카메라에서 체커보드 내부 코너 수, 칸의 실측 길이와 단위를 기록하고 여러 위치/기울기에서 같은 해상도 사진을 촬영합니다.
2. `scripts/calibrate_camera_intrinsics.py`로 내부 파라미터 **초안**을 계산하고 입력 사진과 코너 분포를 사람이 검토합니다.
3. 화면에 잘 분포한 물리 기준점 최소 6개를 정합니다. 4개 이상은 `fit`, 나머지 최소 2개는 homography 계산에서 제외할 독립 `validation`으로 기록합니다.
4. 모든 기준점의 이미지 픽셀과 선반 평면 좌표를 같은 원점·축·단위로 기록합니다. 선반에 적힌 눈금 또는 측정 도구의 출처를 남깁니다.
5. `scripts/calibrate_shelf_plane.py` 결과의 각 점 잔차와 독립 validation 오차를 원본 영상에서 검토합니다. 이 도구는 합격 허용 오차를 정하지 않으며, 작은 수치만으로 보정을 승인하지 않습니다.

대응점 입력 JSON의 `schema_version: 1` 형식은 [카메라 보정 절차](camera-intrinsic-calibration.md#고정-선반-평면-변환-초안)에 있습니다. `source_image`는 입력 JSON과 같은 폴더 또는 하위 폴더에 실제 파일로 두어야 합니다. 보정 산출물에는 입력 JSON 및 각 참조 사진의 SHA-256이 남으므로, 측정 후 원본 JSON과 사진도 해당 artifact와 함께 보관합니다. 다음 값은 실제 측정 후 채웁니다.

| 측정 항목 | 현장 기록 |
|---|---|
| camera ID와 원본 이미지 해상도 | 미확인 |
| 선반 평면 frame ID | 미확인 |
| 평면 원점의 물리적 정의 | 미확인 |
| +X/+Y의 물리적 방향과 관찰 방향 | 미확인 |
| 평면 좌표 단위 | 미확인 |
| 기준점별 `(u,v) pixel ↔ (x,y) plane` 대응과 사진 파일명 | 미확인 |
| fit/validation 구분 | 미확인 |
| 오차 합격 기준과 승인자 | 운영 결정 필요 |

## 5. OMX 기준과 안전 범위

카메라→고정 선반 평면 변환은 OMX base/tool frame 변환이 아닙니다. 다음 사항은 공식 사양, 실제 ROS/드라이버 인터페이스 또는 승인된 현장 계측 출처로 별도 확인해야 합니다.

| 확인 항목 | 기록 또는 출처 |
|---|---|
| OMX 기구 모델/URDF 버전 및 관절 이름·방향·제한 | 미확인 |
| 명령 좌표 frame ID, 축, 원점, 길이 단위 | 미확인 |
| 각 카메라 optical frame ID 및 camera-to-base / camera-to-link extrinsic 출처 | 미확인 |
| Wrist camera-to-tool/link hand-eye calibration 대응 자료와 독립 검증 오차 | 미확인 |
| 카메라 영상 시각과 source가 확인된 joint state의 시간 정렬 오차 | 미확인 |
| tool center point 정의와 gripper 접촉면 | 미확인 |
| 실제 허용 작업 공간과 충돌 장면 정보 | 미확인 |
| 그리퍼 최소/최대 유효 개방 폭, 구동 신호 의미 | 미확인 |
| 작업 명령 수락·진행·취소·최종 결과 신호 | 미확인 |
| 물체 파지/놓기 성공을 확인할 독립 신호 | 미확인 |
| 물리 비상 정지 및 장비 안전 인터록 확인 담당자 | 미확인 |
| 모의/hardware 동시 제어를 막는 운영 설정 | 미확인 |

### TCP·그리퍼 파지 기하 입력

MuJoCo의 `link6`는 시뮬레이션 모델의 body origin입니다. 물리 로봇에서 대응하는 frame이나 그리퍼 TCP라고 확인된 값이 아닙니다. 아래 항목이 확인되기 전에는 `link6` pose를 물체 grasp pose로 사용하거나 그리퍼 joint 목표를 개방 폭으로 환산하지 않습니다.

| 측정 항목 | 현장 기록 | 측정 근거/불확실성 |
|---|---|---|
| 실물 OMX 변형/그리퍼 모델·revision·serial 및 대응하는 공식 description | 미확인 | 명판·제조사 자료·장치 description hash |
| 실제 flange/손목 기준 frame 이름과 원점 정의 | 미확인 | 확인한 URDF/TF 또는 제조사 도면 |
| TCP 기준 frame 이름과 실물 tool point 정의 | 미확인 | 접촉 위치와 대상 작업 기준으로 사람이 정의 |
| TCP translation `(x,y,z)` 및 orientation quaternion `(w,x,y,z)` relative to named parent frame | 미확인 | 측정 장치·반복 측정값·오차 범위 기록; MuJoCo pose와 혼합 금지 |
| 실물 측정 parent frame과 MuJoCo `link6` 간 대응 transform 및 근거 | 미확인 | 대응하는 robot revision/model frame을 확인하고 공통 기준점에서 변환을 측정·검증; 이름 유사성만으로 대응시키지 않음 |
| 좌/우 finger 식별, 닫힘 방향 및 실제 물체 접촉면 표시 | 미확인 | 실제 장치 사진에 접촉면을 표시하고 frame/축 기록 |
| TCP frame 기준의 closing/approach unit axis 3D 방향 | 미확인 | 축 표시 사진/fixture 측정과 축 방향 uncertainty; 두 축의 직교성 검토 |
| 좌/우 finger contact patch vertex 목록, TCP frame `(x,y,z)` m | 미확인 | 접촉 pad만 캘리퍼스/도면으로 측정; 각 patch 최소 3점 및 source file hash |
| 측정된 최소·최대 contact-surface gap (m)과 어느 점/방향을 잰 거리인지 | 미확인 | 실물 접촉면에서 직접 측정; 숫자는 geometric observation이며 안전 허용치가 아님 |
| gap, contact point, 축 방향별 measurement uncertainty | 미확인 | 도구 정밀도와 반복 측정으로 작성; 계산기/mesh 기본값으로 대체 금지 |
| 그리퍼 state/register/command 값과 실제 접촉면 거리 대응 | 미확인 | 단위·방향·영점·반복 오차; 승인된 감독 시험이 필요한 경우 별도 승인 전 미측정 |
| 파지 대상별 허용 폭·접촉 위치·하중 조건 | 미확인 | 물체 실측/공식 질량 출처와 승인된 작업 시나리오 |
| 파지 성공/미끄러짐을 확인하는 독립 관측 신호 | 미확인 | 실제 sensor/controller interface와 terminal meaning |

실제 frame 대응과 TCP 측정이 끝난 뒤에만 [TCP 변환 artifact schema](link6-tcp-transform.schema.json)에 맞는 `DRAFT_ONLY` JSON을 작성합니다. Artifact와 두 근거 파일(frame mapping, TCP 측정 source)은 `.control-station/omx-calibrations/` 아래에 두며, 원본 측정값·불확실성·장치 revision·frame mapping 검토 정보를 기록합니다. 운영자가 수집한 근거 파일을 이 디렉터리에 복사하고, 실제 TCP와 대응 관계를 확인한 후에만 `scripts/create_link6_tcp_transform_draft.py`를 사용합니다. 생성기는 link6 기준으로 이미 계산된 TCP 변환을 요구하며 frame 변환이나 측정을 대신하지 않습니다. 파일 계약과 source hash는 MuJoCo 비의존 `scripts/validate_link6_tcp_transform.py --artifact <artifact.json>`으로 확인할 수 있습니다. MuJoCo IK는 `--tcp-transform-json`으로 검증된 artifact를 읽을 수 있지만, 파일 구조·해시 확인은 물리 변환의 정확성이나 로봇 안전 승인이 아닙니다. 현재 저장소에는 근거 자료나 실측 artifact가 없습니다.

그 다음 단계에서 측정된 tool geometry를 machine-readable 초안으로 기록하려면 [gripper geometry measurement schema](gripper-geometry-measurement.schema.json)를 사용합니다. 작성기 입력은 [measurement input schema](gripper-geometry-measurement-input.schema.json)에 맞춥니다. JSON은 같은 물리 모델/revision과 TCP calibration ID를 참조하고, TCP 좌표계에서 측정한 jaw closing/approach 단위축, 두 finger 접촉 패치 polygon, 측정된 min/max 접촉면 간격 및 원본 evidence hash를 담습니다. `scripts/create_gripper_geometry_draft.py --measurement-record <record.json> --output <geometry.json>`은 관리 디렉터리 안의 운영자 측정 기록과 검증된 TCP artifact를 연결해 no-overwrite `DRAFT_ONLY` artifact를 생성합니다. 측정 기록 JSON 자체가 hash-linked evidence source가 되며, 입력과 출력 모두 `.control-station/omx-calibrations/` 아래에 보관합니다. 생성된 파일은 `scripts/validate_gripper_geometry.py --artifact <geometry.json>`으로 다시 확인합니다. 검증기는 schema 필드, 축 단위/직교성, 접촉 polygon의 중복점·퇴화 vertex 순서 면적, 간격 범위, TCP artifact의 source hash와 모델 일치, geometry evidence hash를 확인합니다. 결과는 계약과 source가 일치한다는 뜻만 가지며 측정 정확도, 실제 접촉 가능성 또는 파지 성공을 판정하지 않습니다. 현재 저장소에는 현장 geometry measurement record나 artifact가 없고, 이 도구들은 robot control 입력을 만들지 않습니다.

생성기에는 다음 값이 필요합니다: 실물 모델명/revision과 물리 parent frame, link6 좌표계로 정리한 TCP translation (m) 및 normalized quaternion `(w,x,y,z)`, mapping/TCP 측정 근거 파일, 측정 방법·timezone이 포함된 ISO 8601 시각·확인자, translation/orientation uncertainty, 그리고 mapping 근거를 사람이 대조했다는 `--frame-mapping-reviewed` 확인. 두 근거 역할은 같은 파일에서 함께 설명해도 됩니다. 이 플래그는 검토 사실을 artifact에 기록할 뿐, 값을 승인하거나 변환의 정확성을 증명하지 않습니다. 출력 파일은 기존 파일을 덮어쓰지 않으며 관리 디렉터리 바로 아래 JSON으로만 생성됩니다. 실측 없는 예시 숫자를 넣어 실행하지 않습니다.

기록 순서는 실물 변형과 기준 frame 출처 확인, TCP 및 손가락 접촉면의 치수화, 개방 폭 정의, 상태값과 실제 폭의 대응 근거 확인, 승인된 제한 시험에서 반복 오차 확인입니다. 하드웨어를 움직이는 단계는 이 문서 작성이나 MuJoCo 계산만으로 허가되지 않습니다. 감독자·중단 조건·비상 정지 절차·명령 인터페이스가 확인된 별도 승인 절차에서만 수행합니다. MuJoCo collision mesh 거리나 pose IK 수렴은 위 물리 측정값과 파지 성공을 대신하지 않습니다.

저장소 MuJoCo joint origins/axes와 ROBOTIS upstream OMX-F URDF의 정적 대조는 확인됐으나, URDF 관절 제한은 MJCF 제한과 다르고 설치 장치가 이 모델 revision과 일치하는지는 검증되지 않았습니다. 현장 작성자는 이 칸을 실제 장치 모델명/공식 URDF 및 펌웨어·설정 출처로 채워야 합니다.

이 정보가 확인되기 전에는 MuJoCo의 placeholder 치수나 관절 범위, LeRobot의 정규화 `*.pos`, 임의 world 좌표를 실제 장치 값으로 변환하지 않습니다. 자동 작업 adapter는 명령 접수와 terminal 결과를 구분해 제공하는 실제 인터페이스를 확인한 뒤 설계합니다.

## 6. 자료 기록 형식

각 조사 결과에는 다음 메타데이터를 추가합니다.

```text
항목:
값과 단위:
출처 또는 측정 도구:
확인 일시 / 확인자:
원본 사진·파일·공식 문서:
적용 대상(camera ID / frame / 장치 ID):
불확실성 또는 제한:
운영 검토 상태: 미검토 / 검토됨 / 승인된 시험 범위에 사용 가능
```

운영 검토 상태는 엔지니어링 노트의 표기일 뿐, 장비의 READY 신호나 안전 인터록을 대신하지 않습니다. 실제 하드웨어 동작은 별도 승인된 현장 절차와 사용자가 허가한 실행에서만 검토합니다.
