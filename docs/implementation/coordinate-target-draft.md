# 좌표 목표 초안

관제 화면의 좌표 입력 폼은 입력한 XYZ 값과 좌표계 이름, 단위를 `.control-station/coordinate-target-drafts/` 아래 JSON 파일로 저장합니다. API는 `frame_id`, `unit`, `position`만 받으며 `position`에는 `x`, `y`, `z`만 허용합니다. 파일에는 `DRAFT_ONLY` 상태와 `arm_command_sent: false`가 기록됩니다. 이 기능은 IK 계산, 팔 명령 전송, 카메라 영상의 픽셀 좌표 변환을 하지 않습니다.

## 사용 방법

1. `./run-control-station.sh`로 관제를 실행하고 `http://127.0.0.1:8000`을 엽니다.
2. **좌표 목표 초안**에서 좌표계 이름, 단위(m 또는 mm), X/Y/Z를 입력합니다.
3. **초안 파일 저장**을 누르면 JSON 파일 경로와 저장 내용이 표시됩니다.

좌표계 이름은 입력자가 적는 문자열이며, 관제에서 실제 로봇 프레임인지 확인하지 않습니다. 파일은 다음과 같은 형태입니다.

```json
{
  "schema_version": 1,
  "id": "생성된 UUID",
  "created_at": "UTC 시각",
  "status": "DRAFT_ONLY",
  "arm_command_sent": false,
  "frame_id": "입력한 좌표계 이름",
  "unit": "m 또는 mm",
  "position": { "x": 0.0, "y": 0.0, "z": 0.0 }
}
```

파일은 Git에서 제외되는 프로젝트 숨김 폴더에 저장되며, 다른 위치를 쓰려면 `CONTROL_STATION_COORDINATE_DRAFT_DIR` 환경 변수를 설정합니다. 초안 저장은 작업 이력이나 로봇 상태를 바꾸지 않습니다.

## 후속 조건

이 초안을 실제 목표로 실행하기 전 OMX용 URDF와 말단 프레임, 기준 좌표계 변환, 위치·속도 한계, 현재 위치 대비 허용 변화량, 완료 피드백을 확인해야 합니다. LeRobot OMX의 `send_action`은 관절 목표를 받으므로, 좌표 목표를 관절 값으로 바꾸는 검증된 IK 경로가 추가로 필요합니다. 카메라 화면을 클릭해 위치를 지정하려면 카메라 보정과 카메라-로봇 좌표 변환도 먼저 마련해야 합니다.
