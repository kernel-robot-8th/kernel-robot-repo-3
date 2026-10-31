# OMX 관절 목표 파일 형식

이 형식은 LeRobot OMX follower의 단일 관절 목표 입력을 오프라인에서 확인하기 위한 첫 구현입니다. 검증 도구는 장치에 연결하지 않고 목표 파일을 검사할 뿐, 모터 명령을 보내지 않습니다.

## 확인한 LeRobot 계약

로봇 환경의 LeRobot 0.4.4 소스에서 OMX follower의 action key는 `shoulder_pan.pos`, `shoulder_lift.pos`, `elbow_flex.pos`, `wrist_flex.pos`, `wrist_roll.pos`, `gripper.pos`입니다. 현재 관제 런처의 기본 프로필은 `use_degrees`를 설정하지 않으므로 LeRobot 기본 정규화 모드를 사용합니다. 이 표현에서 팔 관절은 `[-100, 100]`, 그리퍼는 `[0, 100]` 범위입니다.

이 범위는 LeRobot 입력 표현의 범위이지, OMX의 물리적 안전 한계나 도달 가능 영역을 뜻하지 않습니다. 관절 목표 실행 전에 추가로 관절 한계, 현재 위치 대비 변화량, 속도, 정지 동작을 확인해야 합니다.

## JSON 형식

```json
{
  "schema_version": 1,
  "robot_type": "omx_follower",
  "position_mode": "lerobot_default_normalized",
  "positions": {
    "shoulder_pan.pos": 0,
    "shoulder_lift.pos": 0,
    "elbow_flex.pos": 0,
    "wrist_flex.pos": 0,
    "wrist_roll.pos": 0,
    "gripper.pos": 0
  }
}
```

숫자는 예시일 뿐 실제 자세나 권장 목표가 아닙니다. CLI는 최대 64 KiB의 비-symlink regular file만 읽고 JSON 중복 키를 거부합니다. schema version은 bool이 아닌 정수 `1`이어야 합니다. 파일은 필수 키의 완전성, 타입, 유한 숫자 여부, 위의 LeRobot 입력 범위만 검사합니다. 키 누락이나 추가, degree 표현, 범위 초과 및 float 변환이 넘치는 정수도 거부합니다.

## 오프라인 검사

```bash
python3 scripts/validate_omx_pose.py path/to/pose.json
```

성공 출력은 입력 형식이 유효하다는 뜻일 뿐, 해당 목표가 물리적으로 안전하거나 도달 가능하다는 뜻은 아닙니다. 다음 단계에서 현재 관절 상태 대비 제한과 장치 제어 경로를 별도로 설계해야 합니다.
