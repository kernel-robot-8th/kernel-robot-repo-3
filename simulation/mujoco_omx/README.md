# ROBOTIS OMX-AI MuJoCo 시뮬레이터

이 디렉터리는 ROBOTIS 공식 `robotis_mujoco_menagerie`의 OMX MJCF/mesh를 재사용해 만든 독립 실행형 MuJoCo 실험 환경이다. ROS 2, 카메라 장치, DYNAMIXEL 포트는 열지 않는다. 공식 소스의 로봇 link, joint, actuator, mesh, inertia는 직접 수정하지 않는다. 작업 장면은 [pick_place_scene.xml](robotis_omx/pick_place_scene.xml), 제어 API는 [omx_mujoco](omx_mujoco/)에 둔다.

공식 OMX는 5-DOF 팔과 그리퍼로 구성된다. MuJoCo 모델은 팔 `Joint1`–`Joint5`, 구동 그리퍼 `Gripper`, 그리고 equality constraint로 반대 회전하는 `Gripper_mimic`을 제공한다. 사용자 API에는 여섯 개의 독립 명령 대상(팔 5축 + 그리퍼 1축)만 노출한다.

## 설치

저장소 루트에서 전용 환경을 준비한다. MuJoCo는 관제 서버의 기본 의존성이 아니다.

```bash
python3 -m venv .venv-mujoco
source .venv-mujoco/bin/activate
python -m pip install -r requirements-mujoco.txt
```

확인한 개발 환경은 Python 3.12.3, MuJoCo 3.17.0, NVIDIA RTX 5080 및 X11이다. CPU/EGL 환경에서도 Python RGB 렌더를 사용할 수 있다.

## 실행

기본 GUI는 기존 2층 선반과 딸기 장면 및 저장한 시작 자세를 유지한다. Run/Pause와 actuator control은 MuJoCo 창에서 사용한다.

```bash
python scripts/run_simulation.py
```

테이블, cube, box만 있는 독립 물체 제어 장면을 확인하려면 선택해서 실행한다. 기본 선반/딸기 장면은 그대로 유지된다.

```bash
python scripts/run_simulation.py --scene simulation/mujoco_omx/robotis_omx/pick_place_scene.xml
```

관절값은 라디안이며 지정한 대상은 모델 joint/actuator 범위 검증 후 smoothstep 보간한다.

```bash
python scripts/joint_control.py
python scripts/joint_control.py --joint Joint1=0.2 --joint Joint2=0.4 --duration 1.5
python scripts/gripper_control.py open
python scripts/gripper_control.py close --duration 1.0
```

RGB 이미지는 고정 카메라와 손목 카메라에서 Python API로 렌더해 PNG로 저장한다. OpenCV 설치 시 `--display` 옵션으로 창에 띄울 수 있다.

```bash
MUJOCO_GL=egl python scripts/camera_viewer.py
MUJOCO_GL=egl python scripts/camera_viewer.py --display
```

기본 PNG 저장 위치는 `.control-station/mujoco-python-camera/`다. 손목 카메라의 pose는 기존 placeholder MJCF 카메라를 재사용해 API에서 `link6`에 붙인다. 두 카메라 pose와 작업대/물체 크기는 시각화용이며 실제 카메라 calibration 또는 실물 치수 자료가 아니다.

## 현재 확인 범위

- 공식 OMX MJCF와 mesh를 읽고 MuJoCo 모델을 compile한다.
- 5개 arm joint 및 구동 gripper actuator의 현재값, 한계, 보간 제어를 제공한다.
- 그리퍼의 `Gripper_mimic` 반대 회전 constraint를 MuJoCo equality로 사용한다.
- 작업대 위 free-body cube/box, fixed RGB camera, wrist RGB camera가 포함된다.
- 정적 joint 제어와 camera render는 실물 장치 없이 실행된다.

`strawberry_pick_scene.xml`은 기존 두 층 선반·딸기 장면을 include해 유지하면서 Python API에서 딸기에만 free joint를 더하는 별도 실험 장면이다. 기본 GUI 장면과 공식 OMX MJCF는 바꾸지 않는다. 이 장면에서 고정 카메라의 합성 RGB-D로 빨간 영역을 검출하고 딸기 중심을 복원하는 테스트를 추가했다. 이 검출기는 색상 threshold와 장면의 대략적인 딸기 반경을 쓰므로 실제 카메라 보정이나 일반 물체 인식 기능은 아니다.

공식 그리퍼 collision mesh는 손가락 끝 접촉에 부적합해 딸기를 밀어내는 문제가 있었다. 별도 `strawberry_pick_scene.xml` 실험에서만 활성화되는 opt-in 접촉 프록시를 추가해 손가락 끝 접촉을 근사했고, 중심 높이 파지 시나리오에서 RGB-D 합성 관측 → 접근 → 닫기 → 들어올리기 → 유지가 성공했다. 이는 접촉 형상을 단순화한 시뮬레이션 결과이며 공식 mesh의 파지 검증이나 실물 파지 증거가 아니다. 자세한 수치와 영상을 아래 결과 기록에 연결한다.

## 검증 결과 — 2026-10-31

```bash
MUJOCO_GL=egl .venv-mujoco/bin/python -m unittest tests/test_omx_mujoco_api.py
```

9개 테스트가 통과했다. 팔 joint target 보간/범위 거부, 그리퍼 mimic 회전, cube/box의 테이블 안착, 카메라 렌더, 딸기의 선반 안착, 기본 사진 대응 scene 보존을 확인했다. 별도 strawberry scene에서 고정 카메라의 합성 RGB-D 중심 추정 오차도 10 mm 이내였다. MuJoCo GUI의 기본 사진 대응 선반/딸기 scene은 유지한다.

딸기 중심 높이 파지는 보정한 반복성 스크립트로 합성 장면에서 재검증했다. seed `20261031`, 딸기 X/Y 오프셋 ±6 mm 조건의 20회가 모두 통과했다. 실제 검증 기준은 관측 중심 오차 ≤3 mm, 두 손끝 높이 차 ≤5 mm, 프리그라스프 중 물체 이동 ≤3 mm, 양쪽 프록시 접촉, 30 mm 이상 들어올림, 1초 유지 중 이동 ≤10 mm다. 결과는 [재현성 JSON](../../docs/implementation/assets/strawberry-repeatability-20261031.json)에 있다. 이전 기록의 lift/drift 수치는 잘못된 mimic 관절 동기화에 기반해 대체되었다. 이번 결과는 중심 상승 90.44–94.83 mm, 1초 유지 변위 0.84–1.74 mm다.

재현 명령:

```bash
MUJOCO_GL=egl .venv-mujoco/bin/python scripts/run_mujoco_strawberry_repeatability.py --trials 20 --seed 20261031 --save-frames
```

집은 딸기를 로봇 뒤쪽 바닥으로 운반하는 별도 [실험 스크립트](../../scripts/run_mujoco_strawberry_place_behind.py)도 추가했다. 접촉만으로 운반하는 기본 `contact` 모드는 다섯 위치에서 0/5였으며, 물체가 운반 중 미끄러졌다. 따라서 경로와 안착을 분리 검증하는 `weld` 모드를 추가했다. 양쪽 손끝 접촉을 확인한 뒤 MuJoCo equality weld로 물체를 붙여 운반하고, 목표에 내려온 후 weld를 해제해 바닥 안착을 검사한다. weld 모드의 20회(5개 바닥 목표 × 4회)는 20/20 성공했다. 이는 마찰만으로 물체를 잡은 성공이나 실물 하드웨어 성공을 뜻하지 않는다. 결과는 [상세 실험 기록](../../docs/implementation/mujoco-live-pose-experiment-2026-10-31.md#딸기를-로봇-뒤쪽-바닥에-놓기), [20회 weld JSON](../../docs/implementation/assets/strawberry-place-behind-results.json), [접촉만 사용한 5회 JSON](../../docs/implementation/assets/strawberry-place-behind-contact-only-results.json), [완료 장면](../../docs/implementation/assets/place-behind-trial-01-released-overview.png)에 있다.

```bash
MUJOCO_GL=egl .venv-mujoco/bin/python scripts/run_mujoco_strawberry_place_behind.py --trials 20 --seed 20261031 --grasp-model weld
```

`--grasp-model contact`은 손끝 마찰만으로 운반하는 물리 접촉 시험이다. `--grasp-model weld`는 양쪽 접촉이 최초 확인된 뒤 시뮬레이션 equality constraint를 켜므로 운반 경로/충돌/방출 검증에만 사용한다. 두 결과를 혼동하지 않는다.

두 실험 모두 MuJoCo 합성 카메라와 추정 위치의 6 mm 구형 접촉 프록시를 사용한다. 실제 카메라 보정, 공식 그리퍼 collision mesh의 파지, 실물 로봇 동작은 검증하지 않았다.
