# 실물 관측 자세의 MuJoCo 실험 — 2026-10-31

실물 OMX 관절 snapshot을 바탕으로 MuJoCo 모델에 **근사 시작 자세**를 만들고, placeholder 선반의 물체 위 접근점으로 위치 IK를 탐색한 기록이다. 실물 동작을 재현하거나 실물 명령을 만들지 않았다.

## 모델에 포함된 카메라

MuJoCo scene에는 실제 카메라 장비가 아니라 합성 카메라 3개가 정의되어 있다.

| 이름 | MJCF 배치 | 용도/제한 |
|---|---|---|
| `placeholder_fixed_camera` | world, `(0.05, 0.25, 0.85) m`, 위에서 내려다보는 임시 방향 | 합성 선반 장면을 관측 |
| `placeholder_wrist_camera` | `link6` 아래 `(0.045, 0, 0.015) m` | 임시 손목 시점; 실물 장착값/방향과 무관 |
| `placeholder_wrist_aimed_candidate` | `link6` 아래, placeholder 물체를 향하도록 만든 임시 자세 | 홈 자세용 시뮬레이션 후보; 실물 카메라 보정이 아님 |

MJCF의 fixed camera는 실물 정면 카메라와 다르게 위에서 내려다본다. 현재 MuJoCo 렌더는 시뮬레이션 내 모델과 placeholder 물체를 보여줄 뿐, 실제 카메라 영상이나 그 시야를 복제하지 않는다.

## 시작 상태와 엔코더 환산 가정

2026-10-31 10:03:33 KST의 read-only 실물 snapshot을 시작값으로 사용했다. LeRobot에서 읽은 DYNAMIXEL `Present_Position` raw값에는 해당 시점의 `Homing_Offset=0` 및 `range_min=0`, `range_max=4095` 설정이 확인됐다. 하지만 이 설정만으로 MuJoCo 관절 영점, 회전 방향, 관절별 기계 변환이 보정되지는 않는다.

시뮬레이션을 시험하기 위해 임시로 `raw × 2π / 4096`, 양의 방향, 영점 0을 가정했다. 각도는 `[-π, π]`로 감싼 뒤 MJCF 관절 범위 안에 두었다. 이 단순 환산은 calibration artifact가 아니다.

| 실물 관절 | Raw | MuJoCo 관절 | 임시 `qpos` (rad) | 비고 |
|---|---:|---|---:|---|
| `shoulder_pan` | 1732 | `Joint1` | 2.65685 | 직접 환산 |
| `shoulder_lift` | 729 | `Joint2` | 1.11827 | 직접 환산 |
| `elbow_flex` | 2910 | `Joint3` | -1.81930 | 2π wrap 적용 |
| `wrist_flex` | 2942 | `Joint4` | -1.74533 | 환산값 -1.77021 rad가 MJCF 하한 -1.745329 rad를 1.47° 벗어나 근접 제한값으로 clamp |
| `wrist_roll` | 1934 | `Joint5` | 2.96672 | 직접 환산 |
| `gripper` | 2403 | `Gripper` | 1.02418 | raw 0–4095가 0–100°에 비례한다는 미검증 표시용 가정 |

`Joint4` clamp와 그리퍼 환산 가정 때문에 이 값들은 정확한 실물 자세가 아니다. MuJoCo 모델 link6 body origin으로 계산한 임시 위치는 `[-0.18315, 0.09888, 0.25042] m`이고, placeholder 물체 중심 `[0.05, 0.25, 0.27] m`까지의 거리는 0.27853 m였다. 여기서 link6 body origin은 실측 TCP가 아니다.

## 실행한 시뮬레이션 탐색

현재 근사 시작 상태에서 position-only DLS IK를 placeholder 물체 중심 위의 점으로 실행하고, 현재→후보 joint 선형 보간 50개 표본에서 MuJoCo 접촉을 확인했다. MuJoCo 3.17.0에서 수행했으며 ROS, 카메라 장치, serial port는 열지 않았다.

| link6 목표 (m) | IK 위치 오차 | 종점/경로 관측 |
|---|---:|---|
| `[0.05, 0.25, 0.32]` | 4.8 mm | 선반 deck 및 placeholder 물체와 겹침. 선형 경로는 첫 표본부터 접촉. |
| `[0.05, 0.25, 0.37]` | 34.0 mm | IK 400회 안에 수렴하지 못함. 종점/경로에 선반 접촉. |
| `[0.05, 0.25, 0.42]` | 71.8 mm | IK 400회 안에 수렴하지 못함. 종점/경로에 선반 접촉. |

계산에서 시작 자세는 MuJoCo 접촉 0건이었지만, 물체 위 접근 후보들은 현재 placeholder geometry를 지나가거나 목표 위치에 수렴하지 못했다. 이 결과는 실물 선반에 대한 충돌 판정이 아니다. placeholder 선반은 실제 선반 치수/개구부로 측정되지 않았고, 빨간 물체는 딸기 mesh가 아니라 5 cm 반폭 box다.

아래 렌더는 위 임시 환산 자세를 placeholder scene에 적용한 화면이다. fixed view는 top-down 합성 시점이며, 두 wrist view도 실물 카메라 보정이 아니다.

- [MuJoCo fixed camera](assets/omx-live-pose-2026-10-31/mujoco-current-fixed-camera.png)
- [MuJoCo synthetic wrist camera](assets/omx-live-pose-2026-10-31/mujoco-current-wrist-camera.png)
- [MuJoCo wrist-aimed candidate camera](assets/omx-live-pose-2026-10-31/mujoco-current-wrist-aimed-candidate.png)
- 대응하는 실물 시점: [정면 카메라](assets/omx-live-pose-2026-10-31/front-after-approach.png), [손목 카메라](assets/omx-live-pose-2026-10-31/wrist-after-approach.png)

## 다음 단계

이 장면으로 실물 동작 방향을 결정하지 않는다. 우선 실제 OMX joint zero/sign/scale과 현재 하드웨어 revision에 대응하는 description을 확보하고, 실물 카메라 intrinsics/extrinsics 및 실제 선반 치수를 측정해 모델을 맞춰야 한다. 그 다음 실제 관절 snapshot을 MuJoCo `qpos`에 같은 시각으로 대응시키고, wrist camera와 fixed camera의 투영을 비교한다. 매핑이 검증되기 전 IK 결과는 시뮬레이션 탐색 전용으로 유지한다.

## 사진 기반 2층 장면

정면 사진에서 보이는 작업 배치에 맞춰 기존 IK 실험 장면은 보존하고 별도 [사진 대응 scene](../../simulation/mujoco_omx/robotis_omx/shelf_photo_match_scene.xml)을 추가했다. 이 장면은 2개 선반 단을 두고, 예전 단층 placeholder보다 아래에 배치했다. 아래/위 deck 중심 높이는 각각 0.060 m와 0.170 m이며, 딸기 모형은 위 deck에 놓았다.

새 `photo_match_front_camera`는 실물 정면 카메라처럼 선반 앞쪽에서 보는 **시각 비교용 합성 시점**이다. [초기 사진 대응 렌더](assets/omx-live-pose-2026-10-31/mujoco-photo-match-two-tier-current.png)는 두 단, 딸기 proxy, 팔과 선반 간격을 한 화면에 보여준다. 선반 치수·높이, 딸기 크기, 카메라 pose는 사진에서 미터 단위로 보정한 값이 아니라 시각적 근사값이다. current pose도 encoder zero/scale 불확실성 때문에 정확한 실물 자세 복제는 아니다.

사용자 피드백에 따라 선반을 멀리 옮긴 뒤 다시 가까이 조정했다. 현재 딸기 중심은 world `(0.10, 0.21, 0.215) m`이고 로봇 베이스 원점에서 수평 거리 약 `0.233 m`다. 이 값은 지금 scene의 배치값이며 실물 측정값은 아니다. [가까이 조정한 렌더](assets/omx-live-pose-2026-10-31/mujoco-photo-match-nearer.png)는 팔과 2층 선반을 한 화면에 보여준다.

사용자 설명에 따라 기본 자세의 기준을 딸기 위치에서 선반 전체로 정정했다. 로봇 베이스 정면은 선반 앞면에 수직으로 선반 안쪽을 향하고, 팔은 베이스 정면 기준 오른쪽으로 돌아간다. 이 scene에서는 로봇 정면을 local `+X`, 선반 안쪽을 world `+Y`로 정의해 베이스 yaw를 `+90°`로 둔다. `Joint1`은 약 `-65°`다. 다른 관절은 raw snapshot을 임시 환산한 값이다. 이 방향과 각도는 사진 맞춤용 추정이며, 실제 베이스 heading이나 joint zero/direction을 계측한 값은 아니다.

MuJoCo 기본 Simulate GUI에서 Run/Pause와 actuator control을 같은 창에서 조작하도록 [실행 스크립트](../../scripts/run_mujoco_photo_scene.py)를 추가했다. 실행 명령은 `unset MUJOCO_GL; .venv-mujoco/bin/python scripts/run_mujoco_photo_scene.py`다. 현재 MuJoCo 3.17.0 기본 GUI가 사진 대응 scene 및 근사 자세로 실행 중이며, 실물 로봇과 카메라는 열지 않았다. 모든 조작은 MuJoCo 모델에만 적용된다.

## 딸기 중심 높이 파지 시도 — 합성 카메라와 접촉 프록시

화면상 팔이 딸기 아래로 들어가는 것처럼 보이는지 확인하기 위해 좌표와 접촉을 점검했다. 이 scene의 바닥은 `z=0 m`, 아래 선반의 윗면은 `z=0.072 m`, 딸기가 놓인 위 선반 윗면은 `z=0.182 m`다. 딸기 중심은 약 `z=0.220 m`이므로 로봇이 바닥을 뚫은 것은 아니다. 고정 카메라의 원근과 선반 앞쪽에서 들어오는 팔의 시점 때문에 접근 방향이 아래쪽처럼 보일 수 있다.

아래 시도에서는 손끝 두 접촉점의 중간 높이를 딸기 중심에 맞추고, 두 손끝 높이도 서로 같게 하는 조건을 추가했다. 목표는 장면의 정답 좌표를 직접 쓰지 않고 고정 합성 카메라 RGB-D 관측의 중심 추정치 `(0.09697, 0.21250, 0.21940) m`를 사용했다. 장면 내부 딸기 중심 `(0.10000, 0.21000, 0.22000) m`과의 오차는 3.97 mm였다.

| 항목 | 결과 |
|---|---:|
| 손끝 중간점과 카메라 관측 중심 오차 | 1.28 mm |
| 두 손끝 간 높이 차이 | 1.12 mm |
| 선반 상판과 관절 경로 정적 접촉 | 없음 (201개 보간 자세 확인) |
| 닫기 후 양쪽 손끝 접촉 | link6 및 link7 프록시 모두 검출 |
| 들어올린 뒤 딸기 중심의 위 선반 높이 | 92.46 mm |
| 들어올리고 1초 유지한 뒤 접촉 | 양쪽 손끝 프록시 유지 |

- [중심 높이 정렬 파지 시험 영상](assets/strawberry-vision-centerheight-test.mp4)
- [들어올린 뒤 정지 화면](assets/strawberry-vision-centerheight-lifted-hold.png)
- [시험 정량 결과 JSON](assets/strawberry-vision-centerheight-result.json)

이 결과는 사용자 요청에 맞춰 그리퍼 손끝을 6 mm 구형 접촉 프록시로 근사한 **시뮬레이션 전용** 시험이다. 공식 그리퍼 collision mesh의 파지 능력, 실물 카메라 보정, 실제 딸기 물성, 실물 로봇 동작을 검증한 것은 아니다. 화면에서 바닥을 뚫어 집는 것이 아니라 위 선반 위에 있는 딸기의 중심 높이에서 양쪽으로 접촉해 들어올린 결과다. 다음으로는 프록시 크기/마찰과 물체 위치 오차를 바꿔 재현성을 확인하고, 성공 판정 기준과 시험 스크립트를 고정한다.

후속으로 딸기 시작 위치를 X 또는 Y 방향으로 ±6 mm 이동한 4개 배치에서도 동일한 RGB-D 중심 추정과 중심 높이 IK를 다시 계산했다. 네 경우 모두 101개 관절 보간 표본에서 선반 접촉이 없었고, 닫은 뒤 두 손끝 프록시 접촉과 55.1–56.5 mm의 중심 상승이 확인됐다. 관측 중심 대비 손끝 중간점 오차는 1.19–1.34 mm 범위였다. [네 배치의 개별 수치와 접촉 결과](assets/strawberry-offset-robustness-results.json), 시각 결과: [X −6 mm](assets/strawberry-offset-x_minus_6mm.png), [X +6 mm](assets/strawberry-offset-x_plus_6mm.png), [Y −6 mm](assets/strawberry-offset-y_minus_6mm.png), [Y +6 mm](assets/strawberry-offset-y_plus_6mm.png). 이 반복도 동일한 합성 장면·접촉 프록시를 공유하므로 실물 조건에서의 재현성을 뜻하지 않는다.

## 20회 반복성 검토

재현 가능한 [실행 스크립트](../../scripts/run_mujoco_strawberry_repeatability.py)를 seed `20261031`로 독립 실행했다. 각 회차의 딸기 X/Y 시작 위치를 기준값 주변 ±6 mm 범위에서 바꾸고, 합성 고정 카메라 RGB-D 중심 관측으로 IK를 다시 계산했다. 이 검증 전에 FK 계산에서 mimic 관절 상태가 동기화되지 않던 문제를 수정했다. 따라서 과거에 기록된 20회 결과는 폐기하고 아래 표와 [현재 JSON](assets/strawberry-repeatability-20261031.json)으로 대체한다.

| 측정값 | 20회 범위 | 평균 |
|---|---:|---:|
| 성공 판정 | 20/20 | 100% |
| 장면 기준 중심과 카메라 추정 오차 | 3.48–5.50 mm | 4.54 mm |
| 손끝 중간점과 관측 중심 오차 | 0.86–1.01 mm | 0.92 mm |
| 양손끝 높이 차 | 3.42–4.13 mm | 3.63 mm |
| 초기 중심 대비 최종 상승 | 90.44–94.83 mm | 92.60 mm |
| 1초 유지 중 중심 변위 | 0.84–1.74 mm | 1.34 mm |

성공 판정은 선반 경로 접촉 없음, 중심 정렬 오차 ≤3 mm, 손끝 높이 차 ≤5 mm, 접근 중 딸기 이동 ≤3 mm, 양쪽 손끝 프록시 접촉, 30 mm 이상 상승, 1초 유지 중 이동 ≤10 mm를 모두 만족하는 것이다. 스크립트가 MuJoCo/Python/NumPy 버전과 scene 및 코드 해시를 JSON에 기록한다. 이미지도 저장할 때 실행:

```bash
MUJOCO_GL=egl .venv-mujoco/bin/python scripts/run_mujoco_strawberry_repeatability.py --trials 20 --seed 20261031 --save-frames
```

현재 합성 장면과 ±6 mm 시작 위치 변형 조건에서는 `카메라 관측 → 중심 높이 접근 → 양손끝 접촉 → 들어올림 → 1초 유지`를 반복할 수 있다. 조명·가림·검출 편차, 실제 딸기 물성, 공식 collision mesh나 실물 로봇의 성공률을 뜻하지 않는다. 특히 이 성공 결과는 뒤쪽 바닥으로 운반하는 단계까지 포함하지 않는다.

반복 회차별 이미지를 압축해 모은 [20회 파지 contact sheet](assets/strawberry-repeatability-20261031-contact-sheet.png)를 보존한다. 원본 회차별 PNG는 재현 명령의 `--save-frames` 옵션으로 필요할 때 다시 만들 수 있다.

## 딸기를 로봇 뒤쪽 바닥에 놓기

단일 pick 검증 뒤 [운반/놓기 실행기](../../scripts/run_mujoco_strawberry_place_behind.py)를 추가했다. pick이 끝난 후 선반에서 빠져나오고, 선반 앞쪽 바깥으로 이동한 다음 Joint1을 회전해 로봇 뒤쪽 바닥 목표로 내려놓는다. 장소 후보는 `(-0.20, -0.22)`, `(-0.18, -0.20)`, `(-0.22, -0.18)`, `(-0.16, -0.22)`, `(-0.22, -0.24) m`이며 목표 딸기 중심 높이는 `0.037 m`다. 각 경로는 동작 전에 정적 접촉 검사로 확인한다.

### 접촉만 사용하는 운반 시험

기본 `--grasp-model contact` 모드는 양쪽 손끝 프록시 접촉만으로 운반한다. 6 mm 프록시, 다섯 위치, seed `20261031`에서 **0/5 성공**했다. 정적 경로 검사는 통과했지만 동적 이동 중 손끝 접촉이 풀렸다. 4 mm 프록시도 **0/5**였고, 1000 physics steps/waypoint의 매우 느린 이동도 해결하지 못했다. 결과는 [6 mm 접촉 시험](assets/strawberry-place-behind-contact-only-results.json), [4 mm 비교](assets/strawberry-place-behind-proxy4-results.json), [느린 이동](assets/strawberry-place-behind-slow-results.json)에 보존했다. 요청에 따라 접촉 상실 캡처 5장은 삭제했으며 정량 실패 기록은 JSON에 남아 있다.

### 접촉 확인 후 시뮬레이터 weld로 운반

경로와 바닥 안착을 따로 검증할 수 있도록 MuJoCo 전용 `--grasp-model weld` 모드를 추가했다. 양쪽 손끝 접촉을 먼저 확인한 뒤, 그 순간 측정된 link6–딸기 상대 pose로 MuJoCo equality weld를 활성화한다. 바닥 목표에 도달하면 weld를 끄고 그리퍼를 열어 자유 낙하와 바닥 접촉을 확인한다. 이는 **그립 마찰에 의한 운반이 아니다**. 접촉 기반 grasp는 시작 시점에만 검증하고, 물체가 빠지지 않도록 시뮬레이터 제약으로 붙여 두므로 운반 경로/충돌/놓기 절차를 검증하는 별도 모드다.

seed `20261031`, X/Y ±6 mm 시작 위치 변화, 바닥 목표 다섯 곳을 각각 네 번씩 적용한 20회 시험은 **20/20 성공**했다. 모두 선반 경로 사전 검사를 통과했고, 딸기를 연 뒤 바닥 접촉을 확인했다. 성공 기준은 바닥 목표 XY 오차 ≤15 mm, 방출 후 중심 높이 25–55 mm, 딸기와 손끝의 분리, 바닥 접촉이다.

| 측정값 | 20회 범위 | 평균 |
|---|---:|---:|
| 전체 안착 판정 | 20/20 | 100% |
| 방출 후 목표 XY 오차 | 2.57–14.24 mm | 9.90 mm |
| 방출 후 딸기 중심 높이 | 30.87–32.05 mm | — |
| 정적 경로의 충돌 사전 검사 | 20/20 통과 | — |

재현 명령:

```bash
MUJOCO_GL=egl .venv-mujoco/bin/python scripts/run_mujoco_strawberry_place_behind.py --trials 20 --seed 20261031 --grasp-model weld
```

[20회 상세 결과 JSON](assets/strawberry-place-behind-results.json)에는 trial별 target, 물체 위치, 접촉, 안착 오차, MuJoCo/Python/NumPy 버전 및 소스 해시가 들어 있다. 눈으로 보는 완료 장면은 [측면 overview](assets/place-behind-trial-01-released-overview.png)다. [도달 직전](assets/place-behind-trial-01-rear-floor.png)과 [그리퍼 개방 후](assets/place-behind-trial-01-released.png) fixed-camera 이미지도 보존한다. 재현 명령에 `--save-frames`를 추가하면 회차별 장면을 다시 캡처한다.

이 20/20 결과는 MuJoCo 장면 안에서 **접촉 확인 → weld 운반 → 뒤쪽 바닥 방출** 절차가 반복된다는 뜻이다. 접촉만으로 유지하는 운반은 아직 실패하고 있으며, weld 모드 성공을 그리퍼의 물리적 파지 성공이나 실물 로봇 성공으로 해석하지 않는다. 실제 카메라·딸기 물성·공식 collision mesh·하드웨어 동작도 검증 대상에 포함되지 않았다.

## 기본 자세 위치 미세 조정

후속 요청에 따라 베이스를 직전 시작 위치에서 world `-Y`로 3 cm(뒤), world `-X`로 3 cm(선반을 바라볼 때 왼쪽) 이동하도록 했다. yaw와 팔 자세는 유지했다. 합성 scene에서만 적용된 미세 조정이며 실물 좌표 측정값은 아니다.
