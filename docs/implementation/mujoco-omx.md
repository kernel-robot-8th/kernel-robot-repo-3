# OMX MuJoCo 시뮬레이션 초안

`simulation/mujoco_omx/robotis_omx/`는 ROBOTIS 공식 `robotis_mujoco_menagerie` OMX 모델 기반과 이 프로젝트의 임시 선반 장면을 둡니다. 선반 장면은 `omx.xml`과 같은 폴더에 있어 `meshdir="assets/"` 기준을 유지합니다. 기반 모델은 [upstream `robotis_omx`](https://github.com/ROBOTIS-GIT/robotis_mujoco_menagerie/tree/d8344c0dbe7a00208d0301111523dde65efc174a/robotis_omx) commit `d8344c0dbe7a00208d0301111523dde65efc174a`이고, 원본 mesh 및 Apache-2.0 LICENSE를 보존합니다. `omx.xml`에는 baseline 및 target-aimed 시뮬레이션 카메라를 `link6` 아래 추가했습니다. joint, actuator, robot geometry, 질량 값은 바꾸지 않았습니다. camera pose와 선반/물체 geometry는 모두 임시값이며 실물 장착/치수를 나타내지 않습니다.

## 설치 및 실행

MuJoCo는 관제 서버 기본 의존성이 아닙니다. 별도 가상환경을 권장합니다.

```bash
python3 -m venv .venv-mujoco
source .venv-mujoco/bin/activate
python -m pip install -r requirements-mujoco.txt
python scripts/inspect_mujoco_omx.py
python scripts/step_mujoco_omx.py --target Joint1=0.10 --steps 500
python scripts/solve_mujoco_omx_position_ik.py --target 0.05,0.25,0.27
python scripts/solve_mujoco_omx_position_ik.py --target 0.05,0.25,0.31 --path-samples 41
python scripts/solve_mujoco_omx_position_ik.py --target 0.05,0.25,0.31 --format json
python scripts/capture_mujoco_cameras.py --steps 500
python scripts/verify_mujoco_camera_capture.py /path/to/capture-directory
python scripts/inspect_mujoco_omx.py --viewer
```

`capture_mujoco_cameras.py`는 MuJoCo 내부 상태에서 fixed, baseline wrist 및 target-aimed wrist 후보의 RGB PNG, metric depth `.npy`, `(object_id, object_type)` segmentation `.npy`와 geom ID map manifest를 생성합니다. Manifest schema v3는 각 파일의 SHA-256/크기/shape/dtype, scene와 mesh source hash, capture 당시 simulation joint 상태, 세 camera의 simulation world pose/FOV, MuJoCo model의 `parent_body` 및 body 기준 `position_in_parent_m`/`quaternion_wxyz_in_parent`, `MUJOCO_GL` 설정을 저장합니다. schema v4는 동일한 3개 camera view에 hash-검증된 IK candidate JSON 사본을 추가하고, source candidate의 모델 hash와 arm joint 값을 capture 상태와 대조합니다. source 파일 hash는 이 attachment 값이 파생된 MJCF 입력을 식별합니다. MuJoCo `cam_bodyid`, `cam_pos`, `cam_quat`는 각각 카메라 parent body와 body 기준 pose를 나타내며 [MuJoCo API types](https://mujoco.readthedocs.io/en/3.9.0/APIreference/APItypes.html)에 정의돼 있습니다. 이 필드는 MuJoCo 모델 내부 provenance이며 ROS TF frame이나 실물 camera extrinsic이 아닙니다. verifier는 schema v1(두 기존 카메라), v2(attachment를 가진 두 카메라), v3(세 camera view), v4(세 camera view와 검증된 IK candidate 상태 출처)를 각각 구분합니다. 산출물 경로는 `CAPTURE_DIRECTORY` 기준, scene/model source 경로는 `REPOSITORY_ROOT` 기준임을 manifest에 표시합니다. Pose는 MJCF와 MuJoCo 상태에서 나온 simulation 좌표이며 실제 장착값이 아닙니다. 출력은 매번 새 `.control-station/mujoco-camera-captures/capture-*` 디렉터리에 저장됩니다. `--target Joint1=0.10`처럼 지정하면 actuator 모델 범위 안의 simulation-only 라디안 목표를 적용한 뒤 `--steps`만큼 시뮬레이션을 진행합니다. 모든 값은 MuJoCo 장면에 한정되며 ROS 토픽이나 실제 카메라 프레임으로 게시하지 않습니다. 이미지/깊이/segmentation은 RGB-D 센서의 광학·잡음·정렬 특성을 재현한다고 보장하지 않습니다. MuJoCo Python Renderer의 depth/segmentation 전환 방식과 segmentation object ID/type 표현을 사용합니다. [MuJoCo rendering documentation](https://mujoco.readthedocs.io/en/latest/programming/rendering.html)

**Headless 렌더 관측:** `MUJOCO_GL=egl .venv-mujoco/bin/python scripts/capture_mujoco_cameras.py --steps 0`으로 640×480의 세 camera RGB, depth, segmentation 산출물을 `.control-station/mujoco-camera-captures/capture-20261030T195537204777Z-09213588/`에 생성했습니다. `--steps 0`이므로 초기 keyframe에서 joint target을 적용하지 않았습니다. schema v3 manifest에 fixed camera `world` attachment와 두 wrist camera의 `link6` local pose가 기록됐고, 독립 verifier가 RGB/depth/segmentation 9개 artifact와 scene source 10개를 통과시켰습니다. fixed image에서 placeholder 대상은 1,764 pixel로 보였습니다. baseline wrist 카메라는 home 자세에서 유효 positive-depth target pixel이 없어 localization이 거부됐습니다. 대상 중심을 향하도록 계산한 별도 aimed candidate는 4,184 visible pixel을 얻었지만, RGB 확인에서는 선반 상판이 물체 하단을 가렸습니다. 이는 home 자세에 종속된 임시 MuJoCo 후보이며, 실물 장착 또는 접근 재인식 성공 증거가 아닙니다.

이후 ROS CameraPlugin 연결을 위해 `placeholder_fixed_camera`에 `resolution="640 480"` metadata를 추가했습니다. 그 변경으로 이전 capture manifest의 scene hash가 낡아져, 2026-10-31에 `.control-station/mujoco-camera-captures/capture-20261030T214237341278Z-b249aead/`를 새로 생성했습니다. 현재 source 기준 verifier가 RGB/depth/segmentation 9개 artifact와 scene source 10개를 통과했습니다. 같은 manifest를 사용하는 `localize_mujoco_geom.py`는 고정 camera의 `placeholder_pick_object_geom` 1,764 visible pixel 평균을 MuJoCo world `[0.0500, 0.2500, 0.2950] m`로 출력했습니다. 이는 box 윗면의 visible surface sample이며 object center/grasp pose가 아닙니다. 모두 simulation-only 산출물입니다.

`capture_mujoco_cameras.py --steps 0 --ik-candidate .control-station/mujoco-ik-candidates/<candidate>.json`은 현재 scene hash와 일치하고 link6 pose가 수렴했으며 기록상 종점 접촉이 없는 candidate만 받아 각 arm joint를 simulation qpos에 적용합니다. `mj_forward`로 해당 상태의 MuJoCo 접촉을 다시 계산해 접촉이 하나라도 있으면 렌더를 거부하고, 무접촉일 때만 정적 endpoint view를 렌더합니다. candidate를 capture 디렉터리에 복사해 SHA-256과 함께 보존합니다. actuator target이나 simulation step은 적용하지 않습니다. `verify_mujoco_camera_capture.py <capture-directory>`는 이미 저장된 capture만 읽어 최대 1 MB의 비-symlink manifest와 scope flag, camera artifact의 SHA-256·크기·형식·배열 shape/dtype·depth 범위, simulation camera 회전행렬의 직교성/오른손 방향, 현재 저장소의 scene/model/mesh source hash를 확인합니다. schema v2/v3/v4는 카메라별 model attachment parent가 scene의 body 관계와 맞고 body 기준 위치/정규화 quaternion이 유한한지 확인합니다. v4는 candidate 사본의 hash/source/mode와 실제 capture arm joint state도 일치하는지 검사합니다. JSON 중복 key, artifact 경로 구성요소 symlink, 잘못된 경로, manifest와 다른 source 집합, 변경된 artifact는 거부합니다. 요약에는 `simulation_only: true`, `usable_for_robot_control: false`, `arm_command_sent: false`가 표시됩니다. 이 CLI는 capture를 생성하거나 renderer·ROS·카메라·로봇을 실행하지 않습니다. 현재 schema v3 capture의 camera 3개, artifact 9개와 scene source 10개가 통과했습니다.

`localize_mujoco_geom.py <capture-directory> --camera placeholder_fixed_camera --geom placeholder_pick_object_geom`은 verifier가 검사한 같은 manifest snapshot을 재사용한 뒤 지정 geom의 segmentation mask 및 유효 depth pixel을 pinhole 식으로 역투영합니다. manifest를 별도로 다시 읽지 않으며 출력 hash는 검증에 사용한 정확한 원본 bytes에 해당합니다. 계산 직전 depth/segmentation bytes도 descriptor의 크기와 SHA-256에 다시 대조하고, 해시가 일치하는 그 bytes만 메모리에서 decode해 사용합니다. 출력은 `visible_surface_pixel_mean` 하나를 선택한 synthetic camera frame 및 MuJoCo world frame으로 나타내는 JSON이며, geom 이름과 capture manifest hash를 함께 보존합니다. 식은 manifest의 vertical FOV에서 `fx=fy=height/(2*tan(fovy/2))`를 계산하고, MuJoCo camera의 centered principal point, local `-Z` forward 및 camera-plane depth 의미를 사용합니다. 카메라 frame/world pose와 depth 의미는 [MuJoCo XML reference](https://mujoco.readthedocs.io/en/latest/XMLreference.html)에서 확인한 MuJoCo 규약입니다.

이 값은 보이는 표면 pixel들의 산술 평균이며, 가려진 면이나 물체 내부를 복원하지 않습니다. 물체 기하 중심·크기·방향, 카메라 실측 보정, robot base frame/TF, grasp pose 또는 graspability는 추정하지 않습니다. 출력 scope는 `VISIBLE_SURFACE_PIXEL_MEAN_ONLY`, `simulation_only: true`, `usable_for_robot_control: false`입니다. CLI는 capture 파일을 읽기만 하고 결과 JSON을 stdout으로 보냅니다.

**IK 후보 종점 camera 관측 (2026-10-31):** `MUJOCO_GL=egl .venv-mujoco/bin/python scripts/capture_mujoco_cameras.py --steps 0 --ik-candidate .control-station/mujoco-ik-candidates/staged-y21-z32.json`으로 hash-검증 joint candidate를 직접 qpos에 적용해 schema v4 capture `.control-station/mujoco-camera-captures/capture-20261030T201138160608Z-4d313320/`를 만들었습니다. verifier는 RGB/depth/segmentation 9개와 candidate JSON 1개(총 10 artifact), scene source 10개를 확인했고, candidate source model hash와 다섯 arm joint 값이 capture manifest와 일치했습니다. fixed view에서는 target geom 1,353 visible pixels를 얻었고 world visible-surface mean은 `[0.04665, 0.25245, 0.29500] m`였습니다. 두 wrist view 모두 target geom의 유효 positive-depth pixel이 없어 localization이 거부됐습니다. RGB 검토에서도 현 link6 자세의 wrist 시점에 target이 보이지 않았습니다. 이 시점에서 baseline camera의 시선이 물체를 향하도록 계산한 별도 `link6` quaternion을 같은 위치 목표 `(0.05,0.21,0.32) m`에 pose IK로 요구했을 때 잔차는 44.26 mm/35.82°였고, 최종자세 및 이산 경로에서 shelf/deck와 물체 접촉이 있었습니다. 이 시선 quaternion은 simulation look-at 계산이지 grasp orientation이 아닙니다. 결과는 placeholder 카메라 pose와 link6 body-origin 후보에만 해당하며 실물 wrist-camera 시야나 물체 재인식 가능성을 뜻하지 않습니다.

**접근 chain wrist 시야 표본 (2026-10-31):** schema v4 capture와 IK JSON hash chain을 이용해 front-edge `(0.05,0.10,0.40)`, shelf-edge `(0.05,0.145,0.40)`, lowered `(0.05,0.145,0.32)`, approach `(0.05,0.18,0.32)`, last-clean `(0.05,0.21,0.32) m`의 다섯 정적 candidate endpoint를 각각 렌더했습니다. 각 capture는 10개 artifact, 10개 scene source와 candidate joint-state/model provenance 검증을 통과했습니다. 다섯 단계 모두 baseline 및 target-aimed wrist camera에서 target geom의 유효 positive-depth pixel이 없어 localization을 거부했고, RGB 검토에서도 target이 두 wrist frame 안에 나타나지 않았습니다. 이는 현재 synthetic camera attachment/orientation과 선택한 IK endpoint 다섯 개에 한정된 결과입니다. 물리 wrist camera의 장착, 전체 arm configuration의 시야 불가능성 또는 접근 실패를 증명하지 않습니다.

**Wrist camera 가상 부착점 시야 탐색 (2026-10-31):** `MUJOCO_GL=egl .venv-mujoco/bin/python scripts/search_mujoco_wrist_camera_views.py`는 위 다섯 endpoint와 현재 scene/mesh source hash가 모두 일치하는지 먼저 확인하고, link6 기준 가상 위치 `(x: 0–0.12, y: −0.08–0.08, z: 0.02–0.12 m)` 60개를 320×240 geom segmentation으로 비교합니다. 각 후보의 local optical orientation은 마지막 endpoint에서 placeholder object center `(0.05,0.25,0.27) m`를 향하도록 한 번 계산한 뒤 다섯 endpoint 전체에서 고정합니다. 결과 artifact `.control-station/mujoco-wrist-view-candidates/candidate-20261030T202133121994Z-bbe004a1.json`은 52/60 후보에서 다섯 endpoint 모두 target pixel이 있었고, 34/60은 매 endpoint의 target pixel bounding box가 image edge에서 5 px 이상 떨어졌다고 기록합니다. 상위 후보는 link6 local position `[0.04,0.04,0.07] m`, local quaternion `[0.52142,-0.16700,-0.01869,-0.83659]`, 최소 2,587 target pixel 및 최소 22 px edge margin을 보였습니다. 이는 현재 임시 물체·선반·팔 모델에 대한 가상 camera 위치/orientation 탐색입니다. camera body를 MuJoCo에서 이동해 segmentation을 관찰했을 뿐, 물리 camera 크기·시야·장착 가능성·충돌, 실제 frame 또는 손목 재인식 성능을 검증하지 않았습니다. 따라서 상위 후보를 실물 장착 위치나 제어 입력으로 사용하지 않습니다.

**초기 keyframe localization 관측:** `placeholder_fixed_camera`에서 `placeholder_pick_object_geom`의 1,764 visible pixel을 찾아 MuJoCo world point 평균 `[0.0500, 0.2500, 0.2950] m`를 출력했습니다. scene placeholder box 중심 z=0.270 m와 half-height 0.025 m로 계산되는 윗면 z=0.295 m와 일치하는 synthetic surface sample이며 물체 중심 추정은 아닙니다. baseline `placeholder_wrist_camera`에는 같은 home 자세에서 유효 depth pixel이 없습니다. `placeholder_wrist_aimed_candidate`는 home link6 위치에서 placeholder object center `[0.05,0.25,0.27] m`를 향하도록 quaternion을 계산한 비실물용 orientation candidate입니다. 저장된 capture에서 4,184 visible pixel을 얻었으나 RGB frame에는 선반 상판에 의한 물체 하단 가림이 보였습니다. 따라서 이 후보는 target detection용 synthetic view를 만들 뿐, 온전한 target localization/approach/grasp를 증명하지 않습니다. ROS 연결, 물리 camera/선반 치수, 실제 target approach, grasp 및 실물 동작은 검증하지 않았습니다.

**Wrist 시야 bounded search (2026-10-31):** 임시 scene을 open-front 형태로 바꾼 뒤 MuJoCo joint range의 약 65% 안에서 다섯 arm joint를 직접 설정한 정적 상태 1,000개를 조사했습니다. `mj_forward` 뒤 접촉 없는 상태는 737개였고, 그중 target geom이 100 pixels 이상 보인 상태는 4개였습니다. 이 4개 모두 target이 이미지 경계에 잘려 있었으며, 접촉도 없고 target 전체가 보이는 상태는 찾지 못했습니다. 탐색은 임의 표본이므로 가능한 모든 자세가 없다는 증명은 아닙니다. 관절을 순간 변경한 검색이라 접근 경로, 제어 추종, 실측 camera extrinsic 또는 grasp를 검증하지 않았습니다. wrist camera pose는 visualization-only placeholder이며, 현장 측정 전에는 실물 배치나 제어에 사용하지 않습니다.

## ROS 2 MuJoCo 제어 초안

`simulation/ros2_ws/src/omx_mujoco_sim/`은 기존 Gazebo 창고 패키지와 분리한 simulation-only `ros2_control` 패키지입니다. controller-only URDF/Xacro에는 MuJoCo joint 이름과 현재 MJCF joint range를 넣고, `mujoco_ros2_control/MujocoSystemInterface`를 MJCF scene에 연결합니다. URDF의 단순 link에는 visual/collision/inertial을 복제하지 않습니다. simulation geometry와 접촉의 유일한 기준은 MJCF입니다. launch의 노드와 controller manager는 `/mujoco_omx` namespace에 두며, 두 spawner 모두 Jazzy에서 controller별 parameters를 받도록 `controllers.yaml`을 `--param-file`로 전달합니다. 6개 position interface는 `joint_trajectory_controller`에 연결하고, `Gripper_mimic`은 URDF mimic 및 MJCF equality constraint로만 표현해 독립 command interface를 주지 않습니다. Joint state broadcaster는 `use_local_topics: true`로 `/mujoco_omx/joint_state_broadcaster/joint_states`에 게시하도록 설정해 창고 simulation의 root `/joint_states`와 이름을 분리합니다. launch는 이 sim 전용 토픽만 `robot_state_publisher`에 remap하므로 시뮬레이션 관절 상태에서 TF를 만들되 공용 `/joint_states`와 합치지 않습니다. 해당 TF frame에는 `mujoco_omx/` prefix를 붙여 공유 ROS graph의 같은 이름 frame과 구분합니다. namespace 설정은 controller manager/action/topic 이름의 겹침을 줄이지만 launch가 `ROS_DOMAIN_ID`를 바꾸지는 않으므로, 다른 simulator와 공용 `/clock` 등을 공유할 수 있는 환경에서는 별도 ROS domain을 사용합니다. 이는 launch 설정상 연결이며 runtime에서 publisher/TF 수신까지 확인한 것은 아닙니다. [Jazzy joint state broadcaster 문서](https://control.ros.org/jazzy/doc/ros2_controllers/joint_state_broadcaster/doc/userdoc.html)는 local topic 사용 시 broadcaster namespace 아래에 토픽을 둔다고 명시합니다. [robot_state_publisher 문서](https://github.com/ros/robot_state_publisher)는 `joint_states` subscription에서 TF를 만들고 `frame_prefix` 파라미터를 지원한다고 설명합니다. Effort metadata는 MuJoCo actuator force range magnitude를, velocity metadata는 pinned upstream OMX-F URDF 값을 사용하며 둘 다 physical hardware limit이나 safety setting으로 적용하지 않습니다. `initial_keyframe=home`, `headless=true`입니다.

MuJoCo control plugin은 별도 ROS 의존성입니다. ROS 2 Jazzy의 `ros-jazzy-mujoco-ros2-control` 및 controller 관련 패키지를 설치한 환경에서 다음처럼 사용하도록 구성했습니다.

```bash
sudo apt update
sudo apt install \
  ros-jazzy-control-msgs \
  ros-jazzy-mujoco-ros2-control \
  ros-jazzy-controller-manager \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-joint-trajectory-controller \
  ros-jazzy-launch \
  ros-jazzy-launch-ros \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-xacro
source /opt/ros/jazzy/setup.bash
cd simulation/ros2_ws
colcon build --packages-select omx_mujoco_sim
ROS_DOMAIN_ID=15 ./run_mujoco_sim.sh
```

`15`는 예시일 뿐입니다. 실제 ROS graph에서 실물 로봇 및 다른 simulator와 분리된 domain을 확인한 뒤 같은 domain을 관제 서버에도 설정합니다. launch 내부 node namespace만으로 `/clock` 및 모든 graph traffic이 domain 간 격리되는 것은 아닙니다.

`run_mujoco_sim.sh`는 명시적 숫자 `ROS_DOMAIN_ID` 1–232, ROS 2 Jazzy setup, workspace overlay, `omx_mujoco_sim` 및 backend/controller package index를 모두 확인한 뒤 해당 시뮬레이션 launch만 실행합니다. `--check-only`를 지정하면 같은 선행 조건을 검사하고 ROS node를 실행하지 않으며, 누락 package를 모두 보고합니다. 0(기본 domain)과 범위 밖 값은 거부합니다. 숫자 형식 검사는 실제 graph가 비어 있거나 실물 장치와 분리됐음을 증명하지 않으므로, 운영자가 domain을 확인해야 합니다. package index 조회는 plugin이 실제 로드되는지 확인하지 않습니다. action goal이나 실제 장치 명령은 이 래퍼에서 보내지 않습니다.

이 패키지는 `toktak_warehouse`나 Python 관제 서버의 런처를 변경하지 않으며, LeRobot/실물 port에 연결하지 않습니다. URDF/Xacro expansion과 MJCF 대응 계약은 확인했지만, 현재 호스트에 MuJoCo ROS plugin이 미설치라 launch·controller activation·trajectory 실행은 확인하지 않았습니다. controller의 100 Hz update 설정은 초기 simulation config이며 성능/안전 수용값이 아닙니다. 이 controller description은 MoveIt collision model, 물리 OMX safety limit, hardware adapter 또는 공통 관제 action을 제공하지 않습니다.

**호스트 의존성 재확인 (2026-10-31):** `/opt/ros/jazzy/setup.bash`를 Bash에서 source한 뒤 package lookup을 확인했습니다. `controller_manager`와 `joint_state_broadcaster`는 `/opt/ros/jazzy`에 있지만 `mujoco_ros2_control`, `ros2_control`, `joint_trajectory_controller`는 package index에서 찾지 못했습니다. 저장소의 `.venv-mujoco`에는 `mujoco`가 있으나 `rclpy`는 없습니다. 이번 turn에 Jazzy와 MuJoCo overlay를 source한 뒤 package lookup을 다시 했습니다. `control_msgs`, `controller_manager`, `joint_state_broadcaster`, `launch`, `launch_ros`, `omx_mujoco_sim`, `robot_state_publisher`, `xacro`는 조회됐으며 `joint_trajectory_controller`와 `mujoco_ros2_control`은 미설치 상태였습니다. 따라서 현재 환경에서는 MuJoCo ROS 2 launch를 시작할 실행 의존성이 갖춰지지 않았습니다. apt 저장소에는 `ros-jazzy-mujoco-ros2-control`, `ros-jazzy-joint-state-broadcaster`, `ros-jazzy-joint-trajectory-controller` 후보가 있습니다. 이 세 패키지의 설치 시뮬레이션은 추가 패키지 구성을 확인했지만, 현재 세션의 비대화형 `sudo`는 관리자 비밀번호를 요구해 설치할 수 없었습니다. 패키지는 변경되지 않았습니다. 설치 없이 `colcon build --packages-select omx_mujoco_sim`은 성공했습니다. launch 설치 단계는 `__pycache__`와 `*.pyc`를 제외하도록 하고 source/install 양쪽에 해당 산출물이 없는 것을 확인했습니다. launch·controller 실행은 하지 않았습니다.

`--path-samples`는 home 자세와 IK 자세 사이의 관절 선형 보간 segment 최소 개수이며 기본값은 21, 허용 범위는 2–501입니다. `--max-path-joint-step-rad`는 연속 이산 샘플 사이에 허용하는 최대 관절 변화량이며 기본값은 0.05 rad, 허용 범위는 0.005–0.5 rad입니다. 실제 segment 수는 이 두 조건을 만족하는 큰 쪽으로 정하고 출력에 사용한 segment 수와 최대 joint step을 표시합니다. 도구는 시작(home) 자세와 최종 자세의 접촉을 각각 확인하고, 중간 샘플 하나라도 접촉하면 exit code 5입니다. 시작 자세 접촉은 exit code 6으로 별도 거부합니다. 해상도를 높인 이산 샘플도 경로 안전 증명이 아닙니다.

`inspect_mujoco_omx.py`는 scene을 MuJoCo에서 로드하고 model metadata 및 semantic joint map을 출력합니다. 이 경로는 MuJoCo 3.17에서 `nq=14`, `nv=13`, `nu=6`으로 로드되고 6개 actuator/joint 이름을 확인했습니다. `--viewer`는 MuJoCo 시뮬레이션 장면만 엽니다. `step_mujoco_omx.py` 역시 MuJoCo model 내부 상태만 진행하며 실제 카메라/직렬 포트를 열거나 팔에 명령을 보내지 않습니다. scene 로드는 접촉·파지 동작의 검증을 뜻하지 않습니다.

`step_mujoco_omx.py`는 home keyframe에서 시작해 MuJoCo actuator의 model range 안에 있는 라디안 목표만 설정하고 지정 횟수만큼 simulation step을 진행합니다. 예시의 `Joint1=0.10`은 형식 시연용 목표이며 실제 선반 동작이나 실물 범위가 아닙니다. 반복 `--target`은 여러 관절에 지정할 수 있고 같은 관절을 중복 지정하거나 모델 범위 밖 값은 거부합니다. 출력된 최종 joint qpos는 simulation 관측값일 뿐 목표 도달 보장이나 물리 성능 측정이 아닙니다.

`shelf_pick_scene.xml`의 `placeholder_fixed_camera`는 world 기준 `pos="0.05 0.25 0.85"`, `xyaxes="1 0 0 0 1 0"`, `fovy="55"`로 정의한 임시 선반 위쪽 카메라입니다. `placeholder_wrist_camera`와 `placeholder_wrist_aimed_candidate`는 upstream에 없으며 `link6` body 아래 child로 추가한 visualization-only camera입니다. baseline 임시 body 기준값은 `pos="0.045 0 0.015"`, `xyaxes="0 -1 0 0 0 1"`, `fovy="65"`입니다. aimed candidate는 같은 local position에서 MuJoCo scalar-first quaternion `[0.6094528517, 0.6928035160, 0.2894252225, 0.2546046941]`을 사용합니다. 이 orientation은 home 때 camera world position `[0.29545,0.0075,0.22565] m`에서 scene placeholder object center `[0.05,0.25,0.27] m`를 향하도록 계산했으며 상판 가림이 남았습니다. manifest schema v2/v3는 local attachment를 기록합니다. 이 모든 값은 시뮬레이션 가시성 후보이며 실물 카메라의 위치·광축·시야각이나 ROS TF를 추정하지 않습니다. capture CLI는 모델에 지정된 camera 이름이 존재할 때 동작합니다.

`solve_mujoco_omx_position_ik.py`는 제한된 damped-least-squares IK 후보를 계산합니다. 기본 `--target-frame link6`은 입력한 MuJoCo world `(x,y,z)` 점에 `link6` body origin을 맞추며, 선택적 `--target-quat W,X,Y,Z`는 MuJoCo scalar-first 순서의 world-frame `link6` body orientation도 함께 제한합니다. 위치 수렴 기준은 `--tolerance-mm`이고 pose mode 방향 기준은 `--orientation-tolerance-deg`(기본 3°)입니다. `--orientation-scale-m`(기본 0.1 m/rad)은 위치와 회전 잔차를 결합할 때 사용하는 수치 가중 스케일이지 실측 파라미터가 아닙니다. 관절 5개로 pose 6개 제약을 풀기 때문에 방향 포함 요청은 도달 불가하거나 수치적으로 수렴하지 않을 수 있습니다. 이는 후보 계산이며 6D 성공을 보장하지 않습니다. 기본 text 출력은 유지되고, `--format json`은 결과를 schema version 2 JSON으로 stdout에 출력합니다. JSON에는 MuJoCo 버전, 시작상태(home keyframe/model default/이전 후보), 시작 후보 파일 hash, 시작 관절값, solver 설정과 사용 반복 수, 목표/도달 pose와 link6 body pose, 잔차와 허용치, 관절 후보, 위치·방향별 수렴 필드, 시작/끝/이산 경로 접촉, 경로 sampling 설정, 모드별 candidate status와 scene·모델·assets 파일의 repository-relative SHA-256 목록이 포함됩니다. SHA-256은 입력 파일의 식별 정보이며 시뮬레이션 결과나 실물 일치성을 인증하지 않습니다. `target`/`achieved`/잔차는 `pose_frame` 기준이며 `link6` frame은 body origin pose, `tcp` frame은 명시 입력 변환을 적용한 tool pose입니다. 별도 `link6_body_pose`도 결과에 포함됩니다. 결과 구조는 [JSON Schema](mujoco-ik-candidate.schema.json)에 기록했습니다. `simulation_only=true`, `usable_for_robot_control=false`, `arm_command_sent=false`로 표시하며, 성공 exit code도 오프라인 후보의 성공만 뜻합니다. 실제 TCP·ROS·카메라·그리퍼 동작 검증은 아닙니다.

오프라인 waypoint 점검은 한 후보를 다음 IK의 초기 상태로 넘겨 수행할 수 있습니다.

```bash
.venv-mujoco/bin/python scripts/solve_mujoco_omx_position_ik.py \
  --target 0.25045,0.0075,0.21065 --format json > waypoint-01.json
.venv-mujoco/bin/python scripts/solve_mujoco_omx_position_ik.py \
  --target 0.24,0.015,0.22 --start-joints-json waypoint-01.json --format json > waypoint-02.json
```

`--start-joints-json`은 중복 JSON key가 없는 schema v2 후보만 받으며 schema version의 JSON number가 정수인지 확인합니다. 입력은 최대 1 MB regular non-symlink file이어야 합니다. MuJoCo 버전, scene/model/assets SHA-256, 5개 관절 범위를 확인합니다. 이전 후보의 수렴 상태와 시작·종점·shelf/object·이산 경로 접촉 목록이 모두 깨끗해야 다음 계산을 시작합니다. 결과에는 `start_state: provided_candidate`와 입력 후보 파일 SHA-256을 기록합니다. 새 후보의 path check는 이전 후보 joint state에서 새 목표까지의 이산 관절 선형 보간을 확인합니다. 저장된 후보를 순서대로 연결하는 수치적 waypoint 검토일 뿐, 연속 충돌 검사·trajectory planning·속도/가속도 검증 또는 로봇 명령은 아닙니다. 두 연속 waypoint CLI 결과가 접촉 없는 상태와 source hash 연결을 보였고, 두 payload 모두 Draft 2020-12 JSON Schema validation을 통과했습니다.

**Open-front fixture waypoint 관측 (2026-10-31):** MuJoCo world의 입력 지점들을 `[0.25045,0.0075,0.21065]`(home link6 위치), `[0.25,0.0075,0.24]`, `[0.25,0.0075,0.27]`, `[0.05,0.15,0.32]`, `[0.05,0.20,0.32] m` 순서로 연결했습니다. 5개 schema v2 결과는 각 입력 파일 SHA-256으로 연쇄되고, 각 segment의 solver 수렴 및 기록된 시작·종점·shelf/object·이산 경로 contact 목록이 비어 있음을 확인했습니다. 이는 선반 앞쪽까지의 제한된 position-only 수치 경로입니다. 다음 점 `[0.05,0.23,0.32] m`는 IK가 수렴했지만 종점에서 link3와 shelf deck가 접촉했습니다. 실제 placeholder object 위치 `(0.05,0.25,0.27) m` 위 목표 `[0.05,0.25,0.34] m`는 이 시작 후보에서 12.9 mm 잔차로 수렴하지 않았고 shelf-deck 접촉도 관측됐습니다. 따라서 이 chain은 target object까지 도달하지 않으며, grasp·camera visibility·물리적 접근의 성공으로 해석할 수 없습니다.

`--target-frame tcp`를 지정하면 `--target`과 필수 `--target-quat`은 world 기준 TCP 목표가 됩니다. TCP 변환은 `--tcp-transform-json`으로 검토용 artifact를 제공하거나, `--tcp-offset-link6-m X,Y,Z` 및 `--tcp-quat-link6-wxyz W,X,Y,Z`를 함께 입력합니다. Artifact는 `.control-station/omx-calibrations/` 바로 아래 JSON이어야 하며, `DRAFT_ONLY`/review-required 계약, 장치 revision, 검토된 physical-parent↔MuJoCo `link6` 관계, TCP 측정 방법·불확실성을 담고 source evidence 두 파일의 SHA-256과 현재 내용이 일치해야 합니다. 스키마는 [TCP transform artifact](link6-tcp-transform.schema.json)에 있습니다. 이 검증은 파일·계약 일치 확인이지 frame mapping·물리 측정의 진실성 검증이 아닙니다. Inline 숫자 입력은 임시 operator input이라서 별도 measurement provenance가 JSON에 붙지 않습니다. 어느 입력도 현재 실물 frame 대응 미확인 상태를 해소하지 않으며 임의 값을 넣어 CLI를 실행하지 않았습니다. 기본 `--target-frame link6`은 기존 동작을 유지하며 TCP 값을 받지 않습니다. TCP 모드 JSON은 target/achieved/residual을 TCP frame 기준으로 기록합니다. 현장 측정 절차는 [측정표](field-measurement-sheet.md)의 frame·단위 정의를 따릅니다.

새 artifact 파일은 `python scripts/validate_link6_tcp_transform.py --artifact /repository/.control-station/omx-calibrations/calibration.json`으로 MuJoCo 없이 검사할 수 있습니다. 명령은 artifact 파일과 그 두 근거 source의 구조/크기/SHA-256 일치만 확인하고 JSON 요약을 출력합니다. 이 결과는 측정 방법의 적합성, frame mapping의 사실성, 정확도, 로봇 READY 또는 명령 안전을 입증하지 않습니다. 현재 실제 artifact가 없어 이 CLI를 실행하지 않았습니다.

운영자가 실제 측정과 frame 대응 검토를 끝낸 후에는 `scripts/create_link6_tcp_transform_draft.py`가 그 값과 관리 디렉터리에 저장된 두 근거 파일을 받아 덮어쓰기 불가 `DRAFT_ONLY` artifact를 만들 수 있습니다. CLI는 입력된 TCP 값을 MuJoCo `link6`에 대해 이미 표현된 transform으로 취급하며, physical-parent에서 `link6`로 변환하지 않습니다. 생성 후 위 validator로 파일/hash를 확인할 수 있습니다. 두 CLI 모두 로봇 명령을 보내지 않고, artifact의 `usable_for_robot_control`은 항상 false입니다. 현장 근거가 아직 없으므로 생성기와 validator는 실행하지 않았습니다.

home 시작 자세 접촉은 exit code 6, 최종 자세 접촉은 exit code 4, home 자세에서 후보까지의 **이산 관절 선형 보간 샘플** 중 접촉은 exit code 5로 표시합니다. 샘플 개수는 `--path-samples` 최소값과 `--max-path-joint-step-rad` 상한을 함께 적용합니다. 이 샘플 검사는 연속 swept-volume 검사나 경로 계획기가 아니며, 샘플 사이 충돌을 놓칠 수 있고 관절 선형 보간 경로가 유효한 로봇 궤적이라는 뜻도 아닙니다. `link6` body origin/orientation은 검증된 gripper TCP pose가 아니고, 물체 접촉도 파지 증거가 아닙니다. 출력 joint radians는 후보일 뿐 `step_mujoco_omx.py`에 자동 전달되거나 실행되지 않습니다. 입력 pose는 현재 simulation scene의 world 기준이고 카메라/실측/TF에서 오지 않습니다.

**2026-10-30 headless simulation 관측:** 처음에는 placeholder 선반 상판이 팔 home 자세의 mesh collision geometry와 깊게 겹쳐 `Joint1` 목표 0.10 rad가 500 step 뒤 -0.00176 rad에 머물렀습니다. scene의 shelf placeholder를 home 자세 옆으로 옮긴 후 같은 목표에서 `Joint1` 0.09839 rad, `Joint2` 목표 0.05 rad에서 0.05603 rad를 출력했습니다. 이 조정은 임시 장면 안의 충돌 간섭을 줄인 것이며 실제 선반 측정값이 아닙니다. 이 한 번의 움직임은 MuJoCo actuator-control path 확인이며, 전 관절 정확도·충돌 없는 접근·접촉 파지·실물과의 일치 검증이 아닙니다.

같은 임시 scene 물체 중심 `(0.05, 0.25, 0.27) m`를 `link6` 원점 목표로 전달했을 때 solver는 1.799 mm 위치 잔차를 냈지만, 최종 자세에서 `link3`와 placeholder shelf deck/support, 그리고 여러 gripper/link geoms와 target box 사이 접촉이 감지되어 `POSITION_CONVERGED_BUT_FINAL_POSE_HAS_UNVERIFIED_CONTACTS`와 exit code 4를 반환했습니다. 이 결과는 현재 pose 목표가 파지 가능한 TCP 목표로 정의되지 않았고 scene에서 수용 가능한 grasp 후보가 아니라는 사실을 드러냅니다. IK 수렴만으로 작업을 진행하면 안 됩니다.

**추가 headless 위치 탐색 (2026-10-30, 기존 full-width front support 장면):** 장면의 임의 world 점 `(0.25, 0, 0.20) m`, `(0.20, -0.15, 0.25) m`, `(0, 0, 0.30) m`, `(0.05, 0.05, 0.30) m`에서는 `link6` 위치 IK가 0.108–1.712 mm 잔차로 수렴했고 최종 접촉은 없었습니다. 이 점들은 선반 픽업 목표가 아닌 solver/작업공간의 몇 개 표본입니다. 선반 위 상자 중심 바로 위 `(0.05, 0.25, 0.31) m` 및 `(0.05, 0.25, 0.32) m`는 각각 1.253 mm 및 1.982 mm 위치 잔차로 수렴했지만 선반 deck와 물체에 접촉했습니다. 이력은 이후 open-front geometry로 바꾸기 전 결과입니다.

**Open-front placeholder 재검토 (2026-10-31):** full-width front support를 두 측면 모서리 지지대로 바꾸어 전면을 개방했습니다. 이는 sim 접근성 확인을 위한 장면 가정이며 현장 선반 형상을 반영하지 않습니다. 새 장면에서 `link6` 목표 `(0.05, 0.25, 0.31) m`는 1.253 mm 잔차로 수렴했지만 종점에서 deck와 물체에 접촉했고, home→candidate 관절 선형 경로는 이산 41 segment 중 10번째 sample에서 접촉했습니다. `(0.05, 0.15, 0.31) m`는 종점 접촉 없이 0.199 mm 잔차로 수렴했지만, 이는 물체 중심이 아닌 앞쪽 임의 점이며 경로 23 segment 중 8번째 sample에서 deck 접촉이 있었습니다. 따라서 열린 전면만으로 접근 경로가 해결되지는 않았고 파지 후보도 성립하지 않습니다.

**순차 offline waypoint와 위쪽 접근 재검토 (2026-10-31):** 기존에 기록한 선반 앞쪽 다섯 `link6` 위치 waypoint를 현재 scene/model/mesh source hash로 다시 계산했습니다. 각 단계는 직전 hash-guarded candidate를 시작 상태로 사용했고, 목표 잔차는 각각 0.000, 0.213, 0.420, 0.260, 1.054 mm였습니다. 다섯 후보 모두 시작/끝 자세와 41-segment 관절 선형 보간 표본에서 접촉이 없었습니다. 이는 waypoint-chain 실행이나 연속 경로 검증이 아니라 정적 후보 계산입니다. 다섯 번째 후보 `(0.05, 0.20, 0.32) m`에서 대상 위쪽 후보 `(0.05, 0.25, 0.36)`, `(0.05, 0.25, 0.40)`, `(0.05, 0.25, 0.44)`, `(0.05, 0.25, 0.48) m`를 계산했을 때 위치 잔차는 각각 27.00, 56.50, 87.78, 120.46 mm였고 모든 종점 및 이산 경로에서 `link3`와 `placeholder_shelf_deck` 접촉이 검출됐습니다. 다섯 무접촉 waypoint를 서로 다른 시작 후보로 삼아 `(0.05, 0.25, 0.34) m`를 다시 계산해도 모두 거의 같은 종점 `(0.0478, 0.2408–0.2409, 0.3311–0.3313) m` 부근으로 모였고, 잔차 12.83–12.88 mm 및 link3/deck 접촉으로 실패했습니다. 이 다섯 시작점 탐색은 이 solver 설정에서의 수렴 결과이며, 목표의 전역적인 도달 불가능성 증명은 아닙니다. 추가로 다섯 번째 waypoint의 현재 `link6` 방향을 유지하는 pose IK로 `(0.05, 0.23, 0.34) m`를 계산했을 때 1.983 mm 및 1.793° 오차로 수렴했지만, 동일한 `link3`/deck 접촉이 종점과 경로 sample에서 검출됐습니다. 이 방향은 임시 waypoint에서 복사한 실험값이지 파지 방향이 아닙니다. 위치-only IK는 물체 접근에 필요한 TCP/그리퍼 방향을 지정하지 않으므로, 이 결과만으로 목표의 기구학적 불가능성을 단정할 수 없습니다. 현 placeholder scene에서 안전한 접근을 주장하지 않으며, 다음 검토는 측정된 shelf geometry와 TCP transform을 확보한 뒤 그리퍼 접근 방향을 포함한 pose-IK 후보를 비교하는 것입니다. 후보 JSON은 ignored `.control-station/mujoco-ik-candidates/` 아래에 저장되며 모두 `simulation_only: true`, `usable_for_robot_control: false`, `arm_command_sent: false`입니다.

**단계별 상부 접근 후보 탐색 (2026-10-31):** 기존 무접촉 chain의 세 번째 후보에서 `link6` 원점 `(0.05, 0.10, 0.40) m`로 이동한 뒤, z=0.40 m에서 y=0.11, 0.12, 0.13, 0.14, 0.145 m 순으로, 이어서 `(0.05, 0.145, 0.35)`와 `(0.05, 0.145, 0.32) m`, 마지막으로 z=0.32 m에서 y=0.16, 0.18, 0.20, 0.21 m 순으로 hash-linked 시작 후보를 계산했습니다. 각 단계는 1.552 mm 이하 위치 잔차로 수렴했고 81 또는 101 segment 이산 관절 보간 sample에서 shelf/object 접촉이 없었습니다. `(0.05, 0.215, 0.32)` 및 `(0.05, 0.22, 0.32) m`는 각각 0.718/0.698 mm 위치 잔차로 수렴했으나 link3와 placeholder deck 접촉이 최종 자세 및 경로 sample에서 검출됐습니다. 이 결과는 현재 임시 장면과 `link6` body-origin 목표에 한정되며, 실제 TCP·그리퍼 방향·선반 치수, 연속 swept-volume, 동역학/제어 추종, 물체 접촉 파지나 실물 도달 가능성을 검증하지 않습니다. 무접촉 후보 chain의 최대 y=0.21 m를 grasp pre-approach로 승인하지 않습니다.

## 이 단계의 모델 범위

`robotis_omx/shelf_pick_scene.xml`은 공식 OMX 모델에 바닥, 앞면이 열린 선반 상판/모서리 지지대와 한 변 5cm인 상자형 물체를 추가합니다. 열린 전면은 접근성 확인용 simulation assumption이며 선반과 물체의 위치·크기·질량은 placeholder입니다. 현재 설치된 선반이나 특정 물체를 측정한 값이 아닙니다.

| 관제 action key | 공식 MJCF joint/actuator | 현재 처리 |
|---|---|---|
| `shoulder_pan.pos` | `Joint1` | semantic map만 기록; `step_mujoco_omx.py`에서 직접 simulation radian 목표 입력 가능; action 변환 없음 |
| `shoulder_lift.pos` | `Joint2` | semantic map 및 직접 simulation radian 목표 입력; action 변환 없음 |
| `elbow_flex.pos` | `Joint3` | semantic map 및 직접 simulation radian 목표 입력; action 변환 없음 |
| `wrist_flex.pos` | `Joint4` | semantic map 및 직접 simulation radian 목표 입력; action 변환 없음 |
| `wrist_roll.pos` | `Joint5` | semantic map 및 직접 simulation radian 목표 입력; action 변환 없음 |
| `gripper.pos` | `Gripper` | 직접 simulation radian 목표 입력 가능; opening width 변환 없음; mimic finger는 upstream equality constraint 사용 |

**그리퍼 collision mesh 관측 (2026-10-31):** `MUJOCO_GL=egl .venv-mujoco/bin/python scripts/inspect_mujoco_omx.py --gripper-mesh-distance`는 `link6`의 `follower_07_gripper_motorized`와 `link7`의 `follower_08_gripper_gear` collision mesh 사이 signed distance를 `Gripper=0°,25°,50°,75°,100°`, `Gripper_mimic=-Gripper`의 정적 `mj_forward` 상태에서 읽습니다. MuJoCo 3.17.0에서 다섯 상태 모두 최소 거리는 약 `−1.8 mm`였고 contact count는 0이었습니다. 모델은 두 body 사이 contact를 명시적으로 제외하므로 이 수치는 접촉 없이 geometry가 겹친 compiled mesh 결과이며, jaw 사이 개방폭이나 물체를 잡을 수 있는 크기가 아닙니다. 이 장면에는 그리퍼가 잡은 물체가 없고 simulation stepping도 하지 않았습니다. 따라서 gripper angle→opening width 변환과 graspability는 계속 미확정입니다.

**placeholder 크기별 정적 접촉 조사 (2026-10-31):** `--placeholder-object-half-size-m X,Y,Z`는 `MjSpec` 단계에서 box 크기를 바꾸고 model을 재컴파일합니다. 컴파일된 `MjModel.geom_size`를 직접 키우면 broadphase 경계가 오래된 상태로 남아 충돌이 누락될 수 있어 이 방법은 사용하지 않습니다. 크기 override 시 x/y와 물체 방향은 home keyframe을 유지하고, 상자 바닥이 placeholder 선반 상판을 0.5 mm 관통하도록 z를 옮겨 `mj_forward`가 정적 지지 접촉을 만들게 합니다. 이 0.5 mm는 정적 충돌 관측을 위한 simulation setup이며 동적 안착이나 실제 측정값이 아닙니다. 예상된 물체-선반 상판 접촉은 grasp 충돌 집계에서 분리하고, 로봇/그리퍼와 상자 접촉 및 로봇-환경 접촉은 계속 검사합니다.

동일한 접촉 인접 중심 `Joint1–5=[1.258400,-0.139798,-0.306336,-0.032794,0.210805] rad`, `Gripper=0.044055 rad` 주변에서 각 크기별 50,000개 Gaussian 정적 표본(seed `20261103`, arm/gripper sigma 각각 `0.04 rad`)을 비교했습니다. 세 경우 모두 예상된 shelf support 접촉은 50,000개 표본에 나타났으며 jaw 법선 내적 기준 `≤ −0.5`를 만족한 접촉은 없었습니다.

| 정육면체 한 변 | half-size XYZ | 비지지 target 접촉 표본 | 다른 충돌 없는 양 jaw 접촉 | 반대 법선 접촉 | 최선 jaw-only 표본 |
|---:|---:|---:|---:|---:|---|
| 30 mm | `[0.015, 0.015, 0.015] m` | 752 | 0 | 0 | 없음 |
| 50 mm | `[0.025, 0.025, 0.025] m` | 13,557 | 12 | 0 | 0.249 mm penetration, normal dot `1.0` |
| 70 mm | `[0.035, 0.035, 0.035] m` | 41,004 | 5,677 | 0 | 0.048 mm penetration, normal dot `0.997` |

표본 수는 같은 local center에서 비교한 정적 geometry 탐색이며 작은/큰 상자는 다른 실제 생활용품의 모형으로 검증되지 않았습니다. 접촉면은 여전히 양 jaw에서 거의 같은 방향이므로 표에 있는 jaw-only 수치도 pinch나 grasp 후보 승인으로 볼 수 없습니다.

**정적 gripper/object 접촉 후보 탐색 (2026-10-31):** `scripts/inspect_mujoco_omx.py --sample-static-gripper-contacts 50000 --random-seed 20261031`는 다섯 arm hinge와 `Gripper`를 MJCF range 안에서 독립 uniform 표본화하고 `Gripper_mimic=-Gripper`로 설정한 뒤 `mj_forward` collision만 셉니다. placeholder object의 freejoint는 home keyframe 위치에 고정되고 simulation step/actuator target은 없습니다. MuJoCo 3.17.0에서 50,000개 중 475개가 어떤 target contact, 136개가 `link6`·`link7` 양쪽 target contact였으며, 딱 1개가 target contact body가 두 jaw mesh뿐이고 다른 contact가 없었습니다. 그 표본은 최대 7.71 mm penetration이었고 jaw별 평균 target-to-jaw 접촉 법선 내적은 `−0.0094`로 반대 방향 pinch 기준(`dot ≤ −0.5`)을 만족하지 않았습니다. 이어 inspector에 추가한 Gaussian local sampling mode로 표본 중심을 `Joint1=1.2155972478112027`, `Joint2=-0.14200895171303785`, `Joint3=-0.2935959342671771`, `Joint4=0.005062910514284258`, `Joint5=0.16649072476521148`, `Gripper=0.07545302679508643 rad`에 두고 관절 표준편차 `0.025 rad`, 그리퍼 표준편차 `0.02 rad`, seed `20261101`로 300,000개를 실행했습니다. jaw-only 정적 접촉은 29,411개였으나 반대 방향 법선 기준을 만족한 표본은 0개였습니다. 가장 얕은 후보는 penetration 약 `0.028 mm`, jaw 평균 법선 내적 `1.0`으로 양쪽 jaw 접촉점이 거의 같은 방향이었습니다. 재현 명령은 다음과 같습니다.

```bash
MUJOCO_GL=egl .venv-mujoco/bin/python scripts/inspect_mujoco_omx.py \
  --sample-static-gripper-contacts 300000 --random-seed 20261101 \
  --local-center-joints 'Joint1=1.2155972478112027,Joint2=-0.14200895171303785,Joint3=-0.2935959342671771,Joint4=0.005062910514284258,Joint5=0.16649072476521148' \
  --local-center-gripper-rad 0.07545302679508643 \
  --local-joint-sigma-rad 0.025 --local-gripper-sigma-rad 0.02
```

표본은 정적 `mj_forward`이며 물체가 home keyframe freejoint 위치에 고정됩니다. 이 법선 내적 기준(`dot ≤ −0.5`)은 geometry 선별 휴리스틱이지 grasp 판정이 아니며, 접근 가능성, 힘/마찰 유지, lift/place를 검증하지 않습니다. 표본 탐색은 exhaustive가 아니므로 다른 자세의 가능성을 배제하지 않습니다.

추가로 `staged-y14-z40.json`의 arm 관절 후보 주변(관절 sigma `0.18 rad`, `Gripper=50°` 중심, gripper sigma `0.5 rad`)에서 seed `20261102`로 200,000개를 조사했습니다. target 접촉은 4,248개, 양쪽 jaw target 접촉은 1,428개였지만 다른 contact가 없는 jaw-only 상태는 0개였습니다. 넓은 이 탐색은 self/shelf 충돌과 대상 접촉을 분리해 계획해야 함을 보여주는 정적 표본이며, 경로 검사나 grasp 성공 근거가 아닙니다.

**반대면 접촉과 접근 경로 후보 (2026-10-31):** 50 mm cube를 shelf-support 위에 둔 전역 uniform qpos 1,000,000개(seed `20261104`)에서 opposing-normal 표본 4개를 찾았지만 모두 약 6–13 mm penetration이었습니다. 가장 가까운 후보 주변에서 Gaussian 표본을 좁히자 jaw-only 정적 접촉과 반대 법선 표본이 다수 나왔고, 마지막 seed `20261106`, joint sigma `0.01 rad`, gripper sigma `0.01 rad`인 300,000개 탐색의 가장 얕은 반대면 상태는 `link6`/`link7` 접촉 두 개, penetration 최대 `0.1093 mm`, 평균 법선 내적 `−0.5067`이었습니다. 해당 posture는 `Joint1=1.3257823`, `Joint2=0.0678345`, `Joint3=−1.0237959`, `Joint4=1.5539147`, `Joint5=−0.6059261`, `Gripper=0.4896546 rad`입니다. 이는 정적 geometry pinch 후보일 뿐 힘 유지/lift를 증명하지 않습니다.

정적 sampler의 상위 jaw-only/opposing candidate 결과에는 이제 placeholder object/world, link6/world, link6/object body-origin poses 및 jaw별 평균 contact point가 포함됩니다. 이는 object-relative 자세 후보의 재현을 돕지만, link6은 검증된 TCP가 아니며 정적 collision geometry만으로 grasp를 승인하지 않습니다.

```bash
MUJOCO_GL=egl .venv-mujoco/bin/python scripts/inspect_mujoco_omx.py \
  --sample-static-gripper-contacts 300000 --random-seed 20261106 \
  --placeholder-object-half-size-m 0.025,0.025,0.025 \
  --local-center-joints 'Joint1=1.323994580440341,Joint2=0.09177251132968504,Joint3=-1.0385318599007396,Joint4=1.540809275163728,Joint5=-0.5964115728116679' \
  --local-center-gripper-rad 0.48989869381091644 \
  --local-joint-sigma-rad 0.01 --local-gripper-sigma-rad 0.01
```

동시 관절 보간 home→위 posture는 101점 검사에서 placeholder 전면 우측 지지대에 먼저 닿았고, target 접촉도 목표 전부터 시작했습니다. 다섯 관절의 순서를 모두 열거한 120개 one-joint-at-a-time 경로 중 9개가 30 segment/관절 표본에서는 다른 접촉 없이 끝났습니다. 이 9개를 100 segment/관절로 확인하면 모두 다른 환경/자기 충돌은 없었지만 target contact가 endpoint보다 앞서 시작했습니다. 그중 `Joint3, Joint1, Joint4, Joint5, Joint2` 순서를 1,000 segment/관절로 더 촘촘히 조사하자 target 접촉은 마지막 관절 이동의 약 99.7% 지점부터 나타났습니다. 이 순서의 5,001개 qpos 표본에서 shelf support 접촉 20,004건 외 환경/자기 충돌은 0 sample, target penetration 최대 `0.1093 mm`였습니다. 이 검사에서도 gripper angle을 고정했고, 물체가 잡혀 이동하거나 실제 trajectory controller가 실행된 것은 아닙니다. joint interpolation도 연속 충돌 보장이 아닙니다.

경로 조사는 재실행할 수 있습니다.

```bash
MUJOCO_GL=egl .venv-mujoco/bin/python scripts/inspect_mujoco_omx.py \
  --placeholder-object-half-size-m 0.025,0.025,0.025 --audit-home-path \
  --path-goal-joints 'Joint1=1.3257822951314233,Joint2=0.06783448034090353,Joint3=-1.0237959140043242,Joint4=1.553914708414148,Joint5=-0.6059260770777783' \
  --path-gripper-rad 0.48965459480059115 --path-segments 1000 \
  --path-joint-order Joint3,Joint1,Joint4,Joint5,Joint2
```

**동역학 접촉 및 lift 실패 기준선 (2026-10-31):** `scripts/audit_mujoco_pick_dynamics.py`를 추가해 관절 목표를 명시적으로 받은 MuJoCo 한정 접근→그리퍼 닫기→link6 위치 IK lift 실험을 재현 가능하게 했다. 기존 정적 pinch 후보를 one-joint-at-a-time 순서로 접근하고 그리퍼를 30°에서 `0.48965 rad`로 닫았을 때 두 jaw body가 placeholder box에 각각 접촉했다. 하지만 `link6` 원점을 30 mm 위로 옮기는 위치 IK lift 동안 상자는 초기 lift 자세보다 최대 약 5.80 mm 상승했고, 최대 상판 이격은 약 5.56 mm였다. 두 jaw 동시 접촉은 1,500 lift step 중 322 step에서 관찰됐다. 정착 뒤 최종 상승은 약 0.13 mm, 상자 바닥의 최종 선반 이격은 약 −0.11 mm이고 두 jaw 접촉은 없어 상자는 선반 지지 접촉으로 돌아왔다. 따라서 이 상태는 동적 grasp/lift 성공이 아니다. 위치 IK가 gripper orientation을 보존하지 않고, close target은 MuJoCo actuator target일 뿐 jaw force나 실물 aperture 환산이 아니다. CLI는 모델/scene placeholder에 한해 동작하고 JSON에 `simulation_only`, `usable_for_robot_control: false`, `arm_command_sent: false`, `candidate_verified_grasp: false`를 항상 기록한다. 결과에는 MuJoCo version, timestep, scene/MJCF/mesh source file의 SHA-256·크기, geometry와 step count, 각 `mj_step` 뒤 관찰한 단계별 support/jaw/기타 접촉 수와 pair 목록을 포함해 재생성 및 접촉 관측을 추적한다. 기본은 stdout JSON이며 `--save-artifact`를 지정하면 `audit_id`, UTC 생성 시각 및 `artifact_path`가 포함된 동일 JSON을 `.control-station/mujoco-dynamics-audits/`에 임시 파일+exclusive link로 덮어쓰기 없이 저장한다. 이 접촉 기록은 표본 관찰이며 연속 충돌 검사가 아니다. 이 실험의 MuJoCo actuator dynamics와 단순 위치 IK도 실물 관절 응답이나 안전한 궤적을 뜻하지 않는다.

같은 접근·닫기 조건에서 lift goal을 5/10/15 mm로 바꿔 추가로 확인했다. 각 1,500 lift step에서 두 jaw 동시 접촉은 각각 977/571/459 step이었고, 2초 settle 뒤 최종 상판 이격은 약 0.33/2.34/3.02 mm였다. 세 경우 모두 종료 시 양쪽 jaw 동시 접촉은 없고 link6 한쪽 body와 접촉이 남았다. 따라서 양의 최종 이격도 이 장면에서 안정적인 grasp를 증명하지 않으며, 현재 후보는 5–30 mm 모든 lift 목표에서 grasp 검증을 통과하지 못했다.

후속 비교를 위해 audit CLI에 선택형 weighted pose-lift IK도 추가했다. 기본 `--lift-orientation-scale-m-per-rad 0`은 기존 position-only IK를 사용한다. 양수로 설정하면 close 직후 MuJoCo link6 orientation을 목표로 캡처하고 위치·방향 오차를 가중 least-squares로 함께 줄인다. 이는 현재 link6 자세를 유지하려는 시뮬레이션 제약이며 실물 TCP orientation이나 유효한 grasp 방향이 아니다. 또한 `--candidate-json`은 기존 IK CLI의 schema v2 후보 중 현재 scene/MJCF/mesh hash와 MuJoCo version이 일치하고 수렴·접촉 없는 시작/종점/이산 경로를 기록한 것만 받아 joint 목표를 가져온다. input JSON SHA-256과 상태를 audit 결과에 남긴다. dynamics audit schema v2는 candidate의 요청/달성 link6 위치와 placeholder object center를 기록한다. schema v3는 그 두 점 간 Euclidean 거리를 함께 기록하되 TCP offset/clearance로 해석하지 않도록 범위를 명시한다. schema v4는 매 표본의 object-jaw 접촉 법선을 MuJoCo geom 순서에 맞춰 정렬하고 단계별 bilateral normal dot 요약, `dot ≤ -0.5` 표본 수, 최대 jaw penetration을 추가한다. contact normal은 MuJoCo frame의 첫 축이며 geom1에서 geom2 방향이다([MuJoCo API reference](https://mujoco.readthedocs.io/en/latest/APIreference/APItypes.html)). schema v5는 close-settle 시점의 placeholder-object/world, link6/world, link6/object body-origin pose를 기록한다. schema v6는 `mj_contactForce`가 반환한 contact-frame force/torque 중 normal 및 tangential force magnitude를 jaw별로 contact observation 평균과 최대값으로 기록한다([MuJoCo function reference](https://mujoco.readthedocs.io/en/3.17.0/APIreference/APIfunctions.html)). schema v7은 close-settle에서 arm actuator target과 실현 arm qpos, 접근 상태 대비 관절 이동량, gripper actuator target과 실현 주/mimic gripper qpos를 함께 기록한다. schema v8은 close-settle 종료 순간 물체 접촉별 상대 body/geom, signed distance, world contact point, 물체에서 상대 body로 향하는 단위 normal을 보존한다. 이 값은 현재 MJCF/solver의 simulation result이지 실물 관절 상태, 접촉 센서 측정, TCP pose 또는 파지 성공 판정이 아니다. verifier는 audit schema v1–v8을 읽는다.

**Weighted orientation lift sweep (2026-10-31):** 같은 placeholder joint/gripper 목표, 1,500 lift step 및 settle 조건에서 `--lift-orientation-scale-m-per-rad`를 `0.005`, `0.02`, `0.05`, `0.1`, `0.25`, `0.5`로 바꿔 stdout-only MuJoCo audit을 실행했다. 양 jaw 접촉은 각각 328, 340, 394, 487, 493, 418 표본에서만 나타났고 모든 경우 정착 후 두 jaw 접촉은 없었다. 0.5에서도 상자 최종 상승은 약 0.13 mm, 최종 shelf-bottom clearance는 약 −0.108 mm였다. 가중치가 커질수록 orientation residual은 감소했지만 position residual은 약 18.7–24.6 mm로 증가했다(0.005에서는 각각 약 0.423 rad/0.185 mm, 0.25에서는 0.021 rad/23.7 mm). 이 조절 하나로 grasp가 유지되지 않았으며 위치·자세 IK residual만으로 파지 가능성을 판단할 수 없다는 실패 근거다. sweep 결과는 stdout에서 읽었고 별도 audit artifact는 생성하지 않았다.

**Object-relative 정적 후보 동역학 재평가 (2026-10-31):** 100,000개 local Gaussian qpos 표본에서 best opposing-normal posture는 link6 body origin을 placeholder object frame에서 `[-0.02129, −0.06529, 0.04387] m`에 두고 평균 법선 dot `−0.50677`, 최대 관통 `0.305 mm`를 가졌습니다. 이것은 정적 후보를 설명하는 body-origin 관계일 뿐입니다. 같은 joint/gripper 목표를 actuator 순차 접근과 닫기로 재평가한 audit `.control-station/mujoco-dynamics-audits/audit-20261030T211350716213Z-3a6a120e.json`에서는 close-settle bilateral dot 평균이 `−0.49800`, `dot ≤ −0.5` 표본 0개였고 lift 1,500 표본 중 bilateral contact 334개, opposing-normal 표본 0개였습니다. 정착 후 양 jaw 접촉은 없고 box의 최종 선반 이격은 약 `−0.108 mm`였습니다. verifier가 artifact와 현재 model source 10개 hash 일치를 확인했습니다. 이 차이는 정적 qpos 접촉이 동적 actuator 상태에서 유지된다는 뜻이 아님을 보여줍니다.

**정적 목표와 동역학 실현 상태 비교 (2026-10-31):** 같은 후보의 schema v7 audit `.control-station/mujoco-dynamics-audits/audit-20261030T211524235326Z-5e51b79d.json`은 접근 시 실제 arm qpos와 목표 차이, close-settle 시점 arm qpos, gripper command target 및 실제 주/mimic joint 위치를 기록합니다. 접근 시 가장 큰 arm 목표 오차는 Joint2의 `4.34 mrad`였고, 그리퍼 닫힘 동안 가장 큰 arm 관절 이동량은 Joint1의 `−0.91 mrad`였습니다. gripper target `0.49188 rad`에 대해 close-settle 주 joint는 `0.49908 rad`, mimic joint는 `−0.49924 rad`였습니다. close-settle 상태에는 두 jaw 접촉과 shelf/world 접촉이 관측됐으나, 이를 stable grasp로 보지 않습니다. 이 기록은 목표값을 정적 mj_forward 후보에서 그대로 재현했다고 가정하는 대신, 실제 simulation qpos와 actuator 추종 오차를 조사하기 위한 것입니다. 독립 verifier는 schema v7 구조와 현재 scene/model/mesh source 10개 일치를 확인했습니다.

이후 같은 목표의 close-settle qpos, 실제 gripper 주/mimic 위치, object world pose를 `mj_forward`에 다시 적용해 접촉 기하를 재구성했습니다. jaw 접촉은 link6 `−0.510 mm`, link7 `−0.607 mm` signed distance로 재현됐고, object→jaw 단위 법선 내적은 `−0.49934`였습니다(동적 settle 표본 평균 `−0.49800`). schema v8 artifact `.control-station/mujoco-dynamics-audits/audit-20261030T211755524635Z-9187b2e9.json`은 close 종료 순간 6개 접촉을 보존합니다: placeholder shelf deck 4개와 각 jaw 1개. 각 접촉점, geometry ID/name, signed distance 및 방향 법선은 정적 재생과 대조할 수 있습니다. 독립 verifier는 source 10개 hash 일치 및 `AUDIT_VALID_CURRENT_MODEL`을 확인했습니다. 이 결과는 물리적 접촉 확인이나 lift 성공을 뜻하지 않으며 같은 시뮬레이션 상태 재구성 결과입니다.

**국소 joint 목표 민감도 탐색 (2026-10-31):** 같은 static candidate 주변에서 Joint2/3/4 목표를 각각 `−0.05, 0, +0.05 rad` 바꾸는 27개 조합과, `(+0.05,+0.05,+0.05) rad` 주변 `−0.025, 0, +0.025 rad`씩의 27개 조합을 동일한 접근/닫기/30 mm lift/settle 설정으로 비교했습니다. 두 격자에는 동일한 중앙 조합이 겹칩니다. 닫힘 중 두 jaw 접촉은 54회 모두에서 조건에 따라 변했고, 일부에서는 1,000 close-settle 표본 전부가 `dot ≤ −0.5`였지만, 단 한 후보도 settle 뒤 두 jaw 접촉을 유지하지 못했습니다. 선택한 `(+0.05,+0.075,+0.075) rad` offset은 424/1,500 lift 표본에서 양 jaw 접촉, 최대 shelf-bottom clearance 5.35 mm, 최종 clearance 3.71 mm 및 높이 변화 +4.03 mm를 보였습니다. 그러나 최종 접촉은 link6 한쪽과 shelf뿐이었고 link7 jaw 접촉은 없었습니다. 따라서 이 후보도 grasp 실패입니다. 상세 v8 audit `.control-station/mujoco-dynamics-audits/audit-20261030T211911836700Z-6351a4a9.json`은 settle 중 link6 접촉 992/1,000 표본과 최대 normal force 0.196 N을 기록합니다. 이는 팔 한쪽 geometry가 물체를 밀거나 받친 상태이며, 상승 높이만으로 파지 성공을 판정할 수 없음을 보여주는 시뮬레이션 결과입니다. 이 작은 격자는 전역 최적화나 가능한 grasp 부재의 증명이 아닙니다.

Joint5 yaw offset까지 포함해 Joint2/3/4 offset `{0, 0.05, 0.10} rad`와 Joint5 offset `{−0.05, 0, +0.05} rad`의 81개 조합도 같은 조건에서 비교했습니다. 25개 조합은 닫힘 구간의 normal-dot 휴리스틱을 통과했지만, 어느 조합도 settle 뒤 양 jaw 접촉이 없었습니다. 최댓값 후보는 878/1,500 lift 표본에서 양 jaw가 접촉했고 shelf-bottom clearance가 최대 10.37 mm였지만, 최종 object contact는 shelf만 남았고 최종 높이 변화는 +0.236 mm였습니다. v8 artifact `.control-station/mujoco-dynamics-audits/audit-20261030T212058643269Z-7e7526af.json`의 source 10개 hash는 현재 model과 일치합니다. 탐색은 이 제한된 q-target grid에만 해당하며, 전체 grasp 공간을 조사한 것이 아닙니다.

**선반 접촉을 제거한 close-pose 진단 (2026-10-31):** `scripts/audit_mujoco_gripper_without_shelf.py <managed-audit.json>`은 먼저 입력 audit의 schema/safety/source hash를 독립 verifier로 검사하고, 해당 기록에 들어 있는 close-settle arm qpos와 open/close target을 사용합니다. 진단용 model copy에서 `placeholder_shelf_*` geom collision만 끄고 gravity와 floor는 유지합니다. object는 audit의 initial placeholder center 및 장면 keyframe identity orientation으로 놓이며, 팔은 close pose에 정지시킵니다. 실행 예시는 다음과 같습니다.

```bash
.venv-mujoco/bin/python scripts/audit_mujoco_gripper_without_shelf.py \
  .control-station/mujoco-dynamics-audits/audit-20261030T212058643269Z-7e7526af.json
```

선택한 audit 기준 양 jaw 동시 접촉은 close 57/1,000 표본, settle 0/1,000 표본이었습니다. object center z는 초기 `0.25950 m`에서 final `0.02489 m`로 내려 floor geom과 접촉했습니다. 이 counterfactual은 해당 close posture가 선반 충돌 없이도 중력 아래에서 물체를 지지하지 못했다는 점을 보여줍니다. shelf 제거와 object placement는 진단 조건이므로 물리 환경이나 일반적인 그리퍼 실패 원인으로 확대 해석하지 않습니다. 결과 JSON은 stdout 전용이며 source audit SHA와 model inventory를 포함하고, `simulation_only: true`, `usable_for_robot_control: false`, `arm_command_sent: false`를 출력합니다.

**접근 posture와 grasp target 구분 (2026-10-31):** 현재 scene 안의 `staged-*.json` 중 수렴·경로 표본에서 무접촉으로 표시된 12개를 동일한 동역학 audit에 입력했다. 이 artifact들의 목표는 link6 body origin 기준 대략 shelf 앞쪽 staging pose이며 placeholder box 중심 `[0.05, 0.25, 0.27] m`의 grasp pose가 아니다. 12개 모두 close 후 box-jaw 접촉이 없고 lift 1,500 표본 중 두 jaw 접촉은 0이었다. box는 shelf 위에 그대로 남았다. schema v3 audit `.control-station/mujoco-dynamics-audits/audit-20261030T210540622193Z-1193e4ca.json`은 candidate target `[0.05, 0.145, 0.4] m`, 실제 초기 box center `[0.05, 0.25, 0.2595] m`, 두 위치 간 175.4 mm Euclidean 거리를 기록했다. 이 거리는 link6 body origin과 box center 사이이며 TCP offset 또는 grasp clearance가 아니다. 독립 verifier에서 현재 model source 10개와 일치했고 close 후 양 jaw 접촉은 없었다. 이 결과는 접근 후보를 grasp 후보로 오인하지 않도록 구분하며, 해당 posture 집합에 대한 실패만 말한다. 다음 simulation grasp 후보는 명시적 object-relative pregrasp/grasp pose, gripper clearance 및 contact 조건을 가진 별도 산출물로 만들어야 한다.

schema v4 재현에서는 close-settle 1,000 표본 중 양 jaw 접촉이 유지됐지만 bilateral contact-normal dot 평균은 `−0.4987`, 정적 휴리스틱 `dot ≤ −0.5`를 만족한 표본은 2개였다. lift에서는 양 jaw 접촉 322/1,500 표본, opposing-normal 표본 22개였고 settle 뒤 접촉이 사라졌다. 상세 기록은 `.control-station/mujoco-dynamics-audits/audit-20261030T210652459628Z-1feba95c.json`에 있다. 정상 방향 진단은 기하 접촉 방향의 증거이지 힘 유지나 grasp 성공 판정이 아니다.

schema v5 audit `.control-station/mujoco-dynamics-audits/audit-20261030T210917192341Z-47dcf45d.json`은 close-settle 이후 placeholder object frame에서 `link6` body origin을 `[-0.02045, -0.06529, 0.04276] m`, orientation `[0.49051, -0.37495, 0.14570, 0.77304]` wxyz로 기록했다. 별도 verifier는 저장된 world poses에서 이 relative body transform을 재구성해 일치 여부를 확인했다. 이는 시뮬레이션 close 상태 재현 자료일 뿐, 실제 TCP offset 또는 물체를 들어 올려 고정한 grasp transform이 아니다.

schema v6 baseline `.control-station/mujoco-dynamics-audits/audit-20261030T211051196563Z-1f8ad8f3.json`은 close-settle 중 jaw 접촉 관측의 평균 normal force를 link6 `1.46 N`, link7 `1.98 N`으로 기록했다. lift 표본에서의 평균은 해당 jaw 접촉이 관측된 contact record에 한해 link6 `0.57 N`, link7 `0.97 N`이었다. 이는 `mj_contactForce` simulation 표본 평균이며 시간 평균이나 독립된 sensor 측정이 아니다. MuJoCo 공식 API는 force/torque를 contact frame에서 반환한다([MuJoCo API reference](https://mujoco.readthedocs.io/en/3.17.0/APIreference/APIfunctions.html)). 실물 그리퍼 힘, 모델 정확도, stable grasp 또는 허용 임계값을 뜻하지 않는다.

같은 scene의 MuJoCo model metadata에서 placeholder object body mass `0.08 kg`, gravity `[0, 0, −9.81] m/s²`를 읽었으며, 모델상의 무게는 `0.7848 N`이다. object/shelf friction parameter는 각각 `[1.2, 0.02, 0.001]` 및 `[1.0, 0.01, 0.001]`이다. 이 계수들은 MJCF 시뮬레이션 설정이다. jaw 접촉 normal force의 크기 합만으로 수직 지지력이나 파지 안정성을 판단할 수 없고 실제 물체·선반 마찰값에도 대응하지 않는다.

`verify_mujoco_pick_dynamics_audit.py <audit.json>`은 저장된 managed audit만 읽는다. schema v1–v8, 중복 JSON key, 크기/경로/symlink, audit safety flags, 단계별 contact count 부등식, close/lift 관측 요약, IK 수치 구조 및 source file descriptor를 확인하고 audit JSON SHA-256을 출력한다. v2 이후 candidate target provenance와 초기 placeholder object center를 확인하며, v3에서는 기록된 Euclidean 거리의 재계산 일치를, v4에서는 법선 표본 수·dot 범위·관통량 요약을 검사한다. v5에서는 두 world body pose에서 상대 link6/object body pose를 재계산해 저장값과 비교하고, v6에서는 jaw별 force observation count와 force summary를 검사하며, v7에서는 arm target/실제 위치/접근 대비 이동량과 gripper target/실제 주·mimic joint의 수치 일관성, v8에서는 종료 순간 contact detail 수, body별 count, 유한한 접점/거리와 단위 법선을 확인한다. 현재 checkout의 model source inventory와 source hash가 다르면 JSON을 valid로 표시하되 `AUDIT_VALID_BUT_SOURCE_FILES_DIFFER_FROM_CURRENT_MODEL`을 보고한다. 이 명령은 MuJoCo·ROS·카메라·로봇을 실행하지 않으며 audit 파일을 수정하지 않는다. 저장하지 않은 stdout audit은 managed directory 규칙 때문에 이 verifier의 입력이 아니다.

**저장 artifact 재현 확인 (2026-10-31):** 위 명령을 MuJoCo 3.17.0에서 재실행해 `.control-station/mujoco-dynamics-audits/audit-20261030T210104793553Z-ee4f2b89.json`을 저장했다. 독립 verifier는 현재 모델 source 10개와의 일치 및 `AUDIT_VALID_CURRENT_MODEL`을 보고했고 audit bytes SHA-256은 `4be6a418982085520550b9dc3994248111c8ced0a2ed2cfde64c54ed91fdef24`였다. 관찰된 실패 기준선은 접근/닫기 때 placeholder box의 shelf 및 두 jaw 접촉, lift 동안 양 jaw 접촉 322/1,500 표본, settle 후 grasp 미검증이다. 해당 결과는 simulation-only이며 실물 명령은 보내지 않았다.

또한 5 mm lift에서 gripper actuator target을 `0.30`, `0.35`, `0.40`, `0.42`, `0.45`, `0.48965`, `0.50`, `0.52 rad`로 바꿨다. 처음 일곱 target은 close 단계에서 양 jaw 접촉이 있었으나 settle 뒤에는 모두 link6 한쪽 접촉만 남았고, `0.52 rad`에서는 close 단계부터 양 jaw 접촉이 없었다. 이는 각도 target의 제한된 시뮬레이션 표본 조사이며 jaw aperture/힘의 물리 단위나 실물 보정값이 아니다.

재현 명령:

```bash
MUJOCO_GL=egl .venv-mujoco/bin/python scripts/audit_mujoco_pick_dynamics.py \
  --goal-joints 'Joint1=1.3257822951314233,Joint2=0.06783448034090353,Joint3=-1.0237959140043242,Joint4=1.553914708414148,Joint5=-0.6059260770777783' \
  --gripper-open-rad 0.5235987755982988 \
  --gripper-close-rad 0.48965459480059115 --save-artifact
```

모든 값은 현재 placeholder scene의 MuJoCo radian qpos이며 실물 명령이 아닙니다. 이 후보를 다음에 검토할 때는 target 접촉 시작을 고려한 별도 pregrasp/approach waypoint, 연속 경로 검사, 실제 tool frame과 물체 치수를 확인해야 합니다.

검사 스크립트가 출력하는 라디안 joint 범위는 **upstream simulation model의 범위**입니다. OMX 실물의 허용 목표 범위나 controller/LeRobot normalized 값으로 해석하면 안 됩니다. 실물 읽기에서 관측한 0 torque 상태 및 0–4095 register 범위는 이 시뮬레이션 목표 범위와 대응 관계가 검증되지 않았습니다.

## 다음 연결 작업

ROS 2 Jazzy에는 공식 [ros-controls MuJoCo `SystemInterface`](https://control.ros.org/jazzy/doc/mujoco_ros2_control/doc/index.html)가 제공되고 apt 후보도 확인했지만 이 호스트에는 설치하지 않았습니다. 이 경로는 MuJoCo simulation용 `ros2_control`이며, 기존 LeRobot OMX 하드웨어 adapter와 자동으로 같은 계약이 되지는 않습니다. 공식 예제는 `<ros2_control>` joint mapping과 package 전용 control node를 요구하고 camera 기능은 별도 plugin입니다. 실제 적용 전 현재 MJCF의 joint/actuator 이름, URDF description 유무, gripper command 의미, controller configuration과 하드웨어 adapter 경계를 대조해야 합니다.

**ROS 2 mapping 사전 대조 (2026-10-31):** `omx.xml`에는 `Joint1`–`Joint5`와 `Gripper`의 hinge joint, 그리고 같은 이름의 MuJoCo `position` actuator가 있습니다. 공식 interface 문서는 `ros2_control` joint를 MJCF actuator에 연결하며 position actuator의 `position` command를 native mode로 지원한다고 설명하므로 이름 기반 6개 mapping은 검토 가능한 후보입니다. `Gripper_mimic`은 별도 joint와 actuator 없는 equality constraint로 정의돼 있습니다. 공식 문서는 mimic gripper 예시로 tendon actuator와 equality 조합을 설명하므로 이 OMX 구조가 실제 driver에서 같은 방식으로 동작하는지는 설치 후 확인이 필요합니다.

저장소 안에는 URDF/Xacro가 없지만, ROBOTIS의 [OMX-F URDF/Xacro](https://github.com/ROBOTIS-GIT/open_manipulator/tree/b83daec7b98c08d00adb856d7e31f2b0065d3be9/open_manipulator_description/urdf/omx_f)와 [ros2_control macro](https://github.com/ROBOTIS-GIT/open_manipulator/blob/b83daec7b98c08d00adb856d7e31f2b0065d3be9/open_manipulator_description/ros2_control/omx_f.ros2_control.xacro)를 고정 commit `b83daec7b98c08d00adb856d7e31f2b0065d3be9`에서 읽어 현재 로컬 MJCF의 기반 commit `d8344c0dbe7a00208d0301111523dde65efc174a`와 정적으로 비교했습니다. 일곱 joint 모두 origin xyz/rpy와 axis 값은 같고 gripper mimic 방향도 대응합니다. 하지만 URDF는 모든 joint에 `[-2π, +2π]` 범위를 쓰는 반면 MJCF는 각 joint별 범위가 다르고 더 좁거나 비대칭입니다. 따라서 upstream kinematic description은 재사용 후보지만 그 limit을 MuJoCo motion planning에 재사용하면 안 됩니다. joint 이름도 `joint1`–`joint5`, `gripper_joint_1/2` 대 `Joint1`–`Joint5`, `Gripper/Gripper_mimic`로 달라 plugin name mapping에 맞출 정적 description/config가 필요합니다. upstream control macro는 real mode에서 Dynamixel hardware plugin, `use_sim` mode에서 Gazebo plugin을 선택하므로 MuJoCo용으로 직접 사용할 수 없습니다. 현재 로봇 variant와 upstream OMX-F의 물리적 일치는 별도 확인 대상입니다. 이 대조는 모델 파일의 정적 수치 비교이며 plugin 초기화·controller·접촉 동작은 확인하지 않았습니다. 관제/LeRobot normalized `*.pos`와 MuJoCo 라디안 command 사이 변환도 근거가 없어 미구현입니다. [Jazzy hardware interface 문서](https://control.ros.org/jazzy/doc/mujoco_ros2_control/mujoco_ros2_control/docs/hardware_interface.html)

| MuJoCo joint | upstream OMX-F URDF joint | origin/axis | MuJoCo range (rad) | URDF range (rad) | mimic |
|---|---|---|---:|---:|---|
| `Joint1` | `joint1` | 일치 | `[-4.712389, 6.283185]` | `[-6.283185, 6.283185]` | — |
| `Joint2` | `joint2` | 일치 | `[-2.094395, 1.570796]` | `[-6.283185, 6.283185]` | — |
| `Joint3` | `joint3` | 일치 | `[-2.094395, 1.570796]` | `[-6.283185, 6.283185]` | — |
| `Joint4` | `joint4` | 일치 | `[-1.745329, 1.745329]` | `[-6.283185, 6.283185]` | — |
| `Joint5` | `joint5` | 일치 | `[-4.712389, 4.712389]` | `[-6.283185, 6.283185]` | — |
| `Gripper` | `gripper_joint_1` | 일치 | `[0, 1.745329]` | `[-6.283185, 6.283185]` | — |
| `Gripper_mimic` | `gripper_joint_2` | 일치 | `[-1.745329, 0]` | `[-6.283185, 6.283185]` | `-1` |

URDF의 넓은 범위는 upstream description 값의 기록이며 simulation command limit으로 승인된 값이 아닙니다. 실제 제어 범위는 별도의 공식 motor config와 안전 검토를 확인하기 전까지 미확정입니다.

- upstream 모델과 설치 OMX-F의 링크, tool center point, 그리퍼 접촉 위치를 시각적으로 비교합니다.
- 카메라/선반의 실제 장착 pose와 치수를 읽기 전용 측정 후 placeholder와 별도 calibration 버전으로 교체합니다.
- LeRobot normalized action ↔ MuJoCo radian 변환은 실물 보정/방향/영점 대응을 읽기 전용으로 확인한 뒤 모의 adapter에만 정의합니다.
- `link6` body origin을 실제 tool-center-point와 대조하고, 선반 접촉을 해소한 placeholder grasp target/approach waypoint 및 적절한 swept-path 충돌 계획을 추가합니다. 현재 position IK의 discrete interpolation sample check는 조밀한 continuous collision 검사나 planner가 아닙니다. MuJoCo model limits는 실물 안전 인터록을 대신하지 않습니다.
- 요청받은 정적/런타임 검증 절차로 두 MJCF camera를 실제 렌더하고 출력 shape·depth 값·segmentation map을 확인합니다. 렌더 결과를 실제 camera/depth calibration 또는 ROS 2 sensor topic으로 간주하지 않습니다.
