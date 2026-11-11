#!/usr/bin/env bash
# LeRobot ACT 집기 정책(config/omx-act-policies.json)을 실물 OMX에서 비동기 추론(async_inference)으로 돌리는 도우미.
#
#   set -a; source .control-station/control-station.env; set +a   # PC별 장치 경로
#   터미널 1:  scripts/omx_act_policy_test.sh server        정책 서버 (GPU, 모터 연결 없음). 아래 두 방식 모두 필요
#
# 관제에서 실행 (관제 화면 수동 조작 탭 "당근 집기"·"물티슈 집기" → PICK_WITH_POLICY):
#   터미널 2:  scripts/omx_act_policy_test.sh worker [워커 옵션]   OMX 워커가 정책을 실행하고, 쉬는 자세로 돌아오면 멈춘 뒤 home 정렬
#              실험: worker --policy-any-start  (칸 앞 등 쉬는 자세가 아닌 곳에서도 정책 시작, 한 스텝 40틱 제한)
#
# 관제 없이 수동 시험:
#   터미널 2:  scripts/omx_act_policy_test.sh client [물체]  ← 팔이 움직인다 (기본 물체: 당근). 놓기가 끝나면 Ctrl+C
#              (팔은 마지막 자세에서 토크를 유지한다). 그다음 worker를 켜고 관제 화면 "원상복귀"
#
# 환경 변수
#   CONTROL_STATION_ACT_OVERHEAD_CAMERA  선반을 위에서 보는 카메라 (필수, 정책 학습 때와 같은 위치·시점)
#   CONTROL_STATION_JOINT_CAMERA         그리퍼 카메라 = 관제의 Joint Camera (기본 Innomaker by-id)
#   CONTROL_STATION_FOLLOWER_PORT        follower 포트 (기본 /dev/omx_follower)
#   CONTROL_STATION_ACT_MODEL            client에서 설정 파일 대신 쓸 모델 (예: JJinJJin2/omx_act_policy_v4). 카메라 역할은 물체 설정을 따름
#   CONTROL_STATION_OMX_POSES            워커의 자세 파일. 그 파일을 기록한 보드와 follower가 같아야 함
#                                        (기본 config/omx-slot-poses.json)
# 정책마다 front/wrist에 어느 카메라를 넣을지는 config/omx-act-policies.json의 cameras가 정한다.
#
# 팔 포트는 배타적으로 열리지 않아 두 프로세스가 동시에 모터에 쓸 수 있다. 그래서 정책 클라이언트와 워커를 차례로만 실행한다.
set -euo pipefail

REPO_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PY="${LEROBOT_PYTHON:-$HOME/lerobot-venv/bin/python}"
FOLLOWER="${CONTROL_STATION_FOLLOWER_PORT:-/dev/omx_follower}"
GRIPPER="${CONTROL_STATION_JOINT_CAMERA:-/dev/v4l/by-id/usb-Innomaker_Innomaker-U20CAM-720P_SN0001-video-index0}"
OVERHEAD="${CONTROL_STATION_ACT_OVERHEAD_CAMERA:-}"
POSES="${CONTROL_STATION_OMX_POSES:-$REPO_DIR/config/omx-slot-poses.json}"
POLICIES="$REPO_DIR/config/omx-act-policies.json"

arm_busy() {
  pgrep -f "omx_lerobot_worker.py|omx_ros_worker.py|lerobot-teleoperate|lerobot.teleoperate|lerobot-record|robot_client import async_client" >/dev/null
}

case "${1:-}" in
  server)
    exec "$PY" -m lerobot.async_inference.policy_server \
      --host=127.0.0.1 --port=8080 --fps=30 --inference_latency=0.033 --obs_queue_timeout=1
    ;;
  client)
    [ -n "$OVERHEAD" ] || { echo "CONTROL_STATION_ACT_OVERHEAD_CAMERA를 지정하세요 (선반을 위에서 보는 카메라)." >&2; exit 1; }
    for dev in "$FOLLOWER" "$OVERHEAD" "$GRIPPER"; do
      [ -e "$dev" ] || { echo "장치 없음: $dev" >&2; exit 1; }
    done
    if arm_busy; then echo "OMX 워커·텔레옵·녹화·다른 정책 클라이언트가 실행 중입니다. 끄고 다시 실행하세요." >&2; exit 1; fi
    if fuser "$OVERHEAD" "$GRIPPER" >/dev/null 2>&1; then
      echo "카메라를 다른 프로세스가 쓰고 있습니다: $(fuser "$OVERHEAD" "$GRIPPER" 2>&1)" >&2; exit 1
    fi
    # 물체 이름(당근/carrot/물티슈...) → 모델, task, front·wrist 카메라 역할
    read -r MODEL FRONT_ROLE WRIST_ROLE TASK < <(python3 -I - "$POLICIES" "${2:-당근}" <<'EOF'
import json, sys
name = sys.argv[2].strip().casefold()
for policy in json.load(open(sys.argv[1], encoding="utf-8"))["policies"]:
    if name in {n.casefold() for n in (policy["object_class"], policy["label"], *policy["aliases"])}:
        print(policy["model"], policy["cameras"]["front"], policy["cameras"]["wrist"], policy["task"])
        break
else:
    sys.exit(f"{sys.argv[2]}에 학습된 정책이 없습니다")
EOF
    ) || exit 1
    # 새로 학습한 모델을 관제 설정을 바꾸지 않고 시험할 때. 카메라 역할(front/wrist)은 위 물체의 정책 설정을 그대로 쓴다.
    MODEL="${CONTROL_STATION_ACT_MODEL:-$MODEL}"
    role_device() { [ "$1" = OVERHEAD ] && echo "$OVERHEAD" || echo "$GRIPPER"; }
    FRONT="$(role_device "$FRONT_ROLE")"
    WRIST="$(role_device "$WRIST_ROLE")"
    echo "모델 $MODEL"
    echo "follower $FOLLOWER"
    echo "front ($FRONT_ROLE) $FRONT"
    echo "wrist ($WRIST_ROLE) $WRIST"
    echo "연결할 때 LeRobot이 모터 설정을 쓰느라 토크가 잠깐 꺼집니다. 쉬는 자세에서 시작하세요."
    CLIENT=(
      "$PY" -c 'import lerobot.cameras.opencv.configuration_opencv; import lerobot.robots.omx_follower.config_omx_follower; from lerobot.async_inference.robot_client import async_client; async_client()'
      --robot.type=omx_follower
      "--robot.port=$FOLLOWER"
      --robot.id=omx_follower_arm
      # LeRobot 기본값(True)이면 Ctrl+C로 끌 때 토크가 꺼져 팔이 떨어진다.
      --robot.disable_torque_on_disconnect=false
      "--robot.cameras={ front: {type: opencv, index_or_path: \"$FRONT\", width: 640, height: 480, fps: 30, fourcc: MJPG}, wrist: {type: opencv, index_or_path: \"$WRIST\", width: 640, height: 480, fps: 30, fourcc: MJPG}}"
      "--task=$TASK"
      --server_address=127.0.0.1:8080
      --policy_type=act
      "--pretrained_name_or_path=$MODEL"
      --policy_device=cuda
      --actions_per_chunk=100
      --chunk_size_threshold=0.7
      --aggregate_fn_name=weighted_average
      # True면 종료 뒤 matplotlib 창이 떠서 닫을 때까지 프로세스가 끝나지 않는다.
      --debug_visualize_queue_size=false
    )
    sg dialout -c "$(printf '%q ' "${CLIENT[@]}")" || true
    echo
    echo "정책 종료. 팔과 그리퍼는 마지막 자세에서 토크를 유지합니다."
    echo "물체를 놓았고 그리퍼가 벌어졌는지 확인한 뒤: $0 worker → 관제 화면 원상복귀"
    ;;
  worker)
    if arm_busy; then echo "정책 클라이언트나 다른 팔 프로세스가 아직 실행 중입니다. 먼저 끄세요." >&2; exit 1; fi
    cd "$REPO_DIR"
    WORKER=("$PY" scripts/omx_lerobot_worker.py --server http://127.0.0.1:8000 --port "$FOLLOWER" --poses "$POSES"
            --overhead-camera "$OVERHEAD" --gripper-camera "$GRIPPER" --profile-velocity 5 --max-delta 1500 --tolerance 25 "${@:2}")
    exec sg dialout -c "$(printf '%q ' "${WORKER[@]}")"
    ;;
  *)
    echo "사용: $0 server | worker | client [물체]" >&2; exit 2
    ;;
esac
