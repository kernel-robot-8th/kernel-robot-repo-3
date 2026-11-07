#!/usr/bin/env bash
# LeRobot ACT 집기 정책을 실물 OMX에서 비동기 추론(async_inference)으로 돌려 보고, 끝난 뒤 워커로 원상복귀하는 시험 도우미.
# 관제 작업 흐름과는 아직 연결하지 않았다. 정책에는 완료 신호가 없어서, 사람이 보고 Ctrl+C로 멈춘다.
#
#   set -a; source .control-station/control-station.env; set +a   # PC별 장치 경로
#   터미널 1:  scripts/omx_act_policy_test.sh server
#   터미널 2:  scripts/omx_act_policy_test.sh client [모델]   ← 팔이 움직인다. 전원을 끊을 수 있는 사람이 지켜본다
#              (놓기가 끝나면 Ctrl+C. 팔은 마지막 자세에서 토크를 유지한다)
#   터미널 2:  scripts/omx_act_policy_test.sh worker          ← 그다음 관제 화면 "원상복귀"
#
# 환경 변수
#   CONTROL_STATION_ACT_FRONT_CAMERA  정책 학습 때의 front 카메라 경로 (필수, 모델마다 다름)
#   CONTROL_STATION_JOINT_CAMERA      정책의 wrist 카메라 = 관제의 Joint Camera (기본 Innomaker by-id)
#   CONTROL_STATION_FOLLOWER_PORT     follower 포트 (기본 /dev/omx_follower)
#   CONTROL_STATION_OMX_POSES         워커의 자세 파일. 그 파일을 기록한 보드와 follower가 같아야 함
#                                     (기본 config/omx-slot-poses.json)
#
# 팔 포트는 배타적으로 열리지 않아 두 프로세스가 동시에 모터에 쓸 수 있다. 그래서 정책과 워커를 차례로만 실행한다.
set -euo pipefail

REPO_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PY="${LEROBOT_PYTHON:-$HOME/lerobot-venv/bin/python}"
FOLLOWER="${CONTROL_STATION_FOLLOWER_PORT:-/dev/omx_follower}"
WRIST="${CONTROL_STATION_JOINT_CAMERA:-/dev/v4l/by-id/usb-Innomaker_Innomaker-U20CAM-720P_SN0001-video-index0}"
FRONT="${CONTROL_STATION_ACT_FRONT_CAMERA:-}"
POSES="${CONTROL_STATION_OMX_POSES:-$REPO_DIR/config/omx-slot-poses.json}"
MODEL="${2:-JJinJJin2/omx_act_policy_v2}"

arm_busy() {
  pgrep -f "omx_lerobot_worker.py|omx_ros_worker.py|lerobot-teleoperate|lerobot.teleoperate|lerobot-record|robot_client import async_client" >/dev/null
}

case "${1:-}" in
  server)
    exec "$PY" -m lerobot.async_inference.policy_server \
      --host=127.0.0.1 --port=8080 --fps=30 --inference_latency=0.033 --obs_queue_timeout=1
    ;;
  client)
    [ -n "$FRONT" ] || { echo "CONTROL_STATION_ACT_FRONT_CAMERA를 지정하세요 (정책 학습 때의 front 카메라)." >&2; exit 1; }
    for dev in "$FOLLOWER" "$FRONT" "$WRIST"; do
      [ -e "$dev" ] || { echo "장치 없음: $dev" >&2; exit 1; }
    done
    if arm_busy; then echo "OMX 워커·텔레옵·녹화·다른 정책 클라이언트가 실행 중입니다. 끄고 다시 실행하세요." >&2; exit 1; fi
    if fuser "$FRONT" "$WRIST" >/dev/null 2>&1; then
      echo "카메라를 다른 프로세스가 쓰고 있습니다: $(fuser "$FRONT" "$WRIST" 2>&1)" >&2; exit 1
    fi
    echo "모델 $MODEL"
    echo "follower $FOLLOWER"
    echo "front $FRONT"
    echo "wrist $WRIST"
    echo "연결할 때 LeRobot이 모터 설정을 쓰느라 토크가 잠깐 꺼집니다. 쉬는 자세에서 시작하세요."
    CLIENT=(
      "$PY" -c 'import lerobot.cameras.opencv.configuration_opencv; import lerobot.robots.omx_follower.config_omx_follower; from lerobot.async_inference.robot_client import async_client; async_client()'
      --robot.type=omx_follower
      "--robot.port=$FOLLOWER"
      --robot.id=omx_follower_arm
      # LeRobot 기본값(True)이면 Ctrl+C로 끌 때 토크가 꺼져 팔이 떨어진다.
      --robot.disable_torque_on_disconnect=false
      "--robot.cameras={ front: {type: opencv, index_or_path: \"$FRONT\", width: 640, height: 480, fps: 30, fourcc: MJPG}, wrist: {type: opencv, index_or_path: \"$WRIST\", width: 640, height: 480, fps: 30, fourcc: MJPG}}"
      "--task=Pick up Doll"
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
            --profile-velocity 5 --max-delta 1500)
    exec sg dialout -c "$(printf '%q ' "${WORKER[@]}")"
    ;;
  *)
    echo "사용: $0 server | client [모델] | worker" >&2; exit 2
    ;;
esac
