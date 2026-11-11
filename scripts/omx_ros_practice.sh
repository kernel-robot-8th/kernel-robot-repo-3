#!/usr/bin/env bash
# LeRobot OMX 팔을 ROS 2 action(/omx/move_to_slot)으로 다뤄 보는 단계별 실습 도우미.
# 안내: docs/implementation/omx-ros-practice.md
# 모든 단계가 같은 ROS 환경(Jazzy + omx_interfaces, 격리 domain, localhost)을 쓰도록 맞춰 줄 뿐,
# 이동 로직과 안전장치는 scripts/omx_slot_motion.py에 있습니다. 하드웨어 전원 차단을 대신하지 않습니다.
set -u

REPO_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
SELF="${REPO_DIR}/scripts/$(basename -- "${BASH_SOURCE[0]}")"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
INTERFACES_SETUP="${REPO_DIR}/cpp/install/setup.bash"
LEROBOT_PY="${LEROBOT_PY:-$HOME/lerobot-venv/bin/python}"
FOLLOWER_PORT="${OMX_FOLLOWER_PORT:-/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_F58AE68650323952582E3120FF05260E-if00}"
DOMAIN="${OMX_ROS_DOMAIN_ID:-87}"
POSES="${REPO_DIR}/config/omx-slot-poses.json"

usage() {
  cat <<EOF
사용법: scripts/omx_ros_practice.sh <단계> [옵션]

  check              0단계: 실습 준비 상태 확인 (모터와 통신하지 않음)
  shell              ROS 환경을 맞춘 bash를 엽니다 (ros2 CLI 연습용)
  worker fake        1단계: 가짜 모터 워커 (팔 불필요, 결과 simulated)
                     worker 뒤 옵션은 omx_ros_worker.py로 전달 (예: --profile-velocity 5)
  probe              2단계: 실물 모터 레지스터 읽기만
  record <이름>      3단계: 지금 팔 자세를 기록 (읽기만, 예: home)
  worker dry         4단계: 실물 모터를 읽기만 하는 워커 (쓰지 않음)
  worker real        5단계: 실물 이동 워커 (목표를 받으면 팔이 움직임)
  goal <칸> [옵션]   워커에 목표 하나 보내기 (Ctrl+C = 취소). 옵션은 --help 참고
  release            팔 관절 토크 끄기 (팔을 손으로 받친 뒤)

환경: ROS_DOMAIN_ID=${DOMAIN} (OMX_ROS_DOMAIN_ID로 변경), localhost만
팔로워 포트: ${FOLLOWER_PORT} (OMX_FOLLOWER_PORT로 변경)
EOF
}

ros_env() {
  # setup.bash는 정의되지 않은 변수를 참조하므로 set -u를 잠시 끕니다.
  set +u
  # shellcheck disable=SC1090
  source "$ROS_SETUP" && source "$INTERFACES_SETUP"
  set -u
  export ROS_DOMAIN_ID="$DOMAIN" ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
}

# 현재 로그인 세션에 dialout 그룹이 아직 없으면 sg로 실행합니다 (그룹 추가 뒤 다시 로그인하기 전).
# sg는 setgid라 LD_LIBRARY_PATH가 지워지므로, ROS 워커는 sg 안에서 이 스크립트(__ros_worker)로 환경을 다시 맞춥니다.
as_dialout() {
  if id -nG | grep -qw dialout; then "$@"; else sg dialout -c "$(printf '%q ' "$@")"; fi
}

confirm() {
  printf '%s\n입력 "%s" 후 Enter (그 밖에는 취소): ' "$1" "$2"
  local answer
  read -r answer
  [[ "$answer" == "$2" ]] || { echo "취소했습니다. 아무것도 보내지 않았습니다."; exit 1; }
}

need_port() {
  [[ -e "$FOLLOWER_PORT" ]] && return 0
  echo "팔로워 포트가 없습니다: ${FOLLOWER_PORT}" >&2
  ls /dev/serial/by-id/ 2>/dev/null | sed 's/^/  연결된 장치: /' >&2
  exit 1
}

check() {
  local missing=0
  item() { if eval "$2" >/dev/null 2>&1; then echo "  ✓ $1"; else echo "  ✗ $1${3:+  → $3}"; missing=1; fi; }
  echo "[ROS 2]"
  item "ROS 2 Jazzy (${ROS_SETUP})" "[[ -f '$ROS_SETUP' ]]"
  item "omx_interfaces 빌드" "[[ -f '$INTERFACES_SETUP' ]]" "cd cpp && colcon build --packages-select omx_interfaces"
  echo "[LeRobot]"
  item "LeRobot 가상환경 (${LEROBOT_PY})" "'$LEROBOT_PY' -c 'import lerobot'"
  item "LeRobot 가상환경에서 rclpy·MoveToSlot import" \
    "( ros_env; '$LEROBOT_PY' -c 'import rclpy; from omx_interfaces.action import MoveToSlot' )"
  echo "[실물 팔]"
  item "dialout 그룹 (시리얼 포트 권한)" "getent group dialout | grep -qw '$USER'" "sudo usermod -aG dialout \$USER"
  item "dialout 권한으로 ROS 워커 불러오기 (실물 워커와 같은 경로)" "as_dialout bash '$SELF' __ros_worker --help"
  if [[ -e "$FOLLOWER_PORT" ]]; then echo "  ✓ 팔로워 포트 ${FOLLOWER_PORT}"
  else echo "  △ 팔로워 포트 없음 (1단계는 팔 없이 가능)"; fi
  echo "[기록된 자세] ($(basename "$POSES"))"
  python3 - "$POSES" "$FOLLOWER_PORT" <<'PY'
import json, sys
from pathlib import Path
document = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
print(f"  보드 {document['follower_board']}")
for name, pose in document["poses"].items():
    print(f"  {name:<12} {pose['recorded_at']}  {pose['ticks']}")
if "home" not in document["poses"]:
    print("  △ home 없음: 3단계에서 기록하면 칸 ↔ home 왕복을 연습할 수 있습니다")
if Path(sys.argv[2]).name.find(document["follower_board"]) < 0:
    print("  ✗ 포트 이름이 자세를 기록한 보드와 다릅니다. 워커가 시작을 거부합니다")
PY
  echo
  echo "모터에는 아무것도 읽거나 쓰지 않았습니다. 장치 발견은 팔이 준비됐다는 뜻이 아닙니다."
  return $missing
}

case "${1:-}" in
  check) check ;;
  shell)
    ros_env
    echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}, localhost만. 예: ros2 action list -t, ros2 interface show omx_interfaces/action/MoveToSlot"
    exec bash --norc -i
    ;;
  worker)
    ros_env
    cd "$REPO_DIR" || exit 1
    case "${2:-}" in
      fake) exec "$LEROBOT_PY" scripts/omx_ros_worker.py --fake-follower "${@:3}" ;;
      dry) need_port; as_dialout bash "$SELF" __ros_worker --dry-run --port "$FOLLOWER_PORT" "${@:3}" ;;
      real)
        need_port
        confirm "실물 워커를 시작합니다. 목표를 받으면 팔이 움직입니다.
- 팔 주변에 사람·물건이 없는지, 전원 차단 스위치에 손이 닿는지 확인하세요.
- 리더 팔 텔레오퍼레이션이나 LeRobot 정책이 같은 포트를 쓰고 있지 않은지 확인하세요." "실물"
        as_dialout bash "$SELF" __ros_worker --port "$FOLLOWER_PORT" "${@:3}"
        ;;
      *) usage >&2; exit 2 ;;
    esac
    ;;
  __ros_worker)
    ros_env
    cd "$REPO_DIR" || exit 1
    exec "$LEROBOT_PY" scripts/omx_ros_worker.py "${@:2}"
    ;;
  probe)
    need_port
    cd "$REPO_DIR" || exit 1
    as_dialout "$LEROBOT_PY" scripts/probe_omx_readonly.py --port "$FOLLOWER_PORT"
    ;;
  record)
    [[ -n "${2:-}" ]] || { usage >&2; exit 2; }
    need_port
    cd "$REPO_DIR" || exit 1
    as_dialout "$LEROBOT_PY" scripts/record_omx_pose.py "$2" --port "$FOLLOWER_PORT" "${@:3}"
    ;;
  goal)
    shift
    ros_env
    exec python3 "${REPO_DIR}/scripts/omx_move_to_slot_client.py" "$@"
    ;;
  release)
    need_port
    confirm "팔 관절 토크를 끕니다. 팔이 처질 수 있으니 손으로 받치고 있으세요." "토크해제"
    cd "$REPO_DIR" || exit 1
    as_dialout "$LEROBOT_PY" scripts/omx_lerobot_worker.py --release-torque --port "$FOLLOWER_PORT"
    ;;
  -h|--help|"") usage ;;
  *) usage >&2; exit 2 ;;
esac
