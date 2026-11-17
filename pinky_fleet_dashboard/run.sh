#!/usr/bin/env bash
set -eo pipefail
APP_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
set -u
export ROS_DOMAIN_ID=0
export PYTHONUNBUFFERED=1
cd "$APP_DIR"
mkdir -p logs
if /usr/bin/python3 -c "import socket,sys;s=socket.socket();s.settimeout(.3);sys.exit(s.connect_ex(('127.0.0.1',8765)))"; then
  echo "관제가 이미 실행 중입니다: http://127.0.0.1:8765/"
  echo "서비스 재시작: systemctl --user restart pinky-fleet.service"
  exit 0
fi
# Direct per-domain readers serve the dashboard. Bridges are opt-in for RViz
# or other tools in domain 0: ./run.sh --bridges
PIDS=()
cleanup() {
  for pid in "${PIDS[@]:-}"; do [[ -n "$pid" ]] && kill "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM
if [[ "${1:-}" == "--bridges" ]]; then
  BRIDGE_BIN="$(ros2 pkg prefix domain_bridge)/lib/domain_bridge/domain_bridge"
  for domain in 10 11 12; do
    ROS_DOMAIN_ID=0 "$BRIDGE_BIN" "$APP_DIR/config/bridge/domain-${domain}.yaml" >>"logs/bridge-${domain}.log" 2>&1 &
    PIDS+=("$!")
  done
elif [[ $# -gt 0 ]]; then
  echo "Usage: ./run.sh [--bridges]" >&2
  exit 2
fi
/usr/bin/python3 app.py &
APP_PID=$!
PIDS+=("$APP_PID")
wait "$APP_PID"
