#!/usr/bin/env bash
set -eo pipefail

APP_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# ROS setup scripts reference optional variables that may be unset in a
# user's shell (for example AMENT_TRACE_SETUP_FILES). Do not enable nounset
# until after sourcing the ROS environment.
source /opt/ros/jazzy/setup.bash
set -u
export ROS_DOMAIN_ID=0

BRIDGE_BIN="$(ros2 pkg prefix domain_bridge)/lib/domain_bridge/domain_bridge"
if [[ ! -x "$BRIDGE_BIN" ]]; then
  echo "ROS 2 domain_bridge executable is unavailable: $BRIDGE_BIN" >&2
  exit 1
fi

# Older runs used `ros2 run`, which can leave its child bridge processes alive
# after the wrapper exits. Clean only bridges launched from this dashboard.
for pid in $(pgrep -x domain_bridge 2>/dev/null || true); do
  cmdline="$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null || true)"
  case "$cmdline" in
    "$BRIDGE_BIN $APP_DIR/config/bridge/domain-"*) kill "$pid" 2>/dev/null || true ;;
  esac
done
sleep 1

LOG_DIR="$(mktemp -d -t pinky-fleet.XXXXXX)"
PIDS=()
cleanup() {
  for pid in "${PIDS[@]:-}"; do kill "$pid" 2>/dev/null || true; done
  wait 2>/dev/null || true
  rm -rf "$LOG_DIR"
}
trap cleanup EXIT INT TERM

for domain in 10 11 12; do
  ROS_DOMAIN_ID=0 "$BRIDGE_BIN" "$APP_DIR/config/bridge/domain-${domain}.yaml" \
    >"$LOG_DIR/domain-${domain}.log" 2>&1 &
  PIDS+=("$!")
done

sleep 1
python3 "$APP_DIR/app.py"
