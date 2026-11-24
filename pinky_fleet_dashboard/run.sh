#!/usr/bin/env bash
# 핑키 관제(app.py)를 직접 띄우는 스크립트. 평소에는 사용자 서비스 pinky-fleet.service가 이것 대신 관제를 띄운다.
# 사용법: ./run.sh            관제만 띄운다(로봇 도메인 10·11·12를 app.py가 직접 구독)
#         ./run.sh --bridges  도메인 0의 RViz 등에서 핑키 토픽을 봐야 할 때 domain_bridge 세 개도 함께 띄운다
# 이미 8765 포트에 관제가 떠 있으면 주소만 알려 주고 끝난다. 끝날 때(Ctrl+C 포함) 띄운 프로세스를 모두 정리한다.
set -eo pipefail
APP_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
set -u  # ROS setup.bash는 정의 안 된 변수를 읽으므로 source 뒤에 켠다
export ROS_DOMAIN_ID=0
export PYTHONUNBUFFERED=1  # journalctl에 출력이 바로 보이도록 버퍼링을 끈다
cd "$APP_DIR"
mkdir -p logs
# 0.3초 안에 127.0.0.1:8765에 붙으면(종료 코드 0) 관제가 이미 떠 있다.
if /usr/bin/python3 -c "import socket,sys;s=socket.socket();s.settimeout(.3);sys.exit(s.connect_ex(('127.0.0.1',8765)))"; then
  echo "관제가 이미 실행 중입니다: http://127.0.0.1:8765/"
  echo "서비스 재시작: systemctl --user restart pinky-fleet.service"
  exit 0
fi
# 관제는 도메인마다 직접 구독하는 수신기로 돈다. 브리지는 도메인 0의 RViz나
# 다른 도구가 필요할 때만 켠다: ./run.sh --bridges
PIDS=()  # 이 스크립트가 띄운 백그라운드 프로세스(브리지, app.py)
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
wait "$APP_PID"  # app.py가 끝나면 EXIT trap이 브리지까지 정리한다
