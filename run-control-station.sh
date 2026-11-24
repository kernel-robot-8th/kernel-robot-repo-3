#!/usr/bin/env bash
# 관제 서버 런처. 저장소 루트에서 ./run-control-station.sh 로 실행한다(인수 없음, --help만 받는다).
#
# 하는 일:
#   1. .control-station/control-station.env가 있으면 읽어 모든 값을 환경 변수로 내보낸다.
#   2. 주차 판정(CONTROL_STATION_PARKING_SPOT)이나 ros:<토픽> 카메라를 쓰면 ROS 2 Jazzy와 cpp/ 작업 공간을 source한다.
#   3. 관제 서버(python -m control_station.app)를 띄우고 /api/health가 응답할 때까지 최대 30초 기다린다.
#   4. 팔 포트·자세 파일·천장/관절 카메라가 모두 설정된 팔이 있으면 LeRobot ACT 정책 서버와
#      팔 워커(scripts/omx_lerobot_worker.py)를 확인해, 이미 떠 있으면 재사용하고 빠진 것만 시작한다.
#   5. 모든 출력은 .control-station/logs/session-<시각>-<PID>.log에도 남긴다.
#
# 실물 하드웨어: 이 스크립트가 직접 팔을 움직이지는 않는다. 다만 시작한 팔 워커는 관제 명령을 받아
# ACCEPTED 응답 뒤 실물 OMX 팔을 움직이므로, 워커가 시작된 뒤에는 대기 중인 명령이 실행될 수 있다.
# Ctrl-C로 끝낼 때 실행 중인 팔 명령이 없을 때만 이 스크립트가 시작한 워커와 정책 서버를 함께 끈다.
set -euo pipefail

REPO_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ENV_FILE="${REPO_DIR}/.control-station/control-station.env"
if [[ -f "${ENV_FILE}" ]]; then
  # set -a: 이 사이에서 정의한 변수를 모두 export해 관제·워커 자식 프로세스가 읽게 한다.
  set -a
  # shellcheck disable=SC1090
  source "${ENV_FILE}"
  set +a
fi

# 세션 ID = 시작 시각 + 이 셸의 PID. 같은 초에 두 번 실행해도 로그 파일이 겹치지 않는다.
SESSION_ID="$(date +%Y%m%d-%H%M%S)-$$"
LOG_DIR="${REPO_DIR}/.control-station/logs"
SESSION_LOG="${LOG_DIR}/session-${SESSION_ID}.log"
CONTROL_STATION_URL="${CONTROL_STATION_URL:-http://127.0.0.1:${CONTROL_STATION_PORT:-8000}}"
POLICY_SERVER_HOST="${CONTROL_STATION_POLICY_SERVER_HOST:-127.0.0.1}"
POLICY_SERVER_PORT="${CONTROL_STATION_POLICY_SERVER_PORT:-8080}"
POLICY_SERVER_ADDRESS="${POLICY_SERVER_HOST}:${POLICY_SERVER_PORT}"

usage() {
  printf '사용법: %s [--help]\n' "$0"
  printf '  인수 없음  관제와 설정된 정책 서버·팔 워커 시작\n'
}

case "${1:-}" in
  "") ;;
  -h|--help) usage; exit 0 ;;
  *) usage >&2; exit 2 ;;
esac

# C++ 주차 판정 노드와 ros:<토픽> 카메라는 ROS 2와 cpp/ 작업 공간의 인터페이스가 있어야 한다.
if [[ -n "${CONTROL_STATION_PARKING_SPOT:-}" || "${CONTROL_STATION_HEAD_CAMERA:-}" == ros:* \
      || "${CONTROL_STATION_SHELF_CAMERA:-}" == ros:* ]]; then
  set +u  # ROS setup 스크립트가 정의되지 않은 변수를 읽으므로 잠시 끈다
  if [[ -z "${ROS_DISTRO:-}" && -f /opt/ros/jazzy/setup.bash ]]; then
    source /opt/ros/jazzy/setup.bash
  fi
  if [[ -f "${REPO_DIR}/cpp/install/setup.bash" ]]; then
    source "${REPO_DIR}/cpp/install/setup.bash"
  else
    printf 'cpp/install/setup.bash가 없습니다. cpp/에서 colcon build를 먼저 실행하세요.\n' >&2
    exit 2
  fi
  set -u
fi

mkdir -p "${LOG_DIR}"
# 이 시점부터 표준 출력·오류를 화면과 세션 로그 양쪽에 남긴다(자식 프로세스 출력도 포함).
exec > >(tee -a "${SESSION_LOG}") 2>&1
printf 'Session log: %s\n' "${SESSION_LOG}"

cd "${REPO_DIR}"
export PYTHONPATH="${REPO_DIR}/src${PYTHONPATH:+:${PYTHONPATH}}"
# 관제용 Python 선택 순서: CONTROL_STATION_PYTHON → 저장소 .venv → PATH의 python3.
PYTHON_BIN="${CONTROL_STATION_PYTHON:-}"
if [[ -z "${PYTHON_BIN}" && -x "${REPO_DIR}/.venv/bin/python" ]]; then
  PYTHON_BIN="${REPO_DIR}/.venv/bin/python"
fi
if [[ -z "${PYTHON_BIN}" ]]; then
  PYTHON_BIN="$(command -v python3)"
fi

# 같은 주소에 이미 관제가 떠 있으면 두 번째 관제를 띄우지 않는다(같은 DB·포트를 두 프로세스가 쓰지 않게).
if curl --silent --show-error --max-time 1 "${CONTROL_STATION_URL}/api/health" >/dev/null 2>&1; then
  printf '이미 관제 서버가 실행 중입니다: %s\n' "${CONTROL_STATION_URL}"
  exit 1
fi

# 이 스크립트가 직접 시작한 프로세스만 기록한다. 재사용한 기존 워커·정책 서버는 종료 대상이 아니다.
CONTROL_STATION_PID=""
POLICY_SERVER_PID=""
STARTED_WORKER_PIDS=()
# 종료 정리. 관제를 끄기 전에 실행 중인 팔 명령이 있는지 먼저 묻는다(관제를 끄면 물어볼 곳이 없어진다).
# 실행 중인 명령이 확실히 없을 때("false")만 시작한 워커·정책 서버를 끈다. 팔이 움직이는 도중에 워커를
# 끊으면 결과를 알 수 없게 되므로, 확인할 수 없으면("unknown") 그대로 두고 안내만 한다.
cleanup() {
  local status=$?
  trap - EXIT INT TERM
  local in_flight="unknown"
  if api_ready; then
    if in_flight="$(has_in_flight_commands 2>/dev/null)"; then :; else in_flight="unknown"; fi
  fi
  if [[ -n "${CONTROL_STATION_PID}" ]] && kill -0 "${CONTROL_STATION_PID}" 2>/dev/null; then
    kill -INT "${CONTROL_STATION_PID}" 2>/dev/null || true
    wait "${CONTROL_STATION_PID}" 2>/dev/null || true
  fi
  if [[ "${in_flight}" == "false" ]]; then
    # SIGINT를 먼저 모두 보낸 뒤 기다려, 워커와 정책 서버가 동시에 정리되게 한다.
    local pid
    for pid in "${STARTED_WORKER_PIDS[@]}"; do
      kill -INT "${pid}" 2>/dev/null || true
    done
    if [[ -n "${POLICY_SERVER_PID}" ]]; then
      kill -INT "${POLICY_SERVER_PID}" 2>/dev/null || true
    fi
    for pid in "${STARTED_WORKER_PIDS[@]}"; do wait "${pid}" 2>/dev/null || true; done
    if [[ -n "${POLICY_SERVER_PID}" ]]; then wait "${POLICY_SERVER_PID}" 2>/dev/null || true; fi
  else
    printf '실행 중 명령 상태를 확인할 수 없어 시작한 팔 워커와 정책 서버를 종료하지 않았습니다.\n'
  fi
  exit "${status}"
}
# INT/TERM은 관례적 종료 코드(130 = 128+2, 143 = 128+15)로 exit만 하고, 실제 정리는 EXIT trap 한 곳에서 한다.
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

# 관제 /api/health가 1초 안에 2xx로 답하면 준비된 것으로 본다.
api_ready() {
  curl --silent --show-error --fail --max-time 1 "${CONTROL_STATION_URL}/api/health" >/dev/null 2>&1
}

# GET /api/arm/commands에서 워커가 가져갔거나 실행 중인 명령(DISPATCHED·ACCEPTED·RUNNING·CANCELING)이
# 하나라도 있으면 true, 없으면 false를 출력한다. 조회 실패는 0이 아닌 종료 코드로 알린다.
has_in_flight_commands() {
  "${PYTHON_BIN}" - "${CONTROL_STATION_URL}" <<'PY'
import json
import sys
import urllib.request

request = urllib.request.Request(f"{sys.argv[1].rstrip('/')}/api/arm/commands")
with urllib.request.urlopen(request, timeout=2) as response:
    commands = json.load(response).get("commands", [])
active = {"DISPATCHED", "ACCEPTED", "RUNNING", "CANCELING"}
print("true" if any(command.get("state") in active for command in commands) else "false")
PY
}

# worker_registered <robot_id> <worker_id>
# 관제의 워커 목록에서 그 팔·워커 ID의 마지막 long-poll(last_poll_at)이 최근이면 성공(0)을 돌려준다.
# 워커는 최대 30초 long-poll을 반복하므로 45초까지는 살아 있는 것으로 보고, -2초는 시계 오차를 허용한 값이다.
worker_registered() {
  "${PYTHON_BIN}" - "${CONTROL_STATION_URL}" "$1" "$2" <<'PY'
import json
import sys
from datetime import datetime, timezone
from urllib.request import urlopen

with urlopen(f"{sys.argv[1].rstrip('/')}/api/arm/commands", timeout=2) as response:
    workers = json.load(response).get("workers", [])
for worker in workers:
    if worker.get("robot_id") != sys.argv[2] or worker.get("worker_id") != sys.argv[3]:
        continue
    try:
        last_poll = datetime.fromisoformat(worker["last_poll_at"])
        age = (datetime.now(timezone.utc) - last_poll).total_seconds()
    except (KeyError, ValueError, TypeError):
        break
    if -2 <= age <= 45:
        raise SystemExit(0)
raise SystemExit(1)
PY
}

# LeRobot Python으로 scripts/omx_act_policy_run.py의 check_policy_server를 불러 정책 서버가 Ready인지 본다.
# LEROBOT_PYTHON은 아래에서 정해지며, 이 함수는 그 뒤에만 호출된다.
policy_server_ready() {
  PYTHONPATH="${REPO_DIR}/scripts${PYTHONPATH:+:${PYTHONPATH}}" \
    "${LEROBOT_PYTHON}" - "${POLICY_SERVER_ADDRESS}" <<'PY'
import sys
from omx_act_policy_run import check_policy_server

raise SystemExit(0 if check_policy_server(sys.argv[1], timeout_s=0.4, ready_timeout_s=0.4) is None else 1)
PY
}

# 그 팔(--robot-id)의 omx_lerobot_worker.py 프로세스가 이미 있는지 본다. heartbeat가 없더라도 중복 실행을 막는 용도다.
# [o] 패턴은 검색 명령 자신의 명령줄이 걸리지 않게 하는 관용 표현이다.
worker_process_running() {
  pgrep -f "[o]mx_lerobot_worker.py.*--robot-id $1([[:space:]]|$)" >/dev/null 2>&1
}

printf '관제 서버 시작: %s\n' "${CONTROL_STATION_URL}"
# setsid: 자식을 새 세션에서 띄워 터미널의 Ctrl-C가 자식에게 직접 가지 않게 한다. 종료 순서는 cleanup이 정한다.
# trap - INT: 비대화형 셸의 백그라운드 작업은 SIGINT를 무시한 채 시작하므로, 기본 동작으로 되돌려
# cleanup의 kill -INT가 실제로 자식을 끝낼 수 있게 한다.
setsid bash -c 'trap - INT; exec "$@"' _ "${PYTHON_BIN}" -m control_station.app &
CONTROL_STATION_PID=$!
# 관제가 /api/health에 답할 때까지 최대 30초 기다린다. 그 사이 프로세스가 죽으면 바로 실패로 끝낸다.
for _ in $(seq 1 30); do
  if api_ready; then break; fi
  if ! kill -0 "${CONTROL_STATION_PID}" 2>/dev/null; then
    printf '관제 서버가 시작되지 않았습니다. 세션 로그를 확인하세요.\n' >&2
    exit 1
  fi
  sleep 1
done
if ! api_ready; then
  printf '관제 서버 준비 시간 초과: %s\n' "${CONTROL_STATION_URL}" >&2
  exit 1
fi

# 팔 워커 설정. 왼팔 = arm-01, 오른팔 = arm-02. 왼팔 자세 파일은 예전 단일 팔 설정 CONTROL_STATION_OMX_POSES로도
# 받는다. 자세 파일 경로는 저장소 루트 기준 상대 경로다.
LEFT_PORT="${CONTROL_STATION_LEFT_FOLLOWER_PORT:-}"
LEFT_POSES="${CONTROL_STATION_LEFT_OMX_POSES:-${CONTROL_STATION_OMX_POSES:-}}"
RIGHT_PORT="${CONTROL_STATION_RIGHT_FOLLOWER_PORT:-}"
RIGHT_POSES="${CONTROL_STATION_RIGHT_OMX_POSES:-}"
LEROBOT_DIR="${CONTROL_STATION_LEROBOT_DIR:-${HOME}/Desktop/lerobot}"
LEROBOT_PYTHON="${CONTROL_STATION_LEROBOT_PYTHON:-${LEROBOT_DIR}/venv/bin/python}"
POLICY_SERVER_LOG="${LOG_DIR}/policy-server-${SESSION_ID}.log"

# 팔마다 팔로워 포트·자세 파일·천장(정책 front 입력)·관절(그리퍼) 카메라 장치가 실제로 있어야 워커를 자동 시작한다.
# 하나라도 없으면 그 팔만 건너뛴다. 정책 실행에 필요한 장치가 없는 워커는 명령을 받아도 거부만 하기 때문이다.
LEFT_READY=0
RIGHT_READY=0
if [[ -n "${LEFT_PORT}" && -n "${LEFT_POSES}" && -e "${LEFT_PORT}" && -f "${REPO_DIR}/${LEFT_POSES}" \
      && -n "${CONTROL_STATION_LEFT_CEILING_CAMERA:-}" && -e "${CONTROL_STATION_LEFT_CEILING_CAMERA}" \
      && -n "${CONTROL_STATION_LEFT_JOINT_CAMERA:-}" && -e "${CONTROL_STATION_LEFT_JOINT_CAMERA}" ]]; then
  LEFT_READY=1
else
  printf '왼팔 워커 자동 시작 생략: 포트·자세 파일·좌우(천장/관절) 카메라 경로 설정을 확인하세요.\n'
fi
if [[ -n "${RIGHT_PORT}" && -n "${RIGHT_POSES}" && -e "${RIGHT_PORT}" && -f "${REPO_DIR}/${RIGHT_POSES}" \
      && -n "${CONTROL_STATION_RIGHT_CEILING_CAMERA:-}" && -e "${CONTROL_STATION_RIGHT_CEILING_CAMERA}" \
      && -n "${CONTROL_STATION_RIGHT_JOINT_CAMERA:-}" && -e "${CONTROL_STATION_RIGHT_JOINT_CAMERA}" ]]; then
  RIGHT_READY=1
else
  printf '오른팔 워커 자동 시작 생략: 포트·자세 파일·좌우(천장/관절) 카메라 경로 설정을 확인하세요.\n'
fi

# 팔 런타임 자동 시작. CONTROL_STATION_AUTO_START_ARM_RUNTIME=0이면 관제만 띄운다.
# 순서: 정책 서버를 먼저 준비하고, Ready가 확인된 경우에만 팔 워커를 시작한다(정책 명령이 서버 없이 실행되지 않게).
if [[ "${CONTROL_STATION_AUTO_START_ARM_RUNTIME:-1}" != "0" && ( "${LEFT_READY}" == "1" || "${RIGHT_READY}" == "1" ) ]]; then
  if [[ ! -x "${LEROBOT_PYTHON}" ]]; then
    printf 'LeRobot Python을 찾지 못해 ACT 서버와 팔 워커를 시작하지 않았습니다: %s\n' "${LEROBOT_PYTHON}" >&2
  else
    POLICY_READY=0
    if policy_server_ready; then
      POLICY_READY=1
      printf 'ACT 정책 서버 재사용: %s\n' "${POLICY_SERVER_ADDRESS}"
    else
      # Ready 응답은 없지만 포트에 TCP 연결은 되는지 본다. 열려 있으면 다른 프로그램이나 아직 뜨는 중인
      # 정책 서버가 포트를 쥐고 있는 것이므로 두 번째 서버를 띄우지 않는다.
      if "${PYTHON_BIN}" - "${POLICY_SERVER_HOST}" "${POLICY_SERVER_PORT}" <<'PY'
import socket
import sys

try:
    with socket.create_connection((sys.argv[1], int(sys.argv[2])), timeout=0.5):
        raise SystemExit(0)
except OSError:
    raise SystemExit(1)
PY
      then
        printf '정책 서버 포트는 열려 있지만 Ready 응답이 없습니다. 중복 기동하지 않습니다: %s\n' "${POLICY_SERVER_ADDRESS}" >&2
      else
        POLICY_SERVER_LOG="${LOG_DIR}/policy-server-${SESSION_ID}.log"
        printf 'ACT 정책 서버 시작: %s (로그: %s)\n' "${POLICY_SERVER_ADDRESS}" "${POLICY_SERVER_LOG}"
        # LeRobot async inference 정책 서버. 기본 30 fps, 추론 지연 0.033초(= 1/30초, 한 프레임),
        # 관측 대기열 시간 제한 1초. 작업 디렉터리를 LeRobot 폴더로 바꾸되 이 스크립트에는 영향이 없게 서브셸에서 cd한다.
        (
          cd "${LEROBOT_DIR}"
          exec setsid bash -c 'trap - INT; exec "$@"' _ "${LEROBOT_PYTHON}" \
            -m lerobot.async_inference.policy_server \
            --host="${POLICY_SERVER_HOST}" --port="${POLICY_SERVER_PORT}" \
            --fps="${CONTROL_STATION_POLICY_FPS:-30}" \
            --inference_latency="${CONTROL_STATION_POLICY_INFERENCE_LATENCY:-0.033}" \
            --obs_queue_timeout="${CONTROL_STATION_POLICY_OBS_QUEUE_TIMEOUT:-1}"
        ) >>"${POLICY_SERVER_LOG}" 2>&1 &
        POLICY_SERVER_PID=$!
        # 모델을 불러오는 시간을 고려해 최대 20초 Ready를 기다린다. 프로세스가 먼저 죽으면 바로 멈춘다.
        for _ in $(seq 1 20); do
          if policy_server_ready; then break; fi
          if ! kill -0 "${POLICY_SERVER_PID}" 2>/dev/null; then break; fi
          sleep 1
        done
        if ! policy_server_ready; then
          printf 'ACT 정책 서버가 Ready 응답을 주지 않았습니다. 로그: %s\n' "${POLICY_SERVER_LOG}" >&2
          exit 1
        fi
        POLICY_READY=1
      fi
    fi

    # start_worker <표시 이름> <robot_id> <worker_id> <팔로워 포트> <자세 파일> <로그 파일>
    # 1) 이미 관제에 long-poll 중인 같은 워커가 있으면 5초 안에 확인하고 재사용한다.
    # 2) 프로세스는 있는데 heartbeat가 없으면 같은 포트를 두 프로그램이 쓰지 않도록 새로 띄우지 않는다.
    # 3) 없으면 omx_lerobot_worker.py를 시작하고 최대 15초 동안 관제 등록을 기다린다.
    # 서보 속도 프로파일 기본값 2/1은 실물 확인 뒤 정한 저속 값이다. --policy-rest-ticks 150은 정책 실행 후
    # 다섯 관절이 home에서 이 틱 수 이내로 돌아와 안정되면 정책을 멈추는 기준이다.
    start_worker() {
      local side="$1" robot_id="$2" worker_id="$3" port="$4" poses="$5" log_file="$6"
      for _ in $(seq 1 5); do
        if worker_registered "${robot_id}" "${worker_id}"; then
          printf '%s 워커 재사용: %s\n' "${side}" "${worker_id}"
          return
        fi
        sleep 1
      done
      if worker_process_running "${robot_id}"; then
        printf '%s 워커 프로세스는 있지만 관제 heartbeat가 없습니다. 중복 실행은 피합니다. 로그를 확인하세요: %s\n' \
          "${side}" "${LOG_DIR}"
        return
      fi
      printf '%s 팔 워커 시작: %s (로그: %s)\n' "${side}" "${worker_id}" "${log_file}"
      setsid bash -c 'trap - INT; exec "$@"' _ "${LEROBOT_PYTHON}" \
        "${REPO_DIR}/scripts/omx_lerobot_worker.py" \
        --server "${CONTROL_STATION_URL}" --robot-id "${robot_id}" --worker-id "${worker_id}" \
        --port "${port}" --poses "${REPO_DIR}/${poses}" \
        --policy-server "${POLICY_SERVER_ADDRESS}" \
        --policy-rest-ticks "${CONTROL_STATION_POLICY_REST_TICKS:-150}" \
        --profile-velocity "${CONTROL_STATION_OMX_PROFILE_VELOCITY:-2}" \
        --profile-acceleration "${CONTROL_STATION_OMX_PROFILE_ACCELERATION:-1}" \
        >>"${log_file}" 2>&1 &
      local pid=$!
      STARTED_WORKER_PIDS+=("${pid}")
      for _ in $(seq 1 15); do
        if worker_registered "${robot_id}" "${worker_id}"; then
          printf '%s 워커가 관제에 연결됐습니다.\n' "${side}"
          return
        fi
        if ! kill -0 "${pid}" 2>/dev/null; then
          printf '%s 워커가 종료됐습니다. 로그: %s\n' "${side}" "${log_file}" >&2
          tail -n 40 "${log_file}" >&2 || true
          return
        fi
        sleep 1
      done
      printf '%s 워커가 아직 관제에 등록되지 않았습니다. 로그: %s\n' "${side}" "${log_file}" >&2
    }

    if [[ "${POLICY_READY}" == "1" && "${LEFT_READY}" == "1" ]]; then
      start_worker '왼쪽' arm-01 arm-left "${LEFT_PORT}" "${LEFT_POSES}" \
        "${LOG_DIR}/worker-left-${SESSION_ID}.log"
    fi
    if [[ "${POLICY_READY}" == "1" && "${RIGHT_READY}" == "1" ]]; then
      start_worker '오른쪽' arm-02 arm-right "${RIGHT_PORT}" "${RIGHT_POSES}" \
        "${LOG_DIR}/worker-right-${SESSION_ID}.log"
    fi
  fi
fi

printf '관제와 설정된 런타임이 실행 중입니다. 종료하려면 Ctrl-C를 누르세요.\n'
# 관제 프로세스가 끝날 때까지 기다린다. 끝나면(또는 Ctrl-C면) EXIT trap의 cleanup이 정리한다.
wait "${CONTROL_STATION_PID}"
