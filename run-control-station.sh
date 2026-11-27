#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ENV_FILE="${REPO_DIR}/.control-station/control-station.env"
if [[ -f "${ENV_FILE}" ]]; then
  set -a
  # shellcheck disable=SC1090
  source "${ENV_FILE}"
  set +a
fi

SESSION_ID="$(date +%Y%m%d-%H%M%S)-$$"
LOG_DIR="${REPO_DIR}/.control-station/logs"
SESSION_LOG="${LOG_DIR}/session-${SESSION_ID}.log"
CONTROL_STATION_URL="${CONTROL_STATION_URL:-http://127.0.0.1:${CONTROL_STATION_PORT:-8000}}"
POLICY_SERVER_HOST="${CONTROL_STATION_POLICY_SERVER_HOST:-127.0.0.1}"
POLICY_SERVER_PORT="${CONTROL_STATION_POLICY_SERVER_PORT:-8080}"
POLICY_SERVER_ADDRESS="${POLICY_SERVER_HOST}:${POLICY_SERVER_PORT}"

usage() {
  printf '사용법: %s [--help]\n' "$0"
  printf '  인수 없음  핑키 관제(8765)·천장 캠 주차 launch·관제와 설정된 정책 서버·팔 워커 시작\n'
}

case "${1:-}" in
  "") ;;
  -h|--help) usage; exit 0 ;;
  *) usage >&2; exit 2 ;;
esac

# The C++ parking node and ros:<topic> cameras need ROS 2 and the cpp/ workspace interfaces.
if [[ -n "${CONTROL_STATION_PARKING_SPOT:-}" || "${CONTROL_STATION_HEAD_CAMERA:-}" == ros:* \
      || "${CONTROL_STATION_SHELF_CAMERA:-}" == ros:* ]]; then
  set +u  # ROS setup scripts read unset variables
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
exec > >(tee -a "${SESSION_LOG}") 2>&1
printf 'Session log: %s\n' "${SESSION_LOG}"

cd "${REPO_DIR}"
export PYTHONPATH="${REPO_DIR}/src${PYTHONPATH:+:${PYTHONPATH}}"
PYTHON_BIN="${CONTROL_STATION_PYTHON:-}"
if [[ -z "${PYTHON_BIN}" && -x "${REPO_DIR}/.venv/bin/python" ]]; then
  PYTHON_BIN="${REPO_DIR}/.venv/bin/python"
fi
if [[ -z "${PYTHON_BIN}" ]]; then
  PYTHON_BIN="$(command -v python3)"
fi

# bash는 `&`로 띄운 프로그램에 SIGINT 무시를 물려주고, 그 안의 `trap - INT`로는 되돌릴 수 없습니다.
# 그대로 두면 Ctrl-C 정리 때 보낸 INT를 관제·워커·정책 서버가 무시해 정리가 멈추므로 실행 직전에 기본값으로 되돌립니다.
RESTORE_INT='import os, signal, sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1], sys.argv[1:])'
CONTROL_STATION_PID=""
POLICY_SERVER_PID=""
STARTED_WORKER_PIDS=()
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
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

api_ready() {
  curl --silent --show-error --fail --max-time 1 "${CONTROL_STATION_URL}/api/health" >/dev/null 2>&1
}

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

policy_server_ready() {
  PYTHONPATH="${REPO_DIR}/scripts${PYTHONPATH:+:${PYTHONPATH}}" \
    "${LEROBOT_PYTHON}" - "${POLICY_SERVER_ADDRESS}" <<'PY'
import sys
from omx_act_policy_run import check_policy_server

raise SystemExit(0 if check_policy_server(sys.argv[1], timeout_s=0.4, ready_timeout_s=0.4) is None else 1)
PY
}

worker_process_running() {
  pgrep -f "[o]mx_lerobot_worker.py.*--robot-id $1([[:space:]]|$)" >/dev/null 2>&1
}

# 핑키 관제(8765)와 천장 캠 주차 launch도 함께 켭니다 (src/control_station/runtime_services.py, 관제 화면 사이드바와 같은 코드).
# 둘 다 따로 떠서 이 스크립트를 Ctrl-C로 끝내도 남고, 다음 실행 때 그대로 재사용합니다.
"${PYTHON_BIN}" -m control_station.runtime_services ensure || true

if api_ready; then
  printf '주문 관제는 이미 실행 중입니다: %s (핑키 관제·천장 캠 주차만 확인하고 끝냅니다)\n' "${CONTROL_STATION_URL}"
  exit 0
fi

printf '관제 서버 시작: %s\n' "${CONTROL_STATION_URL}"
setsid python3 -c "${RESTORE_INT}" "${PYTHON_BIN}" -m control_station.app &
CONTROL_STATION_PID=$!
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

LEFT_PORT="${CONTROL_STATION_LEFT_FOLLOWER_PORT:-}"
LEFT_POSES="${CONTROL_STATION_LEFT_OMX_POSES:-${CONTROL_STATION_OMX_POSES:-}}"
RIGHT_PORT="${CONTROL_STATION_RIGHT_FOLLOWER_PORT:-}"
RIGHT_POSES="${CONTROL_STATION_RIGHT_OMX_POSES:-}"
LEROBOT_DIR="${CONTROL_STATION_LEROBOT_DIR:-${HOME}/Desktop/lerobot}"
LEROBOT_PYTHON="${CONTROL_STATION_LEROBOT_PYTHON:-${LEROBOT_DIR}/venv/bin/python}"
POLICY_SERVER_LOG="${LOG_DIR}/policy-server-${SESSION_ID}.log"

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

if [[ "${CONTROL_STATION_AUTO_START_ARM_RUNTIME:-1}" != "0" && ( "${LEFT_READY}" == "1" || "${RIGHT_READY}" == "1" ) ]]; then
  if [[ ! -x "${LEROBOT_PYTHON}" ]]; then
    printf 'LeRobot Python을 찾지 못해 ACT 서버와 팔 워커를 시작하지 않았습니다: %s\n' "${LEROBOT_PYTHON}" >&2
  else
    POLICY_READY=0
    if policy_server_ready; then
      POLICY_READY=1
      printf 'ACT 정책 서버 재사용: %s\n' "${POLICY_SERVER_ADDRESS}"
    else
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
        (
          cd "${LEROBOT_DIR}"
          exec setsid python3 -c "${RESTORE_INT}" "${LEROBOT_PYTHON}" \
            -m lerobot.async_inference.policy_server \
            --host="${POLICY_SERVER_HOST}" --port="${POLICY_SERVER_PORT}" \
            --fps="${CONTROL_STATION_POLICY_FPS:-30}" \
            --inference_latency="${CONTROL_STATION_POLICY_INFERENCE_LATENCY:-0.033}" \
            --obs_queue_timeout="${CONTROL_STATION_POLICY_OBS_QUEUE_TIMEOUT:-1}"
        ) >>"${POLICY_SERVER_LOG}" 2>&1 &
        POLICY_SERVER_PID=$!
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
      setsid python3 -c "${RESTORE_INT}" "${LEROBOT_PYTHON}" \
        "${REPO_DIR}/scripts/omx_lerobot_worker.py" \
        --server "${CONTROL_STATION_URL}" --robot-id "${robot_id}" --worker-id "${worker_id}" \
        --port "${port}" --poses "${REPO_DIR}/${poses}" \
        --policy-server "${POLICY_SERVER_ADDRESS}" \
        --policy-rest-ticks "${CONTROL_STATION_POLICY_REST_TICKS:-150}" \
        --profile-velocity "${CONTROL_STATION_OMX_PROFILE_VELOCITY:-2}" \
        --profile-acceleration "${CONTROL_STATION_OMX_PROFILE_ACCELERATION:-1}" \
        --tolerance "${CONTROL_STATION_OMX_TOLERANCE:-15}" \
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
wait "${CONTROL_STATION_PID}"
