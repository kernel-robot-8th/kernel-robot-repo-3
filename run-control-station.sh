#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ARM_MODE=0
SESSION_ID="$(date +%Y%m%d-%H%M%S)-$$"
LOG_DIR="${REPO_DIR}/.control-station/logs"
SESSION_LOG="${LOG_DIR}/session-${SESSION_ID}.log"

usage() {
  printf '사용법: %s [--arm]\n' "$0"
  printf '  인수 없음  팔 제어를 끈 관제 대시보드 시작\n'
  printf '  --arm      설정된 LeRobot 프로세스 제어 활성화 (MuJoCo 관측 모드에서는 사용할 수 없음)\n'
}

case "${1:-}" in
  "") ;;
  --arm) ARM_MODE=1 ;;
  -h|--help) usage; exit 0 ;;
  *) usage >&2; exit 2 ;;
esac

if [[ "${CONTROL_STATION_MODE:-mock}" == "mujoco" && ${ARM_MODE} -eq 1 ]]; then
  printf 'MuJoCo 관측 모드에서는 --arm을 사용할 수 없습니다.\n' >&2
  exit 2
fi

mkdir -p "${LOG_DIR}"
export CONTROL_STATION_ARM_LOG="${CONTROL_STATION_ARM_LOG:-${LOG_DIR}/arm-${SESSION_ID}.log}"
exec > >(tee -a "${SESSION_LOG}") 2>&1
printf 'Session log: %s\n' "${SESSION_LOG}"

if (( ARM_MODE )); then
  TELEOP_BIN="${CONTROL_STATION_ARM_PROGRAM:-}"
  if [[ -z "${TELEOP_BIN}" ]]; then
    TELEOP_BIN="$(command -v lerobot-teleoperate || true)"
  fi
  if [[ -z "${TELEOP_BIN}" && -x "${HOME}/Desktop/lerobot/venv/bin/lerobot-teleoperate" ]]; then
    TELEOP_BIN="${HOME}/Desktop/lerobot/venv/bin/lerobot-teleoperate"
  fi
  if [[ -z "${TELEOP_BIN}" || ! -x "${TELEOP_BIN}" ]]; then
    printf 'lerobot-teleoperate was not found. Activate its environment first,\n' >&2
    printf 'or set CONTROL_STATION_ARM_PROGRAM to its executable path.\n' >&2
    exit 1
  fi
  export CONTROL_STATION_ARM_ENABLED=1
  export CONTROL_STATION_ARM_PROGRAM="${TELEOP_BIN}"
else
  export CONTROL_STATION_ARM_ENABLED=0
fi

cd "${REPO_DIR}"
export PYTHONPATH="${REPO_DIR}/src${PYTHONPATH:+:${PYTHONPATH}}"
PYTHON_BIN="${CONTROL_STATION_PYTHON:-}"
if [[ -z "${PYTHON_BIN}" && -x "${REPO_DIR}/.venv/bin/python" ]]; then
  PYTHON_BIN="${REPO_DIR}/.venv/bin/python"
fi
if [[ -z "${PYTHON_BIN}" ]]; then
  PYTHON_BIN="$(command -v python3)"
fi
exec "${PYTHON_BIN}" -m control_station.app
