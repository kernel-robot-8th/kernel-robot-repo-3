#!/usr/bin/env bash
# 작업 시작 전 개발 환경 확인. 기본은 읽기만 하고, --install / --vision을 줄 때만 설치한다.
set -u

REPO_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_PY="${REPO_DIR}/.venv/bin/python"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
TEAM_MODEL_COMMIT="2413f4c"
TEAM_MODEL_PATH="yolo/models/best11.pt"
TEAM_MODEL_LOCAL="${REPO_DIR}/.control-station/models/best11_${TEAM_MODEL_COMMIT}.pt"

INSTALL=0
VISION=0
DRY_RUN=0

usage() {
  cat <<'EOF'
사용법: scripts/check_setup.sh [--install] [--vision] [--dry-run]

  (옵션 없음)  설치 상태만 확인하고 빠진 항목의 해결 명령을 보여 줍니다.
  --install    빠진 apt 패키지(sudo 비밀번호 필요)와 관제 .venv 의존성을 설치합니다.
  --vision     YOLO(ultralytics, PyTorch)를 .venv에 설치하고 팀 모델 파일을 준비합니다. 용량이 큽니다.
  --dry-run    --install / --vision이 실행할 명령만 출력합니다.

확인 결과: ✓ 준비됨 · ✗ 필수 항목 없음 · △ 선택 항목 없음(그 기능을 쓸 때만 필요)
필수 항목이 하나라도 없으면 종료 코드 1을 냅니다.
EOF
}

for arg in "$@"; do
  case "$arg" in
    --install) INSTALL=1 ;;
    --vision) VISION=1 ;;
    --dry-run) DRY_RUN=1 ;;
    -h|--help) usage; exit 0 ;;
    *) usage >&2; exit 2 ;;
  esac
done

if [[ -t 1 ]]; then GREEN=$'\e[32m'; RED=$'\e[31m'; YELLOW=$'\e[33m'; BOLD=$'\e[1m'; RESET=$'\e[0m'
else GREEN=""; RED=""; YELLOW=""; BOLD=""; RESET=""; fi

MISSING_REQUIRED=0
MISSING_APT=()

section() { printf '\n%s[%s]%s\n' "$BOLD" "$1" "$RESET"; }
ok() { printf '  %s✓%s %s\n' "$GREEN" "$RESET" "$1"; }
fail() { printf '  %s✗%s %s\n' "$RED" "$RESET" "$1"; [[ -n "${2:-}" ]] && printf '      → %s\n' "$2"; MISSING_REQUIRED=$((MISSING_REQUIRED + 1)); }
warn() { printf '  %s△%s %s\n' "$YELLOW" "$RESET" "$1"; [[ -n "${2:-}" ]] && printf '      → %s\n' "$2"; }

apt_installed() { dpkg-query -W -f='${Status}' "$1" 2>/dev/null | grep -q "install ok installed"; }

# check_apt <패키지> <설명> <required|optional>
check_apt() {
  if apt_installed "$1"; then ok "$2 ($1)"; return; fi
  MISSING_APT+=("$1")
  if [[ "$3" == required ]]; then fail "$2 ($1)" "sudo apt install $1"; else warn "$2 ($1)" "sudo apt install $1"; fi
}

venv_py() { [[ -x "$VENV_PY" ]] && "$VENV_PY" "$@"; }

run() {
  printf '  $ %s\n' "$*"
  [[ $DRY_RUN -eq 1 ]] && return 0
  "$@"
}

# ---------------------------------------------------------------- 확인
section "관제 서버 (필수)"
if command -v python3 >/dev/null && python3 -c 'import sys; sys.exit(sys.version_info < (3, 10))'; then
  ok "Python $(python3 -c 'import platform; print(platform.python_version())')"
else
  fail "Python 3.10 이상" "Ubuntu 24.04 기본 python3 사용"
fi
if command -v uv >/dev/null || [[ -x "$HOME/.local/bin/uv" ]]; then
  ok "uv (가상환경·패키지 설치)"
elif python3 -c 'import ensurepip' 2>/dev/null; then
  ok "python3-venv (가상환경 생성)"
else
  check_apt python3-venv "가상환경 생성 도구" required
fi
if [[ -x "$VENV_PY" ]]; then
  ok ".venv ($($VENV_PY -c 'import platform; print(platform.python_version())'))"
  if venv_py -c 'import sqlalchemy, alembic' 2>/dev/null; then ok "SQLAlchemy, Alembic"; else fail "SQLAlchemy, Alembic" "scripts/check_setup.sh --install"; fi
  cv_version="$(venv_py -c 'import cv2; print(cv2.__version__)' 2>/dev/null)"
  if [[ -z "$cv_version" ]]; then
    fail "OpenCV (opencv-python-headless)" "scripts/check_setup.sh --install"
  elif [[ "$cv_version" == 5.* ]]; then
    fail "OpenCV ${cv_version}: 저장소는 4.x(<5)를 씁니다" "scripts/check_setup.sh --vision (opencv-python 제거 후 headless 4.x 재설치)"
  else
    ok "OpenCV ${cv_version}"
  fi
else
  fail ".venv 없음" "scripts/check_setup.sh --install"
fi

section "카메라"
check_apt v4l-utils "카메라 목록 확인 도구(v4l2-ctl)" optional
if compgen -G "/dev/v4l/by-path/*-video-index0" >/dev/null; then
  declare -A seen_devices=()
  no_access=0
  for path in /dev/v4l/by-path/*-video-index0; do
    device="$(readlink -f "$path")"
    [[ -n "${seen_devices[$device]:-}" ]] && continue  # usb-0과 usbv2-0 이름이 같은 장치를 가리킴
    seen_devices[$device]=1
    name="$(tr -s ' ' < "/sys/class/video4linux/$(basename "$device")/name" 2>/dev/null)"
    printf '      %s → %s  (%s)\n' "$path" "$device" "${name:-?}"
    [[ -r "$device" && -w "$device" ]] || no_access=1
  done
  ok "연결된 카메라 (같은 모델 여러 대는 by-path로 구분: 런북 '천장 카메라 Pinky 주차 판정')"
  if [[ $no_access -eq 0 ]]; then ok "카메라 장치 읽기·쓰기 권한"
  else warn "열 수 없는 카메라 장치가 있음" "sudo usermod -aG video \$USER 후 다시 로그인"; fi
else
  warn "연결된 카메라 없음"
fi

section "Pinky 주차 판정"
if [[ -f "${REPO_DIR}/.control-station/camera-calibrations/ceiling_camera.json" ]]; then ok "천장 카메라 내부 보정 파일"
else warn "천장 카메라 내부 보정 파일 없음" "docs/implementation/camera-intrinsic-calibration.md"; fi
spots="$(ls "${REPO_DIR}"/.control-station/parking-spots/*.json 2>/dev/null | xargs -r -n1 basename | tr '\n' ' ')"
if [[ -n "$spots" ]]; then ok "등록된 주차 위치: ${spots}"; else warn "등록된 주차 위치 없음" "ros2 run parking_vision pinky_parking register (런북 참고)"; fi
if /usr/bin/python3 -c 'import cv2, sys; sys.exit("GTK" not in cv2.getBuildInformation() and "QT" not in cv2.getBuildInformation())' 2>/dev/null; then
  ok "창을 띄우는 OpenCV (시스템 python3, 주차 위치 등록 창)"
else
  check_apt python3-opencv "창을 띄우는 OpenCV (주차 위치 등록 창)" optional
fi

section "YOLO / GPU (선택)"
if command -v nvidia-smi >/dev/null && gpu="$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1)" && [[ -n "$gpu" ]]; then ok "GPU: ${gpu}"; else warn "NVIDIA GPU 없음 (CPU 추론은 느림)"; fi
if venv_py -c 'import ultralytics' 2>/dev/null; then
  ok "ultralytics $(venv_py -c 'import ultralytics; print(ultralytics.__version__)')"
  if venv_py -c 'import torch, sys; sys.exit(not torch.cuda.is_available())' 2>/dev/null; then ok "PyTorch CUDA 사용 가능"; else warn "PyTorch가 GPU를 쓰지 못함"; fi
  if venv_py -c 'import importlib.metadata as m; m.version("opencv-python")' 2>/dev/null; then
    warn "opencv-python이 headless와 함께 설치돼 cv2가 덮어써질 수 있음" "scripts/check_setup.sh --vision"
  fi
else
  warn "ultralytics 없음" "scripts/check_setup.sh --vision"
fi
if [[ -s "$TEAM_MODEL_LOCAL" ]]; then ok "팀 YOLO 모델 (${TEAM_MODEL_LOCAL#"${REPO_DIR}/"})"; else warn "팀 YOLO 모델 없음" "scripts/check_setup.sh --vision"; fi

section "ROS 2 Jazzy (C++ 노드·ROS 모드)"
if [[ -f "$ROS_SETUP" ]]; then ok "ROS 2 Jazzy (${ROS_SETUP})"; else warn "ROS 2 Jazzy 없음" "https://docs.ros.org/en/jazzy/Installation.html"; fi
check_apt python3-colcon-common-extensions "colcon 빌드 도구" optional
for pkg in ros-jazzy-cv-bridge ros-jazzy-image-transport ros-jazzy-compressed-image-transport ros-jazzy-vision-msgs \
           ros-jazzy-rclcpp-components ros-jazzy-usb-cam ros-jazzy-v4l2-camera ros-jazzy-camera-info-manager; do
  check_apt "$pkg" "ROS 패키지" optional
done
if venv_py -c 'import yaml' 2>/dev/null; then ok "PyYAML in .venv (rclpy용)"; else warn "PyYAML in .venv 없음 (ROS 모드에서 필요)" "scripts/check_setup.sh --install"; fi

section "C++ 빌드 (cpp/parking_vision)"
for pkg in build-essential cmake pkg-config libopencv-dev nlohmann-json3-dev libssl-dev libgtest-dev; do
  check_apt "$pkg" "C++ 빌드 의존성" optional
done

# ---------------------------------------------------------------- 설치
uv_bin() { command -v uv 2>/dev/null || { [[ -x "$HOME/.local/bin/uv" ]] && echo "$HOME/.local/bin/uv"; }; }

pip_install() {  # .venv에 패키지 설치 (uv가 있으면 uv)
  local uv; uv="$(uv_bin)"
  if [[ -n "$uv" ]]; then run "$uv" pip install --python "$VENV_PY" "$@"; else run "$VENV_PY" -m pip install "$@"; fi
}

if [[ $INSTALL -eq 1 ]]; then
  section "설치"
  if [[ ${#MISSING_APT[@]} -gt 0 ]]; then
    run sudo apt-get install -y "${MISSING_APT[@]}"
  fi
  if [[ ! -x "$VENV_PY" ]]; then
    uv="$(uv_bin)"
    if [[ -n "$uv" ]]; then run "$uv" venv "${REPO_DIR}/.venv"; else run python3 -m venv "${REPO_DIR}/.venv"; fi
  fi
  pip_install -r "${REPO_DIR}/requirements.txt" -r "${REPO_DIR}/requirements-ros.txt"
fi

if [[ $VISION -eq 1 ]]; then
  section "YOLO 설치"
  pip_install -r "${REPO_DIR}/requirements-vision.txt"
  # ultralytics가 opencv-python(5.x)을 끌고 와 cv2를 덮어쓰므로, 저장소가 쓰는 headless 4.x로 되돌린다.
  if [[ $DRY_RUN -eq 1 ]] || venv_py -c 'import importlib.metadata as m; m.version("opencv-python")' 2>/dev/null; then
    # Both wheels unpack into the same cv2/ folder, so the headless one is reinstalled after the removal.
    uv="$(uv_bin)"
    if [[ -n "$uv" ]]; then
      run "$uv" pip uninstall --python "$VENV_PY" opencv-python
      run "$uv" pip install --python "$VENV_PY" --reinstall "opencv-python-headless>=4.10,<5.0"
    else
      run "$VENV_PY" -m pip uninstall -y opencv-python
      run "$VENV_PY" -m pip install --force-reinstall "opencv-python-headless>=4.10,<5.0"
    fi
  fi
  if [[ ! -s "$TEAM_MODEL_LOCAL" ]]; then
    run git -C "$REPO_DIR" fetch origin feat/yolo-object-detection
    run mkdir -p "$(dirname "$TEAM_MODEL_LOCAL")"
    printf '  $ git show %s:%s > %s\n' "$TEAM_MODEL_COMMIT" "$TEAM_MODEL_PATH" "${TEAM_MODEL_LOCAL#"${REPO_DIR}/"}"
    if [[ $DRY_RUN -eq 0 ]]; then
      # Write to a temporary file first so a failed git show never leaves an empty model that later checks accept.
      if git -C "$REPO_DIR" show "${TEAM_MODEL_COMMIT}:${TEAM_MODEL_PATH}" > "${TEAM_MODEL_LOCAL}.tmp" && [[ -s "${TEAM_MODEL_LOCAL}.tmp" ]]; then
        mv "${TEAM_MODEL_LOCAL}.tmp" "$TEAM_MODEL_LOCAL"
      else
        rm -f "${TEAM_MODEL_LOCAL}.tmp"
        printf '  %s✗%s 팀 YOLO 모델을 받지 못했습니다 (커밋 %s)\n' "$RED" "$RESET" "$TEAM_MODEL_COMMIT"
      fi
    fi
  fi
fi

if [[ $INSTALL -eq 1 || $VISION -eq 1 ]] && [[ $DRY_RUN -eq 0 ]]; then
  # The checks above ran before installing; check again so the summary and exit code describe the result.
  printf '\n설치를 마쳤습니다. 다시 확인합니다.\n'
  exec "$0"
fi

printf '\n'
if [[ $MISSING_REQUIRED -gt 0 ]]; then
  printf '%s필수 항목 %d개가 없습니다.%s 위의 → 명령이나 scripts/check_setup.sh --install로 설치하세요.\n' "$RED" "$MISSING_REQUIRED" "$RESET"
  exit 1
fi
printf '%s필수 항목이 모두 준비됐습니다.%s △ 항목은 그 기능을 쓸 때만 설치하면 됩니다.\n' "$GREEN" "$RESET"
