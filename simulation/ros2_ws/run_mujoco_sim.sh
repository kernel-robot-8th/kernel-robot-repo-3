#!/usr/bin/env bash

set -euo pipefail

check_only=0
point_cloud=0
for argument in "$@"; do
  case "${argument}" in
    --check-only)
      if (( check_only )); then
        echo "--check-only may be specified once." >&2
        exit 2
      fi
      check_only=1
      ;;
    --point-cloud)
      if (( point_cloud )); then
        echo "--point-cloud may be specified once." >&2
        exit 2
      fi
      point_cloud=1
      ;;
    *)
      echo "Usage: $0 [--check-only] [--point-cloud]" >&2
      exit 2
      ;;
  esac
done

if [[ -z "${ROS_DOMAIN_ID:-}" ]]; then
  echo "Set ROS_DOMAIN_ID to a domain confirmed separate from physical robots and other simulators." >&2
  exit 2
fi
if [[ ! "${ROS_DOMAIN_ID}" =~ ^[0-9]+$ ]]; then
  echo "ROS_DOMAIN_ID must be a decimal integer." >&2
  exit 2
fi
if [[ ${#ROS_DOMAIN_ID} -gt 9 ]] || (( 10#${ROS_DOMAIN_ID} < 1 || 10#${ROS_DOMAIN_ID} > 232 )); then
  echo "ROS_DOMAIN_ID must be between 1 and 232 for this simulation wrapper." >&2
  exit 2
fi

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "${script_dir}/../.." && pwd)"
ros_setup="/opt/ros/jazzy/setup.bash"
workspace_setup="${script_dir}/install/setup.bash"

if [[ ! -f "${ros_setup}" ]]; then
  echo "ROS 2 Jazzy setup was not found at ${ros_setup}." >&2
  exit 2
fi
if [[ ! -f "${workspace_setup}" ]]; then
  echo "Build simulation/ros2_ws first; missing ${workspace_setup}." >&2
  exit 2
fi

# ROS setup templates read optional environment variables that may be unset.
set +u
source "${ros_setup}"
source "${workspace_setup}"
set -u

required_packages=(
  control_msgs
  controller_manager
  launch
  launch_ros
  joint_state_broadcaster
  joint_trajectory_controller
  mujoco_ros2_control
  mujoco_ros2_control_plugins
  omx_mujoco_sim
  robot_state_publisher
  tf2_ros
  xacro
)
missing_packages=""
for package in "${required_packages[@]}"; do
  if ! ros2 pkg prefix "${package}" >/dev/null 2>&1; then
    missing_packages+="${missing_packages:+ }${package}"
  fi
done
if [[ -n "${missing_packages}" ]]; then
  printf 'Required ROS 2 packages are unavailable: %s\n' "${missing_packages}" >&2
  exit 2
fi
if (( point_cloud )) && ! ros2 pkg prefix depth_image_proc >/dev/null 2>&1; then
  echo "--point-cloud requires the ROS 2 package depth_image_proc." >&2
  exit 2
fi

cd "${repo_root}"
echo "MuJoCo OMX package preflight passed on ROS_DOMAIN_ID=${ROS_DOMAIN_ID}."
echo "Package presence does not verify DDS isolation, plugin loading, or controller activation."
if (( point_cloud )); then
  echo "Optional depth_image_proc XYZ point-cloud conversion is enabled."
fi
if (( check_only )); then
  echo "Check-only requested; no ROS nodes were launched."
  exit 0
fi
echo "Starting the MuJoCo-only OMX launch on ROS_DOMAIN_ID=${ROS_DOMAIN_ID}."
echo "Confirm this domain is isolated from physical robots and other simulators; this script cannot verify isolation."
launch_arguments=()
if (( point_cloud )); then
  launch_arguments+=("enable_point_cloud:=true")
fi
exec ros2 launch omx_mujoco_sim omx_mujoco_control.launch.py "${launch_arguments[@]}"
