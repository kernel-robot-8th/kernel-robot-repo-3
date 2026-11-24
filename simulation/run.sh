#!/usr/bin/env bash
# 토크탁 창고 Gazebo Harmonic 월드를 빌드하고 띄운다. simulation/ 폴더에서 실행한다.
#
#   ./run.sh                     # 빌드 후 Gazebo 창을 연다
#   ./run.sh headless:=True      # GUI 없이 ROS/Gazebo 서버만 실행
#   ROS_DOMAIN_ID=42 ./run.sh    # 다른 ROS domain 사용
#
# 호스트에 ROS 2 Jazzy, Gazebo Harmonic, ros_gz가 설치되어 있어야 한다. 다시 실행하면 바뀐 부분만 다시 빌드한다.
# 인자는 그대로 ros2 launch에 넘긴다. 시뮬레이터 안의 가상 로봇만 움직이며 실물 핑키와는 통신하지 않는다
# (다른 domain을 쓰면 같은 네트워크의 실물 ROS 노드와 토픽이 섞이지 않는다).
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 기본 domain 31: 기존 Humble 컨테이너와 토픽이 섞이지 않게 하려는 값이다. 관제도 붙이려면 같은 값을 export한다.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-31}"
# ROS setup 스크립트는 정의되지 않은 변수를 참조하므로, source하는 동안만 set -u를 끈다.
set +u
source /opt/ros/jazzy/setup.bash
set -u
cd "${repo_dir}/ros2_ws"

# --symlink-install: 설치 폴더가 소스 파일을 가리키게 해서, 월드·launch를 고쳐도 다시 복사하지 않아도 된다.
colcon build --symlink-install --packages-select toktak_warehouse
set +u
source install/setup.bash
set -u
ros2 launch toktak_warehouse toktak_warehouse.launch.py "$@"
