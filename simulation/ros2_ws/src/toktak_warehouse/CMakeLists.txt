# 토크탁 창고 Gazebo 월드 패키지(toktak_warehouse)의 빌드 설정이다.
# 컴파일할 코드는 없고, 월드·지도·로봇 모델·bridge 설정·launch 파일을 share/toktak_warehouse로 설치만 한다.
# simulation/run.sh가 `colcon build --packages-select toktak_warehouse`로 빌드한다. 실물 로봇과는 관계없다.
cmake_minimum_required(VERSION 3.8)
project(toktak_warehouse)

find_package(ament_cmake REQUIRED)

# launch 파일은 get_package_share_directory('toktak_warehouse')로 이 설치 경로에서 파일을 찾는다.
install(DIRECTORY config launch maps models worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
