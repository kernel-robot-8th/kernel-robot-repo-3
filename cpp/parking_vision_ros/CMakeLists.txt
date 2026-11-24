# parking_vision_ros 패키지의 빌드 설정이다. ROS 의존성 없는 parking_vision 라이브러리를 rclcpp 컴포넌트 두 개로 감싼다.
# - parking_vision_ros::ParkingVisionNode: 영상 토픽을 받아 /pinky_parking/state 발행, /pinky_parking/check 서비스
# - parking_vision_ros::CeilingCameraNode: 천장 카메라를 V4L2로 직접 열어 image_raw와 image_raw/compressed 발행
# 두 컴포넌트는 공유 라이브러리 하나(parking_vision_component)에 들어 있고, 각각 단독 실행 파일로도 등록한다.
cmake_minimum_required(VERSION 3.16)
project(parking_vision_ros LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(parking_vision REQUIRED)
find_package(parking_vision_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS videoio imgcodecs)

add_library(parking_vision_component SHARED src/parking_vision_node.cpp src/ceiling_camera_node.cpp)
target_include_directories(parking_vision_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(parking_vision_component PUBLIC
  parking_vision::parking_vision
  ${parking_vision_interfaces_TARGETS}
  rclcpp::rclcpp
  rclcpp_components::component
  ${sensor_msgs_TARGETS}
  cv_bridge::cv_bridge
  opencv_videoio
  opencv_imgcodecs)
target_compile_options(parking_vision_component PRIVATE -Wall -Wextra -Wpedantic)
# 카메라 드라이버와 같은 컴포넌트 컨테이너에 올려 영상을 프로세스 안에서 받거나(intra-process),
# parking_vision_node로 단독 실행한다. 판정 서비스는 자기 스레드에서 답하므로 단일 스레드 executor로 충분하다.
rclcpp_components_register_node(parking_vision_component
  PLUGIN "parking_vision_ros::ParkingVisionNode"
  EXECUTABLE parking_vision_node)
# 판정 노드에 영상을 대는 천장 카메라 캡처다(usb_cam을 대신한다. ceiling_camera_node.hpp 참고).
rclcpp_components_register_node(parking_vision_component
  PLUGIN "parking_vision_ros::CeilingCameraNode"
  EXECUTABLE ceiling_camera_node)

install(TARGETS parking_vision_component
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
# launch 파일과 설정을 share에 설치해서 `ros2 launch parking_vision_ros ...`로 찾을 수 있게 한다.
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # 테스트의 discovery를 이 호스트로 한정해서 테스트 토픽과 서비스가 네트워크의 로봇에 닿지 않게 하고,
  # runbook이 실물 판정 노드와 관제를 띄우는 domain 87도 피한다. 테스트 노드도 같은 이름
  # /pinky_parking/state와 /pinky_parking/check를 쓰기 때문이다. 두 테스트는 서로 다른 domain(97, 98)을 쓴다.
  # parking_vision_FIXTURE_DIR은 find_package(parking_vision)이 정의한 설치된 기대값 폴더다.
  ament_add_gtest(test_parking_vision_node test/test_parking_vision_node.cpp TIMEOUT 120
    ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_DOMAIN_ID=97)
  target_link_libraries(test_parking_vision_node parking_vision_component)
  target_compile_definitions(test_parking_vision_node PRIVATE
    PARKING_VISION_FIXTURE_DIR="${parking_vision_FIXTURE_DIR}")
  ament_add_gtest(test_ceiling_camera_node test/test_ceiling_camera_node.cpp TIMEOUT 120
    ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_DOMAIN_ID=98)
  target_link_libraries(test_ceiling_camera_node parking_vision_component)
  target_compile_definitions(test_ceiling_camera_node PRIVATE
    PARKING_VISION_FIXTURE_DIR="${parking_vision_FIXTURE_DIR}")
endif()

ament_package()
