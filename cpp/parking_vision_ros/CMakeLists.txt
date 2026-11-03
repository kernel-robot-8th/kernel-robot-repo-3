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
# Loadable into a component container next to the camera driver (intra-process images), or run alone as
# parking_vision_node. The check service answers from its own thread, so a single-threaded executor is enough.
rclcpp_components_register_node(parking_vision_component
  PLUGIN "parking_vision_ros::ParkingVisionNode"
  EXECUTABLE parking_vision_node)
# The ceiling camera capture that feeds it (replaces usb_cam, see ceiling_camera_node.hpp).
rclcpp_components_register_node(parking_vision_component
  PLUGIN "parking_vision_ros::CeilingCameraNode"
  EXECUTABLE ceiling_camera_node)

install(TARGETS parking_vision_component
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Keep test discovery on this host so test topics and services never reach robots on the network.
  ament_add_gtest(test_parking_vision_node test/test_parking_vision_node.cpp TIMEOUT 120
    ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_DOMAIN_ID=87)
  target_link_libraries(test_parking_vision_node parking_vision_component)
  target_compile_definitions(test_parking_vision_node PRIVATE
    PARKING_VISION_FIXTURE_DIR="${parking_vision_FIXTURE_DIR}")
  ament_add_gtest(test_ceiling_camera_node test/test_ceiling_camera_node.cpp TIMEOUT 120
    ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_DOMAIN_ID=88)
  target_link_libraries(test_ceiling_camera_node parking_vision_component)
  target_compile_definitions(test_ceiling_camera_node PRIVATE
    PARKING_VISION_FIXTURE_DIR="${parking_vision_FIXTURE_DIR}")
endif()

ament_package()
