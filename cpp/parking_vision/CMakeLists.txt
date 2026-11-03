cmake_minimum_required(VERSION 3.16)
project(parking_vision LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# System packages on Ubuntu 24.04 / ROS 2 Jazzy: libopencv-dev (+ libopencv-contrib-dev below 4.7),
# nlohmann-json3-dev for the spot JSON, libssl-dev for the calibration file SHA-256 check.
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(nlohmann_json 3.2 REQUIRED)
find_package(OpenSSL REQUIRED)

if(OpenCV_VERSION VERSION_LESS 4.7)
  set(PARKING_VISION_ARUCO_LIB opencv_aruco)
else()
  set(PARKING_VISION_ARUCO_LIB opencv_objdetect)
endif()

add_library(parking_vision src/parking_check.cpp src/parking_observation.cpp src/parking_tracker.cpp)
add_library(parking_vision::parking_vision ALIAS parking_vision)
set_target_properties(parking_vision PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(parking_vision PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(parking_vision
  PUBLIC opencv_core
  PRIVATE opencv_imgproc opencv_calib3d ${PARKING_VISION_ARUCO_LIB} nlohmann_json::nlohmann_json OpenSSL::Crypto)
target_compile_options(parking_vision PRIVATE -Wall -Wextra -Wpedantic)

# Exported as parking_vision::parking_vision; the fixtures are shared with parking_vision_ros tests through
# parking_vision_FIXTURE_DIR (cmake/parking_vision-extras.cmake).
install(TARGETS parking_vision EXPORT export_parking_vision ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY test/fixtures/ DESTINATION share/${PROJECT_NAME}/fixtures)
ament_export_targets(export_parking_vision HAS_LIBRARY_TARGET)
# The library is static, so its private dependencies are linked by the consumer.
ament_export_dependencies(OpenCV nlohmann_json OpenSSL)

add_executable(parking_vision_benchmark tools/benchmark_observe_frame.cpp)
target_link_libraries(parking_vision_benchmark PRIVATE parking_vision opencv_imgproc opencv_imgcodecs)

include(CTest)
if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  include(GoogleTest)
  add_executable(parking_vision_tests test/test_parking_check.cpp test/test_parking_tracker.cpp test/test_python_parity.cpp)
  target_link_libraries(parking_vision_tests PRIVATE
    parking_vision opencv_imgproc opencv_imgcodecs opencv_calib3d ${PARKING_VISION_ARUCO_LIB}
    nlohmann_json::nlohmann_json GTest::gtest_main)
  target_compile_definitions(parking_vision_tests PRIVATE
    PARKING_VISION_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures")
  gtest_discover_tests(parking_vision_tests)
endif()

ament_package(CONFIG_EXTRAS cmake/parking_vision-extras.cmake)
