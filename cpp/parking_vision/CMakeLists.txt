# parking_vision 패키지의 빌드 설정이다. ROS 의존성이 없는 C++17 주차 판정 라이브러리(정적 라이브러리),
# 등록·판정 CLI pinky_parking, 측정 도구 parking_vision_benchmark, gtest 테스트를 만든다.
# ament_cmake로 묶는 이유는 parking_vision_ros가 find_package로 가져다 쓰고 `ros2 run`으로 CLI를 실행하기 위해서다.
cmake_minimum_required(VERSION 3.16)
project(parking_vision LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Ubuntu 24.04 / ROS 2 Jazzy의 시스템 패키지: libopencv-dev(4.7 미만이면 libopencv-contrib-dev도),
# 주차 위치 JSON용 nlohmann-json3-dev, 내부 보정 파일 SHA-256 확인용 libssl-dev.
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(nlohmann_json 3.2 REQUIRED)
find_package(OpenSSL REQUIRED)

# ArUco는 OpenCV 4.7부터 본체 objdetect 모듈로 들어왔고, 그 전에는 contrib의 aruco 모듈에 있다.
# 소스 쪽도 CV_VERSION으로 같은 분기를 한다(src/parking_check.cpp).
if(OpenCV_VERSION VERSION_LESS 4.7)
  set(PARKING_VISION_ARUCO_LIB opencv_aruco)
else()
  set(PARKING_VISION_ARUCO_LIB opencv_objdetect)
endif()

# STATIC/SHARED를 지정하지 않았으므로 BUILD_SHARED_LIBS를 켜지 않는 한 정적 라이브러리가 된다. 공유 라이브러리인
# parking_vision_component에 링크되므로 위치 독립 코드(PIC)로 컴파일한다.
add_library(parking_vision src/parking_check.cpp src/parking_observation.cpp src/parking_tracker.cpp src/registration.cpp)
add_library(parking_vision::parking_vision ALIAS parking_vision)
set_target_properties(parking_vision PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(parking_vision PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(parking_vision
  PUBLIC opencv_core nlohmann_json::nlohmann_json
  PRIVATE opencv_imgproc opencv_calib3d ${PARKING_VISION_ARUCO_LIB} OpenSSL::Crypto)
target_compile_options(parking_vision PRIVATE -Wall -Wextra -Wpedantic)

# parking_vision::parking_vision으로 내보낸다. 기대값(fixtures)은 parking_vision_FIXTURE_DIR
# (cmake/parking_vision-extras.cmake)를 통해 parking_vision_ros 테스트와 함께 쓴다.
install(TARGETS parking_vision EXPORT export_parking_vision ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY test/fixtures/ DESTINATION share/${PROJECT_NAME}/fixtures)
ament_export_targets(export_parking_vision HAS_LIBRARY_TARGET)
# 라이브러리가 정적이라 PRIVATE 의존성도 이 라이브러리를 쓰는 쪽에서 링크해야 한다.
ament_export_dependencies(OpenCV nlohmann_json OpenSSL)

# 주차 위치 등록·판정 CLI(예전 scripts/pinky_parking.py를 대신한다): ros2 run parking_vision pinky_parking
add_executable(pinky_parking tools/pinky_parking.cpp)
target_link_libraries(pinky_parking PRIVATE parking_vision opencv_imgproc opencv_videoio opencv_highgui)
target_compile_options(pinky_parking PRIVATE -Wall -Wextra -Wpedantic)
install(TARGETS pinky_parking RUNTIME DESTINATION lib/${PROJECT_NAME})

# observe_frame 한 번에 걸리는 시간을 재는 도구다(cpp/README.md의 성능 측정). 설치하지 않고 build 폴더에서 실행한다.
add_executable(parking_vision_benchmark tools/benchmark_observe_frame.cpp)
target_link_libraries(parking_vision_benchmark PRIVATE parking_vision opencv_imgproc opencv_imgcodecs)

include(CTest)
if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  include(GoogleTest)
  # 테스트 실행 파일 하나에 모든 테스트를 넣는다. 기대값 폴더 경로와, CLI 테스트가 실행할 pinky_parking 경로를
  # 컴파일 정의로 넘긴다. CLI 테스트가 먼저 빌드된 pinky_parking을 쓰도록 의존성을 건다.
  add_executable(parking_vision_tests test/test_parking_check.cpp test/test_parking_tracker.cpp test/test_python_parity.cpp
    test/test_pinky_parking_cli.cpp)
  target_link_libraries(parking_vision_tests PRIVATE
    parking_vision opencv_imgproc opencv_imgcodecs opencv_calib3d opencv_videoio ${PARKING_VISION_ARUCO_LIB}
    nlohmann_json::nlohmann_json GTest::gtest_main)
  target_compile_definitions(parking_vision_tests PRIVATE
    PARKING_VISION_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures"
    PINKY_PARKING_CLI="$<TARGET_FILE:pinky_parking>")
  add_dependencies(parking_vision_tests pinky_parking)
  gtest_discover_tests(parking_vision_tests)
endif()

# CONFIG_EXTRAS: find_package(parking_vision)을 한 쪽에서 parking_vision_FIXTURE_DIR을 정의한다.
ament_package(CONFIG_EXTRAS cmake/parking_vision-extras.cmake)
