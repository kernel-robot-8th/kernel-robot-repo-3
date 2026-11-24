# 주차 판정 ROS 2 인터페이스 패키지(parking_vision_interfaces)의 빌드 설정이다.
# 코드는 없고 rosidl로 메시지·서비스 타입만 만든다.
# - ParkingState.msg: /pinky_parking/state로 10 Hz 발행하는 화면 표시용 추적 상태
# - CheckParking.srv: /pinky_parking/check 일회성 판정 요청·응답
# 필드는 관제 JSON 계약(/live, /check의 judgment_fields·image_overlay)과 1:1이며,
# JSON에서 null인 값은 has_* 플래그로 표시한다.
cmake_minimum_required(VERSION 3.16)
project(parking_vision_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/ParkingState.msg
  srv/CheckParking.srv
  DEPENDENCIES builtin_interfaces std_msgs)

# 이 패키지를 쓰는 쪽이 생성된 타입을 실행 시점에 찾을 수 있도록 런타임 의존성을 내보낸다.
ament_export_dependencies(rosidl_default_runtime)
ament_package()
