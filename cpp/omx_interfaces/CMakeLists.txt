# OMX 로봇팔 action 정의 패키지(omx_interfaces)의 빌드 설정이다.
# 코드는 없고 rosidl로 action 타입만 만든다.
# - MoveToSlot.action: OMX 워커 scripts/omx_ros_worker.py가 /omx/move_to_slot action 서버로 쓴다(관제는 쓰지 않는다).
# - PickObject.action: 팔마다 scripts/omx_pick_action_server.py가 /arm_0N/pick_object 서버로 쓰고,
#   관제는 CONTROL_STATION_ORDER_ARM_TRANSPORT=ros2일 때 주문의 집기를 이 action으로 보낸다.
cmake_minimum_required(VERSION 3.16)
project(omx_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  action/MoveToSlot.action
  action/PickObject.action
  DEPENDENCIES builtin_interfaces)

# 이 패키지를 쓰는 쪽이 생성된 타입을 실행 시점에 찾을 수 있도록 런타임 의존성을 내보낸다.
ament_export_dependencies(rosidl_default_runtime)
ament_package()
