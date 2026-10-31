cmake_minimum_required(VERSION 3.8)
project(omx_mujoco_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../../../mujoco_omx/robotis_omx/"
  DESTINATION share/${PROJECT_NAME}/models/robotis_omx
)

ament_package()
