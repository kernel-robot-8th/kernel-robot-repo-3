cmake_minimum_required(VERSION 3.8)
project(toktak_warehouse)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch maps models worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
