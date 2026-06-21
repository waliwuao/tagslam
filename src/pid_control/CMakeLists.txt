cmake_minimum_required(VERSION 3.22)
project(pid_control)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.py")

install(DIRECTORY
  ../../config
  DESTINATION share/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "pid_control.yaml")

install(PROGRAMS
  pid_control/pid_node.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
