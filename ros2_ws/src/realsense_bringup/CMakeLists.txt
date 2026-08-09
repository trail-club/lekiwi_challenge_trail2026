cmake_minimum_required(VERSION 3.10)
project(realsense_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/realsense_camera_watchdog.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
