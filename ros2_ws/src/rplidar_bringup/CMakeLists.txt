cmake_minimum_required(VERSION 3.10)
project(rplidar_bringup)

find_package(ament_cmake REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(scan_to_cloud src/scan_to_cloud.cpp)
ament_target_dependencies(scan_to_cloud laser_geometry rclcpp sensor_msgs)

install(TARGETS scan_to_cloud DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})
ament_package()
