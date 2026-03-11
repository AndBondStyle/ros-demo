cmake_minimum_required(VERSION 3.8)
project(simdemo)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(controller_manager REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(velocity_controllers REQUIRED)
find_package(joint_state_broadcaster REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/control.py DESTINATION lib/${PROJECT_NAME})

ament_package()
