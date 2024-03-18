cmake_minimum_required(VERSION 3.8)
project(denso_robot_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config rviz srdf robots
  DESTINATION share/${PROJECT_NAME})

ament_package()
