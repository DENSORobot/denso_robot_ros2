cmake_minimum_required(VERSION 3.8)
project(denso_robot_descriptions)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf robots
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
