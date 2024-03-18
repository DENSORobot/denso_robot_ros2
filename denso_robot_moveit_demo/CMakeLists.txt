cmake_minimum_required(VERSION 3.8)
project(denso_robot_moveit_demo)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
#moveit_package()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_common REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

# This is the general demo
add_executable(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
ament_target_dependencies(
  ${PROJECT_NAME}
  moveit_ros_planning_interface)

# This is the pick-and-place demo with cartesian target poses (6 DOFs robot arms)
add_executable(
  denso_robot_moveit_pickandplace
  src/denso_robot_moveit_pickandplace.cpp)
ament_target_dependencies(
  denso_robot_moveit_pickandplace
  moveit_ros_planning_interface)

# This is the pick-and-place demo with joint target positions (4 DOFs robot arms)
add_executable(
  denso_robot_moveit_pickandplace_scara
  src/denso_robot_moveit_pickandplace_scara.cpp)
ament_target_dependencies(
  denso_robot_moveit_pickandplace_scara
  moveit_ros_planning_interface)

#=============
# Install
#=============

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include)

install(
  TARGETS denso_robot_moveit_pickandplace
  EXPORT export_denso_robot_moveit_pickandplace
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include)

install(
  TARGETS denso_robot_moveit_pickandplace_scara
  EXPORT export_denso_robot_moveit_pickandplace_scara
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include)

ament_export_targets(
  export_${PROJECT_NAME}
  export_denso_robot_moveit_pickandplace
  export_denso_robot_moveit_pickandplace_scara)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
