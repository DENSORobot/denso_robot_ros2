cmake_minimum_required(VERSION 3.8)
project(denso_robot_core)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(bcap_core REQUIRED)
find_package(bcap_service REQUIRED)
find_package(bcap_service_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(denso_robot_core_interfaces REQUIRED)

include_directories(include)

add_library(
  ${PROJECT_NAME} SHARED
  src/denso_base.cpp
  src/denso_controller.cpp
  src/denso_controller_rc8.cpp
  src/denso_controller_rc8_cobotta.cpp
  src/denso_controller_rc9.cpp
  src/denso_robot.cpp
  src/${PROJECT_NAME}.cpp
  src/denso_robot_rc8.cpp
  src/denso_robot_rc8_cobotta.cpp
  src/denso_robot_rc9.cpp
  src/denso_task.cpp
  src/denso_variable.cpp
  src/tinyxml2.cpp)

add_executable(
  ${PROJECT_NAME}_exec
  src/denso_base.cpp
  src/denso_controller.cpp
  src/denso_controller_rc8.cpp
  src/denso_controller_rc8_cobotta.cpp
  src/denso_controller_rc9.cpp
  src/denso_robot.cpp
  src/${PROJECT_NAME}.cpp
  src/denso_robot_rc8.cpp
  src/denso_robot_rc8_cobotta.cpp
  src/denso_robot_rc9.cpp
  src/denso_task.cpp
  src/denso_variable.cpp
  src/tinyxml2.cpp)

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp rclcpp_action
  rclcpp_components
  bcap_core
  bcap_service
  std_msgs
  denso_robot_core_interfaces)

ament_target_dependencies(
  ${PROJECT_NAME}_exec
  rclcpp
  rclcpp_action
  rclcpp_components
  bcap_core
  bcap_service
  std_msgs
  denso_robot_core_interfaces)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${bcap_service_INCLUDE_DIRS}
  ${bcap_service_interfaces_INCLUDE_DIRS})

target_include_directories(
  ${PROJECT_NAME}_exec
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${bcap_service_INCLUDE_DIRS}
  ${bcap_service_interfaces_INCLUDE_DIRS})

target_link_libraries(
  ${PROJECT_NAME}
  ${bcap_service_LIBRARIES}
  ${bcap_core_LIBRARIES})

target_link_libraries(
  ${PROJECT_NAME}_exec
  ${bcap_service_LIBRARIES}
  ${bcap_core_LIBRARIES})


#=============
# Export
#=============

ament_export_libraries(${PROJECT_NAME})


#=============
# Install
#=============

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

install(
  TARGETS ${PROJECT_NAME}_exec
  EXPORT export_${PROJECT_NAME}_exec
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/)

install(
  DIRECTORY include/
  DESTINATION include)

ament_export_include_directories(
  include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
