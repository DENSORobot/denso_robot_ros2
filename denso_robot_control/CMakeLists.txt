cmake_minimum_required(VERSION 3.5)
project(denso_robot_control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(bcap_core REQUIRED)
find_package(bcap_service REQUIRED)
find_package(denso_robot_core REQUIRED)
find_package(std_msgs REQUIRED)
find_package(bcap_service_interfaces REQUIRED)
find_package(denso_robot_core_interfaces REQUIRED)

include_directories(include denso_robot_core hardware_interface)

set(
  THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rclcpp_action
  rclcpp_components
  std_msgs
  denso_robot_core
  bcap_core
  bcap_service_interfaces
  denso_robot_core_interfaces)

add_library(
  ${PROJECT_NAME}_plugin
  SHARED
  src/denso_robot_hw.cpp
  src/${PROJECT_NAME}.cpp)

add_executable(${PROJECT_NAME}_exec src/${PROJECT_NAME}.cpp)

target_link_libraries(
  ${PROJECT_NAME}_exec
  ${bcap_service_LIBRARIES}
  ${bcap_core_LIBRARIES}
  ${denso_robot_core_LIBRARIES})

target_include_directories(
  ${PROJECT_NAME}_exec
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${hardware_interface_INCLUDE_DIRS}
  ${denso_robot_core_INCLUDE_DIRS}
  ${bcap_service_INCLUDE_DIRS})

ament_target_dependencies(
  ${PROJECT_NAME}_exec
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${THIS_PACKAGE_INCLUDE_DEPENDS})

target_link_libraries(
  ${PROJECT_NAME}_plugin
  ${bcap_service_LIBRARIES}
  ${bcap_core_LIBRARIES}
  ${denso_robot_core_LIBRARIES})

target_include_directories(${PROJECT_NAME}_plugin
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${hardware_interface_INCLUDE_DIRS}
  ${denso_robot_core_INCLUDE_DIRS}
  ${bcap_service_INCLUDE_DIRS})

ament_target_dependencies(
  ${PROJECT_NAME}_plugin
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(hardware_interface denso_hardware_interface_plugin.xml)


#=============
# Install
#=============

install(
  TARGETS ${PROJECT_NAME}_plugin
  DESTINATION lib)

install(
  DIRECTORY include/
  DESTINATION include)

install(
  DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/)

install(
  TARGETS ${PROJECT_NAME}_exec
  EXPORT export_${PROJECT_NAME}_exec
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include)


#=============
# Export
#=============

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_plugin)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  ${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

