cmake_minimum_required(VERSION 3.8)
project(denso_robot_control)

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

set(
  THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_action
  rclcpp_components
  rclcpp_lifecycle
  std_msgs
  bcap_core
  bcap_service
  bcap_service_interfaces
  denso_robot_core
  denso_robot_core_interfaces)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(include denso_robot_core hardware_interface)

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
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

