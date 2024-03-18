cmake_minimum_required(VERSION 3.8)
project(bcap_service)

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
find_package(bcap_core REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(bcap_service_interfaces REQUIRED)

include_directories(include ${bcap_core_INCLUDE_DIRS})

add_library(
  ${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}.cpp)

ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs bcap_core bcap_service_interfaces)

target_include_directories(
  ${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${bcap_core_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} ${bcap_core_LIBRARIES})

add_executable(${PROJECT_NAME}_exec
  src/${PROJECT_NAME}.cpp)

ament_target_dependencies(${PROJECT_NAME}_exec rclcpp std_msgs bcap_core bcap_service_interfaces)

target_include_directories(
  ${PROJECT_NAME}_exec PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${bcap_core_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME}_exec ${bcap_core_LIBRARIES})


#=============
# Export
#=============

ament_export_include_directories(include)
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
  INCLUDES DESTINATION INCLUDE)

install(
  TARGETS ${PROJECT_NAME}_exec
  EXPORT export_${PROJECT_NAME}_exec
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION INCLUDE)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
