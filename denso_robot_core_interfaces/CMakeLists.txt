cmake_minimum_required(VERSION 3.8)
project(denso_robot_core_interfaces)

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
find_package(std_msgs REQUIRED)

include_directories(include)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "action/DriveString.action"
  "action/DriveValue.action"
  "action/MoveString.action"
  "action/MoveValue.action"
  "msg/ExJoints.msg"
  "msg/Joints.msg"
  "msg/PoseData.msg"
  "srv/ChangeMode.srv"
  "msg/UserIO.msg")


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
