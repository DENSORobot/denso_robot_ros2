cmake_minimum_required(VERSION 3.8)
project(bcap_core)

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


# Options
option  (_DN_USE_VARIANT_API "Use VARIANT API" ON)
option  (_DN_USE_BSTR_API "Use BSTR API" ON)
option  (_USE_LINUX_API "Use Linux API" ON)
message (STATUS "_USE_LINUX_API=${_USE_LINUX_API}")

# Add definitions
if(_USE_LINUX_API)
  add_definitions(-D_USE_LINUX_API)
# else(_USE_LINUX_API)
else()
  add_definitions(-D_USE_WIN_API)
  add_definitions("-D_DN_EXP_RACSTR=__declspec(dllexport)")
  add_definitions("-D_DN_EXP_COMMON=__declspec(dllexport)")
# endif(_USE_LINUX_API)
endif()

if(_DN_USE_VARIANT_API)
  add_definitions(-D_DN_USE_VARIANT_API=1)
# else(_DN_USE_VARIANT_API)
else()
  add_definitions(-D_DN_USE_VARIANT_API=0)
# endif(_DN_USE_VARIANT_API)
endif()

if(_DN_USE_BSTR_API)
  add_definitions(-D_DN_USE_BSTR_API=1)
# else(_DN_USE_BSTR_API)
else()
  add_definitions(-D_DN_USE_BSTR_API=0)
# endif(_DN_USE_BSTR_API)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

include_directories(include/${PROJECT_NAME})

# bcap_client library
add_library(
  bcap_client SHARED
  src/bCAPClient/bcap_client.c
  src/bcap_common.c
  src/dn_com.c
  src/dn_common.c
  src/dn_device.c
  src/dn_socket.c
  src/dn_tcp.c
  src/dn_thread.c
  src/dn_udp.c)

target_include_directories(
  bcap_client PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/bCAPClient>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}/bCAPClient>")

if(_USE_LINUX_API)
  target_link_libraries(bcap_client  pthread)
# endif(_USE_LINUX_API)
endif()

# bcap_server library
add_library(
  bcap_server SHARED
  src/bCAPServer/bcap_server.c
  src/bcap_common.c
  src/dn_com.c
  src/dn_common.c
  src/dn_device.c
  src/dn_socket.c
  src/dn_tcp.c
  src/dn_thread.c
  src/dn_udp.c)

target_include_directories(
  bcap_server PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/bCAPServer>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}/bCAPServer>")

if(_USE_LINUX_API)
  target_link_libraries(bcap_server pthread)
# endif(_USE_LINUX_API)
endif()

# rac_string library
add_library(
  rac_string SHARED
  src/RACString/rac_string.c
  src/dn_common.c)

target_include_directories(
  rac_string PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/RACString>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}/RACString>")

# tpcomm library
add_library(
  tpcomm SHARED
  src/TPComm/dn_tpcomm.c
  src/dn_com.c
  src/dn_common.c
  src/dn_device.c
  src/dn_robotalk.c
  src/dn_socket.c
  src/dn_thread.c
  src/dn_udp.c)

target_include_directories(
  tpcomm PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/TPComm>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}/TPComm>")

if(_USE_LINUX_API)
  target_link_libraries(tpcomm pthread)
# endif(_USE_LINUX_API)
endif()


#=============
# Export
#=============

ament_export_include_directories(include)
ament_export_libraries(
  bcap_client
  bcap_server
  rac_string
  tpcomm
)


#=============
# Install
#=============

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS bcap_client
  EXPORT export_bcap_client
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  TARGETS bcap_server
  EXPORT export_bcap_server
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  TARGETS rac_string
  EXPORT export_rac_string
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  TARGETS tpcomm
  EXPORT export_tpcomm
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
