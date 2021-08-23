cmake_minimum_required(VERSION 3.15)
project(xqdriver_ros2)

#1.Cfg
set(COMMON_CMAKE ${CMAKE_SOURCE_DIR}/common.cmake CACHE STRING "Common cmake file")
if(NOT EXISTS ${COMMON_CMAKE}) 
message(FATAL_ERROR "${COMMON_CMAKE} not exist") 
endif()
include(${COMMON_CMAKE})
find_package(rosidl_default_generators REQUIRED) ####################add1
#2.Pro
add_executable(xqserial_server_node 
CMakeLists.txt 
main.cpp
StatusPublisher.cpp
DiffDriverController.cpp
)

install(TARGETS xqserial_server_node DESTINATION lib/${PROJECT_NAME}) ####################add2
  
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}) ####################add3
  
ament_package()###m####################add4
#colcon build --merge-install --base-path .. --cmake-args -DCOMMON_CMAKE=/root/app/include/cscv/cmaker/cscv.cmake

