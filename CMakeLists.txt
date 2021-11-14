cmake_minimum_required(VERSION 3.15)
project(xqdriver_ros2)

#1.Cfg
set(CSCV_DIR /root/app/cxx/cscv CACHE STRING "cscv directory")
set(CSCV_CFG ${CSCV_DIR}/cmaker/cscv.cmake)
if(NOT EXISTS ${CSCV_CFG}) 
message(FATAL_ERROR "${CSCV_CFG} not exist") 
endif()
include(${CSCV_CFG})

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

