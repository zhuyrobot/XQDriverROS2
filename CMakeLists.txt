cmake_minimum_required(VERSION 3.15)
project(XQDriverROS2)

#1.Cfg
set(COMMON_CMAKE ${CMAKE_SOURCE_DIR}/common.cmake CACHE STRING "Common cmake file")
if(NOT EXISTS ${COMMON_CMAKE}) 
message(FATAL_ERROR "${COMMON_CMAKE} not exist") 
endif()
include(${COMMON_CMAKE})

#2.Pro
add_executable(XQDriverROS2 
CMakeLists.txt 
main.cpp
StatusPublisher.cpp
DiffDriverController.cpp
)
