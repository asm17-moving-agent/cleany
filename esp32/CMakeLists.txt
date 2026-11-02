cmake_minimum_required(VERSION 3.16)
include("${CMAKE_CURRENT_LIST_DIR}/cmake/microros_python.cmake")

list(APPEND EXTRA_COMPONENT_DIRS
  "${CMAKE_CURRENT_LIST_DIR}/micro_ros/micro_ros_espidf_component"
)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(cleany_motor_controller)
