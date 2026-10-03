cmake_minimum_required(VERSION 3.16)
set(EXTRA_COMPONENT_DIRS
  "${CMAKE_CURRENT_LIST_DIR}/main"
  "${CMAKE_CURRENT_LIST_DIR}/../micro_ros/micro_ros_espidf_component")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(cleany_microros_smoke)
