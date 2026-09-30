cmake_minimum_required(VERSION 3.16)

if(DEFINED ENV{CLEANY_MICROROS_ENABLED} AND "$ENV{CLEANY_MICROROS_ENABLED}" STREQUAL "1")
  list(APPEND EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/micro_ros/micro_ros_espidf_component")
endif()
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(cleany_motor_controller)
