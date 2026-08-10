cmake_minimum_required(VERSION 3.8)
project(cleany_interfaces)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectedObject2D.msg"
  "msg/DetectedObject2DArray.msg"
  "msg/DetectedObject3D.msg"
  "msg/DetectedObject3DArray.msg"
  "action/InspectScene.action"
  "srv/PlanGrasp.srv"
  DEPENDENCIES geometry_msgs sensor_msgs std_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # google-genai can install a newer anyio pytest plugin than the pytest
  # version bundled with ROS 2 Humble. Interface tests need no third-party
  # plugins, so keep discovery deterministic during configure and execution.
  set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} "1")
  ament_add_pytest_test(
    interface_contract
    test/test_interface_contract.py
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
