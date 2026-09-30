cmake_minimum_required(VERSION 3.16)
project(cleany_base_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WheelCommand.msg"
  "msg/WheelState.msg"
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
