cmake_minimum_required(VERSION 3.16)
project(cleany_base_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WheelCommand.msg"
  "msg/WheelState.msg"
)
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} "1")
  ament_add_pytest_test(wheel_message_contract test/test_contract.py
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1")
endif()
ament_export_dependencies(rosidl_default_runtime)
ament_package()
