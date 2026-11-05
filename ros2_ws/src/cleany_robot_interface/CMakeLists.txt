cmake_minimum_required(VERSION 3.16)
project(cleany_robot_interface)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION lib/${PROJECT_NAME})

add_library(cleany_feetech_hardware SHARED src/serial_bus.cpp src/system.cpp)
target_compile_features(cleany_feetech_hardware PUBLIC cxx_std_17)
target_compile_options(cleany_feetech_hardware PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(cleany_feetech_hardware PUBLIC include)
ament_target_dependencies(cleany_feetech_hardware
  hardware_interface pluginlib rclcpp rclcpp_lifecycle)
pluginlib_export_plugin_description_file(hardware_interface plugin.xml)

install(TARGETS cleany_feetech_hardware LIBRARY DESTINATION lib)
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_gtest(test_driver test/test_driver.cpp)
  target_link_libraries(test_driver cleany_feetech_hardware util)
  ament_add_pytest_test(robot_interface test)
endif()

ament_package()
