cmake_minimum_required(VERSION 3.8)
project(cleany_benchmark)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(timed_controller SHARED src/timed_controller.cpp)
target_compile_features(timed_controller PUBLIC cxx_std_17)
target_compile_options(timed_controller PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(timed_controller
  geometry_msgs nav_msgs nav2_core nav2_costmap_2d nav2_util
  pluginlib rclcpp rclcpp_lifecycle tf2_ros)
pluginlib_export_plugin_description_file(nav2_core plugins.xml)
install(TARGETS timed_controller
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_export_libraries(timed_controller)
ament_export_dependencies(nav2_core pluginlib)
ament_package()
