cmake_minimum_required(VERSION 3.16)
project(cleany_scene_mapping)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(octomap REQUIRED)
find_package(moveit_ros_perception REQUIRED)
find_package(moveit_ros_occupancy_map_monitor REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

add_library(known_geometry_core SHARED src/known_geometry.cpp)
target_compile_features(known_geometry_core PUBLIC cxx_std_17)
target_include_directories(known_geometry_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(known_geometry_core geometric_shapes octomap)

add_library(known_geometry_octomap_updater SHARED src/known_geometry_updater.cpp src/partitioned_shape_mask.cpp)
target_compile_features(known_geometry_octomap_updater PUBLIC cxx_std_17)
target_link_libraries(known_geometry_octomap_updater known_geometry_core
  moveit_ros_perception::moveit_point_containment_filter)
ament_target_dependencies(known_geometry_octomap_updater
  moveit_ros_occupancy_map_monitor pluginlib rclcpp sensor_msgs tf2_eigen)
pluginlib_export_plugin_description_file(moveit_ros_occupancy_map_monitor plugins.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(moveit_msgs REQUIRED)
  find_package(urdf REQUIRED)
  add_executable(profile_mask test/profile_mask.cpp)
  target_link_libraries(profile_mask known_geometry_octomap_updater)
  ament_target_dependencies(profile_mask moveit_msgs urdf)
  ament_add_gtest(test_partitioned_shape_mask test/test_partitioned_shape_mask.cpp)
  target_link_libraries(test_partitioned_shape_mask known_geometry_octomap_updater)
  ament_add_gtest(test_known_geometry test/test_known_geometry.cpp)
  target_link_libraries(test_known_geometry known_geometry_core)
  ament_add_gtest(test_updater_plugin test/test_updater_plugin.cpp)
  ament_target_dependencies(test_updater_plugin
    moveit_ros_occupancy_map_monitor pluginlib rclcpp)
  ament_add_gtest(test_frame_integration test/test_frame_integration.cpp)
  target_link_libraries(test_frame_integration known_geometry_octomap_updater)
endif()

install(TARGETS known_geometry_core known_geometry_octomap_updater
  EXPORT export_${PROJECT_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(geometric_shapes octomap moveit_ros_perception
  moveit_ros_occupancy_map_monitor pluginlib rclcpp sensor_msgs tf2_eigen)
ament_package()
