cmake_minimum_required(VERSION 3.8)
project(cleany_moveit_collision)

find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(fcl REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

add_library(cleany_solid_collision SHARED src/solid_collision_env.cpp src/solid_collision_plugin.cpp)
target_compile_features(cleany_solid_collision PUBLIC cxx_std_17)
target_include_directories(cleany_solid_collision PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(cleany_solid_collision moveit_core geometric_shapes pluginlib rclcpp)
target_link_libraries(cleany_solid_collision fcl)
pluginlib_export_plugin_description_file(moveit_core collision_plugins.xml)

add_library(cleany_collision_detector_guard SHARED src/collision_detector_guard.cpp)
target_compile_features(cleany_collision_detector_guard PUBLIC cxx_std_17)
ament_target_dependencies(cleany_collision_detector_guard moveit_core moveit_ros_move_group
  moveit_ros_planning_interface pluginlib rclcpp)
pluginlib_export_plugin_description_file(moveit_ros_move_group capabilities.xml)

install(TARGETS cleany_solid_collision EXPORT solid_collision_targets LIBRARY DESTINATION lib)
install(TARGETS cleany_collision_detector_guard LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(solid_collision_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(moveit_core geometric_shapes fcl)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(urdf REQUIRED)
  find_package(srdfdom REQUIRED)
  find_package(octomap REQUIRED)
  ament_add_gtest(test_solid_collision test/test_solid_collision.cpp)
  target_link_libraries(test_solid_collision cleany_solid_collision ${OCTOMAP_LIBRARIES})
  target_include_directories(test_solid_collision PRIVATE ${OCTOMAP_INCLUDE_DIRS})
  ament_target_dependencies(test_solid_collision urdf srdfdom)
endif()

ament_package()
