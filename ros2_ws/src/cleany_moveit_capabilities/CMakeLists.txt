cmake_minimum_required(VERSION 3.8)
project(cleany_moveit_capabilities)
find_package(ament_cmake REQUIRED)
find_package(cleany_interfaces REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

add_library(cleany_moveit_capabilities SHARED src/state_validity_batch.cpp)
target_compile_features(cleany_moveit_capabilities PUBLIC cxx_std_17)
target_include_directories(cleany_moveit_capabilities PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(cleany_moveit_capabilities cleany_interfaces builtin_interfaces moveit_msgs moveit_core
  moveit_ros_move_group moveit_ros_planning_interface pluginlib rclcpp)
pluginlib_export_plugin_description_file(moveit_ros_move_group capabilities.xml)
install(TARGETS cleany_moveit_capabilities LIBRARY DESTINATION lib)
add_library(cleany_robust_time_parameterization SHARED src/robust_time_parameterization.cpp
  src/totg/time_optimal_trajectory_generation.cpp)
target_compile_features(cleany_robust_time_parameterization PUBLIC cxx_std_17)
target_include_directories(cleany_robust_time_parameterization PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(cleany_robust_time_parameterization moveit_core pluginlib rclcpp)
pluginlib_export_plugin_description_file(moveit_core planning_adapters.xml)
install(TARGETS cleany_robust_time_parameterization LIBRARY DESTINATION lib)
install(FILES src/totg/LICENSE src/totg/README.md DESTINATION share/${PROJECT_NAME}/totg)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(urdf REQUIRED)
  find_package(srdfdom REQUIRED)
  find_package(octomap REQUIRED)
  ament_add_gtest(test_state_validity_batch test/test_state_validity_batch.cpp)
  target_link_libraries(test_state_validity_batch cleany_moveit_capabilities)
  target_include_directories(test_state_validity_batch PRIVATE ${OCTOMAP_INCLUDE_DIRS})
  target_link_libraries(test_state_validity_batch ${OCTOMAP_LIBRARIES})
  ament_target_dependencies(test_state_validity_batch urdf srdfdom)
  # The randomized near-return corpus takes over a minute on an unoptimized Jetson build.
  ament_add_gtest(test_robust_time_parameterization test/test_robust_time_parameterization.cpp
    TIMEOUT 180)
  target_link_libraries(test_robust_time_parameterization cleany_robust_time_parameterization)
  ament_target_dependencies(test_robust_time_parameterization urdf srdfdom)
endif()
ament_package()
