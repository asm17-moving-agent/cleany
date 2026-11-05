# Tuned Humble MPPI implementation owned by cleany_navigation.
find_package(xtensor REQUIRED)
find_package(xsimd REQUIRED)
set(mppi_dependencies
  rclcpp rclcpp_lifecycle nav2_common pluginlib tf2 geometry_msgs visualization_msgs nav_msgs
  nav2_core nav2_costmap_2d nav2_util tf2_geometry_msgs tf2_eigen tf2_ros)
foreach(dependency IN LISTS mppi_dependencies)
  find_package(${dependency} REQUIRED)
endforeach()

add_library(cleany_mppi_controller SHARED
  src/controller.cpp src/optimizer.cpp src/critic_manager.cpp
  src/trajectory_visualizer.cpp src/path_handler.cpp
  src/parameters_handler.cpp src/noise_generator.cpp)
add_library(cleany_mppi_critics SHARED
  src/critics/obstacles_critic.cpp src/critics/cost_critic.cpp
  src/critics/goal_critic.cpp src/critics/goal_angle_critic.cpp
  src/critics/path_align_critic.cpp src/critics/path_align_legacy_critic.cpp
  src/critics/path_follow_critic.cpp src/critics/path_angle_critic.cpp
  src/critics/prefer_forward_critic.cpp src/critics/twirling_critic.cpp
  src/critics/constraint_critic.cpp src/critics/velocity_deadband_critic.cpp)
foreach(target cleany_mppi_controller cleany_mppi_critics)
  target_compile_features(${target} PUBLIC cxx_std_17)
  target_compile_definitions(${target} PUBLIC XTENSOR_ENABLE_XSIMD XTENSOR_USE_XSIMD)
  target_compile_options(${target} PUBLIC -fconcepts)
  target_compile_options(${target} PRIVATE -O3 -finline-limit=10000000 -ffp-contract=fast -ffast-math -mtune=generic)
  target_include_directories(${target} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
    $<INSTALL_INTERFACE:include> ${xsimd_INCLUDE_DIRS})
  target_link_libraries(${target} xtensor xtensor::optimize xtensor::use_xsimd)
  ament_target_dependencies(${target} ${mppi_dependencies})
endforeach()
install(TARGETS cleany_mppi_controller cleany_mppi_critics
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(FILES LICENSE.md README.upstream.md DESTINATION share/${PROJECT_NAME}/mppi)

add_executable(mppi_replay_snapshot src/replay_snapshot.cpp)
target_link_libraries(mppi_replay_snapshot cleany_mppi_controller)
ament_target_dependencies(mppi_replay_snapshot ${mppi_dependencies})
install(TARGETS mppi_replay_snapshot DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()
