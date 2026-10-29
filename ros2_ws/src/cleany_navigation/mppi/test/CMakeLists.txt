set(mppi_test_names
  optimizer_smoke_test controller_state_transition_test models_test
  noise_generator_test parameter_handler_test motion_model_tests
  trajectory_visualizer_tests utils_test path_handler_test
  critic_manager_test optimizer_unit_tests plugin_loading_test path_progress_test)
foreach(name IN LISTS mppi_test_names)
  ament_add_gtest(mppi_${name} ${name}.cpp)
  ament_target_dependencies(mppi_${name} ${mppi_dependencies})
  target_link_libraries(mppi_${name} cleany_mppi_controller)
endforeach()
ament_add_gtest(mppi_critics_tests critics_tests.cpp)
ament_target_dependencies(mppi_critics_tests ${mppi_dependencies})
target_link_libraries(mppi_critics_tests cleany_mppi_controller cleany_mppi_critics)
