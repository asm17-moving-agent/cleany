cmake_minimum_required(VERSION 3.8)
project(cleany_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS cleany_moveit_config/apply_collision_scene.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME apply_handeye_collision_scene
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(
  FILES .setup_assistant
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(
    moveit_config_contract
    test/test_moveit_config.py
  )
  ament_add_pytest_test(
    handeye_collision_scene_contract
    test/test_handeye_collision_scene.py
  )
  ament_add_pytest_test(
    mock_planning_runtime
    test/test_mock_planning_runtime.py
    TIMEOUT 240
  )
  ament_add_pytest_test(
    handeye_collision_scene_runtime
    test/test_handeye_collision_scene_runtime.py
    TIMEOUT 240
  )
endif()

ament_package()
