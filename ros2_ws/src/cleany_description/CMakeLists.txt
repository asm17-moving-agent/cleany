cmake_minimum_required(VERSION 3.8)
project(cleany_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch meshes mjcf urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(model_parity test/test_model_parity.py)
endif()

ament_package()
