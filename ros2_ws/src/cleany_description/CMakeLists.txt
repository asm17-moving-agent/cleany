cmake_minimum_required(VERSION 3.8)
project(cleany_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch meshes mjcf urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(model_parity test/test_model_parity.py TIMEOUT 300)
  ament_add_pytest_test(camera_calibration test/test_camera_calibration.py)
  ament_add_pytest_test(wrist_mount_assets test/test_wrist_mount_assets.py)
  ament_add_pytest_test(wrist_cameras test/test_wrist_cameras.py)
  ament_add_pytest_test(collision_meshes test/test_collision_meshes.py TIMEOUT 120
    ENV OPENBLAS_NUM_THREADS=1)
endif()

ament_package()
