cmake_minimum_required(VERSION 3.8)
project(cleany_description)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/runtime_model_state DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY config launch meshes mjcf urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(runtime_state test/test_runtime_state.py)
  ament_add_pytest_test(runtime_model test/test_runtime_model.py)
  ament_add_pytest_test(cad_coordinates test/test_cad_coordinates.py)
  ament_add_pytest_test(model_parity test/test_model_parity.py TIMEOUT 300)
  ament_add_pytest_test(camera_calibration test/test_camera_calibration.py)
  ament_add_pytest_test(wrist_mount_assets test/test_wrist_mount_assets.py)
  ament_add_pytest_test(wrist_cameras test/test_wrist_cameras.py)
  ament_add_pytest_test(collision_meshes test/test_collision_meshes.py TIMEOUT 120
    ENV OPENBLAS_NUM_THREADS=1)
  ament_add_pytest_test(freecad_model test/test_freecad_model.py TIMEOUT 120)
  ament_add_pytest_test(freecad_kinematics test/test_freecad_kinematics.py TIMEOUT 120)
  ament_add_pytest_test(freecad_tip_mounts test/test_freecad_tip_mounts.py TIMEOUT 120)
  ament_add_pytest_test(freecad_gripper_linkage test/test_freecad_gripper_linkage.py TIMEOUT 120)
endif()

ament_package()
