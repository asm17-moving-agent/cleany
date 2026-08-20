cmake_minimum_required(VERSION 3.8)
project(cleany_navigation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(slam_profile test/test_slam_profile.py)
  ament_add_pytest_test(
    cartographer_profile test/test_cartographer_profile.py
  )
endif()

ament_package()
