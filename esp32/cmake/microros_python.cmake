# PlatformIO의 ESP-IDF 빌더는 CMake 구성 전에 PYTHONPATH를 지웁니다.
# micro-ROS는 IDF의 Python을 호출하지만 ROS 생성기는 고정된 별도 venv에 있습니다.
get_filename_component(_cleany_esp32_dir "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
set(_cleany_generator_path "${_cleany_esp32_dir}/.venv/lib/python3.10/site-packages")
if(NOT EXISTS "${_cleany_generator_path}/catkin_pkg")
  message(FATAL_ERROR "Run make firmware-setup to install pinned ROS generators")
endif()
set(ENV{PYTHONPATH} "${_cleany_generator_path}:$ENV{PYTHONPATH}")
