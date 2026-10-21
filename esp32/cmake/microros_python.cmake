# PlatformIO's ESP-IDF builder clears PYTHONPATH before configuring CMake.
# micro-ROS invokes IDF's Python, but its ROS generators live in our pinned venv.
get_filename_component(_cleany_esp32_dir "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
set(_cleany_generator_path "${_cleany_esp32_dir}/.venv/lib/python3.10/site-packages")
if(NOT EXISTS "${_cleany_generator_path}/catkin_pkg")
  message(FATAL_ERROR "Run make firmware-setup to install pinned ROS generators")
endif()
set(ENV{PYTHONPATH} "${_cleany_generator_path}:$ENV{PYTHONPATH}")
