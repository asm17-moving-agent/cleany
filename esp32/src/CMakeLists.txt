set(runtime_sources "main.cpp")
set(runtime_includes ".")
if(CLEANY_INTEGRATED)
  set(uld "${CMAKE_CURRENT_LIST_DIR}/../tof_vl53l7ch/lib/st-vl53lmz")
  list(APPEND runtime_sources "integrated_sensors.cpp" "tof.cpp"
       "${uld}/platform.c" "${uld}/vl53lmz_api.c")
  list(APPEND runtime_includes "${uld}")
  list(APPEND runtime_private_requires esp_adc)
else()
  list(APPEND runtime_sources "legacy_sensors.cpp" "mpu6050.cpp")
endif()
idf_component_register(
  SRCS ${runtime_sources}
  INCLUDE_DIRS ${runtime_includes}
  REQUIRES driver esp_timer micro_ros_espidf_component
  PRIV_REQUIRES ${runtime_private_requires}
)
