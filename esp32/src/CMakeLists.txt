idf_component_register(
  SRCS "main.cpp" "mpu6050.cpp"
  INCLUDE_DIRS "." "${CMAKE_BINARY_DIR}/cleany_config"
  REQUIRES driver esp_timer micro_ros_espidf_component
)
