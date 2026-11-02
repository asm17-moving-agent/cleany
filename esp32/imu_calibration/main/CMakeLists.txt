idf_component_register(
  SRCS "main.cpp" "imu_task.cpp" "serial_io.cpp" "../../src/mpu6050.cpp"
  INCLUDE_DIRS "." "../../src"
  REQUIRES esp_driver_gpio esp_driver_i2c esp_driver_usb_serial_jtag esp_timer json
)
