#pragma once

#include "driver/i2c_master.h"

namespace cleany::sensors {
inline i2c_master_bus_handle_t openBus(i2c_port_num_t port, gpio_num_t sda, gpio_num_t scl) {
  i2c_master_bus_config_t config{};
  config.i2c_port = port;
  config.sda_io_num = sda;
  config.scl_io_num = scl;
  config.clk_source = I2C_CLK_SRC_DEFAULT;
  config.glitch_ignore_cnt = 7;
  i2c_master_bus_handle_t bus{};
  ESP_ERROR_CHECK(i2c_new_master_bus(&config, &bus));
  return bus;
}

inline i2c_master_dev_handle_t addDevice(i2c_master_bus_handle_t bus, uint8_t address,
                                         uint32_t frequencyHz = 100000) {
  i2c_device_config_t device{};
  device.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  device.device_address = address;
  device.scl_speed_hz = frequencyHz;
  device.scl_wait_us = 10000;
  i2c_master_dev_handle_t handle{};
  ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &device, &handle));
  return handle;
}
}  // namespace cleany::sensors
