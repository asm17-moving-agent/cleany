#include "mpu6050.hpp"

#include <array>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace cleany {

esp_err_t Mpu6050::error(const char* operation, esp_err_t result) {
  lastOperation_ = operation;
  lastError_ = result;
  return result;
}

esp_err_t Mpu6050::readRegisters(uint8_t reg, uint8_t* output, size_t size) {
  return i2c_master_transmit_receive(device_, &reg, 1, output, size,
                                     config_.timeoutMs);
}

esp_err_t Mpu6050::writeRegister(uint8_t reg, uint8_t value) {
  const uint8_t bytes[] = {reg, value};
  return i2c_master_transmit(device_, bytes, sizeof(bytes), config_.timeoutMs);
}

esp_err_t Mpu6050::initialize() {
  shutdown();
  identity_ = 0;
  if (config_.timeoutMs <= 0 || config_.clockHz == 0 ||
      (config_.address != 0x68 && config_.address != 0x69)) {
    return error("configuration", ESP_ERR_INVALID_ARG);
  }
  i2c_master_bus_config_t busConfig{};
  busConfig.i2c_port = I2C_NUM_0;
  busConfig.sda_io_num = config_.sdaPin;
  busConfig.scl_io_num = config_.sclPin;
  busConfig.clk_source = I2C_CLK_SRC_DEFAULT;
  busConfig.glitch_ignore_cnt = 7;
  // J_I2C1 requires module/external pull-ups. Internal pull-ups are not a
  // substitute for verifying the actual 100 kHz bus rise time.
  busConfig.flags.enable_internal_pullup = false;
  esp_err_t result = i2c_new_master_bus(&busConfig, &bus_);
  if (result != ESP_OK) return error("create_bus", result);
  i2c_device_config_t deviceConfig{};
  deviceConfig.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  deviceConfig.device_address = config_.address;
  deviceConfig.scl_speed_hz = config_.clockHz;
  result = i2c_master_bus_add_device(bus_, &deviceConfig, &device_);
  if (result != ESP_OK) return error("add_device", result);
  result = readRegisters(0x75, &identity_, 1);
  if (result != ESP_OK) return error("read_identity", result);
  if (identity_ != 0x68) return error("identity_mismatch", ESP_ERR_INVALID_RESPONSE);

  result = writeRegister(0x6B, 0x80);  // DEVICE_RESET.
  if (result != ESP_OK) return error("reset_sensor", result);
  vTaskDelay(pdMS_TO_TICKS(100));
  // PLL with X gyro reference, temperature enabled, every sensor axis enabled.
  const std::array<std::array<uint8_t, 2>, 7> settings{{
      {{0x6B, 0x01}}, {{0x6C, 0x00}}, {{0x19, 9}}, {{0x1A, 3}},
      {{0x1B, 0}}, {{0x1C, 0}}, {{0x38, 1}},
  }};
  for (const auto& setting : settings) {
    result = writeRegister(setting[0], setting[1]);
    if (result != ESP_OK) return error("configure_sensor", result);
  }
  // Verify the range and divider before decoding with fixed SI scale factors.
  std::array<uint8_t, 4> readback{};
  result = readRegisters(0x19, readback.data(), readback.size());
  if (result != ESP_OK) return error("read_configuration", result);
  if (readback != std::array<uint8_t, 4>{{9, 3, 0, 0}}) {
    return error("configuration_mismatch", ESP_ERR_INVALID_RESPONSE);
  }
  return error("initialized", ESP_OK);
}

Mpu6050ReadStatus Mpu6050::readSample(ImuSample& sample) {
  if (device_ == nullptr || lastError_ != ESP_OK) {
    error("read_uninitialized", ESP_ERR_INVALID_STATE);
    return Mpu6050ReadStatus::kError;
  }
  uint8_t status = 0;
  esp_err_t result = readRegisters(0x3A, &status, 1);
  if (result != ESP_OK) {
    error("read_data_ready", result);
    return Mpu6050ReadStatus::kError;
  }
  if ((status & 1) == 0) return Mpu6050ReadStatus::kNotReady;
  std::array<uint8_t, 14> bytes{};
  result = readRegisters(0x3B, bytes.data(), bytes.size());
  if (result != ESP_OK) {
    error("read_sample", result);
    return Mpu6050ReadStatus::kError;
  }
  // This is MCU monotonic acquisition time, not ROS epoch.
  sample = decodeMpu6050Sample(bytes, esp_timer_get_time());
  return Mpu6050ReadStatus::kFresh;
}

void Mpu6050::shutdown() {
  if (device_ != nullptr) {
    (void)i2c_master_bus_rm_device(device_);
    device_ = nullptr;
  }
  if (bus_ != nullptr) {
    (void)i2c_del_master_bus(bus_);
    bus_ = nullptr;
  }
}

}  // namespace cleany
