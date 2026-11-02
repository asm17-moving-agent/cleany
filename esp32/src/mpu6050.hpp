#pragma once

#include <cstddef>
#include <cstdint>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "mpu6050_sample.hpp"

namespace cleany {

struct Mpu6050Config {
  gpio_num_t sdaPin = GPIO_NUM_41;
  gpio_num_t sclPin = GPIO_NUM_42;
  uint16_t address = 0x68;
  uint32_t clockHz = 100000;
  int timeoutMs = 10;
};

enum class Mpu6050ReadStatus { kFresh, kNotReady, kError };

class Mpu6050 {
 public:
  explicit Mpu6050(Mpu6050Config config = {}) : config_(config) {}
  ~Mpu6050() { shutdown(); }
  Mpu6050(const Mpu6050&) = delete;
  Mpu6050& operator=(const Mpu6050&) = delete;

  // Fixed sample contract: 100 Hz, +/-2 g, +/-250 degrees/s, gyro DLPF 42 Hz.
  esp_err_t initialize();
  Mpu6050ReadStatus readSample(ImuSample& sample);
  void shutdown();
  esp_err_t lastError() const { return lastError_; }
  const char* lastOperation() const { return lastOperation_; }
  uint8_t identity() const { return identity_; }

 private:
  esp_err_t readRegisters(uint8_t reg, uint8_t* output, size_t size);
  esp_err_t writeRegister(uint8_t reg, uint8_t value);
  esp_err_t error(const char* operation, esp_err_t result);
  Mpu6050Config config_;
  i2c_master_bus_handle_t bus_ = nullptr;
  i2c_master_dev_handle_t device_ = nullptr;
  esp_err_t lastError_ = ESP_OK;
  const char* lastOperation_ = "none";
  uint8_t identity_ = 0;
};

}  // namespace cleany
