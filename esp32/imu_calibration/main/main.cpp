#include <array>

#include "diagnostic_tasks.hpp"
#include "driver/gpio.h"
#include "driver/usb_serial_jtag.h"
#include "freertos/task.h"

namespace {
// PINMAP.md: M1, M2, M3, M4 PWM/DIR. No LEDC or command receiver runs here.
constexpr std::array<gpio_num_t, 8> kMotorPins{{
    GPIO_NUM_2, GPIO_NUM_1, GPIO_NUM_12, GPIO_NUM_11,
    GPIO_NUM_5, GPIO_NUM_6, GPIO_NUM_16, GPIO_NUM_17,
}};
cleany::diagnostic::Channels channels;

esp_err_t holdMotorOutputsLow() {
  uint64_t mask = 0;
  for (const auto pin : kMotorPins) {
    mask |= 1ULL << pin;
    const esp_err_t result = gpio_set_level(pin, 0);
    if (result != ESP_OK) return result;
  }
  gpio_config_t config{};
  config.pin_bit_mask = mask;
  config.mode = GPIO_MODE_INPUT_OUTPUT;
  config.pull_up_en = GPIO_PULLUP_DISABLE;
  config.pull_down_en = GPIO_PULLDOWN_DISABLE;
  config.intr_type = GPIO_INTR_DISABLE;
  const esp_err_t result = gpio_config(&config);
  if (result != ESP_OK) return result;
  for (const auto pin : kMotorPins) {
    if (gpio_get_level(pin) != 0) return ESP_ERR_INVALID_STATE;
  }
  return ESP_OK;
}
}  // namespace

extern "C" void app_main() {
  ESP_ERROR_CHECK(holdMotorOutputsLow());
  usb_serial_jtag_driver_config_t usb{};
  usb.tx_buffer_size = 8192;
  usb.rx_buffer_size = 512;
  ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb));
  channels.latest = xQueueCreate(1, sizeof(cleany::diagnostic::Snapshot));
  channels.restarts = xQueueCreate(4, sizeof(cleany::diagnostic::RestartCommand));
  channels.started = xQueueCreate(4, sizeof(cleany::diagnostic::RunStarted));
  ESP_ERROR_CHECK(channels.latest && channels.restarts && channels.started
                      ? ESP_OK : ESP_ERR_NO_MEM);
  ESP_ERROR_CHECK(xTaskCreatePinnedToCore(cleany::diagnostic::imuTask, "imu_calibration",
      8192, &channels, 4, nullptr, cleany::diagnostic::kImuCore) == pdPASS
          ? ESP_OK : ESP_ERR_NO_MEM);
  ESP_ERROR_CHECK(xTaskCreatePinnedToCore(cleany::diagnostic::serialTask, "imu_serial",
      8192, &channels, 3, nullptr, cleany::diagnostic::kSerialCore) == pdPASS
          ? ESP_OK : ESP_ERR_NO_MEM);
}
