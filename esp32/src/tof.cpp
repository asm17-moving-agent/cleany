#include "tof.hpp"

#include <algorithm>
#include <array>
#include <cstring>
#include "board_pins.hpp"
#include "integrated_i2c.hpp"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "vl53lmz_api.h"

namespace cleany::sensors {
namespace {
struct Channel {
  uint8_t index;
  VL53LMZ_Configuration sensor{};
  TofSample sample{};
  QueueHandle_t latest;
};
std::array<Channel, 4> channels{};
i2c_master_dev_handle_t mux, tof;
SemaphoreHandle_t busLock, startLock;

constexpr size_t kChunk = 128;
constexpr int kTimeoutMs = 40;
#ifndef CLEANY_TOF_I2C_HZ
#define CLEANY_TOF_I2C_HZ 100000
#endif
constexpr unsigned kBusHz = CLEANY_TOF_I2C_HZ;
static_assert(kBusHz == 100000 || kBusHz == 400000);
#ifndef CLEANY_TOF_RATE_HZ
#define CLEANY_TOF_RATE_HZ 15
#endif
constexpr unsigned kFrequencyHz = CLEANY_TOF_RATE_HZ;
static_assert(kFrequencyHz >= 1 && kFrequencyHz <= 15);

uint8_t transfer(void* handle, uint16_t reg, uint8_t* bytes, uint32_t size, bool write) {
  const uint8_t select = 1U << static_cast<Channel*>(handle)->index;
  while (size > 0) {
    const auto count = std::min<size_t>(size, kChunk);
    uint8_t packet[kChunk + 2] = {static_cast<uint8_t>(reg >> 8), static_cast<uint8_t>(reg)};
    xSemaphoreTake(busLock, portMAX_DELAY);
    auto result = i2c_master_transmit(mux, &select, 1, kTimeoutMs);
    if (result == ESP_OK) {
      if (write) {
        std::memcpy(packet + 2, bytes, count);
        result = i2c_master_transmit(tof, packet, count + 2, kTimeoutMs);
      } else {
        result = i2c_master_transmit_receive(tof, packet, 2, bytes, count, kTimeoutMs);
      }
    }
    xSemaphoreGive(busLock);
    if (result != ESP_OK) return VL53LMZ_STATUS_ERROR;
    reg += count;
    bytes += count;
    size -= count;
  }
  return VL53LMZ_STATUS_OK;
}

uint8_t read(void* handle, uint16_t reg, uint8_t* bytes, uint32_t size) {
  return transfer(handle, reg, bytes, size, false);
}
uint8_t write(void* handle, uint16_t reg, uint8_t* bytes, uint32_t size) {
  return transfer(handle, reg, bytes, size, true);
}
uint8_t wait(void*, uint32_t ms) {
  vTaskDelay(pdMS_TO_TICKS(ms));
  return 0;
}

bool initialize(Channel& ch) {
  auto* sensor = &ch.sensor;
  uint8_t alive = 0;
  if (vl53lmz_is_alive(sensor, &alive) || !alive ||
      sensor->module_type != VL53LMZ_MODULE_TYPE_L7 || vl53lmz_init(sensor) ||
      vl53lmz_set_resolution(sensor, VL53LMZ_RESOLUTION_8X8) ||
      vl53lmz_set_ranging_frequency_hz(sensor, kFrequencyHz)) return false;
  // The ULD builds the start command in shared, mutable file-scope arrays.
  xSemaphoreTake(startLock, portMAX_DELAY);
  const auto status = vl53lmz_start_ranging(sensor);
  xSemaphoreGive(startLock);
  return status == 0;
}

void channelTask(void* arg) {
  auto& ch = *static_cast<Channel*>(arg);
  auto& frame = ch.sample.frame;
  VL53LMZ_ResultsData result{};
  for (;;) {
    if (initialize(ch)) {
      int64_t lastFrame = esp_timer_get_time();
      for (;;) {
        uint8_t ready = 0;
        if (vl53lmz_check_data_ready(&ch.sensor, &ready)) break;
        const auto now = esp_timer_get_time();
        if (ready) {
          if (vl53lmz_get_ranging_data(&ch.sensor, &result)) break;
          // A frame can already be one ranging period old when the poll reports ready.
          frame.device_time_us = esp_timer_get_time();
          ch.sample.acquisitionUs = now - 1000000 / kFrequencyHz;
          frame.sensor_ready = true;
          ++frame.sequence;
          for (size_t z = 0; z < 64; ++z) {
            frame.distance_m[z] = result.distance_mm[z] * 0.001F;
            frame.sigma_m[z] = result.range_sigma_mm[z] * 0.001F;
            frame.target_status[z] = result.target_status[z];
            frame.target_count[z] = result.nb_target_detected[z];
            frame.valid[z] = result.nb_target_detected[z] > 0 &&
                (result.target_status[z] == 5 || result.target_status[z] == 9);
          }
          xQueueOverwrite(ch.latest, &ch.sample);
          lastFrame = now;
        }
        if (now - lastFrame > 3000000 / kFrequencyHz) break;
        vTaskDelay(pdMS_TO_TICKS(10));
      }
    }
    frame.sensor_ready = false;
    frame.device_time_us = esp_timer_get_time();
    ch.sample.acquisitionUs = frame.device_time_us;
    xQueueOverwrite(ch.latest, &ch.sample);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
}  // namespace

void startTof() {
  auto bus = openBus(I2C_NUM_0, board::tofSda, board::tofScl);
  mux = addDevice(bus, 0x70, kBusHz);
  tof = addDevice(bus, VL53LMZ_DEFAULT_I2C_ADDRESS >> 1, kBusHz);
  busLock = xSemaphoreCreateMutex();
  startLock = xSemaphoreCreateMutex();
  ESP_ERROR_CHECK(busLock && startLock ? ESP_OK : ESP_ERR_NO_MEM);
  for (uint8_t i = 0; i < channels.size(); ++i) {
    auto& ch = channels[i];
    ch.index = i;

    ch.latest = xQueueCreate(1, sizeof(TofSample));
    ESP_ERROR_CHECK(ch.latest ? ESP_OK : ESP_ERR_NO_MEM);
    ch.sensor.platform = {VL53LMZ_DEFAULT_I2C_ADDRESS, write, read, wait, &ch};
    ESP_ERROR_CHECK(xTaskCreate(channelTask, "tof_range", 6144, &ch, 2, nullptr) == pdPASS
                    ? ESP_OK : ESP_ERR_NO_MEM);
  }
}

bool takeTof(unsigned channel, TofSample& sample) {
  return xQueueReceive(channels[channel].latest, &sample, 0) == pdTRUE;
}
}  // namespace cleany::sensors
