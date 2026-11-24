#include "board_sensors.hpp"
#include "board_pins.hpp"
#include "integrated_i2c.hpp"
#include "tof.hpp"
#include "cleany_interfaces/msg/detail/tof_frame__type_support.h"
#include "sensor_msgs/sensor_msgs/msg/imu.h"
#include "sensor_msgs/sensor_msgs/msg/detail/imu__type_support.h"
#include "std_msgs/std_msgs/msg/float32.h"
#include "std_msgs/std_msgs/msg/detail/float32__type_support.h"
#include "rmw_microros/rmw_microros.h"
#include <cstring>
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "rclc/rclc.h"

namespace cleany::sensors {
namespace {
#ifndef CLEANY_ENABLE_BNO055
#define CLEANY_ENABLE_BNO055 0
#endif
constexpr bool kBno055Enabled = CLEANY_ENABLE_BNO055 != 0;

struct MotionSample {
  int64_t timestampUs;
  float gyro[3], acceleration[3];
};
struct BatterySample {
  int64_t timestampUs;
  float volts;
};
QueueHandle_t imuQueue;
QueueHandle_t batteryQueue;
rcl_publisher_t imuPub{}, batteryPub{}, tofPubs[4]{};
constexpr const char* topics[] = {"tof/frame", "tof/left/frame", "tof/right/frame", "tof/rear/frame"};
constexpr const char* frames[] = {"tof_link", "tof_left_link", "tof_right_link", "tof_rear_link"};
constexpr char kBatteryTag[] = "battery_adc";
constexpr int kBatteryPeriodMs = 1000;
constexpr int kBatterySampleCount = 16;
constexpr int kBatterySampleSpacingMs = 2;
constexpr adc_atten_t kBatteryAttenuation = ADC_ATTEN_DB_12;
constexpr adc_bitwidth_t kBatteryBitWidth = ADC_BITWIDTH_12;
constexpr int kBatteryFullScale = (1 << kBatteryBitWidth) - 1;
constexpr float kBatteryDividerRatio = (100.0F + 22.0F) / 22.0F;

esp_err_t configureBatteryAdc(adc_oneshot_unit_handle_t& adc, adc_channel_t& channel,
                              adc_cali_handle_t& calibration) {
  adc_unit_t unit{};
  auto status = adc_oneshot_io_to_channel(board::batteryAdc, &unit, &channel);
  if (status != ESP_OK) return status;

  adc_oneshot_unit_init_cfg_t unitConfig{};
  unitConfig.unit_id = unit;
  unitConfig.ulp_mode = ADC_ULP_MODE_DISABLE;
  status = adc_oneshot_new_unit(&unitConfig, &adc);
  if (status != ESP_OK) return status;

  const adc_oneshot_chan_cfg_t channelConfig{kBatteryAttenuation, kBatteryBitWidth};
  status = adc_oneshot_config_channel(adc, channel, &channelConfig);
  if (status != ESP_OK) return status;

  const adc_cali_curve_fitting_config_t calibrationConfig{
      unit, channel, kBatteryAttenuation, kBatteryBitWidth};
  return adc_cali_create_scheme_curve_fitting(&calibrationConfig, &calibration);
}

bool readBatteryVolts(adc_oneshot_unit_handle_t adc, adc_channel_t channel,
                      adc_cali_handle_t calibration, float& volts) {
  int totalMillivolts = 0;
  for (int sample = 0; sample < kBatterySampleCount; ++sample) {
    int raw = 0;
    int millivolts = 0;
    if (adc_oneshot_read(adc, channel, &raw) != ESP_OK || raw < 0 || raw >= kBatteryFullScale ||
        adc_cali_raw_to_voltage(calibration, raw, &millivolts) != ESP_OK || millivolts < 0) {
      return false;
    }
    totalMillivolts += millivolts;
    vTaskDelay(pdMS_TO_TICKS(kBatterySampleSpacingMs));
  }
  volts = totalMillivolts * kBatteryDividerRatio / (1000.0F * kBatterySampleCount);
  return true;
}

void batteryTask(void*) {
  adc_oneshot_unit_handle_t adc{};
  adc_channel_t channel{};
  adc_cali_handle_t calibration{};
  const auto status = configureBatteryAdc(adc, channel, calibration);
  if (status != ESP_OK) {
    if (adc) (void)adc_oneshot_del_unit(adc);
    ESP_LOGE(kBatteryTag, "ADC initialization failed: %s", esp_err_to_name(status));
    vTaskDelete(nullptr);
    return;
  }

  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    BatterySample sample{};
    if (readBatteryVolts(adc, channel, calibration, sample.volts)) {
      sample.timestampUs = esp_timer_get_time();
      xQueueOverwrite(batteryQueue, &sample);
    } else {
      xQueueReset(batteryQueue);
    }
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(kBatteryPeriodMs));
  }
}

bool publishBatteryVoltage() {
  BatterySample sample{};
  if (!batteryQueue || xQueueReceive(batteryQueue, &sample, 0) != pdTRUE ||
      esp_timer_get_time() - sample.timestampUs > kBatteryPeriodMs * 1000) {
    return true;
  }
  const std_msgs__msg__Float32 message{sample.volts};
  return rcl_publish(&batteryPub, &message, nullptr) == RCL_RET_OK;
}

bool configureImu(i2c_master_dev_handle_t device) {
  const auto write = [device](uint8_t reg, uint8_t value) {
    const uint8_t command[] = {reg, value};
    return i2c_master_transmit(device, command, sizeof(command), 30) == ESP_OK;
  };
  const uint8_t idReg = 0x00;
  uint8_t id = 0;
  if (!write(0x07, 0) ||
      i2c_master_transmit_receive(device, &idReg, 1, &id, 1, 30) != ESP_OK ||
      id != 0xA0 || !write(0x3D, 0)) return false;
  vTaskDelay(pdMS_TO_TICKS(20));  // Operating mode -> CONFIGMODE.
  if (!write(0x3B, 0x02) || !write(0x41, 0x24) || !write(0x42, 0) ||
      !write(0x3D, 0x05)) return false;  // rad/s, native axes, ACCGYRO.
  vTaskDelay(pdMS_TO_TICKS(10));
  uint8_t mode = 0;
  const uint8_t modeReg = 0x3D;
  return i2c_master_transmit_receive(device, &modeReg, 1, &mode, 1, 30) == ESP_OK &&
         mode == 0x05;
}

void gyroTask(void*) {
  auto bus = openBus(I2C_NUM_1, board::imuSda, board::imuScl);
  auto device = addDevice(bus, 0x28);
  MotionSample sample{};
  for (;;) {
    if (configureImu(device)) {
      for (;;) {
        // ACC, MAG and GYR vectors are contiguous on register page zero.
        constexpr uint8_t dataReg = 0x08;
        uint8_t bytes[18]{};
        sample.timestampUs = esp_timer_get_time();
        if (i2c_master_transmit_receive(device, &dataReg, 1, bytes, sizeof(bytes), 30) != ESP_OK)
          break;
        const auto signedWord = [&bytes](unsigned i) {
          return static_cast<int16_t>(bytes[i] | (bytes[i+1] << 8));
        };
        for (unsigned i = 0; i < 3; ++i) {
          sample.acceleration[i] = signedWord(2*i) / 100.0F;
          sample.gyro[i] = signedWord(12 + 2*i) / 900.0F;
        }
        xQueueOverwrite(imuQueue, &sample);
        vTaskDelay(pdMS_TO_TICKS(10));
      }
    }
    xQueueReset(imuQueue);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}
bool stamp(std_msgs__msg__Header& header, const char* frame, int64_t timestampUs) {
  const int64_t now = esp_timer_get_time();
  const int64_t epoch = rmw_uros_epoch_nanos();
  if (timestampUs <= 0 || timestampUs > now || (now - timestampUs) >= epoch/1000) return false;
  const int64_t ns = epoch - (now - timestampUs)*1000;
  if (ns / 1000000000 > INT32_MAX) return false;
  header.stamp.sec = ns / 1000000000;
  header.stamp.nanosec = ns % 1000000000;
  header.frame_id.data = const_cast<char*>(frame);
  header.frame_id.size = std::strlen(frame);
  header.frame_id.capacity = header.frame_id.size + 1;
  return true;
}

void finish(rcl_publisher_t& pub, rcl_node_t* node) {
  const auto result = rcl_publisher_fini(&pub, node);
  (void)result;
}
}  // namespace

void start(uint32_t) {
  startTof();
  if constexpr (kBno055Enabled) {
    imuQueue = xQueueCreate(1, sizeof(MotionSample));
    ESP_ERROR_CHECK(imuQueue ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(xTaskCreate(gyroTask, "bno_motion", 4096, nullptr, 2, nullptr) == pdPASS
                    ? ESP_OK : ESP_ERR_NO_MEM);
  }
  batteryQueue = xQueueCreate(1, sizeof(BatterySample));
  if (batteryQueue && xTaskCreate(batteryTask, "battery_adc", 4096, nullptr, 2, nullptr) == pdPASS) {
    return;
  }
  if (batteryQueue) vQueueDelete(batteryQueue);
  batteryQueue = nullptr;
  ESP_LOGE(kBatteryTag, "Could not start battery ADC task");
}

bool open(rcl_node_t* node, const rmw_qos_profile_t& sensorQos) {
  if constexpr (kBno055Enabled) {
    imuPub = rcl_get_zero_initialized_publisher();
    if (rclc_publisher_init(&imuPub, node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
                            "imu_uncalibrated/data_raw", &sensorQos) != RCL_RET_OK) return false;
  }
  batteryPub = rcl_get_zero_initialized_publisher();
  if (rclc_publisher_init(&batteryPub, node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
                          "base/battery_voltage", &sensorQos) != RCL_RET_OK) {
    if constexpr (kBno055Enabled) finish(imuPub, node);
    return false;
  }
  auto qos = rmw_qos_profile_default;
  qos.depth = 1;
  for (unsigned i = 0; i < 4; ++i) {
    auto& pub = tofPubs[i];
    pub = rcl_get_zero_initialized_publisher();
    if (rclc_publisher_init(&pub, node, ROSIDL_GET_MSG_TYPE_SUPPORT(cleany_interfaces, msg, TofFrame),
                            topics[i], &qos) != RCL_RET_OK) {
      for (unsigned j = 0; j < i; ++j) finish(tofPubs[j], node);
      finish(batteryPub, node);
      if constexpr (kBno055Enabled) finish(imuPub, node);
      return false;
    }
    rmw_uros_set_publisher_session_timeout(rcl_publisher_get_rmw_handle(&pub), 5);
  }
  return true;
}

void close(rcl_node_t* node) {
  for (auto& pub : tofPubs) finish(pub, node);
  finish(batteryPub, node);
  if constexpr (kBno055Enabled) finish(imuPub, node);
}

bool publish(bool synchronized) {
  if (!publishBatteryVoltage()) return false;
  if (!synchronized) return true;
  MotionSample sample{};
  if (kBno055Enabled && xQueueReceive(imuQueue, &sample, 0) == pdTRUE) {
    sensor_msgs__msg__Imu imu{};
    if (stamp(imu.header, "imu_link", sample.timestampUs) &&
        esp_timer_get_time() - sample.timestampUs < 100000) {
      imu.orientation_covariance[0] = -1;
      imu.angular_velocity = {sample.gyro[0], sample.gyro[1], sample.gyro[2]};
      imu.linear_acceleration = {sample.acceleration[0], sample.acceleration[1], sample.acceleration[2]};
      if (rcl_publish(&imuPub, &imu, nullptr) != RCL_RET_OK) return false;
    }
  }
  // Bound reliable transport work per executor cycle so a failed link cannot starve wheel commands.
  static unsigned next = 0;
  const unsigned i = next;
  next = (next + 1) % 4;
  TofSample tof{};
  if (takeTof(i, tof)) {
    auto& frame = tof.frame;
    if (stamp(frame.header, frames[i], tof.acquisitionUs) &&
        rcl_publish(&tofPubs[i], &frame, nullptr) != RCL_RET_OK) return false;
  }
  return true;
}
}  // namespace cleany::sensors
