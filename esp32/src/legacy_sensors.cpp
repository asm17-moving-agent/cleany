#include "board_sensors.hpp"
#include <algorithm>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "imu_runtime.hpp"
#include "mpu6050.hpp"
#include "rclc/rclc.h"
#include "rmw_microros/rmw_microros.h"
#include "sensor_msgs/sensor_msgs/msg/imu.h"
#include "sensor_msgs/sensor_msgs/msg/detail/imu__type_support.h"

namespace cleany::sensors {
namespace {
constexpr char kTag[] = "motor_controller";
QueueHandle_t imuQueue;
rcl_publisher_t imuPub{};
void imuAcquisitionTask(void*) {
  cleany::Mpu6050 sensor;
  uint32_t retryMs = 250;
  TickType_t wake = xTaskGetTickCount();
  const auto enqueue = [&](cleany::ImuRuntimeValue value = cleany::ImuRuntimeValue{}) {
    (void)xQueueOverwrite(imuQueue, &value);
  };
  while (true) {
    if (sensor.initialize() != ESP_OK) {
      enqueue();
      ESP_LOGW(kTag, "MPU6050 init failed: %s", sensor.lastOperation());
      vTaskDelay(pdMS_TO_TICKS(retryMs));
      retryMs = std::min<uint32_t>(retryMs * 2, 5000);
      continue;
    }
    wake = xTaskGetTickCount();
    while (true) {
      // Poll data-ready faster than the independent 100 Hz sensor clock.
      vTaskDelayUntil(&wake, pdMS_TO_TICKS(2));
      cleany::ImuSample sample{};
      const auto status = sensor.readSample(sample);
      if (status == cleany::Mpu6050ReadStatus::kFresh) {
        retryMs = 250;
        enqueue(cleany::makeImuValue(sample, sensor.identity()));
      } else if (status == cleany::Mpu6050ReadStatus::kError) {
        enqueue();
        ESP_LOGW(kTag, "MPU6050 read failed: %s", sensor.lastOperation());
        sensor.shutdown();
        break;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(retryMs));
    retryMs = std::min<uint32_t>(retryMs * 2, 5000);
  }
}

}  // namespace

void start(uint32_t) {
  imuQueue = xQueueCreate(1, sizeof(ImuRuntimeValue));
  ESP_ERROR_CHECK(imuQueue == nullptr ? ESP_ERR_NO_MEM : ESP_OK);
  ESP_ERROR_CHECK(xTaskCreate(imuAcquisitionTask, "imu_acquire", 6144, nullptr, 2, nullptr)
                  == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
}

bool open(rcl_node_t* node, const rmw_qos_profile_t& qos) {
  imuPub = rcl_get_zero_initialized_publisher();
  return rclc_publisher_init(&imuPub, node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
                             "imu_uncalibrated/data_raw", &qos) == RCL_RET_OK;
}

void close(rcl_node_t* node) {
  const auto result = rcl_publisher_fini(&imuPub, node);
  (void)result;
}

bool publish(bool clockSynchronized) {
  sensor_msgs__msg__Imu imuState{};
  static char imuFrame[] = "imu_link";
  imuState.header.frame_id.data = imuFrame;
  imuState.header.frame_id.size = sizeof(imuFrame) - 1;
  imuState.header.frame_id.capacity = sizeof(imuFrame);
  imuState.orientation_covariance[0] = -1.0;
  static cleany::ImuPublicationGate imuPublicationGate{};
  cleany::ImuRuntimeValue imu{};
  const bool received = xQueueReceive(imuQueue, &imu, 0) == pdTRUE;
  const int64_t imuNow = esp_timer_get_time();
  const bool freshImu = received && imuPublicationGate.consume(imu, imuNow);
  if (!freshImu || !clockSynchronized) {
    return true;
  }
  const int64_t ageUs = imuNow - imu.sample.timestampUs;
  int64_t stampNs = 0;
  if (ageUs < 0 || ageUs > cleany::kImuMaximumAgeUs ||
      !cleany::acquisitionEpochNanoseconds(rmw_uros_epoch_nanos(), ageUs, stampNs)) {
    return true;
  }
  imuState.header.stamp.sec = static_cast<int32_t>(stampNs / 1000000000LL);
  imuState.header.stamp.nanosec = static_cast<uint32_t>(stampNs % 1000000000LL);
  imuState.angular_velocity.x = imu.sample.angularVelocityRadS[0];
  imuState.angular_velocity.y = imu.sample.angularVelocityRadS[1];
  imuState.angular_velocity.z = imu.sample.angularVelocityRadS[2];
  imuState.linear_acceleration.x = imu.sample.accelerationMps2[0];
  imuState.linear_acceleration.y = imu.sample.accelerationMps2[1];
  imuState.linear_acceleration.z = imu.sample.accelerationMps2[2];
  return rcl_publish(&imuPub, &imuState, nullptr) == RCL_RET_OK;
}
}  // namespace cleany::sensors
