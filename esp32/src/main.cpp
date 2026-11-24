#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <new>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/usb_serial_jtag.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "modular_int32.hpp"
#include "motor_command_filter.hpp"
#include "wheel_command_gate.hpp"
#include "wheel_order.hpp"
#include "wheel_duty.hpp"
#include "wheel_friction_values.hpp"
#include "cleany_base_interfaces/msg/wheel_drive_state.h"
#include "wheel_velocity_controller.hpp"
#include "board_pins.hpp"
#include "board_sensors.hpp"
#include "usb_stream_transport.hpp"
#include "rcl/rcl.h"
#include "rclc/rclc.h"
#include "rclc/executor.h"
#include "rmw_microros/rmw_microros.h"
#include "rmw_microros/custom_transport.h"
#include "cleany_base_interfaces/msg/wheel_command.h"
#include "cleany_base_interfaces/msg/wheel_state.h"
#include "cleany_base_interfaces/msg/detail/wheel_command__type_support.h"
#include "cleany_base_interfaces/msg/detail/wheel_state__type_support.h"

namespace {
using cleany::board::wheels;
constexpr char kTag[] = "motor_controller";
constexpr ledc_mode_t kPwmMode = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_t kPwmTimer = LEDC_TIMER_0;
constexpr uint32_t kPwmFrequencyHz = 20000;
constexpr ledc_timer_bit_t kPwmResolution = static_cast<ledc_timer_bit_t>(cleany::kPwmBits);
constexpr uint32_t kMotorControlPeriodMs = 5;
// Conservative command rampup: step 1% every 20ms = 50% per second (0.6s to 0.3 m/s)
constexpr int64_t kCommandSlewStepIntervalUs = 20000;
constexpr cleany::MotorFilterConfig kDefaultMotorFilterConfig{
    1, kCommandSlewStepIntervalUs, 0.5F, 50000, 3};
constexpr float kVelocityFilterTimeConstantSeconds = 0.02F;
constexpr float kMaximumTargetVelocityRadPerSecond = 10.0F;
constexpr uint16_t kMicroRosWatchdogMs = 250;

struct Motor {
  gpio_num_t pwmPin;
  gpio_num_t directionPin;
  ledc_channel_t channel;
  int polarity;
  int encoderPolarity;
  cleany::MotorCommandFilter commandFilter;
  cleany::WheelVelocityController velocityController;
  int dutyTicks = 0;
  float requestedPercent = 0.0F;
  int32_t startCount = 0;
  int32_t firstUs = -1, fifthUs = -1;
  int onsetTicks = 0;
  float commandedVelocityRadPerSecond = 0.0F;
  int directionLevel = 0;
  int32_t lastEncoderCount = 0;
  int64_t lastEncoderUpdateUs = 0;
  float feedbackVelocityRadPerSecond = 0.0F;
};
struct Encoder {
  gpio_num_t pinA, pinB;
  volatile int32_t count = 0;
  volatile uint8_t previousState = 0;
};

// Per-wheel PI + FF settings: FF scale (rad/s), Kp, Ki, PWM limit (%).
// Ground-loaded calibrated on floor across 0.5m surge & sway to equalize per-wheel velocity:
constexpr cleany::WheelVelocityControllerConfig kFlControllerConfig{
    11.9352F, 6.058F, 12.118F, 100.0F};
constexpr cleany::WheelVelocityControllerConfig kFrControllerConfig{
    12.2596F, 6.030F, 12.058F, 100.0F};
constexpr cleany::WheelVelocityControllerConfig kRlControllerConfig{
    11.7678F, 6.246F, 12.492F, 100.0F};
constexpr cleany::WheelVelocityControllerConfig kRrControllerConfig{
    11.3872F, 6.154F, 12.309F, 100.0F};

// The resolution-only comparison uses the same output path and telemetry.
constexpr const cleany::WheelFriction* friction(const cleany::WheelFriction& values) {
#ifdef CLEANY_WHEEL_RESOLUTION_ONLY
  (void)values;
  return nullptr;
#else
  return &values;
#endif
}

std::array<Motor, 4> motors = {{
    {wheels[0].pwm, wheels[0].direction, LEDC_CHANNEL_0, -1, 1,
     cleany::MotorCommandFilter{kDefaultMotorFilterConfig},
     cleany::WheelVelocityController{kFlControllerConfig, friction(cleany::kFlFriction)}},   // M1 FL
    {wheels[1].pwm, wheels[1].direction, LEDC_CHANNEL_1, 1, -1,
     cleany::MotorCommandFilter{kDefaultMotorFilterConfig},
     cleany::WheelVelocityController{kFrControllerConfig, friction(cleany::kFrFriction)}},   // M2 FR
    {wheels[2].pwm, wheels[2].direction, LEDC_CHANNEL_2, 1, -1,
     cleany::MotorCommandFilter{kDefaultMotorFilterConfig},
     cleany::WheelVelocityController{kRrControllerConfig, friction(cleany::kRrFriction)}},   // M3 RR
    {wheels[3].pwm, wheels[3].direction, LEDC_CHANNEL_3, -1, 1,
     cleany::MotorCommandFilter{kDefaultMotorFilterConfig},
     cleany::WheelVelocityController{kRlControllerConfig, friction(cleany::kRlFriction)}},   // M4 RL
}};
std::array<Encoder, 4> encoders = {{
    {wheels[0].encoderA, wheels[0].encoderB},
    {wheels[1].encoderA, wheels[1].encoderB},
    {wheels[2].encoderA, wheels[2].encoderB},
    {wheels[3].encoderA, wheels[3].encoderB},
}};
SemaphoreHandle_t motorMutex;
portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;
QueueHandle_t wheelCommandQueue;
uint64_t targetReceivedUs = 0;
uint32_t firstTargetSequence = 0;
uint32_t pwmFrequencyHz = 0;
portMUX_TYPE microStopMux = portMUX_INITIALIZER_UNLOCKED;
bool microStopPending = false;
uint32_t microStopSequence = 0;
bool microStopHasSequence = false;
cleany::WheelCommandGate* wheelGate = nullptr;
uint32_t microBootId = 0, microStateSequence = 0;

constexpr int8_t kEncoderTable[16] = {
    0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0,
};

uint8_t readEncoderState(const Encoder& e) {
  return (static_cast<uint8_t>(gpio_get_level(e.pinA)) << 1) |
         static_cast<uint8_t>(gpio_get_level(e.pinB));
}
void updateEncoder(void* arg) {
  auto* e = static_cast<Encoder*>(arg);
  const uint8_t state = readEncoderState(*e);
  portENTER_CRITICAL_ISR(&encoderMux);
  e->count = cleany::modularAdd(
      e->count, static_cast<int32_t>(kEncoderTable[(e->previousState << 2) | state]));
  e->previousState = state;
  portEXIT_CRITICAL_ISR(&encoderMux);
}
std::array<int32_t, 4> readEncoderCounts() {
  std::array<int32_t, 4> result{};
  portENTER_CRITICAL(&encoderMux);
  for (size_t i = 0; i < encoders.size(); ++i) result[i] = encoders[i].count;
  portEXIT_CRITICAL(&encoderMux);
  return result;
}
int32_t logicalEncoderCount(size_t i, int32_t raw) {
  return motors[i].encoderPolarity < 0 ? cleany::modularDifference(0, raw) : raw;
}

esp_err_t applyMotorDutyLocked(size_t i, int speed) {
  Motor& m = motors[i];
  if (speed == m.dutyTicks) return ESP_OK;
  const int raw = speed * m.polarity;
  const uint32_t duty = static_cast<uint32_t>(std::abs(speed));
  const int direction = raw > 0 ? 1 : 0;
  esp_err_t result = ESP_OK;
  if (speed == 0) {
    result = ledc_set_duty(kPwmMode, m.channel, 0);
    if (result == ESP_OK) result = ledc_update_duty(kPwmMode, m.channel);
  } else {
    if (direction != m.directionLevel) {
      if (m.dutyTicks != 0) return ESP_ERR_INVALID_STATE;
      result = gpio_set_level(m.directionPin, direction);
      if (result == ESP_OK) m.directionLevel = direction;
    }
    if (result == ESP_OK) result = ledc_set_duty(kPwmMode, m.channel, duty);
    if (result == ESP_OK) result = ledc_update_duty(kPwmMode, m.channel);
  }
  if (result == ESP_OK) m.dutyTicks = speed;
  return result;
}

esp_err_t configureMotors() {
  uint64_t mask = 0;
  for (const Motor& m : motors) mask |= 1ULL << m.directionPin;
  const gpio_config_t direction = {.pin_bit_mask = mask, .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE, .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE};
  esp_err_t r = gpio_config(&direction);
  if (r != ESP_OK) return r;
  const ledc_timer_config_t timer = {.speed_mode = kPwmMode,
      .duty_resolution = kPwmResolution, .timer_num = kPwmTimer,
      .freq_hz = kPwmFrequencyHz, .clk_cfg = LEDC_USE_APB_CLK, .deconfigure = false};
  r = ledc_timer_config(&timer);
  if (r != ESP_OK) return r;
  pwmFrequencyHz = ledc_get_freq(kPwmMode, kPwmTimer);
  if (pwmFrequencyHz != kPwmFrequencyHz) return ESP_ERR_INVALID_STATE;
  for (const Motor& m : motors) {
    const ledc_channel_config_t channel = {.gpio_num = m.pwmPin,
        .speed_mode = kPwmMode, .channel = m.channel,
        .intr_type = LEDC_INTR_DISABLE, .timer_sel = kPwmTimer, .duty = 0,
        .hpoint = 0, .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD, .flags = {}};
    r = ledc_channel_config(&channel);
    if (r != ESP_OK) return r;
    gpio_set_level(m.directionPin, 0);
  }
  return ESP_OK;
}
esp_err_t configureEncoders() {
  uint64_t mask = 0;
  for (const Encoder& e : encoders) mask |= (1ULL << e.pinA) | (1ULL << e.pinB);
  const gpio_config_t config = {.pin_bit_mask = mask, .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE, .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_ANYEDGE};
  esp_err_t r = gpio_config(&config);
  if (r != ESP_OK) return r;
  r = gpio_install_isr_service(0);
  if (r != ESP_OK) return r;
  for (Encoder& e : encoders) {
    e.previousState = readEncoderState(e);
    r = gpio_isr_handler_add(e.pinA, updateEncoder, &e);
    if (r == ESP_OK) r = gpio_isr_handler_add(e.pinB, updateEncoder, &e);
    if (r != ESP_OK) return r;
  }
  return ESP_OK;
}
void updateMotorVelocityLocked(Motor& m, int32_t count, int64_t nowUs) {
  if (m.lastEncoderUpdateUs == 0) {
    m.lastEncoderCount = count;
    m.lastEncoderUpdateUs = nowUs;
    return;
  }
  const int64_t elapsedUs = nowUs - m.lastEncoderUpdateUs;
  if (elapsedUs <= 0) return;
  const int32_t delta = cleany::modularDifference(count, m.lastEncoderCount);
  const float elapsed = static_cast<float>(elapsedUs) / 1000000.0F;
  const float raw = cleany::encoderVelocityRadPerSecond(delta, elapsed);
  const float logical = raw * static_cast<float>(m.encoderPolarity);
  const float alpha = elapsed / (kVelocityFilterTimeConstantSeconds + elapsed);
  m.feedbackVelocityRadPerSecond += alpha * (logical - m.feedbackVelocityRadPerSecond);
  m.lastEncoderCount = count;
  m.lastEncoderUpdateUs = nowUs;
}
void stopMicroRosMotorsLocked() {
  targetReceivedUs = 0;
  for (size_t i = 0; i < motors.size(); ++i) {
    auto& m = motors[i];
    m.requestedPercent = 0.0F;
    m.commandFilter.forceStop();
    m.velocityController.reset();
    m.commandedVelocityRadPerSecond = 0.0F;
    (void)applyMotorDutyLocked(i, 0);
  }
}
bool sequenceNewer(uint32_t value, uint32_t reference) {
  const uint32_t delta = value - reference;
  return delta != 0 && delta < 0x80000000U;
}
void requestMicroRosStop(bool sequenced, uint32_t sequence = 0) {
  portENTER_CRITICAL(&microStopMux);
  microStopPending = true;
  if (sequenced && (!microStopHasSequence ||
                    sequenceNewer(sequence, microStopSequence))) {
    microStopSequence = sequence;
    microStopHasSequence = true;
  }
  portEXIT_CRITICAL(&microStopMux);
}
bool takeMicroRosStop(bool& sequenced, uint32_t& sequence) {
  portENTER_CRITICAL(&microStopMux);
  const bool pending = microStopPending;
  sequenced = microStopHasSequence;
  sequence = microStopSequence;
  microStopPending = false;
  microStopHasSequence = false;
  portEXIT_CRITICAL(&microStopMux);
  return pending;
}
void processMicroRosCommandLocked(int64_t nowUs, bool allowMotion,
                                 const std::array<int32_t, 4>& counts) {
  if (wheelGate->expire(static_cast<uint64_t>(nowUs))) {
    stopMicroRosMotorsLocked();
  }
  cleany::WheelCommand command{};
  const bool haveCommand = xQueueReceive(wheelCommandQueue, &command, 0) == pdTRUE;
  uint32_t stopSequence = 0;
  bool sequencedStop = false;
  const bool stop = takeMicroRosStop(sequencedStop, stopSequence);
  const auto previousTarget = wheelGate->target();
  if (stop) {
    xQueueReset(wheelCommandQueue);
    if (sequencedStop)
      (void)wheelGate->accept({2, stopSequence, 0, cleany::WheelMode::kStop, {}},
                              static_cast<uint64_t>(nowUs));
    else wheelGate->disconnect();
    stopMicroRosMotorsLocked();
  } else if (haveCommand && allowMotion &&
             wheelGate->accept(command, static_cast<uint64_t>(nowUs))) {
    if (command.mode == cleany::WheelMode::kEnable) {
      stopMicroRosMotorsLocked();
    } else if (command.mode == cleany::WheelMode::kVelocity) {
      const auto& target = wheelGate->target();
      const bool newTarget = targetReceivedUs == 0 || previousTarget != target;
      if (newTarget) {
        targetReceivedUs = command.receivedUs;
        firstTargetSequence = command.sequence;
      }
      for (size_t wire = 0; wire < cleany::kWireToMotorIndex.size(); ++wire) {
        const size_t i = cleany::kWireToMotorIndex[wire];
        const int percent = static_cast<int>(std::lround(target[wire] * 10.0F));
        motors[i].commandFilter.setTargetSpeed(percent);
        if (newTarget) {
          Motor& m = motors[i];
          m.startCount = logicalEncoderCount(i, counts[i]);
          m.firstUs = m.fifthUs = -1;
          m.onsetTicks = 0;
        }
      }
    }
  }
}
void motorControlTask(void*) {
  TickType_t wake = xTaskGetTickCount();
  while (true) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(kMotorControlPeriodMs));
    if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(5)) != pdTRUE) continue;
    const auto counts = readEncoderCounts();
    const int64_t nowUs = esp_timer_get_time();
    processMicroRosCommandLocked(nowUs, true, counts);
    for (size_t i = 0; i < motors.size(); ++i) {
      Motor& m = motors[i];
      const bool still = counts[i] == m.lastEncoderCount && m.dutyTicks == 0;
      updateMotorVelocityLocked(m, counts[i], nowUs);
      const int direction = cleany::speedSign(m.commandFilter.targetSpeed());
      if (targetReceivedUs && direction && m.fifthUs < 0) {
        const int64_t progress = direction * static_cast<int64_t>(
            cleany::modularDifference(logicalEncoderCount(i, counts[i]), m.startCount));
        const int32_t age = std::min<int64_t>(nowUs - targetReceivedUs, INT32_MAX);
        if (progress >= 1 && m.firstUs < 0) {
          m.firstUs = age;
          m.onsetTicks = m.dutyTicks;
        }
        if (progress >= 5) m.fifthUs = age;
      }
      const int commandPercent = m.commandFilter.step(m.feedbackVelocityRadPerSecond, nowUs, still);
      m.commandedVelocityRadPerSecond =
          static_cast<float>(commandPercent) * kMaximumTargetVelocityRadPerSecond / 100.0F;
      const float elapsed = static_cast<float>(kMotorControlPeriodMs) / 1000.0F;
      m.requestedPercent = m.velocityController.update(
          m.commandedVelocityRadPerSecond, m.feedbackVelocityRadPerSecond, elapsed,
          logicalEncoderCount(i, counts[i]), nowUs);
      const int requested = cleany::dutyTicks(m.requestedPercent);
      const int output = cleany::moveToward(m.dutyTicks, requested, cleany::kDutySlewTicks);
      if (output != m.dutyTicks && applyMotorDutyLocked(i, output) != ESP_OK) {
        ESP_LOGE(kTag, "Failed to apply motor %u speed", static_cast<unsigned>(i + 1));
        m.commandFilter.forceStop();
        m.velocityController.reset();
        m.commandedVelocityRadPerSecond = 0.0F;
        (void)applyMotorDutyLocked(i, 0);
      }
    }
    xSemaphoreGive(motorMutex);
  }
}

class NativeUsbPort final : public cleany::UsbStreamPort {
 public:
  bool open() override {
    usb_serial_jtag_driver_config_t c = {.tx_buffer_size = 2048, .rx_buffer_size = 2048};
    return usb_serial_jtag_driver_install(&c) == ESP_OK;
  }
  void close() override { (void)usb_serial_jtag_driver_uninstall(); }
  int read(uint8_t* d, size_t n, uint32_t ms) override {
    return usb_serial_jtag_read_bytes(d, n, pdMS_TO_TICKS(ms));
  }
  int write(const uint8_t* d, size_t n, uint32_t ms) override {
    return usb_serial_jtag_write_bytes(d, n, pdMS_TO_TICKS(ms));
  }
  uint64_t nowMs() const override { return static_cast<uint64_t>(esp_timer_get_time() / 1000); }
};
NativeUsbPort nativeUsbPort;
cleany::UsbStreamTransport nativeUsbTransport(nativeUsbPort);
bool microTransportOpen(uxrCustomTransport*) { return nativeUsbTransport.open(); }
bool microTransportClose(uxrCustomTransport*) { nativeUsbTransport.close(); return true; }
size_t microTransportWrite(uxrCustomTransport* t, const uint8_t* b, size_t n, uint8_t* e) {
  *e = 0;
  const int written = static_cast<cleany::UsbStreamTransport*>(t->args)->write(b, n, 50);
  if (written < 0) { *e = 1; return 0; }
  return static_cast<size_t>(written);
}
size_t microTransportRead(uxrCustomTransport* t, uint8_t* b, size_t n, int timeout, uint8_t* e) {
  *e = 0;
  const int got = static_cast<cleany::UsbStreamTransport*>(t->args)->read(b, n, std::max(timeout, 0));
  if (got < 0) { *e = 1; return 0; }
  return static_cast<size_t>(got);
}
void wheelCommandCallback(const void* msg) {
  const auto* in = static_cast<const cleany_base_interfaces__msg__WheelCommand*>(msg);
  if (in->mode == cleany_base_interfaces__msg__WheelCommand__STOP) {
    requestMicroRosStop(true, in->sequence);
    return;
  }
  cleany::WheelCommand c{};
  c.receivedUs = esp_timer_get_time();
  c.protocolVersion = in->protocol_version;
  c.sequence = in->sequence;
  c.validUntilUs = in->valid_until_us; c.mode = static_cast<cleany::WheelMode>(in->mode);
  for (size_t i = 0; i < c.velocity.size(); ++i) c.velocity[i] = in->velocity_rad_s[i];
  (void)xQueueOverwrite(wheelCommandQueue, &c);
}
bool fillWheelState(cleany_base_interfaces__msg__WheelState* s,
                    cleany_base_interfaces__msg__WheelDriveState& drive) {
  if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(5)) != pdTRUE) return false;
  const auto counts = readEncoderCounts();
  const int64_t now = esp_timer_get_time();
  s->protocol_version = 2; s->boot_id = microBootId;
  s->timestamp_us = static_cast<uint64_t>(now);
  s->counts_per_revolution = cleany::kEncoderCountsPerOutputRevolution;
  s->max_velocity_rad_s = kMaximumTargetVelocityRadPerSecond;
  s->watchdog_ms = kMicroRosWatchdogMs; s->sequence = microStateSequence++;
  drive.boot_id = microBootId;
  drive.sequence = s->sequence;
  drive.timestamp_us = s->timestamp_us;
  drive.pwm_period_ticks = cleany::kPwmPeriodTicks;
  drive.pwm_frequency_hz = pwmFrequencyHz;
  drive.first_sequence = firstTargetSequence;
  drive.received_us = targetReceivedUs;
  s->last_command_sequence = wheelGate->lastSequence();
  s->enabled = wheelGate->enabled();
  s->fault_bits = wheelGate->timedOut()
      ? cleany_base_interfaces__msg__WheelState__FAULT_COMMAND_TIMEOUT : 0;
  const uint64_t acceptedAt = wheelGate->lastAcceptedUs();
  s->command_age_ms = acceptedAt == 0 ? 65535 :
      static_cast<uint16_t>(std::min<int64_t>(
          std::max<int64_t>(0, now - static_cast<int64_t>(acceptedAt)) / 1000, 65534));
  bool reverse = false;
  for (size_t wire = 0; wire < cleany::kWireToMotorIndex.size(); ++wire) {
    const size_t i = cleany::kWireToMotorIndex[wire];
    const Motor& m = motors[i];
    s->encoder_counts[wire] = logicalEncoderCount(i, counts[i]);
    s->velocity_rad_s[wire] = m.feedbackVelocityRadPerSecond;
    s->target_rad_s[wire] = static_cast<float>(m.commandFilter.targetSpeed()) / 10.0F;
    s->commanded_rad_s[wire] = m.commandedVelocityRadPerSecond;
    s->pwm_percent[wire] = static_cast<int8_t>(std::lround(cleany::dutyPercent(m.dutyTicks)));
    const int logicalDirection = (m.directionLevel ? 1 : -1) * m.polarity;
    drive.duty_ticks[wire] = logicalDirection * static_cast<int>(ledc_get_duty(kPwmMode, m.channel));
    drive.requested_pwm_percent[wire] = m.requestedPercent;
    drive.start_counts[wire] = m.startCount;
    drive.first_us[wire] = m.firstUs;
    drive.fifth_us[wire] = m.fifthUs;
    drive.onset_ticks[wire] = m.onsetTicks;
    reverse = reverse || m.commandFilter.waitingForReverse();
  }
  if (!wheelGate->enabled())
    s->control_mode = wheelGate->timedOut()
        ? static_cast<uint8_t>(cleany_base_interfaces__msg__WheelState__TIMEOUT_STOP)
        : static_cast<uint8_t>(cleany_base_interfaces__msg__WheelState__STOPPED);
  else
    s->control_mode = reverse
        ? static_cast<uint8_t>(cleany_base_interfaces__msg__WheelState__REVERSE_WAIT)
        : static_cast<uint8_t>(cleany_base_interfaces__msg__WheelState__VELOCITY);
  xSemaphoreGive(motorMutex);
  return true;
}
void ignoreRclCleanupResult(rcl_ret_t r) { (void)r; }
bool createMicroRosEntities(rcl_allocator_t* a, rclc_support_t* support, rcl_node_t* node,
                            rcl_publisher_t* pub,
                            rcl_subscription_t* sub,
                            rclc_executor_t* ex,
                            cleany_base_interfaces__msg__WheelCommand* command) {
  if (rclc_support_init(support, 0, nullptr, a) != RCL_RET_OK) return false;
  if (rmw_uros_set_context_entity_destroy_session_timeout(
          rcl_context_get_rmw_context(&support->context), 0) != RMW_RET_OK) {
    (void)rclc_support_fini(support); return false;
  }
  if (rclc_node_init_default(node, "cleany_motor_controller", "", support) != RCL_RET_OK) {
    (void)rclc_support_fini(support); return false;
  }
  rmw_qos_profile_t qos = rmw_qos_profile_sensor_data; qos.depth = 1;
  if (rclc_publisher_init(pub, node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(cleany_base_interfaces, msg, WheelState),
      "base/wheel_state", &qos) != RCL_RET_OK) {
    ignoreRclCleanupResult(rcl_node_fini(node)); (void)rclc_support_fini(support); return false;
  }
  if (!cleany::sensors::open(node, qos)) {
    ignoreRclCleanupResult(rcl_publisher_fini(pub,node));
    ignoreRclCleanupResult(rcl_node_fini(node)); (void)rclc_support_fini(support); return false;
  }
  if (rclc_subscription_init(sub, node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(cleany_base_interfaces, msg, WheelCommand),
      "base/wheel_command", &qos) != RCL_RET_OK) {
    cleany::sensors::close(node); ignoreRclCleanupResult(rcl_publisher_fini(pub,node)); ignoreRclCleanupResult(rcl_node_fini(node));
    (void)rclc_support_fini(support); return false;
  }
  if (rclc_executor_init(ex, &support->context, 1, a) != RCL_RET_OK) {
    ignoreRclCleanupResult(rcl_subscription_fini(sub,node)); cleany::sensors::close(node); ignoreRclCleanupResult(rcl_publisher_fini(pub,node));
    ignoreRclCleanupResult(rcl_node_fini(node)); (void)rclc_support_fini(support); return false;
  }
  if (rclc_executor_add_subscription(ex, sub, command, wheelCommandCallback, ON_NEW_DATA) != RCL_RET_OK) {
    (void)rclc_executor_fini(ex); ignoreRclCleanupResult(rcl_subscription_fini(sub,node));
    cleany::sensors::close(node); ignoreRclCleanupResult(rcl_publisher_fini(pub,node)); ignoreRclCleanupResult(rcl_node_fini(node));
    (void)rclc_support_fini(support); return false;
  }
  return true;
}
void destroyMicroRosEntities(rclc_support_t* support, rcl_node_t* node, rcl_publisher_t* pub,
                             rcl_subscription_t* sub, rclc_executor_t* ex) {
  (void)rclc_executor_fini(ex); ignoreRclCleanupResult(rcl_subscription_fini(sub,node));
  cleany::sensors::close(node); ignoreRclCleanupResult(rcl_publisher_fini(pub,node)); ignoreRclCleanupResult(rcl_node_fini(node));
  (void)rclc_support_fini(support);
}
void microRosTask(void*) {
  rcl_allocator_t allocator = rcl_get_default_allocator();
  cleany_base_interfaces__msg__WheelCommand command{};
  cleany_base_interfaces__msg__WheelState state{};
  cleany_base_interfaces__msg__WheelDriveState drive{};
  if (rmw_uros_set_custom_transport(true, &nativeUsbTransport, microTransportOpen,
      microTransportClose, microTransportWrite, microTransportRead) != RMW_RET_OK) {
    vTaskDelete(nullptr); return;
  }
  while (true) {
    if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
      requestMicroRosStop(false);
      vTaskDelay(pdMS_TO_TICKS(500)); continue;
    }
    rclc_support_t support{}; rcl_node_t node = rcl_get_zero_initialized_node();
    rcl_publisher_t pub{}; rcl_subscription_t sub{};
    rclc_executor_t ex = rclc_executor_get_zero_initialized_executor();
    bool connected = createMicroRosEntities(&allocator,&support,&node,&pub,&sub,&ex,&command);
    const bool ready = connected;
    rcl_publisher_t drivePub = rcl_get_zero_initialized_publisher();
    rmw_qos_profile_t qos = rmw_qos_profile_sensor_data;
    qos.depth = 1;
    const bool driveReady = ready && rclc_publisher_init(&drivePub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(cleany_base_interfaces, msg, WheelDriveState),
        "base/wheel_drive_state", &qos) == RCL_RET_OK;
    connected = driveReady;
    int64_t nextStateUs = esp_timer_get_time(), lastPing = nextStateUs;
    bool clockSynchronized = false;
    int64_t nextClockSyncUs = 0;
    while (connected) {
      const rcl_ret_t spin = rclc_executor_spin_some(&ex, RCL_MS_TO_NS(2));
      if (spin != RCL_RET_OK && spin != RCL_RET_TIMEOUT) { connected = false; break; }
      const int64_t now = esp_timer_get_time();
      if (!clockSynchronized && now >= nextClockSyncUs) {
        clockSynchronized = rmw_uros_sync_session(20) == RMW_RET_OK;
        nextClockSyncUs = esp_timer_get_time() + 1000000;
      }
      if (now-lastPing >= 100000) {
        if (rmw_uros_ping_agent(20,1) != RMW_RET_OK) { connected = false; break; }
        lastPing = esp_timer_get_time();
      }
      if (now >= nextStateUs) {
        if (!fillWheelState(&state, drive)) { vTaskDelay(pdMS_TO_TICKS(2)); continue; }
        if (rcl_publish(&pub,&state,nullptr) != RCL_RET_OK ||
            rcl_publish(&drivePub,&drive,nullptr) != RCL_RET_OK) { connected = false; break; }
        // Keep the 50 Hz phase across executor wakeups and skip missed slots.
        constexpr int64_t periodUs = 20000;
        nextStateUs += ((esp_timer_get_time() - nextStateUs) / periodUs + 1) * periodUs;
      }
      if (!cleany::sensors::publish(clockSynchronized)) { connected = false; break; }
      vTaskDelay(pdMS_TO_TICKS(2));
    }
    requestMicroRosStop(false);
    if (driveReady) ignoreRclCleanupResult(rcl_publisher_fini(&drivePub, &node));
    if (ready) destroyMicroRosEntities(&support,&node,&pub,&sub,&ex);
    vTaskDelay(pdMS_TO_TICKS(250));
  }
}
}  // namespace

extern "C" void app_main() {
  motorMutex = xSemaphoreCreateMutex();
  ESP_ERROR_CHECK(motorMutex == nullptr ? ESP_ERR_NO_MEM : ESP_OK);
#ifdef CLEANY_INTEGRATED
  ESP_ERROR_CHECK(gpio_set_level(cleany::board::ledData, 0));
  ESP_ERROR_CHECK(gpio_set_direction(cleany::board::ledData, GPIO_MODE_OUTPUT));
#endif
  ESP_ERROR_CHECK(configureMotors());
  ESP_ERROR_CHECK(configureEncoders());
  microBootId = esp_random();
  if (microBootId == 0) microBootId = 1;
  wheelGate = new cleany::WheelCommandGate({250000, 10.0F});
  ESP_ERROR_CHECK(wheelGate == nullptr ? ESP_ERR_NO_MEM : ESP_OK);
  wheelCommandQueue = xQueueCreate(1, sizeof(cleany::WheelCommand));
  ESP_ERROR_CHECK(wheelCommandQueue == nullptr ? ESP_ERR_NO_MEM : ESP_OK);
  ESP_ERROR_CHECK(xTaskCreate(motorControlTask, "motor_control", 3072, nullptr, 6, nullptr) == pdPASS
                      ? ESP_OK : ESP_ERR_NO_MEM);
  cleany::sensors::start(microBootId);
  ESP_ERROR_CHECK(xTaskCreate(microRosTask, "micro_ros", 8192, nullptr, 4, nullptr) == pdPASS
                      ? ESP_OK : ESP_ERR_NO_MEM);
}
