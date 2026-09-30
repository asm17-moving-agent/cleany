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
#include "wheel_velocity_controller.hpp"
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
constexpr char kTag[] = "motor_controller";
constexpr ledc_mode_t kPwmMode = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_t kPwmTimer = LEDC_TIMER_0;
constexpr uint32_t kPwmFrequencyHz = 20000;
constexpr ledc_timer_bit_t kPwmResolution = LEDC_TIMER_8_BIT;
constexpr uint32_t kMotorControlPeriodMs = 5;
constexpr int kOutputSlewStepPercent = 1;
constexpr float kVelocityFilterTimeConstantSeconds = 0.05F;
constexpr float kMaximumTargetVelocityRadPerSecond = 10.0F;
constexpr int64_t kMicroRosWatchdogUs = 250000;
constexpr uint16_t kMicroRosWatchdogMs = 250;

struct Motor {
  gpio_num_t pwmPin;
  gpio_num_t directionPin;
  ledc_channel_t channel;
  int polarity;
  int encoderPolarity;
  cleany::MotorCommandFilter commandFilter;
  cleany::WheelVelocityController velocityController;
  int outputSpeed = 0;
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

std::array<Motor, 4> motors = {{
    {GPIO_NUM_2, GPIO_NUM_1, LEDC_CHANNEL_0, -1, 1, {}, {}},    // M1 FL
    {GPIO_NUM_12, GPIO_NUM_11, LEDC_CHANNEL_1, 1, -1, {}, {}},  // M2 FR
    {GPIO_NUM_5, GPIO_NUM_6, LEDC_CHANNEL_2, 1, -1, {}, {}},    // M3 RR
    {GPIO_NUM_16, GPIO_NUM_17, LEDC_CHANNEL_3, -1, 1, {}, {}},  // M4 RL
}};
std::array<Encoder, 4> encoders = {{
    {GPIO_NUM_10, GPIO_NUM_9}, {GPIO_NUM_14, GPIO_NUM_13},
    {GPIO_NUM_7, GPIO_NUM_15}, {GPIO_NUM_18, GPIO_NUM_8},
}};
SemaphoreHandle_t motorMutex;
portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;
QueueHandle_t wheelCommandQueue;
portMUX_TYPE microStopMux = portMUX_INITIALIZER_UNLOCKED;
bool microStopPending = false;
cleany::WheelCommandGate* wheelGate = nullptr;
uint32_t microBootId = 0, microStateSequence = 0, microLastSequence = 0;
uint32_t microFaultBits = 0;
int64_t microLastAcceptedUs = 0;
bool microArmed = false;
uint32_t microSessionId = 0;

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

esp_err_t applyMotorSpeedLocked(size_t i, int speed) {
  Motor& m = motors[i];
  if (speed == m.outputSpeed) return ESP_OK;
  const int raw = speed * m.polarity;
  const uint32_t duty = static_cast<uint32_t>(std::abs(speed) * 255 / 100);
  const int direction = raw > 0 ? 1 : 0;
  esp_err_t result = ESP_OK;
  if (speed == 0) {
    result = ledc_set_duty(kPwmMode, m.channel, 0);
    if (result == ESP_OK) result = ledc_update_duty(kPwmMode, m.channel);
  } else {
    if (direction != m.directionLevel) {
      if (m.outputSpeed != 0) return ESP_ERR_INVALID_STATE;
      result = gpio_set_level(m.directionPin, direction);
      if (result == ESP_OK) m.directionLevel = direction;
    }
    if (result == ESP_OK) result = ledc_set_duty(kPwmMode, m.channel, duty);
    if (result == ESP_OK) result = ledc_update_duty(kPwmMode, m.channel);
  }
  if (result == ESP_OK) m.outputSpeed = speed;
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
      .freq_hz = kPwmFrequencyHz, .clk_cfg = LEDC_AUTO_CLK, .deconfigure = false};
  r = ledc_timer_config(&timer);
  if (r != ESP_OK) return r;
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
  for (size_t i = 0; i < motors.size(); ++i) {
    auto& m = motors[i];
    m.commandFilter.forceStop();
    m.velocityController.reset();
    m.commandedVelocityRadPerSecond = 0.0F;
    (void)applyMotorSpeedLocked(i, 0);
  }
}
bool takeMicroRosStop() {
  portENTER_CRITICAL(&microStopMux);
  const bool pending = microStopPending;
  microStopPending = false;
  portEXIT_CRITICAL(&microStopMux);
  return pending;
}
void processMicroRosCommandLocked(int64_t nowUs) {
  cleany::WheelCommand command{};
  const bool haveCommand = xQueueReceive(wheelCommandQueue, &command, 0) == pdTRUE;
  const bool stop = takeMicroRosStop();
  if (stop) {
    (void)wheelGate->accept({1, 0, 0, 0, 0, cleany::WheelMode::kStop, {}},
                            static_cast<uint64_t>(nowUs));
    stopMicroRosMotorsLocked();
    microArmed = false;
    microSessionId = 0;
  } else if (haveCommand &&
             wheelGate->accept(command, static_cast<uint64_t>(nowUs))) {
    microLastSequence = wheelGate->lastSequence();
    microSessionId = wheelGate->sessionId();
    microArmed = wheelGate->armed();
    if (command.mode == cleany::WheelMode::kBeginSession ||
        command.mode == cleany::WheelMode::kArm) {
      stopMicroRosMotorsLocked();
      microLastAcceptedUs = command.mode == cleany::WheelMode::kArm ? nowUs : 0;
      if (command.mode == cleany::WheelMode::kBeginSession) microFaultBits = 0;
    } else if (command.mode == cleany::WheelMode::kVelocity) {
      const auto& target = wheelGate->target();
      for (size_t wire = 0; wire < cleany::kWireToMotorIndex.size(); ++wire) {
        const size_t i = cleany::kWireToMotorIndex[wire];
        const int percent = static_cast<int>(std::lround(target[wire] * 10.0F));
        motors[i].commandFilter.setTargetSpeed(percent);
      }
      microLastAcceptedUs = nowUs;
    }
  }
  if (wheelGate->watchdog(static_cast<uint64_t>(nowUs))) {
    stopMicroRosMotorsLocked();
    microArmed = false;
    microSessionId = 0;
    microFaultBits |= 1U;
  }
}
void motorControlTask(void*) {
  TickType_t wake = xTaskGetTickCount();
  while (true) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(kMotorControlPeriodMs));
    const auto counts = readEncoderCounts();
    if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(5)) != pdTRUE) continue;
    const int64_t nowUs = esp_timer_get_time();
    processMicroRosCommandLocked(nowUs);
    for (size_t i = 0; i < motors.size(); ++i) {
      Motor& m = motors[i];
      updateMotorVelocityLocked(m, counts[i], nowUs);
      const int commandPercent = m.commandFilter.step(m.feedbackVelocityRadPerSecond, nowUs);
      m.commandedVelocityRadPerSecond =
          static_cast<float>(commandPercent) * kMaximumTargetVelocityRadPerSecond / 100.0F;
      const float elapsed = static_cast<float>(kMotorControlPeriodMs) / 1000.0F;
      const int requested = static_cast<int>(std::lround(m.velocityController.update(
          m.commandedVelocityRadPerSecond, m.feedbackVelocityRadPerSecond, elapsed)));
      const int output = cleany::moveToward(m.outputSpeed, requested, kOutputSlewStepPercent);
      if (output != m.outputSpeed && applyMotorSpeedLocked(i, output) != ESP_OK) {
        ESP_LOGE(kTag, "Failed to apply motor %u speed", static_cast<unsigned>(i + 1));
        m.commandFilter.forceStop();
        m.velocityController.reset();
        m.commandedVelocityRadPerSecond = 0.0F;
        (void)applyMotorSpeedLocked(i, 0);
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
    portENTER_CRITICAL(&microStopMux);
    microStopPending = true;
    portEXIT_CRITICAL(&microStopMux);
    return;
  }
  cleany::WheelCommand c{};
  c.protocolVersion = in->protocol_version; c.bootId = in->boot_id;
  c.sessionId = in->session_id; c.sequence = in->sequence;
  c.validUntilUs = in->valid_until_us; c.mode = static_cast<cleany::WheelMode>(in->mode);
  for (size_t i = 0; i < c.velocity.size(); ++i) c.velocity[i] = in->velocity_rad_s[i];
  (void)xQueueOverwrite(wheelCommandQueue, &c);
}
bool fillWheelState(cleany_base_interfaces__msg__WheelState* s) {
  const auto counts = readEncoderCounts();
  const int64_t now = esp_timer_get_time();
  s->protocol_version = 1; s->boot_id = microBootId;
  s->timestamp_us = static_cast<uint64_t>(now);
  s->counts_per_revolution = cleany::kEncoderCountsPerOutputRevolution;
  s->max_velocity_rad_s = kMaximumTargetVelocityRadPerSecond;
  s->watchdog_ms = kMicroRosWatchdogMs; s->sequence = microStateSequence++;
  if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(5)) != pdTRUE) return false;
  s->session_id = microSessionId; s->last_command_sequence = microLastSequence;
  s->armed = microArmed; s->fault_bits = microFaultBits;
  s->command_age_ms = microLastAcceptedUs == 0 ? 65535 :
      static_cast<uint16_t>(std::min<int64_t>(std::max<int64_t>(0, (now-microLastAcceptedUs)/1000),65535));
  bool reverse = false;
  for (size_t wire = 0; wire < cleany::kWireToMotorIndex.size(); ++wire) {
    const size_t i = cleany::kWireToMotorIndex[wire];
    const Motor& m = motors[i];
    s->encoder_counts[wire] = logicalEncoderCount(i, counts[i]);
    s->velocity_rad_s[wire] = m.feedbackVelocityRadPerSecond;
    s->target_rad_s[wire] = static_cast<float>(m.commandFilter.targetSpeed()) / 10.0F;
    s->commanded_rad_s[wire] = m.commandedVelocityRadPerSecond;
    s->pwm_percent[wire] = static_cast<int8_t>(m.outputSpeed);
    reverse = reverse || m.commandFilter.waitingForReverse();
  }
  if (!microArmed)
    s->control_mode = microFaultBits
        ? static_cast<uint8_t>(cleany_base_interfaces__msg__WheelState__WATCHDOG_STOP)
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
                            rcl_publisher_t* pub, rcl_subscription_t* sub,
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
  if (rclc_subscription_init(sub, node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(cleany_base_interfaces, msg, WheelCommand),
      "base/wheel_command", &qos) != RCL_RET_OK) {
    ignoreRclCleanupResult(rcl_publisher_fini(pub,node)); ignoreRclCleanupResult(rcl_node_fini(node));
    (void)rclc_support_fini(support); return false;
  }
  if (rclc_executor_init(ex, &support->context, 1, a) != RCL_RET_OK) {
    ignoreRclCleanupResult(rcl_subscription_fini(sub,node)); ignoreRclCleanupResult(rcl_publisher_fini(pub,node));
    ignoreRclCleanupResult(rcl_node_fini(node)); (void)rclc_support_fini(support); return false;
  }
  if (rclc_executor_add_subscription(ex, sub, command, wheelCommandCallback, ON_NEW_DATA) != RCL_RET_OK) {
    (void)rclc_executor_fini(ex); ignoreRclCleanupResult(rcl_subscription_fini(sub,node));
    ignoreRclCleanupResult(rcl_publisher_fini(pub,node)); ignoreRclCleanupResult(rcl_node_fini(node));
    (void)rclc_support_fini(support); return false;
  }
  return true;
}
void destroyMicroRosEntities(rclc_support_t* support, rcl_node_t* node, rcl_publisher_t* pub,
                             rcl_subscription_t* sub, rclc_executor_t* ex) {
  (void)rclc_executor_fini(ex); ignoreRclCleanupResult(rcl_subscription_fini(sub,node));
  ignoreRclCleanupResult(rcl_publisher_fini(pub,node)); ignoreRclCleanupResult(rcl_node_fini(node));
  (void)rclc_support_fini(support);
}
void microRosTask(void*) {
  rcl_allocator_t allocator = rcl_get_default_allocator();
  cleany_base_interfaces__msg__WheelCommand command{};
  cleany_base_interfaces__msg__WheelState state{};
  if (rmw_uros_set_custom_transport(true, &nativeUsbTransport, microTransportOpen,
      microTransportClose, microTransportWrite, microTransportRead) != RMW_RET_OK) {
    vTaskDelete(nullptr); return;
  }
  while (true) {
    if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
      portENTER_CRITICAL(&microStopMux); microStopPending = true; portEXIT_CRITICAL(&microStopMux);
      vTaskDelay(pdMS_TO_TICKS(500)); continue;
    }
    rclc_support_t support{}; rcl_node_t node = rcl_get_zero_initialized_node();
    rcl_publisher_t pub{}; rcl_subscription_t sub{};
    rclc_executor_t ex = rclc_executor_get_zero_initialized_executor();
    bool connected = createMicroRosEntities(&allocator,&support,&node,&pub,&sub,&ex,&command);
    const bool ready = connected; int64_t lastState = 0, lastPing = esp_timer_get_time();
    while (connected) {
      const rcl_ret_t spin = rclc_executor_spin_some(&ex, RCL_MS_TO_NS(2));
      if (spin != RCL_RET_OK && spin != RCL_RET_TIMEOUT) { connected = false; break; }
      const int64_t now = esp_timer_get_time();
      if (now-lastPing >= 100000) {
        if (rmw_uros_ping_agent(20,1) != RMW_RET_OK) { connected = false; break; }
        lastPing = esp_timer_get_time();
      }
      if (lastState == 0 || now-lastState >= 20000) {
        if (!fillWheelState(&state)) { vTaskDelay(pdMS_TO_TICKS(2)); continue; }
        if (rcl_publish(&pub,&state,nullptr) != RCL_RET_OK) { connected = false; break; }
        lastState = now;
      }
      vTaskDelay(pdMS_TO_TICKS(2));
    }
    portENTER_CRITICAL(&microStopMux); microStopPending = true; portEXIT_CRITICAL(&microStopMux);
    if (ready) destroyMicroRosEntities(&support,&node,&pub,&sub,&ex);
    vTaskDelay(pdMS_TO_TICKS(250));
  }
}
}  // namespace

extern "C" void app_main() {
  motorMutex = xSemaphoreCreateMutex();
  ESP_ERROR_CHECK(motorMutex == nullptr ? ESP_ERR_NO_MEM : ESP_OK);
  ESP_ERROR_CHECK(configureMotors());
  ESP_ERROR_CHECK(configureEncoders());
  microBootId = esp_random();
  if (microBootId == 0) microBootId = 1;
  wheelGate = new cleany::WheelCommandGate(
      {microBootId, static_cast<uint32_t>(kMicroRosWatchdogUs), 10.0F});
  ESP_ERROR_CHECK(wheelGate == nullptr ? ESP_ERR_NO_MEM : ESP_OK);
  wheelCommandQueue = xQueueCreate(1, sizeof(cleany::WheelCommand));
  ESP_ERROR_CHECK(wheelCommandQueue == nullptr ? ESP_ERR_NO_MEM : ESP_OK);
  ESP_ERROR_CHECK(xTaskCreate(motorControlTask, "motor_control", 3072, nullptr, 6, nullptr) == pdPASS
                      ? ESP_OK : ESP_ERR_NO_MEM);
  ESP_ERROR_CHECK(xTaskCreate(microRosTask, "micro_ros", 8192, nullptr, 4, nullptr) == pdPASS
                      ? ESP_OK : ESP_ERR_NO_MEM);
}
