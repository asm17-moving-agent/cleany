#include <cstdint>
#include <cstdio>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <unity.h>

#define ASSERT_ESP_OK(expression) TEST_ASSERT_EQUAL_HEX32(ESP_OK, (expression))

extern "C" {

void unityOutputStart(unsigned long baudrate) {
  (void)baudrate;
}

void unityOutputChar(unsigned int character) {
  putchar(static_cast<int>(character));
}

void unityOutputFlush() {
  fflush(stdout);
}

void unityOutputComplete() {
  fflush(stdout);
}

}  // extern "C"

namespace {

constexpr gpio_num_t kEncoderAPin = GPIO_NUM_4;
constexpr gpio_num_t kEncoderBPin = GPIO_NUM_3;
constexpr gpio_num_t kMotorDirectionPin = GPIO_NUM_10;
constexpr gpio_num_t kMotorPwmPin = GPIO_NUM_11;

constexpr ledc_channel_t kPwmChannel = LEDC_CHANNEL_0;
constexpr ledc_timer_t kPwmTimer = LEDC_TIMER_0;
constexpr ledc_mode_t kPwmMode = LEDC_LOW_SPEED_MODE;
constexpr uint32_t kPwmFrequencyHz = 20000;
constexpr ledc_timer_bit_t kPwmResolution = LEDC_TIMER_8_BIT;
constexpr uint32_t kTestDuty = 128;
constexpr int32_t kCountsPerRevolution = 3172;
constexpr int32_t kTargetCounts = kCountsPerRevolution / 4;
constexpr uint32_t kMotionTimeoutMs = 3000;

volatile int32_t encoderCount = 0;
volatile uint8_t previousState = 0;
portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;

constexpr int8_t kEncoderTable[16] = {
    0, -1, 1, 0,
    1, 0, 0, -1,
    -1, 0, 0, 1,
    0, 1, -1, 0,
};

uint8_t readEncoderState() {
  return (static_cast<uint8_t>(gpio_get_level(kEncoderAPin)) << 1) |
         static_cast<uint8_t>(gpio_get_level(kEncoderBPin));
}

void updateEncoder(void*) {
  const uint8_t currentState = readEncoderState();
  const uint8_t tableIndex = (previousState << 2) | currentState;

  portENTER_CRITICAL_ISR(&encoderMux);
  encoderCount += kEncoderTable[tableIndex];
  previousState = currentState;
  portEXIT_CRITICAL_ISR(&encoderMux);
}

int32_t readEncoder() {
  portENTER_CRITICAL(&encoderMux);
  const int32_t count = encoderCount;
  portEXIT_CRITICAL(&encoderMux);
  return count;
}

esp_err_t configureMotorPwm() {
  const ledc_timer_config_t timerConfig = {
      .speed_mode = kPwmMode,
      .duty_resolution = kPwmResolution,
      .timer_num = kPwmTimer,
      .freq_hz = kPwmFrequencyHz,
      .clk_cfg = LEDC_AUTO_CLK,
      .deconfigure = false,
  };
  esp_err_t result = ledc_timer_config(&timerConfig);
  if (result != ESP_OK) {
    return result;
  }

  const ledc_channel_config_t channelConfig = {
      .gpio_num = kMotorPwmPin,
      .speed_mode = kPwmMode,
      .channel = kPwmChannel,
      .intr_type = LEDC_INTR_DISABLE,
      .timer_sel = kPwmTimer,
      .duty = 0,
      .hpoint = 0,
      .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
      .flags = {},
  };
  return ledc_channel_config(&channelConfig);
}

esp_err_t setMotorDuty(uint32_t duty) {
  const esp_err_t result = ledc_set_duty(kPwmMode, kPwmChannel, duty);
  if (result != ESP_OK) {
    return result;
  }
  return ledc_update_duty(kPwmMode, kPwmChannel);
}

uint32_t uptimeMs() {
  return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}

bool targetReached(int32_t delta) {
  return delta >= kTargetCounts || delta <= -kTargetCounts;
}

void testMotorReachesEncoderTarget() {
  const int32_t startCount = readEncoder();
  const uint32_t startTimeMs = uptimeMs();

  ASSERT_ESP_OK(gpio_set_level(kMotorDirectionPin, 1));
  ASSERT_ESP_OK(setMotorDuty(kTestDuty));

  int32_t count = startCount;
  while (!targetReached(count - startCount) &&
         uptimeMs() - startTimeMs < kMotionTimeoutMs) {
    vTaskDelay(pdMS_TO_TICKS(1));
    count = readEncoder();
  }

  ASSERT_ESP_OK(setMotorDuty(0));

  const int32_t delta = count - startCount;
  const uint32_t elapsedMs = uptimeMs() - startTimeMs;
  printf("motor: delta=%ld elapsed_ms=%lu\n",
         static_cast<long>(delta),
         static_cast<unsigned long>(elapsedMs));
  TEST_ASSERT_TRUE_MESSAGE(targetReached(delta),
                           "Encoder target was not reached before timeout");
}

}  // namespace

void setUp() {}

void tearDown() {
  ASSERT_ESP_OK(setMotorDuty(0));
  ASSERT_ESP_OK(gpio_set_level(kMotorDirectionPin, 0));
}

extern "C" void app_main() {
  const gpio_config_t encoderConfig = {
      .pin_bit_mask = (1ULL << kEncoderAPin) | (1ULL << kEncoderBPin),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_ANYEDGE,
  };
  ESP_ERROR_CHECK(gpio_config(&encoderConfig));

  const gpio_config_t directionConfig = {
      .pin_bit_mask = 1ULL << kMotorDirectionPin,
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  ESP_ERROR_CHECK(gpio_config(&directionConfig));

  ESP_ERROR_CHECK(configureMotorPwm());
  ESP_ERROR_CHECK(setMotorDuty(0));
  ESP_ERROR_CHECK(gpio_set_level(kMotorDirectionPin, 0));

  previousState = readEncoderState();
  ESP_ERROR_CHECK(gpio_install_isr_service(0));
  ESP_ERROR_CHECK(gpio_isr_handler_add(kEncoderAPin, updateEncoder, nullptr));
  ESP_ERROR_CHECK(gpio_isr_handler_add(kEncoderBPin, updateEncoder, nullptr));

  vTaskDelay(pdMS_TO_TICKS(2000));
  UNITY_BEGIN();
  RUN_TEST(testMotorReachesEncoderTarget);
  UNITY_END();

  while (true) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
