#include <Arduino.h>
#include <esp_arduino_version.h>
#include <unity.h>

extern "C" {

void unityOutputStart(unsigned long baudrate) {
  Serial.begin(baudrate);
}

void unityOutputChar(unsigned int character) {
  Serial.write(character);
}

void unityOutputFlush() {
  Serial.flush();
}

void unityOutputComplete() {
  Serial.flush();
}

}  // extern "C"

namespace {

constexpr uint8_t kEncoderAPin = 4;
constexpr uint8_t kEncoderBPin = 3;
constexpr uint8_t kMotorDirectionPin = 10;
constexpr uint8_t kMotorPwmPin = 11;

constexpr uint8_t kPwmChannel = 0;
constexpr uint32_t kPwmFrequencyHz = 20000;
constexpr uint8_t kPwmResolutionBits = 8;
constexpr uint8_t kTestDuty = 128;
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
  return (static_cast<uint8_t>(digitalRead(kEncoderAPin)) << 1) |
         static_cast<uint8_t>(digitalRead(kEncoderBPin));
}

void ARDUINO_ISR_ATTR updateEncoder() {
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

void configureMotorPwm() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(kMotorPwmPin, kPwmFrequencyHz, kPwmResolutionBits);
#else
  ledcSetup(kPwmChannel, kPwmFrequencyHz, kPwmResolutionBits);
  ledcAttachPin(kMotorPwmPin, kPwmChannel);
#endif
}

void setMotorDuty(uint8_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(kMotorPwmPin, duty);
#else
  ledcWrite(kPwmChannel, duty);
#endif
}

bool targetReached(int32_t delta) {
  return delta >= kTargetCounts || delta <= -kTargetCounts;
}

void testMotorReachesEncoderTarget() {
  const int32_t startCount = readEncoder();
  const uint32_t startTimeMs = millis();

  digitalWrite(kMotorDirectionPin, HIGH);
  setMotorDuty(kTestDuty);

  int32_t count = startCount;
  while (!targetReached(count - startCount) &&
         millis() - startTimeMs < kMotionTimeoutMs) {
    delay(1);
    count = readEncoder();
  }

  setMotorDuty(0);

  const int32_t delta = count - startCount;
  const uint32_t elapsedMs = millis() - startTimeMs;
  Serial.printf("motor: delta=%ld elapsed_ms=%lu\n",
                static_cast<long>(delta),
                static_cast<unsigned long>(elapsedMs));
  TEST_ASSERT_TRUE_MESSAGE(targetReached(delta),
                           "Encoder target was not reached before timeout");
}

}  // namespace

void setUp() {}

void tearDown() {
  setMotorDuty(0);
  digitalWrite(kMotorDirectionPin, LOW);
}

void setup() {
  Serial.begin(115200);

  pinMode(kEncoderAPin, INPUT_PULLUP);
  pinMode(kEncoderBPin, INPUT_PULLUP);
  pinMode(kMotorDirectionPin, OUTPUT);
  pinMode(kMotorPwmPin, OUTPUT);

  configureMotorPwm();
  setMotorDuty(0);
  digitalWrite(kMotorDirectionPin, LOW);

  previousState = readEncoderState();
  attachInterrupt(digitalPinToInterrupt(kEncoderAPin), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(kEncoderBPin), updateEncoder, CHANGE);

  delay(2000);
  UNITY_BEGIN();
  RUN_TEST(testMotorReachesEncoderTarget);
  UNITY_END();
}

void loop() {
  delay(1000);
}
