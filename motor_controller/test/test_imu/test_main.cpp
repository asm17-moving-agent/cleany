#include <Arduino.h>
#include <Wire.h>
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

constexpr uint8_t kSdaPin = 8;
constexpr uint8_t kSclPin = 9;
constexpr uint32_t kI2cFrequencyHz = 100000;

constexpr uint8_t kAddressAd0Low = 0x68;
constexpr uint8_t kAddressAd0High = 0x69;
constexpr uint8_t kRegisterAccelXoutHigh = 0x3B;
constexpr uint8_t kRegisterPowerManagement1 = 0x6B;
constexpr uint8_t kRegisterWhoAmI = 0x75;
constexpr uint8_t kExpectedWhoAmI = 0x68;

bool deviceResponds(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

uint8_t detectMpu6050Address() {
  if (deviceResponds(kAddressAd0Low)) {
    return kAddressAd0Low;
  }
  if (deviceResponds(kAddressAd0High)) {
    return kAddressAd0High;
  }
  return 0;
}

bool writeRegister(uint8_t address, uint8_t registerAddress, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(registerAddress);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readRegisters(uint8_t address,
                   uint8_t registerAddress,
                   uint8_t* data,
                   size_t length) {
  Wire.beginTransmission(address);
  Wire.write(registerAddress);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  const size_t received = Wire.requestFrom(
      address, static_cast<uint8_t>(length), static_cast<uint8_t>(true));
  if (received != length) {
    return false;
  }

  for (size_t index = 0; index < length; ++index) {
    data[index] = Wire.read();
  }
  return true;
}

int16_t decodeSigned16(const uint8_t* bytes) {
  return static_cast<int16_t>((static_cast<uint16_t>(bytes[0]) << 8) |
                              bytes[1]);
}

void testMpu6050Identity() {
  const uint8_t address = detectMpu6050Address();
  TEST_ASSERT_NOT_EQUAL_MESSAGE(0, address,
                                "No I2C device responded at 0x68 or 0x69");

  uint8_t whoAmI = 0;
  TEST_ASSERT_TRUE_MESSAGE(
      readRegisters(address, kRegisterWhoAmI, &whoAmI, 1),
      "Failed to read WHO_AM_I");
  Serial.printf("imu: address=0x%02X who_am_i=0x%02X\n", address, whoAmI);
  TEST_ASSERT_EQUAL_HEX8(kExpectedWhoAmI, whoAmI);
}

void testMpu6050SensorFrame() {
  const uint8_t address = detectMpu6050Address();
  TEST_ASSERT_NOT_EQUAL_MESSAGE(0, address,
                                "No I2C device responded at 0x68 or 0x69");
  TEST_ASSERT_TRUE_MESSAGE(
      writeRegister(address, kRegisterPowerManagement1, 0x00),
      "Failed to wake MPU6050");
  delay(100);

  uint8_t frame[14] = {};
  TEST_ASSERT_TRUE_MESSAGE(
      readRegisters(address, kRegisterAccelXoutHigh, frame, sizeof(frame)),
      "Failed to read accelerometer/temperature/gyroscope frame");

  bool allZero = true;
  bool allOnes = true;
  for (uint8_t value : frame) {
    allZero = allZero && value == 0x00;
    allOnes = allOnes && value == 0xFF;
  }

  const int16_t accelX = decodeSigned16(&frame[0]);
  const int16_t accelY = decodeSigned16(&frame[2]);
  const int16_t accelZ = decodeSigned16(&frame[4]);
  const int16_t gyroX = decodeSigned16(&frame[8]);
  const int16_t gyroY = decodeSigned16(&frame[10]);
  const int16_t gyroZ = decodeSigned16(&frame[12]);
  Serial.printf("imu: accel=[%d,%d,%d] gyro=[%d,%d,%d]\n",
                accelX, accelY, accelZ, gyroX, gyroY, gyroZ);

  TEST_ASSERT_FALSE_MESSAGE(allZero, "Sensor frame contains only zeroes");
  TEST_ASSERT_FALSE_MESSAGE(allOnes, "Sensor frame contains only 0xFF");
}

}  // namespace

void setUp() {}
void tearDown() {}

void setup() {
  Serial.begin(115200);
  Wire.begin(kSdaPin, kSclPin, kI2cFrequencyHz);

  delay(2000);
  UNITY_BEGIN();
  RUN_TEST(testMpu6050Identity);
  RUN_TEST(testMpu6050SensorFrame);
  UNITY_END();
}

void loop() {
  delay(1000);
}
