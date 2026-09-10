#include <cstddef>
#include <cstdint>
#include <cstdio>

#include "driver/i2c.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <unity.h>

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

constexpr gpio_num_t kSdaPin = GPIO_NUM_8;
constexpr gpio_num_t kSclPin = GPIO_NUM_9;
constexpr uint32_t kI2cFrequencyHz = 100000;
constexpr i2c_port_t kI2cPort = I2C_NUM_0;
constexpr TickType_t kI2cTimeout = pdMS_TO_TICKS(100);

constexpr uint8_t kAddressAd0Low = 0x68;
constexpr uint8_t kAddressAd0High = 0x69;
constexpr uint8_t kRegisterAccelXoutHigh = 0x3B;
constexpr uint8_t kRegisterPowerManagement1 = 0x6B;
constexpr uint8_t kRegisterWhoAmI = 0x75;
constexpr uint8_t kExpectedWhoAmI = 0x68;

bool deviceResponds(uint8_t address) {
  i2c_cmd_handle_t command = i2c_cmd_link_create();
  i2c_master_start(command);
  i2c_master_write_byte(command, (address << 1) | I2C_MASTER_WRITE, true);
  i2c_master_stop(command);
  const esp_err_t result =
      i2c_master_cmd_begin(kI2cPort, command, kI2cTimeout);
  i2c_cmd_link_delete(command);
  return result == ESP_OK;
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
  const uint8_t data[] = {registerAddress, value};
  return i2c_master_write_to_device(
             kI2cPort, address, data, sizeof(data), kI2cTimeout) == ESP_OK;
}

bool readRegisters(uint8_t address,
                   uint8_t registerAddress,
                   uint8_t* data,
                   size_t length) {
  return i2c_master_write_read_device(kI2cPort,
                                      address,
                                      &registerAddress,
                                      1,
                                      data,
                                      length,
                                      kI2cTimeout) == ESP_OK;
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
  printf("imu: address=0x%02X who_am_i=0x%02X\n", address, whoAmI);
  TEST_ASSERT_EQUAL_HEX8(kExpectedWhoAmI, whoAmI);
}

void testMpu6050SensorFrame() {
  const uint8_t address = detectMpu6050Address();
  TEST_ASSERT_NOT_EQUAL_MESSAGE(0, address,
                                "No I2C device responded at 0x68 or 0x69");
  TEST_ASSERT_TRUE_MESSAGE(
      writeRegister(address, kRegisterPowerManagement1, 0x00),
      "Failed to wake MPU6050");
  vTaskDelay(pdMS_TO_TICKS(100));

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
  printf("imu: accel=[%d,%d,%d] gyro=[%d,%d,%d]\n",
         accelX, accelY, accelZ, gyroX, gyroY, gyroZ);

  TEST_ASSERT_FALSE_MESSAGE(allZero, "Sensor frame contains only zeroes");
  TEST_ASSERT_FALSE_MESSAGE(allOnes, "Sensor frame contains only 0xFF");
}

}  // namespace

void setUp() {}
void tearDown() {}

extern "C" void app_main() {
  const i2c_config_t config = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = kSdaPin,
      .scl_io_num = kSclPin,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master = {.clk_speed = kI2cFrequencyHz},
      .clk_flags = 0,
  };
  ESP_ERROR_CHECK(i2c_param_config(kI2cPort, &config));
  ESP_ERROR_CHECK(i2c_driver_install(kI2cPort, config.mode, 0, 0, 0));

  vTaskDelay(pdMS_TO_TICKS(2000));
  UNITY_BEGIN();
  RUN_TEST(testMpu6050Identity);
  RUN_TEST(testMpu6050SensorFrame);
  UNITY_END();

  while (true) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
