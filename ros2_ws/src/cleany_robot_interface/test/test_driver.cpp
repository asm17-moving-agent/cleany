#include "cleany_feetech_hardware/system.hpp"
#include <gtest/gtest.h>
#include <pty.h>
#include <unistd.h>
#include <thread>
#include <poll.h>

using cleany_feetech_hardware::Joint;
using cleany_feetech_hardware::SerialBus;
using cleany_feetech_hardware::ReadTimeout;

bool read_request(int master, uint8_t * request, size_t size)
{
  size_t offset = 0;
  while (offset < size) {
    pollfd descriptor{master, POLLIN, 0};
    if (poll(&descriptor, 1, 300) <= 0) {return false;}
    const auto received = ::read(master, request + offset, size - offset);
    if (received <= 0) {return false;}
    offset += received;
  }
  return true;
}

Joint sample()
{
  Joint j;
  j.name = "test"; j.zero = 2048; j.direction = -1; j.offset = 0;
  j.lower = -1; j.upper = 1; j.raw_min = 1500; j.raw_max = 2600;
  j.max_velocity = .15; j.max_acceleration = .5; j.tracking_error = .08;
  j.position = j.command = j.previous = 0;
  j.raw = 2048;
  return j;
}

TEST(Mapping, DirectionAndNoDuplicateHoming)
{
  auto j = sample();
  j.homing = 500;
  EXPECT_DOUBLE_EQ(j.radians(2048), 0);
  EXPECT_LT(j.radians(2148), 0);
  EXPECT_EQ(j.ticks(j.radians(2100)), 2100);
  EXPECT_THROW(j.radians(4096), std::runtime_error);
  EXPECT_THROW(j.ticks(std::numeric_limits<double>::quiet_NaN()), std::runtime_error);
  EXPECT_THROW(j.ticks(2), std::runtime_error);
  j.raw_min = 2000;
  EXPECT_THROW(j.ticks(.2), std::runtime_error);
}

TEST(Commands, VelocityLimitingTrackingAndInvalidTime)
{
  auto j = sample();
  j.tracking_error = 0;
  j.command = .004;
  EXPECT_DOUBLE_EQ(j.limited_command(.1), .004);
  EXPECT_THROW(j.limited_command(0), std::runtime_error);
  j.command = .09;
  EXPECT_NEAR(j.limited_command(.1), .015, 1e-12);
  j.command = -.09;
  EXPECT_NEAR(j.limited_command(.1), -.015, 1e-12);
}

TEST(Commands, ZeroDisablesOnlyTrackingError)
{
  auto j = sample();
  j.tracking_error = 0;
  j.position = -.5;
  j.command = .001;
  EXPECT_NO_THROW(j.limited_command(.1));
  j.command = .1;
  EXPECT_NO_THROW(j.limited_command(.1));
  j.tracking_error = .08;
  EXPECT_THROW(j.limited_command(.1), std::runtime_error);
}

TEST(Commands, MeasuredHoldStopsALaggingTargetWithoutAllowingTargetJumps)
{
  auto j = sample();
  j.previous = .02;
  j.command = j.position;
  EXPECT_TRUE(j.measured_hold());
  EXPECT_DOUBLE_EQ(j.limited_command(.02), j.position);
  EXPECT_THROW(j.limited_command(0), std::runtime_error);
  // Encoder-rounded feedback is accepted, but a larger movement request
  // retains the speed/acceleration checks.
  j.command = j.radians(j.raw - 2);
  EXPECT_TRUE(j.measured_hold());
  EXPECT_DOUBLE_EQ(j.limited_command(.02), j.position);
  j.command = j.radians(j.raw - 3);
  EXPECT_FALSE(j.measured_hold());
  EXPECT_NE(j.limited_command(.02), j.command);
  j.position = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(j.limited_command(.02), std::runtime_error);
}

TEST(Serial, ReadOnlyCannotWriteOrOpenOccupiedPort)
{
  int master, slave;
  char path[128];
  ASSERT_EQ(openpty(&master, &slave, path, nullptr, nullptr), 0);
  close(slave);
  SerialBus bus;
  bus.open(path, 20, false);
  SerialBus other;
  EXPECT_THROW(other.open(path, 20, true), std::runtime_error);
  EXPECT_THROW(bus.write_value(1, 42, 2, 2048), std::runtime_error);
  EXPECT_THROW(bus.goals({{1, 2048}}), std::runtime_error);
  bus.close();
  EXPECT_NO_THROW(other.open(path, 20, false));
  other.close(); close(master);
}

TEST(Serial, ValidReadAndCorruptChecksum)
{
  for (bool corrupt : {false, true}) {
    int master, slave;
    char path[128];
    ASSERT_EQ(openpty(&master, &slave, path, nullptr, nullptr), 0);
    close(slave);
    SerialBus bus;
    bus.open(path, 100, false);
    std::thread servo([=]() {
      uint8_t request[8]; size_t count = 0;
      while (count < sizeof(request)) {
        auto n = ::read(master, request + count, sizeof(request) - count);
        if (n <= 0) {return;}
        count += n;
      }
      std::vector<uint8_t> response{255, 255, 1, 4, 0, 0, 8, static_cast<uint8_t>(~13)};
      if (corrupt) {response.back() = 0;}
      ::write(master, response.data(), response.size());
    });
    if (corrupt) {EXPECT_THROW(bus.read_value(1, 56, 2), std::runtime_error);}
    else {EXPECT_EQ(bus.read_value(1, 56, 2), 2048);}
    servo.join(); bus.close(); close(master);
  }
}

TEST(Serial, TimeoutAndWriteWhitelist)
{
  int master, slave;
  char path[128];
  ASSERT_EQ(openpty(&master, &slave, path, nullptr, nullptr), 0);
  close(slave);
  SerialBus bus;
  bus.open(path, 5, true);
  EXPECT_THROW(bus.read_value(1, 56, 2), std::runtime_error);
  EXPECT_THROW(bus.write_value(1, 13, 1, 80), std::runtime_error);  // EEPROM
  EXPECT_THROW(bus.write_value(1, 40, 1, 0), std::runtime_error);   // automatic load release
  EXPECT_THROW(bus.write_value(13, 42, 2, 2048), std::runtime_error);
  EXPECT_THROW(bus.goals({{1, 2048}, {1, 2049}}), std::runtime_error);
  EXPECT_THROW(bus.goals({{1, 4096}}), std::runtime_error);
  bus.close(); close(master);
}

TEST(Serial, OneDroppedReadRecoversWithinBudget)
{
  int master, slave;
  char path[128];
  ASSERT_EQ(openpty(&master, &slave, path, nullptr, nullptr), 0);
  close(slave);
  SerialBus bus;
  bus.open(path, 20, false);
  int holds = 0;
  bus.configure_read_recovery(1, 60, [&](int id, int address) {
      EXPECT_EQ(id, 1); EXPECT_EQ(address, 56); ++holds;
    });
  std::thread servo([=]() {
      uint8_t request[8];
      if (!read_request(master, request, sizeof(request))) {return;}
      if (!read_request(master, request, sizeof(request))) {return;}
      std::vector<uint8_t> response{255, 255, 1, 4, 0, 0, 8, static_cast<uint8_t>(~13)};
      ::write(master, response.data(), response.size());
    });
  EXPECT_EQ(bus.read_value(1, 56, 2), 2048);
  EXPECT_EQ(holds, 1);
  EXPECT_EQ(bus.read_recoveries(), 1u);
  servo.join(); bus.close(); close(master);
}

TEST(Serial, PersistentReadTimeoutAndWriteTimeoutNeverLoop)
{
  int master, slave;
  char path[128];
  ASSERT_EQ(openpty(&master, &slave, path, nullptr, nullptr), 0);
  close(slave);
  SerialBus bus;
  bus.open(path, 5, true);
  int holds = 0;
  bus.configure_read_recovery(1, 15, [&](int, int) {++holds;});
  const auto start = std::chrono::steady_clock::now();
  EXPECT_THROW(bus.read_value(1, 56, 2), ReadTimeout);
  EXPECT_LT(std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count(), .1);
  EXPECT_EQ(holds, 1);
  EXPECT_EQ(bus.read_recoveries(), 0u);
  EXPECT_THROW(bus.write_value(1, 42, 2, 2048), std::runtime_error);
  EXPECT_EQ(holds, 1);  // WRITE acknowledgement failures are never retried.
  bus.close(); close(master);
}

TEST(Serial, MotorErrorOrCorruptPacketIsNotRecoverable)
{
  for (bool corrupt : {false, true}) {
    int master, slave;
    char path[128];
    ASSERT_EQ(openpty(&master, &slave, path, nullptr, nullptr), 0);
    close(slave);
    SerialBus bus;
    bus.open(path, 50, false);
    int retries = 0;
    bus.configure_read_recovery(1, 100, [&](int, int) {++retries;});
    std::thread servo([=]() {
        uint8_t request[8];
        if (!read_request(master, request, sizeof(request))) {return;}
        std::vector<uint8_t> response{255, 255, 1, 4, 1, 0, 8, static_cast<uint8_t>(~14)};
        if (corrupt) {response.back() = 0;}
        ::write(master, response.data(), response.size());
      });
    EXPECT_THROW(bus.read_value(1, 56, 2), std::runtime_error);
    EXPECT_EQ(retries, 0);
    servo.join(); bus.close(); close(master);
  }
}

TEST(Serial, CompleteFrameDeadlineBoundsReadRecovery)
{
  int master, slave;
  char path[128];
  ASSERT_EQ(openpty(&master, &slave, path, nullptr, nullptr), 0);
  close(slave);
  SerialBus bus;
  bus.open(path, 20, false);
  bus.configure_read_recovery(1, 60);
  const auto started = std::chrono::steady_clock::now();
  bus.set_deadline(started + std::chrono::milliseconds(10));
  EXPECT_THROW(bus.read_value(1, 56, 2), std::runtime_error);
  EXPECT_LT(std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count(), .05);
  EXPECT_EQ(bus.read_recoveries(), 0u);
  bus.set_deadline({});
  bus.close(); close(master);
}
