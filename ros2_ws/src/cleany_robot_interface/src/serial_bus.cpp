#include "cleany_feetech_hardware/serial_bus.hpp"
#include <algorithm>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <stdexcept>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

namespace cleany_feetech_hardware
{
SerialBus::~SerialBus() {close();}

void SerialBus::configure_read_recovery(
  int retries, int budget_ms, std::function<void(int, int)> before_retry)
{
  if (retries < 0 || retries > 1 || budget_ms < 1 || budget_ms > 100) {
    throw std::runtime_error("Invalid bounded READ recovery settings");
  }
  read_retries_ = retries;
  read_budget_ms_ = budget_ms;
  before_read_retry_ = std::move(before_retry);
}

void SerialBus::open(const std::string & path, int timeout_ms, bool writable)
{
  if (fd_ >= 0 || timeout_ms < 1 || timeout_ms > 100) {
    throw std::runtime_error("Invalid serial open/timeout");
  }
  fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) {throw std::runtime_error("Serial open: " + std::string(strerror(errno)));}
  try {
    if (flock(fd_, LOCK_EX | LOCK_NB) != 0 || ioctl(fd_, TIOCEXCL) != 0) {
      throw std::runtime_error("Serial device is already owned");
    }
    exclusive_ = true;
    termios settings{};
    if (tcgetattr(fd_, &settings) != 0) {throw std::runtime_error("tcgetattr failed");}
    cfmakeraw(&settings);
    settings.c_cflag |= CLOCAL | CREAD;
    settings.c_cflag &= ~CRTSCTS;
    cfsetispeed(&settings, B1000000);
    cfsetospeed(&settings, B1000000);
    if (tcsetattr(fd_, TCSANOW, &settings) != 0) {throw std::runtime_error("tcsetattr failed");}
    timeout_ms_ = timeout_ms;
    writable_ = writable;
  } catch (...) {close(); throw;}
}

void SerialBus::close()
{
  if (fd_ >= 0) {
    if (exclusive_) {ioctl(fd_, TIOCNXCL);}
    ::close(fd_);
  }
  fd_ = -1;
  exclusive_ = writable_ = false;
}

std::vector<uint8_t> SerialBus::transaction(
  int id, int instruction, const std::vector<uint8_t> & params, int response_size, int timeout_ms)
{
  if (fd_ < 0) {throw std::runtime_error("Serial closed");}
  std::vector<uint8_t> packet{255, 255, static_cast<uint8_t>(id),
    static_cast<uint8_t>(params.size() + 2), static_cast<uint8_t>(instruction)};
  packet.insert(packet.end(), params.begin(), params.end());
  unsigned checksum = 0;
  for (size_t i = 2; i < packet.size(); ++i) {checksum += packet[i];}
  packet.push_back(static_cast<uint8_t>(~checksum));
  auto deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(timeout_ms > 0 ? timeout_ms : timeout_ms_);
  if (deadline_) {deadline = std::min(deadline, *deadline_);}
  bool reading = false;
  auto timeout = [&]() {
      const std::string detail = "Serial timeout ID " + std::to_string(id) +
        " instruction=" + std::to_string(instruction) +
        " address=" + (params.empty() ? "none" : std::to_string(params[0]));
      if (instruction == 2 && reading) {throw ReadTimeout(detail);}
      throw std::runtime_error(detail);
    };
  auto ready = [&](short events) {
    while (true) {
      const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - std::chrono::steady_clock::now()).count();
      if (remaining <= 0) {timeout();}
      pollfd p{fd_, events, 0};
      const int result = poll(&p, 1, static_cast<int>(remaining));
      if (result < 0 && errno == EINTR) {continue;}
      if (result < 0 || (p.revents & (POLLERR | POLLHUP | POLLNVAL))) {
        throw std::runtime_error("Serial disconnect/poll failure ID " + std::to_string(id));
      }
      if (result == 0) {timeout();}
      if (p.revents & events) {return;}
    }
  };
  if (tcflush(fd_, TCIFLUSH) != 0) {throw std::runtime_error("Serial flush failed");}
  size_t offset = 0;
  while (offset < packet.size()) {
    ready(POLLOUT);
    const auto n = ::write(fd_, packet.data() + offset, packet.size() - offset);
    if (n < 0 && (errno == EAGAIN || errno == EINTR)) {continue;}
    if (n <= 0) {throw std::runtime_error("Serial write failed");}
    offset += n;
  }
  if (response_size < 0) {return {};}
  reading = true;
  std::vector<uint8_t> response(response_size + 6);
  offset = 0;
  while (offset < response.size()) {
    ready(POLLIN);
    const auto n = ::read(fd_, response.data() + offset, response.size() - offset);
    if (n < 0 && (errno == EAGAIN || errno == EINTR)) {continue;}
    if (n <= 0) {throw std::runtime_error("Serial read failed");}
    offset += n;
  }
  checksum = 0;
  for (size_t i = 2; i < response.size(); ++i) {checksum += response[i];}
  if (response[0] != 255 || response[1] != 255 || response[2] != id ||
    response[3] != response_size + 2 || (checksum & 255) != 255 || response[4] != 0)
  {
    throw std::runtime_error("Invalid checksum/ID/length or motor fault ID " + std::to_string(id));
  }
  return {response.begin() + 5, response.end() - 1};
}

std::vector<uint8_t> SerialBus::read(int id, int address, int size)
{
  if (id < 1 || id > 14 || address < 0 || size < 1 || size > 32 || address + size > 256) {
    throw std::runtime_error("Invalid read");
  }
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(read_budget_ms_);
  for (int attempt = 0; ; ++attempt) {
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
      deadline - std::chrono::steady_clock::now()).count();
    if (attempt > 0 && remaining <= 0) {
      throw ReadTimeout("READ recovery budget exhausted ID " + std::to_string(id));
    }
    try {
      auto result = transaction(id, 2,
        {static_cast<uint8_t>(address), static_cast<uint8_t>(size)}, size,
        read_retries_ > 0 ? std::min(timeout_ms_, static_cast<int>(remaining)) : timeout_ms_);
      if (attempt > 0) {++read_recoveries_;}
      return result;
    } catch (const ReadTimeout &) {
      if (attempt >= read_retries_) {throw;}
      if (before_read_retry_) {before_read_retry_(id, address);}
    }
  }
}

int SerialBus::read_value(int id, int address, int size)
{
  if (size != 1 && size != 2) {throw std::runtime_error("Invalid register width");}
  const auto bytes = read(id, address, size);
  return bytes[0] + (size == 2 ? bytes[1] * 256 : 0);
}

void SerialBus::write_value(int id, int address, int size, int value)
{
  const bool valid = (address == 40 && size == 1 && value == 1) ||
    (address == 41 && size == 1 && value >= 1 && value <= 254) ||
    (address == 42 && size == 2 && value >= 0 && value <= 4095) ||
    (address == 44 && size == 2 && value == 0) ||
    (address == 46 && size == 2 && value >= 1 && value <= 4095) ||
    (address == 48 && size == 2 && value >= 1 && value <= 1000);
  if (!writable_ || id < 1 || id > 12 || !valid) {
    throw std::runtime_error("Register write forbidden");
  }
  std::vector<uint8_t> params{static_cast<uint8_t>(address), static_cast<uint8_t>(value & 255)};
  if (size == 2) {params.push_back(static_cast<uint8_t>(value >> 8));}
  transaction(id, 3, params, 0);
}

void SerialBus::goals(const std::vector<std::pair<int, int>> & values)
{
  if (!writable_ || values.empty() || values.size() > 12) {
    throw std::runtime_error("Goal write forbidden");
  }
  std::vector<uint8_t> params{42, 2};
  unsigned seen = 0;
  for (const auto & [id, value] : values) {
    if (id < 1 || id > 12 || value < 0 || value > 4095 || (seen & (1u << id))) {
      throw std::runtime_error("Invalid goal write");
    }
    seen |= 1u << id;
    params.insert(params.end(), {static_cast<uint8_t>(id), static_cast<uint8_t>(value & 255),
      static_cast<uint8_t>(value >> 8)});
  }
  transaction(254, 131, params, -1);  // Broadcast sync-write has no acknowledgement.
}
}  // namespace cleany_feetech_hardware
