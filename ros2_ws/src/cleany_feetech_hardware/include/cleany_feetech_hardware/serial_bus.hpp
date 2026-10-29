#pragma once
#include <chrono>
#include <cstdint>
#include <functional>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace cleany_feetech_hardware
{
class ReadTimeout : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};
// Feetech protocol 1, little-endian STS registers. No EEPROM write API.
class SerialBus
{
public:
  SerialBus() = default;
  ~SerialBus();
  SerialBus(const SerialBus &) = delete;
  SerialBus & operator=(const SerialBus &) = delete;
  void open(const std::string & path, int timeout_ms, bool writable);
  void close();
  bool is_open() const {return fd_ >= 0;}
  void configure_read_recovery(int retries, int budget_ms,
    std::function<void(int, int)> before_retry = {});
  uint64_t read_recoveries() const {return read_recoveries_;}
  void set_deadline(std::optional<std::chrono::steady_clock::time_point> deadline)
  {deadline_ = deadline;}
  std::vector<uint8_t> read(int id, int address, int size);
  int read_value(int id, int address, int size);
  void write_value(int id, int address, int size, int value);
  void goals(const std::vector<std::pair<int, int>> & values);
private:
  int fd_{-1}, timeout_ms_{20};
  bool exclusive_{false}, writable_{false};
  int read_retries_{0}, read_budget_ms_{60};
  uint64_t read_recoveries_{0};
  std::function<void(int, int)> before_read_retry_;
  std::optional<std::chrono::steady_clock::time_point> deadline_;
  std::vector<uint8_t> transaction(int id, int instruction,
    const std::vector<uint8_t> & params, int response_size, int timeout_ms = 0);
};
}  // namespace cleany_feetech_hardware
