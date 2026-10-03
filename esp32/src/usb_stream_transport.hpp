#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>

namespace cleany {

// Port operations are non-blocking or bounded by the supplied timeout. Return
// positive byte count, zero on timeout, and negative on I/O error.
class UsbStreamPort {
 public:
  virtual ~UsbStreamPort() = default;
  virtual bool open() = 0;
  virtual void close() = 0;
  virtual int read(uint8_t* data, size_t size, uint32_t timeoutMs) = 0;
  virtual int write(const uint8_t* data, size_t size, uint32_t timeoutMs) = 0;
  virtual uint64_t nowMs() const = 0;
};

class UsbStreamTransport {
 public:
  explicit UsbStreamTransport(UsbStreamPort& port) : port_(port) {}

  bool open() {
    if (opened_) return true;
    opened_ = port_.open();
    return opened_;
  }
  void close() {
    if (opened_) port_.close();
    opened_ = false;
  }
  bool isOpen() const { return opened_; }

  // Returns partial data on timeout, 0 if no bytes arrived, and -1 on error.
  int read(uint8_t* data, size_t size, int timeoutMs) {
    if (!opened_ || data == nullptr || timeoutMs < 0) return -1;
    if (size == 0) return 0;
    const uint64_t deadline = port_.nowMs() + static_cast<uint32_t>(timeoutMs);
    size_t received = 0;
    while (received < size) {
      const uint64_t now = port_.nowMs();
      if (now >= deadline && timeoutMs != 0) break;
      const uint32_t remaining = timeoutMs == 0 ? 0 :
          static_cast<uint32_t>(deadline - now);
      const int n = port_.read(data + received, size - received, remaining);
      if (n < 0 || static_cast<size_t>(n) > size - received) return -1;
      if (n == 0) break;
      received += static_cast<size_t>(n);
      if (timeoutMs == 0) break;
    }
    return static_cast<int>(received);
  }

  // XRCE's reliable framing needs the complete payload; failure is explicit.
  int write(const uint8_t* data, size_t size, int timeoutMs) {
    if (!opened_ || (data == nullptr && size != 0) || timeoutMs < 0) return -1;
    const uint64_t deadline = port_.nowMs() + static_cast<uint32_t>(timeoutMs);
    size_t sent = 0;
    while (sent < size) {
      const uint64_t now = port_.nowMs();
      if (timeoutMs != 0 && now >= deadline) return -1;
      const uint32_t remaining = timeoutMs == 0 ? 0 :
          static_cast<uint32_t>(deadline - now);
      const int n = port_.write(data + sent, size - sent, remaining);
      if (n <= 0 || static_cast<size_t>(n) > size - sent) return -1;
      sent += static_cast<size_t>(n);
    }
    return static_cast<int>(sent);
  }

 private:
  UsbStreamPort& port_;
  bool opened_ = false;
};
}  // namespace cleany
