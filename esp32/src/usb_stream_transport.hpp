#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>

namespace cleany {

/// 포트 연산은 즉시 반환하거나 지정된 제한 시간 안에 끝납니다.
/// 양수는 처리한 바이트 수, 0은 시간 초과, 음수는 I/O 오류입니다.
class UsbStreamPort {
 public:
  virtual ~UsbStreamPort() = default;
  virtual bool open() = 0;
  virtual void close() = 0;
  virtual int read(uint8_t* data, size_t size, uint32_t timeoutMs) = 0;
  virtual int write(const uint8_t* data, size_t size, uint32_t timeoutMs) = 0;
  virtual uint64_t nowMs() const = 0;
};

/**
 * @brief XRCE 스트림 프레임의 부분 USB 전송 시간을 제한합니다.
 * 참조하는 포트는 이 객체보다 오래 유지되어야 하며 단조 증가 nowMs()를 제공해야 합니다.
 * 쓰기 실패 전에 일부 바이트가 전송될 수 있으므로 호출자가 프레임 복구를 처리해야 합니다.
 */
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

  /// 시간 초과 시 이미 받은 데이터를 반환하며, 수신이 없으면 0, 오류이면 -1입니다.
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

  /// XRCE의 신뢰성 프레임에는 전체 payload가 필요하므로 불완전한 쓰기는 실패로 반환합니다.
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
