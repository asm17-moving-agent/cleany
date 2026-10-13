#include <cassert>
#include <deque>
#include <vector>
#include "usb_stream_transport.hpp"

class MockPort final : public cleany::UsbStreamPort {
 public:
  bool open() override { ++opens; return openWorks; }
  void close() override { ++closes; }
  int read(uint8_t* out, size_t size, uint32_t) override {
    ++reads;
    if (readFailure) return -1;
    if (input.empty()) { ++clock; return 0; }
    int n = std::min<int>(size, input.size());
    if (readLimit > 0) n = std::min(n, readLimit);
    const int result = n;
    while (n--) { *out++ = input.front(); input.pop_front(); }
    return result;
  }
  int write(const uint8_t* data, size_t size, uint32_t) override {
    ++writes;
    if (writeFailure) return -1;
    int n = writeLimit > 0 ? std::min<int>(size, writeLimit) : size;
    output.insert(output.end(), data, data + n);
    return n;
  }
  uint64_t nowMs() const override { return clock; }
  mutable uint64_t clock = 0;
  bool openWorks = true, readFailure = false, writeFailure = false;
  int readLimit = 0, writeLimit = 0, opens = 0, closes = 0, reads = 0, writes = 0;
  std::deque<uint8_t> input;
  std::vector<uint8_t> output;
};

int main() {
  MockPort port;
  cleany::UsbStreamTransport transport(port);
  uint8_t bytes[8]{};
  assert(transport.read(bytes, 1, 0) == -1);
  port.openWorks = false;
  assert(!transport.open());
  port.openWorks = true;
  assert(transport.open());
  assert(port.opens == 2);
  port.input = {1, 2, 3};
  port.readLimit = 1;
  assert(transport.read(bytes, 4, 10) == 3);
  assert(bytes[0] == 1 && bytes[2] == 3);
  // Empty reads advance the fake clock; a multi-byte deadline read preserves
  // bytes already received rather than waiting forever.
  port.input = {9};
  port.readLimit = 1;
  assert(transport.read(bytes, 4, 2) == 1 && bytes[0] == 9);
  port.readLimit = 0;
  assert(transport.read(bytes, 4, 0) == 0);
  port.readFailure = true;
  assert(transport.read(bytes, 1, 10) == -1);
  port.readFailure = false;
  port.writeLimit = 2;
  const uint8_t output[] = {4, 5, 6, 7, 8};
  assert(transport.write(output, sizeof(output), 10) == 5);
  assert(port.output.size() == 5);
  port.writeFailure = true;
  assert(transport.write(output, sizeof(output), 10) == -1);
  port.writeFailure = false;
  transport.close();
  assert(port.closes == 1 && !transport.isOpen());
  assert(transport.write(output, 1, 1) == -1);
  assert(transport.read(bytes, 1, 1) == -1);
  transport.close();
  assert(port.closes == 1);
}
