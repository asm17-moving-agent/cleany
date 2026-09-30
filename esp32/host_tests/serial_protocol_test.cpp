#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>

#include "serial_protocol.hpp"

namespace {

void testKnownCrcVector() {
  constexpr uint8_t input[] = "123456789";
  assert(cleany::serial::crc16Ccitt(input, sizeof(input) - 1) == 0x29B1);
}

void testPrimitiveEncoding() {
  uint8_t bytes[16]{};
  cleany::serial::writeU16(bytes, 0x1234);
  cleany::serial::writeU32(bytes + 2, 0x89ABCDEF);
  cleany::serial::writeU64(bytes + 6, 0x0123456789ABCDEFULL);
  assert(cleany::serial::readU16(bytes) == 0x1234);
  assert(cleany::serial::readU32(bytes + 2) == 0x89ABCDEF);
  assert(cleany::serial::readU64(bytes + 6) == 0x0123456789ABCDEFULL);

  cleany::serial::writeF32(bytes, -2.5F);
  assert(cleany::serial::readF32(bytes) == -2.5F);
}

void testCobsRoundTrip() {
  constexpr std::array<uint8_t, 9> input = {
      0, 1, 2, 0, 3, 0, 0, 4, 5,
  };
  uint8_t encoded[32]{};
  uint8_t decoded[32]{};
  const size_t encodedLength = cleany::serial::cobsEncode(
      input.data(), input.size(), encoded, sizeof(encoded));
  assert(encodedLength > 0);
  for (size_t i = 0; i < encodedLength; ++i) {
    assert(encoded[i] != 0);
  }
  const size_t decodedLength = cleany::serial::cobsDecode(
      encoded, encodedLength, decoded, sizeof(decoded));
  assert(decodedLength == input.size());
  assert(std::memcmp(decoded, input.data(), input.size()) == 0);
}

void testCobsRoundTripAcrossPacketSizes() {
  std::array<uint8_t, cleany::serial::kMaximumDecodedPacketSize> input{};
  std::array<uint8_t, cleany::serial::kMaximumEncodedPacketSize> encoded{};
  std::array<uint8_t, cleany::serial::kMaximumDecodedPacketSize> decoded{};
  uint32_t state = 0x12345678;
  for (size_t length = 1; length <= input.size(); ++length) {
    state = state * 1664525U + 1013904223U;
    input[length - 1] =
        length % 7 == 0 ? 0 : static_cast<uint8_t>(state >> 24);
    const size_t encodedLength = cleany::serial::cobsEncode(
        input.data(), length, encoded.data(), encoded.size());
    assert(encodedLength > 0);
    const size_t decodedLength = cleany::serial::cobsDecode(
        encoded.data(), encodedLength, decoded.data(), decoded.size());
    assert(decodedLength == length);
    assert(std::memcmp(decoded.data(), input.data(), length) == 0);
  }
}

void testPacketRoundTrip() {
  uint8_t payload[20]{};
  cleany::serial::writeU32(payload, 0x12345678);
  cleany::serial::writeF32(payload + 4, 1.0F);
  cleany::serial::writeF32(payload + 8, -2.0F);
  cleany::serial::writeF32(payload + 12, 3.0F);
  cleany::serial::writeF32(payload + 16, -4.0F);

  uint8_t wire[cleany::serial::kMaximumWireFrameSize]{};
  const size_t wireLength = cleany::serial::encodePacket(
      cleany::serial::MessageType::kWheelCommand,
      cleany::serial::kFlagAckRequired, 42, payload, sizeof(payload),
      wire, sizeof(wire));
  assert(wireLength > 2);
  assert(wire[0] == 0);
  assert(wire[wireLength - 1] == 0);

  uint8_t decoded[cleany::serial::kMaximumDecodedPacketSize]{};
  cleany::serial::DecodedPacket packet{};
  assert(cleany::serial::decodePacket(
      wire + 1, wireLength - 2, decoded, sizeof(decoded), &packet));
  assert(packet.type == cleany::serial::MessageType::kWheelCommand);
  assert(packet.flags == cleany::serial::kFlagAckRequired);
  assert(packet.sequence == 42);
  assert(packet.payloadLength == sizeof(payload));
  assert(cleany::serial::readU32(packet.payload) == 0x12345678);
  assert(cleany::serial::readF32(packet.payload + 4) == 1.0F);
  assert(cleany::serial::readF32(packet.payload + 16) == -4.0F);
}

void testRejectsCorruptionAndMalformedCobs() {
  uint8_t wire[cleany::serial::kMaximumWireFrameSize]{};
  const size_t wireLength = cleany::serial::encodePacket(
      cleany::serial::MessageType::kStop, 0, 7, nullptr, 0,
      wire, sizeof(wire));
  assert(wireLength > 2);
  wire[wireLength / 2] ^= 0x01;

  uint8_t decoded[cleany::serial::kMaximumDecodedPacketSize]{};
  cleany::serial::DecodedPacket packet{};
  assert(!cleany::serial::decodePacket(
      wire + 1, wireLength - 2, decoded, sizeof(decoded), &packet));

  constexpr uint8_t malformed[] = {4, 1, 2};
  assert(cleany::serial::cobsDecode(
             malformed, sizeof(malformed), decoded, sizeof(decoded)) == 0);
}

void testSequenceWrapAround() {
  assert(cleany::serial::sequenceIsNewer(11, 10));
  assert(!cleany::serial::sequenceIsNewer(10, 10));
  assert(!cleany::serial::sequenceIsNewer(9, 10));
  assert(cleany::serial::sequenceIsNewer(0, UINT32_MAX));
}

}  // namespace

int main() {
  testKnownCrcVector();
  testPrimitiveEncoding();
  testCobsRoundTrip();
  testCobsRoundTripAcrossPacketSizes();
  testPacketRoundTrip();
  testRejectsCorruptionAndMalformedCobs();
  testSequenceWrapAround();
}
