#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace cleany::serial {

constexpr uint8_t kProtocolMajor = 1;
constexpr uint8_t kProtocolMinor = 0;
constexpr uint16_t kMagic = 0x4C43;  // "CL" as little-endian bytes.
constexpr size_t kHeaderSize = 12;
constexpr size_t kCrcSize = 2;
constexpr size_t kMaximumDecodedPacketSize = 240;
constexpr size_t kMaximumEncodedPacketSize =
    kMaximumDecodedPacketSize +
    (kMaximumDecodedPacketSize / 254) + 1;
constexpr size_t kMaximumWireFrameSize = kMaximumEncodedPacketSize + 2;

enum class MessageType : uint8_t {
  kHelloRequest = 0x01,
  kStreamConfig = 0x02,
  kTimeSyncRequest = 0x03,
  kWheelCommand = 0x10,
  kStop = 0x11,
  kHello = 0x81,
  kAck = 0x82,
  kTimeSyncResponse = 0x83,
  kWheelState = 0x90,
  kImuState = 0x91,
};

enum class Status : uint8_t {
  kOk = 0,
  kBadPayload = 1,
  kOutOfRange = 2,
  kUnsupported = 3,
  kBadSession = 4,
  kStaleSequence = 5,
  kBusy = 6,
  kInternalError = 7,
};

constexpr uint8_t kFlagAckRequired = 1U << 0;
constexpr uint32_t kCapabilityWheelVelocity = 1U << 0;
constexpr uint32_t kCapabilityImu = 1U << 1;
constexpr uint32_t kCapabilityTimeSync = 1U << 2;

struct DecodedPacket {
  MessageType type;
  uint8_t flags;
  uint32_t sequence;
  const uint8_t* payload;
  uint16_t payloadLength;
};

inline uint16_t readU16(const uint8_t* data) {
  return static_cast<uint16_t>(
      static_cast<uint16_t>(data[0]) |
      static_cast<uint16_t>(data[1]) << 8);
}

inline uint32_t readU32(const uint8_t* data) {
  return static_cast<uint32_t>(data[0]) |
         static_cast<uint32_t>(data[1]) << 8 |
         static_cast<uint32_t>(data[2]) << 16 |
         static_cast<uint32_t>(data[3]) << 24;
}

inline uint64_t readU64(const uint8_t* data) {
  return static_cast<uint64_t>(readU32(data)) |
         static_cast<uint64_t>(readU32(data + 4)) << 32;
}

inline float readF32(const uint8_t* data) {
  const uint32_t bits = readU32(data);
  float value = 0.0F;
  static_assert(sizeof(value) == sizeof(bits));
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

inline void writeU16(uint8_t* data, uint16_t value) {
  data[0] = static_cast<uint8_t>(value);
  data[1] = static_cast<uint8_t>(value >> 8);
}

inline void writeU32(uint8_t* data, uint32_t value) {
  data[0] = static_cast<uint8_t>(value);
  data[1] = static_cast<uint8_t>(value >> 8);
  data[2] = static_cast<uint8_t>(value >> 16);
  data[3] = static_cast<uint8_t>(value >> 24);
}

inline void writeU64(uint8_t* data, uint64_t value) {
  writeU32(data, static_cast<uint32_t>(value));
  writeU32(data + 4, static_cast<uint32_t>(value >> 32));
}

inline void writeF32(uint8_t* data, float value) {
  uint32_t bits = 0;
  static_assert(sizeof(value) == sizeof(bits));
  std::memcpy(&bits, &value, sizeof(bits));
  writeU32(data, bits);
}

inline uint16_t crc16Ccitt(const uint8_t* data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000U) != 0
                ? static_cast<uint16_t>((crc << 1) ^ 0x1021U)
                : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

inline size_t cobsEncode(const uint8_t* input, size_t inputLength,
                         uint8_t* output, size_t outputCapacity) {
  if (outputCapacity == 0) {
    return 0;
  }
  size_t readIndex = 0;
  size_t writeIndex = 1;
  size_t codeIndex = 0;
  uint8_t code = 1;
  while (readIndex < inputLength) {
    if (input[readIndex] == 0) {
      if (codeIndex >= outputCapacity) {
        return 0;
      }
      output[codeIndex] = code;
      code = 1;
      codeIndex = writeIndex++;
      if (writeIndex > outputCapacity) {
        return 0;
      }
      ++readIndex;
    } else {
      if (writeIndex >= outputCapacity) {
        return 0;
      }
      output[writeIndex++] = input[readIndex++];
      ++code;
      if (code == 0xFF) {
        if (codeIndex >= outputCapacity) {
          return 0;
        }
        output[codeIndex] = code;
        code = 1;
        codeIndex = writeIndex++;
        if (writeIndex > outputCapacity) {
          return 0;
        }
      }
    }
  }
  if (codeIndex >= outputCapacity) {
    return 0;
  }
  output[codeIndex] = code;
  return writeIndex;
}

inline size_t cobsDecode(const uint8_t* input, size_t inputLength,
                         uint8_t* output, size_t outputCapacity) {
  size_t readIndex = 0;
  size_t writeIndex = 0;
  while (readIndex < inputLength) {
    const uint8_t code = input[readIndex++];
    if (code == 0 || readIndex + static_cast<size_t>(code - 1) > inputLength) {
      return 0;
    }
    for (uint8_t i = 1; i < code; ++i) {
      if (writeIndex >= outputCapacity) {
        return 0;
      }
      output[writeIndex++] = input[readIndex++];
    }
    if (code != 0xFF && readIndex < inputLength) {
      if (writeIndex >= outputCapacity) {
        return 0;
      }
      output[writeIndex++] = 0;
    }
  }
  return writeIndex;
}

inline size_t encodePacket(MessageType type, uint8_t flags, uint32_t sequence,
                           const uint8_t* payload, uint16_t payloadLength,
                           uint8_t* wireFrame, size_t wireCapacity) {
  const size_t decodedLength =
      kHeaderSize + static_cast<size_t>(payloadLength) + kCrcSize;
  if (decodedLength > kMaximumDecodedPacketSize ||
      wireCapacity < kMaximumWireFrameSize ||
      (payloadLength > 0 && payload == nullptr)) {
    return 0;
  }

  uint8_t decoded[kMaximumDecodedPacketSize]{};
  writeU16(decoded, kMagic);
  decoded[2] = kProtocolMajor;
  decoded[3] = kProtocolMinor;
  decoded[4] = static_cast<uint8_t>(type);
  decoded[5] = flags;
  writeU16(decoded + 6, payloadLength);
  writeU32(decoded + 8, sequence);
  if (payloadLength > 0) {
    std::memcpy(decoded + kHeaderSize, payload, payloadLength);
  }
  writeU16(decoded + decodedLength - kCrcSize,
           crc16Ccitt(decoded, decodedLength - kCrcSize));

  wireFrame[0] = 0;
  const size_t encodedLength =
      cobsEncode(decoded, decodedLength, wireFrame + 1, wireCapacity - 2);
  if (encodedLength == 0) {
    return 0;
  }
  wireFrame[encodedLength + 1] = 0;
  return encodedLength + 2;
}

inline bool decodePacket(const uint8_t* encoded, size_t encodedLength,
                         uint8_t* decoded, size_t decodedCapacity,
                         DecodedPacket* packet) {
  if (encoded == nullptr || decoded == nullptr || packet == nullptr ||
      encodedLength == 0) {
    return false;
  }
  const size_t decodedLength =
      cobsDecode(encoded, encodedLength, decoded, decodedCapacity);
  if (decodedLength < kHeaderSize + kCrcSize ||
      readU16(decoded) != kMagic ||
      decoded[2] != kProtocolMajor) {
    return false;
  }
  const uint16_t payloadLength = readU16(decoded + 6);
  if (decodedLength !=
      kHeaderSize + static_cast<size_t>(payloadLength) + kCrcSize) {
    return false;
  }
  const uint16_t expectedCrc = readU16(decoded + decodedLength - kCrcSize);
  if (crc16Ccitt(decoded, decodedLength - kCrcSize) != expectedCrc) {
    return false;
  }
  packet->type = static_cast<MessageType>(decoded[4]);
  packet->flags = decoded[5];
  packet->sequence = readU32(decoded + 8);
  packet->payload = decoded + kHeaderSize;
  packet->payloadLength = payloadLength;
  return true;
}

inline bool sequenceIsNewer(uint32_t sequence, uint32_t previous) {
  return static_cast<int32_t>(sequence - previous) > 0;
}

}  // namespace cleany::serial
