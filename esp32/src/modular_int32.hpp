#pragma once

#include <cstdint>
#include <limits>

namespace cleany {
inline int32_t signedFromModularBits(uint32_t bits) {
  constexpr uint32_t kSignBit = uint32_t{1} << 31;
  if (bits < kSignBit) return static_cast<int32_t>(bits);
  return std::numeric_limits<int32_t>::min() +
         static_cast<int32_t>(bits - kSignBit);
}

inline int32_t modularAdd(int32_t value, int32_t delta) {
  return signedFromModularBits(static_cast<uint32_t>(value) +
                               static_cast<uint32_t>(delta));
}

inline int32_t modularDifference(int32_t newer, int32_t older) {
  return signedFromModularBits(static_cast<uint32_t>(newer) -
                               static_cast<uint32_t>(older));
}
}  // namespace cleany
