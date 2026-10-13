#include <cassert>
#include <cstdint>
#include <limits>
#include "modular_int32.hpp"

int main() {
  using cleany::modularAdd;
  using cleany::modularDifference;
  using cleany::signedFromModularBits;
  assert(signedFromModularBits(0) == 0);
  assert(signedFromModularBits(0x7fffffffU) ==
         std::numeric_limits<int32_t>::max());
  assert(signedFromModularBits(0x80000000U) ==
         std::numeric_limits<int32_t>::min());
  assert(signedFromModularBits(0xffffffffU) == -1);
  assert(modularAdd(std::numeric_limits<int32_t>::max(), 1) ==
         std::numeric_limits<int32_t>::min());
  assert(modularAdd(std::numeric_limits<int32_t>::min(), -1) ==
         std::numeric_limits<int32_t>::max());
  assert(modularDifference(std::numeric_limits<int32_t>::min(),
                           std::numeric_limits<int32_t>::max()) == 1);
}
