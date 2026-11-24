#pragma once

#include "driver/gpio.h"

namespace cleany::board {
struct WheelPins {
  gpio_num_t pwm, direction, encoderA, encoderB;
};

// Physical connector order: M1 FL, M2 FR, M3 RR, M4 RL.
#ifdef CLEANY_INTEGRATED
constexpr WheelPins wheels[] = {
    {GPIO_NUM_8, GPIO_NUM_18, GPIO_NUM_17, GPIO_NUM_16},
    {GPIO_NUM_15, GPIO_NUM_7, GPIO_NUM_6, GPIO_NUM_5},
    {GPIO_NUM_12, GPIO_NUM_11, GPIO_NUM_10, GPIO_NUM_9},
    {GPIO_NUM_38, GPIO_NUM_21, GPIO_NUM_14, GPIO_NUM_13},
};
constexpr auto tofSda = GPIO_NUM_1;
constexpr auto tofScl = GPIO_NUM_2;
constexpr auto imuSda = GPIO_NUM_41;
constexpr auto imuScl = GPIO_NUM_42;
constexpr auto batteryAdc = GPIO_NUM_3;
constexpr auto ledData = GPIO_NUM_4;
#else
constexpr WheelPins wheels[] = {
    {GPIO_NUM_2, GPIO_NUM_1, GPIO_NUM_10, GPIO_NUM_9},
    {GPIO_NUM_12, GPIO_NUM_11, GPIO_NUM_14, GPIO_NUM_13},
    {GPIO_NUM_5, GPIO_NUM_6, GPIO_NUM_7, GPIO_NUM_15},
    {GPIO_NUM_16, GPIO_NUM_17, GPIO_NUM_18, GPIO_NUM_8},
};
#endif
}  // namespace cleany::board
