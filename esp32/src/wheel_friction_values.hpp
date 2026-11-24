#pragma once

#include "wheel_friction.hpp"

namespace cleany {
// Unloaded single-wheel measurements. Knots are at 0.2, 0.3, 0.4, 0.5 rad/s.
constexpr WheelFriction kFlFriction{
    {5.859F, {3.483F, 4.002F, 4.635F, 5.345F}},  // forward
    {4.688F, {3.305F, 3.888F, 4.547F, 5.298F}},  // reverse
};
constexpr WheelFriction kFrFriction{
    {6.641F, {4.891F, 5.459F, 6.074F, 6.762F}},  // forward
    {5.859F, {4.406F, 5.053F, 5.829F, 6.537F}},  // reverse
};
constexpr WheelFriction kRlFriction{
    {5.859F, {4.363F, 4.976F, 5.726F, 6.479F}},  // forward
    {5.859F, {4.036F, 4.713F, 5.477F, 6.203F}},  // reverse
};
constexpr WheelFriction kRrFriction{
    {7.812F, {6.053F, 6.485F, 7.139F, 7.848F}},  // forward
    {7.812F, {5.833F, 6.596F, 7.432F, 8.100F}},  // reverse
};
}  // namespace cleany
