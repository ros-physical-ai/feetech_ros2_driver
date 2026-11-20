// SPDX-License-Identifier: Apache-2.0
// SCS0009 conversions adapted from rustypot. See NOTICE-SCS0009.
#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <stdexcept>

namespace feetech_driver::scs0009 {
constexpr uint8_t torque_enable = 40;
constexpr uint8_t goal_position = 42;
constexpr uint8_t goal_time = 44;
constexpr uint8_t goal_speed = 46;
constexpr uint8_t present_position = 56;
constexpr double step = (300.0 * std::numbers::pi / 180.0) / 1024.0;
using Word = std::array<uint8_t, 2>;

inline Word word(uint16_t value) { return {static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value)}; }
inline double position_radians(Word value) {
  const int raw = (value[0] << 8) | value[1];
  if (raw > 1023) throw std::out_of_range("Invalid SCS0009 encoder word");
  return (raw - 511) * step;
}
inline uint16_t position_raw(double radians) {
  const double raw = radians / step + 511;
  if (!(raw >= 0 && raw <= 1023)) throw std::out_of_range("Position outside SCS0009 encoder range");
  return static_cast<uint16_t>(raw);
}
inline uint16_t speed_raw(double radians_per_second) {
  const double raw = std::abs(radians_per_second) / step;
  if (!(raw >= 1 && raw <= 32767)) throw std::out_of_range("Invalid SCS0009 goal speed");
  const auto magnitude = static_cast<uint16_t>(raw);
  return radians_per_second < 0 ? static_cast<uint16_t>(magnitude | 0x8000u) : magnitude;
}
}  // namespace feetech_driver::scs0009
