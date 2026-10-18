// Copyright (c) 2026
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//

#ifndef ROS2_ROAMADOME__VELOCITY_MAP_HPP_
#define ROS2_ROAMADOME__VELOCITY_MAP_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace ros2_roamadome
{

/// Calibration of one direction of rotation (sign of the joint velocity).
struct VelocityMapDirection
{
  double k_percent_per_rad_s;  ///< slope of the fit, % per rad/s
  double p0_percent;           ///< intercept of the fit, %
  double min_percent;          ///< smallest percent that moves the dome, 0..100
  int percent_sign;            ///< +1 or -1: sign of the percent sent in this direction
};

/// What to send when the fit gives less than min_percent.
enum class BelowMinimum
{
  ZERO,     ///< send 0 (stop)
  MINIMUM   ///< send min_percent
};

struct VelocityMap
{
  VelocityMapDirection positive;  ///< used for a positive joint velocity
  VelocityMapDirection negative;  ///< used for a negative joint velocity
  BelowMinimum below_minimum;
};

/// Converts a joint velocity (rad/s) to the signed :DPR percent.
/// 0 and NaN give 0. Otherwise p = p0 + |v| * k for the direction of v, replaced by
/// 0 or min_percent when below min_percent, capped at 100, rounded, and given the sign
/// percent_sign of the direction of v (+1 for the positive direction and -1 for the
/// negative one is the identity mapping; the opposite signs flip the direction).
inline int32_t velocityToPercent(double rad_per_sec, const VelocityMap & map)
{
  if (std::isnan(rad_per_sec) || rad_per_sec == 0.0) {
    return 0;
  }
  const bool positive = rad_per_sec > 0.0;
  const VelocityMapDirection & dir = positive ? map.positive : map.negative;
  double percent = dir.p0_percent + std::fabs(rad_per_sec) * dir.k_percent_per_rad_s;
  if (percent < dir.min_percent) {
    percent = (map.below_minimum == BelowMinimum::MINIMUM) ? dir.min_percent : 0.0;
  }
  percent = std::min(percent, 100.0);
  const int32_t magnitude = static_cast<int32_t>(std::round(percent));
  return dir.percent_sign * magnitude;
}

/// The original linear mapping (percent sign +1 for v > 0, -1 for v < 0): percent = v / max_speed * 100, clamped to +-100.
inline VelocityMap linearVelocityMap(double max_speed_rad_per_sec)
{
  const double k = 100.0 / max_speed_rad_per_sec;
  return VelocityMap{
    VelocityMapDirection{k, 0.0, 0.0, 1},
    VelocityMapDirection{k, 0.0, 0.0, -1},
    BelowMinimum::ZERO};
}

}  // namespace ros2_roamadome

#endif  // ROS2_ROAMADOME__VELOCITY_MAP_HPP_
