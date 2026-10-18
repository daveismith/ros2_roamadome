// Copyright (c) 2026
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
#include <gtest/gtest.h>
#include "ros2_roamadome/velocity_map.hpp"

#include <cmath>
#include <cstdint>
#include <limits>

namespace ros2_roamadome
{

namespace
{

// The mapping before phase 7.
int32_t oldLinear(double rad_per_sec, double max_speed)
{
  double percentage = (rad_per_sec / max_speed) * 100.0;
  if (percentage > 100.0) {
    percentage = 100.0;
  } else if (percentage < -100.0) {
    percentage = -100.0;
  }
  return static_cast<int32_t>(std::round(percentage));
}

// Measured calibration, 2026-10-16.
VelocityMap calibration(BelowMinimum below)
{
  return VelocityMap{
    VelocityMapDirection{11.94, 18.3, 32.0, -1},
    VelocityMapDirection{11.75, 18.2, 32.0, 1},
    below};
}

}  // namespace

TEST(VelocityMap, LinearDefaultsEqualOldFormula)
{
  for (const double max_speed : {3.14, 6.58, 1.0}) {
    const VelocityMap map = linearVelocityMap(max_speed);
    for (double v = -12.0; v <= 12.0; v += 0.0137) {
      EXPECT_EQ(velocityToPercent(v, map), oldLinear(v, max_speed)) << "v=" << v;
    }
    for (const double v : {0.0, 1.0, -1.0, 3.14, -3.14, 7.0, -7.0, 100.0, -100.0}) {
      EXPECT_EQ(velocityToPercent(v, map), oldLinear(v, max_speed)) << "v=" << v;
    }
  }
}

TEST(VelocityMap, LinearClampsAtHundred)
{
  const VelocityMap map = linearVelocityMap(3.14);
  EXPECT_EQ(velocityToPercent(10.0, map), 100);
  EXPECT_EQ(velocityToPercent(-10.0, map), -100);
  EXPECT_EQ(velocityToPercent(1.57, map), 50);
}

TEST(VelocityMap, CalibrationPoints)
{
  const VelocityMap map = calibration(BelowMinimum::ZERO);
  EXPECT_EQ(velocityToPercent(1.5, map), -36);
  EXPECT_EQ(velocityToPercent(-1.5, map), 36);
  EXPECT_EQ(velocityToPercent(3.0, map), -54);
  EXPECT_EQ(velocityToPercent(-3.0, map), 53);  // 18.2 + 3 * 11.75 = 53.45
  EXPECT_EQ(velocityToPercent(7.0, map), -100);
  EXPECT_EQ(velocityToPercent(-7.0, map), 100);
}

TEST(VelocityMap, BelowMinimumZero)
{
  const VelocityMap map = calibration(BelowMinimum::ZERO);
  EXPECT_EQ(velocityToPercent(1.0, map), 0);
  EXPECT_EQ(velocityToPercent(-1.0, map), 0);
}

TEST(VelocityMap, BelowMinimumMinimum)
{
  const VelocityMap map = calibration(BelowMinimum::MINIMUM);
  EXPECT_EQ(velocityToPercent(1.0, map), -32);
  EXPECT_EQ(velocityToPercent(-1.0, map), 32);
  EXPECT_EQ(velocityToPercent(0.01, map), -32);
}

TEST(VelocityMap, ZeroAndNanGiveZero)
{
  for (const BelowMinimum below : {BelowMinimum::ZERO, BelowMinimum::MINIMUM}) {
    const VelocityMap map = calibration(below);
    EXPECT_EQ(velocityToPercent(0.0, map), 0);
    EXPECT_EQ(velocityToPercent(-0.0, map), 0);
    EXPECT_EQ(velocityToPercent(std::numeric_limits<double>::quiet_NaN(), map), 0);
  }
}

TEST(VelocityMap, InfinityClampsToHundred)
{
  const VelocityMap map = calibration(BelowMinimum::ZERO);
  EXPECT_EQ(velocityToPercent(std::numeric_limits<double>::infinity(), map), -100);
  EXPECT_EQ(velocityToPercent(-std::numeric_limits<double>::infinity(), map), 100);
}

namespace
{

// k 1, p0 0 so that p equals |v|; min 32.
VelocityMap identityMap(BelowMinimum below, double p0 = 0.0)
{
  return VelocityMap{
    VelocityMapDirection{1.0, p0, 32.0, 1},
    VelocityMapDirection{1.0, p0, 32.0, -1},
    below};
}

}  // namespace

TEST(VelocityMap, RoundedValueIsComparedWithMinimum)
{
  // p in [31.5, 32) rounds to 32, which is not below the minimum.
  for (const BelowMinimum below : {BelowMinimum::ZERO, BelowMinimum::MINIMUM}) {
    const VelocityMap map = identityMap(below);
    EXPECT_EQ(velocityToPercent(31.5, map), 32);
    EXPECT_EQ(velocityToPercent(31.9, map), 32);
    EXPECT_EQ(velocityToPercent(-31.9, map), -32);
    EXPECT_EQ(velocityToPercent(32.0, map), 32);
  }
  EXPECT_EQ(velocityToPercent(31.4, identityMap(BelowMinimum::ZERO)), 0);
  EXPECT_EQ(velocityToPercent(31.4, identityMap(BelowMinimum::MINIMUM)), 32);
}

TEST(VelocityMap, CapAtHundredAfterRounding)
{
  const VelocityMap map = identityMap(BelowMinimum::ZERO);
  EXPECT_EQ(velocityToPercent(99.6, map), 100);
  EXPECT_EQ(velocityToPercent(100.4, map), 100);
  EXPECT_EQ(velocityToPercent(-100.4, map), -100);
  EXPECT_EQ(velocityToPercent(99.4, map), 99);
}

TEST(VelocityMap, MinimumModeWithP0AboveMinimum)
{
  const VelocityMap map = identityMap(BelowMinimum::MINIMUM, 40.0);
  EXPECT_EQ(velocityToPercent(0.01, map), 40);   // 40.01 is not below 32
  EXPECT_EQ(velocityToPercent(-0.01, map), -40);
}

TEST(VelocityMap, TinyVelocityInZeroModeIsZero)
{
  const VelocityMap map = calibration(BelowMinimum::ZERO);
  EXPECT_EQ(velocityToPercent(1e-9, map), 0);
  EXPECT_EQ(velocityToPercent(-1e-9, map), 0);
  EXPECT_EQ(velocityToPercent(0.005, map), 0);
}

TEST(VelocityMap, TinyVelocityInMinimumModeIsMinimum)
{
  const VelocityMap map = calibration(BelowMinimum::MINIMUM);
  EXPECT_EQ(velocityToPercent(0.005, map), -32);
  EXPECT_EQ(velocityToPercent(-0.005, map), 32);
}

}  // namespace ros2_roamadome
