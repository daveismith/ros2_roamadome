#include <gtest/gtest.h>
#include "ros2_roamadome/position_tracker.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>

namespace ros2_roamadome
{

namespace
{

constexpr double kDegToRad = M_PI / 180.0;

PositionTracker::TimePoint at(double seconds)
{
  return PositionTracker::TimePoint(
    std::chrono::duration_cast<PositionTracker::Clock::duration>(
      std::chrono::duration<double>(seconds)));
}

}  // namespace

TEST(PositionTracker, ShortestDelta_WrapsAtBoundary)
{
  EXPECT_EQ(PositionTracker::shortestDelta(359, 0), 1);
  EXPECT_EQ(PositionTracker::shortestDelta(0, 359), -1);
  EXPECT_EQ(PositionTracker::shortestDelta(350, 10), 20);
  EXPECT_EQ(PositionTracker::shortestDelta(10, 350), -20);
  EXPECT_EQ(PositionTracker::shortestDelta(100, 100), 0);
  EXPECT_EQ(PositionTracker::shortestDelta(0, 180), -180);
  EXPECT_EQ(PositionTracker::shortestDelta(0, 179), 179);
}

TEST(PositionTracker, Unwrap_ForwardAcross359To0)
{
  PositionTracker tracker(0.5);
  double t = 0.0;
  PositionTracker::Estimate estimate{};
  for (uint16_t deg : {357, 358, 359, 0, 1, 2}) {
    estimate = tracker.update(at(t), deg);
    t += 0.028;
  }

  // 357 -> 2 forwards is +5 degrees of travel from a start of 357.
  EXPECT_NEAR(estimate.unwrapped_rad, 362.0 * kDegToRad, 1e-9);
  EXPECT_NEAR(estimate.position_rad, 2.0 * kDegToRad, 1e-9);
}

TEST(PositionTracker, Unwrap_BackwardAcross0To359)
{
  PositionTracker tracker(0.5);
  double t = 0.0;
  PositionTracker::Estimate estimate{};
  for (uint16_t deg : {2, 1, 0, 359, 358, 357}) {
    estimate = tracker.update(at(t), deg);
    t += 0.028;
  }

  // 2 -> 357 backwards is -5 degrees of travel from a start of 2.
  EXPECT_NEAR(estimate.unwrapped_rad, -3.0 * kDegToRad, 1e-9);
  EXPECT_NEAR(estimate.position_rad, 357.0 * kDegToRad, 1e-9);
}

TEST(PositionTracker, Unwrap_MultipleRevolutionsAccumulate)
{
  PositionTracker tracker(0.5);
  double t = 0.0;
  PositionTracker::Estimate estimate{};
  // Three full forward revolutions in 10 degree steps.
  for (int step = 0; step <= 108; ++step) {
    estimate = tracker.update(at(t), static_cast<uint16_t>((step * 10) % 360));
    t += 0.028;
  }
  EXPECT_NEAR(estimate.unwrapped_rad, 1080.0 * kDegToRad, 1e-9);
  EXPECT_NEAR(estimate.position_rad, 0.0, 1e-9);
}

TEST(PositionTracker, Velocity_NaNUntilWindowPopulated_ThenWithinTwoPercent)
{
  const double window_s = 0.5;
  const double true_deg_s = 90.0;
  PositionTracker tracker(window_s);

  // An exact constant-velocity stream with no quantisation error: the reports
  // are integer degrees, so advance one degree per 1/90 s.
  const double exact_dt = 1.0 / true_deg_s;

  double t = 0.0;
  int deg = 0;
  PositionTracker::Estimate estimate{};

  // Fewer than 4 samples: must be NaN.
  for (int i = 0; i < 3; ++i) {
    estimate = tracker.update(at(t), static_cast<uint16_t>(deg % 360));
    EXPECT_TRUE(std::isnan(estimate.velocity_rad_s)) << "sample " << i;
    t += exact_dt;
    ++deg;
  }

  // Keep feeding until the samples span at least 40% of the window.
  // Before that span is reached the estimate must stay NaN.
  while (t < 0.4 * window_s - 1e-9) {
    estimate = tracker.update(at(t), static_cast<uint16_t>(deg % 360));
    EXPECT_TRUE(std::isnan(estimate.velocity_rad_s)) << "t=" << t;
    t += exact_dt;
    ++deg;
  }

  // Run on until the window is completely full, then check.
  while (t < 2.0 * window_s) {
    estimate = tracker.update(at(t), static_cast<uint16_t>(deg % 360));
    t += exact_dt;
    ++deg;
  }

  ASSERT_FALSE(std::isnan(estimate.velocity_rad_s));
  const double expected = true_deg_s * kDegToRad;
  EXPECT_NEAR(estimate.velocity_rad_s, expected, 0.02 * expected);
  EXPECT_LE(tracker.sampleCount(), static_cast<std::size_t>(window_s / exact_dt) + 1);
}

TEST(PositionTracker, Velocity_NegativeDirectionHasNegativeSign)
{
  const double window_s = 0.5;
  PositionTracker tracker(window_s);
  double t = 0.0;
  int deg = 720;
  PositionTracker::Estimate estimate{};
  for (int i = 0; i < 100; ++i) {
    estimate = tracker.update(at(t), static_cast<uint16_t>(deg % 360));
    t += 1.0 / 90.0;
    --deg;
  }
  ASSERT_FALSE(std::isnan(estimate.velocity_rad_s));
  EXPECT_LT(estimate.velocity_rad_s, 0.0);
  EXPECT_LT(estimate.unwrapped_rad, 0.0);
  EXPECT_NEAR(estimate.velocity_rad_s, -90.0 * kDegToRad, 0.02 * 90.0 * kDegToRad);
}

TEST(PositionTracker, Velocity_QuantisedStreamAt30DegPerSecWithinFivePercent)
{
  const double window_s = 0.5;
  const double true_deg_s = 30.0;
  const double dt = 0.028;
  PositionTracker tracker(window_s);

  double worst_error = 0.0;
  const double expected = true_deg_s * kDegToRad;

  // Run for 3 seconds; once the window is full every estimate must be within 5%.
  for (int i = 0; i < static_cast<int>(3.0 / dt); ++i) {
    const double t = i * dt;
    const double true_deg = 17.3 + true_deg_s * t;
    const uint16_t reported = static_cast<uint16_t>(
      static_cast<long>(std::floor(true_deg)) % 360);
    const auto estimate = tracker.update(at(t), reported);

    if (t >= window_s) {
      ASSERT_FALSE(std::isnan(estimate.velocity_rad_s)) << "t=" << t;
      const double error = std::fabs(estimate.velocity_rad_s - expected) / expected;
      worst_error = std::max(worst_error, error);
      EXPECT_LE(error, 0.05) << "t=" << t << " velocity=" << estimate.velocity_rad_s;
    }
  }
  EXPECT_LE(worst_error, 0.05);
}

TEST(PositionTracker, Velocity_StationaryIsZero)
{
  PositionTracker tracker(0.5);
  PositionTracker::Estimate estimate{};
  for (int i = 0; i < 40; ++i) {
    estimate = tracker.update(at(i * 0.028), 123);
  }
  ASSERT_FALSE(std::isnan(estimate.velocity_rad_s));
  EXPECT_DOUBLE_EQ(estimate.velocity_rad_s, 0.0);
  EXPECT_NEAR(estimate.unwrapped_rad, 123.0 * kDegToRad, 1e-9);
}

TEST(PositionTracker, Reset_ClearsHistoryAndRestartsUnwrap)
{
  PositionTracker tracker(0.5);
  for (int i = 0; i < 40; ++i) {
    tracker.update(at(i * 0.028), static_cast<uint16_t>((i * 3) % 360));
  }
  tracker.reset();
  EXPECT_FALSE(tracker.hasSamples());
  EXPECT_EQ(tracker.sampleCount(), 0u);

  const auto estimate = tracker.update(at(10.0), 200);
  EXPECT_NEAR(estimate.unwrapped_rad, 200.0 * kDegToRad, 1e-9);
  EXPECT_TRUE(std::isnan(estimate.velocity_rad_s));
}

TEST(PositionTracker, Velocity_RequiresMinimumSpanNotJustSampleCount)
{
  // Four samples 1 ms apart do not span 40% of a 0.5 s window.
  PositionTracker tracker(0.5);
  PositionTracker::Estimate estimate{};
  for (int i = 0; i < 4; ++i) {
    estimate = tracker.update(at(i * 0.001), static_cast<uint16_t>(i));
  }
  EXPECT_TRUE(std::isnan(estimate.velocity_rad_s));
}

}  // namespace ros2_roamadome

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
