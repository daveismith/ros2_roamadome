// Copyright (c) 2026
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROS2_ROAMADOME__POSITION_TRACKER_HPP_
#define ROS2_ROAMADOME__POSITION_TRACKER_HPP_

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>

namespace ros2_roamadome
{

/// @brief Unwraps the firmware's integer dome angle and estimates angular velocity.
///
/// Header-only, no ROS dependency. Feed it every "DOME POSITION: <n>" report with
/// the host receive time. It accumulates a continuous (unwrapped) angle using the
/// shortest signed step between successive readings, keeps the samples from the
/// trailing window, and returns the least-squares slope of unwrapped angle against
/// time as the velocity estimate.
///
/// The velocity is NaN until the window holds at least `min_samples` samples that
/// span at least `min_span_fraction` of the window. Adjacent samples are never
/// differentiated directly: one degree of quantisation at a 28 ms report interval
/// would be 36 deg/s of noise per sample.
class PositionTracker
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  struct Estimate
  {
    /// Reported angle in radians, wrapped to [0, 2*pi).
    double position_rad;
    /// Continuous angle in radians since the first sample after reset().
    double unwrapped_rad;
    /// Regression velocity in rad/s, or NaN while the window is not populated.
    double velocity_rad_s;
  };

  /// @param window_s Trailing regression window in seconds (must be > 0)
  /// @param min_samples Minimum samples in the window before a slope is reported
  /// @param min_span_fraction Minimum fraction of window_s the samples must span
  explicit PositionTracker(
    double window_s = 0.5,
    std::size_t min_samples = 4,
    double min_span_fraction = 0.4)
  : window_s_(window_s > 0.0 ? window_s : 0.5),
    min_samples_(min_samples < 2 ? 2 : min_samples),
    min_span_fraction_(min_span_fraction)
  {
  }

  /// @brief Forget all samples; the next update() starts a new unwrapped angle at 0 offset.
  void reset()
  {
    have_last_ = false;
    last_deg_ = 0;
    unwrapped_deg_ = 0.0;
    samples_.clear();
  }

  /// @brief Feed one position report.
  /// @param time Host receive time of the report line
  /// @param degrees Home-relative integer angle from the firmware, 0..359
  /// @return Wrapped angle, unwrapped angle and velocity estimate
  Estimate update(TimePoint time, uint16_t degrees)
  {
    const uint16_t wrapped = static_cast<uint16_t>(degrees % 360);

    if (!have_last_) {
      unwrapped_deg_ = static_cast<double>(wrapped);
      have_last_ = true;
    } else {
      unwrapped_deg_ += static_cast<double>(shortestDelta(last_deg_, wrapped));
    }
    last_deg_ = wrapped;

    samples_.push_back({time, unwrapped_deg_});
    pruneOlderThanWindow(time);

    Estimate estimate;
    estimate.position_rad = degreesToRadians(static_cast<double>(wrapped));
    estimate.unwrapped_rad = degreesToRadians(unwrapped_deg_);
    estimate.velocity_rad_s = degreesToRadians(regressionSlopeDegPerSec());
    return estimate;
  }

  double windowSeconds() const {return window_s_;}
  std::size_t sampleCount() const {return samples_.size();}
  bool hasSamples() const {return have_last_;}

  /// @brief Shortest signed step from `from` to `to` on a 360 degree circle, in [-180, 180).
  static int shortestDelta(uint16_t from, uint16_t to)
  {
    int delta = (static_cast<int>(to) - static_cast<int>(from) + 180) % 360;
    if (delta < 0) {
      delta += 360;
    }
    return delta - 180;
  }

private:
  struct Sample
  {
    TimePoint time;
    double unwrapped_deg;
  };

  static double degreesToRadians(double degrees)
  {
    return degrees * (M_PI / 180.0);
  }

  void pruneOlderThanWindow(TimePoint now)
  {
    const auto window = std::chrono::duration<double>(window_s_);
    while (!samples_.empty() &&
      std::chrono::duration<double>(now - samples_.front().time) > window)
    {
      samples_.pop_front();
    }
  }

  /// Least-squares slope of unwrapped degrees against seconds over the window, or NaN.
  double regressionSlopeDegPerSec() const
  {
    constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

    if (samples_.size() < min_samples_) {
      return kNaN;
    }

    const double span_s = std::chrono::duration<double>(
      samples_.back().time - samples_.front().time).count();
    if (span_s < min_span_fraction_ * window_s_) {
      return kNaN;
    }

    const TimePoint origin = samples_.front().time;
    const double n = static_cast<double>(samples_.size());
    double sum_x = 0.0;
    double sum_y = 0.0;
    for (const Sample & sample : samples_) {
      sum_x += std::chrono::duration<double>(sample.time - origin).count();
      sum_y += sample.unwrapped_deg;
    }
    const double mean_x = sum_x / n;
    const double mean_y = sum_y / n;

    double sxy = 0.0;
    double sxx = 0.0;
    for (const Sample & sample : samples_) {
      const double dx = std::chrono::duration<double>(sample.time - origin).count() - mean_x;
      const double dy = sample.unwrapped_deg - mean_y;
      sxy += dx * dy;
      sxx += dx * dx;
    }

    if (sxx <= 0.0) {
      return kNaN;
    }
    return sxy / sxx;
  }

  double window_s_;
  std::size_t min_samples_;
  double min_span_fraction_;
  bool have_last_ = false;
  uint16_t last_deg_ = 0;
  double unwrapped_deg_ = 0.0;
  std::deque<Sample> samples_;
};

}  // namespace ros2_roamadome

#endif  // ROS2_ROAMADOME__POSITION_TRACKER_HPP_
