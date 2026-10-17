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

#ifndef ROS2_ROAMADOME__REPORT_PLAUSIBILITY_GATE_HPP_
#define ROS2_ROAMADOME__REPORT_PLAUSIBILITY_GATE_HPP_

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <optional>

#include "ros2_roamadome/position_tracker.hpp"

namespace ros2_roamadome
{

/// @brief Holds back a position report that implies an impossible speed until the
/// next report confirms it.
///
/// Header-only, no ROS dependency. A sensor-ring misread shows up as a single
/// report far from the previous one, followed by a report back on the track
/// (docs/bench_results/2026-10-15-sensor-ring-raw-debug.md). Fed straight into the
/// unwrap, such a report shifts the continuous angle by up to a full turn. The
/// gate judges each report against the last accepted one: a step larger than
/// `max_deg_s * dt + slack_deg` (plus `wrap_allowance_deg` when the step crosses
/// 0, where the reported angle skips about 9 degrees) is held. The next report
/// decides: if it is plausible from the last accepted report, the held one was a
/// glitch and is dropped; if it is only plausible from the held one, the dome
/// really moved and both are accepted, in order; if it is plausible from neither,
/// the held one is dropped and the new one is held. A real report is therefore
/// delayed by one report interval, and lost only if the one after it disagrees
/// with it. If `kResyncAfter` reports in a row disagree with the accepted history
/// (the dome is faster than `max_deg_s`, or the history is wrong), the newest one
/// is accepted as a resync so the position can never freeze.
///
/// Two reports read in the same controller cycle are stamped microseconds apart
/// although the firmware sent them a report interval apart, so the interval used
/// in the test is never shorter than `min_dt_s` (the report interval).
///
/// `max_deg_s <= 0` disables the gate: every report is accepted at once.
class ReportPlausibilityGate
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  struct Report
  {
    TimePoint time;
    uint16_t degrees = 0;
  };

  struct Decision
  {
    /// Reports to apply, in order (0, 1 or 2).
    std::array<Report, 2> accepted{};
    std::size_t accepted_count = 0;
    /// The new report was held for confirmation.
    bool held = false;
    /// A previously held report was dropped as a glitch (`dropped` says which).
    bool rejected = false;
    Report dropped{};
    /// A previously held report was confirmed by this one: a real jump.
    bool confirmed = false;
    /// Accepted although implausible, after kResyncAfter disagreements in a row.
    bool resynced = false;
  };

  /// Consecutive reports disagreeing with the accepted history before a resync.
  static constexpr uint32_t kResyncAfter = 3;

  explicit ReportPlausibilityGate(
    double max_deg_s = 0.0,
    double min_dt_s = 0.028,
    double slack_deg = 2.0,
    double wrap_allowance_deg = 10.0)
  : max_deg_s_(max_deg_s),
    min_dt_s_(min_dt_s > 0.0 ? min_dt_s : 0.0),
    slack_deg_(slack_deg),
    wrap_allowance_deg_(wrap_allowance_deg)
  {
  }

  bool enabled() const {return max_deg_s_ > 0.0;}
  double maxDegPerSec() const {return max_deg_s_;}
  uint32_t heldCount() const {return held_count_;}
  uint32_t rejectedCount() const {return rejected_count_;}
  uint32_t confirmedCount() const {return confirmed_count_;}
  uint32_t resyncCount() const {return resync_count_;}
  bool holding() const {return held_.has_value();}

  /// @brief Forget the history; the counters are kept (see resetCounters()).
  void reset()
  {
    last_.reset();
    held_.reset();
    disagreements_ = 0;
  }

  /// @brief Zero the counters (the owner's diagnostics are "since activation").
  void resetCounters()
  {
    held_count_ = 0;
    rejected_count_ = 0;
    confirmed_count_ = 0;
    resync_count_ = 0;
  }

  /// @brief Judge one report.
  Decision update(TimePoint time, uint16_t degrees)
  {
    const Report report{time, static_cast<uint16_t>(degrees % 360)};
    Decision decision;
    if (!enabled() || !last_.has_value()) {
      accept(&decision, report);
      return decision;
    }

    if (held_.has_value()) {
      if (plausible(*last_, report)) {
        // Back on the track: the held report was a glitch.
        drop(&decision);
        accept(&decision, report);
        return decision;
      }
      if (plausible(*held_, report)) {
        // Two reports agree with each other but not with the history: a real jump.
        const Report confirmed = *held_;
        held_.reset();
        disagreements_ = 0;
        ++confirmed_count_;
        decision.confirmed = true;
        accept(&decision, confirmed);
        accept(&decision, report);
        return decision;
      }
      // Agrees with nothing: drop the held report and hold this one instead.
      drop(&decision);
    }

    if (plausible(*last_, report)) {
      accept(&decision, report);
      return decision;
    }
    if (++disagreements_ >= kResyncAfter) {
      // Reports keep disagreeing with the history (for example a dome faster than
      // max_deg_s, whose every report is a little beyond the allowance): the history
      // is what is wrong. Accept the newest report rather than freeze.
      ++resync_count_;
      decision.resynced = true;
      accept(&decision, report);
      return decision;
    }
    held_ = report;
    ++held_count_;
    decision.held = true;
    return decision;
  }

  /// @brief True when the shortest step from `from` to `to` passes through 0 degrees.
  static bool crossesZero(uint16_t from, int delta)
  {
    const int end = static_cast<int>(from) + delta;
    return end < 0 || end >= 360;
  }

  /// @brief Largest step allowed between two reports `dt_s` apart.
  double allowedStepDeg(double dt_s, bool crosses_zero) const
  {
    const double dt = dt_s < min_dt_s_ ? min_dt_s_ : dt_s;
    return max_deg_s_ * dt + slack_deg_ + (crosses_zero ? wrap_allowance_deg_ : 0.0);
  }

  bool plausible(const Report & from, const Report & to) const
  {
    const int delta = PositionTracker::shortestDelta(from.degrees, to.degrees);
    const double dt_s = std::chrono::duration<double>(to.time - from.time).count();
    return std::abs(delta) <= allowedStepDeg(dt_s, crossesZero(from.degrees, delta));
  }

private:
  void accept(Decision * decision, const Report & report)
  {
    if (decision->accepted_count < decision->accepted.size()) {
      decision->accepted[decision->accepted_count++] = report;
    }
    last_ = report;
    disagreements_ = 0;
  }

  void drop(Decision * decision)
  {
    decision->rejected = true;
    decision->dropped = *held_;
    held_.reset();
    ++rejected_count_;
  }

  double max_deg_s_;
  double min_dt_s_;
  double slack_deg_;
  double wrap_allowance_deg_;
  std::optional<Report> last_;
  std::optional<Report> held_;
  uint32_t disagreements_ = 0;
  uint32_t held_count_ = 0;
  uint32_t rejected_count_ = 0;
  uint32_t confirmed_count_ = 0;
  uint32_t resync_count_ = 0;
};

}  // namespace ros2_roamadome

#endif  // ROS2_ROAMADOME__REPORT_PLAUSIBILITY_GATE_HPP_
