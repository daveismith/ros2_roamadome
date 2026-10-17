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

#ifndef ROS2_ROAMADOME__DOME_DIAGNOSTICS_HPP_
#define ROS2_ROAMADOME__DOME_DIAGNOSTICS_HPP_

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "ros2_roamadome/firmware_line_classifier.hpp"
#include "ros2_roamadome/visibility_control.h"

namespace ros2_roamadome
{

/// @brief Counters and last-seen values accumulated from firmware console lines.
///
/// Owned by the hardware interface, reset on activation, mutated from the serial
/// read path and read by the 1 Hz diagnostics timer (the owner serialises access).
/// Pure data so the update and reporting logic below is testable without ROS.
struct DomeDiagnosticsState
{
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  // Link
  uint64_t position_reports = 0;
  std::optional<TimePoint> last_position_report;
  /// Report intervals longer than the warning threshold, and the last one seen.
  uint32_t report_gaps = 0;
  double last_report_gap_s = 0.0;
  bool position_reporting_enabled = false;
  uint32_t status_polls_answered = 0;
  std::optional<TimePoint> last_status_response;
  /// READY seen while the interface was active: the controller rebooted.
  bool reboot_detected = false;
  std::optional<TimePoint> last_ready;

  // Sensor
  std::optional<uint32_t> sensor_errors;
  /// Sensor error count grew between the two most recent status polls.
  bool sensor_errors_increased = false;
  std::optional<uint32_t> packet_serial_errors;
  /// Plausibility gate (max_plausible_deg_s): reports held for confirmation,
  /// dropped as glitches, and held reports the next one confirmed as real jumps.
  uint32_t reports_held = 0;
  uint32_t reports_rejected = 0;
  uint32_t jumps_confirmed = 0;
  /// Reports accepted although implausible after repeated disagreement (gate too tight).
  uint32_t gate_resyncs = 0;
  std::optional<TimePoint> last_report_rejected;

  // Motion
  uint32_t position_timeouts = 0;
  std::optional<TimePoint> last_position_timeout;
  /// False from a position timeout until the next REACHED TARGET / REACHED HOME.
  bool move_succeeded_since_timeout = true;
  std::optional<int32_t> last_accepted_rotate_percent;
  std::optional<int32_t> last_rotary_target_deg;
  std::optional<double> last_rotary_speed;
  std::optional<int32_t> last_target_reached_deg;
  std::optional<int32_t> last_home_reached_deg;
  std::optional<TimePoint> last_move_completed;
  uint32_t out_of_range_rejections = 0;
  uint32_t setup_failures = 0;
  std::optional<int32_t> setup_good_max_speed;

  // Settings writes (ack-tracked #DP... commands from device.* writes and set_home_here)
  /// Completed by the firmware: "Write Settings" + "Updated", or "Unchanged".
  uint32_t settings_acks = 0;
  /// Abandoned after the ack timeout without a reply (the write may not be in flash).
  uint32_t settings_ack_timeouts = 0;
};

/// @brief Live values the diagnostics report alongside the accumulated state.
struct DomeDiagnosticsReadings
{
  std::string hardware_id;
  bool serial_open = false;
  double position_rad = 0.0;
  double velocity_rad_s = 0.0;
  int32_t commanded_percent = 0;
  /// 0 idle, 1 position, 2 velocity, 3 bench (astromech_msgs/DomeTelemetry).
  uint8_t command_mode = 0;
  /// Ack-tracked settings writes queued, plus one if a write is waiting for its ack.
  uint32_t settings_writes_pending = 0;
};

struct DomeDiagnosticsOptions
{
  /// Link is WARN when no position report arrived within this many seconds.
  double report_stale_s = 0.5;
  /// Motion stays ERROR this long after a position timeout, then WARN.
  double timeout_error_s = 5.0;
  /// Sensor is WARN this long after the plausibility gate rejected a report.
  double rejection_warn_s = 10.0;
};

/// @brief Fold one classified console line into the state.
/// @param lifecycle_active True when the interface is active; READY then means a reboot.
ros2_roamadome_PUBLIC
void applyFirmwareLine(
  DomeDiagnosticsState * state,
  const FirmwareLine & line,
  DomeDiagnosticsState::TimePoint now,
  bool lifecycle_active);

/// @brief Fold a #DPSTATUS section into the state.
///
/// The firmware prints "Packet Serial Errors" only when the count is non-zero,
/// so a section that carries a sensor error line but no packet error line sets
/// the packet count to zero. Sections without a sensor error line are ignored.
ros2_roamadome_PUBLIC
void applyStatusSection(
  DomeDiagnosticsState * state,
  const std::vector<std::string> & lines,
  DomeDiagnosticsState::TimePoint now);

/// @brief Note that a "DOME POSITION:" report arrived.
///
/// A flowing report also proves that position reporting is enabled, whether or
/// not the "Position Reporting Enabled" ack of the startup #DPREPORT was seen.
/// @return the interval since the previous report in seconds (0 for the first one)
ros2_roamadome_PUBLIC
double recordPositionReport(
  DomeDiagnosticsState * state,
  DomeDiagnosticsState::TimePoint now,
  double gap_warn_s = 0.0);

/// @brief Build the three statuses "roamadome: link", "roamadome: sensor" and
/// "roamadome: motion" for a diagnostic_msgs/DiagnosticArray.
ros2_roamadome_PUBLIC
std::vector<diagnostic_msgs::msg::DiagnosticStatus> buildDomeDiagnostics(
  const DomeDiagnosticsState & state,
  const DomeDiagnosticsReadings & readings,
  const DomeDiagnosticsOptions & options,
  DomeDiagnosticsState::TimePoint now);

}  // namespace ros2_roamadome

#endif  // ROS2_ROAMADOME__DOME_DIAGNOSTICS_HPP_
