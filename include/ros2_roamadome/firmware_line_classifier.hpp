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

#ifndef ROS2_ROAMADOME__FIRMWARE_LINE_CLASSIFIER_HPP_
#define ROS2_ROAMADOME__FIRMWARE_LINE_CLASSIFIER_HPP_

#include <cstdint>
#include <optional>
#include <string>

#include "ros2_roamadome/visibility_control.h"

namespace ros2_roamadome
{

/// @brief The console lines the Roam-A-Dome firmware prints that the interface
/// used to drop (docs/dome_diagnostics_plan.md, section 2.5).
enum class FirmwareLineKind
{
  UNKNOWN,
  ROTATE_SPEED,                 ///< "ROTATE SPEED: <p>" accepted, clamped :DPR percent
  ROTARY_DEGREE,                ///< "ROTARY DEGREE: <d> speed: <f>" accepted absolute or relative move
  REACHED_TARGET,               ///< "REACHED TARGET: <n>" position move finished at home-relative n
  REACHED_HOME,                 ///< "REACHED HOME: <n>" home move finished
  POSITION_TIMEOUT,             ///< "ERROR: DOME POSITION TIMEOUT" firmware aborted a move
  SENSOR_ERRORS,                ///< "Dome Sensor Errors: <n>" or "No Dome Sensor Errors" (value 0)
  PACKET_SERIAL_ERRORS,         ///< "Packet Serial Errors: <n>"
  POSITION_REPORTING_ENABLED,   ///< "Position Reporting Enabled"
  POSITION_REPORTING_DISABLED,  ///< "Position Reporting Disabled"
  SETUP_SPEED,                  ///< "SPEED: <n>" during #DPSETUP
  SETUP_ANGULAR_VELOCITY,       ///< "Angular velocity: <f> cm/s" during #DPSETUP
  SETUP_GOOD_MAX_SPEED,         ///< "GOOD MAX SPEED: <n>" during #DPSETUP
  SETUP_FAILED,                 ///< "AUTO SETUP FAILED"
  OUT_OF_RANGE,                 ///< "Out of range." setting rejected
  INVALID,                      ///< "Invalid" unknown command (also the #DPINVALID probe reply)
  UNCHANGED,                    ///< "Unchanged" setting write was a no-op
  READY                         ///< "READY" firmware finished boot
};

struct FirmwareLine
{
  FirmwareLineKind kind = FirmwareLineKind::UNKNOWN;
  /// Integer payload: percent, degrees or a count, depending on kind.
  std::optional<int32_t> value;
  /// Floating payload: the speed of a ROTARY_DEGREE line or the setup angular velocity.
  std::optional<double> speed;
};

/// @brief Map one console line to its kind and parsed numbers.
/// Leading and trailing whitespace (including a carriage return) is ignored.
/// Lines that match no known pattern return FirmwareLineKind::UNKNOWN.
ros2_roamadome_PUBLIC
FirmwareLine classifyFirmwareLine(const std::string & line);

/// @brief Stable lower-case name of a kind, for logs and diagnostics values.
ros2_roamadome_PUBLIC
const char * firmwareLineKindName(FirmwareLineKind kind);

}  // namespace ros2_roamadome

#endif  // ROS2_ROAMADOME__FIRMWARE_LINE_CLASSIFIER_HPP_
