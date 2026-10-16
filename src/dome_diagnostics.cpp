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

#include "ros2_roamadome/dome_diagnostics.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>

#include "diagnostic_msgs/msg/key_value.hpp"

namespace ros2_roamadome
{

namespace
{

using TimePoint = DomeDiagnosticsState::TimePoint;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;

double secondsSince(const std::optional<TimePoint> & then, TimePoint now)
{
  if (!then.has_value()) {
    return -1.0;
  }
  return std::chrono::duration<double>(now - *then).count();
}

std::string formatDouble(double value, int precision = 3)
{
  if (std::isnan(value)) {
    return "nan";
  }
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

std::string formatAge(const std::optional<TimePoint> & then, TimePoint now)
{
  if (!then.has_value()) {
    return "never";
  }
  return formatDouble(secondsSince(then, now), 1);
}

template<typename T>
std::string formatOptional(const std::optional<T> & value)
{
  if (!value.has_value()) {
    return "unknown";
  }
  std::ostringstream stream;
  stream << *value;
  return stream.str();
}

const char * commandModeName(uint8_t mode)
{
  switch (mode) {
    case 1: return "position";
    case 2: return "velocity";
    case 3: return "bench";
    case 0:
    default:
      return "idle";
  }
}

void addValue(DiagnosticStatus * status, const char * key, const std::string & value)
{
  KeyValue entry;
  entry.key = key;
  entry.value = value;
  status->values.push_back(entry);
}

DiagnosticStatus makeStatus(const char * name, const std::string & hardware_id)
{
  DiagnosticStatus status;
  status.name = name;
  status.hardware_id = hardware_id;
  status.level = DiagnosticStatus::OK;
  return status;
}

/// Raise the level and set the message, keeping the most severe level and the
/// first message of that level.
void raise(DiagnosticStatus * status, unsigned char level, const std::string & message)
{
  if (level > status->level) {
    status->level = level;
    status->message = message;
  } else if (level == status->level && status->message.empty()) {
    status->message = message;
  }
}

}  // namespace

void applyFirmwareLine(
  DomeDiagnosticsState * state,
  const FirmwareLine & line,
  TimePoint now,
  bool lifecycle_active)
{
  switch (line.kind) {
    case FirmwareLineKind::ROTATE_SPEED:
      if (line.value.has_value()) {
        state->last_accepted_rotate_percent = *line.value;
      }
      break;

    case FirmwareLineKind::ROTARY_DEGREE:
      if (line.value.has_value()) {
        state->last_rotary_target_deg = *line.value;
      }
      if (line.speed.has_value()) {
        state->last_rotary_speed = *line.speed;
      }
      break;

    case FirmwareLineKind::REACHED_TARGET:
      if (line.value.has_value()) {
        state->last_target_reached_deg = *line.value;
      }
      state->last_move_completed = now;
      state->move_succeeded_since_timeout = true;
      break;

    case FirmwareLineKind::REACHED_HOME:
      if (line.value.has_value()) {
        state->last_home_reached_deg = *line.value;
      }
      state->last_move_completed = now;
      state->move_succeeded_since_timeout = true;
      break;

    case FirmwareLineKind::POSITION_TIMEOUT:
      ++state->position_timeouts;
      state->last_position_timeout = now;
      state->move_succeeded_since_timeout = false;
      break;

    case FirmwareLineKind::SENSOR_ERRORS:
      // Normally arrives inside a status section; handle a stray line the same way.
      if (line.value.has_value()) {
        const uint32_t count = static_cast<uint32_t>(*line.value < 0 ? 0 : *line.value);
        state->sensor_errors_increased =
          state->sensor_errors.has_value() && count > *state->sensor_errors;
        state->sensor_errors = count;
      }
      break;

    case FirmwareLineKind::PACKET_SERIAL_ERRORS:
      if (line.value.has_value()) {
        state->packet_serial_errors = static_cast<uint32_t>(*line.value < 0 ? 0 : *line.value);
      }
      break;

    case FirmwareLineKind::POSITION_REPORTING_ENABLED:
      state->position_reporting_enabled = true;
      break;

    case FirmwareLineKind::POSITION_REPORTING_DISABLED:
      state->position_reporting_enabled = false;
      break;

    case FirmwareLineKind::SETUP_GOOD_MAX_SPEED:
      if (line.value.has_value()) {
        state->setup_good_max_speed = *line.value;
      }
      break;

    case FirmwareLineKind::SETUP_FAILED:
      ++state->setup_failures;
      break;

    case FirmwareLineKind::OUT_OF_RANGE:
      ++state->out_of_range_rejections;
      break;

    case FirmwareLineKind::READY:
      state->last_ready = now;
      if (lifecycle_active) {
        state->reboot_detected = true;
      }
      break;

    case FirmwareLineKind::SETUP_SPEED:
    case FirmwareLineKind::SETUP_ANGULAR_VELOCITY:
    case FirmwareLineKind::INVALID:
    case FirmwareLineKind::UNCHANGED:
    case FirmwareLineKind::UNKNOWN:
    default:
      break;
  }
}

void applyStatusSection(
  DomeDiagnosticsState * state,
  const std::vector<std::string> & lines,
  TimePoint now)
{
  std::optional<uint32_t> sensor_errors;
  std::optional<uint32_t> packet_serial_errors;

  for (const std::string & raw : lines) {
    const FirmwareLine line = classifyFirmwareLine(raw);
    if (FirmwareLineKind::SENSOR_ERRORS == line.kind && line.value.has_value()) {
      sensor_errors = static_cast<uint32_t>(*line.value < 0 ? 0 : *line.value);
    } else if (FirmwareLineKind::PACKET_SERIAL_ERRORS == line.kind && line.value.has_value()) {
      packet_serial_errors = static_cast<uint32_t>(*line.value < 0 ? 0 : *line.value);
    }
  }

  if (!sensor_errors.has_value()) {
    // Not a #DPSTATUS section (or a truncated one); leave the counters alone.
    return;
  }

  state->sensor_errors_increased =
    state->sensor_errors.has_value() && *sensor_errors > *state->sensor_errors;
  state->sensor_errors = sensor_errors;
  state->packet_serial_errors = packet_serial_errors.value_or(0);
  state->last_status_response = now;
  ++state->status_polls_answered;
}

double recordPositionReport(DomeDiagnosticsState * state, TimePoint now, double gap_warn_s)
{
  double gap_s = 0.0;
  if (state->last_position_report.has_value()) {
    gap_s = std::chrono::duration<double>(now - *state->last_position_report).count();
    if (gap_warn_s > 0.0 && gap_s > gap_warn_s) {
      ++state->report_gaps;
      state->last_report_gap_s = gap_s;
    }
  }
  ++state->position_reports;
  state->last_position_report = now;
  return gap_s;
}

std::vector<DiagnosticStatus> buildDomeDiagnostics(
  const DomeDiagnosticsState & state,
  const DomeDiagnosticsReadings & readings,
  const DomeDiagnosticsOptions & options,
  TimePoint now)
{
  std::vector<DiagnosticStatus> statuses;

  // ---- link -------------------------------------------------------------
  {
    DiagnosticStatus link = makeStatus("roamadome: link", readings.hardware_id);
    const double report_age = secondsSince(state.last_position_report, now);

    if (!readings.serial_open) {
      raise(&link, DiagnosticStatus::ERROR, "serial port is not open");
    }
    if (state.reboot_detected) {
      raise(&link, DiagnosticStatus::ERROR, "controller rebooted: READY seen while active");
    }
    if (!state.last_position_report.has_value()) {
      raise(&link, DiagnosticStatus::WARN, "no position report received yet");
    } else if (report_age > options.report_stale_s) {
      raise(&link, DiagnosticStatus::WARN,
        "position reports stale for " + formatDouble(report_age, 1) + " s");
    }
    if (link.message.empty()) {
      link.message = "position reports flowing";
    }

    addValue(&link, "serial_open", readings.serial_open ? "true" : "false");
    addValue(&link, "position_reports", std::to_string(state.position_reports));
    addValue(&link, "last_report_age_s", formatAge(state.last_position_report, now));
    addValue(&link, "report_gaps", std::to_string(state.report_gaps));
    addValue(&link, "last_report_gap_s", formatDouble(state.last_report_gap_s, 3));
    addValue(&link, "position_reporting_enabled",
      state.position_reporting_enabled ? "true" : "false");
    addValue(&link, "status_polls_answered", std::to_string(state.status_polls_answered));
    addValue(&link, "last_status_response_age_s", formatAge(state.last_status_response, now));
    addValue(&link, "reboot_detected", state.reboot_detected ? "true" : "false");
    addValue(&link, "last_ready_age_s", formatAge(state.last_ready, now));
    statuses.push_back(link);
  }

  // ---- sensor -----------------------------------------------------------
  {
    DiagnosticStatus sensor = makeStatus("roamadome: sensor", readings.hardware_id);

    if (state.sensor_errors_increased) {
      raise(&sensor, DiagnosticStatus::WARN, "sensor error count increased since last poll");
    }
    if (sensor.message.empty()) {
      sensor.message = state.sensor_errors.has_value() ?
        "sensor error count steady" : "no status poll answered yet";
    }

    addValue(&sensor, "sensor_errors", formatOptional(state.sensor_errors));
    addValue(&sensor, "sensor_errors_increased", state.sensor_errors_increased ? "true" : "false");
    addValue(&sensor, "packet_serial_errors", formatOptional(state.packet_serial_errors));
    addValue(&sensor, "position_deg",
      formatDouble(readings.position_rad * 180.0 / M_PI, 1));
    addValue(&sensor, "velocity_rad_s", formatDouble(readings.velocity_rad_s));
    addValue(&sensor, "last_status_response_age_s", formatAge(state.last_status_response, now));
    statuses.push_back(sensor);
  }

  // ---- motion -----------------------------------------------------------
  {
    DiagnosticStatus motion = makeStatus("roamadome: motion", readings.hardware_id);

    if (state.last_position_timeout.has_value()) {
      const double timeout_age = secondsSince(state.last_position_timeout, now);
      if (timeout_age < options.timeout_error_s) {
        raise(&motion, DiagnosticStatus::ERROR,
          "firmware aborted a move: dome position timeout");
      } else if (!state.move_succeeded_since_timeout) {
        raise(&motion, DiagnosticStatus::WARN,
          "no successful move since the last position timeout");
      }
    }
    if (motion.message.empty()) {
      motion.message = std::string("command mode ") + commandModeName(readings.command_mode);
    }

    addValue(&motion, "command_mode", commandModeName(readings.command_mode));
    addValue(&motion, "commanded_percent", std::to_string(readings.commanded_percent));
    addValue(&motion, "last_accepted_rotate_percent",
      formatOptional(state.last_accepted_rotate_percent));
    addValue(&motion, "last_rotary_target_deg", formatOptional(state.last_rotary_target_deg));
    addValue(&motion, "last_target_reached_deg", formatOptional(state.last_target_reached_deg));
    addValue(&motion, "last_home_reached_deg", formatOptional(state.last_home_reached_deg));
    addValue(&motion, "last_move_completed_age_s", formatAge(state.last_move_completed, now));
    addValue(&motion, "position_timeouts", std::to_string(state.position_timeouts));
    addValue(&motion, "last_timeout_age_s", formatAge(state.last_position_timeout, now));
    addValue(&motion, "out_of_range_rejections", std::to_string(state.out_of_range_rejections));
    addValue(&motion, "setup_failures", std::to_string(state.setup_failures));
    addValue(&motion, "velocity_rad_s", formatDouble(readings.velocity_rad_s));
    addValue(&motion, "settings_writes_pending",
      std::to_string(readings.settings_writes_pending));
    addValue(&motion, "settings_acks", std::to_string(state.settings_acks));
    addValue(&motion, "settings_ack_timeouts", std::to_string(state.settings_ack_timeouts));
    statuses.push_back(motion);
  }

  return statuses;
}

}  // namespace ros2_roamadome
