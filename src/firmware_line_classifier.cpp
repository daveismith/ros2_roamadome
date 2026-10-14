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

#include "ros2_roamadome/firmware_line_classifier.hpp"

#include <cstdlib>
#include <cstring>
#include <string>

namespace ros2_roamadome
{

namespace
{

std::string trimWhitespace(const std::string & text)
{
  const char * whitespace = " \t\r\n";
  const size_t start = text.find_first_not_of(whitespace);
  if (std::string::npos == start) {
    return "";
  }
  const size_t end = text.find_last_not_of(whitespace);
  return text.substr(start, end - start + 1);
}

bool takePrefix(const std::string & line, const char * prefix, std::string * rest)
{
  const size_t length = std::strlen(prefix);
  if (0 != line.compare(0, length, prefix)) {
    return false;
  }
  *rest = trimWhitespace(line.substr(length));
  return true;
}

std::optional<int32_t> parseLeadingInt(const std::string & text)
{
  const char * begin = text.c_str();
  char * end = nullptr;
  const long value = std::strtol(begin, &end, 10);
  if (end == begin) {
    return std::nullopt;
  }
  return static_cast<int32_t>(value);
}

std::optional<double> parseLeadingDouble(const std::string & text)
{
  const char * begin = text.c_str();
  char * end = nullptr;
  const double value = std::strtod(begin, &end);
  if (end == begin) {
    return std::nullopt;
  }
  return value;
}

}  // namespace

FirmwareLine classifyFirmwareLine(const std::string & raw_line)
{
  const std::string line = trimWhitespace(raw_line);
  FirmwareLine result;
  std::string rest;

  if (line.empty()) {
    return result;
  }

  if (takePrefix(line, "ROTATE SPEED:", &rest)) {
    result.kind = FirmwareLineKind::ROTATE_SPEED;
    result.value = parseLeadingInt(rest);
    return result;
  }

  if (takePrefix(line, "ROTARY DEGREE:", &rest)) {
    result.kind = FirmwareLineKind::ROTARY_DEGREE;
    result.value = parseLeadingInt(rest);
    const size_t speed_pos = rest.find("speed:");
    if (std::string::npos != speed_pos) {
      result.speed = parseLeadingDouble(trimWhitespace(rest.substr(speed_pos + 6)));
    }
    return result;
  }

  if (takePrefix(line, "REACHED TARGET:", &rest)) {
    result.kind = FirmwareLineKind::REACHED_TARGET;
    result.value = parseLeadingInt(rest);
    return result;
  }

  if (takePrefix(line, "REACHED HOME:", &rest)) {
    result.kind = FirmwareLineKind::REACHED_HOME;
    result.value = parseLeadingInt(rest);
    return result;
  }

  if ("ERROR: DOME POSITION TIMEOUT" == line) {
    result.kind = FirmwareLineKind::POSITION_TIMEOUT;
    return result;
  }

  if (takePrefix(line, "Dome Sensor Errors:", &rest)) {
    result.kind = FirmwareLineKind::SENSOR_ERRORS;
    result.value = parseLeadingInt(rest);
    return result;
  }

  if ("No Dome Sensor Errors" == line) {
    result.kind = FirmwareLineKind::SENSOR_ERRORS;
    result.value = 0;
    return result;
  }

  if (takePrefix(line, "Packet Serial Errors:", &rest)) {
    result.kind = FirmwareLineKind::PACKET_SERIAL_ERRORS;
    result.value = parseLeadingInt(rest);
    return result;
  }

  if ("Position Reporting Enabled" == line) {
    result.kind = FirmwareLineKind::POSITION_REPORTING_ENABLED;
    return result;
  }

  if ("Position Reporting Disabled" == line) {
    result.kind = FirmwareLineKind::POSITION_REPORTING_DISABLED;
    return result;
  }

  if (takePrefix(line, "GOOD MAX SPEED:", &rest)) {
    result.kind = FirmwareLineKind::SETUP_GOOD_MAX_SPEED;
    result.value = parseLeadingInt(rest);
    return result;
  }

  if (takePrefix(line, "SPEED:", &rest)) {
    result.kind = FirmwareLineKind::SETUP_SPEED;
    result.value = parseLeadingInt(rest);
    return result;
  }

  if (takePrefix(line, "Angular velocity:", &rest)) {
    result.kind = FirmwareLineKind::SETUP_ANGULAR_VELOCITY;
    result.speed = parseLeadingDouble(rest);
    return result;
  }

  if ("AUTO SETUP FAILED" == line) {
    result.kind = FirmwareLineKind::SETUP_FAILED;
    return result;
  }

  if ("Out of range." == line) {
    result.kind = FirmwareLineKind::OUT_OF_RANGE;
    return result;
  }

  if ("Invalid" == line) {
    result.kind = FirmwareLineKind::INVALID;
    return result;
  }

  if ("Unchanged" == line) {
    result.kind = FirmwareLineKind::UNCHANGED;
    return result;
  }

  if ("READY" == line) {
    result.kind = FirmwareLineKind::READY;
    return result;
  }

  return result;
}

const char * firmwareLineKindName(FirmwareLineKind kind)
{
  switch (kind) {
    case FirmwareLineKind::ROTATE_SPEED: return "rotate_speed";
    case FirmwareLineKind::ROTARY_DEGREE: return "rotary_degree";
    case FirmwareLineKind::REACHED_TARGET: return "reached_target";
    case FirmwareLineKind::REACHED_HOME: return "reached_home";
    case FirmwareLineKind::POSITION_TIMEOUT: return "position_timeout";
    case FirmwareLineKind::SENSOR_ERRORS: return "sensor_errors";
    case FirmwareLineKind::PACKET_SERIAL_ERRORS: return "packet_serial_errors";
    case FirmwareLineKind::POSITION_REPORTING_ENABLED: return "position_reporting_enabled";
    case FirmwareLineKind::POSITION_REPORTING_DISABLED: return "position_reporting_disabled";
    case FirmwareLineKind::SETUP_SPEED: return "setup_speed";
    case FirmwareLineKind::SETUP_ANGULAR_VELOCITY: return "setup_angular_velocity";
    case FirmwareLineKind::SETUP_GOOD_MAX_SPEED: return "setup_good_max_speed";
    case FirmwareLineKind::SETUP_FAILED: return "setup_failed";
    case FirmwareLineKind::OUT_OF_RANGE: return "out_of_range";
    case FirmwareLineKind::INVALID: return "invalid";
    case FirmwareLineKind::UNCHANGED: return "unchanged";
    case FirmwareLineKind::READY: return "ready";
    case FirmwareLineKind::UNKNOWN:
    default:
      return "unknown";
  }
}

}  // namespace ros2_roamadome
