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

#include "ros2_roamadome/parameter_parser.hpp"

#include <algorithm>
#include <cctype>
#include <exception>
#include <limits>
#include <sstream>
#include <string>

#include "rclcpp/logging.hpp"

namespace ros2_roamadome
{

std::string toLower(const std::string & value)
{
  std::string lowered = value;
  std::transform(lowered.begin(), lowered.end(), lowered.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return lowered;
}

bool parseRequiredStringParameter(
  const std::unordered_map<std::string, std::string> & parameters,
  rclcpp::Logger logger,
  const std::string & parameter_name,
  std::string * value)
{
  const auto parameter_it = parameters.find(parameter_name);
  if (parameter_it == parameters.end() || parameter_it->second.empty()) {
    RCLCPP_ERROR(logger, "'%s' parameter is required and must be non-empty",
          parameter_name.c_str());
    return false;
  }

  *value = parameter_it->second;
  return true;
}

bool parseOptionalBoolParameter(
  const std::unordered_map<std::string, std::string> & parameters,
  rclcpp::Logger logger,
  const std::string & parameter_name,
  bool default_value,
  bool * value)
{
  const auto parameter_it = parameters.find(parameter_name);
  if (parameter_it == parameters.end()) {
    *value = default_value;
    return true;
  }

  const std::string lowered_value = toLower(parameter_it->second);
  if ("true" == lowered_value || "1" == lowered_value || "yes" == lowered_value) {
    *value = true;
    return true;
  }

  if ("false" == lowered_value || "0" == lowered_value || "no" == lowered_value) {
    *value = false;
    return true;
  }

  RCLCPP_ERROR(
    logger,
    "Invalid '%s' value '%s'. Supported values: true/false/1/0/yes/no",
    parameter_name.c_str(),
    parameter_it->second.c_str());
  return false;
}

bool parseOptionalEnumParameter(
  const std::unordered_map<std::string, std::string> & parameters,
  rclcpp::Logger logger,
  const std::string & parameter_name,
  const std::string & default_value,
  const std::vector<std::string> & allowed_values,
  std::string * value)
{
  const auto parameter_it = parameters.find(parameter_name);
  if (parameter_it == parameters.end()) {
    *value = default_value;
    return true;
  }

  const std::string lowered_value = toLower(parameter_it->second);
  for (const auto & allowed : allowed_values) {
    if (allowed == lowered_value) {
      *value = lowered_value;
      return true;
    }
  }

  std::ostringstream allowed_stream;
  for (size_t idx = 0; idx < allowed_values.size(); ++idx) {
    allowed_stream << allowed_values[idx];
    if (idx + 1 < allowed_values.size()) {
      allowed_stream << ", ";
    }
  }

  RCLCPP_ERROR(
    logger,
    "Invalid '%s' value '%s'. Supported values: %s",
    parameter_name.c_str(),
    parameter_it->second.c_str(),
    allowed_stream.str().c_str());
  return false;
}

bool parseOptionalUInt32Parameter(
  const std::unordered_map<std::string, std::string> & parameters,
  rclcpp::Logger logger,
  const std::string & parameter_name,
  uint32_t default_value,
  const std::function<bool(uint32_t)> & validator,
  const std::string & validation_description,
  uint32_t * value)
{
  const auto parameter_it = parameters.find(parameter_name);
  if (parameter_it == parameters.end()) {
    *value = default_value;
    return true;
  }

  const std::string & raw_value = parameter_it->second;
  if (!raw_value.empty() && '-' == raw_value.front()) {
    RCLCPP_ERROR(logger, "'%s' must be an unsigned integer, got '%s'", parameter_name.c_str(),
      raw_value.c_str());
    return false;
  }

  try {
    size_t parsed_length = 0;
    const unsigned long parsed = std::stoul(raw_value, &parsed_length, 10);
    if (parsed_length != raw_value.size()) {
      RCLCPP_ERROR(logger, "'%s' has invalid numeric format '%s'", parameter_name.c_str(),
        raw_value.c_str());
      return false;
    }
    if (parsed > std::numeric_limits<uint32_t>::max()) {
      RCLCPP_ERROR(logger, "'%s' is out of range for uint32: '%s'", parameter_name.c_str(),
        raw_value.c_str());
      return false;
    }

    const uint32_t parsed_value = static_cast<uint32_t>(parsed);
    if (validator && !validator(parsed_value)) {
      RCLCPP_ERROR(logger, "'%s' must satisfy '%s', got %u", parameter_name.c_str(),
        validation_description.c_str(), parsed_value);
      return false;
    }

    *value = parsed_value;
    return true;
  } catch (const std::exception &) {
    RCLCPP_ERROR(logger, "'%s' has invalid numeric format '%s'", parameter_name.c_str(),
      raw_value.c_str());
    return false;
  }
}

bool parseOptionalDoubleParameter(
  const std::unordered_map<std::string, std::string> & parameters,
  rclcpp::Logger logger,
  const std::string & parameter_name,
  double default_value,
  const std::function<bool(double)> & validator,
  const std::string & validation_description,
  double * value)
{
  const auto parameter_it = parameters.find(parameter_name);
  if (parameter_it == parameters.end()) {
    *value = default_value;
    return true;
  }

  const std::string & raw_value = parameter_it->second;
  try {
    size_t parsed_length = 0;
    const double parsed_value = std::stod(raw_value, &parsed_length);
    if (parsed_length != raw_value.size()) {
      RCLCPP_ERROR(logger, "'%s' has invalid numeric format '%s'", parameter_name.c_str(),
        raw_value.c_str());
      return false;
    }

    if (validator && !validator(parsed_value)) {
      RCLCPP_ERROR(logger, "'%s' must satisfy '%s', got %.6f", parameter_name.c_str(),
        validation_description.c_str(), parsed_value);
      return false;
    }

    *value = parsed_value;
    return true;
  } catch (const std::exception &) {
    RCLCPP_ERROR(logger, "'%s' has invalid numeric format '%s'", parameter_name.c_str(),
      raw_value.c_str());
    return false;
  }
}

}  // namespace ros2_roamadome
