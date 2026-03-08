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

#ifndef ROS2_ROAMADOME__PARAMETER_PARSER_HPP_
#define ROS2_ROAMADOME__PARAMETER_PARSER_HPP_

#include <cstdint>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/logger.hpp"
#include "ros2_roamadome/visibility_control.h"

namespace ros2_roamadome
{

/// @brief Converts a string to lowercase
/// @param value The string to convert
/// @return Lowercase version of the input string
ros2_roamadome_PUBLIC
std::string toLower(const std::string & value);

/// @brief Parses a required string parameter from a parameter map
/// @param parameters Map of parameter names to values
/// @param logger ROS logger for error messages
/// @param parameter_name Name of the parameter to parse
/// @param value Output pointer to store the parsed value
/// @return true if parameter exists and is non-empty, false otherwise
ros2_roamadome_PUBLIC
bool parseRequiredStringParameter(
  const std::unordered_map<std::string, std::string> & parameters,
  rclcpp::Logger logger,
  const std::string & parameter_name,
  std::string * value);

/// @brief Parses an optional boolean parameter from a parameter map
/// @param parameters Map of parameter names to values
/// @param logger ROS logger for error messages
/// @param parameter_name Name of the parameter to parse
/// @param default_value Default value to use if parameter is not found
/// @param value Output pointer to store the parsed value
/// @return true if parameter is valid or missing (uses default), false if invalid format
ros2_roamadome_PUBLIC
bool parseOptionalBoolParameter(
  const std::unordered_map<std::string, std::string> & parameters,
  rclcpp::Logger logger,
  const std::string & parameter_name,
  bool default_value,
  bool * value);

/// @brief Parses an optional enum parameter from a parameter map
/// @param parameters Map of parameter names to values
/// @param logger ROS logger for error messages
/// @param parameter_name Name of the parameter to parse
/// @param default_value Default value to use if parameter is not found (must be lowercase)
/// @param allowed_values Vector of allowed values (must be lowercase; comparison is against a lowercased parameter value)
/// @param value Output pointer to store the parsed value
/// @return true if parameter is valid or missing (uses default), false if not in allowed_values
ros2_roamadome_PUBLIC
bool parseOptionalEnumParameter(
  const std::unordered_map<std::string, std::string> & parameters,
  rclcpp::Logger logger,
  const std::string & parameter_name,
  const std::string & default_value,
  const std::vector<std::string> & allowed_values,
  std::string * value);

/// @brief Parses an optional uint32_t parameter from a parameter map with optional validation
/// @param parameters Map of parameter names to values
/// @param logger ROS logger for error messages
/// @param parameter_name Name of the parameter to parse
/// @param default_value Default value to use if parameter is not found
/// @param validator Optional validation function to check parsed value
/// @param validation_description Description of validation rules for error messages
/// @param value Output pointer to store the parsed value
/// @return true if parameter is valid or missing (uses default), false if invalid or fails validation
ros2_roamadome_PUBLIC
bool parseOptionalUInt32Parameter(
  const std::unordered_map<std::string, std::string> & parameters,
  rclcpp::Logger logger,
  const std::string & parameter_name,
  uint32_t default_value,
  const std::function<bool(uint32_t)> & validator,
  const std::string & validation_description,
  uint32_t * value);

/// @brief Parses an optional double parameter from a parameter map with optional validation
/// @param parameters Map of parameter names to values
/// @param logger ROS logger for error messages
/// @param parameter_name Name of the parameter to parse
/// @param default_value Default value to use if parameter is not found
/// @param validator Optional validation function to check parsed value
/// @param validation_description Description of validation rules for error messages
/// @param value Output pointer to store the parsed value
/// @return true if parameter is valid or missing (uses default), false if invalid or fails validation
ros2_roamadome_PUBLIC
bool parseOptionalDoubleParameter(
  const std::unordered_map<std::string, std::string> & parameters,
  rclcpp::Logger logger,
  const std::string & parameter_name,
  double default_value,
  const std::function<bool(double)> & validator,
  const std::string & validation_description,
  double * value);

}  // namespace ros2_roamadome

#endif  // ROS2_ROAMADOME__PARAMETER_PARSER_HPP_
