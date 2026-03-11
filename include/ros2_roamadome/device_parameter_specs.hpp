#ifndef ros2_roamadome__DEVICE_PARAMETER_SPECS_HPP_
#define ros2_roamadome__DEVICE_PARAMETER_SPECS_HPP_

#include "rclcpp/rclcpp.hpp"
#include "ros2_roamadome/roamadome_config_parser.hpp"

#include <cstdint>
#include <exception>
#include <functional>
#include <limits>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

namespace ros2_roamadome
{

class DeviceParameterSpecBase
{
public:
  virtual ~DeviceParameterSpecBase() = default;

  virtual void declareParameter(const rclcpp::Node::SharedPtr & node) const = 0;
  virtual bool isWritable() const = 0;
  virtual rclcpp::Parameter toParameter(const DeviceConfiguration & config) const = 0;
  virtual bool parseAndApplyConfigValue(
    const std::string & value_str,
    DeviceConfiguration * config,
    const rclcpp::Logger & logger) const = 0;

  const std::string & fullName() const;
  const std::string & description() const;
  const rclcpp::ParameterValue & defaultValue() const;
  const std::string & firmwareConfigKey() const;
  bool hasFirmwareConfigKey() const;

protected:
  DeviceParameterSpecBase(
    std::string full_name,
    rclcpp::ParameterValue default_value,
    std::string description,
    std::string firmware_config_key = "");

private:
  std::string full_name_;
  rclcpp::ParameterValue default_value_;
  std::string description_;
  std::string firmware_config_key_;
};

class WritableParameterSpecBase : public DeviceParameterSpecBase
{
public:
  WritableParameterSpecBase(
    std::string field_name,
    std::string full_name,
    rclcpp::ParameterValue default_value,
    std::string description,
    std::string command_prefix,
    std::string firmware_config_key = "");

  bool isWritable() const override;
  const std::string & fieldName() const;

  virtual bool validateAndBuildCommand(
    const rclcpp::Parameter & parameter,
    std::string * command,
    std::string * reason) const = 0;

protected:
  const std::string & commandPrefix() const;

private:
  std::string field_name_;
  std::string command_prefix_;
};

template<typename TValue>
class ReadOnlyParameterSpec : public DeviceParameterSpecBase
{
public:
  static_assert(std::is_integral<TValue>::value || std::is_same<TValue, bool>::value,
    "ReadOnlyParameterSpec only supports integral and bool types");

  using MemberPointer = TValue DeviceConfiguration::*;

  ReadOnlyParameterSpec(
    std::string field_name,
    TValue default_value,
    std::string description,
    MemberPointer member_ptr,
    std::string firmware_config_key,
    TValue min_value = std::numeric_limits<TValue>::lowest(),
    TValue max_value = std::numeric_limits<TValue>::max())
  : DeviceParameterSpecBase(
      buildFullName(field_name),
      toParameterValue(default_value),
      std::move(description),
      std::move(firmware_config_key)),
    member_ptr_(member_ptr),
    min_value_(min_value),
    max_value_(max_value)
  {
  }

  void declareParameter(const rclcpp::Node::SharedPtr & node) const override;
  bool isWritable() const override;
  rclcpp::Parameter toParameter(const DeviceConfiguration & config) const override;
  bool parseAndApplyConfigValue(
    const std::string & value_str,
    DeviceConfiguration * config,
    const rclcpp::Logger & logger) const override;

private:
  static std::string buildFullName(const std::string & field_name)
  {
    return "device." + field_name;
  }

  static rclcpp::ParameterValue toParameterValue(TValue value)
  {
    if constexpr (std::is_same<TValue, bool>::value) {
      return rclcpp::ParameterValue(value);
    }
    return rclcpp::ParameterValue(static_cast<int64_t>(value));
  }

  MemberPointer member_ptr_;
  TValue min_value_;
  TValue max_value_;
};

template<typename TValue>
class WritableParameterSpec : public WritableParameterSpecBase
{
public:
  static_assert(std::is_integral<TValue>::value || std::is_same<TValue, bool>::value,
    "WritableParameterSpec only supports integral and bool types");

  using MemberPointer = TValue DeviceConfiguration::*;
  using ParameterValidator = std::function<bool(const rclcpp::Parameter &, std::string *)>;

  WritableParameterSpec(
    std::string field_name,
    TValue default_value,
    std::string description,
    std::string command_prefix,
    MemberPointer member_ptr,
    std::string firmware_config_key,
    TValue min_value = std::numeric_limits<TValue>::lowest(),
    TValue max_value = std::numeric_limits<TValue>::max(),
    ParameterValidator validator = nullptr)
  : WritableParameterSpecBase(
      field_name,
      buildFullName(field_name),
      toParameterValue(default_value),
      std::move(description),
      std::move(command_prefix),
      std::move(firmware_config_key)),
    member_ptr_(member_ptr),
    min_value_(min_value),
    max_value_(max_value),
    validator_(std::move(validator))
  {
  }

  void declareParameter(const rclcpp::Node::SharedPtr & node) const override;
  rclcpp::Parameter toParameter(const DeviceConfiguration & config) const override;
  bool parseAndApplyConfigValue(
    const std::string & value_str,
    DeviceConfiguration * config,
    const rclcpp::Logger & logger) const override;
  bool validateAndBuildCommand(
    const rclcpp::Parameter & parameter,
    std::string * command,
    std::string * reason) const override;

private:
  static std::string buildFullName(const std::string & field_name)
  {
    return "device." + field_name;
  }

  static rclcpp::ParameterValue toParameterValue(TValue value)
  {
    if constexpr (std::is_same<TValue, bool>::value) {
      return rclcpp::ParameterValue(value);
    }
    return rclcpp::ParameterValue(static_cast<int64_t>(value));
  }

  bool parseParameterValue(
    const rclcpp::Parameter & parameter,
    TValue * output,
    std::string * reason) const
  {
    try {
      if constexpr (std::is_same<TValue, bool>::value) {
        *output = parameter.as_bool();
      } else {
        (void) parameter.as_int();
        *output = TValue{};
      }
    } catch (const rclcpp::ParameterTypeException & e) {
      *reason = std::string("invalid type for ") + parameter.get_name() + ": " + e.what();
      return false;
    }
    return true;
  }

  MemberPointer member_ptr_;
  TValue min_value_;
  TValue max_value_;
  ParameterValidator validator_;
};

template<typename TValue>
typename WritableParameterSpec<TValue>::ParameterValidator getBoolTypeLambda()
{
  return [](const rclcpp::Parameter & parameter, std::string * reason) {
           try {
             (void) parameter.as_bool();
           } catch (const rclcpp::ParameterTypeException & e) {
             *reason = std::string("invalid type for ") + parameter.get_name() + ": " +
               e.what();
             return false;
           }
           return true;
         };
}

const std::vector<const DeviceParameterSpecBase *> & getAllParameterSpecs();
const WritableParameterSpecBase * findWritableParameterSpec(const std::string & field_name);
const DeviceParameterSpecBase * findParameterSpecByConfigKey(const std::string & config_key);

template<typename TValue>
bool parseConfigScalar(
  const std::string & value_str,
  TValue min_value,
  TValue max_value,
  TValue * output,
  const rclcpp::Logger & logger,
  const std::string & field_name)
{
  if constexpr (std::is_same<TValue, bool>::value) {
    if (value_str == "0" || value_str == "false" || value_str == "False") {
      *output = false;
      return true;
    }
    if (value_str == "1" || value_str == "true" || value_str == "True") {
      *output = true;
      return true;
    }
    RCLCPP_WARN(logger, "Invalid boolean value for %s: '%s'", field_name.c_str(),
        value_str.c_str());
    return false;
  } else {
    try {
      size_t parse_end = 0;
      const unsigned long long parsed = std::stoull(value_str, &parse_end);
      if (parse_end != value_str.size()) {
        RCLCPP_WARN(
          logger, "Invalid numeric value for %s: '%s'", field_name.c_str(), value_str.c_str());
        return false;
      }

      if (parsed < static_cast<unsigned long long>(min_value) ||
        parsed > static_cast<unsigned long long>(max_value))
      {
        RCLCPP_WARN(
          logger, "Value %s out of range [%lld, %lld] for %s", value_str.c_str(),
          static_cast<long long>(min_value), static_cast<long long>(max_value), field_name.c_str());
        return false;
      }

      *output = static_cast<TValue>(parsed);
      return true;
    } catch (const std::exception & ex) {
      RCLCPP_WARN(
        logger, "Failed to parse numeric value for %s ('%s'): %s", field_name.c_str(),
        value_str.c_str(), ex.what());
      return false;
    }
  }
}

template<typename TValue>
void ReadOnlyParameterSpec<TValue>::declareParameter(const rclcpp::Node::SharedPtr & node) const
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  descriptor.description = description();
  node->declare_parameter(fullName(), defaultValue(), descriptor);
}

template<typename TValue>
bool ReadOnlyParameterSpec<TValue>::isWritable() const
{
  return false;
}

template<typename TValue>
rclcpp::Parameter ReadOnlyParameterSpec<TValue>::toParameter(
  const DeviceConfiguration & config) const
{
  const TValue value = config.*member_ptr_;
  return rclcpp::Parameter(fullName(), toParameterValue(value));
}

template<typename TValue>
bool ReadOnlyParameterSpec<TValue>::parseAndApplyConfigValue(
  const std::string & value_str,
  DeviceConfiguration * config,
  const rclcpp::Logger & logger) const
{
  TValue parsed_value{};
  if (!parseConfigScalar(
      value_str, min_value_, max_value_, &parsed_value, logger,
      firmwareConfigKey()))
  {
    return false;
  }

  config->*member_ptr_ = parsed_value;
  return true;
}

template<typename TValue>
void WritableParameterSpec<TValue>::declareParameter(const rclcpp::Node::SharedPtr & node) const
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = false;
  descriptor.description = description();
  node->declare_parameter(fullName(), defaultValue(), descriptor);
}

template<typename TValue>
rclcpp::Parameter WritableParameterSpec<TValue>::toParameter(
  const DeviceConfiguration & config) const
{
  const TValue value = config.*member_ptr_;
  return rclcpp::Parameter(fullName(), toParameterValue(value));
}

template<typename TValue>
bool WritableParameterSpec<TValue>::parseAndApplyConfigValue(
  const std::string & value_str,
  DeviceConfiguration * config,
  const rclcpp::Logger & logger) const
{
  TValue parsed_value{};
  if (!parseConfigScalar(
      value_str, min_value_, max_value_, &parsed_value, logger,
      firmwareConfigKey()))
  {
    return false;
  }

  config->*member_ptr_ = parsed_value;
  return true;
}

template<typename TValue>
bool WritableParameterSpec<TValue>::validateAndBuildCommand(
  const rclcpp::Parameter & parameter,
  std::string * command,
  std::string * reason) const
{
  TValue parsed_value{};
  if (!parseParameterValue(parameter, &parsed_value, reason)) {
    return false;
  }

  if (validator_) {
    if (!validator_(parameter, reason)) {
      return false;
    }
  } else if constexpr (!std::is_same<TValue, bool>::value) {
    const int64_t value = parameter.as_int();
    const int64_t min_i64 = static_cast<int64_t>(min_value_);
    const int64_t max_i64 = static_cast<int64_t>(max_value_);

    if (value < min_i64 || value > max_i64) {
      std::ostringstream stream;
      stream << parameter.get_name() << " must be in range [" << min_i64 << ", "
             << max_i64 << "]";
      *reason = stream.str();
      return false;
    }
  }

  std::ostringstream stream;
  stream << commandPrefix();
  if constexpr (std::is_same<TValue, bool>::value) {
    stream << (parsed_value ? 1 : 0);
  } else {
    stream << parameter.as_int();
  }
  *command = stream.str();
  return true;
}

}  // namespace ros2_roamadome

#endif  // ros2_roamadome__DEVICE_PARAMETER_SPECS_HPP_
