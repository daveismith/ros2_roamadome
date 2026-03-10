#include "ros2_roamadome/roamadome_config_parser.hpp"
#include <sstream>
#include <algorithm>
#include <limits>
#include <map>

namespace ros2_roamadome
{

bool ConfigurationParser::parseUInt8(
  const std::string & value_str,
  uint8_t & output,
  const rclcpp::Logger & logger,
  uint8_t min_val,
  uint8_t max_val)
{
  try {
    // Parse as unsigned long then validate range
    unsigned long parsed = std::stoul(value_str);
    if (parsed < min_val || parsed > max_val) {
      RCLCPP_WARN(logger, "Value %lu out of range [%u, %u]", parsed, min_val, max_val);
      return false;
    }
    output = static_cast<uint8_t>(parsed);
    return true;
  } catch (const std::exception & ex) {
    RCLCPP_WARN(logger, "Failed to parse uint8 value '%s': %s", value_str.c_str(), ex.what());
    return false;
  }
}

bool ConfigurationParser::parseUInt16(
  const std::string & value_str,
  uint16_t & output,
  const rclcpp::Logger & logger,
  uint16_t min_val,
  uint16_t max_val)
{
  try {
    unsigned long parsed = std::stoul(value_str);
    if (parsed < min_val || parsed > max_val) {
      RCLCPP_WARN(logger, "Value %lu out of range [%u, %u]", parsed, min_val, max_val);
      return false;
    }
    output = static_cast<uint16_t>(parsed);
    return true;
  } catch (const std::exception & ex) {
    RCLCPP_WARN(logger, "Failed to parse uint16 value '%s': %s", value_str.c_str(), ex.what());
    return false;
  }
}

bool ConfigurationParser::parseUInt32(
  const std::string & value_str,
  uint32_t & output,
  const rclcpp::Logger & logger)
{
  try {
    unsigned long parsed = std::stoul(value_str);
    if (parsed > std::numeric_limits<uint32_t>::max()) {
      RCLCPP_WARN(logger, "Value %lu exceeds uint32_t max", parsed);
      return false;
    }
    output = static_cast<uint32_t>(parsed);
    return true;
  } catch (const std::exception & ex) {
    RCLCPP_WARN(logger, "Failed to parse uint32 value '%s': %s", value_str.c_str(), ex.what());
    return false;
  }
}

bool ConfigurationParser::parseBool(
  const std::string & value_str,
  bool & output,
  const rclcpp::Logger & logger)
{
  if (value_str == "0" || value_str == "false" || value_str == "False") {
    output = false;
    return true;
  } else if (value_str == "1" || value_str == "true" || value_str == "True") {
    output = true;
    return true;
  } else {
    RCLCPP_WARN(logger, "Invalid boolean value '%s'", value_str.c_str());
    return false;
  }
}

std::optional<DeviceConfiguration> ConfigurationParser::parse(
  const std::map<std::string, std::string> & config_map,
  const rclcpp::Logger & logger)
{
  DeviceConfiguration config;
  bool parse_errors = false;

  // Field type descriptor
  enum class FieldType
  {
    UINT8_UNBOUNDED,
    UINT8_RANGE,
    UINT16_UNBOUNDED,
    UINT16_RANGE,
    UINT32,
    BOOL
  };

  struct FieldSpec
  {
    FieldType type;
    uint8_t min_u8 = 0, max_u8 = 255;
    uint16_t min_u16 = 0, max_u16 = 65535;
    // Setter function: takes value string, config, logger, returns success
    std::function<bool(const std::string &, DeviceConfiguration &, const rclcpp::Logger &)>
    setter;
  };

  // Data-driven field map with type descriptors and setters
  static const std::map<std::string, FieldSpec> field_specs = {
    {"HomePos",
      {FieldType::UINT16_RANGE, 0, 0, 0, 359,
        [](const std::string & v, DeviceConfiguration & c, const rclcpp::Logger & l) {
          return parseUInt16(v, c.home_pos, l, 0, 359);
        }}},
    {"MaxSpeed",
      {FieldType::UINT8_RANGE, 0, 100, 0, 0,
        [](const std::string & v, DeviceConfiguration & c, const rclcpp::Logger & l) {
          return parseUInt8(v, c.max_speed, l, 0, 100);
        }}},
    {"MinSpeed",
      {FieldType::UINT8_RANGE, 0, 100, 0, 0,
        [](const std::string & v, DeviceConfiguration & c, const rclcpp::Logger & l) {
          return parseUInt8(v, c.min_speed, l, 0, 100);
        }}},
    {"AutoMode",
      {FieldType::BOOL, 0, 0, 0, 0,
        [](const std::string & v, DeviceConfiguration & c, const rclcpp::Logger & l) {
          return parseBool(v, c.auto_mode, l);
        }}},
    {"HomeMode",
      {FieldType::BOOL, 0, 0, 0, 0,
        [](const std::string & v, DeviceConfiguration & c, const rclcpp::Logger & l) {
          return parseBool(v, c.home_mode, l);
        }}},
    {"InputSpeed",
      {FieldType::UINT8_RANGE, 0, 100, 0, 0,
        [](const std::string & v, DeviceConfiguration & c, const rclcpp::Logger & l) {
          return parseUInt8(v, c.input_speed, l, 0, 100);
        }}},
    {"Scaling",
      {FieldType::BOOL, 0, 0, 0, 0,
        [](const std::string & v, DeviceConfiguration & c, const rclcpp::Logger & l) {
          return parseBool(v, c.scaling, l);
        }}},
    {"Inverted",
      {FieldType::BOOL, 0, 0, 0, 0,
        [](const std::string & v, DeviceConfiguration & c, const rclcpp::Logger & l) {
          return parseBool(v, c.inverted, l);
        }}},
    {"Timeout",
      {FieldType::UINT8_RANGE, 0, 30, 0, 0,
        [](const std::string & v, DeviceConfiguration & c, const rclcpp::Logger & l) {
          return parseUInt8(v, c.timeout, l, 0, 30);
        }}},
    {"AutoSafety",
      {FieldType::BOOL, 0, 0, 0, 0,
        [](const std::string & v, DeviceConfiguration & c, const rclcpp::Logger & l) {
          return parseBool(v, c.auto_safety, l);
        }}},
    {"AutoRestart",
      {FieldType::BOOL, 0, 0, 0, 0,
        [](const std::string & v, DeviceConfiguration & c, const rclcpp::Logger & l) {
          return parseBool(v, c.auto_restart, l);
        }}},
    {"AccelerationScale",
      {FieldType::UINT8_UNBOUNDED, 0, 0, 0, 0,
        [](const std::string & v, DeviceConfiguration & c, const rclcpp::Logger & l) {
          return parseUInt8(v, c.acceleration_scale, l);
        }}},
    {"DecelerationScale",
      {FieldType::UINT8_UNBOUNDED, 0, 0, 0, 0,
        [](const std::string & v, DeviceConfiguration & c, const rclcpp::Logger & l) {
          return parseUInt8(v, c.deceleration_scale, l);
        }}},
    {"HomeMinDelay",
      {FieldType::UINT8_UNBOUNDED, 0, 0, 0, 0,
        [](const std::string & v, DeviceConfiguration & c, const rclcpp::Logger & l) {
          return parseUInt8(v, c.home_min_delay, l);
        }}},
    {"HomeMaxDelay",
      {FieldType::UINT8_UNBOUNDED, 0, 0, 0, 0,
        [](const std::string & v, DeviceConfiguration & c, const rclcpp::Logger & l) {
          return parseUInt8(v, c.home_max_delay, l);
        }}},
    {"AutoMinDelay",
      {FieldType::UINT8_UNBOUNDED, 0, 0, 0, 0,
        [](const std::string & v, DeviceConfiguration & c, const rclcpp::Logger & l) {
          return parseUInt8(v, c.auto_min_delay, l);
        }}},
    {"AutoMaxDelay",
      {FieldType::UINT8_UNBOUNDED, 0, 0, 0, 0,
        [](const std::string & v, DeviceConfiguration & c, const rclcpp::Logger & l) {
          return parseUInt8(v, c.auto_max_delay, l);
        }}},
    {"TargetMinDelay",
      {FieldType::UINT8_UNBOUNDED, 0, 0, 0, 0,
        [](const std::string & v, DeviceConfiguration & c, const rclcpp::Logger & l) {
          return parseUInt8(v, c.target_min_delay, l);
        }}},
    {"TargetMaxDelay",
      {FieldType::UINT8_UNBOUNDED, 0, 0, 0, 0,
        [](const std::string & v, DeviceConfiguration & c, const rclcpp::Logger & l) {
          return parseUInt8(v, c.target_max_delay, l);
        }}},
    {"SetupAngularVelocity",
      {FieldType::UINT16_UNBOUNDED, 0, 0, 0, 0,
        [](const std::string & v, DeviceConfiguration & c, const rclcpp::Logger & l) {
          return parseUInt16(v, c.setup_angular_velocity, l);
        }}},
    {"AutoLeft",
      {FieldType::UINT8_RANGE, 0, 180, 0, 0,
        [](const std::string & v, DeviceConfiguration & c, const rclcpp::Logger & l) {
          return parseUInt8(v, c.auto_left, l, 0, 180);
        }}},
    {"AutoRight",
      {FieldType::UINT8_RANGE, 0, 180, 0, 0,
        [](const std::string & v, DeviceConfiguration & c, const rclcpp::Logger & l) {
          return parseUInt8(v, c.auto_right, l, 0, 180);
        }}},
    {"Fudge",
      {FieldType::UINT8_RANGE, 0, 20, 0, 0,
        [](const std::string & v, DeviceConfiguration & c, const rclcpp::Logger & l) {
          return parseUInt8(v, c.fudge, l, 0, 20);
        }}},
    {"SpeedHome",
      {FieldType::UINT8_RANGE, 0, 100, 0, 0,
        [](const std::string & v, DeviceConfiguration & c, const rclcpp::Logger & l) {
          return parseUInt8(v, c.speed_home, l, 0, 100);
        }}},
    {"SpeedAuto",
      {FieldType::UINT8_RANGE, 0, 100, 0, 0,
        [](const std::string & v, DeviceConfiguration & c, const rclcpp::Logger & l) {
          return parseUInt8(v, c.speed_auto, l, 0, 100);
        }}},
    {"SpeedTarget",
      {FieldType::UINT8_RANGE, 0, 100, 0, 0,
        [](const std::string & v, DeviceConfiguration & c, const rclcpp::Logger & l) {
          return parseUInt8(v, c.speed_target, l, 0, 100);
        }}},
    {"SyrenAddressIn",
      {FieldType::UINT8_UNBOUNDED, 0, 0, 0, 0,
        [](const std::string & v, DeviceConfiguration & c, const rclcpp::Logger & l) {
          return parseUInt8(v, c.syren_address_in, l);
        }}},
    {"SyrenAddressOut",
      {FieldType::UINT8_UNBOUNDED, 0, 0, 0, 0,
        [](const std::string & v, DeviceConfiguration & c, const rclcpp::Logger & l) {
          return parseUInt8(v, c.syren_address_out, l);
        }}},
    {"SensorBaud",
      {FieldType::UINT32, 0, 0, 0, 0,
        [](const std::string & v, DeviceConfiguration & c, const rclcpp::Logger & l) {
          return parseUInt32(v, c.sensor_baud, l);
        }}},
    {"SyrenBaud",
      {FieldType::UINT32, 0, 0, 0, 0,
        [](const std::string & v, DeviceConfiguration & c, const rclcpp::Logger & l) {
          return parseUInt32(v, c.syren_baud, l);
        }}},
    {"SerialBaud",
      {FieldType::UINT32, 0, 0, 0, 0,
        [](const std::string & v, DeviceConfiguration & c, const rclcpp::Logger & l) {
          return parseUInt32(v, c.serial_baud, l);
        }}},
    {"SerialIn",
      {FieldType::BOOL, 0, 0, 0, 0,
        [](const std::string & v, DeviceConfiguration & c, const rclcpp::Logger & l) {
          return parseBool(v, c.serial_in, l);
        }}},
    {"SerialOut",
      {FieldType::BOOL, 0, 0, 0, 0,
        [](const std::string & v, DeviceConfiguration & c, const rclcpp::Logger & l) {
          return parseBool(v, c.serial_out, l);
        }}},
    {"PWMIn",
      {FieldType::BOOL, 0, 0, 0, 0,
        [](const std::string & v, DeviceConfiguration & c, const rclcpp::Logger & l) {
          return parseBool(v, c.pwm_in, l);
        }}},
    {"PWMOut",
      {FieldType::BOOL, 0, 0, 0, 0,
        [](const std::string & v, DeviceConfiguration & c, const rclcpp::Logger & l) {
          return parseBool(v, c.pwm_out, l);
        }}},
    {"PWMMinPulse",
      {FieldType::UINT16_UNBOUNDED, 0, 0, 0, 0,
        [](const std::string & v, DeviceConfiguration & c, const rclcpp::Logger & l) {
          return parseUInt16(v, c.pwm_min_pulse, l);
        }}},
    {"PWMMaxPulse",
      {FieldType::UINT16_UNBOUNDED, 0, 0, 0, 0,
        [](const std::string & v, DeviceConfiguration & c, const rclcpp::Logger & l) {
          return parseUInt16(v, c.pwm_max_pulse, l);
        }}},
    {"PWMNeutralPulse",
      {FieldType::UINT16_UNBOUNDED, 0, 0, 0, 0,
        [](const std::string & v, DeviceConfiguration & c, const rclcpp::Logger & l) {
          return parseUInt16(v, c.pwm_neutral_pulse, l);
        }}},
    {"PWMDeadband",
      {FieldType::UINT8_UNBOUNDED, 0, 0, 0, 0,
        [](const std::string & v, DeviceConfiguration & c, const rclcpp::Logger & l) {
          return parseUInt8(v, c.pwm_deadband, l);
        }}},
    {"PWMArcMode",
      {FieldType::BOOL, 0, 0, 0, 0,
        [](const std::string & v, DeviceConfiguration & c, const rclcpp::Logger & l) {
          return parseBool(v, c.pwm_arc_mode, l);
        }}},
    {"DOut",
      {FieldType::UINT8_UNBOUNDED, 0, 0, 0, 0,
        [](const std::string & v, DeviceConfiguration & c, const rclcpp::Logger & l) {
          return parseUInt8(v, c.digital_out, l);
        }}}
  };

  // Process all fields in config_map using the field_specs map
  for (const auto & [key, value] : config_map) {
    auto it = field_specs.find(key);
    if (it == field_specs.end()) {
      RCLCPP_DEBUG(logger, "Unknown configuration field: %s", key.c_str());
      continue;
    }

    if (!it->second.setter(value, config, logger)) {
      parse_errors = true;
    }
  }

  // Set timestamp to now
  config.timestamp = std::chrono::steady_clock::now();

  if (parse_errors) {
    RCLCPP_WARN(logger, "Configuration parse completed with errors");
    // Still return the configuration with whatever we could parse
  }

  return config;
}

}  // namespace ros2_roamadome
