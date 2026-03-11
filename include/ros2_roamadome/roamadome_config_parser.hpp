#ifndef roamadome_config_parser__ROAMADOME_CONFIG_PARSER_HPP_
#define roamadome_config_parser__ROAMADOME_CONFIG_PARSER_HPP_

#include "rclcpp/rclcpp.hpp"
#include <map>
#include <string>
#include <functional>
#include <optional>
#include <chrono>

namespace ros2_roamadome
{

// DeviceConfiguration is defined here (not in roamadome_control.hpp) to avoid
// a circular include: roamadome_control.hpp includes device_parameter_specs.hpp,
// which includes this header.
struct DeviceConfiguration
{
  uint16_t home_pos = 0;
  uint8_t max_speed = 50;
  uint8_t min_speed = 15;
  bool auto_mode = false;
  bool home_mode = false;
  uint8_t input_speed = 100;
  bool scaling = false;
  bool inverted = true;
  uint8_t timeout = 5;
  bool auto_safety = false;
  bool auto_restart = true;
  uint8_t acceleration_scale = 20;
  uint8_t deceleration_scale = 50;
  uint8_t home_min_delay = 6;
  uint8_t home_max_delay = 8;
  uint8_t auto_min_delay = 6;
  uint8_t auto_max_delay = 8;
  uint8_t target_min_delay = 0;
  uint8_t target_max_delay = 1;
  uint16_t setup_angular_velocity = 100;
  uint8_t auto_left = 80;
  uint8_t auto_right = 80;
  uint8_t fudge = 5;
  uint8_t speed_home = 40;
  uint8_t speed_auto = 30;
  uint8_t speed_target = 100;
  uint8_t syren_address_in = 129;
  uint8_t syren_address_out = 129;
  uint32_t sensor_baud = 115200;
  uint32_t syren_baud = 9600;
  uint32_t serial_baud = 9600;
  bool serial_in = true;
  bool serial_out = true;
  bool pwm_in = false;
  bool pwm_out = false;
  uint16_t pwm_min_pulse = 1000;
  uint16_t pwm_max_pulse = 2000;
  uint16_t pwm_neutral_pulse = 1500;
  uint8_t pwm_deadband = 5;
  bool pwm_arc_mode = false;
  uint8_t digital_out = 0;
  std::chrono::steady_clock::time_point timestamp;
};

/**
 * @brief Parser for device configuration data from #DPCONFIG serial output
 *
 * Provides data-driven parsing of key=value configuration format into a typed
 * DeviceConfiguration struct. Uses field parsers to convert string values to
 * appropriate types with validation.
 */
class ConfigurationParser
{
public:
  /**
   * @brief Parse configuration map into typed DeviceConfiguration struct
   *
   * @param config_map Map of configuration key-value pairs from device
   * @param logger ROS logger for error/warning messages
   * @return Parsed configuration with timestamp, or std::nullopt if parsing failed
   */
  static std::optional<DeviceConfiguration> parse(
    const std::map<std::string, std::string> & config_map,
    const rclcpp::Logger & logger);

private:
  // Type-specific field parsers
  template<typename T>
  using FieldParser = std::function<bool(const std::string &, T &, const rclcpp::Logger &)>;

  // Parse uint8_t field with range validation
  static bool parseUInt8(
    const std::string & value_str,
    uint8_t & output,
    const rclcpp::Logger & logger,
    uint8_t min_val = 0,
    uint8_t max_val = 255);

  // Parse uint16_t field with range validation
  static bool parseUInt16(
    const std::string & value_str,
    uint16_t & output,
    const rclcpp::Logger & logger,
    uint16_t min_val = 0,
    uint16_t max_val = 65535);

  // Parse uint32_t field with range validation
  static bool parseUInt32(
    const std::string & value_str,
    uint32_t & output,
    const rclcpp::Logger & logger);

  // Parse bool field (0/1 or true/false)
  static bool parseBool(
    const std::string & value_str,
    bool & output,
    const rclcpp::Logger & logger);
};

}  // namespace ros2_roamadome

#endif  // roamadome_config_parser__ROAMADOME_CONFIG_PARSER_HPP_
