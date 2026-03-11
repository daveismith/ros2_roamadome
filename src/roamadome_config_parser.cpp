#include "ros2_roamadome/roamadome_config_parser.hpp"
#include "ros2_roamadome/device_parameter_specs.hpp"

namespace ros2_roamadome
{

std::optional<DeviceConfiguration> ConfigurationParser::parse(
  const std::map<std::string, std::string> & config_map,
  const rclcpp::Logger & logger)
{
  DeviceConfiguration config;
  bool parse_errors = false;

  for (const auto & [key, value] : config_map) {
    const DeviceParameterSpecBase * spec = findParameterSpecByConfigKey(key);
    if (nullptr == spec) {
      RCLCPP_DEBUG(logger, "Unknown configuration field: %s", key.c_str());
      continue;
    }

    if (!spec->parseAndApplyConfigValue(value, &config, logger)) {
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
