#include "ros2_roamadome/roamadome_config_parser.hpp"

#include "ros2_roamadome/device_parameter_specs.hpp"

namespace ros2_roamadome
{

bool ConfigurationParser::parseIntoRegistry(
  const std::map<std::string, std::string> & config_map,
  DeviceParameterRegistry * registry,
  const rclcpp::Logger & logger)
{
  if (nullptr == registry) {
    RCLCPP_ERROR(logger, "ConfigurationParser requires a valid parameter registry");
    return false;
  }

  return registry->parseConfigMap(config_map, logger);
}

}  // namespace ros2_roamadome
