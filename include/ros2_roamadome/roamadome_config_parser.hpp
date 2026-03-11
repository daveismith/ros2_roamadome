#ifndef roamadome_config_parser__ROAMADOME_CONFIG_PARSER_HPP_
#define roamadome_config_parser__ROAMADOME_CONFIG_PARSER_HPP_

#include "rclcpp/rclcpp.hpp"
#include <map>
#include <string>

namespace ros2_roamadome
{

class DeviceParameterRegistry;

/**
 * @brief Parser for device configuration data from #DPCONFIG serial output
 *
 * Provides data-driven parsing of key=value configuration format into the
 * instance-owned DeviceParameterRegistry. Field definitions and storage live in
 * the parameter specs themselves.
 */
class ConfigurationParser
{
public:
  /**
   * @brief Parse configuration map into an instance-owned parameter registry
   *
   * @param config_map Map of configuration key-value pairs from device
   * @param registry Registry whose parameter specs own the parsed values
   * @param logger ROS logger for error/warning messages
   * @return True when all known fields parsed successfully, false if one or more
   *   recognized fields failed validation.
   */
  static bool parseIntoRegistry(
    const std::map<std::string, std::string> & config_map,
    DeviceParameterRegistry * registry,
    const rclcpp::Logger & logger);

private:
  // Intentionally empty for now; parsing is delegated to device parameter specs.
};

}  // namespace ros2_roamadome

#endif  // roamadome_config_parser__ROAMADOME_CONFIG_PARSER_HPP_
