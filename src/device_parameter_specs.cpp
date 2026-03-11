#include "ros2_roamadome/device_parameter_specs.hpp"

namespace ros2_roamadome
{

namespace
{

std::vector<std::unique_ptr<DeviceParameterSpecBase>> buildParameterSpecs()
{
  std::vector<std::unique_ptr<DeviceParameterSpecBase>> specs;

  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<uint16_t>>(
      "home_pos", 0, "Home position", "HomePos", 0, 359));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<uint8_t>>(
      "max_speed", 50, "Maximum speed", "MaxSpeed", 0, 100));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<uint8_t>>(
      "min_speed", 15, "Minimum speed", "MinSpeed", 0, 100));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<uint8_t>>(
      "input_speed", 100, "Input speed", "InputSpeed", 0, 100));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<bool>>(
      "scaling", false, "Scaling mode", "Scaling"));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<bool>>(
      "inverted", true, "Inverted mode", "Inverted"));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<uint8_t>>(
      "timeout", 5, "Timeout", "Timeout", 0, 30));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<bool>>(
      "auto_safety", false, "Auto safety", "AutoSafety"));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<bool>>(
      "auto_restart", true, "Auto restart", "AutoRestart"));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<uint8_t>>(
      "acceleration_scale", 20, "Acceleration scale", "AccelerationScale"));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<uint8_t>>(
      "deceleration_scale", 50, "Deceleration scale", "DecelerationScale"));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<uint8_t>>(
      "home_min_delay", 6, "Home minimum delay", "HomeMinDelay"));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<uint8_t>>(
      "home_max_delay", 8, "Home maximum delay", "HomeMaxDelay"));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<uint8_t>>(
      "target_min_delay", 0, "Target minimum delay", "TargetMinDelay"));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<uint8_t>>(
      "target_max_delay", 1, "Target maximum delay", "TargetMaxDelay"));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<uint16_t>>(
      "setup_angular_velocity", 100, "Setup angular velocity", "SetupAngularVelocity"));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<uint8_t>>(
      "speed_home", 40, "Home speed", "SpeedHome", 0, 100));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<uint8_t>>(
      "speed_target", 100, "Target speed", "SpeedTarget", 0, 100));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<uint8_t>>(
      "syren_address_in", 129, "Syren input address", "SyrenAddressIn"));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<uint8_t>>(
      "syren_address_out", 129, "Syren output address", "SyrenAddressOut"));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<uint32_t>>(
      "sensor_baud", 115200, "Sensor baud rate", "SensorBaud"));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<uint32_t>>(
      "syren_baud", 9600, "Syren baud rate", "SyrenBaud"));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<uint32_t>>(
      "serial_baud", 9600, "Controller serial baud rate", "SerialBaud"));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<bool>>(
      "serial_in", true, "Serial input enabled", "SerialIn"));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<bool>>(
      "serial_out", true, "Serial output enabled", "SerialOut"));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<bool>>(
      "pwm_in", false, "PWM input enabled", "PWMIn"));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<bool>>(
      "pwm_out", false, "PWM output enabled", "PWMOut"));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<uint16_t>>(
      "pwm_min_pulse", 1000, "PWM minimum pulse width", "PWMMinPulse"));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<uint16_t>>(
      "pwm_max_pulse", 2000, "PWM maximum pulse width", "PWMMaxPulse"));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<uint16_t>>(
      "pwm_neutral_pulse", 1500, "PWM neutral pulse width", "PWMNeutralPulse"));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<uint8_t>>(
      "pwm_deadband", 5, "PWM deadband", "PWMDeadband"));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<bool>>(
      "pwm_arc_mode", false, "PWM arc mode enabled", "PWMArcMode"));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<uint8_t>>(
      "digital_out", 0, "Digital output", "DOut"));

  specs.emplace_back(std::make_unique<WritableParameterSpec<bool>>(
      "auto_mode", false, "Enable/disable automatic dome movement mode", "#DPAUTO",
      "AutoMode", false, true, getBoolTypeLambda<bool>()));
  specs.emplace_back(std::make_unique<WritableParameterSpec<bool>>(
      "home_mode", false, "Enable/disable home mode", "#DPHOME", "HomeMode", false, true,
      getBoolTypeLambda<bool>()));
  specs.emplace_back(std::make_unique<WritableParameterSpec<uint8_t>>(
      "auto_left", 80, "Maximum distance to auto left (0-180 degrees)", "#DPAUTOLEFT",
      "AutoLeft", 0, 180));
  specs.emplace_back(std::make_unique<WritableParameterSpec<uint8_t>>(
      "auto_right", 80, "Maximum distance to auto right (0-180 degrees)", "#DPAUTORIGHT",
      "AutoRight", 0, 180));
  specs.emplace_back(std::make_unique<WritableParameterSpec<uint8_t>>(
      "auto_min_delay", 6, "Minimum delay for auto mode (seconds)", "#DPAUTOMIN",
      "AutoMinDelay", 0, 255));
  specs.emplace_back(std::make_unique<WritableParameterSpec<uint8_t>>(
      "auto_max_delay", 8, "Maximum delay for auto mode (seconds)", "#DPAUTOMAX",
      "AutoMaxDelay", 0, 255));
  specs.emplace_back(std::make_unique<WritableParameterSpec<uint8_t>>(
      "speed_auto", 30, "Speed for auto mode (0-100)", "#DPAUTOSPEED", "SpeedAuto", 0, 100));
  specs.emplace_back(std::make_unique<WritableParameterSpec<uint8_t>>(
      "fudge", 5, "Target position tolerance (0-20 degrees)", "#DPFUDGE", "Fudge", 0, 20));

  return specs;
}

DeviceParameterRegistry & getMetadataRegistry()
{
  static DeviceParameterRegistry registry;
  return registry;
}

}  // namespace

DeviceParameterSpecBase::DeviceParameterSpecBase(
  std::string full_name,
  rclcpp::ParameterValue default_value,
  std::string description,
  std::string firmware_config_key)
: full_name_(std::move(full_name)),
  default_value_(std::move(default_value)),
  description_(std::move(description)),
  firmware_config_key_(std::move(firmware_config_key))
{
}

const std::string & DeviceParameterSpecBase::fullName() const
{
  return full_name_;
}

const std::string & DeviceParameterSpecBase::description() const
{
  return description_;
}

const rclcpp::ParameterValue & DeviceParameterSpecBase::defaultValue() const
{
  return default_value_;
}

const std::string & DeviceParameterSpecBase::firmwareConfigKey() const
{
  return firmware_config_key_;
}

bool DeviceParameterSpecBase::hasFirmwareConfigKey() const
{
  return !firmware_config_key_.empty();
}

WritableParameterSpecBase::WritableParameterSpecBase(
  std::string field_name,
  std::string full_name,
  rclcpp::ParameterValue default_value,
  std::string description,
  std::string command_prefix,
  std::string firmware_config_key)
: DeviceParameterSpecBase(
    std::move(full_name),
    std::move(default_value),
    std::move(description),
    std::move(firmware_config_key)),
  field_name_(std::move(field_name)),
  command_prefix_(std::move(command_prefix))
{
}

bool WritableParameterSpecBase::isWritable() const
{
  return true;
}

const std::string & WritableParameterSpecBase::fieldName() const
{
  return field_name_;
}

const std::string & WritableParameterSpecBase::commandPrefix() const
{
  return command_prefix_;
}

DeviceParameterRegistry::DeviceParameterRegistry()
: specs_(buildParameterSpecs())
{
  all_specs_.reserve(specs_.size());
  for (const auto & spec : specs_) {
    all_specs_.push_back(spec.get());
    if (spec->hasFirmwareConfigKey()) {
      specs_by_config_key_.emplace(spec->firmwareConfigKey(), spec.get());
    }
    if (spec->isWritable()) {
      const auto * writable = dynamic_cast<const WritableParameterSpecBase *>(spec.get());
      if (nullptr != writable) {
        writable_specs_by_field_name_.emplace(writable->fieldName(), writable);
      }
    }
  }
}

const std::vector<const DeviceParameterSpecBase *> & DeviceParameterRegistry::allSpecs() const
{
  return all_specs_;
}

std::vector<rclcpp::Parameter> DeviceParameterRegistry::currentParameters() const
{
  std::vector<rclcpp::Parameter> parameters;
  parameters.reserve(all_specs_.size());
  for (const auto * spec : all_specs_) {
    parameters.push_back(spec->currentParameter());
  }
  return parameters;
}

std::vector<rclcpp::Parameter> DeviceParameterRegistry::currentWritableParameters() const
{
  std::vector<rclcpp::Parameter> parameters;
  for (const auto * spec : all_specs_) {
    if (spec->isWritable()) {
      parameters.push_back(spec->currentParameter());
    }
  }
  return parameters;
}

bool DeviceParameterRegistry::parseConfigMap(
  const std::map<std::string, std::string> & config_map,
  const rclcpp::Logger & logger)
{
  resetToDefaults();
  bool parse_errors = false;

  for (const auto & [key, value] : config_map) {
    const auto it = specs_by_config_key_.find(key);
    if (specs_by_config_key_.end() == it) {
      RCLCPP_DEBUG(logger, "Unknown configuration field: %s", key.c_str());
      continue;
    }

    if (!it->second->parseAndStoreConfigValue(value, logger)) {
      parse_errors = true;
    }
  }

  if (parse_errors) {
    RCLCPP_WARN(logger, "Configuration parse completed with errors");
  }

  return !parse_errors;
}

void DeviceParameterRegistry::resetToDefaults()
{
  for (auto & spec : specs_) {
    spec->resetToDefault();
  }
}

const WritableParameterSpecBase * DeviceParameterRegistry::findWritableParameterSpec(
  const std::string & field_name) const
{
  const auto it = writable_specs_by_field_name_.find(field_name);
  if (writable_specs_by_field_name_.end() != it) {
    return it->second;
  }
  return nullptr;
}

const DeviceParameterSpecBase * DeviceParameterRegistry::findParameterSpecByConfigKey(
  const std::string & config_key) const
{
  const auto it = specs_by_config_key_.find(config_key);
  if (specs_by_config_key_.end() != it) {
    return it->second;
  }
  return nullptr;
}

const std::vector<const DeviceParameterSpecBase *> & getAllParameterSpecs()
{
  return getMetadataRegistry().allSpecs();
}

const WritableParameterSpecBase * findWritableParameterSpec(const std::string & field_name)
{
  return getMetadataRegistry().findWritableParameterSpec(field_name);
}

const DeviceParameterSpecBase * findParameterSpecByConfigKey(const std::string & config_key)
{
  return getMetadataRegistry().findParameterSpecByConfigKey(config_key);
}

}  // namespace ros2_roamadome
