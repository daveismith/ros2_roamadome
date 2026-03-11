#include "ros2_roamadome/device_parameter_specs.hpp"

namespace ros2_roamadome
{

namespace
{

std::vector<std::unique_ptr<DeviceParameterSpecBase>> buildParameterSpecs()
{
  std::vector<std::unique_ptr<DeviceParameterSpecBase>> specs;

  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<uint16_t>>(
      "home_pos", "HomePos", "Home position", 0, 0, 359));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<uint8_t>>(
      "max_speed", "MaxSpeed", "Maximum speed", 50, 0, 100));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<uint8_t>>(
      "min_speed", "MinSpeed", "Minimum speed", 15, 0, 100));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<uint8_t>>(
      "input_speed", "InputSpeed", "Input speed", 100, 0, 100));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<bool>>(
      "scaling", "Scaling", "Scaling mode", false));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<bool>>(
      "inverted", "Inverted", "Inverted mode", true));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<uint8_t>>(
      "timeout", "Timeout", "Timeout", 5, 0, 30));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<bool>>(
      "auto_safety", "AutoSafety", "Auto safety", false));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<bool>>(
      "auto_restart", "AutoRestart", "Auto restart", true));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<uint8_t>>(
      "acceleration_scale", "AccelerationScale", "Acceleration scale", 20));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<uint8_t>>(
      "deceleration_scale", "DecelerationScale", "Deceleration scale", 50));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<uint8_t>>(
      "home_min_delay", "HomeMinDelay", "Home minimum delay", 6));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<uint8_t>>(
      "home_max_delay", "HomeMaxDelay", "Home maximum delay", 8));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<uint8_t>>(
      "target_min_delay", "TargetMinDelay", "Target minimum delay", 0));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<uint8_t>>(
      "target_max_delay", "TargetMaxDelay", "Target maximum delay", 1));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<uint16_t>>(
      "setup_angular_velocity", "SetupAngularVelocity", "Setup angular velocity", 100));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<uint8_t>>(
      "speed_home", "SpeedHome", "Home speed", 40, 0, 100));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<uint8_t>>(
      "speed_target", "SpeedTarget", "Target speed", 100, 0, 100));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<uint8_t>>(
      "syren_address_in", "SyrenAddressIn", "Syren input address", 129));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<uint8_t>>(
      "syren_address_out", "SyrenAddressOut", "Syren output address", 129));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<uint32_t>>(
      "sensor_baud", "SensorBaud", "Sensor baud rate", 115200));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<uint32_t>>(
      "syren_baud", "SyrenBaud", "Syren baud rate", 9600));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<uint32_t>>(
      "serial_baud", "SerialBaud", "Controller serial baud rate", 9600));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<bool>>(
      "serial_in", "SerialIn", "Serial input enabled", true));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<bool>>(
      "serial_out", "SerialOut", "Serial output enabled", true));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<bool>>(
      "pwm_in", "PWMIn", "PWM input enabled", false));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<bool>>(
      "pwm_out", "PWMOut", "PWM output enabled", false));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<uint16_t>>(
      "pwm_min_pulse", "PWMMinPulse", "PWM minimum pulse width", 1000));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<uint16_t>>(
      "pwm_max_pulse", "PWMMaxPulse", "PWM maximum pulse width", 2000));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<uint16_t>>(
      "pwm_neutral_pulse", "PWMNeutralPulse", "PWM neutral pulse width", 1500));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<uint8_t>>(
      "pwm_deadband", "PWMDeadband", "PWM deadband", 5));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<bool>>(
      "pwm_arc_mode", "PWMArcMode", "PWM arc mode enabled", false));
  specs.emplace_back(std::make_unique<ReadOnlyParameterSpec<uint8_t>>(
      "digital_out", "DOut", "Digital output", 0));

  specs.emplace_back(std::make_unique<WritableParameterSpec<bool>>(
      "auto_mode", "AutoMode", "Enable/disable automatic dome movement mode", "#DPAUTO",
      false, false, true, getBoolTypeLambda<bool>()));
  specs.emplace_back(std::make_unique<WritableParameterSpec<bool>>(
      "home_mode", "HomeMode", "Enable/disable home mode", "#DPHOME",
      false, false, true, getBoolTypeLambda<bool>()));
  specs.emplace_back(std::make_unique<WritableParameterSpec<uint8_t>>(
      "auto_left", "AutoLeft", "Maximum distance to auto left (0-180 degrees)",
      "#DPAUTOLEFT", 80, 0, 180));
  specs.emplace_back(std::make_unique<WritableParameterSpec<uint8_t>>(
      "auto_right", "AutoRight", "Maximum distance to auto right (0-180 degrees)",
      "#DPAUTORIGHT", 80, 0, 180));
  specs.emplace_back(std::make_unique<WritableParameterSpec<uint8_t>>(
      "auto_min_delay", "AutoMinDelay", "Minimum delay for auto mode (seconds)",
      "#DPAUTOMIN", 6, 0, 255));
  specs.emplace_back(std::make_unique<WritableParameterSpec<uint8_t>>(
      "auto_max_delay", "AutoMaxDelay", "Maximum delay for auto mode (seconds)",
      "#DPAUTOMAX", 8, 0, 255));
  specs.emplace_back(std::make_unique<WritableParameterSpec<uint8_t>>(
      "speed_auto", "SpeedAuto", "Speed for auto mode (0-100)", "#DPAUTOSPEED",
      30, 0, 100));
  specs.emplace_back(std::make_unique<WritableParameterSpec<uint8_t>>(
      "fudge", "Fudge", "Target position tolerance (0-20 degrees)", "#DPFUDGE",
      5, 0, 20));

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
  std::string firmware_config_key,
  std::string description,
  rclcpp::ParameterValue default_value)
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

WritableParameterSpecBase::WritableParameterSpecBase(
  std::string field_name,
  std::string full_name,
  std::string firmware_config_key,
  std::string description,
  std::string command_prefix,
  rclcpp::ParameterValue default_value)
: DeviceParameterSpecBase(
    std::move(full_name),
    std::move(firmware_config_key),
    std::move(description),
    std::move(default_value)),
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
    specs_by_config_key_.emplace(spec->firmwareConfigKey(), spec.get());
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
