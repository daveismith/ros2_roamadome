#include "ros2_roamadome/device_parameter_specs.hpp"

namespace ros2_roamadome
{

DeviceParameterSpecBase::DeviceParameterSpecBase(
  std::string full_name,
  rclcpp::ParameterValue default_value,
  std::string description)
: full_name_(std::move(full_name)),
  default_value_(std::move(default_value)),
  description_(std::move(description))
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

WritableParameterSpecBase::WritableParameterSpecBase(
  std::string field_name,
  std::string full_name,
  rclcpp::ParameterValue default_value,
  std::string description,
  std::string command_prefix)
: DeviceParameterSpecBase(std::move(full_name), std::move(default_value), std::move(description)),
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

const std::vector<const DeviceParameterSpecBase *> & getAllParameterSpecs()
{
  static const std::vector<const DeviceParameterSpecBase *> all_specs = []() {
    // Read-only specs
      static const ReadOnlyParameterSpec<uint16_t> kDeviceHomePos(
        "home_pos", 0, "Home position", &DeviceConfiguration::home_pos);
      static const ReadOnlyParameterSpec<uint8_t> kDeviceMaxSpeed(
        "max_speed", 50, "Maximum speed", &DeviceConfiguration::max_speed);
      static const ReadOnlyParameterSpec<uint8_t> kDeviceMinSpeed(
        "min_speed", 15, "Minimum speed", &DeviceConfiguration::min_speed);
      static const ReadOnlyParameterSpec<uint8_t> kDeviceInputSpeed(
        "input_speed", 100, "Input speed", &DeviceConfiguration::input_speed);
      static const ReadOnlyParameterSpec<bool> kDeviceScaling(
        "scaling", false, "Scaling mode", &DeviceConfiguration::scaling);
      static const ReadOnlyParameterSpec<bool> kDeviceInverted(
        "inverted", true, "Inverted mode", &DeviceConfiguration::inverted);
      static const ReadOnlyParameterSpec<uint8_t> kDeviceTimeout(
        "timeout", 5, "Timeout", &DeviceConfiguration::timeout);
      static const ReadOnlyParameterSpec<bool> kDeviceAutoSafety(
        "auto_safety", false, "Auto safety", &DeviceConfiguration::auto_safety);
      static const ReadOnlyParameterSpec<bool> kDeviceAutoRestart(
        "auto_restart", true, "Auto restart", &DeviceConfiguration::auto_restart);
      static const ReadOnlyParameterSpec<uint8_t> kDeviceAccelerationScale(
        "acceleration_scale", 20, "Acceleration scale", &DeviceConfiguration::acceleration_scale);
      static const ReadOnlyParameterSpec<uint8_t> kDeviceDecelerationScale(
        "deceleration_scale", 50, "Deceleration scale", &DeviceConfiguration::deceleration_scale);
      static const ReadOnlyParameterSpec<uint8_t> kDeviceHomeMinDelay(
        "home_min_delay", 6, "Home minimum delay", &DeviceConfiguration::home_min_delay);
      static const ReadOnlyParameterSpec<uint8_t> kDeviceHomeMaxDelay(
        "home_max_delay", 8, "Home maximum delay", &DeviceConfiguration::home_max_delay);
      static const ReadOnlyParameterSpec<uint8_t> kDeviceTargetMinDelay(
        "target_min_delay", 0, "Target minimum delay", &DeviceConfiguration::target_min_delay);
      static const ReadOnlyParameterSpec<uint8_t> kDeviceTargetMaxDelay(
        "target_max_delay", 1, "Target maximum delay", &DeviceConfiguration::target_max_delay);
      static const ReadOnlyParameterSpec<uint16_t> kDeviceSetupAngularVelocity(
        "setup_angular_velocity", 100, "Setup angular velocity",
        &DeviceConfiguration::setup_angular_velocity);
      static const ReadOnlyParameterSpec<uint8_t> kDeviceSpeedHome(
        "speed_home", 40, "Home speed", &DeviceConfiguration::speed_home);
      static const ReadOnlyParameterSpec<uint8_t> kDeviceSpeedTarget(
        "speed_target", 100, "Target speed", &DeviceConfiguration::speed_target);

    // Writable specs
      static const WritableParameterSpec<bool> kDeviceAutoMode(
        "auto_mode", false, "Enable/disable automatic dome movement mode", "#DPAUTO",
        &DeviceConfiguration::auto_mode, false, true, getBoolTypeLambda<bool>());
      static const WritableParameterSpec<bool> kDeviceHomeMode(
        "home_mode", false, "Enable/disable home mode", "#DPHOME", &DeviceConfiguration::home_mode,
        false, true, getBoolTypeLambda<bool>());
      static const WritableParameterSpec<uint8_t> kDeviceAutoLeft(
        "auto_left", 80, "Maximum distance to auto left (0-180 degrees)", "#DPAUTOLEFT",
        &DeviceConfiguration::auto_left, 0, 180);
      static const WritableParameterSpec<uint8_t> kDeviceAutoRight(
        "auto_right", 80, "Maximum distance to auto right (0-180 degrees)", "#DPAUTORIGHT",
        &DeviceConfiguration::auto_right, 0, 180);
      static const WritableParameterSpec<uint8_t> kDeviceAutoMinDelay(
        "auto_min_delay", 6, "Minimum delay for auto mode (seconds)", "#DPAUTOMIN",
        &DeviceConfiguration::auto_min_delay, 0, 255);
      static const WritableParameterSpec<uint8_t> kDeviceAutoMaxDelay(
        "auto_max_delay", 8, "Maximum delay for auto mode (seconds)", "#DPAUTOMAX",
        &DeviceConfiguration::auto_max_delay, 0, 255);
      static const WritableParameterSpec<uint8_t> kDeviceSpeedAuto(
        "speed_auto", 30, "Speed for auto mode (0-100)", "#DPAUTOSPEED",
        &DeviceConfiguration::speed_auto, 0, 100);
      static const WritableParameterSpec<uint8_t> kDeviceFudge(
        "fudge", 5, "Target position tolerance (0-20 degrees)", "#DPFUDGE",
        &DeviceConfiguration::fudge, 0, 20);

      return std::vector<const DeviceParameterSpecBase *>{
      &kDeviceHomePos,
      &kDeviceMaxSpeed,
      &kDeviceMinSpeed,
      &kDeviceInputSpeed,
      &kDeviceScaling,
      &kDeviceInverted,
      &kDeviceTimeout,
      &kDeviceAutoSafety,
      &kDeviceAutoRestart,
      &kDeviceAccelerationScale,
      &kDeviceDecelerationScale,
      &kDeviceHomeMinDelay,
      &kDeviceHomeMaxDelay,
      &kDeviceTargetMinDelay,
      &kDeviceTargetMaxDelay,
      &kDeviceSetupAngularVelocity,
      &kDeviceSpeedHome,
      &kDeviceSpeedTarget,
      &kDeviceAutoMode,
      &kDeviceHomeMode,
      &kDeviceAutoLeft,
      &kDeviceAutoRight,
      &kDeviceAutoMinDelay,
      &kDeviceAutoMaxDelay,
      &kDeviceSpeedAuto,
      &kDeviceFudge,
      };
    }();

  return all_specs;
}

const WritableParameterSpecBase * findWritableParameterSpec(const std::string & field_name)
{
  for (const auto * spec : getAllParameterSpecs()) {
    if (!spec->isWritable()) {
      continue;
    }
    const auto * writable = dynamic_cast<const WritableParameterSpecBase *>(spec);
    if (writable && field_name == writable->fieldName()) {
      return writable;
    }
  }
  return nullptr;
}

}  // namespace ros2_roamadome
