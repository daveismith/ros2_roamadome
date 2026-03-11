#include "ros2_roamadome/device_parameter_specs.hpp"

namespace ros2_roamadome
{

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

const std::vector<const DeviceParameterSpecBase *> & getAllParameterSpecs()
{
  static const std::vector<const DeviceParameterSpecBase *> all_specs = []() {
    // Read-only specs
      static const ReadOnlyParameterSpec<uint16_t> kDeviceHomePos(
        "home_pos", 0, "Home position", &DeviceConfiguration::home_pos, "HomePos", 0, 359);
      static const ReadOnlyParameterSpec<uint8_t> kDeviceMaxSpeed(
        "max_speed", 50, "Maximum speed", &DeviceConfiguration::max_speed, "MaxSpeed", 0, 100);
      static const ReadOnlyParameterSpec<uint8_t> kDeviceMinSpeed(
        "min_speed", 15, "Minimum speed", &DeviceConfiguration::min_speed, "MinSpeed", 0, 100);
      static const ReadOnlyParameterSpec<uint8_t> kDeviceInputSpeed(
        "input_speed", 100, "Input speed", &DeviceConfiguration::input_speed, "InputSpeed", 0,
        100);
      static const ReadOnlyParameterSpec<bool> kDeviceScaling(
        "scaling", false, "Scaling mode", &DeviceConfiguration::scaling, "Scaling");
      static const ReadOnlyParameterSpec<bool> kDeviceInverted(
        "inverted", true, "Inverted mode", &DeviceConfiguration::inverted, "Inverted");
      static const ReadOnlyParameterSpec<uint8_t> kDeviceTimeout(
        "timeout", 5, "Timeout", &DeviceConfiguration::timeout, "Timeout", 0, 30);
      static const ReadOnlyParameterSpec<bool> kDeviceAutoSafety(
        "auto_safety", false, "Auto safety", &DeviceConfiguration::auto_safety, "AutoSafety");
      static const ReadOnlyParameterSpec<bool> kDeviceAutoRestart(
        "auto_restart", true, "Auto restart", &DeviceConfiguration::auto_restart, "AutoRestart");
      static const ReadOnlyParameterSpec<uint8_t> kDeviceAccelerationScale(
        "acceleration_scale", 20, "Acceleration scale", &DeviceConfiguration::acceleration_scale,
        "AccelerationScale");
      static const ReadOnlyParameterSpec<uint8_t> kDeviceDecelerationScale(
        "deceleration_scale", 50, "Deceleration scale", &DeviceConfiguration::deceleration_scale,
        "DecelerationScale");
      static const ReadOnlyParameterSpec<uint8_t> kDeviceHomeMinDelay(
        "home_min_delay", 6, "Home minimum delay", &DeviceConfiguration::home_min_delay,
        "HomeMinDelay");
      static const ReadOnlyParameterSpec<uint8_t> kDeviceHomeMaxDelay(
        "home_max_delay", 8, "Home maximum delay", &DeviceConfiguration::home_max_delay,
        "HomeMaxDelay");
      static const ReadOnlyParameterSpec<uint8_t> kDeviceTargetMinDelay(
        "target_min_delay", 0, "Target minimum delay", &DeviceConfiguration::target_min_delay,
        "TargetMinDelay");
      static const ReadOnlyParameterSpec<uint8_t> kDeviceTargetMaxDelay(
        "target_max_delay", 1, "Target maximum delay", &DeviceConfiguration::target_max_delay,
        "TargetMaxDelay");
      static const ReadOnlyParameterSpec<uint16_t> kDeviceSetupAngularVelocity(
        "setup_angular_velocity", 100, "Setup angular velocity",
        &DeviceConfiguration::setup_angular_velocity, "SetupAngularVelocity");
      static const ReadOnlyParameterSpec<uint8_t> kDeviceSpeedHome(
        "speed_home", 40, "Home speed", &DeviceConfiguration::speed_home, "SpeedHome", 0, 100);
      static const ReadOnlyParameterSpec<uint8_t> kDeviceSpeedTarget(
        "speed_target", 100, "Target speed", &DeviceConfiguration::speed_target, "SpeedTarget",
        0, 100);
      static const ReadOnlyParameterSpec<uint8_t> kDeviceSyrenAddressIn(
        "syren_address_in", 129, "Syren input address", &DeviceConfiguration::syren_address_in,
        "SyrenAddressIn");
      static const ReadOnlyParameterSpec<uint8_t> kDeviceSyrenAddressOut(
        "syren_address_out", 129, "Syren output address",
        &DeviceConfiguration::syren_address_out, "SyrenAddressOut");
      static const ReadOnlyParameterSpec<uint32_t> kDeviceSensorBaud(
        "sensor_baud", 115200, "Sensor baud rate", &DeviceConfiguration::sensor_baud,
        "SensorBaud");
      static const ReadOnlyParameterSpec<uint32_t> kDeviceSyrenBaud(
        "syren_baud", 9600, "Syren baud rate", &DeviceConfiguration::syren_baud,
        "SyrenBaud");
      static const ReadOnlyParameterSpec<uint32_t> kDeviceSerialBaud(
        "serial_baud", 9600, "Controller serial baud rate", &DeviceConfiguration::serial_baud,
        "SerialBaud");
      static const ReadOnlyParameterSpec<bool> kDeviceSerialIn(
        "serial_in", true, "Serial input enabled", &DeviceConfiguration::serial_in,
        "SerialIn");
      static const ReadOnlyParameterSpec<bool> kDeviceSerialOut(
        "serial_out", true, "Serial output enabled", &DeviceConfiguration::serial_out,
        "SerialOut");
      static const ReadOnlyParameterSpec<bool> kDevicePwmIn(
        "pwm_in", false, "PWM input enabled", &DeviceConfiguration::pwm_in,
        "PWMIn");
      static const ReadOnlyParameterSpec<bool> kDevicePwmOut(
        "pwm_out", false, "PWM output enabled", &DeviceConfiguration::pwm_out,
        "PWMOut");
      static const ReadOnlyParameterSpec<uint16_t> kDevicePwmMinPulse(
        "pwm_min_pulse", 1000, "PWM minimum pulse width",
        &DeviceConfiguration::pwm_min_pulse, "PWMMinPulse");
      static const ReadOnlyParameterSpec<uint16_t> kDevicePwmMaxPulse(
        "pwm_max_pulse", 2000, "PWM maximum pulse width",
        &DeviceConfiguration::pwm_max_pulse, "PWMMaxPulse");
      static const ReadOnlyParameterSpec<uint16_t> kDevicePwmNeutralPulse(
        "pwm_neutral_pulse", 1500, "PWM neutral pulse width",
        &DeviceConfiguration::pwm_neutral_pulse, "PWMNeutralPulse");
      static const ReadOnlyParameterSpec<uint8_t> kDevicePwmDeadband(
        "pwm_deadband", 5, "PWM deadband", &DeviceConfiguration::pwm_deadband,
        "PWMDeadband");
      static const ReadOnlyParameterSpec<bool> kDevicePwmArcMode(
        "pwm_arc_mode", false, "PWM arc mode enabled", &DeviceConfiguration::pwm_arc_mode,
        "PWMArcMode");
      static const ReadOnlyParameterSpec<uint8_t> kDeviceDigitalOut(
        "digital_out", 0, "Digital output", &DeviceConfiguration::digital_out,
        "DOut");

    // Writable specs
      static const WritableParameterSpec<bool> kDeviceAutoMode(
        "auto_mode", false, "Enable/disable automatic dome movement mode", "#DPAUTO",
        &DeviceConfiguration::auto_mode, "AutoMode", false, true, getBoolTypeLambda<bool>());
      static const WritableParameterSpec<bool> kDeviceHomeMode(
        "home_mode", false, "Enable/disable home mode", "#DPHOME", &DeviceConfiguration::home_mode,
        "HomeMode", false, true, getBoolTypeLambda<bool>());
      static const WritableParameterSpec<uint8_t> kDeviceAutoLeft(
        "auto_left", 80, "Maximum distance to auto left (0-180 degrees)", "#DPAUTOLEFT",
        &DeviceConfiguration::auto_left, "AutoLeft", 0, 180);
      static const WritableParameterSpec<uint8_t> kDeviceAutoRight(
        "auto_right", 80, "Maximum distance to auto right (0-180 degrees)", "#DPAUTORIGHT",
        &DeviceConfiguration::auto_right, "AutoRight", 0, 180);
      static const WritableParameterSpec<uint8_t> kDeviceAutoMinDelay(
        "auto_min_delay", 6, "Minimum delay for auto mode (seconds)", "#DPAUTOMIN",
        &DeviceConfiguration::auto_min_delay, "AutoMinDelay", 0, 255);
      static const WritableParameterSpec<uint8_t> kDeviceAutoMaxDelay(
        "auto_max_delay", 8, "Maximum delay for auto mode (seconds)", "#DPAUTOMAX",
        &DeviceConfiguration::auto_max_delay, "AutoMaxDelay", 0, 255);
      static const WritableParameterSpec<uint8_t> kDeviceSpeedAuto(
        "speed_auto", 30, "Speed for auto mode (0-100)", "#DPAUTOSPEED",
        &DeviceConfiguration::speed_auto, "SpeedAuto", 0, 100);
      static const WritableParameterSpec<uint8_t> kDeviceFudge(
        "fudge", 5, "Target position tolerance (0-20 degrees)", "#DPFUDGE",
        &DeviceConfiguration::fudge, "Fudge", 0, 20);

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
      &kDeviceSyrenAddressIn,
      &kDeviceSyrenAddressOut,
      &kDeviceSensorBaud,
      &kDeviceSyrenBaud,
      &kDeviceSerialBaud,
      &kDeviceSerialIn,
      &kDeviceSerialOut,
      &kDevicePwmIn,
      &kDevicePwmOut,
      &kDevicePwmMinPulse,
      &kDevicePwmMaxPulse,
      &kDevicePwmNeutralPulse,
      &kDevicePwmDeadband,
      &kDevicePwmArcMode,
      &kDeviceDigitalOut,
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

const std::map<std::string, const WritableParameterSpecBase *> & getWritableSpecsByFieldName()
{
  static const std::map<std::string, const WritableParameterSpecBase *> writable_specs = []() {
      std::map<std::string, const WritableParameterSpecBase *> specs;
      for (const auto * spec : getAllParameterSpecs()) {
        if (!spec->isWritable()) {
          continue;
        }
        const auto * writable = dynamic_cast<const WritableParameterSpecBase *>(spec);
        if (nullptr != writable) {
          specs.emplace(writable->fieldName(), writable);
        }
      }
      return specs;
    }();
  return writable_specs;
}

const std::map<std::string, const DeviceParameterSpecBase *> & getSpecsByConfigKey()
{
  static const std::map<std::string, const DeviceParameterSpecBase *> config_key_specs = []() {
      std::map<std::string, const DeviceParameterSpecBase *> specs;
      for (const auto * spec : getAllParameterSpecs()) {
        if (spec->hasFirmwareConfigKey()) {
          specs.emplace(spec->firmwareConfigKey(), spec);
        }
      }
      return specs;
    }();
  return config_key_specs;
}

const WritableParameterSpecBase * findWritableParameterSpec(const std::string & field_name)
{
  const auto & writable_specs = getWritableSpecsByFieldName();
  const auto it = writable_specs.find(field_name);
  if (writable_specs.end() != it) {
    return it->second;
  }
  return nullptr;
}

const DeviceParameterSpecBase * findParameterSpecByConfigKey(const std::string & config_key)
{
  const auto & specs = getSpecsByConfigKey();
  const auto it = specs.find(config_key);
  if (specs.end() != it) {
    return it->second;
  }
  return nullptr;
}

}  // namespace ros2_roamadome
