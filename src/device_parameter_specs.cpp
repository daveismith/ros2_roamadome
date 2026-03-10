#include "ros2_roamadome/device_parameter_specs.hpp"

#include <sstream>

namespace ros2_roamadome
{

namespace
{

template<typename TValue>
std::string buildCommand(const std::string & prefix, TValue value)
{
  std::ostringstream stream;
  stream << prefix << value;
  return stream.str();
}

}  // namespace

DeviceParameterSpecBase::DeviceParameterSpecBase(
  std::string full_name,
  rclcpp::ParameterValue default_value,
  std::string description,
  ConfigGetter getter)
: full_name_(std::move(full_name)),
  default_value_(std::move(default_value)),
  description_(std::move(description)),
  getter_(std::move(getter))
{
}

rclcpp::Parameter DeviceParameterSpecBase::toParameter(const DeviceConfiguration & config) const
{
  return rclcpp::Parameter(full_name_, getter_(config));
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

bool DeviceParameterSpecBase::defaultIsInteger() const
{
  return rclcpp::ParameterType::PARAMETER_INTEGER == default_value_.get_type();
}

bool DeviceParameterSpecBase::defaultIsBool() const
{
  return rclcpp::ParameterType::PARAMETER_BOOL == default_value_.get_type();
}

ReadOnlyParameterSpec::ReadOnlyParameterSpec(
  std::string full_name,
  rclcpp::ParameterValue default_value,
  std::string description,
  ConfigGetter getter)
: DeviceParameterSpecBase(
    std::move(full_name), std::move(default_value), std::move(description), std::move(getter))
{
}

void ReadOnlyParameterSpec::declareParameter(const rclcpp::Node::SharedPtr & node) const
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  descriptor.description = description();
  node->declare_parameter(fullName(), defaultValue(), descriptor);
}

bool ReadOnlyParameterSpec::isWritable() const
{
  return false;
}

WritableParameterSpec::WritableParameterSpec(
  std::string field_name,
  std::string full_name,
  rclcpp::ParameterValue default_value,
  std::string description,
  std::string command_prefix,
  ConfigGetter getter,
  ParameterValidator validator)
: DeviceParameterSpecBase(
    std::move(full_name), std::move(default_value), std::move(description), std::move(getter)),
  field_name_(std::move(field_name)),
  command_prefix_(std::move(command_prefix)),
  validator_(std::move(validator))
{
}

void WritableParameterSpec::declareParameter(const rclcpp::Node::SharedPtr & node) const
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = false;
  descriptor.description = description();
  node->declare_parameter(fullName(), defaultValue(), descriptor);
}

bool WritableParameterSpec::isWritable() const
{
  return true;
}

bool WritableParameterSpec::validateAndBuildCommand(
  const rclcpp::Parameter & parameter,
  std::string * command,
  std::string * reason) const
{
  if (!validator_) {
    *reason = "missing validator for " + fullName();
    return false;
  }

  if (!validator_(parameter, reason)) {
    return false;
  }

  if (defaultIsBool()) {
    *command = buildCommand(command_prefix_, parameter.as_bool() ? 1 : 0);
    return true;
  }

  *command = buildCommand(command_prefix_, parameter.as_int());
  return true;
}

const std::string & WritableParameterSpec::fieldName() const
{
  return field_name_;
}

WritableParameterSpec::ParameterValidator getInclusiveBoundsLambda(int min_value, int max_value)
{
  return [min_value, max_value](const rclcpp::Parameter & parameter, std::string * reason) {
           int value = 0;
           try {
             value = static_cast<int>(parameter.as_int());
           } catch (const rclcpp::ParameterTypeException & e) {
             *reason = std::string("invalid type for ") + parameter.get_name() + ": " + e.what();
             return false;
           }

           if (value < min_value || value > max_value) {
             std::ostringstream stream;
             stream << parameter.get_name() << " must be in range [" << min_value << ", "
                    << max_value << "]";
             *reason = stream.str();
             return false;
           }
           return true;
         };
}

WritableParameterSpec::ParameterValidator getBoolTypeLambda()
{
  return [](const rclcpp::Parameter & parameter, std::string * reason) {
           try {
             (void) parameter.as_bool();
           } catch (const rclcpp::ParameterTypeException & e) {
             *reason = std::string("invalid type for ") + parameter.get_name() + ": " + e.what();
             return false;
           }
           return true;
         };
}

const std::vector<const DeviceParameterSpecBase *> & getAllParameterSpecs()
{
  static const std::vector<const DeviceParameterSpecBase *> all_specs = []() {
    // Read-only specs
      static const ReadOnlyParameterSpec kDeviceHomePos(
        "device.home_pos", rclcpp::ParameterValue(0), "Home position",
        [](const DeviceConfiguration & c) {
          return rclcpp::ParameterValue(static_cast<int>(c.home_pos));
        });
      static const ReadOnlyParameterSpec kDeviceMaxSpeed(
        "device.max_speed", rclcpp::ParameterValue(50), "Maximum speed",
        [](const DeviceConfiguration & c) {
          return rclcpp::ParameterValue(static_cast<int>(c.max_speed));
        });
      static const ReadOnlyParameterSpec kDeviceMinSpeed(
        "device.min_speed", rclcpp::ParameterValue(15), "Minimum speed",
        [](const DeviceConfiguration & c) {
          return rclcpp::ParameterValue(static_cast<int>(c.min_speed));
        });
      static const ReadOnlyParameterSpec kDeviceInputSpeed(
        "device.input_speed", rclcpp::ParameterValue(100), "Input speed",
        [](const DeviceConfiguration & c) {
          return rclcpp::ParameterValue(static_cast<int>(c.input_speed));
        });
      static const ReadOnlyParameterSpec kDeviceScaling(
        "device.scaling", rclcpp::ParameterValue(false), "Scaling mode",
        [](const DeviceConfiguration & c) {return rclcpp::ParameterValue(c.scaling);}
      );
      static const ReadOnlyParameterSpec kDeviceInverted(
        "device.inverted", rclcpp::ParameterValue(true), "Inverted mode",
        [](const DeviceConfiguration & c) {return rclcpp::ParameterValue(c.inverted);}
      );
      static const ReadOnlyParameterSpec kDeviceTimeout(
        "device.timeout", rclcpp::ParameterValue(5), "Timeout",
        [](const DeviceConfiguration & c) {
          return rclcpp::ParameterValue(static_cast<int>(c.timeout));
        });
      static const ReadOnlyParameterSpec kDeviceAutoSafety(
        "device.auto_safety", rclcpp::ParameterValue(false), "Auto safety",
        [](const DeviceConfiguration & c) {return rclcpp::ParameterValue(c.auto_safety);}
      );
      static const ReadOnlyParameterSpec kDeviceAutoRestart(
        "device.auto_restart", rclcpp::ParameterValue(true), "Auto restart",
        [](const DeviceConfiguration & c) {return rclcpp::ParameterValue(c.auto_restart);}
      );
      static const ReadOnlyParameterSpec kDeviceAccelerationScale(
        "device.acceleration_scale", rclcpp::ParameterValue(20),
        "Acceleration scale", [](const DeviceConfiguration & c) {
          return rclcpp::ParameterValue(static_cast<int>(c.acceleration_scale));
        });
      static const ReadOnlyParameterSpec kDeviceDecelerationScale(
        "device.deceleration_scale", rclcpp::ParameterValue(50),
        "Deceleration scale", [](const DeviceConfiguration & c) {
          return rclcpp::ParameterValue(static_cast<int>(c.deceleration_scale));
        });
      static const ReadOnlyParameterSpec kDeviceHomeMinDelay(
        "device.home_min_delay", rclcpp::ParameterValue(6),
        "Home minimum delay", [](const DeviceConfiguration & c) {
          return rclcpp::ParameterValue(static_cast<int>(c.home_min_delay));
        });
      static const ReadOnlyParameterSpec kDeviceHomeMaxDelay(
        "device.home_max_delay", rclcpp::ParameterValue(8),
        "Home maximum delay", [](const DeviceConfiguration & c) {
          return rclcpp::ParameterValue(static_cast<int>(c.home_max_delay));
        });
      static const ReadOnlyParameterSpec kDeviceTargetMinDelay(
        "device.target_min_delay", rclcpp::ParameterValue(0),
        "Target minimum delay", [](const DeviceConfiguration & c) {
          return rclcpp::ParameterValue(static_cast<int>(c.target_min_delay));
        });
      static const ReadOnlyParameterSpec kDeviceTargetMaxDelay(
        "device.target_max_delay", rclcpp::ParameterValue(1),
        "Target maximum delay", [](const DeviceConfiguration & c) {
          return rclcpp::ParameterValue(static_cast<int>(c.target_max_delay));
        });
      static const ReadOnlyParameterSpec kDeviceSetupAngularVelocity(
        "device.setup_angular_velocity", rclcpp::ParameterValue(100),
        "Setup angular velocity", [](const DeviceConfiguration & c) {
          return rclcpp::ParameterValue(static_cast<int>(c.setup_angular_velocity));
        });
      static const ReadOnlyParameterSpec kDeviceSpeedHome(
        "device.speed_home", rclcpp::ParameterValue(40), "Home speed",
        [](const DeviceConfiguration & c) {
          return rclcpp::ParameterValue(static_cast<int>(c.speed_home));
        });
      static const ReadOnlyParameterSpec kDeviceSpeedTarget(
        "device.speed_target", rclcpp::ParameterValue(100), "Target speed",
        [](const DeviceConfiguration & c) {
          return rclcpp::ParameterValue(static_cast<int>(c.speed_target));
        });

    // Writable specs
      static const WritableParameterSpec kDeviceAutoMode(
        "auto_mode", "device.auto_mode", rclcpp::ParameterValue(false),
        "Enable/disable automatic dome movement mode", "#DPAUTO",
        [](const DeviceConfiguration & c) {return rclcpp::ParameterValue(c.auto_mode);},
        getBoolTypeLambda());
      static const WritableParameterSpec kDeviceHomeMode(
        "home_mode", "device.home_mode", rclcpp::ParameterValue(false),
        "Enable/disable home mode", "#DPHOME",
        [](const DeviceConfiguration & c) {return rclcpp::ParameterValue(c.home_mode);},
        getBoolTypeLambda());
      static const WritableParameterSpec kDeviceAutoLeft(
        "auto_left", "device.auto_left", rclcpp::ParameterValue(80),
        "Maximum distance to auto left (0-180 degrees)", "#DPAUTOLEFT",
        [](const DeviceConfiguration & c) {
          return rclcpp::ParameterValue(static_cast<int>(c.auto_left));
        }, getInclusiveBoundsLambda(0, 180));
      static const WritableParameterSpec kDeviceAutoRight(
        "auto_right", "device.auto_right", rclcpp::ParameterValue(80),
        "Maximum distance to auto right (0-180 degrees)", "#DPAUTORIGHT",
        [](const DeviceConfiguration & c) {
          return rclcpp::ParameterValue(static_cast<int>(c.auto_right));
        }, getInclusiveBoundsLambda(0, 180));
      static const WritableParameterSpec kDeviceAutoMinDelay(
        "auto_min_delay", "device.auto_min_delay", rclcpp::ParameterValue(6),
        "Minimum delay for auto mode (seconds)", "#DPAUTOMIN",
        [](const DeviceConfiguration & c) {
          return rclcpp::ParameterValue(static_cast<int>(c.auto_min_delay));
        }, getInclusiveBoundsLambda(0, 255));
      static const WritableParameterSpec kDeviceAutoMaxDelay(
        "auto_max_delay", "device.auto_max_delay", rclcpp::ParameterValue(8),
        "Maximum delay for auto mode (seconds)", "#DPAUTOMAX",
        [](const DeviceConfiguration & c) {
          return rclcpp::ParameterValue(static_cast<int>(c.auto_max_delay));
        }, getInclusiveBoundsLambda(0, 255));
      static const WritableParameterSpec kDeviceSpeedAuto(
        "speed_auto", "device.speed_auto", rclcpp::ParameterValue(30),
        "Speed for auto mode (0-100)", "#DPAUTOSPEED",
        [](const DeviceConfiguration & c) {
          return rclcpp::ParameterValue(static_cast<int>(c.speed_auto));
        }, getInclusiveBoundsLambda(0, 100));
      static const WritableParameterSpec kDeviceFudge(
        "fudge", "device.fudge", rclcpp::ParameterValue(5),
        "Target position tolerance (0-20 degrees)", "#DPFUDGE",
        [](const DeviceConfiguration & c) {
          return rclcpp::ParameterValue(static_cast<int>(c.fudge));
        }, getInclusiveBoundsLambda(0, 20));

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

const WritableParameterSpec * findWritableParameterSpec(const std::string & field_name)
{
  for (const auto * spec : getAllParameterSpecs()) {
    if (!spec->isWritable()) {
      continue;
    }
    const auto * writable = dynamic_cast<const WritableParameterSpec *>(spec);
    if (writable && field_name == writable->fieldName()) {
      return writable;
    }
  }
  return nullptr;
}

}  // namespace ros2_roamadome
