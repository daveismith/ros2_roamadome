#ifndef ros2_roamadome__DEVICE_PARAMETER_SPECS_HPP_
#define ros2_roamadome__DEVICE_PARAMETER_SPECS_HPP_

#include "rclcpp/rclcpp.hpp"
#include "ros2_roamadome/roamadome_config_parser.hpp"

#include <array>
#include <functional>
#include <vector>
#include <string>

namespace ros2_roamadome
{

class DeviceParameterSpecBase
{
public:
  using ConfigGetter = std::function<rclcpp::ParameterValue(const DeviceConfiguration &)>;

  virtual ~DeviceParameterSpecBase() = default;

  virtual void declareParameter(const rclcpp::Node::SharedPtr & node) const = 0;
  virtual bool isWritable() const = 0;
  rclcpp::Parameter toParameter(const DeviceConfiguration & config) const;

  const std::string & fullName() const;
  const std::string & description() const;
  const rclcpp::ParameterValue & defaultValue() const;

protected:
  DeviceParameterSpecBase(
    std::string full_name,
    rclcpp::ParameterValue default_value,
    std::string description,
    ConfigGetter getter);

  bool defaultIsInteger() const;
  bool defaultIsBool() const;

private:
  std::string full_name_;
  rclcpp::ParameterValue default_value_;
  std::string description_;
  ConfigGetter getter_;
};

class ReadOnlyParameterSpec : public DeviceParameterSpecBase
{
public:
  ReadOnlyParameterSpec(
    std::string full_name,
    rclcpp::ParameterValue default_value,
    std::string description,
    ConfigGetter getter);

  void declareParameter(const rclcpp::Node::SharedPtr & node) const override;
  bool isWritable() const override;
};

class WritableParameterSpec : public DeviceParameterSpecBase
{
public:
  using ParameterValidator = std::function<bool(const rclcpp::Parameter &, std::string *)>;

  WritableParameterSpec(
    std::string field_name,
    std::string full_name,
    rclcpp::ParameterValue default_value,
    std::string description,
    std::string command_prefix,
    ConfigGetter getter,
    ParameterValidator validator);

  void declareParameter(const rclcpp::Node::SharedPtr & node) const override;
  bool isWritable() const override;

  bool validateAndBuildCommand(
    const rclcpp::Parameter & parameter,
    std::string * command,
    std::string * reason) const;

  const std::string & fieldName() const;

private:
  std::string field_name_;
  std::string command_prefix_;
  ParameterValidator validator_;
};

WritableParameterSpec::ParameterValidator getInclusiveBoundsLambda(int min_value, int max_value);
WritableParameterSpec::ParameterValidator getBoolTypeLambda();

const std::vector<const DeviceParameterSpecBase *> & getAllParameterSpecs();
const WritableParameterSpec * findWritableParameterSpec(const std::string & field_name);

}  // namespace ros2_roamadome

#endif  // ros2_roamadome__DEVICE_PARAMETER_SPECS_HPP_
