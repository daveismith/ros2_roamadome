#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "ros2_roamadome/device_parameter_specs.hpp"

namespace ros2_roamadome
{

TEST(DeviceParameterSpecsTest, GetAllParameterSpecs_ContainsExpectedCounts)
{
  const auto & specs = getAllParameterSpecs();
  EXPECT_EQ(specs.size(), 41U);

  size_t writable_count = 0;
  for (const auto * spec : specs) {
    if (spec->isWritable()) {
      writable_count++;
    }
  }

  EXPECT_EQ(writable_count, 8U);
}

TEST(DeviceParameterSpecsTest, DeviceConfigurationFields_HaveConfigKeyMappings)
{
  const std::vector<std::string> required_config_keys = {
    "HomePos",
    "MaxSpeed",
    "MinSpeed",
    "AutoMode",
    "HomeMode",
    "InputSpeed",
    "Scaling",
    "Inverted",
    "Timeout",
    "AutoSafety",
    "AutoRestart",
    "AccelerationScale",
    "DecelerationScale",
    "HomeMinDelay",
    "HomeMaxDelay",
    "AutoMinDelay",
    "AutoMaxDelay",
    "TargetMinDelay",
    "TargetMaxDelay",
    "SetupAngularVelocity",
    "AutoLeft",
    "AutoRight",
    "Fudge",
    "SpeedHome",
    "SpeedAuto",
    "SpeedTarget",
    "SyrenAddressIn",
    "SyrenAddressOut",
    "SensorBaud",
    "SyrenBaud",
    "SerialBaud",
    "SerialIn",
    "SerialOut",
    "PWMIn",
    "PWMOut",
    "PWMMinPulse",
    "PWMMaxPulse",
    "PWMNeutralPulse",
    "PWMDeadband",
    "PWMArcMode",
    "DOut"
  };

  for (const auto & config_key : required_config_keys) {
    const auto * spec = findParameterSpecByConfigKey(config_key);
    ASSERT_NE(spec, nullptr) << "Missing spec for config key: " << config_key;
  }
}

TEST(DeviceParameterSpecsTest, FindWritableParameterSpec_DetectsWritableAndReadOnly)
{
  const auto * writable = findWritableParameterSpec("auto_mode");
  ASSERT_NE(writable, nullptr);
  EXPECT_EQ(writable->fieldName(), "auto_mode");

  const auto * read_only = findWritableParameterSpec("home_pos");
  EXPECT_EQ(read_only, nullptr);
}

TEST(DeviceParameterSpecsTest, ValidateAndBuildCommand_BoolParameter)
{
  const auto * spec = findWritableParameterSpec("auto_mode");
  ASSERT_NE(spec, nullptr);

  std::string command;
  std::string reason;

  EXPECT_TRUE(spec->validateAndBuildCommand(
      rclcpp::Parameter("device.auto_mode", true), &command, &reason));
  EXPECT_EQ(command, "#DPAUTO1");

  EXPECT_TRUE(spec->validateAndBuildCommand(
      rclcpp::Parameter("device.auto_mode", false), &command, &reason));
  EXPECT_EQ(command, "#DPAUTO0");
}

TEST(DeviceParameterSpecsTest, ValidateAndBuildCommand_RejectsWrongType)
{
  const auto * spec = findWritableParameterSpec("auto_mode");
  ASSERT_NE(spec, nullptr);

  std::string command;
  std::string reason;
  EXPECT_FALSE(spec->validateAndBuildCommand(
      rclcpp::Parameter("device.auto_mode", 1), &command, &reason));
  EXPECT_FALSE(reason.empty());
}

TEST(DeviceParameterSpecsTest, ValidateAndBuildCommand_EnforcesRange)
{
  const auto * spec = findWritableParameterSpec("auto_left");
  ASSERT_NE(spec, nullptr);

  std::string command;
  std::string reason;

  EXPECT_TRUE(spec->validateAndBuildCommand(
      rclcpp::Parameter("device.auto_left", 180), &command, &reason));
  EXPECT_EQ(command, "#DPAUTOLEFT180");

  EXPECT_FALSE(spec->validateAndBuildCommand(
      rclcpp::Parameter("device.auto_left", 181), &command, &reason));
  EXPECT_NE(reason.find("range"), std::string::npos);
}

TEST(DeviceParameterSpecsTest, FindParameterSpecByConfigKey_ResolvesKnownKey)
{
  const auto * spec = findParameterSpecByConfigKey("AutoMode");
  ASSERT_NE(spec, nullptr);
  EXPECT_EQ(spec->fullName(), "device.auto_mode");

  const auto * missing = findParameterSpecByConfigKey("NotARealField");
  EXPECT_EQ(missing, nullptr);
}

TEST(DeviceParameterSpecsTest, ParseAndApplyConfigValue_UpdatesStructAndValidatesRange)
{
  DeviceConfiguration config;
  const auto logger = rclcpp::get_logger("device_parameter_specs_test");

  const auto * auto_left = findParameterSpecByConfigKey("AutoLeft");
  ASSERT_NE(auto_left, nullptr);
  EXPECT_TRUE(auto_left->parseAndApplyConfigValue("180", &config, logger));
  EXPECT_EQ(config.auto_left, static_cast<uint8_t>(180));

  EXPECT_FALSE(auto_left->parseAndApplyConfigValue("181", &config, logger));
  EXPECT_EQ(config.auto_left, static_cast<uint8_t>(180));

  const auto * auto_mode = findParameterSpecByConfigKey("AutoMode");
  ASSERT_NE(auto_mode, nullptr);
  EXPECT_TRUE(auto_mode->parseAndApplyConfigValue("true", &config, logger));
  EXPECT_TRUE(config.auto_mode);

  EXPECT_FALSE(auto_mode->parseAndApplyConfigValue("banana", &config, logger));
}

}  // namespace ros2_roamadome
