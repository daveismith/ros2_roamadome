#include <gtest/gtest.h>

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

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

  EXPECT_EQ(writable_count, 16U);
}

TEST(DeviceParameterSpecsTest, ConfigFields_HaveConfigKeyMappings)
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

TEST(DeviceParameterSpecsTest, SpeedSettings_AreWritableWithFirmwareCommandPrefixes)
{
  struct Case
  {
    const char * field;
    int64_t value;
    const char * expected;
  };
  const std::vector<Case> cases = {
    {"max_speed", 60, "#DPMAXSPEED60"},
    {"min_speed", 15, "#DPMINSPEED15"},
    {"speed_target", 100, "#DPTARGETSPEED100"},
    {"speed_home", 40, "#DPHOMESPEED40"},
    {"acceleration_scale", 255, "#DPASCALE255"},
    {"deceleration_scale", 0, "#DPDSCALE0"},
    {"timeout", 1, "#DPTIMEOUT1"},
    {"timeout", 30, "#DPTIMEOUT30"},
  };
  for (const auto & c : cases) {
    const auto * spec = findWritableParameterSpec(c.field);
    ASSERT_NE(spec, nullptr) << c.field;
    std::string command;
    std::string reason;
    EXPECT_TRUE(spec->validateAndBuildCommand(
        rclcpp::Parameter(std::string("device.") + c.field, c.value), &command, &reason))
      << c.field << ": " << reason;
    EXPECT_EQ(command, c.expected);
  }

  const auto * scaling = findWritableParameterSpec("scaling");
  ASSERT_NE(scaling, nullptr);
  std::string command;
  std::string reason;
  EXPECT_TRUE(scaling->validateAndBuildCommand(
      rclcpp::Parameter("device.scaling", true), &command, &reason));
  EXPECT_EQ(command, "#DPSCALE1");
  EXPECT_TRUE(scaling->validateAndBuildCommand(
      rclcpp::Parameter("device.scaling", false), &command, &reason));
  EXPECT_EQ(command, "#DPSCALE0");
  EXPECT_FALSE(scaling->validateAndBuildCommand(
      rclcpp::Parameter("device.scaling", 1), &command, &reason));
}

TEST(DeviceParameterSpecsTest, SpeedSettings_RejectOutOfRangeValues)
{
  struct Case
  {
    const char * field;
    int64_t value;
  };
  const std::vector<Case> cases = {
    {"max_speed", 101}, {"max_speed", -1}, {"min_speed", 101}, {"speed_target", 101},
    {"speed_home", 101}, {"acceleration_scale", 256}, {"deceleration_scale", -1},
    {"timeout", 0}, {"timeout", 31},
  };
  for (const auto & c : cases) {
    const auto * spec = findWritableParameterSpec(c.field);
    ASSERT_NE(spec, nullptr) << c.field;
    std::string command;
    std::string reason;
    EXPECT_FALSE(spec->validateAndBuildCommand(
        rclcpp::Parameter(std::string("device.") + c.field, c.value), &command, &reason))
      << c.field << "=" << c.value;
    EXPECT_FALSE(reason.empty());
  }
}

TEST(DeviceParameterSpecsTest, ProtectedSettings_StayReadOnly)
{
  for (const char * field : {"inverted", "home_pos", "auto_safety", "sensor_baud",
      "syren_baud", "serial_baud", "syren_address_in", "syren_address_out"})
  {
    EXPECT_EQ(findWritableParameterSpec(field), nullptr) << field;
  }
}

TEST(DeviceParameterSpecsTest, FindParameterSpecByConfigKey_ResolvesKnownKey)
{
  const auto * spec = findParameterSpecByConfigKey("AutoMode");
  ASSERT_NE(spec, nullptr);
  EXPECT_EQ(spec->fullName(), "device.auto_mode");

  const auto * missing = findParameterSpecByConfigKey("NotARealField");
  EXPECT_EQ(missing, nullptr);
}

TEST(DeviceParameterSpecsTest, ParseAndStoreConfigValue_UpdatesRegistryAndValidatesRange)
{
  DeviceParameterRegistry registry;
  const auto logger = rclcpp::get_logger("device_parameter_specs_test");

  auto * auto_left = const_cast<DeviceParameterSpecBase *>(
    registry.findParameterSpecByConfigKey("AutoLeft"));
  ASSERT_NE(auto_left, nullptr);
  EXPECT_TRUE(auto_left->parseAndStoreConfigValue("180", logger));
  EXPECT_EQ(auto_left->currentParameter().as_int(), 180);

  EXPECT_FALSE(auto_left->parseAndStoreConfigValue("181", logger));
  EXPECT_EQ(auto_left->currentParameter().as_int(), 180);

  auto * auto_mode = const_cast<DeviceParameterSpecBase *>(
    registry.findParameterSpecByConfigKey("AutoMode"));
  ASSERT_NE(auto_mode, nullptr);
  EXPECT_TRUE(auto_mode->parseAndStoreConfigValue("true", logger));
  EXPECT_TRUE(auto_mode->currentParameter().as_bool());

  EXPECT_FALSE(auto_mode->parseAndStoreConfigValue("banana", logger));
}

TEST(DeviceParameterSpecsTest, ReadOnlySpec_UsesWritableDescriptorForInternalSync)
{
  if (!rclcpp::ok()) {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }

  auto node = std::make_shared<rclcpp::Node>("read_only_spec_internal_sync_test");
  DeviceParameterRegistry registry;

  const auto * home_pos_spec = registry.findParameterSpecByConfigKey("HomePos");
  ASSERT_NE(home_pos_spec, nullptr);
  home_pos_spec->declareParameter(node);

  const auto descriptor = node->describe_parameter(home_pos_spec->fullName());
  EXPECT_FALSE(descriptor.read_only);

  rclcpp::Parameter initial_value;
  ASSERT_TRUE(node->get_parameter(home_pos_spec->fullName(), initial_value));
  ASSERT_EQ(initial_value.get_type(), rclcpp::ParameterType::PARAMETER_INTEGER);

  const int64_t new_value = initial_value.as_int() + 1;
  auto result = node->set_parameters({rclcpp::Parameter(home_pos_spec->fullName(), new_value)});
  ASSERT_EQ(result.size(), 1u);
  EXPECT_TRUE(result[0].successful);
}

}  // namespace ros2_roamadome
