#include <gtest/gtest.h>

#include <chrono>
#include <map>
#include <string>

#include "ros2_roamadome/device_parameter_specs.hpp"

namespace ros2_roamadome
{

const DeviceParameterSpecBase * findSpecByConfigKey(
  const DeviceParameterRegistry & registry,
  const std::string & config_key)
{
  return registry.findParameterSpecByConfigKey(config_key);
}

TEST(DeviceParameterRegistryParseTest, Parse_UnknownFieldIsIgnored)
{
  const std::map<std::string, std::string> config_map = {
    {"UnknownThing", "123"},
    {"AutoMode", "1"}
  };

  const auto logger = rclcpp::get_logger("config_parser_test");
  DeviceParameterRegistry registry;
  const bool parsed = registry.parseConfigMap(config_map, logger);

  EXPECT_TRUE(parsed);
  const auto * auto_mode = findSpecByConfigKey(registry, "AutoMode");
  ASSERT_NE(auto_mode, nullptr);
  EXPECT_TRUE(auto_mode->currentParameter().as_bool());
}

TEST(DeviceParameterRegistryParseTest, Parse_InvalidValueKeepsDefault)
{
  const std::map<std::string, std::string> config_map = {
    {"AutoLeft", "181"},
    {"AutoMode", "false"}
  };

  const auto logger = rclcpp::get_logger("config_parser_test");
  DeviceParameterRegistry registry;
  const bool parsed = registry.parseConfigMap(config_map, logger);

  EXPECT_FALSE(parsed);
  const auto * auto_left = findSpecByConfigKey(registry, "AutoLeft");
  const auto * auto_mode = findSpecByConfigKey(registry, "AutoMode");
  ASSERT_NE(auto_left, nullptr);
  ASSERT_NE(auto_mode, nullptr);
  EXPECT_EQ(auto_left->currentParameter().as_int(), 80);
  EXPECT_FALSE(auto_mode->currentParameter().as_bool());
}

TEST(DeviceParameterRegistryParseTest, Parse_ResetsMissingFieldsToDefaults)
{
  const std::map<std::string, std::string> config_map = {
    {"AutoMode", "1"},
    {"AutoLeft", "90"}
  };

  const auto logger = rclcpp::get_logger("config_parser_test");
  DeviceParameterRegistry registry;

  ASSERT_TRUE(registry.parseConfigMap(config_map, logger));

  const std::map<std::string, std::string> partial_update = {
    {"AutoMode", "0"}
  };

  ASSERT_TRUE(registry.parseConfigMap(partial_update, logger));

  const auto * auto_mode = findSpecByConfigKey(registry, "AutoMode");
  const auto * auto_left = findSpecByConfigKey(registry, "AutoLeft");
  ASSERT_NE(auto_mode, nullptr);
  ASSERT_NE(auto_left, nullptr);

  EXPECT_FALSE(auto_mode->currentParameter().as_bool());
  EXPECT_EQ(auto_left->currentParameter().as_int(), 80);
}

TEST(DeviceParameterRegistryParseTest, Parse_AllConfigFieldsRoundTripIntoRegistry)
{
  const std::map<std::string, std::string> config_map = {
    {"HomePos", "10"},
    {"MaxSpeed", "99"},
    {"MinSpeed", "14"},
    {"AutoMode", "1"},
    {"HomeMode", "0"},
    {"InputSpeed", "75"},
    {"Scaling", "1"},
    {"Inverted", "0"},
    {"Timeout", "30"},
    {"AutoSafety", "1"},
    {"AutoRestart", "0"},
    {"AccelerationScale", "21"},
    {"DecelerationScale", "42"},
    {"HomeMinDelay", "3"},
    {"HomeMaxDelay", "4"},
    {"AutoMinDelay", "8"},
    {"AutoMaxDelay", "9"},
    {"TargetMinDelay", "5"},
    {"TargetMaxDelay", "6"},
    {"SetupAngularVelocity", "111"},
    {"AutoLeft", "90"},
    {"AutoRight", "91"},
    {"Fudge", "7"},
    {"SpeedHome", "60"},
    {"SpeedAuto", "61"},
    {"SpeedTarget", "62"},
    {"SyrenAddressIn", "130"},
    {"SyrenAddressOut", "131"},
    {"SensorBaud", "115200"},
    {"SyrenBaud", "38400"},
    {"SerialBaud", "19200"},
    {"SerialIn", "0"},
    {"SerialOut", "1"},
    {"PWMIn", "1"},
    {"PWMOut", "0"},
    {"PWMMinPulse", "1100"},
    {"PWMMaxPulse", "1900"},
    {"PWMNeutralPulse", "1501"},
    {"PWMDeadband", "8"},
    {"PWMArcMode", "1"},
    {"DOut", "12"}
  };

  const auto logger = rclcpp::get_logger("config_parser_test");
  DeviceParameterRegistry registry;
  ASSERT_TRUE(registry.parseConfigMap(config_map, logger));

  EXPECT_EQ(findSpecByConfigKey(registry, "HomePos")->currentParameter().as_int(), 10);
  EXPECT_EQ(findSpecByConfigKey(registry, "MaxSpeed")->currentParameter().as_int(), 99);
  EXPECT_EQ(findSpecByConfigKey(registry, "MinSpeed")->currentParameter().as_int(), 14);
  EXPECT_TRUE(findSpecByConfigKey(registry, "AutoMode")->currentParameter().as_bool());
  EXPECT_FALSE(findSpecByConfigKey(registry, "HomeMode")->currentParameter().as_bool());
  EXPECT_EQ(findSpecByConfigKey(registry, "InputSpeed")->currentParameter().as_int(), 75);
  EXPECT_TRUE(findSpecByConfigKey(registry, "Scaling")->currentParameter().as_bool());
  EXPECT_FALSE(findSpecByConfigKey(registry, "Inverted")->currentParameter().as_bool());
  EXPECT_EQ(findSpecByConfigKey(registry, "Timeout")->currentParameter().as_int(), 30);
  EXPECT_TRUE(findSpecByConfigKey(registry, "AutoSafety")->currentParameter().as_bool());
  EXPECT_FALSE(findSpecByConfigKey(registry, "AutoRestart")->currentParameter().as_bool());
  EXPECT_EQ(findSpecByConfigKey(registry, "AccelerationScale")->currentParameter().as_int(), 21);
  EXPECT_EQ(findSpecByConfigKey(registry, "DecelerationScale")->currentParameter().as_int(), 42);
  EXPECT_EQ(findSpecByConfigKey(registry, "HomeMinDelay")->currentParameter().as_int(), 3);
  EXPECT_EQ(findSpecByConfigKey(registry, "HomeMaxDelay")->currentParameter().as_int(), 4);
  EXPECT_EQ(findSpecByConfigKey(registry, "AutoMinDelay")->currentParameter().as_int(), 8);
  EXPECT_EQ(findSpecByConfigKey(registry, "AutoMaxDelay")->currentParameter().as_int(), 9);
  EXPECT_EQ(findSpecByConfigKey(registry, "TargetMinDelay")->currentParameter().as_int(), 5);
  EXPECT_EQ(findSpecByConfigKey(registry, "TargetMaxDelay")->currentParameter().as_int(), 6);
  EXPECT_EQ(findSpecByConfigKey(registry, "SetupAngularVelocity")->currentParameter().as_int(),
      111);
  EXPECT_EQ(findSpecByConfigKey(registry, "AutoLeft")->currentParameter().as_int(), 90);
  EXPECT_EQ(findSpecByConfigKey(registry, "AutoRight")->currentParameter().as_int(), 91);
  EXPECT_EQ(findSpecByConfigKey(registry, "Fudge")->currentParameter().as_int(), 7);
  EXPECT_EQ(findSpecByConfigKey(registry, "SpeedHome")->currentParameter().as_int(), 60);
  EXPECT_EQ(findSpecByConfigKey(registry, "SpeedAuto")->currentParameter().as_int(), 61);
  EXPECT_EQ(findSpecByConfigKey(registry, "SpeedTarget")->currentParameter().as_int(), 62);
  EXPECT_EQ(findSpecByConfigKey(registry, "SyrenAddressIn")->currentParameter().as_int(), 130);
  EXPECT_EQ(findSpecByConfigKey(registry, "SyrenAddressOut")->currentParameter().as_int(), 131);
  EXPECT_EQ(findSpecByConfigKey(registry, "SensorBaud")->currentParameter().as_int(), 115200);
  EXPECT_EQ(findSpecByConfigKey(registry, "SyrenBaud")->currentParameter().as_int(), 38400);
  EXPECT_EQ(findSpecByConfigKey(registry, "SerialBaud")->currentParameter().as_int(), 19200);
  EXPECT_FALSE(findSpecByConfigKey(registry, "SerialIn")->currentParameter().as_bool());
  EXPECT_TRUE(findSpecByConfigKey(registry, "SerialOut")->currentParameter().as_bool());
  EXPECT_TRUE(findSpecByConfigKey(registry, "PWMIn")->currentParameter().as_bool());
  EXPECT_FALSE(findSpecByConfigKey(registry, "PWMOut")->currentParameter().as_bool());
  EXPECT_EQ(findSpecByConfigKey(registry, "PWMMinPulse")->currentParameter().as_int(), 1100);
  EXPECT_EQ(findSpecByConfigKey(registry, "PWMMaxPulse")->currentParameter().as_int(), 1900);
  EXPECT_EQ(findSpecByConfigKey(registry, "PWMNeutralPulse")->currentParameter().as_int(), 1501);
  EXPECT_EQ(findSpecByConfigKey(registry, "PWMDeadband")->currentParameter().as_int(), 8);
  EXPECT_TRUE(findSpecByConfigKey(registry, "PWMArcMode")->currentParameter().as_bool());
  EXPECT_EQ(findSpecByConfigKey(registry, "DOut")->currentParameter().as_int(), 12);
}

}  // namespace ros2_roamadome
