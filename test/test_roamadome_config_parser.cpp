#include <gtest/gtest.h>

#include <chrono>
#include <map>
#include <string>

#include "ros2_roamadome/roamadome_config_parser.hpp"

namespace ros2_roamadome
{

TEST(ConfigurationParserTest, Parse_UnknownFieldIsIgnored)
{
  const std::map<std::string, std::string> config_map = {
    {"UnknownThing", "123"},
    {"AutoMode", "1"}
  };

  const auto logger = rclcpp::get_logger("config_parser_test");
  auto parsed = ConfigurationParser::parse(config_map, logger);

  ASSERT_TRUE(parsed.has_value());
  EXPECT_TRUE(parsed->auto_mode);
}

TEST(ConfigurationParserTest, Parse_InvalidValueKeepsDefault)
{
  const std::map<std::string, std::string> config_map = {
    {"AutoLeft", "181"},
    {"AutoMode", "false"}
  };

  const auto logger = rclcpp::get_logger("config_parser_test");
  auto parsed = ConfigurationParser::parse(config_map, logger);

  ASSERT_TRUE(parsed.has_value());
  EXPECT_EQ(parsed->auto_left, static_cast<uint8_t>(80));
  EXPECT_FALSE(parsed->auto_mode);
}

TEST(ConfigurationParserTest, Parse_SetsTimestamp)
{
  const auto before = std::chrono::steady_clock::now();

  const std::map<std::string, std::string> config_map = {
    {"AutoMode", "1"}
  };

  const auto logger = rclcpp::get_logger("config_parser_test");
  auto parsed = ConfigurationParser::parse(config_map, logger);

  ASSERT_TRUE(parsed.has_value());
  const auto after = std::chrono::steady_clock::now();

  EXPECT_GE(parsed->timestamp, before);
  EXPECT_LE(parsed->timestamp, after);
}

TEST(ConfigurationParserTest, Parse_AllDeviceConfigurationFieldsRoundTrip)
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
  auto parsed = ConfigurationParser::parse(config_map, logger);

  ASSERT_TRUE(parsed.has_value());
  const auto & cfg = parsed.value();

  EXPECT_EQ(cfg.home_pos, static_cast<uint16_t>(10));
  EXPECT_EQ(cfg.max_speed, static_cast<uint8_t>(99));
  EXPECT_EQ(cfg.min_speed, static_cast<uint8_t>(14));
  EXPECT_TRUE(cfg.auto_mode);
  EXPECT_FALSE(cfg.home_mode);
  EXPECT_EQ(cfg.input_speed, static_cast<uint8_t>(75));
  EXPECT_TRUE(cfg.scaling);
  EXPECT_FALSE(cfg.inverted);
  EXPECT_EQ(cfg.timeout, static_cast<uint8_t>(30));
  EXPECT_TRUE(cfg.auto_safety);
  EXPECT_FALSE(cfg.auto_restart);
  EXPECT_EQ(cfg.acceleration_scale, static_cast<uint8_t>(21));
  EXPECT_EQ(cfg.deceleration_scale, static_cast<uint8_t>(42));
  EXPECT_EQ(cfg.home_min_delay, static_cast<uint8_t>(3));
  EXPECT_EQ(cfg.home_max_delay, static_cast<uint8_t>(4));
  EXPECT_EQ(cfg.auto_min_delay, static_cast<uint8_t>(8));
  EXPECT_EQ(cfg.auto_max_delay, static_cast<uint8_t>(9));
  EXPECT_EQ(cfg.target_min_delay, static_cast<uint8_t>(5));
  EXPECT_EQ(cfg.target_max_delay, static_cast<uint8_t>(6));
  EXPECT_EQ(cfg.setup_angular_velocity, static_cast<uint16_t>(111));
  EXPECT_EQ(cfg.auto_left, static_cast<uint8_t>(90));
  EXPECT_EQ(cfg.auto_right, static_cast<uint8_t>(91));
  EXPECT_EQ(cfg.fudge, static_cast<uint8_t>(7));
  EXPECT_EQ(cfg.speed_home, static_cast<uint8_t>(60));
  EXPECT_EQ(cfg.speed_auto, static_cast<uint8_t>(61));
  EXPECT_EQ(cfg.speed_target, static_cast<uint8_t>(62));
  EXPECT_EQ(cfg.syren_address_in, static_cast<uint8_t>(130));
  EXPECT_EQ(cfg.syren_address_out, static_cast<uint8_t>(131));
  EXPECT_EQ(cfg.sensor_baud, static_cast<uint32_t>(115200));
  EXPECT_EQ(cfg.syren_baud, static_cast<uint32_t>(38400));
  EXPECT_EQ(cfg.serial_baud, static_cast<uint32_t>(19200));
  EXPECT_FALSE(cfg.serial_in);
  EXPECT_TRUE(cfg.serial_out);
  EXPECT_TRUE(cfg.pwm_in);
  EXPECT_FALSE(cfg.pwm_out);
  EXPECT_EQ(cfg.pwm_min_pulse, static_cast<uint16_t>(1100));
  EXPECT_EQ(cfg.pwm_max_pulse, static_cast<uint16_t>(1900));
  EXPECT_EQ(cfg.pwm_neutral_pulse, static_cast<uint16_t>(1501));
  EXPECT_EQ(cfg.pwm_deadband, static_cast<uint8_t>(8));
  EXPECT_TRUE(cfg.pwm_arc_mode);
  EXPECT_EQ(cfg.digital_out, static_cast<uint8_t>(12));
}

}  // namespace ros2_roamadome
