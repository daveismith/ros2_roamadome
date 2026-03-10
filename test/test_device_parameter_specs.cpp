#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "ros2_roamadome/device_parameter_specs.hpp"

namespace ros2_roamadome
{

TEST(DeviceParameterSpecsTest, GetAllParameterSpecs_ContainsExpectedCounts)
{
  const auto & specs = getAllParameterSpecs();
  EXPECT_EQ(specs.size(), 26U);

  size_t writable_count = 0;
  for (const auto * spec : specs) {
    if (spec->isWritable()) {
      writable_count++;
    }
  }

  EXPECT_EQ(writable_count, 8U);
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

}  // namespace ros2_roamadome
