#include <gtest/gtest.h>
#include "ros2_roamadome/firmware_line_classifier.hpp"

#include <string>

namespace ros2_roamadome
{

TEST(FirmwareLineClassifier, RotateSpeed)
{
  const auto line = classifyFirmwareLine("ROTATE SPEED: 25");
  EXPECT_EQ(line.kind, FirmwareLineKind::ROTATE_SPEED);
  ASSERT_TRUE(line.value.has_value());
  EXPECT_EQ(*line.value, 25);
  EXPECT_FALSE(line.speed.has_value());

  const auto negative = classifyFirmwareLine("ROTATE SPEED: -100\r");
  EXPECT_EQ(negative.kind, FirmwareLineKind::ROTATE_SPEED);
  EXPECT_EQ(*negative.value, -100);
}

TEST(FirmwareLineClassifier, RotaryDegreeWithSpeed)
{
  const auto line = classifyFirmwareLine("ROTARY DEGREE: 90 speed: 0.40");
  EXPECT_EQ(line.kind, FirmwareLineKind::ROTARY_DEGREE);
  ASSERT_TRUE(line.value.has_value());
  EXPECT_EQ(*line.value, 90);
  ASSERT_TRUE(line.speed.has_value());
  EXPECT_DOUBLE_EQ(*line.speed, 0.40);
}

TEST(FirmwareLineClassifier, ReachedTargetAndHome)
{
  const auto target = classifyFirmwareLine("REACHED TARGET: 270");
  EXPECT_EQ(target.kind, FirmwareLineKind::REACHED_TARGET);
  EXPECT_EQ(*target.value, 270);

  const auto home = classifyFirmwareLine("REACHED HOME: 0");
  EXPECT_EQ(home.kind, FirmwareLineKind::REACHED_HOME);
  EXPECT_EQ(*home.value, 0);
}

TEST(FirmwareLineClassifier, PositionTimeout)
{
  EXPECT_EQ(classifyFirmwareLine("ERROR: DOME POSITION TIMEOUT").kind,
    FirmwareLineKind::POSITION_TIMEOUT);
  EXPECT_EQ(classifyFirmwareLine("ERROR: DOME POSITION TIMEOUT\r\n").kind,
    FirmwareLineKind::POSITION_TIMEOUT);
}

TEST(FirmwareLineClassifier, StatusCounters)
{
  const auto sensor = classifyFirmwareLine("Dome Sensor Errors: 7");
  EXPECT_EQ(sensor.kind, FirmwareLineKind::SENSOR_ERRORS);
  EXPECT_EQ(*sensor.value, 7);

  const auto none = classifyFirmwareLine("No Dome Sensor Errors");
  EXPECT_EQ(none.kind, FirmwareLineKind::SENSOR_ERRORS);
  EXPECT_EQ(*none.value, 0);

  const auto packet = classifyFirmwareLine("Packet Serial Errors: 3");
  EXPECT_EQ(packet.kind, FirmwareLineKind::PACKET_SERIAL_ERRORS);
  EXPECT_EQ(*packet.value, 3);
}

TEST(FirmwareLineClassifier, PositionReportingAcks)
{
  EXPECT_EQ(classifyFirmwareLine("Position Reporting Enabled").kind,
    FirmwareLineKind::POSITION_REPORTING_ENABLED);
  EXPECT_EQ(classifyFirmwareLine("Position Reporting Disabled").kind,
    FirmwareLineKind::POSITION_REPORTING_DISABLED);
}

TEST(FirmwareLineClassifier, SetupLines)
{
  const auto speed = classifyFirmwareLine("SPEED: 40");
  EXPECT_EQ(speed.kind, FirmwareLineKind::SETUP_SPEED);
  EXPECT_EQ(*speed.value, 40);

  const auto velocity = classifyFirmwareLine("Angular velocity: 12.50 cm/s");
  EXPECT_EQ(velocity.kind, FirmwareLineKind::SETUP_ANGULAR_VELOCITY);
  ASSERT_TRUE(velocity.speed.has_value());
  EXPECT_DOUBLE_EQ(*velocity.speed, 12.5);

  const auto good = classifyFirmwareLine("GOOD MAX SPEED: 55");
  EXPECT_EQ(good.kind, FirmwareLineKind::SETUP_GOOD_MAX_SPEED);
  EXPECT_EQ(*good.value, 55);

  EXPECT_EQ(classifyFirmwareLine("AUTO SETUP FAILED").kind, FirmwareLineKind::SETUP_FAILED);
}

TEST(FirmwareLineClassifier, RejectionsAndNoOps)
{
  EXPECT_EQ(classifyFirmwareLine("Out of range.").kind, FirmwareLineKind::OUT_OF_RANGE);
  EXPECT_EQ(classifyFirmwareLine("Invalid").kind, FirmwareLineKind::INVALID);
  EXPECT_EQ(classifyFirmwareLine("Unchanged").kind, FirmwareLineKind::UNCHANGED);
}

TEST(FirmwareLineClassifier, Ready)
{
  EXPECT_EQ(classifyFirmwareLine("READY").kind, FirmwareLineKind::READY);
  EXPECT_EQ(classifyFirmwareLine("  READY \r").kind, FirmwareLineKind::READY);
}

TEST(FirmwareLineClassifier, UnknownLines)
{
  EXPECT_EQ(classifyFirmwareLine("").kind, FirmwareLineKind::UNKNOWN);
  EXPECT_EQ(classifyFirmwareLine("PROCESS: \"#DPINVALID\"").kind, FirmwareLineKind::UNKNOWN);
  EXPECT_EQ(classifyFirmwareLine("DOME POSITION: 12").kind, FirmwareLineKind::UNKNOWN);
  EXPECT_EQ(classifyFirmwareLine("Write Settings").kind, FirmwareLineKind::UNKNOWN);
  EXPECT_EQ(classifyFirmwareLine("Updated").kind, FirmwareLineKind::UNKNOWN);
  EXPECT_EQ(classifyFirmwareLine("Auto Safety Disengaged").kind, FirmwareLineKind::UNKNOWN);
  // Prefix-only matches are not accepted as different kinds.
  EXPECT_EQ(classifyFirmwareLine("READYish").kind, FirmwareLineKind::UNKNOWN);
  EXPECT_EQ(classifyFirmwareLine("Invalid command").kind, FirmwareLineKind::UNKNOWN);
}

TEST(FirmwareLineClassifier, MissingNumberLeavesValueEmpty)
{
  const auto line = classifyFirmwareLine("REACHED TARGET: ");
  EXPECT_EQ(line.kind, FirmwareLineKind::REACHED_TARGET);
  EXPECT_FALSE(line.value.has_value());
}

TEST(FirmwareLineClassifier, KindNamesAreStable)
{
  EXPECT_STREQ(firmwareLineKindName(FirmwareLineKind::POSITION_TIMEOUT), "position_timeout");
  EXPECT_STREQ(firmwareLineKindName(FirmwareLineKind::READY), "ready");
  EXPECT_STREQ(firmwareLineKindName(FirmwareLineKind::UNKNOWN), "unknown");
}

}  // namespace ros2_roamadome

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
