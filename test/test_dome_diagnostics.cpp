#include <gtest/gtest.h>
#include "ros2_roamadome/dome_diagnostics.hpp"

#include <chrono>
#include <string>
#include <vector>

namespace ros2_roamadome
{

namespace
{

using diagnostic_msgs::msg::DiagnosticStatus;
using TimePoint = DomeDiagnosticsState::TimePoint;

TimePoint at(double seconds)
{
  return TimePoint(
    std::chrono::duration_cast<DomeDiagnosticsState::Clock::duration>(
      std::chrono::duration<double>(seconds)));
}

const DiagnosticStatus & find(const std::vector<DiagnosticStatus> & statuses, const char * name)
{
  for (const auto & status : statuses) {
    if (status.name == name) {
      return status;
    }
  }
  throw std::runtime_error(std::string("status not found: ") + name);
}

std::string value(const DiagnosticStatus & status, const char * key)
{
  for (const auto & kv : status.values) {
    if (kv.key == key) {
      return kv.value;
    }
  }
  throw std::runtime_error(std::string("key not found: ") + key);
}

DomeDiagnosticsReadings healthyReadings()
{
  DomeDiagnosticsReadings readings;
  readings.hardware_id = "/dev/ttyACM0";
  readings.serial_open = true;
  readings.position_rad = 1.0;
  readings.velocity_rad_s = 0.0;
  readings.commanded_percent = 0;
  readings.command_mode = 0;
  return readings;
}

}  // namespace

TEST(DomeDiagnostics, ThreeStatusesWithStableNames)
{
  DomeDiagnosticsState state;
  recordPositionReport(&state, at(10.0));
  const auto statuses = buildDomeDiagnostics(state, healthyReadings(), {}, at(10.1));
  ASSERT_EQ(statuses.size(), 3u);
  EXPECT_EQ(statuses[0].name, "roamadome: link");
  EXPECT_EQ(statuses[1].name, "roamadome: sensor");
  EXPECT_EQ(statuses[2].name, "roamadome: motion");
  for (const auto & status : statuses) {
    EXPECT_EQ(status.hardware_id, "/dev/ttyACM0");
    EXPECT_EQ(status.level, DiagnosticStatus::OK) << status.name << ": " << status.message;
  }
}

TEST(DomeDiagnostics, ReportGapsAreCountedAndReported)
{
  DomeDiagnosticsState state;
  EXPECT_DOUBLE_EQ(recordPositionReport(&state, at(0.0), 0.25), 0.0);
  EXPECT_NEAR(recordPositionReport(&state, at(0.033), 0.25), 0.033, 1e-6);
  EXPECT_EQ(state.report_gaps, 0u);
  EXPECT_NEAR(recordPositionReport(&state, at(0.4), 0.25), 0.367, 1e-6);
  EXPECT_EQ(state.report_gaps, 1u);
  EXPECT_NEAR(state.last_report_gap_s, 0.367, 1e-6);
  // Disabled threshold counts nothing.
  recordPositionReport(&state, at(2.0), 0.0);
  EXPECT_EQ(state.report_gaps, 1u);

  const auto statuses = buildDomeDiagnostics(state, healthyReadings(), {}, at(2.1));
  EXPECT_EQ(value(find(statuses, "roamadome: link"), "report_gaps"), "1");
  EXPECT_EQ(value(find(statuses, "roamadome: link"), "last_report_gap_s"), "0.367");
}

TEST(DomeDiagnostics, FlowingReportsProveReportingIsEnabled)
{
  // The startup #DPREPORT ack can be missed (or wiped by the activation reset);
  // a report that arrives is proof enough.
  DomeDiagnosticsState state;
  EXPECT_FALSE(state.position_reporting_enabled);
  recordPositionReport(&state, at(1.0));
  EXPECT_TRUE(state.position_reporting_enabled);
  auto statuses = buildDomeDiagnostics(state, healthyReadings(), {}, at(1.1));
  EXPECT_EQ(value(find(statuses, "roamadome: link"), "position_reporting_enabled"), "true");
  // An explicit "Position Reporting Disabled" still turns it off.
  applyFirmwareLine(&state, classifyFirmwareLine("Position Reporting Disabled"), at(2.0), true);
  EXPECT_FALSE(state.position_reporting_enabled);
}

TEST(DomeDiagnostics, SensorWarnsAfterARejectedReportThenClears)
{
  DomeDiagnosticsState state;
  recordPositionReport(&state, at(1.0));
  state.reports_held = 2;
  state.reports_rejected = 1;
  state.jumps_confirmed = 1;
  state.gate_resyncs = 4;
  state.last_report_rejected = at(1.0);

  auto statuses = buildDomeDiagnostics(state, healthyReadings(), {}, at(4.0));
  const auto & sensor = find(statuses, "roamadome: sensor");
  EXPECT_EQ(sensor.level, DiagnosticStatus::WARN);
  EXPECT_NE(sensor.message.find("implausible position report rejected"), std::string::npos);
  EXPECT_EQ(value(sensor, "reports_held"), "2");
  EXPECT_EQ(value(sensor, "reports_rejected"), "1");
  EXPECT_EQ(value(sensor, "jumps_confirmed"), "1");
  EXPECT_EQ(value(sensor, "gate_resyncs"), "4");
  EXPECT_EQ(value(sensor, "last_rejection_age_s"), "3.0");

  statuses = buildDomeDiagnostics(state, healthyReadings(), {}, at(12.0));
  EXPECT_EQ(find(statuses, "roamadome: sensor").level, DiagnosticStatus::OK);

  // Never rejected: no warning, "never" age.
  DomeDiagnosticsState clean;
  statuses = buildDomeDiagnostics(clean, healthyReadings(), {}, at(1.0));
  EXPECT_EQ(value(find(statuses, "roamadome: sensor"), "last_rejection_age_s"), "never");
}

TEST(DomeDiagnostics, LinkWarnsWithoutReportsAndWhenStale)
{
  DomeDiagnosticsState state;
  auto statuses = buildDomeDiagnostics(state, healthyReadings(), {}, at(1.0));
  EXPECT_EQ(find(statuses, "roamadome: link").level, DiagnosticStatus::WARN);

  recordPositionReport(&state, at(1.0));
  statuses = buildDomeDiagnostics(state, healthyReadings(), {}, at(1.2));
  EXPECT_EQ(find(statuses, "roamadome: link").level, DiagnosticStatus::OK);

  statuses = buildDomeDiagnostics(state, healthyReadings(), {}, at(2.0));
  EXPECT_EQ(find(statuses, "roamadome: link").level, DiagnosticStatus::WARN);
  EXPECT_EQ(value(find(statuses, "roamadome: link"), "position_reports"), "1");
}

TEST(DomeDiagnostics, LinkErrorsWhenSerialClosed)
{
  DomeDiagnosticsState state;
  recordPositionReport(&state, at(1.0));
  auto readings = healthyReadings();
  readings.serial_open = false;
  const auto statuses = buildDomeDiagnostics(state, readings, {}, at(1.1));
  EXPECT_EQ(find(statuses, "roamadome: link").level, DiagnosticStatus::ERROR);
}

TEST(DomeDiagnostics, ReadyWhileActiveIsAReboot)
{
  DomeDiagnosticsState state;
  recordPositionReport(&state, at(1.0));

  // READY while inactive (boot during configure) is not a reboot.
  applyFirmwareLine(&state, classifyFirmwareLine("READY"), at(1.0), false);
  EXPECT_FALSE(state.reboot_detected);

  applyFirmwareLine(&state, classifyFirmwareLine("READY"), at(2.0), true);
  EXPECT_TRUE(state.reboot_detected);
  const auto statuses = buildDomeDiagnostics(state, healthyReadings(), {}, at(2.1));
  // Reports are stale too, but the reboot is the more severe condition.
  EXPECT_EQ(find(statuses, "roamadome: link").level, DiagnosticStatus::ERROR);
  EXPECT_NE(find(statuses, "roamadome: link").message.find("rebooted"), std::string::npos);
  EXPECT_EQ(value(find(statuses, "roamadome: link"), "reboot_detected"), "true");
}

TEST(DomeDiagnostics, PositionTimeoutIsErrorThenWarnUntilSuccessfulMove)
{
  DomeDiagnosticsState state;
  recordPositionReport(&state, at(100.0));
  applyFirmwareLine(&state, classifyFirmwareLine("ERROR: DOME POSITION TIMEOUT"), at(100.0), true);
  EXPECT_EQ(state.position_timeouts, 1u);

  auto statuses = buildDomeDiagnostics(state, healthyReadings(), {}, at(100.1));
  EXPECT_EQ(find(statuses, "roamadome: motion").level, DiagnosticStatus::ERROR);

  recordPositionReport(&state, at(104.9));
  statuses = buildDomeDiagnostics(state, healthyReadings(), {}, at(104.9));
  EXPECT_EQ(find(statuses, "roamadome: motion").level, DiagnosticStatus::ERROR);

  recordPositionReport(&state, at(105.1));
  statuses = buildDomeDiagnostics(state, healthyReadings(), {}, at(105.1));
  EXPECT_EQ(find(statuses, "roamadome: motion").level, DiagnosticStatus::WARN);
  EXPECT_EQ(value(find(statuses, "roamadome: motion"), "position_timeouts"), "1");

  applyFirmwareLine(&state, classifyFirmwareLine("REACHED TARGET: 90"), at(110.0), true);
  recordPositionReport(&state, at(110.0));
  statuses = buildDomeDiagnostics(state, healthyReadings(), {}, at(110.0));
  EXPECT_EQ(find(statuses, "roamadome: motion").level, DiagnosticStatus::OK);
  EXPECT_EQ(value(find(statuses, "roamadome: motion"), "last_target_reached_deg"), "90");
}

TEST(DomeDiagnostics, ReachedHomeAlsoClearsTimeoutWarning)
{
  DomeDiagnosticsState state;
  applyFirmwareLine(&state, classifyFirmwareLine("ERROR: DOME POSITION TIMEOUT"), at(0.0), true);
  applyFirmwareLine(&state, classifyFirmwareLine("REACHED HOME: 0"), at(1.0), true);
  EXPECT_TRUE(state.move_succeeded_since_timeout);
  EXPECT_EQ(*state.last_home_reached_deg, 0);
}

TEST(DomeDiagnostics, SensorErrorIncreaseWarnsUntilNextSteadyPoll)
{
  DomeDiagnosticsState state;
  recordPositionReport(&state, at(0.0));

  const std::vector<std::string> first = {"Auto Safety Disengaged", "Dome Sensor Errors: 2"};
  applyStatusSection(&state, first, at(0.0));
  EXPECT_EQ(*state.sensor_errors, 2u);
  EXPECT_EQ(*state.packet_serial_errors, 0u);
  EXPECT_FALSE(state.sensor_errors_increased);
  auto statuses = buildDomeDiagnostics(state, healthyReadings(), {}, at(0.1));
  EXPECT_EQ(find(statuses, "roamadome: sensor").level, DiagnosticStatus::OK);

  const std::vector<std::string> second = {
    "Auto Safety Disengaged", "Packet Serial Errors: 1", "Dome Sensor Errors: 5"};
  applyStatusSection(&state, second, at(10.0));
  EXPECT_TRUE(state.sensor_errors_increased);
  EXPECT_EQ(*state.packet_serial_errors, 1u);
  recordPositionReport(&state, at(10.0));
  statuses = buildDomeDiagnostics(state, healthyReadings(), {}, at(10.1));
  EXPECT_EQ(find(statuses, "roamadome: sensor").level, DiagnosticStatus::WARN);
  EXPECT_EQ(value(find(statuses, "roamadome: sensor"), "sensor_errors"), "5");

  const std::vector<std::string> third = {"Auto Safety Disengaged", "Dome Sensor Errors: 5"};
  applyStatusSection(&state, third, at(20.0));
  EXPECT_FALSE(state.sensor_errors_increased);
  recordPositionReport(&state, at(20.0));
  statuses = buildDomeDiagnostics(state, healthyReadings(), {}, at(20.1));
  EXPECT_EQ(find(statuses, "roamadome: sensor").level, DiagnosticStatus::OK);
  EXPECT_EQ(state.status_polls_answered, 3u);
}

TEST(DomeDiagnostics, StatusSectionWithoutSensorLineIsIgnored)
{
  DomeDiagnosticsState state;
  applyStatusSection(&state, {"Auto Safety Engaged", "WiFi Disabled"}, at(0.0));
  EXPECT_FALSE(state.sensor_errors.has_value());
  EXPECT_EQ(state.status_polls_answered, 0u);
}

TEST(DomeDiagnostics, NoSensorErrorsLineReadsAsZero)
{
  DomeDiagnosticsState state;
  applyStatusSection(&state, {"No Dome Sensor Errors"}, at(0.0));
  EXPECT_EQ(*state.sensor_errors, 0u);
}

TEST(DomeDiagnostics, MotionReportsSettingsWritePipeline)
{
  DomeDiagnosticsState state;
  DomeDiagnosticsReadings readings = healthyReadings();
  auto motion = find(buildDomeDiagnostics(state, readings, DomeDiagnosticsOptions(), at(1.0)),
      "roamadome: motion");
  EXPECT_EQ(value(motion, "settings_writes_pending"), "0");
  EXPECT_EQ(value(motion, "settings_acks"), "0");
  EXPECT_EQ(value(motion, "settings_ack_timeouts"), "0");

  state.settings_acks = 3;
  state.settings_ack_timeouts = 1;
  readings.settings_writes_pending = 2;
  motion = find(buildDomeDiagnostics(state, readings, DomeDiagnosticsOptions(), at(1.0)),
      "roamadome: motion");
  EXPECT_EQ(value(motion, "settings_writes_pending"), "2");
  EXPECT_EQ(value(motion, "settings_acks"), "3");
  EXPECT_EQ(value(motion, "settings_ack_timeouts"), "1");
  // The counters are informational: they never change the level.
  EXPECT_EQ(motion.level, DiagnosticStatus::OK);
}

TEST(DomeDiagnostics, MotionReportsTheEchoedMoveSpeed)
{
  DomeDiagnosticsState state;
  applyFirmwareLine(&state, classifyFirmwareLine("ROTARY DEGREE: 90 speed: 0.40"), at(1.0), true);
  const auto statuses = buildDomeDiagnostics(state, healthyReadings(), {}, at(1.1));
  EXPECT_EQ(value(find(statuses, "roamadome: motion"), "last_rotary_target_deg"), "90");
  EXPECT_EQ(value(find(statuses, "roamadome: motion"), "last_rotary_speed"), "0.4");
  EXPECT_EQ(value(find(buildDomeDiagnostics(DomeDiagnosticsState(), healthyReadings(), {},
    at(1.0)), "roamadome: motion"), "last_rotary_speed"), "unknown");
}

TEST(DomeDiagnostics, MotionValuesTrackAcceptedCommands)
{
  DomeDiagnosticsState state;
  applyFirmwareLine(&state, classifyFirmwareLine("ROTATE SPEED: 48"), at(0.0), true);
  applyFirmwareLine(&state, classifyFirmwareLine("ROTARY DEGREE: 180 speed: 0.50"), at(1.0), true);
  applyFirmwareLine(&state, classifyFirmwareLine("Out of range."), at(2.0), true);
  EXPECT_EQ(*state.last_accepted_rotate_percent, 48);
  EXPECT_EQ(*state.last_rotary_target_deg, 180);
  EXPECT_DOUBLE_EQ(*state.last_rotary_speed, 0.5);
  EXPECT_EQ(state.out_of_range_rejections, 1u);

  auto readings = healthyReadings();
  readings.command_mode = 2;
  readings.commanded_percent = 48;
  const auto statuses = buildDomeDiagnostics(state, readings, {}, at(2.0));
  const auto & motion = find(statuses, "roamadome: motion");
  EXPECT_EQ(value(motion, "command_mode"), "velocity");
  EXPECT_EQ(value(motion, "commanded_percent"), "48");
  EXPECT_EQ(value(motion, "last_accepted_rotate_percent"), "48");
  EXPECT_EQ(value(motion, "out_of_range_rejections"), "1");
}

}  // namespace ros2_roamadome

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
