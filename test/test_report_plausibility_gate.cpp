// Copyright (c) 2026
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>
#include "ros2_roamadome/report_plausibility_gate.hpp"

#include <chrono>
#include <cstdint>
#include <vector>

namespace ros2_roamadome
{

namespace
{

using Gate = ReportPlausibilityGate;

Gate::TimePoint at(double seconds)
{
  return Gate::TimePoint(
    std::chrono::duration_cast<Gate::Clock::duration>(std::chrono::duration<double>(seconds)));
}

std::vector<uint16_t> accepted(const Gate::Decision & d)
{
  std::vector<uint16_t> out;
  for (std::size_t i = 0; i < d.accepted_count; ++i) {
    out.push_back(d.accepted[i].degrees);
  }
  return out;
}

// 360 deg/s, 28 ms report interval: 12 deg allowed per report, 22 across 0.
Gate gateOf360() {return Gate(360.0, 0.028);}

}  // namespace

TEST(ReportPlausibilityGate, DisabledAcceptsEverythingAtOnce)
{
  Gate gate(0.0, 0.028);
  EXPECT_FALSE(gate.enabled());
  EXPECT_EQ(accepted(gate.update(at(0.0), 10)), std::vector<uint16_t>{10});
  EXPECT_EQ(accepted(gate.update(at(0.028), 200)), std::vector<uint16_t>{200});
  EXPECT_EQ(gate.heldCount(), 0u);
}

TEST(ReportPlausibilityGate, PlausibleStepsPassStraightThrough)
{
  Gate gate = gateOf360();
  double t = 0.0;
  for (uint16_t deg : {100, 110, 121, 131, 140}) {
    const auto d = gate.update(at(t), deg);
    EXPECT_EQ(accepted(d), std::vector<uint16_t>{deg});
    EXPECT_FALSE(d.held);
    t += 0.028;
  }
  EXPECT_EQ(gate.heldCount(), 0u);
  EXPECT_EQ(gate.rejectedCount(), 0u);
}

TEST(ReportPlausibilityGate, SingleGlitchIsHeldThenRejected)
{
  Gate gate = gateOf360();
  gate.update(at(0.000), 100);
  gate.update(at(0.028), 110);
  // A sensor misread 40 deg off the track.
  auto d = gate.update(at(0.056), 160);
  EXPECT_TRUE(d.held);
  EXPECT_EQ(d.accepted_count, 0u);
  EXPECT_TRUE(gate.holding());
  // Back on the track: the held report is dropped, the new one accepted.
  d = gate.update(at(0.084), 130);
  EXPECT_TRUE(d.rejected);
  EXPECT_EQ(d.dropped.degrees, 160);
  EXPECT_EQ(accepted(d), std::vector<uint16_t>{130});
  EXPECT_FALSE(gate.holding());
  EXPECT_EQ(gate.heldCount(), 1u);
  EXPECT_EQ(gate.rejectedCount(), 1u);
  EXPECT_EQ(gate.confirmedCount(), 0u);
}

TEST(ReportPlausibilityGate, RealJumpIsConfirmedByTheNextReportInOrder)
{
  Gate gate = gateOf360();
  gate.update(at(0.000), 100);
  auto d = gate.update(at(0.028), 200);  // e.g. home moved, or a genuine fast slip
  EXPECT_TRUE(d.held);
  d = gate.update(at(0.056), 205);
  EXPECT_TRUE(d.confirmed);
  EXPECT_FALSE(d.rejected);
  EXPECT_EQ(accepted(d), (std::vector<uint16_t>{200, 205}));
  EXPECT_EQ(d.accepted[0].time, at(0.028));  // the held report keeps its own time
  EXPECT_EQ(gate.confirmedCount(), 1u);
  EXPECT_EQ(gate.rejectedCount(), 0u);
}

TEST(ReportPlausibilityGate, AmbiguousNextReportPrefersTheAcceptedHistory)
{
  // Held 30 deg ahead; the next report, 15 deg ahead, is plausible from both
  // the last accepted report and the held one: keep continuity, drop the held one.
  Gate gate(720.0, 0.028);  // 22 deg allowed per report
  gate.update(at(0.000), 0);
  auto d = gate.update(at(0.028), 30);
  ASSERT_TRUE(d.held);
  d = gate.update(at(0.056), 15);
  EXPECT_TRUE(d.rejected);
  EXPECT_EQ(accepted(d), std::vector<uint16_t>{15});
}

TEST(ReportPlausibilityGate, TwoDisagreeingGlitchesHoldTheSecond)
{
  Gate gate = gateOf360();
  gate.update(at(0.000), 100);
  EXPECT_TRUE(gate.update(at(0.028), 160).held);
  auto d = gate.update(at(0.056), 250);   // agrees with nothing
  EXPECT_TRUE(d.rejected);
  EXPECT_EQ(d.dropped.degrees, 160);
  EXPECT_TRUE(d.held);
  EXPECT_EQ(d.accepted_count, 0u);
  d = gate.update(at(0.084), 103);        // back on the track
  EXPECT_TRUE(d.rejected);
  EXPECT_EQ(d.dropped.degrees, 250);
  EXPECT_EQ(accepted(d), std::vector<uint16_t>{103});
  EXPECT_EQ(gate.rejectedCount(), 2u);
}

TEST(ReportPlausibilityGate, SameCycleStampsUseTheReportInterval)
{
  // Two reports read in one controller cycle are stamped 60 us apart although the
  // firmware sent them 28 ms apart: a normal 11 deg step must not be held.
  Gate gate = gateOf360();
  gate.update(at(0.000), 100);
  const auto d = gate.update(at(0.00006), 111);
  EXPECT_FALSE(d.held);
  EXPECT_EQ(accepted(d), std::vector<uint16_t>{111});
}

TEST(ReportPlausibilityGate, LongerIntervalsAllowLongerSteps)
{
  Gate gate = gateOf360();
  gate.update(at(0.0), 100);
  // 0.2 s at 360 deg/s is 72 deg plus the slack.
  EXPECT_FALSE(gate.update(at(0.2), 170).held);
  EXPECT_TRUE(gate.update(at(0.4), 260).held);   // 90 > 74
}

TEST(ReportPlausibilityGate, CrossingZeroGetsTheWrapAllowance)
{
  // The controller never reports 359..2 deg, so the step across 0 is about
  // 9 deg longer than elsewhere. 355 -> 17 is 22 deg: held at 360 deg/s
  // without the allowance, accepted with it.
  Gate gate = gateOf360();
  gate.update(at(0.000), 345);
  EXPECT_FALSE(gate.update(at(0.028), 355).held);
  EXPECT_FALSE(gate.update(at(0.056), 17).held);
  EXPECT_FALSE(gate.update(at(0.084), 28).held);
  // The same 22 deg step away from 0 is held.
  Gate other = gateOf360();
  other.update(at(0.000), 100);
  EXPECT_TRUE(other.update(at(0.028), 122).held);
  // And the other way round across 0.
  Gate down = gateOf360();
  down.update(at(0.000), 4);
  EXPECT_FALSE(down.update(at(0.028), 342).held);
}

TEST(ReportPlausibilityGate, CrossesZeroHelper)
{
  EXPECT_TRUE(Gate::crossesZero(355, 22));
  EXPECT_TRUE(Gate::crossesZero(4, -22));
  EXPECT_FALSE(Gate::crossesZero(100, 22));
  EXPECT_FALSE(Gate::crossesZero(0, 5));
  EXPECT_TRUE(Gate::crossesZero(0, -1));
  EXPECT_TRUE(Gate::crossesZero(359, 1));
}

TEST(ReportPlausibilityGate, ResetForgetsHistoryButKeepsCounters)
{
  Gate gate = gateOf360();
  gate.update(at(0.000), 100);
  EXPECT_TRUE(gate.update(at(0.028), 200).held);
  gate.reset();
  EXPECT_FALSE(gate.holding());
  // First report after a reset is accepted whatever it is.
  const auto d = gate.update(at(1.0), 300);
  EXPECT_EQ(accepted(d), std::vector<uint16_t>{300});
  EXPECT_EQ(gate.heldCount(), 1u);
}

}  // namespace ros2_roamadome
