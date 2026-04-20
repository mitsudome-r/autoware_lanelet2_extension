// Copyright 2026 Autoware Foundation. All rights reserved.
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

// NOLINTBEGIN(readability-identifier-naming)

// Phase 5 signal-builder tests per §5.12 / §7 / §8: classification rules,
// physical-light + stop-line geometry, <validity> filtering, <signalReference>
// dedup, and attachment to the correct lanelets.

#include "../../lib/opendrive/lane_builder.hpp"
#include "../../lib/opendrive/signal_builder.hpp"
#include "../../lib/opendrive/xodr_types.hpp"

#include "autoware_lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp"

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>

#include <cmath>
#include <string>
#include <vector>

using lanelet::io_handlers::opendrive::GeomLine;
using lanelet::io_handlers::opendrive::Geometry;
using lanelet::io_handlers::opendrive::Lane;
using lanelet::io_handlers::opendrive::LaneBuilderOptions;
using lanelet::io_handlers::opendrive::LaneSection;
using lanelet::io_handlers::opendrive::Point3dDeduper;
using lanelet::io_handlers::opendrive::Poly3Record;
using lanelet::io_handlers::opendrive::Road;
using lanelet::io_handlers::opendrive::Signal;
using lanelet::io_handlers::opendrive::SignalBuilderOptions;
using lanelet::io_handlers::opendrive::SignalReference;
using lanelet::io_handlers::opendrive::SignalValidity;
using lanelet::io_handlers::opendrive::XodrDocument;
using lanelet::io_handlers::opendrive::buildRoadLanelets;
using lanelet::io_handlers::opendrive::buildSignals;

namespace
{
Lane makeLane(int id, const std::string & type, double width)
{
  Lane l;
  l.id = id;
  l.type = type;
  Poly3Record w;
  w.s = 0.0;
  w.a = width;
  l.widths.push_back(w);
  return l;
}

Signal makeLight(
  const std::string & id, double s, double t, const std::string & type = "1000001",
  const std::string & orientation = "+")
{
  Signal sig;
  sig.id = id;
  sig.s = s;
  sig.t = t;
  sig.dynamic = true;
  sig.orientation = orientation;
  sig.type = type;
  sig.subtype = "1";
  sig.country = "OpenDRIVE";
  sig.height = 3.0;
  sig.width = 0.5;
  sig.z_offset = 3.0;  // high above ground
  return sig;
}

Road makeStraightRoad(
  const std::string & id, double x0, double y0, double length,
  std::vector<Lane> right_lanes = {makeLane(-1, "driving", 3.5)},
  std::vector<Lane> left_lanes = {})
{
  Road road;
  road.id = id;
  road.length = length;
  road.junction = "-1";
  Geometry g;
  g.s = 0.0;
  g.x = x0;
  g.y = y0;
  g.hdg = 0.0;
  g.length = length;
  g.primitive = GeomLine{};
  road.plan_view.geometries.push_back(g);
  LaneSection ls;
  ls.s = 0.0;
  ls.right = std::move(right_lanes);
  ls.left = std::move(left_lanes);
  road.lanes.lane_sections.push_back(ls);
  return road;
}

// Build lanelets + run the signal builder in one shot. Returns the map.
struct Built
{
  lanelet::LaneletMap map;
  lanelet::ErrorMessages errors;
};

Built buildAll(
  const XodrDocument & doc, const SignalBuilderOptions & sig_opts = {},
  const LaneBuilderOptions & lane_opts = {})
{
  Built out;
  Point3dDeduper deduper{lane_opts.merge_tol_m};
  for (const auto & road : doc.roads) {
    buildRoadLanelets(road, lane_opts, deduper, out.map, out.errors);
  }
  buildSignals(doc, sig_opts, deduper, out.map, out.errors);
  return out;
}

lanelet::ConstLanelet laneletFor(
  const lanelet::LaneletMap & map, const std::string & road_id, int lane_id)
{
  for (const auto & ll : map.laneletLayer) {
    if (
      ll.attribute("opendrive:road_id").value() == road_id &&
      std::stoi(ll.attribute("opendrive:lane_id").value()) == lane_id) {
      return ll;
    }
  }
  return lanelet::ConstLanelet{};
}
}  // namespace

TEST(SignalBuilder, StaticSignalIsSilentlyIgnored)
{
  XodrDocument doc;
  auto road = makeStraightRoad("1", 0.0, 0.0, 30.0);
  Signal stop_sign;
  stop_sign.id = "10";
  stop_sign.s = 15.0;
  stop_sign.t = -6.0;
  stop_sign.dynamic = false;  // static
  stop_sign.orientation = "+";
  stop_sign.type = "206";
  road.signals.signals.push_back(stop_sign);
  doc.roads.push_back(road);

  const auto built = buildAll(doc);
  EXPECT_EQ(built.map.regulatoryElementLayer.size(), 0u);
  EXPECT_TRUE(built.errors.empty());
}

TEST(SignalBuilder, DynamicOutsideAllowlistEmitsWarningAndSkips)
{
  XodrDocument doc;
  auto road = makeStraightRoad("1", 0.0, 0.0, 30.0);
  auto sig = makeLight("10", 15.0, -6.0, /*type=*/"999999");
  road.signals.signals.push_back(sig);
  doc.roads.push_back(road);

  const auto built = buildAll(doc);
  EXPECT_EQ(built.map.regulatoryElementLayer.size(), 0u);
  ASSERT_EQ(built.errors.size(), 1u);
  EXPECT_NE(built.errors.front().find("not in traffic_light_types"), std::string::npos);
}

TEST(SignalBuilder, DynamicAllowlistedEmitsRegElement)
{
  XodrDocument doc;
  auto road = makeStraightRoad("1", 0.0, 0.0, 30.0);
  road.signals.signals.push_back(makeLight("10", 15.0, -6.0));
  doc.roads.push_back(road);

  const auto built = buildAll(doc);
  ASSERT_EQ(built.map.regulatoryElementLayer.size(), 1u);
  EXPECT_TRUE(built.errors.empty());
}

TEST(SignalBuilder, CustomAllowlistPromotesOtherTypes)
{
  XodrDocument doc;
  auto road = makeStraightRoad("1", 0.0, 0.0, 30.0);
  road.signals.signals.push_back(makeLight("10", 15.0, -6.0, /*type=*/"274"));
  doc.roads.push_back(road);

  SignalBuilderOptions opts;
  opts.traffic_light_types = {"274"};
  const auto built = buildAll(doc, opts);
  EXPECT_EQ(built.map.regulatoryElementLayer.size(), 1u);
  EXPECT_TRUE(built.errors.empty());
}

TEST(SignalBuilder, ZeroWidthFallbackAndWarning)
{
  XodrDocument doc;
  auto road = makeStraightRoad("1", 0.0, 0.0, 30.0);
  auto sig = makeLight("10", 15.0, -6.0);
  sig.width = 0.0;
  road.signals.signals.push_back(sig);
  doc.roads.push_back(road);

  const auto built = buildAll(doc);
  ASSERT_EQ(built.map.regulatoryElementLayer.size(), 1u);
  // Find the traffic-light linestring (type="traffic_light").
  const lanelet::ConstLineString3d * light = nullptr;
  for (const auto & ls : built.map.lineStringLayer) {
    if (ls.hasAttribute("type") && ls.attribute("type").value() == "traffic_light") {
      light = &ls;
      break;
    }
  }
  ASSERT_NE(light, nullptr);
  ASSERT_EQ(light->size(), 2u);
  const double d = std::hypot(
    (*light)[1].x() - (*light)[0].x(), (*light)[1].y() - (*light)[0].y());
  SignalBuilderOptions defaults;
  EXPECT_NEAR(d, defaults.fallback_light_width_m, 1e-6);

  bool saw_warning = false;
  for (const auto & e : built.errors) {
    if (e.find("width missing or zero") != std::string::npos) {
      saw_warning = true;
      break;
    }
  }
  EXPECT_TRUE(saw_warning);
}

TEST(SignalBuilder, OrientationNoneWarnsButStillEmits)
{
  XodrDocument doc;
  auto road = makeStraightRoad(
    "1", 0.0, 0.0, 30.0, {makeLane(-1, "driving", 3.5)}, {makeLane(1, "driving", 3.5)});
  road.signals.signals.push_back(makeLight("10", 15.0, 0.0, "1000001", /*orientation=*/"none"));
  doc.roads.push_back(road);

  const auto built = buildAll(doc);
  EXPECT_EQ(built.map.regulatoryElementLayer.size(), 1u);
  bool saw_warning = false;
  for (const auto & e : built.errors) {
    if (e.find("orientation='none'") != std::string::npos) {
      saw_warning = true;
      break;
    }
  }
  EXPECT_TRUE(saw_warning);

  // Both direction lanelets receive the reg element.
  const auto ll_right = laneletFor(built.map, "1", -1);
  const auto ll_left = laneletFor(built.map, "1", 1);
  EXPECT_EQ(ll_right.regulatoryElements().size(), 1u);
  EXPECT_EQ(ll_left.regulatoryElements().size(), 1u);
}

TEST(SignalBuilder, AttachesOnlyToSameDirectionLanesByDefault)
{
  // Signal orientation "+" applies to +s-bound lanes (id < 0). Left-side
  // lanes (id > 0) are unaffected.
  XodrDocument doc;
  auto road = makeStraightRoad(
    "1", 0.0, 0.0, 30.0,
    {makeLane(-1, "driving", 3.5), makeLane(-2, "driving", 3.5)},
    {makeLane(1, "driving", 3.5)});
  road.signals.signals.push_back(makeLight("10", 15.0, -6.0, "1000001", /*orientation=*/"+"));
  doc.roads.push_back(road);

  const auto built = buildAll(doc);
  EXPECT_EQ(laneletFor(built.map, "1", -1).regulatoryElements().size(), 1u);
  EXPECT_EQ(laneletFor(built.map, "1", -2).regulatoryElements().size(), 1u);
  EXPECT_EQ(laneletFor(built.map, "1", +1).regulatoryElements().size(), 0u);
}

TEST(SignalBuilder, ValidityNarrowsApplicableLanes)
{
  XodrDocument doc;
  auto road = makeStraightRoad(
    "1", 0.0, 0.0, 30.0,
    {makeLane(-1, "driving", 3.5), makeLane(-2, "driving", 3.5)}, {});
  auto sig = makeLight("10", 15.0, -6.0);
  SignalValidity v;
  v.from_lane = -1;
  v.to_lane = -1;
  sig.validities.push_back(v);
  road.signals.signals.push_back(sig);
  doc.roads.push_back(road);

  const auto built = buildAll(doc);
  EXPECT_EQ(laneletFor(built.map, "1", -1).regulatoryElements().size(), 1u);
  EXPECT_EQ(laneletFor(built.map, "1", -2).regulatoryElements().size(), 0u);
}

TEST(SignalBuilder, StopLineSpansFullTExtentOfApplicableLanes)
{
  // Two right lanes each 3.5 m wide, both applicable. The stop line must
  // cover the whole t-extent of the applicable lanes: from the inner edge of
  // lane -1 (t = 0) to the outer edge of lane -2 (t = -7). Ordered from
  // smaller id side (t_min = -7) to larger id side (t_max = 0).
  XodrDocument doc;
  auto road = makeStraightRoad(
    "1", 0.0, 0.0, 30.0,
    {makeLane(-1, "driving", 3.5), makeLane(-2, "driving", 3.5)}, {});
  auto sig = makeLight("10", 15.0, 0.0);
  SignalValidity v;
  v.from_lane = -2;
  v.to_lane = -1;
  sig.validities.push_back(v);
  road.signals.signals.push_back(sig);
  doc.roads.push_back(road);

  const auto built = buildAll(doc);
  const lanelet::ConstLineString3d * stop = nullptr;
  for (const auto & ls : built.map.lineStringLayer) {
    if (ls.hasAttribute("type") && ls.attribute("type").value() == "stop_line") {
      stop = &ls;
      break;
    }
  }
  ASSERT_NE(stop, nullptr);
  ASSERT_EQ(stop->size(), 2u);

  // Road heading is 0 → reference line runs +x, t > 0 direction is +y. So
  // y-coordinate of the endpoints equals t.
  EXPECT_NEAR((*stop)[0].x(), 15.0, 1e-6);
  EXPECT_NEAR((*stop)[0].y(), -7.0, 1e-6);
  EXPECT_NEAR((*stop)[1].x(), 15.0, 1e-6);
  EXPECT_NEAR((*stop)[1].y(), 0.0, 1e-6);
  EXPECT_EQ(stop->attribute("opendrive:signal_id").value(), "10");
}

TEST(SignalBuilder, StopLineForSingleLaneSpansThatLane)
{
  // Degenerate single-applicable-lane case: the stop line must still span
  // the lane's full width (3.5 m here), not collapse to a zero-width line.
  XodrDocument doc;
  auto road = makeStraightRoad("1", 0.0, 0.0, 30.0, {makeLane(-1, "driving", 3.5)});
  auto sig = makeLight("10", 15.0, 0.0);
  SignalValidity v;
  v.from_lane = -1;
  v.to_lane = -1;
  sig.validities.push_back(v);
  road.signals.signals.push_back(sig);
  doc.roads.push_back(road);

  const auto built = buildAll(doc);
  const lanelet::ConstLineString3d * stop = nullptr;
  for (const auto & ls : built.map.lineStringLayer) {
    if (ls.hasAttribute("type") && ls.attribute("type").value() == "stop_line") {
      stop = &ls;
      break;
    }
  }
  ASSERT_NE(stop, nullptr);
  ASSERT_EQ(stop->size(), 2u);
  const double span = std::hypot(
    (*stop)[1].x() - (*stop)[0].x(), (*stop)[1].y() - (*stop)[0].y());
  EXPECT_NEAR(span, 3.5, 1e-6);
}

TEST(SignalBuilder, RegElementCarriesTraceabilityAttributes)
{
  XodrDocument doc;
  auto road = makeStraightRoad("42", 0.0, 0.0, 30.0);
  auto sig = makeLight("99", 15.0, -6.0);
  road.signals.signals.push_back(sig);
  doc.roads.push_back(road);

  const auto built = buildAll(doc);
  ASSERT_EQ(built.map.regulatoryElementLayer.size(), 1u);
  const auto & re = *built.map.regulatoryElementLayer.begin();
  EXPECT_EQ(re->attribute("opendrive:signal_id").value(), "99");
  EXPECT_EQ(re->attribute("opendrive:signal_type").value(), "1000001");
  EXPECT_EQ(re->attribute("opendrive:road_id").value(), "42");
  EXPECT_EQ(re->attribute("subtype").value(), "traffic_light");
}

TEST(SignalBuilder, SignalReferenceReusesSingleRegElement)
{
  // Road A owns the signal; road B carries a <signalReference> pointing at
  // it. Per §5.12 dedup, a single AutowareTrafficLight is shared across both
  // roads' lanelets.
  XodrDocument doc;
  auto road_a = makeStraightRoad("A", 0.0, 0.0, 30.0);
  road_a.signals.signals.push_back(makeLight("100", 15.0, -6.0));

  auto road_b = makeStraightRoad("B", 100.0, 0.0, 30.0);
  SignalReference ref;
  ref.id = "100";
  ref.s = 15.0;
  ref.t = -6.0;
  ref.orientation = "+";
  road_b.signals.signal_references.push_back(ref);

  doc.roads.push_back(road_a);
  doc.roads.push_back(road_b);

  const auto built = buildAll(doc);
  ASSERT_EQ(built.map.regulatoryElementLayer.size(), 1u);
  const auto re_id = (*built.map.regulatoryElementLayer.begin())->id();

  const auto ll_a = laneletFor(built.map, "A", -1);
  const auto ll_b = laneletFor(built.map, "B", -1);
  ASSERT_EQ(ll_a.regulatoryElements().size(), 1u);
  ASSERT_EQ(ll_b.regulatoryElements().size(), 1u);
  EXPECT_EQ(ll_a.regulatoryElements().front()->id(), re_id);
  EXPECT_EQ(ll_b.regulatoryElements().front()->id(), re_id);
}

TEST(SignalBuilder, DanglingSignalReferenceWarnsAndSkips)
{
  XodrDocument doc;
  auto road = makeStraightRoad("A", 0.0, 0.0, 30.0);
  SignalReference ref;
  ref.id = "does-not-exist";
  ref.s = 15.0;
  ref.t = -6.0;
  ref.orientation = "+";
  road.signals.signal_references.push_back(ref);
  doc.roads.push_back(road);

  const auto built = buildAll(doc);
  EXPECT_EQ(built.map.regulatoryElementLayer.size(), 0u);
  bool saw_warning = false;
  for (const auto & e : built.errors) {
    if (e.find("signalReference id='does-not-exist'") != std::string::npos) {
      saw_warning = true;
      break;
    }
  }
  EXPECT_TRUE(saw_warning);
}

TEST(SignalBuilder, SignalReferenceValidityNarrowsAttachedLanelets)
{
  // Road A owns the signal with validity covering -1 and -2. Road B
  // references it but narrows to only lane -1.
  XodrDocument doc;
  auto road_a = makeStraightRoad(
    "A", 0.0, 0.0, 30.0,
    {makeLane(-1, "driving", 3.5), makeLane(-2, "driving", 3.5)});
  auto sig = makeLight("100", 15.0, -6.0);
  SignalValidity v_a;
  v_a.from_lane = -2;
  v_a.to_lane = -1;
  sig.validities.push_back(v_a);
  road_a.signals.signals.push_back(sig);

  auto road_b = makeStraightRoad(
    "B", 100.0, 0.0, 30.0,
    {makeLane(-1, "driving", 3.5), makeLane(-2, "driving", 3.5)});
  SignalReference ref;
  ref.id = "100";
  ref.s = 15.0;
  ref.t = -6.0;
  ref.orientation = "+";
  SignalValidity v_b;
  v_b.from_lane = -1;
  v_b.to_lane = -1;
  ref.validities.push_back(v_b);
  road_b.signals.signal_references.push_back(ref);

  doc.roads.push_back(road_a);
  doc.roads.push_back(road_b);

  const auto built = buildAll(doc);
  // Owner road attaches to both lanes.
  EXPECT_EQ(laneletFor(built.map, "A", -1).regulatoryElements().size(), 1u);
  EXPECT_EQ(laneletFor(built.map, "A", -2).regulatoryElements().size(), 1u);
  // Referencing road attaches only to the narrowed lane.
  EXPECT_EQ(laneletFor(built.map, "B", -1).regulatoryElements().size(), 1u);
  EXPECT_EQ(laneletFor(built.map, "B", -2).regulatoryElements().size(), 0u);
}

TEST(SignalBuilder, SidewalkOnlyLaneDropsTheSignal)
{
  // Signal points at a lane that will not be emitted as a lanelet (sidewalk).
  // With no drivable lane to attach to, the signal is skipped with a warning.
  XodrDocument doc;
  auto road = makeStraightRoad("1", 0.0, 0.0, 30.0, {makeLane(-1, "sidewalk", 1.5)});
  auto sig = makeLight("10", 15.0, -2.0);
  SignalValidity v;
  v.from_lane = -1;
  v.to_lane = -1;
  sig.validities.push_back(v);
  road.signals.signals.push_back(sig);
  doc.roads.push_back(road);

  const auto built = buildAll(doc);
  EXPECT_EQ(built.map.regulatoryElementLayer.size(), 0u);
  bool saw_warning = false;
  for (const auto & e : built.errors) {
    if (e.find("no lanelets available") != std::string::npos) {
      saw_warning = true;
      break;
    }
  }
  EXPECT_TRUE(saw_warning);
}

// NOLINTEND(readability-identifier-naming)
