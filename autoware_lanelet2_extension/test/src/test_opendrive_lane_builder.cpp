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

// Phase 3 lane-building tests per §5.5–§5.8, §5.11: evaluator sanity,
// spatial-hash dedup, shared linestrings within a section, shared Point3d's
// across section boundaries, and lane-type → subtype classification.

#include "../../lib/opendrive/lane_builder.hpp"
#include "../../lib/opendrive/xodr_types.hpp"

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>

#include <cmath>
#include <string>
#include <unordered_set>
#include <vector>

using lanelet::io_handlers::opendrive::GeomLine;
using lanelet::io_handlers::opendrive::Geometry;
using lanelet::io_handlers::opendrive::Lane;
using lanelet::io_handlers::opendrive::LaneBuilderOptions;
using lanelet::io_handlers::opendrive::LaneOffsetEvaluator;
using lanelet::io_handlers::opendrive::LaneSection;
using lanelet::io_handlers::opendrive::Point3dDeduper;
using lanelet::io_handlers::opendrive::Poly3Record;
using lanelet::io_handlers::opendrive::Road;
using lanelet::io_handlers::opendrive::WidthEvaluator;
using lanelet::io_handlers::opendrive::buildRoadLanelets;

namespace
{
Poly3Record makePoly3(double s, double a, double b = 0.0, double c = 0.0, double d = 0.0)
{
  Poly3Record r;
  r.s = s;
  r.a = a;
  r.b = b;
  r.c = c;
  r.d = d;
  return r;
}

Lane makeLane(int id, const std::string & type, double constant_width)
{
  Lane l;
  l.id = id;
  l.type = type;
  Poly3Record w;
  w.s = 0.0;
  w.a = constant_width;
  l.widths.push_back(w);
  return l;
}

// A minimal Road: one <geometry> of type "line", one <laneSection>, constant
// width lanes, zero laneOffset, zero elevation.
Road makeStraightRoad(double length, const std::vector<Lane> & left_lanes,
                      const std::vector<Lane> & right_lanes,
                      const std::string & road_id = "1",
                      const std::string & junction = "-1")
{
  Road road;
  road.id = road_id;
  road.length = length;
  road.junction = junction;

  Geometry g;
  g.s = 0.0;
  g.x = 0.0;
  g.y = 0.0;
  g.hdg = 0.0;
  g.length = length;
  g.primitive = GeomLine{};
  road.plan_view.geometries.push_back(g);

  LaneSection ls;
  ls.s = 0.0;
  ls.left = left_lanes;
  ls.right = right_lanes;
  road.lanes.lane_sections.push_back(ls);
  return road;
}
}  // namespace

TEST(LaneOffsetEvaluator, EmptyIsZero)
{
  LaneOffsetEvaluator e{{}};
  EXPECT_NEAR(e.evaluate(0.0), 0.0, 1e-12);
  EXPECT_NEAR(e.evaluate(100.0), 0.0, 1e-12);
}

TEST(LaneOffsetEvaluator, SingleRecord)
{
  LaneOffsetEvaluator e{{makePoly3(0.0, 0.5, 0.1)}};
  EXPECT_NEAR(e.evaluate(0.0), 0.5, 1e-12);
  EXPECT_NEAR(e.evaluate(10.0), 0.5 + 0.1 * 10.0, 1e-12);
}

TEST(LaneOffsetEvaluator, PiecewiseSwitchesAtBreakpoint)
{
  // r0 valid for s ∈ [0, 10), r1 valid for s ≥ 10.
  LaneOffsetEvaluator e{{makePoly3(0.0, 0.0, 1.0), makePoly3(10.0, 50.0, -1.0)}};
  EXPECT_NEAR(e.evaluate(5.0), 5.0, 1e-12);
  EXPECT_NEAR(e.evaluate(10.0), 50.0, 1e-12);
  EXPECT_NEAR(e.evaluate(15.0), 50.0 - 5.0, 1e-12);
  ASSERT_EQ(e.breakpoints().size(), 1u);
  EXPECT_NEAR(e.breakpoints()[0], 10.0, 1e-12);
}

TEST(WidthEvaluator, ZeroWhenEmpty)
{
  WidthEvaluator w{{}};
  EXPECT_NEAR(w.evaluate(0.0), 0.0, 1e-12);
  EXPECT_NEAR(w.evaluate(10.0), 0.0, 1e-12);
}

TEST(WidthEvaluator, PiecewiseRespectsSOffset)
{
  // sOffset=0: w=3.5; sOffset=5: w starts tapering.
  WidthEvaluator w{{makePoly3(0.0, 3.5), makePoly3(5.0, 3.5, -0.1)}};
  EXPECT_NEAR(w.evaluate(0.0), 3.5, 1e-12);
  EXPECT_NEAR(w.evaluate(4.999), 3.5, 1e-12);
  EXPECT_NEAR(w.evaluate(5.0), 3.5, 1e-12);
  EXPECT_NEAR(w.evaluate(10.0), 3.5 + (-0.1) * 5.0, 1e-9);
}

TEST(Point3dDeduper, CollapsesNearbyPoints)
{
  Point3dDeduper d{0.01};
  const auto a = d.getOrCreate(1.0, 2.0, 3.0);
  const auto b = d.getOrCreate(1.001, 2.001, 3.001);  // well inside ε
  EXPECT_EQ(a.id(), b.id());
}

TEST(Point3dDeduper, KeepsDistantPointsDistinct)
{
  Point3dDeduper d{0.01};
  const auto a = d.getOrCreate(0.0, 0.0, 0.0);
  const auto b = d.getOrCreate(1.0, 0.0, 0.0);  // far outside ε
  EXPECT_NE(a.id(), b.id());
}

TEST(Point3dDeduper, LookupIsCellBased)
{
  // Points > ε apart but in the same quantized cell still merge.
  Point3dDeduper d{0.1};
  const auto a = d.getOrCreate(0.0, 0.0, 0.0);
  const auto b = d.getOrCreate(0.04, 0.0, 0.0);  // round(0.04/0.1)=0, same cell
  EXPECT_EQ(a.id(), b.id());
}

TEST(BuildRoadLanelets, SingleDrivingLaneEmitsOneLanelet)
{
  const auto road = makeStraightRoad(10.0, {}, {makeLane(-1, "driving", 3.5)});
  LaneBuilderOptions opts;
  Point3dDeduper deduper{opts.merge_tol_m};
  lanelet::LaneletMap map;
  lanelet::ErrorMessages errors;
  buildRoadLanelets(road, opts, deduper, map, errors);

  ASSERT_EQ(map.laneletLayer.size(), 1u);
  const auto & ll = *map.laneletLayer.begin();
  EXPECT_EQ(ll.attribute("type").value(), "lanelet");
  EXPECT_EQ(ll.attribute("subtype").value(), "road");
  EXPECT_EQ(ll.attribute("location").value(), "urban");
  EXPECT_EQ(ll.attribute("one_way").value(), "yes");
  EXPECT_EQ(ll.attribute("opendrive:road_id").value(), "1");
  EXPECT_EQ(ll.attribute("opendrive:lane_section").value(), "0");
  EXPECT_EQ(ll.attribute("opendrive:lane_id").value(), "-1");
  EXPECT_FALSE(ll.hasAttribute("opendrive:junction_id"));

  // Geometry sanity: right-lane inner at t = 0, outer at t = -3.5.
  const auto & left = ll.leftBound();
  const auto & right = ll.rightBound();
  EXPECT_EQ(left.size(), right.size());
  EXPECT_NEAR(left.front().y(), 0.0, 1e-9);
  EXPECT_NEAR(left.back().y(), 0.0, 1e-9);
  EXPECT_NEAR(right.front().y(), -3.5, 1e-9);
  EXPECT_NEAR(right.back().y(), -3.5, 1e-9);
  // Points run in ascending s for id<0 per §5.8.
  EXPECT_LT(left.front().x(), left.back().x());
}

TEST(BuildRoadLanelets, AdjacentRightLanesShareBoundaryLinestring)
{
  // Two right-side driving lanes: -1 and -2. Per §5.11 the outer of -1 must
  // be the inner of -2 as the same LineString3d (identified by equal Id).
  const auto road = makeStraightRoad(
    10.0, {}, {makeLane(-1, "driving", 3.5), makeLane(-2, "driving", 3.5)});
  LaneBuilderOptions opts;
  Point3dDeduper deduper{opts.merge_tol_m};
  lanelet::LaneletMap map;
  lanelet::ErrorMessages errors;
  buildRoadLanelets(road, opts, deduper, map, errors);

  ASSERT_EQ(map.laneletLayer.size(), 2u);
  lanelet::ConstLanelet lane_m1;
  lanelet::ConstLanelet lane_m2;
  for (const auto & ll : map.laneletLayer) {
    if (ll.attribute("opendrive:lane_id").value() == "-1") {
      lane_m1 = ll;
    } else if (ll.attribute("opendrive:lane_id").value() == "-2") {
      lane_m2 = ll;
    }
  }
  EXPECT_EQ(lane_m1.rightBound().id(), lane_m2.leftBound().id());
}

TEST(BuildRoadLanelets, LeftAndRightShareCenterLinestring)
{
  // Lane +1 and lane -1 both use the lane-0 center line as their inner
  // boundary (§5.11). Their leftBounds point at the same underlying
  // LineString3d — same Id, even though lane +1 iterates it in reverse.
  const auto road = makeStraightRoad(
    10.0, {makeLane(1, "driving", 3.5)}, {makeLane(-1, "driving", 3.5)});
  LaneBuilderOptions opts;
  Point3dDeduper deduper{opts.merge_tol_m};
  lanelet::LaneletMap map;
  lanelet::ErrorMessages errors;
  buildRoadLanelets(road, opts, deduper, map, errors);

  ASSERT_EQ(map.laneletLayer.size(), 2u);
  lanelet::ConstLanelet lane_m1;
  lanelet::ConstLanelet lane_p1;
  for (const auto & ll : map.laneletLayer) {
    if (ll.attribute("opendrive:lane_id").value() == "-1") {
      lane_m1 = ll;
    } else if (ll.attribute("opendrive:lane_id").value() == "1") {
      lane_p1 = ll;
    }
  }
  EXPECT_EQ(lane_m1.leftBound().id(), lane_p1.leftBound().id());
  // The left-side lane's bound is inverted — point order opposite to the
  // right-side lane's leftBound.
  EXPECT_NEAR(lane_p1.leftBound().front().x(), lane_m1.leftBound().back().x(), 1e-9);
  EXPECT_NEAR(lane_p1.leftBound().back().x(), lane_m1.leftBound().front().x(), 1e-9);
}

TEST(BuildRoadLanelets, SidewalkEmitsNoLaneletButOuterLaneStillGetsBoundary)
{
  // Inner driving lane + outer sidewalk lane. Only the driving lane becomes a
  // lanelet; the sidewalk is silently skipped with default skip_sidewalks.
  const auto road = makeStraightRoad(
    10.0, {}, {makeLane(-1, "driving", 3.5), makeLane(-2, "sidewalk", 1.5)});
  LaneBuilderOptions opts;
  Point3dDeduper deduper{opts.merge_tol_m};
  lanelet::LaneletMap map;
  lanelet::ErrorMessages errors;
  buildRoadLanelets(road, opts, deduper, map, errors);

  ASSERT_EQ(map.laneletLayer.size(), 1u);
  const auto & ll = *map.laneletLayer.begin();
  EXPECT_EQ(ll.attribute("opendrive:lane_id").value(), "-1");
  // Sidewalks are silently skipped — no warning per the default policy.
  EXPECT_TRUE(errors.empty());
}

TEST(BuildRoadLanelets, UnsupportedLaneTypeWarnsAndSkips)
{
  const auto road = makeStraightRoad(
    10.0, {}, {makeLane(-1, "driving", 3.5), makeLane(-2, "restricted", 2.0)});
  LaneBuilderOptions opts;
  Point3dDeduper deduper{opts.merge_tol_m};
  lanelet::LaneletMap map;
  lanelet::ErrorMessages errors;
  buildRoadLanelets(road, opts, deduper, map, errors);

  ASSERT_EQ(map.laneletLayer.size(), 1u);
  EXPECT_EQ(errors.size(), 1u);
  EXPECT_NE(errors.front().find("restricted"), std::string::npos);
}

TEST(BuildRoadLanelets, JunctionRoadCarriesJunctionAttribute)
{
  const auto road = makeStraightRoad(
    10.0, {}, {makeLane(-1, "driving", 3.5)}, /*road_id=*/"42", /*junction=*/"7");
  LaneBuilderOptions opts;
  Point3dDeduper deduper{opts.merge_tol_m};
  lanelet::LaneletMap map;
  lanelet::ErrorMessages errors;
  buildRoadLanelets(road, opts, deduper, map, errors);

  ASSERT_EQ(map.laneletLayer.size(), 1u);
  const auto & ll = *map.laneletLayer.begin();
  EXPECT_EQ(ll.attribute("opendrive:junction_id").value(), "7");
}

TEST(BuildRoadLanelets, MultiSectionSharesEndpointPoints)
{
  // Two lane sections that both use lane -1 of different width. The point at
  // s = section boundary (absolute s = 5) must be shared between sections
  // (Id equality) per §5.11.
  Road road;
  road.id = "1";
  road.length = 10.0;
  road.junction = "-1";

  Geometry g;
  g.s = 0.0;
  g.x = 0.0;
  g.y = 0.0;
  g.hdg = 0.0;
  g.length = 10.0;
  g.primitive = GeomLine{};
  road.plan_view.geometries.push_back(g);

  LaneSection s0;
  s0.s = 0.0;
  s0.right.push_back(makeLane(-1, "driving", 3.5));
  LaneSection s1;
  s1.s = 5.0;
  s1.right.push_back(makeLane(-1, "driving", 3.5));
  road.lanes.lane_sections = {s0, s1};

  LaneBuilderOptions opts;
  Point3dDeduper deduper{opts.merge_tol_m};
  lanelet::LaneletMap map;
  lanelet::ErrorMessages errors;
  buildRoadLanelets(road, opts, deduper, map, errors);

  ASSERT_EQ(map.laneletLayer.size(), 2u);
  lanelet::ConstLanelet sec0;
  lanelet::ConstLanelet sec1;
  for (const auto & ll : map.laneletLayer) {
    if (ll.attribute("opendrive:lane_section").value() == "0") {
      sec0 = ll;
    } else {
      sec1 = ll;
    }
  }
  // Section 0's last point on either bound is Section 1's first point.
  EXPECT_EQ(sec0.leftBound().back().id(), sec1.leftBound().front().id());
  EXPECT_EQ(sec0.rightBound().back().id(), sec1.rightBound().front().id());
}

// NOLINTEND(readability-identifier-naming)
