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

// Phase 4 linker tests per §5.9 / §5.10 / §5.11 / §7:
//   - Cross-road endpoint Point3d Id unification via the shared deduper
//   - Junction connecting-road endpoint sharing
//   - Metadata validation: dangling <road>/<link>, internal roads with <link>,
//     dangling <junction>/<connection> references

#include "../../lib/opendrive/junction_linker.hpp"
#include "../../lib/opendrive/lane_builder.hpp"
#include "../../lib/opendrive/road_linker.hpp"
#include "../../lib/opendrive/xodr_types.hpp"

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <string>
#include <vector>

using lanelet::io_handlers::opendrive::GeomLine;
using lanelet::io_handlers::opendrive::Geometry;
using lanelet::io_handlers::opendrive::Junction;
using lanelet::io_handlers::opendrive::JunctionConnection;
using lanelet::io_handlers::opendrive::Lane;
using lanelet::io_handlers::opendrive::LaneBuilderOptions;
using lanelet::io_handlers::opendrive::LaneSection;
using lanelet::io_handlers::opendrive::Point3dDeduper;
using lanelet::io_handlers::opendrive::Poly3Record;
using lanelet::io_handlers::opendrive::Road;
using lanelet::io_handlers::opendrive::RoadLinkEnd;
using lanelet::io_handlers::opendrive::XodrDocument;
using lanelet::io_handlers::opendrive::buildRoadLanelets;
using lanelet::io_handlers::opendrive::validateJunctions;
using lanelet::io_handlers::opendrive::validateRoadLinks;

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

// Single-geometry straight road, one laneSection, right-lane driving.
Road makeStraightRoad(
  const std::string & id, double x0, double y0, double hdg, double length,
  const std::string & junction = "-1")
{
  Road road;
  road.id = id;
  road.length = length;
  road.junction = junction;
  Geometry g;
  g.s = 0.0;
  g.x = x0;
  g.y = y0;
  g.hdg = hdg;
  g.length = length;
  g.primitive = GeomLine{};
  road.plan_view.geometries.push_back(g);
  LaneSection ls;
  ls.s = 0.0;
  ls.right.push_back(makeLane(-1, "driving", 3.5));
  road.lanes.lane_sections.push_back(ls);
  return road;
}

lanelet::ConstLanelet findLaneletByRoadId(
  const lanelet::LaneletMap & map, const std::string & road_id)
{
  for (const auto & ll : map.laneletLayer) {
    if (ll.attribute("opendrive:road_id").value() == road_id) {
      return ll;
    }
  }
  return lanelet::ConstLanelet{};
}
}  // namespace

TEST(SharedDeduper, RoadEndMeetsRoadStartWithEqualPointIds)
{
  // Road A: x ∈ [0, 10], heading 0. End at x = 10.
  // Road B: x ∈ [10, 20], heading 0. Start at x = 10.
  // With a shared deduper the last point of A's boundary and the first point
  // of B's boundary must carry identical lanelet2 Ids (§5.11).
  const auto a = makeStraightRoad("A", 0.0, 0.0, 0.0, 10.0);
  const auto b = makeStraightRoad("B", 10.0, 0.0, 0.0, 10.0);

  LaneBuilderOptions opts;
  Point3dDeduper deduper{opts.merge_tol_m};
  lanelet::LaneletMap map;
  lanelet::ErrorMessages errors;
  buildRoadLanelets(a, opts, deduper, map, errors);
  buildRoadLanelets(b, opts, deduper, map, errors);

  const auto ll_a = findLaneletByRoadId(map, "A");
  const auto ll_b = findLaneletByRoadId(map, "B");
  EXPECT_EQ(ll_a.leftBound().back().id(), ll_b.leftBound().front().id());
  EXPECT_EQ(ll_a.rightBound().back().id(), ll_b.rightBound().front().id());
}

TEST(SharedDeduper, OffsetBeyondMergeTolKeepsPointsDistinct)
{
  // Road B starts 1 cm past where Road A ends — outside the default 1 cm
  // quantization cell boundary (0.04 falls in cell 0, but 0.10 falls in
  // cell 10). Confirm points remain distinct.
  const auto a = makeStraightRoad("A", 0.0, 0.0, 0.0, 10.0);
  const auto b = makeStraightRoad("B", 10.10, 0.0, 0.0, 10.0);

  LaneBuilderOptions opts;  // merge_tol_m default = 0.01
  Point3dDeduper deduper{opts.merge_tol_m};
  lanelet::LaneletMap map;
  lanelet::ErrorMessages errors;
  buildRoadLanelets(a, opts, deduper, map, errors);
  buildRoadLanelets(b, opts, deduper, map, errors);

  const auto ll_a = findLaneletByRoadId(map, "A");
  const auto ll_b = findLaneletByRoadId(map, "B");
  EXPECT_NE(ll_a.leftBound().back().id(), ll_b.leftBound().front().id());
}

TEST(SharedDeduper, JunctionConnectingRoadSharesWithIncoming)
{
  // Incoming "I" at x ∈ [0, 10]; connecting road "C" (internal to junction
  // "J") at x ∈ [10, 20]. The junction metadata is not required for the
  // geometric unification to happen — the deduper alone handles it.
  auto incoming = makeStraightRoad("I", 0.0, 0.0, 0.0, 10.0);
  auto connecting = makeStraightRoad("C", 10.0, 0.0, 0.0, 10.0, /*junction=*/"J");

  LaneBuilderOptions opts;
  Point3dDeduper deduper{opts.merge_tol_m};
  lanelet::LaneletMap map;
  lanelet::ErrorMessages errors;
  buildRoadLanelets(incoming, opts, deduper, map, errors);
  buildRoadLanelets(connecting, opts, deduper, map, errors);

  const auto ll_i = findLaneletByRoadId(map, "I");
  const auto ll_c = findLaneletByRoadId(map, "C");
  EXPECT_EQ(ll_i.leftBound().back().id(), ll_c.leftBound().front().id());
  // The connecting road also picks up the junction traceability tag.
  EXPECT_EQ(ll_c.attribute("opendrive:junction_id").value(), "J");
}

TEST(ValidateRoadLinks, WarnsOnDanglingPredecessor)
{
  auto a = makeStraightRoad("A", 0.0, 0.0, 0.0, 10.0);
  RoadLinkEnd pred;
  pred.element_type = "road";
  pred.element_id = "does-not-exist";
  pred.contact_point = "end";
  a.link.predecessor = pred;

  XodrDocument doc;
  doc.roads.push_back(a);

  lanelet::ErrorMessages errors;
  validateRoadLinks(doc, errors);
  ASSERT_EQ(errors.size(), 1u);
  EXPECT_NE(errors.front().find("unknown road"), std::string::npos);
  EXPECT_NE(errors.front().find("predecessor"), std::string::npos);
}

TEST(ValidateRoadLinks, WarnsOnDanglingSuccessorJunction)
{
  auto a = makeStraightRoad("A", 0.0, 0.0, 0.0, 10.0);
  RoadLinkEnd succ;
  succ.element_type = "junction";
  succ.element_id = "99";
  a.link.successor = succ;

  XodrDocument doc;
  doc.roads.push_back(a);

  lanelet::ErrorMessages errors;
  validateRoadLinks(doc, errors);
  ASSERT_EQ(errors.size(), 1u);
  EXPECT_NE(errors.front().find("unknown junction"), std::string::npos);
}

TEST(ValidateRoadLinks, InternalRoadWithLinkEmitsSingleWarning)
{
  // Internal roads (junction != -1) must not carry <road>/<link>. §5.10.
  // Exactly one warning regardless of how many link ends are set.
  auto internal = makeStraightRoad("X", 0.0, 0.0, 0.0, 10.0, /*junction=*/"J");
  RoadLinkEnd pred;
  pred.element_type = "road";
  pred.element_id = "A";
  internal.link.predecessor = pred;
  RoadLinkEnd succ;
  succ.element_type = "road";
  succ.element_id = "B";
  internal.link.successor = succ;

  XodrDocument doc;
  doc.roads.push_back(makeStraightRoad("A", 0.0, 0.0, 0.0, 10.0));
  doc.roads.push_back(makeStraightRoad("B", 20.0, 0.0, 0.0, 10.0));
  doc.roads.push_back(internal);

  lanelet::ErrorMessages errors;
  validateRoadLinks(doc, errors);
  ASSERT_EQ(errors.size(), 1u);
  EXPECT_NE(errors.front().find("internal to junction"), std::string::npos);
}

TEST(ValidateRoadLinks, ValidMetadataEmitsNoWarnings)
{
  auto a = makeStraightRoad("A", 0.0, 0.0, 0.0, 10.0);
  auto b = makeStraightRoad("B", 10.0, 0.0, 0.0, 10.0);
  RoadLinkEnd a_succ;
  a_succ.element_type = "road";
  a_succ.element_id = "B";
  a_succ.contact_point = "start";
  a.link.successor = a_succ;
  RoadLinkEnd b_pred;
  b_pred.element_type = "road";
  b_pred.element_id = "A";
  b_pred.contact_point = "end";
  b.link.predecessor = b_pred;

  XodrDocument doc;
  doc.roads.push_back(a);
  doc.roads.push_back(b);

  lanelet::ErrorMessages errors;
  validateRoadLinks(doc, errors);
  EXPECT_TRUE(errors.empty());
}

TEST(ValidateJunctions, WarnsOnUnknownIncomingRoad)
{
  XodrDocument doc;
  doc.roads.push_back(makeStraightRoad("A", 0.0, 0.0, 0.0, 10.0));
  Junction j;
  j.id = "1";
  JunctionConnection c;
  c.id = "0";
  c.incoming_road = "MISSING";
  c.connecting_road = "A";
  j.connections.push_back(c);
  doc.junctions.push_back(j);

  lanelet::ErrorMessages errors;
  validateJunctions(doc, errors);
  ASSERT_EQ(errors.size(), 1u);
  EXPECT_NE(errors.front().find("incomingRoad"), std::string::npos);
  EXPECT_NE(errors.front().find("MISSING"), std::string::npos);
}

TEST(ValidateJunctions, WarnsOnUnknownConnectingRoad)
{
  XodrDocument doc;
  doc.roads.push_back(makeStraightRoad("A", 0.0, 0.0, 0.0, 10.0));
  Junction j;
  j.id = "1";
  JunctionConnection c;
  c.id = "0";
  c.incoming_road = "A";
  c.connecting_road = "MISSING";
  j.connections.push_back(c);
  doc.junctions.push_back(j);

  lanelet::ErrorMessages errors;
  validateJunctions(doc, errors);
  ASSERT_EQ(errors.size(), 1u);
  EXPECT_NE(errors.front().find("connectingRoad"), std::string::npos);
}

TEST(ValidateJunctions, ValidMetadataEmitsNoWarnings)
{
  XodrDocument doc;
  doc.roads.push_back(makeStraightRoad("A", 0.0, 0.0, 0.0, 10.0));
  doc.roads.push_back(makeStraightRoad("C", 10.0, 0.0, 0.0, 5.0, /*junction=*/"1"));
  Junction j;
  j.id = "1";
  JunctionConnection c;
  c.id = "0";
  c.incoming_road = "A";
  c.connecting_road = "C";
  j.connections.push_back(c);
  doc.junctions.push_back(j);

  lanelet::ErrorMessages errors;
  validateJunctions(doc, errors);
  EXPECT_TRUE(errors.empty());
}

// NOLINTEND(readability-identifier-naming)
