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

#ifndef OPENDRIVE__XODR_TYPES_HPP_
#define OPENDRIVE__XODR_TYPES_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace lanelet::io_handlers::opendrive
{
// POD mirrors of a subset of the OpenDRIVE XML tree. Only fields the spec's v1
// scope uses are carried; anything "ignored in v1" is dropped at read time and
// emits a non-fatal warning where §7 requires one.

struct Header
{
  int rev_major{0};
  int rev_minor{0};
  std::string name;
  std::string version;
  std::string date;
  double north{0.0};
  double south{0.0};
  double east{0.0};
  double west{0.0};
  std::string vendor;
  std::string geo_reference;  // PROJ string, possibly empty
};

// Reference-line geometry primitives (§5.3).

struct GeomLine
{
};

struct GeomArc
{
  double curvature{0.0};
};

struct GeomSpiral
{
  double curv_start{0.0};
  double curv_end{0.0};
};

struct GeomPoly3
{
  double a{0.0};
  double b{0.0};
  double c{0.0};
  double d{0.0};
};

struct GeomParamPoly3
{
  double a_u{0.0};
  double b_u{0.0};
  double c_u{0.0};
  double d_u{0.0};
  double a_v{0.0};
  double b_v{0.0};
  double c_v{0.0};
  double d_v{0.0};
  bool p_range_normalized{true};  // true = "normalized" [0,1]; false = "arcLength" [0,length]
};

struct Geometry
{
  double s{0.0};
  double x{0.0};
  double y{0.0};
  double hdg{0.0};
  double length{0.0};
  std::variant<GeomLine, GeomArc, GeomSpiral, GeomPoly3, GeomParamPoly3> primitive;
};

struct PlanView
{
  std::vector<Geometry> geometries;
};

// Elevation (§5.4). Also used for <laneOffset> and <lane>/<width>
// because the OpenDRIVE encoding is identical piecewise-cubic polynomial.

struct Poly3Record
{
  double s{0.0};  // for <width>, this is sOffset (relative to laneSection start)
  double a{0.0};
  double b{0.0};
  double c{0.0};
  double d{0.0};
};

struct ElevationProfile
{
  std::vector<Poly3Record> records;
};

// Lanes (§5.5).

struct LaneLinkIds
{
  std::optional<int> predecessor_id;
  std::optional<int> successor_id;
};

struct Lane
{
  int id{0};
  std::string type;   // "driving", "biking", "sidewalk", "shoulder", "border", ...
  std::string level;  // "true" / "false" / empty
  LaneLinkIds link;
  std::vector<Poly3Record> widths;  // <width> records, s = sOffset
  bool has_border{false};           // v1 treats <border> as zero width with warning
};

struct LaneSection
{
  double s{0.0};
  std::vector<Lane> left;   // ordered ascending by id (1, 2, 3, ...)
  std::vector<Lane> right;  // ordered descending by id (-1, -2, -3, ...)
  // <center> lane 0 is skipped at read time: in v1 it carries only roadMarks.
};

struct Lanes
{
  std::vector<Poly3Record> lane_offsets;  // <laneOffset> records
  std::vector<LaneSection> lane_sections;
};

// Road link (§5.9).

struct RoadLinkEnd
{
  std::string element_type;   // "road" | "junction"
  std::string element_id;
  std::string contact_point;  // "start" | "end" (road only)
};

struct RoadLink
{
  std::optional<RoadLinkEnd> predecessor;
  std::optional<RoadLinkEnd> successor;
};

// Signals (§5.12).

struct SignalValidity
{
  int from_lane{0};
  int to_lane{0};
};

struct Signal
{
  std::string id;
  double s{0.0};
  double t{0.0};
  std::string name;
  bool dynamic{false};
  std::string orientation;  // "+", "-", "none"
  double z_offset{0.0};
  std::string type;
  std::string subtype;
  std::string country;
  std::string value_str;
  std::string unit;
  double height{0.0};
  double width{0.0};
  double h_offset{0.0};
  double pitch{0.0};
  double roll{0.0};
  std::string text;
  std::vector<SignalValidity> validities;
};

struct SignalReference
{
  std::string id;
  double s{0.0};
  double t{0.0};
  std::string orientation;
  std::vector<SignalValidity> validities;
};

struct Signals
{
  std::vector<Signal> signals;
  std::vector<SignalReference> signal_references;
};

// Road.

struct Road
{
  std::string id;
  std::string name;
  double length{0.0};
  std::string junction;  // "-1" for non-junction roads
  RoadLink link;
  PlanView plan_view;
  ElevationProfile elevation_profile;
  bool has_lateral_profile{false};  // v1 ignores contents; flag for warning
  Lanes lanes;
  Signals signals;
};

// Junction (§5.10).

struct JunctionLaneLink
{
  int from{0};
  int to{0};
};

struct JunctionConnection
{
  std::string id;
  std::string incoming_road;
  std::string connecting_road;
  std::string contact_point;  // "start" | "end"
  std::vector<JunctionLaneLink> lane_links;
};

struct Junction
{
  std::string id;
  std::string name;
  std::vector<JunctionConnection> connections;
};

struct XodrDocument
{
  Header header;
  std::vector<Road> roads;
  std::vector<Junction> junctions;
};

}  // namespace lanelet::io_handlers::opendrive

#endif  // OPENDRIVE__XODR_TYPES_HPP_
