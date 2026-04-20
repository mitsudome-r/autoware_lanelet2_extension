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

#include "xodr_reader.hpp"

#include <lanelet2_io/Exceptions.h>
#include <pugixml.hpp>

#include <algorithm>
#include <sstream>
#include <string>

namespace lanelet::io_handlers::opendrive
{
namespace
{
constexpr const char * kErrPrefix = "[autoware_opendrive_handler]";

// Append a non-fatal error/warning, with uniform prefix per §7.
void warn(ErrorMessages & errors, const std::string & msg)
{
  errors.emplace_back(std::string{kErrPrefix} + " " + msg);
}

double attrDouble(const pugi::xml_node & n, const char * key, double fallback = 0.0)
{
  const auto attr = n.attribute(key);
  return attr ? attr.as_double(fallback) : fallback;
}

int attrInt(const pugi::xml_node & n, const char * key, int fallback = 0)
{
  const auto attr = n.attribute(key);
  return attr ? attr.as_int(fallback) : fallback;
}

std::string attrStr(const pugi::xml_node & n, const char * key, const char * fallback = "")
{
  const auto attr = n.attribute(key);
  return attr ? std::string{attr.value()} : std::string{fallback};
}

bool attrBoolYesNo(const pugi::xml_node & n, const char * key, bool fallback = false)
{
  const auto attr = n.attribute(key);
  if (!attr) {
    return fallback;
  }
  std::string v = attr.value();
  std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) { return std::tolower(c); });
  return v == "yes" || v == "true" || v == "1";
}

Poly3Record readPoly3(const pugi::xml_node & n, const char * s_key)
{
  Poly3Record r;
  r.s = attrDouble(n, s_key, 0.0);
  r.a = attrDouble(n, "a", 0.0);
  r.b = attrDouble(n, "b", 0.0);
  r.c = attrDouble(n, "c", 0.0);
  r.d = attrDouble(n, "d", 0.0);
  return r;
}

void readHeader(const pugi::xml_node & root, Header & header, ErrorMessages & /*errors*/)
{
  const auto h = root.child("header");
  if (!h) {
    return;  // missing header is tolerated; root attributes carry enough
  }
  header.rev_major = attrInt(h, "revMajor", 0);
  header.rev_minor = attrInt(h, "revMinor", 0);
  header.name = attrStr(h, "name");
  header.version = attrStr(h, "version");
  header.date = attrStr(h, "date");
  header.north = attrDouble(h, "north");
  header.south = attrDouble(h, "south");
  header.east = attrDouble(h, "east");
  header.west = attrDouble(h, "west");
  header.vendor = attrStr(h, "vendor");
  if (const auto geo = h.child("geoReference"); geo) {
    // pugixml exposes CDATA via child_value().
    header.geo_reference = geo.child_value();
    // Trim leading/trailing whitespace typical of CDATA blocks.
    const auto first = header.geo_reference.find_first_not_of(" \t\r\n");
    const auto last = header.geo_reference.find_last_not_of(" \t\r\n");
    if (first == std::string::npos) {
      header.geo_reference.clear();
    } else {
      header.geo_reference = header.geo_reference.substr(first, last - first + 1);
    }
  }
}

Geometry readGeometry(const pugi::xml_node & n, ErrorMessages & errors)
{
  Geometry g;
  g.s = attrDouble(n, "s");
  g.x = attrDouble(n, "x");
  g.y = attrDouble(n, "y");
  g.hdg = attrDouble(n, "hdg");
  g.length = attrDouble(n, "length");

  if (n.child("line")) {
    g.primitive = GeomLine{};
  } else if (const auto a = n.child("arc"); a) {
    GeomArc arc;
    arc.curvature = attrDouble(a, "curvature");
    g.primitive = arc;
  } else if (const auto sp = n.child("spiral"); sp) {
    GeomSpiral s;
    s.curv_start = attrDouble(sp, "curvStart");
    s.curv_end = attrDouble(sp, "curvEnd");
    g.primitive = s;
  } else if (const auto p3 = n.child("poly3"); p3) {
    GeomPoly3 p;
    p.a = attrDouble(p3, "a");
    p.b = attrDouble(p3, "b");
    p.c = attrDouble(p3, "c");
    p.d = attrDouble(p3, "d");
    g.primitive = p;
  } else if (const auto pp = n.child("paramPoly3"); pp) {
    GeomParamPoly3 p;
    p.a_u = attrDouble(pp, "aU");
    p.b_u = attrDouble(pp, "bU");
    p.c_u = attrDouble(pp, "cU");
    p.d_u = attrDouble(pp, "dU");
    p.a_v = attrDouble(pp, "aV");
    p.b_v = attrDouble(pp, "bV");
    p.c_v = attrDouble(pp, "cV");
    p.d_v = attrDouble(pp, "dV");
    const std::string p_range = attrStr(pp, "pRange", "normalized");
    p.p_range_normalized = (p_range != "arcLength");
    g.primitive = p;
  } else {
    std::ostringstream oss;
    oss << "unknown <geometry> primitive at s=" << g.s
        << "; assuming line for forward progress";
    warn(errors, oss.str());
    g.primitive = GeomLine{};
  }
  return g;
}

void readPlanView(const pugi::xml_node & road_node, PlanView & plan, ErrorMessages & errors)
{
  const auto pv = road_node.child("planView");
  if (!pv) {
    return;
  }
  for (const auto & geom : pv.children("geometry")) {
    plan.geometries.push_back(readGeometry(geom, errors));
  }
}

void readElevation(const pugi::xml_node & road_node, ElevationProfile & elev)
{
  const auto ep = road_node.child("elevationProfile");
  if (!ep) {
    return;
  }
  for (const auto & e : ep.children("elevation")) {
    elev.records.push_back(readPoly3(e, "s"));
  }
}

Lane readLane(const pugi::xml_node & lane_node, ErrorMessages & errors, const std::string & road_id)
{
  Lane lane;
  lane.id = attrInt(lane_node, "id");
  lane.type = attrStr(lane_node, "type");
  lane.level = attrStr(lane_node, "level");

  if (const auto link = lane_node.child("link"); link) {
    if (const auto p = link.child("predecessor"); p) {
      lane.link.predecessor_id = attrInt(p, "id");
    }
    if (const auto s = link.child("successor"); s) {
      lane.link.successor_id = attrInt(s, "id");
    }
  }

  for (const auto & w : lane_node.children("width")) {
    lane.widths.push_back(readPoly3(w, "sOffset"));
  }

  if (lane_node.child("border")) {
    lane.has_border = true;
    std::ostringstream oss;
    oss << "road " << road_id << " lane " << lane.id
        << ": <border> is ignored in v1 (treated as zero width)";
    warn(errors, oss.str());
  }
  // <roadMark>, <material>, <speed>, <access> are v1-ignored. They do not
  // meaningfully affect geometry, so no warning per §4.

  return lane;
}

void readLanes(
  const pugi::xml_node & road_node, Lanes & lanes, ErrorMessages & errors,
  const std::string & road_id)
{
  const auto ln = road_node.child("lanes");
  if (!ln) {
    return;
  }

  for (const auto & lo : ln.children("laneOffset")) {
    lanes.lane_offsets.push_back(readPoly3(lo, "s"));
  }

  for (const auto & ls_node : ln.children("laneSection")) {
    LaneSection ls;
    ls.s = attrDouble(ls_node, "s");
    if (const auto left = ls_node.child("left"); left) {
      for (const auto & lane : left.children("lane")) {
        ls.left.push_back(readLane(lane, errors, road_id));
      }
      std::sort(ls.left.begin(), ls.left.end(), [](const Lane & a, const Lane & b) {
        return a.id < b.id;
      });
    }
    if (const auto right = ls_node.child("right"); right) {
      for (const auto & lane : right.children("lane")) {
        ls.right.push_back(readLane(lane, errors, road_id));
      }
      std::sort(ls.right.begin(), ls.right.end(), [](const Lane & a, const Lane & b) {
        return a.id > b.id;  // -1, -2, -3, ...
      });
    }
    // <center> lane 0 carries only roadMarks in v1; ignored.
    lanes.lane_sections.push_back(std::move(ls));
  }
}

void readRoadLink(const pugi::xml_node & road_node, RoadLink & link)
{
  const auto ln = road_node.child("link");
  if (!ln) {
    return;
  }
  auto readEnd = [](const pugi::xml_node & n) {
    RoadLinkEnd end;
    end.element_type = attrStr(n, "elementType");
    end.element_id = attrStr(n, "elementId");
    end.contact_point = attrStr(n, "contactPoint");
    return end;
  };
  if (const auto p = ln.child("predecessor"); p) {
    link.predecessor = readEnd(p);
  }
  if (const auto s = ln.child("successor"); s) {
    link.successor = readEnd(s);
  }
}

std::vector<SignalValidity> readValidities(const pugi::xml_node & parent)
{
  std::vector<SignalValidity> out;
  for (const auto & v : parent.children("validity")) {
    SignalValidity sv;
    sv.from_lane = attrInt(v, "fromLane");
    sv.to_lane = attrInt(v, "toLane");
    out.push_back(sv);
  }
  return out;
}

void readSignals(
  const pugi::xml_node & road_node, Signals & signals, ErrorMessages & errors,
  const std::string & road_id)
{
  const auto ss = road_node.child("signals");
  if (!ss) {
    return;
  }
  for (const auto & s : ss.children("signal")) {
    Signal sig;
    sig.id = attrStr(s, "id");
    sig.s = attrDouble(s, "s");
    sig.t = attrDouble(s, "t");
    sig.name = attrStr(s, "name");
    sig.dynamic = attrBoolYesNo(s, "dynamic");
    sig.orientation = attrStr(s, "orientation");
    sig.z_offset = attrDouble(s, "zOffset");
    sig.type = attrStr(s, "type");
    sig.subtype = attrStr(s, "subtype");
    sig.country = attrStr(s, "country");
    sig.value_str = attrStr(s, "value");
    sig.unit = attrStr(s, "unit");
    sig.height = attrDouble(s, "height");
    sig.width = attrDouble(s, "width");
    sig.h_offset = attrDouble(s, "hOffset");
    sig.pitch = attrDouble(s, "pitch");
    sig.roll = attrDouble(s, "roll");
    sig.text = attrStr(s, "text");
    sig.validities = readValidities(s);
    signals.signals.push_back(std::move(sig));
  }
  for (const auto & sr : ss.children("signalReference")) {
    SignalReference ref;
    ref.id = attrStr(sr, "id");
    ref.s = attrDouble(sr, "s");
    ref.t = attrDouble(sr, "t");
    ref.orientation = attrStr(sr, "orientation");
    ref.validities = readValidities(sr);
    signals.signal_references.push_back(std::move(ref));
  }
  (void)errors;
  (void)road_id;
}

Road readRoad(const pugi::xml_node & road_node, ErrorMessages & errors)
{
  Road road;
  road.id = attrStr(road_node, "id");
  road.name = attrStr(road_node, "name");
  road.length = attrDouble(road_node, "length");
  road.junction = attrStr(road_node, "junction", "-1");

  readRoadLink(road_node, road.link);
  readPlanView(road_node, road.plan_view, errors);
  readElevation(road_node, road.elevation_profile);

  if (const auto lp = road_node.child("lateralProfile"); lp) {
    // §7: non-empty <lateralProfile> is a non-fatal warning. Check any child.
    if (lp.first_child()) {
      road.has_lateral_profile = true;
      std::ostringstream oss;
      oss << "road " << road.id
          << ": <lateralProfile> is ignored in v1 (superelevation/shape not modeled)";
      warn(errors, oss.str());
    }
  }

  readLanes(road_node, road.lanes, errors, road.id);
  readSignals(road_node, road.signals, errors, road.id);

  return road;
}

Junction readJunction(const pugi::xml_node & node)
{
  Junction j;
  j.id = attrStr(node, "id");
  j.name = attrStr(node, "name");
  for (const auto & c : node.children("connection")) {
    JunctionConnection conn;
    conn.id = attrStr(c, "id");
    conn.incoming_road = attrStr(c, "incomingRoad");
    conn.connecting_road = attrStr(c, "connectingRoad");
    conn.contact_point = attrStr(c, "contactPoint");
    for (const auto & ll : c.children("laneLink")) {
      JunctionLaneLink link;
      link.from = attrInt(ll, "from");
      link.to = attrInt(ll, "to");
      conn.lane_links.push_back(link);
    }
    j.connections.push_back(std::move(conn));
  }
  return j;
}

}  // namespace

XodrDocument readXodrFile(const std::string & filename, ErrorMessages & errors)
{
  pugi::xml_document doc;
  const auto result = doc.load_file(filename.c_str());
  if (!result) {
    throw lanelet::ParseError(
      std::string{kErrPrefix} + " failed to parse " + filename + ": " + result.description());
  }

  const auto root = doc.child("OpenDRIVE");
  if (!root) {
    throw lanelet::ParseError(
      std::string{kErrPrefix} + " missing <OpenDRIVE> root element in " + filename);
  }

  XodrDocument out;
  readHeader(root, out.header, errors);

  for (const auto & road : root.children("road")) {
    out.roads.push_back(readRoad(road, errors));
  }
  for (const auto & j : root.children("junction")) {
    out.junctions.push_back(readJunction(j));
  }
  // Root-level <controller> and <junctionGroup> are v1-ignored (§4). No
  // warning per spec: they do not affect geometry.

  return out;
}

}  // namespace lanelet::io_handlers::opendrive
