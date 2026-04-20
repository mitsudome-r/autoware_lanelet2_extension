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

#include "signal_builder.hpp"

#include "geometry.hpp"
#include "lane_builder.hpp"

#include "autoware_lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp"

#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/LineStringOrPolygon.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_core/utility/Optional.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <map>
#include <sstream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace lanelet::io_handlers::opendrive
{
namespace
{
constexpr const char * kErrPrefix = "[autoware_opendrive_handler]";

void warn(lanelet::ErrorMessages & errors, const std::string & msg)
{
  errors.emplace_back(std::string{kErrPrefix} + " " + msg);
}

using LaneletKey = std::tuple<std::string, std::size_t, int>;

// Index the map's lanelets by their §8 traceability attributes so the signal
// builder can reach each (road, section, lane) tuple in O(log n). We stash
// mutable `Lanelet` handles because we have to call `addRegulatoryElement`
// on them after the reg element is minted.
std::map<LaneletKey, lanelet::Lanelet> indexLanelets(lanelet::LaneletMap & map)
{
  std::map<LaneletKey, lanelet::Lanelet> idx;
  for (auto & ll : map.laneletLayer) {
    if (
      !ll.hasAttribute("opendrive:road_id") || !ll.hasAttribute("opendrive:lane_section") ||
      !ll.hasAttribute("opendrive:lane_id")) {
      continue;  // not ours — ignore foreign lanelets
    }
    const std::string road_id = ll.attribute("opendrive:road_id").value();
    const auto section_idx = static_cast<std::size_t>(
      std::stoul(ll.attribute("opendrive:lane_section").value()));
    const int lane_id = std::stoi(ll.attribute("opendrive:lane_id").value());
    idx.emplace(std::make_tuple(road_id, section_idx, lane_id), ll);
  }
  return idx;
}

// Last lane-section index whose `s` is ≤ the query s. Matches §5.5 semantics
// (sections are right-open: [s_k, s_{k+1})).
std::size_t findSectionIdx(const Road & road, double s)
{
  std::size_t idx = 0;
  const auto & sections = road.lanes.lane_sections;
  for (std::size_t i = 1; i < sections.size(); ++i) {
    if (sections[i].s <= s) {
      idx = i;
    } else {
      break;
    }
  }
  return idx;
}

// §5.12 applicable-lane filter, a function of the authored `<validity>` list
// and the signal `orientation`. `validities` may be empty (§5.12 default:
// "all same-direction lanes in the current lane section").
std::vector<int> applicableLaneIds(
  const LaneSection & section, const std::vector<SignalValidity> & validities,
  const std::string & orientation)
{
  auto matches_orientation = [&](int id) {
    if (orientation == "+") return id < 0;  // +s-bound lanes
    if (orientation == "-") return id > 0;  // -s-bound lanes
    return true;                            // "none" or empty: both directions
  };
  auto matches_validity = [&](int id) {
    if (validities.empty()) {
      return true;
    }
    for (const auto & v : validities) {
      if (v.from_lane <= id && id <= v.to_lane) {
        return true;
      }
    }
    return false;
  };

  std::vector<int> ids;
  for (const auto & l : section.left) {
    if (matches_orientation(l.id) && matches_validity(l.id)) {
      ids.push_back(l.id);
    }
  }
  for (const auto & l : section.right) {
    if (matches_orientation(l.id) && matches_validity(l.id)) {
      ids.push_back(l.id);
    }
  }
  std::sort(ids.begin(), ids.end());
  return ids;
}

bool isDrivableType(const std::string & type)
{
  // Mirrors §5.8: only these types produce lanelets, so only these can have
  // regulatory elements attached.
  return type == "driving" || type == "biking" || type == "shoulder";
}

bool lookupLaneType(
  const Road & road, std::size_t section_idx, int lane_id, std::string & out_type)
{
  if (section_idx >= road.lanes.lane_sections.size()) {
    return false;
  }
  const auto & section = road.lanes.lane_sections[section_idx];
  const auto & side = lane_id < 0 ? section.right : section.left;
  for (const auto & l : side) {
    if (l.id == lane_id) {
      out_type = l.type;
      return true;
    }
  }
  return false;
}

struct RoadEvaluators
{
  ReferenceLineEvaluator rle;
  ElevationEvaluator ele;
  LaneOffsetEvaluator loe;
};

RoadEvaluators makeEvaluators(const Road & road)
{
  return RoadEvaluators{
    ReferenceLineEvaluator{road.plan_view}, ElevationEvaluator{road.elevation_profile},
    LaneOffsetEvaluator{road.lanes.lane_offsets}};
}

bool typeInAllowlist(const std::string & type, const std::vector<std::string> & allowlist)
{
  return std::find(allowlist.begin(), allowlist.end(), type) != allowlist.end();
}

// Build the 2-point physical-light `LineString3d` (§5.12 "World position").
// Uses the deduper for its endpoints so identical signals across `<signalReference>`s
// *would* collapse if we were to ever re-emit them (we don't in v1, so the
// effect is only to keep the Point3d id-space dense).
lanelet::LineString3d buildLightLineString(
  const Signal & sig, const RoadEvaluators & ev, const SignalBuilderOptions & opts,
  Point3dDeduper & deduper, lanelet::ErrorMessages & errors, const std::string & road_id)
{
  const auto ref = ev.rle.evaluate(sig.s);
  const double z_ref = ev.ele.evaluate(sig.s);
  const double x0 = ref.x - std::sin(ref.hdg) * sig.t;
  const double y0 = ref.y + std::cos(ref.hdg) * sig.t;
  const double z0 = z_ref + sig.z_offset;
  const double theta = ref.hdg + sig.h_offset;

  double width_eff = sig.width;
  if (width_eff <= 0.0) {
    width_eff = opts.fallback_light_width_m;
    std::ostringstream oss;
    oss << "road " << road_id << " signal " << sig.id
        << ": width missing or zero; falling back to " << opts.fallback_light_width_m << " m";
    warn(errors, oss.str());
  }

  const double dx = std::cos(theta) * width_eff * 0.5;
  const double dy = std::sin(theta) * width_eff * 0.5;
  const auto p1 = deduper.getOrCreate(x0 - dx, y0 - dy, z0);
  const auto p2 = deduper.getOrCreate(x0 + dx, y0 + dy, z0);

  lanelet::LineString3d ls{lanelet::utils::getId()};
  ls.push_back(p1);
  ls.push_back(p2);
  ls.setAttribute("type", "traffic_light");
  ls.setAttribute("subtype", "red_yellow_green");
  if (sig.height > 0.0) {
    ls.setAttribute("height", std::to_string(sig.height));
  }
  ls.setAttribute("opendrive:signal_id", sig.id);
  return ls;
}

// §5.12 stop-line synthesis. Returns boost::none iff no applicable lanelet
// could be located on `owner_road` — this is the "no applicable lanes in
// section" §7 case and the caller treats it as "skip the signal".
//
// The spec's literal "t_min = min(outer t of applicable lanes)" collapses a
// single-lane signal into a zero-width stop line, which is clearly not what
// the feature is for. The sensible reading — and the one this implementation
// uses — is: the stop line spans the full t-extent of the applicable lanes,
// i.e. both the inner and outer boundary of every applicable lane feed into
// the min/max aggregation. A shared boundary between two applicable lanes
// just appears twice and folds away in min/max.
lanelet::Optional<lanelet::LineString3d> buildStopLine(
  const Signal & sig, const Road & owner_road, const LaneSection & section,
  const std::vector<int> & applicable_ids, const RoadEvaluators & ev,
  Point3dDeduper & deduper)
{
  if (applicable_ids.empty()) {
    return {};
  }

  const double s_sec = sig.s - section.s;
  const double t_center = ev.loe.evaluate(sig.s);
  double t_min = std::numeric_limits<double>::infinity();
  double t_max = -std::numeric_limits<double>::infinity();
  auto update = [&](double v) {
    t_min = std::min(t_min, v);
    t_max = std::max(t_max, v);
  };

  // Right side: walk -1, -2, -3, ... cumulating width in the −t direction.
  {
    double cumul = t_center;
    for (const auto & lane : section.right) {
      const double inner = cumul;
      cumul -= std::max(0.0, WidthEvaluator{lane.widths}.evaluate(s_sec));
      const double outer = cumul;
      if (std::find(applicable_ids.begin(), applicable_ids.end(), lane.id) != applicable_ids.end()) {
        update(inner);
        update(outer);
      }
    }
  }
  // Left side: walk +1, +2, +3, ... in the +t direction.
  {
    double cumul = t_center;
    for (const auto & lane : section.left) {
      const double inner = cumul;
      cumul += std::max(0.0, WidthEvaluator{lane.widths}.evaluate(s_sec));
      const double outer = cumul;
      if (std::find(applicable_ids.begin(), applicable_ids.end(), lane.id) != applicable_ids.end()) {
        update(inner);
        update(outer);
      }
    }
  }

  const auto ref = ev.rle.evaluate(sig.s);
  const double z_ref = ev.ele.evaluate(sig.s);
  const double sh = std::sin(ref.hdg);
  const double ch = std::cos(ref.hdg);

  const auto p_a =
    deduper.getOrCreate(ref.x - sh * t_min, ref.y + ch * t_min, z_ref);
  const auto p_b =
    deduper.getOrCreate(ref.x - sh * t_max, ref.y + ch * t_max, z_ref);

  lanelet::LineString3d stop_line{lanelet::utils::getId()};
  stop_line.push_back(p_a);
  stop_line.push_back(p_b);
  stop_line.setAttribute("type", "stop_line");
  stop_line.setAttribute("opendrive:signal_id", sig.id);
  (void)owner_road;
  return stop_line;
}

// Build the `AttributeMap` that goes onto the AutowareTrafficLight
// regulatory element per §5.12 and §8. The Type / Subtype fields are set
// inside TrafficLight's constructor so we don't duplicate them here.
lanelet::AttributeMap buildRegElemAttrs(const Signal & sig, const Road & owner_road)
{
  lanelet::AttributeMap attrs;
  attrs[std::string{"opendrive:signal_id"}] = sig.id;
  attrs[std::string{"opendrive:signal_type"}] = sig.type;
  attrs[std::string{"opendrive:signal_subtype"}] = sig.subtype;
  attrs[std::string{"opendrive:signal_country"}] = sig.country;
  attrs[std::string{"opendrive:road_id"}] = owner_road.id;
  return attrs;
}

// Attach `regelem` to every lanelet identified by (road_id, section_idx,
// lane_id) that both exists in the map and maps to a drivable lane type.
// Returns the count of successful attachments — a caller that expected at
// least one can warn if this is 0.
std::size_t attachToLanelets(
  const std::string & road_id, std::size_t section_idx, const std::vector<int> & lane_ids,
  const Road & road, lanelet::RegulatoryElementPtr regelem,
  std::map<LaneletKey, lanelet::Lanelet> & lanelet_idx)
{
  std::size_t n = 0;
  for (int lane_id : lane_ids) {
    std::string type;
    if (!lookupLaneType(road, section_idx, lane_id, type) || !isDrivableType(type)) {
      continue;  // sidewalks / borders / skipped types have no lanelet
    }
    const auto it = lanelet_idx.find(std::make_tuple(road_id, section_idx, lane_id));
    if (it == lanelet_idx.end()) {
      continue;
    }
    it->second.addRegulatoryElement(regelem);
    ++n;
  }
  return n;
}
}  // namespace

void buildSignals(
  const XodrDocument & doc, const SignalBuilderOptions & opts, Point3dDeduper & deduper,
  lanelet::LaneletMap & map, lanelet::ErrorMessages & errors)
{
  auto lanelet_idx = indexLanelets(map);

  // Index roads by id so `<signalReference>` owner-road lookup is O(log n).
  std::map<std::string, const Road *> road_by_id;
  for (const auto & r : doc.roads) {
    road_by_id.emplace(r.id, &r);
  }

  // signal id → emitted reg element. Populated during the first pass;
  // consulted during the `<signalReference>` pass.
  std::map<std::string, lanelet::RegulatoryElementPtr> regelem_by_signal_id;

  // First pass: walk every `<signal>` and emit a reg element for each
  // traffic-light-classified entry.
  for (const auto & road : doc.roads) {
    if (road.signals.signals.empty()) {
      continue;
    }
    const auto ev = makeEvaluators(road);

    for (const auto & sig : road.signals.signals) {
      if (!sig.dynamic) {
        // Static signs are silently ignored in v1 (§5.12). No warning.
        continue;
      }
      if (!typeInAllowlist(sig.type, opts.traffic_light_types)) {
        std::ostringstream oss;
        oss << "road " << road.id << " signal " << sig.id
            << ": dynamic=yes type='" << sig.type
            << "' not in traffic_light_types allowlist; skipping";
        warn(errors, oss.str());
        continue;
      }

      if (sig.orientation == "none") {
        // §7: both-directions assumed warning. Still produces a reg element.
        std::ostringstream oss;
        oss << "road " << road.id << " signal " << sig.id
            << ": orientation='none'; applying to both directions";
        warn(errors, oss.str());
      }

      if (road.lanes.lane_sections.empty()) {
        std::ostringstream oss;
        oss << "road " << road.id << " signal " << sig.id
            << ": road has no lane sections; skipping";
        warn(errors, oss.str());
        continue;
      }

      const auto section_idx = findSectionIdx(road, sig.s);
      const auto & section = road.lanes.lane_sections[section_idx];
      const auto applicable = applicableLaneIds(section, sig.validities, sig.orientation);
      if (applicable.empty()) {
        std::ostringstream oss;
        oss << "road " << road.id << " signal " << sig.id
            << ": no applicable lanes at s=" << sig.s << "; skipping";
        warn(errors, oss.str());
        continue;
      }

      auto light_ls = buildLightLineString(sig, ev, opts, deduper, errors, road.id);
      auto stop_line_opt = buildStopLine(sig, road, section, applicable, ev, deduper);

      lanelet::LineStringsOrPolygons3d refers;
      refers.emplace_back(light_ls);
      auto tl = lanelet::autoware::AutowareTrafficLight::make(
        lanelet::utils::getId(), buildRegElemAttrs(sig, road), refers, stop_line_opt,
        lanelet::LineStrings3d{});

      const auto attached =
        attachToLanelets(road.id, section_idx, applicable, road, tl, lanelet_idx);
      if (attached == 0) {
        std::ostringstream oss;
        oss << "road " << road.id << " signal " << sig.id
            << ": no lanelets available to attach (all applicable lanes non-drivable or missing);"
            << " skipping";
        warn(errors, oss.str());
        continue;
      }

      map.add(static_cast<lanelet::RegulatoryElementPtr>(tl));
      regelem_by_signal_id.emplace(sig.id, tl);
    }
  }

  // Second pass: walk every `<signalReference>` and attach the previously
  // emitted reg element to lanelets on the referencing road, narrowed by the
  // reference's own `<validity>` (§5.12).
  for (const auto & road : doc.roads) {
    if (road.signals.signal_references.empty()) {
      continue;
    }

    for (const auto & ref : road.signals.signal_references) {
      const auto it = regelem_by_signal_id.find(ref.id);
      if (it == regelem_by_signal_id.end()) {
        // §7: dangling `<signalReference>` skipped with a warning.
        std::ostringstream oss;
        oss << "road " << road.id << " signalReference id='" << ref.id
            << "' not found in any road's signals; skipping";
        warn(errors, oss.str());
        continue;
      }
      if (road.lanes.lane_sections.empty()) {
        continue;
      }
      const auto section_idx = findSectionIdx(road, ref.s);
      const auto & section = road.lanes.lane_sections[section_idx];
      const auto applicable = applicableLaneIds(section, ref.validities, ref.orientation);
      if (applicable.empty()) {
        std::ostringstream oss;
        oss << "road " << road.id << " signalReference id='" << ref.id
            << "' has no applicable lanes at s=" << ref.s << "; skipping";
        warn(errors, oss.str());
        continue;
      }
      (void)attachToLanelets(road.id, section_idx, applicable, road, it->second, lanelet_idx);
    }
  }
}

}  // namespace lanelet::io_handlers::opendrive
