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

#include "lane_builder.hpp"

#include "geometry.hpp"
#include "xodr_types.hpp"

#include <lanelet2_core/primitives/Lanelet.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace lanelet::io_handlers::opendrive
{
namespace
{
constexpr const char * kErrPrefix = "[autoware_opendrive_handler]";

void warn(ErrorMessages & errors, const std::string & msg)
{
  errors.emplace_back(std::string{kErrPrefix} + " " + msg);
}

double evalPoly3Record(const Poly3Record & r, double s)
{
  const double ds = s - r.s;
  return r.a + r.b * ds + r.c * ds * ds + r.d * ds * ds * ds;
}

// Walk a sorted Poly3Record list with the piecewise-cubic convention the spec
// uses everywhere (§5.4, §5.5): before the first record, the first record is
// extrapolated back; for s inside a record's span, that record applies; past
// the last record, the last record is extrapolated forward.
//
// Returns 0 when the list is empty (caller's responsibility to decide if that
// is meaningful — e.g. WidthEvaluator treats it as "no width record" → 0).
double evalPiecewise(const std::vector<Poly3Record> & rs, double s)
{
  if (rs.empty()) {
    return 0.0;
  }
  std::size_t k = 0;
  for (std::size_t i = 1; i < rs.size(); ++i) {
    if (rs[i].s <= s) {
      k = i;
    } else {
      break;
    }
  }
  return evalPoly3Record(rs[k], s);
}

struct WorldPoint
{
  double x;
  double y;
  double z;
};

WorldPoint stToWorld(
  const ReferenceLineEvaluator & rle, const ElevationEvaluator & ele, double s, double t)
{
  const auto ref = rle.evaluate(s);
  const double z = ele.evaluate(s);
  // §5.6: t > 0 is to the left of +s. Superelevation is ignored.
  return WorldPoint{
    ref.x - std::sin(ref.hdg) * t, ref.y + std::cos(ref.hdg) * t, z};
}

// Collect sample s-values for a section. Includes the native stride,
// section endpoints, and all break points from the evaluators / width
// records (§5.7). Deduplicates and sorts ascending.
std::vector<double> collectBreakpoints(
  double s_start, double s_end, double sample_step_m,
  const ReferenceLineEvaluator & rle, const ElevationEvaluator & ele,
  const LaneOffsetEvaluator & loe, const LaneSection & section,
  const std::vector<WidthEvaluator> & right_widths,
  const std::vector<WidthEvaluator> & left_widths)
{
  std::vector<double> samples;
  // Fixed-step stride. The final stride bucket is truncated by adding s_end
  // explicitly below.
  const double span = s_end - s_start;
  const auto n_steps = static_cast<std::size_t>(std::floor(span / sample_step_m));
  samples.reserve(n_steps + 8);
  samples.push_back(s_start);
  for (std::size_t i = 1; i <= n_steps; ++i) {
    samples.push_back(s_start + static_cast<double>(i) * sample_step_m);
  }
  samples.push_back(s_end);

  auto add_in_range = [&](double s) {
    if (s > s_start && s < s_end) {
      samples.push_back(s);
    }
  };
  for (double bp : rle.breakpoints()) add_in_range(bp);
  for (double bp : ele.breakpoints()) add_in_range(bp);
  for (double bp : loe.breakpoints()) add_in_range(bp);
  // Width breakpoints are expressed in s_section → convert to absolute.
  for (const auto & w : right_widths) {
    for (double bp : w.breakpoints()) add_in_range(s_start + bp);
  }
  for (const auto & w : left_widths) {
    for (double bp : w.breakpoints()) add_in_range(s_start + bp);
  }
  (void)section;

  std::sort(samples.begin(), samples.end());
  constexpr double kDedupEps = 1e-9;
  samples.erase(
    std::unique(
      samples.begin(), samples.end(),
      [](double a, double b) { return std::fabs(a - b) < kDedupEps; }),
    samples.end());
  return samples;
}

// Subtype + drivability per §5.8. Returns empty subtype when no lanelet
// should be emitted for this lane type (caller still uses the boundary
// linestring, which has already been built at this point).
std::string classifyLaneSubtype(
  const Lane & lane, const LaneBuilderOptions & opts, const std::string & road_id,
  std::size_t section_idx, ErrorMessages & errors)
{
  if (lane.type == "driving") {
    return "road";
  }
  if (lane.type == "biking") {
    return "bicycle_lane";
  }
  if (lane.type == "shoulder") {
    return "road_shoulder";
  }
  if (lane.type == "sidewalk") {
    if (opts.skip_sidewalks) {
      return "";
    }
    // v1 has no in-tree semantics for a non-skipped sidewalk lanelet — §5.8
    // explicitly defers this. The flag exists for future PRs; for now we
    // still produce boundaries only.
    return "";
  }
  // restricted, parking, median, border, stop, none, etc.
  std::ostringstream oss;
  oss << "road " << road_id << " section " << section_idx << " lane " << lane.id
      << ": type '" << lane.type << "' is not supported in v1; boundary built, no lanelet emitted";
  warn(errors, oss.str());
  return "";
}

lanelet::LineString3d buildLineString(const std::vector<lanelet::Point3d> & points)
{
  lanelet::LineString3d ls{lanelet::utils::getId()};
  ls.reserve(points.size());
  for (const auto & p : points) {
    ls.push_back(p);
  }
  return ls;
}

void emitLaneletIfDrivable(
  const Road & road, std::size_t section_idx, const Lane & lane,
  const lanelet::LineString3d & inner_ls, const lanelet::LineString3d & outer_ls,
  bool is_left, const LaneBuilderOptions & opts, lanelet::LaneletMap & map,
  ErrorMessages & errors)
{
  const auto subtype = classifyLaneSubtype(lane, opts, road.id, section_idx, errors);
  if (subtype.empty()) {
    return;
  }

  // §5.8 bound direction. Left-side lanes are driven in -s, so their bounds
  // run in descending s. LineString3d::invert() shares the same underlying
  // data (and Id), preserving the shared-Id guarantee of §5.11.
  const auto left_bound = is_left ? inner_ls.invert() : inner_ls;
  const auto right_bound = is_left ? outer_ls.invert() : outer_ls;

  lanelet::Lanelet ll{lanelet::utils::getId(), left_bound, right_bound};
  ll.setAttribute("type", "lanelet");
  ll.setAttribute("subtype", subtype);
  ll.setAttribute("location", "urban");
  ll.setAttribute("one_way", "yes");
  ll.setAttribute("opendrive:road_id", road.id);
  ll.setAttribute("opendrive:lane_section", std::to_string(section_idx));
  ll.setAttribute("opendrive:lane_id", std::to_string(lane.id));
  if (road.junction != "-1" && !road.junction.empty()) {
    ll.setAttribute("opendrive:junction_id", road.junction);
  }
  map.add(ll);
}

void buildSection(
  const Road & road, std::size_t section_idx, double s_end,
  const ReferenceLineEvaluator & rle, const ElevationEvaluator & ele,
  const LaneOffsetEvaluator & loe, const LaneBuilderOptions & opts,
  Point3dDeduper & deduper, lanelet::LaneletMap & map, ErrorMessages & errors)
{
  const auto & section = road.lanes.lane_sections[section_idx];
  const double s_start = section.s;
  if (s_end <= s_start) {
    return;  // degenerate section — nothing to sample.
  }

  std::vector<WidthEvaluator> right_widths;
  right_widths.reserve(section.right.size());
  for (const auto & lane : section.right) {
    right_widths.emplace_back(lane.widths);
  }
  std::vector<WidthEvaluator> left_widths;
  left_widths.reserve(section.left.size());
  for (const auto & lane : section.left) {
    left_widths.emplace_back(lane.widths);
  }

  const auto s_samples = collectBreakpoints(
    s_start, s_end, opts.sample_step_m, rle, ele, loe, section, right_widths, left_widths);

  std::vector<lanelet::Point3d> center_points;
  center_points.reserve(s_samples.size());
  std::vector<std::vector<lanelet::Point3d>> right_points(right_widths.size());
  std::vector<std::vector<lanelet::Point3d>> left_points(left_widths.size());
  for (auto & v : right_points) v.reserve(s_samples.size());
  for (auto & v : left_points) v.reserve(s_samples.size());

  for (double s : s_samples) {
    const double t_center = loe.evaluate(s);
    const double s_sec = s - s_start;

    {
      const auto wp = stToWorld(rle, ele, s, t_center);
      center_points.push_back(deduper.getOrCreate(wp.x, wp.y, wp.z));
    }

    // Right lanes walk outward: t_outer(k) = t_center − Σ_{j=1..k} w_j.
    double cumul = 0.0;
    for (std::size_t k = 0; k < right_widths.size(); ++k) {
      // Widths can evaluate slightly negative near cubic cusps; clamp to zero.
      const double w = std::max(0.0, right_widths[k].evaluate(s_sec));
      cumul += w;
      const auto wp = stToWorld(rle, ele, s, t_center - cumul);
      right_points[k].push_back(deduper.getOrCreate(wp.x, wp.y, wp.z));
    }

    // Left lanes, symmetric.
    cumul = 0.0;
    for (std::size_t k = 0; k < left_widths.size(); ++k) {
      const double w = std::max(0.0, left_widths[k].evaluate(s_sec));
      cumul += w;
      const auto wp = stToWorld(rle, ele, s, t_center + cumul);
      left_points[k].push_back(deduper.getOrCreate(wp.x, wp.y, wp.z));
    }
  }

  const auto center_ls = buildLineString(center_points);
  std::vector<lanelet::LineString3d> right_ls;
  right_ls.reserve(right_widths.size());
  for (const auto & pts : right_points) {
    right_ls.push_back(buildLineString(pts));
  }
  std::vector<lanelet::LineString3d> left_ls;
  left_ls.reserve(left_widths.size());
  for (const auto & pts : left_points) {
    left_ls.push_back(buildLineString(pts));
  }

  for (std::size_t k = 0; k < section.right.size(); ++k) {
    const auto inner = (k == 0) ? center_ls : right_ls[k - 1];
    const auto outer = right_ls[k];
    emitLaneletIfDrivable(
      road, section_idx, section.right[k], inner, outer, /*is_left=*/false, opts, map, errors);
  }
  for (std::size_t k = 0; k < section.left.size(); ++k) {
    const auto inner = (k == 0) ? center_ls : left_ls[k - 1];
    const auto outer = left_ls[k];
    emitLaneletIfDrivable(
      road, section_idx, section.left[k], inner, outer, /*is_left=*/true, opts, map, errors);
  }
}
}  // namespace

LaneOffsetEvaluator::LaneOffsetEvaluator(std::vector<Poly3Record> records)
: records_{std::move(records)}
{
  if (records_.size() > 1) {
    breakpoints_.reserve(records_.size() - 1);
    for (std::size_t i = 1; i < records_.size(); ++i) {
      breakpoints_.push_back(records_[i].s);
    }
  }
}

double LaneOffsetEvaluator::evaluate(double s) const
{
  return evalPiecewise(records_, s);
}

WidthEvaluator::WidthEvaluator(std::vector<Poly3Record> records) : records_{std::move(records)}
{
  if (records_.size() > 1) {
    breakpoints_.reserve(records_.size() - 1);
    for (std::size_t i = 1; i < records_.size(); ++i) {
      breakpoints_.push_back(records_[i].s);
    }
  }
}

double WidthEvaluator::evaluate(double s_section) const
{
  return evalPiecewise(records_, s_section);
}

std::size_t Point3dDeduper::KeyHash::operator()(const Key & k) const noexcept
{
  // 64-bit mix à la boost::hash_combine. Good enough for the dedup workload;
  // points tend to be spread out spatially, so collisions are rare.
  auto mix = [](std::size_t seed, std::int64_t v) {
    const auto uv = static_cast<std::uint64_t>(v);
    return seed ^ (uv + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2));
  };
  std::size_t h = 0;
  h = mix(h, k.ix);
  h = mix(h, k.iy);
  h = mix(h, k.iz);
  return h;
}

Point3dDeduper::Point3dDeduper(double merge_tol_m) : eps_{merge_tol_m}
{
  // A non-positive tolerance would put every point in cell (0, 0, 0) and
  // collapse the whole map into one point. Clamp to a very small positive
  // value to preserve dedup semantics even if misconfigured.
  if (eps_ <= 0.0) {
    eps_ = 1e-9;
  }
}

lanelet::Point3d Point3dDeduper::getOrCreate(double x, double y, double z)
{
  const auto quantize = [this](double v) {
    return static_cast<std::int64_t>(std::llround(v / eps_));
  };
  const Key k{quantize(x), quantize(y), quantize(z)};
  const auto it = map_.find(k);
  if (it != map_.end()) {
    return it->second;
  }
  lanelet::Point3d p{lanelet::utils::getId(), x, y, z};
  map_.emplace(k, p);
  return p;
}

void buildRoadLanelets(
  const Road & road, const LaneBuilderOptions & opts, Point3dDeduper & deduper,
  lanelet::LaneletMap & map, ErrorMessages & errors)
{
  if (road.plan_view.geometries.empty() || road.lanes.lane_sections.empty()) {
    return;
  }

  ReferenceLineEvaluator rle{road.plan_view};
  ElevationEvaluator ele{road.elevation_profile};
  LaneOffsetEvaluator loe{road.lanes.lane_offsets};

  const auto & sections = road.lanes.lane_sections;
  for (std::size_t i = 0; i < sections.size(); ++i) {
    const double s_end =
      (i + 1 < sections.size()) ? sections[i + 1].s : road.length;
    buildSection(road, i, s_end, rle, ele, loe, opts, deduper, map, errors);
  }
}

}  // namespace lanelet::io_handlers::opendrive
