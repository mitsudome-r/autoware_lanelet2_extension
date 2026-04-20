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

#ifndef OPENDRIVE__LANE_BUILDER_HPP_
#define OPENDRIVE__LANE_BUILDER_HPP_

#include "geometry.hpp"
#include "xodr_types.hpp"

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_io/io_handlers/IoHandler.h>

#include <array>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace lanelet::io_handlers::opendrive
{
// Piecewise cubic evaluator for <laneOffset> records — the t-offset of lane 0
// from the road reference line. Before the first record, the first record is
// extrapolated back to s = 0; after the last record it is extrapolated forward
// (matches the evaluator's established convention for piecewise polynomials).
class LaneOffsetEvaluator
{
public:
  explicit LaneOffsetEvaluator(std::vector<Poly3Record> records);

  double evaluate(double s) const;

  const std::vector<double> & breakpoints() const { return breakpoints_; }

private:
  std::vector<Poly3Record> records_;
  std::vector<double> breakpoints_;
};

// Piecewise cubic evaluator for <lane>/<width> records. Each record's `s`
// field is sOffset, interpreted relative to the laneSection start — so this
// evaluator takes `s_section` (= absolute s − laneSection.s) rather than
// absolute s, per §5.5.
class WidthEvaluator
{
public:
  explicit WidthEvaluator(std::vector<Poly3Record> records);

  double evaluate(double s_section) const;

  // Breakpoints are expressed in s_section coordinates (sOffset values of the
  // 2nd … nth records).
  const std::vector<double> & breakpoints() const { return breakpoints_; }

private:
  std::vector<Poly3Record> records_;
  std::vector<double> breakpoints_;
};

// Spatial-hash deduper on a fixed-cell lattice (§5.11). A candidate point
// (x, y, z) is quantized with `round(coord / ε)` and looked up — if a
// previously issued Point3d falls in the same cell, its Id is returned.
// Otherwise a new Point3d is created with a fresh lanelet2 Id.
//
// The lattice-only lookup is simpler and cheaper than a radius query, and the
// spec (§5.11) explicitly specifies this form. A corner case is that two
// points lying just across a cell boundary can be slightly more than ε apart
// and still not merge — acceptable because:
//   1. ε is the merge tolerance, not a guaranteed merge radius, and
//   2. within-section points always come from the same sample grid, so they
//      trivially share cells; cross-section / cross-road endpoint pairs are
//      already near-identical in well-authored maps.
class Point3dDeduper
{
public:
  explicit Point3dDeduper(double merge_tol_m);

  // Returns an existing Point3d at (x, y, z) if the dedup lattice already has
  // one within ε, otherwise mints a new Point3d with a fresh Id.
  lanelet::Point3d getOrCreate(double x, double y, double z);

private:
  struct Key
  {
    std::int64_t ix;
    std::int64_t iy;
    std::int64_t iz;
    bool operator==(const Key & o) const noexcept
    {
      return ix == o.ix && iy == o.iy && iz == o.iz;
    }
  };

  struct KeyHash
  {
    std::size_t operator()(const Key & k) const noexcept;
  };

  double eps_;
  std::unordered_map<Key, lanelet::Point3d, KeyHash> map_;
};

// Options controlling lane building. Values mirror the `autoware_opendrive/*`
// config knobs in §6.1, which are wired up in Phase 6.
struct LaneBuilderOptions
{
  double sample_step_m{0.5};
  double merge_tol_m{0.01};
  bool skip_sidewalks{true};
};

// Build one road's lanelets into `map` per §5.5–§5.8, §5.11. Adjacent lanes
// in the same laneSection share LineString3d Ids for their common boundary;
// adjacent laneSections share Point3d Ids at the section boundary (both via
// `deduper` and, for the connecting linestrings, via explicit endpoint reuse).
void buildRoadLanelets(
  const Road & road, const LaneBuilderOptions & opts, Point3dDeduper & deduper,
  lanelet::LaneletMap & map, lanelet::ErrorMessages & errors);

}  // namespace lanelet::io_handlers::opendrive

#endif  // OPENDRIVE__LANE_BUILDER_HPP_
