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

#ifndef OPENDRIVE__GEOMETRY_HPP_
#define OPENDRIVE__GEOMETRY_HPP_

#include "xodr_types.hpp"

#include <cstddef>
#include <vector>

namespace lanelet::io_handlers::opendrive
{
struct PlanarPose
{
  double x{0.0};
  double y{0.0};
  double hdg{0.0};  // radians, global frame
};

// Evaluate a road's reference line (§5.3). Given absolute s along the road,
// returns (x, y, hdg) in the world frame. s outside [0, road.length] clamps
// to the nearest endpoint and emits no warning (callers should not query).
class ReferenceLineEvaluator
{
public:
  // The evaluator takes its own copy of the PlanView — paramPoly3 tables are
  // derived from it at construction and must stay in sync.
  explicit ReferenceLineEvaluator(PlanView plan_view);

  PlanarPose evaluate(double s) const;

  // Total reference-line length inferred from the last geometry record.
  double length() const { return length_; }

  // Boundary s-values where geometry records meet (sorted ascending, unique).
  // Callers need these for §5.7 break-point sampling.
  const std::vector<double> & breakpoints() const { return breakpoints_; }

private:
  // Per-paramPoly3 arc-length tabulation (s along the primitive → parameter p).
  struct ParamPoly3Table
  {
    std::vector<double> s_samples;  // cumulative arc length, 0 … length
    std::vector<double> p_samples;  // matching parameter, 0 … p_max
  };

  PlanView plan_view_;
  double length_{0.0};
  std::vector<double> breakpoints_;
  // Parallel to plan_view_.geometries: empty entries for non-paramPoly3
  // primitives, populated tables for paramPoly3.
  std::vector<ParamPoly3Table> param_poly3_tables_;

  std::size_t findRecord(double s) const;
  PlanarPose evaluateRecord(const Geometry & g, double ds, std::size_t idx) const;
  static ParamPoly3Table tabulateParamPoly3(const Geometry & g, const GeomParamPoly3 & p);
};

// Elevation evaluator (§5.4). Piecewise cubic z(s). For s < first.s, returns
// 0. For s > last.s + L, extrapolates the last polynomial.
class ElevationEvaluator
{
public:
  explicit ElevationEvaluator(ElevationProfile profile);

  double evaluate(double s) const;

  // Boundary s-values between piecewise records (the `s` field of each record
  // beyond the first). Useful for §5.7 break-point sampling.
  const std::vector<double> & breakpoints() const { return breakpoints_; }

private:
  ElevationProfile profile_;
  std::vector<double> breakpoints_;
};

}  // namespace lanelet::io_handlers::opendrive

#endif  // OPENDRIVE__GEOMETRY_HPP_
