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

#include "geometry.hpp"

#include "fresnel.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace lanelet::io_handlers::opendrive
{
namespace
{
constexpr double kPi = 3.141592653589793;

// Curvature below this magnitude lets us treat an arc as a line. The
// threshold is a trade-off between accuracy (the closed-form arc formula has
// a removable singularity at κ → 0) and robustness on authored data where
// "straight" arcs carry residual κ ~1e-12 from FP conversion.
constexpr double kArcDegenerateCurvature = 1e-12;

// Similarly, a spiral with near-constant curvature is treated as an arc.
constexpr double kSpiralDegenerateCurvatureRate = 1e-12;

double evalPoly3(const Poly3Record & r, double s)
{
  const double ds = s - r.s;
  return r.a + r.b * ds + r.c * ds * ds + r.d * ds * ds * ds;
}

PlanarPose evaluateLine(const Geometry & g, double ds)
{
  return PlanarPose{g.x + ds * std::cos(g.hdg), g.y + ds * std::sin(g.hdg), g.hdg};
}

PlanarPose evaluateArc(const Geometry & g, const GeomArc & arc, double ds)
{
  if (std::fabs(arc.curvature) < kArcDegenerateCurvature) {
    return evaluateLine(g, ds);
  }
  const double k = arc.curvature;
  const double theta = g.hdg + k * ds;
  // Closed-form integration of (cos θ(σ), sin θ(σ)) with θ(σ) = hdg + κσ:
  //   ∫ cos → ( sin θ - sin hdg) / κ
  //   ∫ sin → (-cos θ + cos hdg) / κ
  const double x = g.x + (std::sin(theta) - std::sin(g.hdg)) / k;
  const double y = g.y + (-std::cos(theta) + std::cos(g.hdg)) / k;
  return PlanarPose{x, y, theta};
}

PlanarPose evaluateSpiral(const Geometry & g, const GeomSpiral & sp, double ds)
{
  const double k0 = sp.curv_start;
  const double k1 = sp.curv_end;
  const double k_rate = (k1 - k0) / g.length;  // dκ/ds (linear in s)

  // θ(ds) = hdg + k0·ds + ½·k_rate·ds² (analytic, always exact).
  const double theta = g.hdg + k0 * ds + 0.5 * k_rate * ds * ds;

  // Degenerate: curvature effectively constant → arc with κ = k0.
  if (std::fabs(k_rate) < kSpiralDegenerateCurvatureRate) {
    GeomArc arc_equiv;
    arc_equiv.curvature = k0;
    return evaluateArc(g, arc_equiv, ds);
  }

  // Transform the integrals ∫₀^ds cos(θ(σ)) dσ and ∫₀^ds sin(θ(σ)) dσ
  // into Fresnel integrals via completing the square:
  //   θ(σ) = A + (k_rate/2) · (σ + c)²,
  //   c    = k0 / k_rate,
  //   A    = hdg − k0² / (2·k_rate).
  // Then u = σ + c, du = dσ, and substitute t = u · √(|k_rate|/π).
  const double c = k0 / k_rate;
  const double A = g.hdg - k0 * k0 / (2.0 * k_rate);

  const double sgn = (k_rate > 0.0) ? 1.0 : -1.0;
  const double alpha = std::sqrt(std::fabs(k_rate) / kPi);
  const double scale = 1.0 / alpha;  // √(π / |k_rate|) — the du/dt factor.

  const double t0 = c * alpha;
  const double t1 = (ds + c) * alpha;

  double s0 = 0.0;
  double c0 = 0.0;
  double s1 = 0.0;
  double c1 = 0.0;
  fresnel(t0, s0, c0);
  fresnel(t1, s1, c1);

  // Note: cos(k_rate/2 · u²) = cos((π/2) · (αu)²) regardless of sgn, because
  // cos is even. sin picks up the sign.
  const double dC = (c1 - c0) * scale;
  const double dS = (s1 - s0) * scale * sgn;

  const double dx = std::cos(A) * dC - std::sin(A) * dS;
  const double dy = std::sin(A) * dC + std::cos(A) * dS;

  return PlanarPose{g.x + dx, g.y + dy, theta};
}

PlanarPose evaluatePoly3(const Geometry & g, const GeomPoly3 & p, double ds)
{
  // In the record's local uv frame, u = ds (approximately arc length — the
  // OpenDRIVE spec treats `length` as the u-range for poly3 and assumes low
  // curvature), v = a + b·u + c·u² + d·u³.
  const double u = ds;
  const double v = p.a + p.b * u + p.c * u * u + p.d * u * u * u;
  const double dvdu = p.b + 2.0 * p.c * u + 3.0 * p.d * u * u;

  // Rotate (u, v) by hdg and translate.
  const double ch = std::cos(g.hdg);
  const double sh = std::sin(g.hdg);
  return PlanarPose{g.x + u * ch - v * sh, g.y + u * sh + v * ch, g.hdg + std::atan(dvdu)};
}

PlanarPose evaluateParamPoly3At(const Geometry & g, const GeomParamPoly3 & p, double param)
{
  const double u = p.a_u + p.b_u * param + p.c_u * param * param +
                   p.d_u * param * param * param;
  const double v = p.a_v + p.b_v * param + p.c_v * param * param +
                   p.d_v * param * param * param;
  const double dudp = p.b_u + 2.0 * p.c_u * param + 3.0 * p.d_u * param * param;
  const double dvdp = p.b_v + 2.0 * p.c_v * param + 3.0 * p.d_v * param * param;

  const double ch = std::cos(g.hdg);
  const double sh = std::sin(g.hdg);
  // atan2 over atan so a vertical tangent (dudp = 0) stays well-defined and
  // the heading follows the correct quadrant.
  return PlanarPose{
    g.x + u * ch - v * sh, g.y + u * sh + v * ch, g.hdg + std::atan2(dvdp, dudp)};
}
}  // namespace

ReferenceLineEvaluator::ReferenceLineEvaluator(PlanView plan_view)
: plan_view_{std::move(plan_view)}
{
  if (plan_view_.geometries.empty()) {
    return;
  }
  param_poly3_tables_.resize(plan_view_.geometries.size());
  breakpoints_.reserve(plan_view_.geometries.size() + 1);
  for (std::size_t i = 0; i < plan_view_.geometries.size(); ++i) {
    const auto & g = plan_view_.geometries[i];
    if (i > 0) {
      breakpoints_.push_back(g.s);
    }
    if (std::holds_alternative<GeomParamPoly3>(g.primitive)) {
      param_poly3_tables_[i] = tabulateParamPoly3(g, std::get<GeomParamPoly3>(g.primitive));
    }
  }
  const auto & last = plan_view_.geometries.back();
  length_ = last.s + last.length;
  breakpoints_.push_back(length_);
}

PlanarPose ReferenceLineEvaluator::evaluate(double s) const
{
  if (plan_view_.geometries.empty()) {
    return PlanarPose{};
  }
  // Clamp to the reference-line domain. Callers that need to know they went
  // past the end can consult length(); evaluate() itself is total.
  s = std::clamp(s, 0.0, length_);
  const auto idx = findRecord(s);
  const auto & g = plan_view_.geometries[idx];
  const double ds = std::clamp(s - g.s, 0.0, g.length);
  return evaluateRecord(g, ds, idx);
}

std::size_t ReferenceLineEvaluator::findRecord(double s) const
{
  // Find the last geometry record with start s ≤ query s. Binary search over
  // the sorted-by-s vector; the record's span is [g.s, g.s + g.length].
  const auto & gs = plan_view_.geometries;
  std::size_t lo = 0;
  std::size_t hi = gs.size();
  while (hi - lo > 1) {
    const std::size_t mid = lo + (hi - lo) / 2;
    if (gs[mid].s <= s) {
      lo = mid;
    } else {
      hi = mid;
    }
  }
  return lo;
}

PlanarPose ReferenceLineEvaluator::evaluateRecord(
  const Geometry & g, double ds, std::size_t idx) const
{
  if (std::holds_alternative<GeomLine>(g.primitive)) {
    return evaluateLine(g, ds);
  }
  if (std::holds_alternative<GeomArc>(g.primitive)) {
    return evaluateArc(g, std::get<GeomArc>(g.primitive), ds);
  }
  if (std::holds_alternative<GeomSpiral>(g.primitive)) {
    return evaluateSpiral(g, std::get<GeomSpiral>(g.primitive), ds);
  }
  if (std::holds_alternative<GeomPoly3>(g.primitive)) {
    return evaluatePoly3(g, std::get<GeomPoly3>(g.primitive), ds);
  }
  const auto & pp = std::get<GeomParamPoly3>(g.primitive);
  const auto & table = param_poly3_tables_[idx];
  // Table never ends up empty for paramPoly3 (zero-length records are rejected
  // at read time); if somehow it is, fall back to a line.
  if (table.s_samples.empty()) {
    return evaluateLine(g, ds);
  }
  // Map ds → parameter p via linear interpolation over the pre-tabulated
  // cumulative-arc-length array.
  const auto & ss = table.s_samples;
  auto it = std::upper_bound(ss.begin(), ss.end(), ds);
  if (it == ss.begin()) {
    return evaluateParamPoly3At(g, pp, table.p_samples.front());
  }
  if (it == ss.end()) {
    return evaluateParamPoly3At(g, pp, table.p_samples.back());
  }
  const std::size_t k = std::distance(ss.begin(), it) - 1;
  const double s0 = ss[k];
  const double s1 = ss[k + 1];
  const double frac = (s1 > s0) ? (ds - s0) / (s1 - s0) : 0.0;
  const double p = table.p_samples[k] + frac * (table.p_samples[k + 1] - table.p_samples[k]);
  return evaluateParamPoly3At(g, pp, p);
}

ReferenceLineEvaluator::ParamPoly3Table ReferenceLineEvaluator::tabulateParamPoly3(
  const Geometry & g, const GeomParamPoly3 & p)
{
  // Fixed-step cumulative trapezoidal integration of |du/dp, dv/dp|. 2000
  // sub-intervals: per §5.3 the tabulation step sets the arc-length-mapping
  // error bound, default ~1e-3 of the primitive length; 2000 points on a
  // typical ~50 m record puts mapping error below a millimetre.
  constexpr int kN = 2000;
  const double p_max = p.p_range_normalized ? 1.0 : g.length;

  ParamPoly3Table table;
  table.s_samples.resize(kN + 1);
  table.p_samples.resize(kN + 1);

  const double dp = p_max / static_cast<double>(kN);

  auto speed = [&p](double q) {
    const double dudp = p.b_u + 2.0 * p.c_u * q + 3.0 * p.d_u * q * q;
    const double dvdp = p.b_v + 2.0 * p.c_v * q + 3.0 * p.d_v * q * q;
    return std::sqrt(dudp * dudp + dvdp * dvdp);
  };

  table.s_samples[0] = 0.0;
  table.p_samples[0] = 0.0;
  double prev_speed = speed(0.0);
  double s_cum = 0.0;
  for (int i = 1; i <= kN; ++i) {
    const double pi = static_cast<double>(i) * dp;
    const double cur_speed = speed(pi);
    s_cum += 0.5 * (prev_speed + cur_speed) * dp;
    prev_speed = cur_speed;
    table.p_samples[i] = pi;
    table.s_samples[i] = s_cum;
  }
  return table;
}

ElevationEvaluator::ElevationEvaluator(ElevationProfile profile) : profile_{std::move(profile)}
{
  if (profile_.records.size() > 1) {
    breakpoints_.reserve(profile_.records.size() - 1);
    for (std::size_t i = 1; i < profile_.records.size(); ++i) {
      breakpoints_.push_back(profile_.records[i].s);
    }
  }
}

double ElevationEvaluator::evaluate(double s) const
{
  if (profile_.records.empty()) {
    return 0.0;
  }
  const auto & r = profile_.records;
  if (s < r.front().s) {
    return 0.0;
  }
  // Last record with r[k].s <= s. Simple linear scan is fine: elevation
  // profiles rarely exceed a few records per road.
  std::size_t k = 0;
  for (std::size_t i = 1; i < r.size(); ++i) {
    if (r[i].s <= s) {
      k = i;
    } else {
      break;
    }
  }
  return evalPoly3(r[k], s);
}

}  // namespace lanelet::io_handlers::opendrive
