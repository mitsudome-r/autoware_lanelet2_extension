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

// Phase 2 unit tests per §9.1 of the OpenDRIVE parser plan: closed-form checks
// of each reference-line primitive (tolerance 1e-4 m), elevation continuity,
// spiral-as-degenerate-arc, and paramPoly3 arc-length monotonicity.

#include "../../lib/opendrive/fresnel.hpp"
#include "../../lib/opendrive/geometry.hpp"
#include "../../lib/opendrive/xodr_types.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

using lanelet::io_handlers::opendrive::ElevationEvaluator;
using lanelet::io_handlers::opendrive::ElevationProfile;
using lanelet::io_handlers::opendrive::fresnel;
using lanelet::io_handlers::opendrive::GeomArc;
using lanelet::io_handlers::opendrive::Geometry;
using lanelet::io_handlers::opendrive::GeomLine;
using lanelet::io_handlers::opendrive::GeomParamPoly3;
using lanelet::io_handlers::opendrive::GeomPoly3;
using lanelet::io_handlers::opendrive::GeomSpiral;
using lanelet::io_handlers::opendrive::PlanView;
using lanelet::io_handlers::opendrive::Poly3Record;
using lanelet::io_handlers::opendrive::ReferenceLineEvaluator;

namespace
{
constexpr double kTol = 1e-4;

PlanView singleGeometry(const Geometry & g)
{
  PlanView pv;
  pv.geometries.push_back(g);
  return pv;
}
}  // namespace

TEST(Fresnel, KnownValues)
{
  // Spot-checks against tabulated values of Fresnel integrals with the π/2
  // normalization (e.g., Abramowitz & Stegun Table 7.7).
  double s = 0.0;
  double c = 0.0;

  fresnel(0.0, s, c);
  EXPECT_NEAR(s, 0.0, 1e-7);
  EXPECT_NEAR(c, 0.0, 1e-7);

  fresnel(1.0, s, c);
  EXPECT_NEAR(c, 0.7798934003768228, 1e-6);
  EXPECT_NEAR(s, 0.4382591473903548, 1e-6);

  fresnel(2.0, s, c);
  EXPECT_NEAR(c, 0.4882534060753407, 1e-6);
  EXPECT_NEAR(s, 0.3434156783636982, 1e-6);

  // Asymptotic limit: C(∞) = S(∞) = 0.5. At x = 5 we should already be close.
  fresnel(5.0, s, c);
  EXPECT_NEAR(c, 0.5636311887, 1e-6);
  EXPECT_NEAR(s, 0.4991913819, 1e-6);

  // Odd symmetry.
  double s_neg = 0.0;
  double c_neg = 0.0;
  fresnel(-1.3, s_neg, c_neg);
  fresnel(1.3, s, c);
  EXPECT_NEAR(s_neg, -s, 1e-10);
  EXPECT_NEAR(c_neg, -c, 1e-10);
}

TEST(ReferenceLineEvaluator, Line)
{
  Geometry g;
  g.s = 0.0;
  g.x = 10.0;
  g.y = 20.0;
  g.hdg = M_PI / 6.0;  // 30°
  g.length = 50.0;
  g.primitive = GeomLine{};

  ReferenceLineEvaluator eval{singleGeometry(g)};
  EXPECT_NEAR(eval.length(), 50.0, 1e-12);

  for (double ds : {0.0, 5.0, 25.0, 49.9}) {
    const auto p = eval.evaluate(ds);
    EXPECT_NEAR(p.x, g.x + ds * std::cos(g.hdg), kTol);
    EXPECT_NEAR(p.y, g.y + ds * std::sin(g.hdg), kTol);
    EXPECT_NEAR(p.hdg, g.hdg, 1e-12);
  }
}

TEST(ReferenceLineEvaluator, ArcQuarterCircle)
{
  // Unit-radius arc starting at origin heading +x. After 90° the endpoint is
  // (R, R) heading +y. κ = 1/R with R = 10.
  constexpr double R = 10.0;
  constexpr double kappa = 1.0 / R;
  constexpr double quarter = 0.5 * M_PI * R;  // arc length of a quarter circle

  Geometry g;
  g.s = 0.0;
  g.x = 0.0;
  g.y = 0.0;
  g.hdg = 0.0;
  g.length = quarter;
  GeomArc arc;
  arc.curvature = kappa;
  g.primitive = arc;

  ReferenceLineEvaluator eval{singleGeometry(g)};
  const auto p = eval.evaluate(quarter);
  EXPECT_NEAR(p.x, R, kTol);
  EXPECT_NEAR(p.y, R, kTol);
  EXPECT_NEAR(p.hdg, M_PI / 2.0, kTol);

  // Midpoint: 45° along the quarter circle.
  const auto mid = eval.evaluate(quarter / 2.0);
  EXPECT_NEAR(mid.x, R * std::sin(M_PI / 4.0), kTol);
  EXPECT_NEAR(mid.y, R * (1.0 - std::cos(M_PI / 4.0)), kTol);
  EXPECT_NEAR(mid.hdg, M_PI / 4.0, kTol);
}

TEST(ReferenceLineEvaluator, ArcDegeneratesToLine)
{
  // κ ≈ 0 should fall through to the line formula without blowing up.
  Geometry g;
  g.s = 0.0;
  g.x = 1.0;
  g.y = 2.0;
  g.hdg = 1.0;
  g.length = 30.0;
  GeomArc arc;
  arc.curvature = 1e-15;
  g.primitive = arc;

  ReferenceLineEvaluator eval{singleGeometry(g)};
  const auto p = eval.evaluate(15.0);
  EXPECT_NEAR(p.x, g.x + 15.0 * std::cos(g.hdg), kTol);
  EXPECT_NEAR(p.y, g.y + 15.0 * std::sin(g.hdg), kTol);
  EXPECT_NEAR(p.hdg, g.hdg, kTol);
}

TEST(ReferenceLineEvaluator, SpiralAsDegenerateArc)
{
  // curvStart == curvEnd — spiral reduces to an arc. Compare against the
  // closed-form arc evaluator at several points.
  constexpr double kappa = 0.05;
  constexpr double length = 20.0;

  Geometry g_spiral;
  g_spiral.s = 0.0;
  g_spiral.x = 3.0;
  g_spiral.y = -1.0;
  g_spiral.hdg = 0.2;
  g_spiral.length = length;
  GeomSpiral sp;
  sp.curv_start = kappa;
  sp.curv_end = kappa;
  g_spiral.primitive = sp;

  Geometry g_arc = g_spiral;
  GeomArc arc;
  arc.curvature = kappa;
  g_arc.primitive = arc;

  ReferenceLineEvaluator eval_spiral{singleGeometry(g_spiral)};
  ReferenceLineEvaluator eval_arc{singleGeometry(g_arc)};

  for (double ds : {0.0, 5.0, 10.0, 15.0, 20.0}) {
    const auto ps = eval_spiral.evaluate(ds);
    const auto pa = eval_arc.evaluate(ds);
    EXPECT_NEAR(ps.x, pa.x, kTol);
    EXPECT_NEAR(ps.y, pa.y, kTol);
    EXPECT_NEAR(ps.hdg, pa.hdg, kTol);
  }
}

TEST(ReferenceLineEvaluator, SpiralZeroCurvatureIsLine)
{
  // curvStart == curvEnd == 0 is a straight line regardless of length.
  Geometry g;
  g.s = 0.0;
  g.x = 0.0;
  g.y = 0.0;
  g.hdg = M_PI / 3.0;
  g.length = 25.0;
  GeomSpiral sp;
  sp.curv_start = 0.0;
  sp.curv_end = 0.0;
  g.primitive = sp;

  ReferenceLineEvaluator eval{singleGeometry(g)};
  for (double ds : {0.0, 7.5, 15.0, 25.0}) {
    const auto p = eval.evaluate(ds);
    EXPECT_NEAR(p.x, ds * std::cos(g.hdg), kTol);
    EXPECT_NEAR(p.y, ds * std::sin(g.hdg), kTol);
    EXPECT_NEAR(p.hdg, g.hdg, kTol);
  }
}

TEST(ReferenceLineEvaluator, SpiralEndHeadingAndContinuity)
{
  // Exact end heading for a linear-curvature clothoid: θ(L) = hdg + κ₀L + ½(κ₁−κ₀)L.
  // Also verifies internal continuity with a dense sampling (no jumps).
  Geometry g;
  g.s = 0.0;
  g.x = 0.0;
  g.y = 0.0;
  g.hdg = 0.0;
  g.length = 20.0;
  GeomSpiral sp;
  sp.curv_start = 0.0;
  sp.curv_end = 0.1;
  g.primitive = sp;

  ReferenceLineEvaluator eval{singleGeometry(g)};
  const auto end = eval.evaluate(20.0);
  const double expected_hdg = 0.0 + 0.0 * 20.0 + 0.5 * (0.1 - 0.0) * 20.0;
  EXPECT_NEAR(end.hdg, expected_hdg, kTol);

  // Sampling continuity: no point should jump more than the arc-length step
  // would permit.
  constexpr double step = 0.1;
  auto prev = eval.evaluate(0.0);
  for (double s = step; s <= 20.0 + 1e-9; s += step) {
    const auto cur = eval.evaluate(s);
    const double dist = std::hypot(cur.x - prev.x, cur.y - prev.y);
    EXPECT_LE(dist, step + 1e-6);
    prev = cur;
  }
}

TEST(ReferenceLineEvaluator, Poly3)
{
  // v(u) = 0.1 u + 0.01 u²; record at origin, heading 0 (identity rotation).
  Geometry g;
  g.s = 0.0;
  g.x = 0.0;
  g.y = 0.0;
  g.hdg = 0.0;
  g.length = 10.0;
  GeomPoly3 p;
  p.a = 0.0;
  p.b = 0.1;
  p.c = 0.01;
  p.d = 0.0;
  g.primitive = p;

  ReferenceLineEvaluator eval{singleGeometry(g)};
  for (double ds : {0.0, 2.5, 5.0, 7.5, 10.0}) {
    const auto pose = eval.evaluate(ds);
    const double v_expected = 0.1 * ds + 0.01 * ds * ds;
    const double dvdu_expected = 0.1 + 0.02 * ds;
    EXPECT_NEAR(pose.x, ds, kTol);
    EXPECT_NEAR(pose.y, v_expected, kTol);
    EXPECT_NEAR(pose.hdg, std::atan(dvdu_expected), kTol);
  }
}

TEST(ReferenceLineEvaluator, Poly3WithRotation)
{
  // Non-zero hdg: the local (u, v) offset must be rotated into world frame.
  Geometry g;
  g.s = 0.0;
  g.x = 2.0;
  g.y = -3.0;
  g.hdg = M_PI / 4.0;
  g.length = 10.0;
  GeomPoly3 p;
  p.a = 0.0;
  p.b = 0.0;
  p.c = 0.02;  // v = 0.02 u²
  p.d = 0.0;
  g.primitive = p;

  ReferenceLineEvaluator eval{singleGeometry(g)};
  const double ds = 5.0;
  const auto pose = eval.evaluate(ds);
  const double u = ds;
  const double v = 0.02 * u * u;
  const double ch = std::cos(g.hdg);
  const double sh = std::sin(g.hdg);
  EXPECT_NEAR(pose.x, g.x + u * ch - v * sh, kTol);
  EXPECT_NEAR(pose.y, g.y + u * sh + v * ch, kTol);
}

TEST(ReferenceLineEvaluator, ParamPoly3Line)
{
  // u(p) = 10p, v(p) = 0 — a 10 m straight line parameterized in [0, 1].
  // Arc-length map must be exactly linear: p(s) = s / 10.
  Geometry g;
  g.s = 0.0;
  g.x = 0.0;
  g.y = 0.0;
  g.hdg = 0.0;
  g.length = 10.0;
  GeomParamPoly3 p;
  p.a_u = 0.0;
  p.b_u = 10.0;
  p.a_v = 0.0;
  p.p_range_normalized = true;
  g.primitive = p;

  ReferenceLineEvaluator eval{singleGeometry(g)};
  for (double ds : {0.0, 2.5, 5.0, 7.5, 10.0}) {
    const auto pose = eval.evaluate(ds);
    EXPECT_NEAR(pose.x, ds, kTol);
    EXPECT_NEAR(pose.y, 0.0, kTol);
    EXPECT_NEAR(pose.hdg, 0.0, kTol);
  }
}

TEST(ReferenceLineEvaluator, ParamPoly3ArcLengthMonotonic)
{
  // Nontrivial cubic in both u and v. Re-sampling at a fine ds grid and
  // measuring |Δ(x, y)| should yield the ds step back (modulo integration
  // tolerance): this is both a monotonicity check and a consistency check
  // between the arc-length table and the position evaluator.
  Geometry g;
  g.s = 0.0;
  g.x = 0.0;
  g.y = 0.0;
  g.hdg = 0.0;
  g.length = 40.0;
  GeomParamPoly3 p;
  // Nonlinear u(p), curved v(p), still arc-length-reasonable.
  p.a_u = 0.0;
  p.b_u = 50.0;
  p.c_u = -10.0;
  p.d_u = 0.0;
  p.a_v = 0.0;
  p.b_v = 0.0;
  p.c_v = 5.0;
  p.d_v = 0.0;
  p.p_range_normalized = true;
  g.primitive = p;

  ReferenceLineEvaluator eval{singleGeometry(g)};

  // Monotonicity: cumulative arc length along the sampled points should stay
  // strictly increasing and approach the requested ds within table tolerance.
  constexpr double step = 0.1;
  auto prev = eval.evaluate(0.0);
  double cum = 0.0;
  for (double s = step; s <= eval.length() + 1e-9; s += step) {
    const auto cur = eval.evaluate(s);
    const double d = std::hypot(cur.x - prev.x, cur.y - prev.y);
    EXPECT_GT(d, 0.0);
    cum += d;
    prev = cur;
  }
  // With trapezoidal tabulation at 2000 intervals over p ∈ [0, 1], the
  // cumulative error should be well under 1 mm on this ~arc-length-40-m curve.
  EXPECT_NEAR(cum, eval.length(), 1e-3);
}

TEST(ReferenceLineEvaluator, MultiRecordBreakpoints)
{
  // Two records end-to-end. Breakpoints should expose both the join and the
  // total length; evaluating around the join should be continuous.
  PlanView pv;
  Geometry g1;
  g1.s = 0.0;
  g1.x = 0.0;
  g1.y = 0.0;
  g1.hdg = 0.0;
  g1.length = 10.0;
  g1.primitive = GeomLine{};
  pv.geometries.push_back(g1);

  Geometry g2;
  g2.s = 10.0;
  g2.x = 10.0;
  g2.y = 0.0;
  g2.hdg = M_PI / 2.0;  // turn left
  g2.length = 5.0;
  g2.primitive = GeomLine{};
  pv.geometries.push_back(g2);

  ReferenceLineEvaluator eval{pv};
  EXPECT_NEAR(eval.length(), 15.0, 1e-12);
  ASSERT_EQ(eval.breakpoints().size(), 2u);
  EXPECT_NEAR(eval.breakpoints()[0], 10.0, 1e-12);
  EXPECT_NEAR(eval.breakpoints()[1], 15.0, 1e-12);

  const auto before = eval.evaluate(9.999);
  const auto after = eval.evaluate(10.001);
  EXPECT_NEAR(before.x, 9.999, 1e-6);
  EXPECT_NEAR(after.x, 10.0, 1e-6);
  EXPECT_NEAR(after.y, 0.001, 1e-6);
}

TEST(ElevationEvaluator, EmptyProfileIsZero)
{
  ElevationProfile profile;
  ElevationEvaluator eval{profile};
  EXPECT_NEAR(eval.evaluate(0.0), 0.0, 1e-12);
  EXPECT_NEAR(eval.evaluate(100.0), 0.0, 1e-12);
}

TEST(ElevationEvaluator, PiecewiseCubicContinuity)
{
  // Two records chosen so z matches at the join — typical of a well-authored
  // elevation profile. Evaluator must be continuous at s = 10.
  ElevationProfile profile;
  Poly3Record r0;
  r0.s = 0.0;
  r0.a = 1.0;
  r0.b = 0.5;
  r0.c = 0.01;
  r0.d = 0.0;
  profile.records.push_back(r0);

  Poly3Record r1;
  r1.s = 10.0;
  // z at s = 10 under record 0: a + b·10 + c·100 = 1 + 5 + 1 = 7.
  r1.a = 7.0;
  r1.b = 0.7;  // matches dz/ds at s = 10 under r0: 0.5 + 2·0.01·10 = 0.7.
  r1.c = -0.02;
  r1.d = 0.001;
  profile.records.push_back(r1);

  ElevationEvaluator eval{profile};
  const double before = eval.evaluate(9.9999);
  const double after = eval.evaluate(10.0001);
  EXPECT_NEAR(before, after, 1e-3);
  EXPECT_NEAR(eval.evaluate(10.0), 7.0, 1e-6);
}

TEST(ElevationEvaluator, BeforeFirstRecordIsZero)
{
  ElevationProfile profile;
  Poly3Record r0;
  r0.s = 5.0;
  r0.a = 2.0;
  profile.records.push_back(r0);

  ElevationEvaluator eval{profile};
  EXPECT_NEAR(eval.evaluate(0.0), 0.0, 1e-12);
  EXPECT_NEAR(eval.evaluate(4.999), 0.0, 1e-12);
  EXPECT_NEAR(eval.evaluate(5.0), 2.0, 1e-12);
}

TEST(ElevationEvaluator, ExtrapolationPastLastRecord)
{
  // Beyond the last record, the last polynomial is extrapolated (§5.4).
  ElevationProfile profile;
  Poly3Record r;
  r.s = 0.0;
  r.a = 0.0;
  r.b = 1.0;
  r.c = 0.0;
  r.d = 0.0;
  profile.records.push_back(r);

  ElevationEvaluator eval{profile};
  // z(s) = s for all s ≥ 0 under this record, including past any "end".
  EXPECT_NEAR(eval.evaluate(100.0), 100.0, 1e-9);
  EXPECT_NEAR(eval.evaluate(1000.0), 1000.0, 1e-9);
}

// NOLINTEND(readability-identifier-naming)
