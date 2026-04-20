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

#include "fresnel.hpp"

#include <cmath>
#include <complex>

namespace lanelet::io_handlers::opendrive
{
namespace
{
constexpr double kPi = 3.141592653589793;
constexpr double kPiBy2 = 1.5707963267948966;
constexpr double kEps = 6.0e-8;
constexpr double kFpMin = 1.0e-30;
constexpr double kBig = 1.0e40;
constexpr double kXMin = 1.5;
constexpr int kMaxIt = 100;
}  // namespace

void fresnel(double x, double & s, double & c)
{
  const double ax = std::fabs(x);

  if (ax < std::sqrt(kFpMin)) {
    // For vanishingly small x, S ≈ 0 and C ≈ x (leading-order Taylor).
    s = 0.0;
    c = ax;
  } else if (ax <= kXMin) {
    // Power-series branch.
    double sum = 0.0;
    double sums = 0.0;
    double sumc = ax;
    double sign = 1.0;
    const double fact = kPiBy2 * ax * ax;
    bool odd = true;
    double term = ax;
    int n = 3;
    double test = 0.0;
    int k = 1;
    for (; k <= kMaxIt; ++k) {
      term *= fact / k;
      sum += sign * term / n;
      test = std::fabs(sum) * kEps;
      if (odd) {
        sign = -sign;
        sums = sum;
        sum = sumc;
      } else {
        sumc = sum;
        sum = sums;
      }
      if (term < test) {
        break;
      }
      odd = !odd;
      n += 2;
    }
    s = sums;
    c = sumc;
  } else {
    // Continued-fraction branch (complex form — NR3 §6.8.1).
    const double pix2 = kPi * ax * ax;
    std::complex<double> b{1.0, -pix2};
    std::complex<double> cc{kBig, 0.0};
    std::complex<double> d = 1.0 / b;
    std::complex<double> h = d;
    int n = -1;
    for (int k = 2; k <= kMaxIt; ++k) {
      n += 2;
      const double a = -static_cast<double>(n) * static_cast<double>(n + 1);
      b += std::complex<double>{4.0, 0.0};
      d = 1.0 / (a * d + b);
      cc = b + a / cc;
      const std::complex<double> del = cc * d;
      h *= del;
      if (std::fabs(del.real() - 1.0) + std::fabs(del.imag()) <= kEps) {
        break;
      }
    }
    h = std::complex<double>{ax, -ax} * h;
    const std::complex<double> cs =
      std::complex<double>{0.5, 0.5} *
      (1.0 - std::complex<double>{std::cos(0.5 * pix2), std::sin(0.5 * pix2)} * h);
    c = cs.real();
    s = cs.imag();
  }

  if (x < 0.0) {
    c = -c;
    s = -s;
  }
}

}  // namespace lanelet::io_handlers::opendrive
