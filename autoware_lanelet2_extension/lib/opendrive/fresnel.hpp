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

#ifndef OPENDRIVE__FRESNEL_HPP_
#define OPENDRIVE__FRESNEL_HPP_

namespace lanelet::io_handlers::opendrive
{
// Fresnel integrals, normalized with π/2 (the convention used in the clothoid
// literature and required for §5.3 spiral evaluation):
//
//   C(x) = ∫₀ˣ cos(π t² / 2) dt
//   S(x) = ∫₀ˣ sin(π t² / 2) dt
//
// Implementation follows Numerical Recipes 3e §6.8.1: power series for
// |x| ≤ 1.5, complex continued fraction for |x| > 1.5. Accurate to ~1e-7
// over the full real line.
void fresnel(double x, double & s, double & c);

}  // namespace lanelet::io_handlers::opendrive

#endif  // OPENDRIVE__FRESNEL_HPP_
