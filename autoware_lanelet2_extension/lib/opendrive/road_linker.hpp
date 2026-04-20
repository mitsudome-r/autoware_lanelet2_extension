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

#ifndef OPENDRIVE__ROAD_LINKER_HPP_
#define OPENDRIVE__ROAD_LINKER_HPP_

#include "xodr_types.hpp"

#include <lanelet2_io/io_handlers/IoHandler.h>

namespace lanelet::io_handlers::opendrive
{
// §5.9 / §5.10 / §7 metadata validation for `<road>/<link>`.
//
// The actual geometric wiring (shared endpoint `Point3d` Ids across
// neighbouring roads) is performed implicitly by the shared
// `Point3dDeduper` that the lane builder uses, so this function has no
// geometric work of its own. It is strictly a consistency check over the
// authored `<link>` metadata, per §5.11 which treats cross-road unification
// as a coordinate-matching concern rather than a metadata-driven one.
//
// Emits a non-fatal warning (per §7) when:
//   - `<road>/<link>/<predecessor|successor>` with `elementType="road"`
//     points at a road id that does not exist in `doc.roads`;
//   - `<road>/<link>/<predecessor|successor>` with `elementType="junction"`
//     points at a junction id that does not exist in `doc.junctions`;
//   - A road with `@junction != "-1"` (an internal junction road) carries
//     any `<road>/<link>` — §5.10 requires linkage to come entirely from
//     `<connection>` records in that case.
void validateRoadLinks(const XodrDocument & doc, lanelet::ErrorMessages & errors);

}  // namespace lanelet::io_handlers::opendrive

#endif  // OPENDRIVE__ROAD_LINKER_HPP_
