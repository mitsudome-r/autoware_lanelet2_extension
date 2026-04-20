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

#ifndef OPENDRIVE__JUNCTION_LINKER_HPP_
#define OPENDRIVE__JUNCTION_LINKER_HPP_

#include "xodr_types.hpp"

#include <lanelet2_io/io_handlers/IoHandler.h>

namespace lanelet::io_handlers::opendrive
{
// §5.10 / §7 metadata validation for `<junction>/<connection>` records.
// Lane-level endpoint sharing (§5.11) is performed implicitly by the shared
// `Point3dDeduper` used during lane building — this function is a pure
// consistency check over junction metadata.
//
// Emits a non-fatal warning when a `<connection>` references an
// `@incomingRoad` or `@connectingRoad` that does not exist in `doc.roads`
// (§7: "Junction `<connection>` referencing a non-existent road").
void validateJunctions(const XodrDocument & doc, lanelet::ErrorMessages & errors);

}  // namespace lanelet::io_handlers::opendrive

#endif  // OPENDRIVE__JUNCTION_LINKER_HPP_
