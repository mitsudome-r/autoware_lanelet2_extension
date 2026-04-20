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

#include "autoware_lanelet2_extension/io/autoware_opendrive_parser.hpp"

#include "opendrive/lane_builder.hpp"
#include "opendrive/xodr_reader.hpp"

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/io_handlers/Factory.h>

#include <memory>
#include <string>

namespace lanelet::io_handlers
{
std::unique_ptr<LaneletMap> AutowareOpenDriveParser::parse(
  const std::string & filename, ErrorMessages & errors) const
{
  const auto doc = opendrive::readXodrFile(filename, errors);

  auto map = std::make_unique<LaneletMap>();
  // Phase 3: per-road lanelets with shared boundaries within a lane section
  // and Point3d unification across section boundaries on the same road. Each
  // road is still a disconnected island — cross-road linking arrives in
  // Phase 4. Config knobs are wired in Phase 6; defaults apply for now.
  opendrive::LaneBuilderOptions opts;
  for (const auto & road : doc.roads) {
    // Per-road deduper: within-road section boundaries share Point3ds.
    // Cross-road unification is intentionally left to road_linker.
    opendrive::Point3dDeduper deduper{opts.merge_tol_m};
    opendrive::buildRoadLanelets(road, opts, deduper, *map, errors);
  }

  return map;
}

namespace
{
RegisterParser<AutowareOpenDriveParser> regParser;  // NOLINT(cert-err58-cpp)
}  // namespace

}  // namespace lanelet::io_handlers

// NOLINTEND(readability-identifier-naming)
