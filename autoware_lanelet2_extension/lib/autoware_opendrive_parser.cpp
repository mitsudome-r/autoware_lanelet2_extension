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

#include "opendrive/junction_linker.hpp"
#include "opendrive/lane_builder.hpp"
#include "opendrive/road_linker.hpp"
#include "opendrive/signal_builder.hpp"
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
  opendrive::LaneBuilderOptions opts;

  // Phase 4: one deduper across every road in the file. Per §5.11, endpoint
  // Point3d's are unified at road–road `<link>` contact points and junction
  // entries/exits via the same spatial hash that handles within-road
  // section boundaries — that unification happens implicitly as soon as
  // the deduper is shared, because both sides of a link sample the same
  // world coordinates (within merge_tol_m) at the contact point.
  opendrive::Point3dDeduper deduper{opts.merge_tol_m};
  for (const auto & road : doc.roads) {
    opendrive::buildRoadLanelets(road, opts, deduper, *map, errors);
  }

  // Metadata-level validation per §5.9 / §5.10 / §7. The deduper already did
  // the geometric work; these passes flag dangling ids and internal roads
  // that mistakenly carry <road>/<link> (a v1 non-fatal warning).
  opendrive::validateRoadLinks(doc, errors);
  opendrive::validateJunctions(doc, errors);

  // Phase 5: `<signal>` / `<signalReference>` → AutowareTrafficLight. Must
  // run after the lane builder because reg-element attachment queries the
  // lanelet index emitted in Phase 3.
  opendrive::SignalBuilderOptions sig_opts;
  opendrive::buildSignals(doc, sig_opts, deduper, *map, errors);

  return map;
}

namespace
{
RegisterParser<AutowareOpenDriveParser> regParser;  // NOLINT(cert-err58-cpp)
}  // namespace

}  // namespace lanelet::io_handlers

// NOLINTEND(readability-identifier-naming)
