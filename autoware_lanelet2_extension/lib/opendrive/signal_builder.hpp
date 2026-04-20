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

#ifndef OPENDRIVE__SIGNAL_BUILDER_HPP_
#define OPENDRIVE__SIGNAL_BUILDER_HPP_

#include "lane_builder.hpp"
#include "xodr_types.hpp"

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/io_handlers/IoHandler.h>

#include <string>
#include <vector>

namespace lanelet::io_handlers::opendrive
{
// Configuration for signal → regulatory-element emission (§5.12, §6.1). The
// allowlist is compared against the OpenDRIVE `<signal @type>` attribute as a
// string; defaults cover the standard OpenDRIVE traffic-light vocabulary.
struct SignalBuilderOptions
{
  std::vector<std::string> traffic_light_types{"1000001", "1000002", "1000013"};

  // If a traffic-light signal has no authored `width`, synthesize a linestring
  // this long centered on the signal position and emit a non-fatal warning
  // (§5.12). 0.3 m is a reasonable default — ~the footprint of a single bulb
  // housing — and is the value the spec pins.
  double fallback_light_width_m{0.3};
};

// Walk `<signals>/<signal>` and `<signals>/<signalReference>` across all
// roads and emit `AutowareTrafficLight` regulatory elements for entries that
// pass the §5.12 classification. Must be called *after* the lane builder has
// populated `map` with lanelets — signal attachment queries the
// `opendrive:road_id` / `opendrive:lane_section` / `opendrive:lane_id`
// tuple emitted by Phase 3.
//
// The deduper is threaded through so stop-line endpoints co-locate with
// existing lane-boundary `Point3d`s at the signal's s, keeping the map
// topologically clean.
void buildSignals(
  const XodrDocument & doc, const SignalBuilderOptions & opts, Point3dDeduper & deduper,
  lanelet::LaneletMap & map, lanelet::ErrorMessages & errors);

}  // namespace lanelet::io_handlers::opendrive

#endif  // OPENDRIVE__SIGNAL_BUILDER_HPP_
