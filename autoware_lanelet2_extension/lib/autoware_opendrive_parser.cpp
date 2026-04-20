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
#include <lanelet2_io/Configuration.h>
#include <lanelet2_io/io_handlers/Factory.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace lanelet::io_handlers
{
namespace
{
constexpr const char * kErrPrefix = "[autoware_opendrive_handler]";
constexpr const char * kNamespace = "autoware_opendrive/";

std::vector<std::string> splitCsv(const std::string & s)
{
  std::vector<std::string> out;
  std::string cur;
  for (char c : s) {
    if (c == ',') {
      if (!cur.empty()) {
        out.push_back(cur);
      }
      cur.clear();
      continue;
    }
    if (c == ' ' || c == '\t') {
      continue;  // tolerate surrounding whitespace
    }
    cur.push_back(c);
  }
  if (!cur.empty()) {
    out.push_back(cur);
  }
  return out;
}

// §6.1 config-knob dispatch. Ignores keys outside the `autoware_opendrive/`
// namespace so other parsers' knobs pass through untouched; warns on unknown
// keys inside our namespace per the §6.1 "unknown keys emit a warning" rule.
void applyConfig(
  const lanelet::io::Configuration & cfg, opendrive::LaneBuilderOptions & lane_opts,
  opendrive::SignalBuilderOptions & sig_opts, ErrorMessages & errors)
{
  for (const auto & kv : cfg) {
    const auto & key = kv.first;
    const auto & attr = kv.second;
    if (key == "autoware_opendrive/sample_step_m") {
      if (const auto v = attr.asDouble()) {
        lane_opts.sample_step_m = *v;
      }
      continue;
    }
    if (key == "autoware_opendrive/merge_tol_m") {
      if (const auto v = attr.asDouble()) {
        lane_opts.merge_tol_m = *v;
      }
      continue;
    }
    if (key == "autoware_opendrive/skip_sidewalks") {
      if (const auto v = attr.asBool()) {
        lane_opts.skip_sidewalks = *v;
      }
      continue;
    }
    if (key == "autoware_opendrive/traffic_light_types") {
      auto list = splitCsv(attr.value());
      if (!list.empty()) {
        sig_opts.traffic_light_types = std::move(list);
      }
      continue;
    }
    const std::string ns{kNamespace};
    if (key.compare(0, ns.size(), ns) == 0) {
      errors.emplace_back(
        std::string{kErrPrefix} + " unknown config key '" + key + "'; ignored");
    }
  }
}
}  // namespace

std::unique_ptr<LaneletMap> AutowareOpenDriveParser::parse(
  const std::string & filename, ErrorMessages & errors) const
{
  const auto doc = opendrive::readXodrFile(filename, errors);

  opendrive::LaneBuilderOptions lane_opts;
  opendrive::SignalBuilderOptions sig_opts;
  // IOHandler::config() is declared non-const upstream even though it only
  // copies out; a local const_cast keeps the override signature intact.
  applyConfig(
    const_cast<AutowareOpenDriveParser *>(this)->config(), lane_opts, sig_opts, errors);

  auto map = std::make_unique<LaneletMap>();

  // Phase 4: one deduper across every road in the file. Per §5.11, endpoint
  // Point3d's are unified at road–road `<link>` contact points and junction
  // entries/exits via the same spatial hash that handles within-road
  // section boundaries — that unification happens implicitly as soon as
  // the deduper is shared, because both sides of a link sample the same
  // world coordinates (within merge_tol_m) at the contact point.
  opendrive::Point3dDeduper deduper{lane_opts.merge_tol_m};
  for (const auto & road : doc.roads) {
    opendrive::buildRoadLanelets(road, lane_opts, deduper, *map, errors);
  }

  // Metadata-level validation per §5.9 / §5.10 / §7. The deduper already did
  // the geometric work; these passes flag dangling ids and internal roads
  // that mistakenly carry <road>/<link> (a v1 non-fatal warning).
  opendrive::validateRoadLinks(doc, errors);
  opendrive::validateJunctions(doc, errors);

  // Phase 5: `<signal>` / `<signalReference>` → AutowareTrafficLight. Must
  // run after the lane builder because reg-element attachment queries the
  // lanelet index emitted in Phase 3.
  opendrive::buildSignals(doc, sig_opts, deduper, *map, errors);

  return map;
}

namespace
{
RegisterParser<AutowareOpenDriveParser> regParser;  // NOLINT(cert-err58-cpp)
}  // namespace

}  // namespace lanelet::io_handlers

// NOLINTEND(readability-identifier-naming)
