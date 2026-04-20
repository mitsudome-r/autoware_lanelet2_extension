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

#include "road_linker.hpp"

#include <optional>
#include <sstream>
#include <string>
#include <unordered_set>

namespace lanelet::io_handlers::opendrive
{
namespace
{
constexpr const char * kErrPrefix = "[autoware_opendrive_handler]";

void warn(lanelet::ErrorMessages & errors, const std::string & msg)
{
  errors.emplace_back(std::string{kErrPrefix} + " " + msg);
}

bool isInternalRoad(const Road & road)
{
  return !road.junction.empty() && road.junction != "-1";
}

void checkLinkEnd(
  const Road & road, const char * which, const std::optional<RoadLinkEnd> & end,
  const std::unordered_set<std::string> & road_ids,
  const std::unordered_set<std::string> & junction_ids, lanelet::ErrorMessages & errors)
{
  if (!end) {
    return;
  }
  if (end->element_type == "road") {
    if (road_ids.find(end->element_id) == road_ids.end()) {
      std::ostringstream oss;
      oss << "road " << road.id << " <link>/<" << which << "> references unknown road id '"
          << end->element_id << "'; skipping this link";
      warn(errors, oss.str());
    }
  } else if (end->element_type == "junction") {
    if (junction_ids.find(end->element_id) == junction_ids.end()) {
      std::ostringstream oss;
      oss << "road " << road.id << " <link>/<" << which << "> references unknown junction id '"
          << end->element_id << "'; skipping this link";
      warn(errors, oss.str());
    }
  }
  // element_type other than "road"/"junction" is rejected at the XML schema
  // level; we do not emit a second warning here to avoid double-counting.
}
}  // namespace

void validateRoadLinks(const XodrDocument & doc, lanelet::ErrorMessages & errors)
{
  std::unordered_set<std::string> road_ids;
  road_ids.reserve(doc.roads.size());
  for (const auto & r : doc.roads) {
    road_ids.insert(r.id);
  }
  std::unordered_set<std::string> junction_ids;
  junction_ids.reserve(doc.junctions.size());
  for (const auto & j : doc.junctions) {
    junction_ids.insert(j.id);
  }

  for (const auto & road : doc.roads) {
    if (isInternalRoad(road)) {
      // §5.10: internal roads (those inside a junction) must draw their
      // linkage from the parent `<connection>` records, never from
      // `<road>/<link>`. Authored files that violate this are tolerated but
      // warned about.
      if (road.link.predecessor || road.link.successor) {
        std::ostringstream oss;
        oss << "road " << road.id << " (internal to junction " << road.junction
            << ") carries <road>/<link>; ignored per §5.10";
        warn(errors, oss.str());
      }
      continue;
    }
    checkLinkEnd(road, "predecessor", road.link.predecessor, road_ids, junction_ids, errors);
    checkLinkEnd(road, "successor", road.link.successor, road_ids, junction_ids, errors);
  }
}

}  // namespace lanelet::io_handlers::opendrive
