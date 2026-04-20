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

#include "junction_linker.hpp"

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
}  // namespace

void validateJunctions(const XodrDocument & doc, lanelet::ErrorMessages & errors)
{
  std::unordered_set<std::string> road_ids;
  road_ids.reserve(doc.roads.size());
  for (const auto & r : doc.roads) {
    road_ids.insert(r.id);
  }

  for (const auto & j : doc.junctions) {
    for (const auto & c : j.connections) {
      auto check = [&](const char * which, const std::string & id) {
        if (!id.empty() && road_ids.find(id) == road_ids.end()) {
          std::ostringstream oss;
          oss << "junction " << j.id << " connection " << c.id << " references unknown "
              << which << " '" << id << "'; skipping this connection";
          warn(errors, oss.str());
        }
      };
      check("incomingRoad", c.incoming_road);
      check("connectingRoad", c.connecting_road);
    }
  }
}

}  // namespace lanelet::io_handlers::opendrive
