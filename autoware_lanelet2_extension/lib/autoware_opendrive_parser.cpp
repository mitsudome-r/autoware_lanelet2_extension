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
  // Phase 1 scaffolding: read the .xodr into a POD tree and return an empty
  // map. Later phases (geometry, lane building, linking, signals) will consume
  // `doc` to populate the LaneletMap.
  const auto doc = opendrive::readXodrFile(filename, errors);
  (void)doc;

  return std::make_unique<LaneletMap>();
}

namespace
{
RegisterParser<AutowareOpenDriveParser> regParser;  // NOLINT(cert-err58-cpp)
}  // namespace

}  // namespace lanelet::io_handlers

// NOLINTEND(readability-identifier-naming)
