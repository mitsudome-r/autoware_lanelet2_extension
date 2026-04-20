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

#ifndef AUTOWARE_LANELET2_EXTENSION__IO__AUTOWARE_OPENDRIVE_PARSER_HPP_
#define AUTOWARE_LANELET2_EXTENSION__IO__AUTOWARE_OPENDRIVE_PARSER_HPP_

// NOLINTBEGIN(readability-identifier-naming)

#include <lanelet2_io/io_handlers/Parser.h>

#include <memory>
#include <string>

namespace lanelet::io_handlers
{
class AutowareOpenDriveParser : public Parser
{
public:
  using Parser::Parser;

  std::unique_ptr<LaneletMap> parse(
    const std::string & filename, ErrorMessages & errors) const override;

  static constexpr const char * extension() { return ".xodr"; }

  static constexpr const char * name() { return "autoware_opendrive_handler"; }
};
}  // namespace lanelet::io_handlers

// NOLINTEND(readability-identifier-naming)

#endif  // AUTOWARE_LANELET2_EXTENSION__IO__AUTOWARE_OPENDRIVE_PARSER_HPP_
