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

// Phase 1 scaffolding tests: verify factory registration and that the
// .xodr → POD read step completes without throwing on the canonical fixture.
// Full geometry/lanelet assertions arrive in phases 2-6.

#include "autoware_lanelet2_extension/io/autoware_opendrive_parser.hpp"
#include "autoware_lanelet2_extension/projection/mgrs_projector.hpp"

#include <gtest/gtest.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/io_handlers/Factory.h>

#include <algorithm>
#include <string>
#include <vector>

namespace
{
std::string fixturePath()
{
  // When this test runs via `ros2 test`, it is launched from the build dir;
  // ament_add_ros_isolated_gtest does not set CWD. We rely on a compile-time
  // define passed from CMakeLists so relocating the tree does not break things.
#ifdef TEST_FIXTURE_DIR
  return std::string{TEST_FIXTURE_DIR} + "/Town10HD.xodr";
#else
  return "resource/Town10HD.xodr";
#endif
}
}  // namespace

TEST(OpenDriveParserPhase1, ExtensionIsRegistered)
{
  const auto extensions = lanelet::io_handlers::ParserFactory::availableExtensions();
  EXPECT_NE(
    std::find(extensions.begin(), extensions.end(), std::string{".xodr"}),
    extensions.end())
    << "`.xodr` extension not registered with the parser factory";
}

TEST(OpenDriveParserPhase1, NameIsRegistered)
{
  const auto parsers = lanelet::io_handlers::ParserFactory::availableParsers();
  EXPECT_NE(
    std::find(parsers.begin(), parsers.end(), std::string{"autoware_opendrive_handler"}),
    parsers.end())
    << "`autoware_opendrive_handler` name not registered with the parser factory";
}

TEST(OpenDriveParserPhase1, LoadsTown10HDWithoutFatalError)
{
  lanelet::projection::MGRSProjector projector;
  lanelet::ErrorMessages errors;
  auto map = lanelet::load(fixturePath(), projector, &errors);
  EXPECT_NE(map, nullptr);
  // Phase 3 populates the map with per-road lanelets; Town10HD has hundreds
  // of driving-lane instances. A generous lower bound here catches full
  // regressions without hard-coding an exact count — §9.2 (Phase 6) will.
  EXPECT_GT(map->laneletLayer.size(), 100u);
}

// NOLINTEND(readability-identifier-naming)
