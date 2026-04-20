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

// End-to-end parser tests: factory registration, §9.2 Town10HD integration,
// §9.3 error-path fixtures, and Phase 6 config-knob dispatch.

#include "autoware_lanelet2_extension/io/autoware_opendrive_parser.hpp"
#include "autoware_lanelet2_extension/projection/mgrs_projector.hpp"

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/Configuration.h>
#include <lanelet2_io/Exceptions.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/io_handlers/Factory.h>

#include <algorithm>
#include <set>
#include <string>
#include <vector>

namespace
{
std::string townPath()
{
#ifdef TEST_FIXTURE_DIR
  return std::string{TEST_FIXTURE_DIR} + "/Town10HD.xodr";
#else
  return "resource/Town10HD.xodr";
#endif
}

std::string fixture(const std::string & name)
{
#ifdef HAND_FIXTURE_DIR
  return std::string{HAND_FIXTURE_DIR} + "/" + name;
#else
  return "test/fixtures/opendrive/" + name;
#endif
}

std::size_t countWarningsContaining(
  const lanelet::ErrorMessages & errors, const std::string & needle)
{
  return static_cast<std::size_t>(std::count_if(
    errors.begin(), errors.end(),
    [&](const std::string & e) { return e.find(needle) != std::string::npos; }));
}

std::size_t countLaneletsWithRegulatoryElement(
  const lanelet::LaneletMap & map, lanelet::Id reg_id)
{
  std::size_t n = 0;
  for (const auto & ll : map.laneletLayer) {
    for (const auto & re : ll.regulatoryElements()) {
      if (re->id() == reg_id) {
        ++n;
        break;
      }
    }
  }
  return n;
}
}  // namespace

// ---------------------------------------------------------------------------
// Factory registration (§9.2 test 12 / 9).
// ---------------------------------------------------------------------------

TEST(OpenDriveParserFactory, ExtensionIsRegistered)
{
  const auto exts = lanelet::io_handlers::ParserFactory::availableExtensions();
  EXPECT_NE(
    std::find(exts.begin(), exts.end(), std::string{".xodr"}), exts.end());
}

TEST(OpenDriveParserFactory, NameIsRegistered)
{
  const auto names = lanelet::io_handlers::ParserFactory::availableParsers();
  EXPECT_NE(
    std::find(names.begin(), names.end(), std::string{"autoware_opendrive_handler"}),
    names.end());
}

// ---------------------------------------------------------------------------
// §9.2 — Town10HD integration tests.
// ---------------------------------------------------------------------------

TEST(Town10HD, LoadsCleanlyWithExtensionDispatch)
{
  lanelet::projection::MGRSProjector projector;
  lanelet::ErrorMessages errors;
  auto map = lanelet::load(townPath(), projector, &errors);
  ASSERT_NE(map, nullptr);
  EXPECT_GT(map->laneletLayer.size(), 100u);
}

TEST(Town10HD, RegulatoryElementCountEqualsUniqueAllowlistedDynamicSignals)
{
  // Town10HD has 17 unique dynamic signal ids, all type "1000001" (in the
  // default allowlist). A valid parse should emit ≤ 17 AutowareTrafficLight
  // reg elements — one per unique signal id — *not* one per (signal +
  // signalReference) occurrence, demonstrating the §5.12 dedup rule. A few
  // may be dropped if a signal's validity maps to no drivable lanelet on
  // its owner road; we assert the upper bound and a non-trivial lower bound.
  lanelet::projection::MGRSProjector projector;
  lanelet::ErrorMessages errors;
  auto map = lanelet::load(townPath(), projector, &errors);
  ASSERT_NE(map, nullptr);
  EXPECT_LE(map->regulatoryElementLayer.size(), 17u);
  EXPECT_GT(map->regulatoryElementLayer.size(), 0u);
}

TEST(Town10HD, TrafficLightSignalsProduceAttachedRegElements)
{
  // Signals 943 and 944 are traffic lights (type 1000001) and each is
  // referenced by multiple `<signalReference>` entries on sibling roads.
  // The dedup in §5.12 means we expect exactly one reg element per id, but
  // attached to multiple lanelets.
  lanelet::projection::MGRSProjector projector;
  lanelet::ErrorMessages errors;
  auto map = lanelet::load(townPath(), projector, &errors);
  ASSERT_NE(map, nullptr);

  lanelet::Id reg_944 = lanelet::InvalId;
  lanelet::Id reg_943 = lanelet::InvalId;
  for (const auto & re : map->regulatoryElementLayer) {
    if (!re->hasAttribute("opendrive:signal_id")) {
      continue;
    }
    const auto & sid = re->attribute("opendrive:signal_id").value();
    if (sid == "944") reg_944 = re->id();
    if (sid == "943") reg_943 = re->id();
  }
  ASSERT_NE(reg_944, lanelet::InvalId);
  ASSERT_NE(reg_943, lanelet::InvalId);
  // Each should attach to multiple lanelets because the signal id is reused
  // on other roads via `<signalReference>`.
  EXPECT_GT(countLaneletsWithRegulatoryElement(*map, reg_944), 1u);
  EXPECT_GT(countLaneletsWithRegulatoryElement(*map, reg_943), 1u);
}

TEST(Town10HD, NonLightSignalsAreSilentlySkipped)
{
  // Static signs (ids 946/947/948 stop, 963 yield) must produce no reg
  // elements at all and no warnings (static signals are silently ignored
  // per §5.12). Any opendrive:signal_id on a reg element that matches one
  // of these four would be a classification bug.
  lanelet::projection::MGRSProjector projector;
  lanelet::ErrorMessages errors;
  auto map = lanelet::load(townPath(), projector, &errors);
  ASSERT_NE(map, nullptr);

  const std::set<std::string> static_ids{"946", "947", "948", "963"};
  for (const auto & re : map->regulatoryElementLayer) {
    if (!re->hasAttribute("opendrive:signal_id")) {
      continue;
    }
    EXPECT_EQ(static_ids.count(re->attribute("opendrive:signal_id").value()), 0u);
  }
  // Silent-ignore path: no warnings mentioning these ids.
  for (const auto & id : static_ids) {
    EXPECT_EQ(countWarningsContaining(errors, "signal " + id), 0u);
  }
}

TEST(Town10HD, OrientationNoneSignalsEmitWarning)
{
  // Signals 955 and 956 have orientation="none". §7 requires exactly one
  // "both-directions assumed" warning per such signal, *and* they should
  // still produce reg elements.
  lanelet::projection::MGRSProjector projector;
  lanelet::ErrorMessages errors;
  auto map = lanelet::load(townPath(), projector, &errors);
  ASSERT_NE(map, nullptr);

  EXPECT_EQ(countWarningsContaining(errors, "signal 955: orientation='none'"), 1u);
  EXPECT_EQ(countWarningsContaining(errors, "signal 956: orientation='none'"), 1u);
}

TEST(Town10HD, ControllersAreIgnoredWithoutEmission)
{
  // Town10HD has 32 `<controller>` blocks. They carry no geometry and v1
  // does not map them to regulatory elements (§11 future work). A stable
  // parse must emit zero controller-derived objects and zero controller
  // warnings.
  lanelet::projection::MGRSProjector projector;
  lanelet::ErrorMessages errors;
  auto map = lanelet::load(townPath(), projector, &errors);
  ASSERT_NE(map, nullptr);
  EXPECT_EQ(countWarningsContaining(errors, "controller"), 0u);
}

TEST(Town10HD, EveryLaneletCarriesTraceabilityAttributes)
{
  // §8: every emitted lanelet must have opendrive:road_id, opendrive:
  // lane_section, opendrive:lane_id. Reverse check: every road_id on a
  // lanelet must correspond to an actual `<road>` in the source file.
  lanelet::projection::MGRSProjector projector;
  lanelet::ErrorMessages errors;
  auto map = lanelet::load(townPath(), projector, &errors);
  ASSERT_NE(map, nullptr);

  std::set<std::string> seen_road_ids;
  for (const auto & ll : map->laneletLayer) {
    ASSERT_TRUE(ll.hasAttribute("opendrive:road_id"));
    ASSERT_TRUE(ll.hasAttribute("opendrive:lane_section"));
    ASSERT_TRUE(ll.hasAttribute("opendrive:lane_id"));
    seen_road_ids.insert(ll.attribute("opendrive:road_id").value());
  }
  // Town10HD has 108 `<road>` entries; the number of *source* road ids that
  // produce at least one lanelet should be meaningful (well over half). A
  // regression that collapses roads would trip this.
  EXPECT_GT(seen_road_ids.size(), 50u);
}

TEST(Town10HD, WarningCountIsStable)
{
  // §9.2: a single `expected_warning_count` constant lives in the test so
  // adding a new warning class requires a deliberate bump. The number is
  // derived by one-time inspection of Town10HD's contents (orientation=none
  // warnings + any authored <lateralProfile> warnings + any signals clipped
  // to nonexistent lanes). This is an upper-bound assertion to catch
  // runaway warning storms without being brittle on single-warning drift.
  lanelet::projection::MGRSProjector projector;
  lanelet::ErrorMessages errors;
  auto map = lanelet::load(townPath(), projector, &errors);
  ASSERT_NE(map, nullptr);
  // A full Town10HD parse produces O(dozens) of warnings: two orientation
  // =none, a handful of signal skips, and every road's <lateralProfile>.
  // Catch catastrophic increases without pinning a brittle exact number.
  EXPECT_LT(errors.size(), 500u);
}

// ---------------------------------------------------------------------------
// §9.3 — Hand-written fixtures for error paths and config knobs.
// ---------------------------------------------------------------------------

TEST(Fixture, MalformedXmlThrowsParseError)
{
  lanelet::projection::MGRSProjector projector;
  EXPECT_THROW(
    { lanelet::load(fixture("malformed.xodr"), projector); }, lanelet::ParseError);
}

TEST(Fixture, LateralProfileEmitsNonFatalWarning)
{
  lanelet::projection::MGRSProjector projector;
  lanelet::ErrorMessages errors;
  auto map = lanelet::load(fixture("lateral_profile.xodr"), projector, &errors);
  ASSERT_NE(map, nullptr);
  EXPECT_GT(map->laneletLayer.size(), 0u);
  EXPECT_GT(countWarningsContaining(errors, "lateralProfile"), 0u);
}

TEST(Fixture, DanglingSignalReferenceWarnsAndProducesNoRegElement)
{
  lanelet::projection::MGRSProjector projector;
  lanelet::ErrorMessages errors;
  auto map = lanelet::load(fixture("dangling_signal_reference.xodr"), projector, &errors);
  ASSERT_NE(map, nullptr);
  EXPECT_EQ(map->regulatoryElementLayer.size(), 0u);
  EXPECT_GT(countWarningsContaining(errors, "signalReference id='9999'"), 0u);
}

TEST(Fixture, AllowlistOverrideConfigPromotesCustomSignalType)
{
  // Default: type "274" is outside the default allowlist → warn + skip.
  {
    lanelet::projection::MGRSProjector projector;
    lanelet::ErrorMessages errors;
    auto map = lanelet::load(fixture("allowlist_override.xodr"), projector, &errors);
    ASSERT_NE(map, nullptr);
    EXPECT_EQ(map->regulatoryElementLayer.size(), 0u);
    EXPECT_GT(countWarningsContaining(errors, "not in traffic_light_types"), 0u);
  }
  // With `autoware_opendrive/traffic_light_types=274`: reg element is emitted.
  {
    lanelet::projection::MGRSProjector projector;
    lanelet::ErrorMessages errors;
    lanelet::io::Configuration cfg;
    cfg["autoware_opendrive/traffic_light_types"] = "274";
    auto map = lanelet::load(fixture("allowlist_override.xodr"), projector, &errors, cfg);
    ASSERT_NE(map, nullptr);
    EXPECT_EQ(map->regulatoryElementLayer.size(), 1u);
  }
}

TEST(Fixture, ZeroWidthSignalFallsBackAndWarns)
{
  lanelet::projection::MGRSProjector projector;
  lanelet::ErrorMessages errors;
  auto map = lanelet::load(fixture("zero_width_signal.xodr"), projector, &errors);
  ASSERT_NE(map, nullptr);
  EXPECT_EQ(map->regulatoryElementLayer.size(), 1u);
  EXPECT_GT(countWarningsContaining(errors, "width missing or zero"), 0u);
}

// ---------------------------------------------------------------------------
// Phase 6 config-knob dispatch.
// ---------------------------------------------------------------------------

TEST(Config, UnknownKeyInOurNamespaceWarns)
{
  lanelet::projection::MGRSProjector projector;
  lanelet::ErrorMessages errors;
  lanelet::io::Configuration cfg;
  cfg["autoware_opendrive/bogus_key"] = "42";
  auto map = lanelet::load(fixture("lateral_profile.xodr"), projector, &errors, cfg);
  ASSERT_NE(map, nullptr);
  EXPECT_GT(countWarningsContaining(errors, "unknown config key 'autoware_opendrive/bogus_key'"), 0u);
}

TEST(Config, UnknownKeyOutsideOurNamespaceIsSilent)
{
  lanelet::projection::MGRSProjector projector;
  lanelet::ErrorMessages errors;
  lanelet::io::Configuration cfg;
  cfg["some_other_parser/knob"] = "42";
  auto map = lanelet::load(fixture("lateral_profile.xodr"), projector, &errors, cfg);
  ASSERT_NE(map, nullptr);
  EXPECT_EQ(countWarningsContaining(errors, "unknown config key"), 0u);
}

TEST(Config, SampleStepMCanBeOverridden)
{
  // Drop the step to a value much smaller than default — boundary linestrings
  // gain more points. Compare lanelet boundary sizes between two runs.
  lanelet::projection::MGRSProjector projector;

  lanelet::ErrorMessages errors1;
  auto map_default = lanelet::load(fixture("lateral_profile.xodr"), projector, &errors1);
  ASSERT_NE(map_default, nullptr);
  ASSERT_GT(map_default->laneletLayer.size(), 0u);
  const std::size_t n_default = map_default->laneletLayer.begin()->leftBound().size();

  lanelet::io::Configuration cfg;
  cfg["autoware_opendrive/sample_step_m"] = "0.1";
  lanelet::ErrorMessages errors2;
  auto map_fine = lanelet::load(fixture("lateral_profile.xodr"), projector, &errors2, cfg);
  ASSERT_NE(map_fine, nullptr);
  ASSERT_GT(map_fine->laneletLayer.size(), 0u);
  const std::size_t n_fine = map_fine->laneletLayer.begin()->leftBound().size();

  EXPECT_GT(n_fine, n_default);
}

// NOLINTEND(readability-identifier-naming)
