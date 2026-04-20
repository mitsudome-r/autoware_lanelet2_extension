# OpenDRIVE Parser — Implementation Phases

Status: **Plan, not yet implemented**
Companion document to [`opendrive_parser.md`](opendrive_parser.md) (the design spec).
Scope: breaks the v1 parser into six self-contained phases. Each phase is a reviewable commit; each leaves the tree buildable and the test suite green.

---

## Phase 1 — Scaffolding & factory registration

**Files created:**

- `include/autoware_lanelet2_extension/io/autoware_opendrive_parser.hpp` — class declaration, `extension()` / `name()`
- `lib/autoware_opendrive_parser.cpp` — factory registration, `parse()` entry point (delegates to later phases)
- `lib/opendrive/xodr_types.hpp` — POD types: `Header`, `Road`, `PlanView`, `Geometry` (variant over `line` / `arc` / `spiral` / `poly3` / `paramPoly3`), `Elevation`, `LaneOffset`, `LaneSection`, `Lane`, `Width`, `Link`, `Signal`, `SignalReference`, `Junction`, `Connection`, `LaneLink`, `XodrDocument`
- `lib/opendrive/xodr_reader.{hpp,cpp}` — pugixml → `XodrDocument`, no interpretation
- `CMakeLists.txt` — add new sources

**`parse()` returns an empty `LaneletMap` at this stage.**

**Verifies:** `lanelet::load("Town10HD.xodr")` dispatches to us without errors.

---

## Phase 2 — Geometry evaluators (§5.3, §5.4)

**Files created:**

- `lib/opendrive/fresnel.{hpp,cpp}` — Fresnel `C(t)`, `S(t)` via Numerical Recipes rational approximation
- `lib/opendrive/geometry.{hpp,cpp}`:
  - `ReferenceLineEvaluator` — walks `<geometry>` records, dispatches per primitive, returns `(x, y, hdg)` at arc length `s`
  - Per-primitive evaluators: line, arc, spiral (Fresnel), poly3, paramPoly3 (with Simpson-tabulated arc-length map)
  - `ElevationEvaluator` — piecewise cubic, extrapolation past last record
- `test/src/test_opendrive_geometry.cpp` — per §9.1: closed-form checks per primitive (tol `1e-4 m`), elevation continuity, spiral-as-degenerate-arc, paramPoly3 arc-length monotonicity

**`parse()` now samples reference lines but produces no lanelets yet.** Outputs can be eyeballed by sampling a road from `Town10HD.xodr` and logging.

---

## Phase 3 — Lane building (§5.5–§5.8, §5.11)

**Files created:**

- `lib/opendrive/lane_builder.{hpp,cpp}`:
  - `LaneOffsetEvaluator` (piecewise cubic `t_center(s)`)
  - `WidthEvaluator` (piecewise cubic per `<width>` record, with `sOffset`)
  - `buildLaneSection(...)` — produces **shared** `LineString3d`s (inner bound of lane _n_ = outer bound of lane _n−1_ via `Id` equality per §5.11)
  - `Point3d` spatial-hash deduper keyed on `round(coord/ε)` (`ε = merge_tol_m`)
  - `(s, t) → world` per §5.6
  - Sample-step strategy per §5.7 (break points at section / width / offset / elevation boundaries, default 0.5 m)
  - Lane-type → subtype mapping per §5.8
  - Emits `opendrive:road_id`, `opendrive:lane_section`, `opendrive:lane_id`, `opendrive:junction_id`, `type`, `subtype`, `location`, `one_way`

**`parse()` now produces a `LaneletMap` with lanelets and shared boundaries for a single road in isolation** (no cross-road linking yet — each road is a disconnected island).

---

## Phase 4 — Linking (§5.9, §5.10)

**Files created:**

- `lib/opendrive/road_linker.{hpp,cpp}` — resolves `<road>/<link>/<predecessor>` and `<successor>` with `contactPoint`, wires lane-level predecessor / successor via shared endpoint `Point3d` unification (NOT shared `LineString3d` — see §5.11)
- `lib/opendrive/junction_linker.{hpp,cpp}` — resolves `<connection>/<laneLink>`, skips `road.link` for internal roads per §5.10
- `Point3d` deduper is reused from phase 3 with `merge_tol_m` tolerance across road endpoints

**`parse()` now produces a topologically connected map.** Spot checks: a road pair with known `<link>` has endpoints unified; a junction's connecting roads resolve lane links correctly.

---

## Phase 5 — Signals → regulatory elements (§5.12)

**Files created:**

- `lib/opendrive/signal_builder.{hpp,cpp}`:
  - `<signal>` classification (dynamic + allowlist)
  - Physical-light `LineString3d` construction (bottom edge of housing, width / height attributes)
  - Stop-line synthesis (spans applicable lanes per `<validity>`)
  - `AutowareTrafficLight` regulatory element emission, attached to applicable lanelets
  - `<signalReference>` resolution: look up by signal id, share reg element across roads, narrow lanelets by reference's `<validity>`
  - Attribute emission per §5.12 table (`opendrive:signal_id`, `signal_type`, `signal_subtype`, `signal_country`, `road_id`)

**`parse()` now produces a full map with traffic-light regulatory elements.**

---

## Phase 6 — Config, full test suite, polish (§6.1, §9.2, §9.3, §7)

**Files created / modified:**

- Config plumbing: `autoware_opendrive/sample_step_m`, `merge_tol_m`, `skip_sidewalks`, `traffic_light_types` via `lanelet::io::Configuration`
- Full error-message formatting per §7 (`[autoware_opendrive_handler]` prefix, road / junction id, pugixml `offset_debug` source line)
- `test/src/test_opendrive_parser.cpp` — §9.2 `Town10HD` integration tests:
  1. Loads cleanly + warning-count constant
  2. Road / junction counts
  3. Traffic-light classification (signal 944, 943)
  4. Non-light signals skipped (946, 947, 948, 963)
  5. `orientation="none"` warning (955, 956)
  6. Stop-line geometry spot-check
  7. `<signalReference>` dedup count
  8. Controllers ignored
  9. Factory registration via extension
  10. Traceability attributes
- `test/fixtures/opendrive/` — hand-written tiny fixtures for §9.3:
  - malformed XML
  - `<lateralProfile>` non-fatal
  - dangling `<signalReference>`
  - allowlist override
  - zero-width signal
  - each geometry primitive in isolation (phase-2 tests also use these)

**End state:** full v1 parser per the spec, ready to merge.

---

## Deviations from the spec expected during implementation

- **Town10HD only uses `line` and `arc`** — spiral / poly3 / paramPoly3 are still implemented per spec, but their real-world exercise is limited to the phase-2 unit fixtures.
- **No `<border>` in Town10HD** — the warning path is exercised only by a unit fixture.
- **Real `<lateralProfile>` blocks in Town10HD** — the "non-fatal warning when non-empty" path fires on the canonical fixture. `expected_warning_count` in §9.2 must account for this.
