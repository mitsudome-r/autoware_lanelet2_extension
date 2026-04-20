# OpenDRIVE Parser for `autoware_lanelet2_extension`

Status: **DRAFT / spec, not yet implemented**
Target package: `autoware_lanelet2_extension`
Scope: v1 of an `.xodr` → `lanelet2::LaneletMap` parser, registered with `lanelet2_io`.

---

## 1. Overview

This spec defines a parser that reads ASAM OpenDRIVE maps (`.xodr`) and produces an in-memory `lanelet2::LaneletMap` compatible with the rest of `autoware_lanelet2_extension`. The parser plugs into `lanelet2_io::io_handlers::Factory` , so existing callers can do:

```cpp
auto map = lanelet::load("example.xodr", projector);
```

The parser is a standalone implementation (no runtime dependency on external OpenDRIVE tools) and lives entirely inside this package.

---

## 2. Goals / non-goals

### Goals (v1)

- Supports up to OpenDRIVE 1.8 XML.
- Evaluate all five reference-line geometry primitives: `line`, `arc`, `spiral`, `poly3`, `paramPoly3`.
- Handle `<elevationProfile>` (z along the reference line).
- Build lanelets from `<lanes>` with `<laneOffset>`, `<laneSection>`, `<lane>`, `<width>`.
- Link lanelets across roads via `<road>/<link>/<predecessor>` / `<successor>` including `contactPoint`.
- Support junctions as collections of connecting roads, with predecessor/successor wiring at the lane level via `<connection>/<laneLink>`.
- Parse `<signals>/<signal>` entries that describe traffic lights and emit one `AutowareTrafficLight` regulatory element per signal, including a stop line synthesized at the signal's `s` across the lanes listed in `<validity>`.
- Register the parser with `lanelet2_io` so `lanelet::load("*.xodr")` works.
- Accept the same `Projector` argument `lanelet2_io` already uses.

### Non-goals (v1, explicitly deferred)

- `<lateralProfile>` (superelevation / shape) — z tilt ignored, lanes are flat in the t direction.
- `<signals>` beyond traffic lights: speed-limit signs, warning signs, priority signs, stand-alone stop signs, and any other static signage are **ignored** in v1. Traffic lights are supported — see §5.12.
- `<objects>`, `<railroad>`.
- `<roadMark>` (color/type/width carried to lanelet bound tags).
- Right-of-way / priority regulatory elements inside junctions.
- OpenDRIVE 1.4 / 1.5 quirks (best-effort only; no compatibility shims). Note: the canonical integration fixture [`resource/Town10HD.xodr`](../resource/Town10HD.xodr) is OpenDRIVE 1.4, so the parser must accept 1.4 input for the features this spec covers — "best-effort" applies only to 1.4/1.5 elements or attribute semantics that differ from 1.6+.
- `border` lane records (only `width` is honored).
- Lane type `driving` vs `biking` semantic mapping beyond lanelet `subtype` (see §5.8).
- Round-trip writing (this is a parser, not a writer).

A follow-up PR can add signals, road marks, and junction priority as regulatory elements.

---

## 3. Architecture

### 3.1 Class

```cpp
namespace lanelet::io_handlers
{
class AutowareOpenDriveParser : public Parser
{
public:
  using Parser::Parser;

  std::unique_ptr<LaneletMap> parse(
    const std::string & filename, ErrorMessages & errors) const override;

  static constexpr const char * extension() { return ".xodr"; }
  static constexpr const char * name()      { return "autoware_opendrive_handler"; }
};
}  // namespace lanelet::io_handlers
```

Registered with the factory via:

```cpp
namespace { RegisterParser<AutowareOpenDriveParser> regParser; }
```

Unlike `AutowareOsmParser`, this class inherits from `lanelet::io_handlers::Parser` directly — there is no upstream `OpenDriveParser` to delegate to.

### 3.2 File layout

```
autoware_lanelet2_extension/
  include/autoware_lanelet2_extension/io/
    autoware_opendrive_parser.hpp          (public class)
  lib/
    autoware_opendrive_parser.cpp          (entry point, factory registration)
    opendrive/
      xodr_types.hpp                       (POD structs for XML contents)
      xodr_reader.hpp/.cpp                 (pugixml → xodr_types)
      geometry.hpp/.cpp                    (ref-line evaluators)
      fresnel.hpp/.cpp                     (Fresnel integrals for spiral)
      lane_builder.hpp/.cpp                (laneSection → boundary polylines)
      road_linker.hpp/.cpp                 (predecessor/successor stitching)
      junction_linker.hpp/.cpp             (junction connection resolution)
```

Everything under `lib/opendrive/` is an implementation detail of the parser and not installed as a public header.

### 3.3 Data flow

```
xodr XML
  │  (pugixml)
  ▼
XodrDocument (POD tree: Header, Road[], Junction[])
  │  (reference-line evaluator + elevation profile)
  ▼
Per-road sampled centerline  { s, x, y, z, hdg }
  │  (lane offset + lane widths)
  ▼
Per-road per-lane boundary polylines (shared between adjacent lanes)
  │  (LineString3d / Point3d construction, id assignment)
  ▼
Per-road Lanelet3d objects
  │  (predecessor / successor stitching, junction wiring)
  ▼
LaneletMap
```

---

## 4. Supported OpenDRIVE elements

| Element                                      | v1 status                 |
| -------------------------------------------- | ------------------------- |
| `<header>` / `<geoReference>`                | parsed; used for origin   |
| `<road>`                                     | yes                       |
| `<road>/<link>`                              | yes (road ↔ road, road ↔ junction) |
| `<planView>/<geometry>`                      | yes (all 5 types)         |
| `<elevationProfile>/<elevation>`             | yes                       |
| `<lateralProfile>`                           | **ignored**               |
| `<lanes>/<laneOffset>`                       | yes                       |
| `<lanes>/<laneSection>`                      | yes                       |
| `<lane>` (`type`, `level`, `id`)             | `type` → subtype, others noted |
| `<lane>/<link>`                              | yes                       |
| `<lane>/<width>`                             | yes (piecewise cubic)     |
| `<lane>/<border>`                            | **ignored** (treated as zero width) with warning |
| `<lane>/<roadMark>`                          | **ignored** (v1)          |
| `<lane>/<material>` / `<speed>` / `<access>` | **ignored** (v1)          |
| `<signals>/<signal>` (traffic lights)        | yes (see §5.12)           |
| `<signal>/<validity>`                        | yes (selects applicable lanes) |
| `<signalReference>`                          | yes (reuses the same reg element across roads) |
| `<signal>/<dependency>` / `<controller>`     | **ignored** (v1)          |
| `<signals>/<signal>` (non-traffic-light)     | **ignored** (v1) — warning if `dynamic="yes"` |
| `<objects>` / `<railroad>`                   | **ignored** (v1)          |
| `<junction>`                                 | yes (connection + laneLink) |
| `<junction>/<priority>` / `<controller>`     | **ignored** (v1)          |
| `<junctionGroup>`                            | **ignored** (v1)          |
| `<controller>` (root-level)                  | **ignored** (v1)          |

A parser error is emitted (but not fatal) whenever an ignored element is encountered and could meaningfully affect geometry (e.g. `border`, `lateralProfile`).

---

## 5. Conversion model

### 5.1 Coordinate system

OpenDRIVE local frame:
- `s` runs along the reference line of a road, `s ∈ [0, road.length]`.
- `t` is perpendicular to the reference line in the road plane; **`t > 0` is to the left** looking in the `+s` direction.
- `z` is elevation above the reference plane.
- The reference line is defined in the global X/Y plane with heading `hdg` (radians) at `s = 0` of each geometry record.

Right-hand traffic convention (assumed unless a road says otherwise):
- Lanes with `id < 0` drive in `+s`.
- Lanes with `id > 0` drive in `−s`.

OpenDRIVE does not encode traffic hand globally; v1 uses the RHT convention. A future PR can read an `autoware:traffic_hand` hint from the header `<userData>`.

### 5.2 Projection

- The OpenDRIVE `<header>/<geoReference>` string (a PROJ.4 string) is read but **not required**.
- Parsed `x, y, z` are treated as local Cartesian meters.
- The `lanelet2_io` `Projector` passed to `parse()` is used in reverse to fill `GPSPoint` lat/lon/ele for each `Point3d`. If no projector is supplied, lanelet2's default behavior applies (same as OSM parser today).
- No separate PROJ dependency is added in v1. If `<geoReference>` disagrees with the supplied projector, an error is appended (but the supplied projector wins).

### 5.3 Reference line evaluation

Each `<geometry>` record provides `(s, x, y, hdg, length)` at its start. Given an offset `ds ∈ [0, length]` into the record, the evaluator returns `(x(ds), y(ds), hdg(ds))`.

**`line`**

```
x(ds) = x + ds * cos(hdg)
y(ds) = y + ds * sin(hdg)
hdg(ds) = hdg
```

**`arc`** (curvature `κ`, constant)

```
θ    = hdg + κ * ds
dx_l = sin(θ)/κ − sin(hdg)/κ        // local integration
dy_l = −cos(θ)/κ + cos(hdg)/κ
x(ds), y(ds) = (x, y) + rotate_hdg(...)  // directly: already in world
hdg(ds) = θ
```

(with `κ → 0` falling back to `line`.)

**`spiral`** (clothoid, `curvStart`, `curvEnd`, curvature varies linearly in `ds`)

Parametrize with standard Fresnel integrals:

```
c(ds) = curvStart + (curvEnd − curvStart) * ds / length
θ(ds) = hdg + curvStart * ds + (curvEnd − curvStart) * ds² / (2 * length)
```

Position requires integrating `cos(θ(s)), sin(θ(s))` — implemented via Fresnel `C(t), S(t)` after an affine change of variable. See `fresnel.hpp/.cpp`; implementation uses the rational approximation from _Numerical Recipes_ 3e §6.8.1 (accurate to ~1e-7 over the full real line, no external dependency).

**`poly3`** (deprecated in OpenDRIVE 1.6+; still read)

In the geometry record's local uv frame (`u` = along initial hdg, `v` = perpendicular):

```
v(u) = a + b*u + c*u² + d*u³
```

The caller walks `u` from `0` to `length`; `(u, v)` is rotated by `hdg` and translated by `(x, y)` to reach world coordinates. `hdg(ds)` is taken from the tangent of `v(u)`.

**`paramPoly3`**

```
u(p) = aU + bU*p + cU*p² + dU*p³
v(p) = aV + bV*p + cV*p² + dV*p³
```

`p` ranges over `[0, 1]` (default) or `[0, length]` depending on `pRange`. Mapping from `ds` to `p` (for arc-length–correct sampling) requires a 1-D root find on arc length:

```
s(p) = ∫₀ᵖ √(u'(q)² + v'(q)²) dq
```

Solved by fixed-step Simpson integration (tabulated at creation time) followed by linear interpolation when sampling — error is bounded by the tabulation step, default `1e-3 * length`.

### 5.4 Elevation profile

```
z(s) = a + b*ds + c*ds² + d*ds³,   ds = s − record.s
```

Piecewise, with records ordered by `s`. For `s` beyond the last record, the last polynomial is extrapolated (matches OpenDRIVE convention). For `s` before the first record, `z = 0`.

### 5.5 Lane section, lane offset, lane width

- `<laneOffset>` is a piecewise cubic giving the `t`-offset of lane 0's centerline from the reference line:
  ```
  t_center(s) = a + b*ds + c*ds² + d*ds³,  ds = s − record.s
  ```
- Each `<laneSection>` contains `<left>`, `<center>`, `<right>`. Lane 0 (in `<center>`) has zero width; its sole purpose is to carry road marks (skipped in v1).
- Lane widths are stacked from the reference outward:

  For right lanes (`id = −1, −2, …`), walking from lane 0 outward (decreasing id):
  ```
  t_inner(lane −1) = t_center(s)
  t_outer(lane −1) = t_center(s) − width(lane −1, ds_section)
  t_inner(lane −2) = t_outer(lane −1)
  ...
  ```
  Similarly for left lanes with `+` signs.

- `<width>` records use `ds_section = s − (laneSection.s + width.sOffset)` as their local parameter.

### 5.6 From `(s, t)` to world

```
(x_ref, y_ref, hdg) = reference_line(s)
z_ref               = elevation(s)
x = x_ref − sin(hdg) * t
y = y_ref + cos(hdg) * t
z = z_ref              // superelevation ignored
```

### 5.7 Sampling strategy

- Default step: `0.5 m` along `s`, configurable via a `Parser` config option (see §8).
- Each geometry record is sampled independently at its native `s`; lane-section boundaries, `<width>` breakpoints, `<laneOffset>` breakpoints, and `<elevation>` breakpoints are inserted as additional sample points to avoid missing discontinuities.
- Adjacent lanes in the same section **share the same `LineString3d`** for their common boundary — the linestring is sampled once, constructed once, and referenced (by `Id`) as the right bound of lane _n_ and the left bound of lane _n+1_. This is a stronger guarantee than coincident points in two separate linestrings: lanelet2 identifies lane adjacency by linestring-`Id` equality, so duplicate boundaries (same coordinates, different `Id`) would break neighbour queries. See §5.11 for the full sharing model.
- At road–road boundaries, end-of-road sample points are unified with start-of-next-road sample points when they match within `1e-2 m` (hard-coded tolerance for v1; can be lifted to config later).

### 5.8 Lane → Lanelet mapping

A drivable lane becomes one `Lanelet`.

- `type = driving` → `lanelet.attributes().subtype = "road"`
- `type = biking` → `subtype = "bicycle_lane"` (see [extra_lanelet_subtypes.md](extra_lanelet_subtypes.md))
- `type = sidewalk` → NOT created as a Lanelet in v1 (would need a crosswalk/lanelet distinction); its boundary is still produced as a LineString and stored in the map for later use.
- `type = shoulder` → `subtype = "road_shoulder"`
- All other lane types (`restricted`, `parking`, `median`, `border`, `stop`, `none`, etc.) are **skipped** in v1 with a non-fatal warning.

Bound direction:

| Lane id | Driving dir | Lanelet left bound            | Lanelet right bound           | Point order      |
| ------- | ----------- | ----------------------------- | ----------------------------- | ---------------- |
| `< 0`   | `+s`        | inner boundary (closer to 0)  | outer boundary (farther out)  | ascending `s`    |
| `> 0`   | `−s`        | inner boundary (closer to 0)  | outer boundary (farther out)  | descending `s`   |

Lanelet `attributes`:
- `type = "lanelet"`
- `subtype` as above
- `location = "urban"` (default; override via `<userData>` — future PR)
- `one_way = "yes"` (OpenDRIVE lanes are always directional)
- `speed_limit`: **not set** in v1 (no `<speed>` parsing)
- traceability back to the source: `opendrive:road_id`, `opendrive:lane_section`, `opendrive:lane_id`, `opendrive:junction_id` (see §8)

### 5.9 Road linking

For each `<road>`:

- `<link>/<predecessor>` with `elementType = road`:
  - Predecessor's `contactPoint = end` → predecessor's last lane section connects to this road's first section.
  - `contactPoint = start` → predecessor's first section connects to this road's first section (U-turn-like).
- `<link>/<successor>` with `elementType = road`: symmetric at the `+s` end.
- The parser uses each lane's `<lane>/<link>/<predecessor @id>` / `<successor @id>` to attach `Lanelet` IDs bidirectionally. The result shows up as shared boundary points (already unified in §5.7) — lanelet2's routing later derives adjacency from geometry, so no `AdjacentLeft`/`Right` relations are explicitly emitted in v1.

### 5.10 Junctions

Each `<junction>` is a set of connecting roads. In v1:

1. The connecting roads listed in `<connection @connectingRoad>` are parsed as ordinary roads (their lanes become lanelets).
2. `<laneLink @from @to>` records resolve `incomingRoad.lane → connectingRoad.lane` and propagate endpoint sharing so boundaries unify at the correct ends.
3. No `RightOfWay` regulatory element is generated. Intersection semantics are the caller's responsibility in v1.

Internal roads (those with `@junction != -1`) are **not** linked via `road.link` — linkage comes entirely from `<connection>` records, per the OpenDRIVE spec.

### 5.11 Point and linestring sharing

Two distinct mechanisms keep the emitted map free of duplicate geometry. Both are load-bearing for correctness, not just for byte-count: lanelet2 routing, adjacency, and topological queries all rely on shared `Id`s, not on coordinate comparison.

**LineString3d sharing — within a lane section.** When two adjacent lanes in the same `<laneSection>` are both emitted as lanelets, the shared boundary is built as a single `LineString3d`. That one linestring is referenced (by `Id`) as:

- the **outer (right) bound of lane _n_** and the **inner (left) bound of lane _n−1_** for lanes with `id < 0`;
- the **inner (right) bound of lane _n_** and the **outer (left) bound of lane _n+1_** for lanes with `id > 0` (mirrored because traversal direction is flipped).

The inner bound of lane ±1 is sampled from the reference line (offset by `<laneOffset>`), and the outermost bound of the outermost lane belongs solely to that lane's lanelet. Non-drivable lanes (skipped types in §5.8) still produce their inner linestring — it remains the outer boundary of the neighbouring drivable lane.

**Point3d deduplication — everywhere boundaries abut but are not the same linestring.** A spatial hash keyed on `(round(x/ε), round(y/ε), round(z/ε))` with `ε = merge_tol_m` (default `0.01 m`) looks up whether a candidate sample matches an existing `Point3d`; if so, the existing `Id` is reused. This applies:

- at **lane-section boundaries on the same road** — adjacent sections share only the connecting `Point3d`s, not the full `LineString3d`, because lane count / width / structure can change at the section break;
- at **road–road `<link>` contact points** — endpoint `Point3d`s are unified across `predecessor` / `successor` (matching within `merge_tol_m`);
- at **junction entries and exits** — each `<connection>` connecting road shares endpoint `Point3d`s with the incoming and outgoing roads per `<laneLink>`.

Point sharing keeps the map topologically connected; linestring sharing additionally keeps adjacency queries correct. Together they avoid accidental split boundaries and stray duplicate linestrings. Because only §5.9 road-linking happens at the Point level (not the LineString level), §5.9's note that "no `AdjacentLeft`/`Right` relations are explicitly emitted" is specifically about cross-road adjacency — within-section adjacency is implicit in shared linestring `Id`s and needs no regulatory element.

### 5.12 Signals → regulatory elements

Traffic-light signals under `<road>/<signals>/<signal>` are emitted as [`AutowareTrafficLight`](../include/autoware_lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp) regulatory elements.

**Classification.** A `<signal>` is treated as a traffic light when **both**:

- `dynamic="yes"`, and
- its `type` is in the configurable allowlist `autoware_opendrive/traffic_light_types` (default: `"1000001", "1000002", "1000013"` — standard OpenDRIVE traffic-light vocabulary, country-agnostic).

Dynamic signals outside the allowlist emit a non-fatal warning and are skipped. Non-dynamic signals (speed limits, static signs, etc.) are silently ignored in v1.

**World position.** Given signal `(s, t, zOffset, hOffset, height, width)` on road R:

```
(x_ref, y_ref, hdg) = reference_line_R(s)
z_ref               = elevation_R(s)
x0 = x_ref − sin(hdg) * t
y0 = y_ref + cos(hdg) * t
z0 = z_ref + zOffset
θ  = hdg + hOffset
```

The physical light is a 2-point `LineString3d` representing the bottom edge of the housing, centered on `(x0, y0, z0)` and oriented by `θ`:

```
p1 = (x0 − cos(θ) * width/2, y0 − sin(θ) * width/2, z0)
p2 = (x0 + cos(θ) * width/2, y0 + sin(θ) * width/2, z0)
```

The linestring carries `type="traffic_light"`, `subtype="red_yellow_green"` (default), and `height = signal.height`. If `width` is missing or zero, the two points are placed `0.3 m` apart along `θ` and a non-fatal warning is emitted.

**Light bulbs.** Not produced in v1 — OpenDRIVE does not describe individual bulb positions, and inferring them from `type` would require a country-specific table. Future work.

**Stop line.** Synthesized at the signal's `s`, perpendicular to the reference line, spanning only the lanes given by `<signal>/<validity>`:

```
lanes_applicable = lanes in <validity fromLane..toLane> at laneSection(s)
                   ∩ { lanes whose driving direction crosses `s` towards the signal }
t_min = min(outer t of applicable lanes)
t_max = max(outer t of applicable lanes)

(x_ref, y_ref, hdg) = reference_line_R(s)
z_ref               = elevation_R(s)
p_a = (x_ref − sin(hdg)*t_min, y_ref + cos(hdg)*t_min, z_ref)
p_b = (x_ref − sin(hdg)*t_max, y_ref + cos(hdg)*t_max, z_ref)
```

Emitted as `LineString3d` with `type = "stop_line"`. Point order: from the side with smaller lane `id` to the side with larger lane `id` (stable across lanelets that share the line).

If `<validity>` is omitted, applicable lanes default to **all same-direction lanes** in the current lane section: `id < 0` if `orientation="+"`, `id > 0` if `orientation="-"`, both directions if `orientation="none"` (with a warning — the signal is likely mis-authored).

Stop lines are **not** read from `<object type="roadMark" subtype="stopLine">` in v1. If such an object exists alongside a signal, the synthesized line wins; a non-fatal note is emitted.

**Association.** The regulatory element is attached to every lanelet whose source lane is in `lanes_applicable`. The physical-light linestring is the sole `refers` member; the synthesized stop line is `ref_line`.

**`<signalReference>`** on a different road reuses the same regulatory element, looked up by source signal `id`. Lanelets on the referencing road are attached to the existing element; no new linestrings are created. The reference's own `<validity>` narrows the set of attached lanelets on the referencing road.

**`<signal>/<dependency>` and root-level `<controller>`.** Controller groupings (which lights change together) are **not** parsed in v1. Each light is an independent regulatory element.

**Attributes on the emitted regulatory element:**

| Key                          | Value                                    |
| ---------------------------- | ---------------------------------------- |
| `type`                       | `"regulatory_element"`                   |
| `subtype`                    | `"traffic_light"`                        |
| `opendrive:signal_id`        | source OpenDRIVE signal `id`             |
| `opendrive:signal_type`      | source OpenDRIVE signal `type`           |
| `opendrive:signal_subtype`   | source OpenDRIVE signal `subtype`        |
| `opendrive:signal_country`   | source OpenDRIVE signal `country`        |
| `opendrive:road_id`          | road that owns the signal (owner road, not referencing roads) |

---

## 6. Public API

`include/autoware_lanelet2_extension/io/autoware_opendrive_parser.hpp`:

```cpp
namespace lanelet::io_handlers
{
class AutowareOpenDriveParser : public Parser
{
public:
  using Parser::Parser;

  std::unique_ptr<LaneletMap> parse(
    const std::string & filename, ErrorMessages & errors) const override;

  static constexpr const char * extension() { return ".xodr"; }
  static constexpr const char * name()      { return "autoware_opendrive_handler"; }
};
}  // namespace lanelet::io_handlers
```

### 6.1 Config options (via `io::Configuration`)

Exposed through the standard `lanelet2_io` config map (strings passed to `lanelet::load`):

| Key                                  | Type   | Default | Meaning                                  |
| ------------------------------------ | ------ | ------- | ---------------------------------------- |
| `autoware_opendrive/sample_step_m`   | double | `0.5`   | Sampling step along `s`                  |
| `autoware_opendrive/merge_tol_m`     | double | `0.01`  | Point-dedup tolerance                    |
| `autoware_opendrive/skip_sidewalks`  | bool   | `true`  | Skip `sidewalk` lanes as lanelets (still produces boundary linestrings when false; kept for future) |
| `autoware_opendrive/traffic_light_types` | string | `"1000001,1000002,1000013"` | Comma-separated OpenDRIVE signal `type` codes recognized as traffic lights (see §5.12) |

Unknown keys emit a warning and are ignored.

---

## 7. Error handling

Behavior mirrors `AutowareOsmParser`:

- Fatal (throws `lanelet::ParseError`):
  - XML cannot be opened or is malformed.
  - `<OpenDRIVE>` root element missing.
- Non-fatal (appended to `ErrorMessages`, parse continues):
  - Unknown / unsupported element or attribute.
  - `<border>` encountered.
  - `<lateralProfile>` encountered (non-empty).
  - Lane type not in {`driving`, `biking`, `shoulder`, `sidewalk`}.
  - Junction `<connection>` referencing a non-existent road.
  - `<geoReference>` disagreeing with supplied projector.
  - `<signal dynamic="yes">` whose `type` is not in `traffic_light_types` (skipped).
  - Traffic-light `<signal>` with missing / zero `width` (fallback geometry used).
  - Traffic-light `<signal>` with `orientation="none"` (both directions assumed).
  - `<signalReference>` pointing to an unknown signal `id` (skipped).
  - `<signal>/<validity>` referencing lanes not present at the signal's `s` (clipped to existing lanes).

Error messages are prefixed with `[autoware_opendrive_handler]` and include the road id / junction id / source line where the offending element was found (pugixml gives us `offset_debug`).

---

## 8. Output attributes

All emitted lanelets carry at minimum:

| Key           | Value                                     |
| ------------- | ----------------------------------------- |
| `type`        | `"lanelet"`                               |
| `subtype`     | `"road"`, `"bicycle_lane"`, `"road_shoulder"` |
| `location`    | `"urban"`                                 |
| `one_way`     | `"yes"`                                   |
| `opendrive:road_id`         | source OpenDRIVE `<road @id>`            |
| `opendrive:lane_section`    | source `<laneSection>` index within the road (0-based, ordered by ascending `s`) |
| `opendrive:lane_id`         | source OpenDRIVE `<lane @id>` — signed lane offset from the reference line (negative = right, positive = left, `|id|` = lanes out from centerline) |
| `opendrive:junction_id`     | set iff the source road has `@junction != -1` |

Together, `opendrive:road_id` + `opendrive:lane_section` + `opendrive:lane_id` uniquely identify the emitting `<lane>` within a single `.xodr` file, giving full traceability from a lanelet back to its source.

Traffic-light linestrings (the `refers` member, `type="traffic_light"`) carry:

| Key                          | Value                                    |
| ---------------------------- | ---------------------------------------- |
| `type`                       | `"traffic_light"`                        |
| `subtype`                    | `"red_yellow_green"`                     |
| `height`                     | source `<signal @height>` (meters)       |
| `opendrive:signal_id`        | source OpenDRIVE signal `id`             |

Synthesized stop-line linestrings (the `ref_line` member) carry `type="stop_line"` and `opendrive:signal_id` pointing back to the owning signal.

See §5.12 for the full set of attributes on the `AutowareTrafficLight` regulatory element itself.

The `opendrive:*` tags are intended for traceability, debugging, and round-trip analysis. Autoware runtime code should not depend on them for driving logic, but external tooling (map editors, QA pipelines, round-trip validators) is free to key off them.

The map root gets a `MetaInfo`-style pair stored in the first lanelet's attributes (matching how [`AutowareOsmParser::parseVersions`](../lib/autoware_osm_parser.cpp) handles versions), with `opendrive:format_version` taken from `<header @revMajor.revMinor>`.

---

## 9. Testing

Unit tests live under `autoware_lanelet2_extension/test/` (new subdirectory `test_opendrive_parser/`). The primary fixture is the existing [`resource/Town10HD.xodr`](../resource/Town10HD.xodr) (OpenDRIVE 1.4, ~18 000 lines, full CARLA town with junctions, traffic lights, stop/yield signs, and `<signalReference>`s). A handful of tiny hand-written fixtures live under `test/fixtures/opendrive/` only where a Town10HD check would be too noisy to isolate the behavior (e.g., single-primitive geometry, error paths, allowlist override).

### 9.1 Geometry and evaluator unit tests

Hand-written fixtures, closed-form comparisons:

1. **Geometry primitives** — each of `line` / `arc` / `spiral` / `poly3` / `paramPoly3` evaluated at known points, verified against closed-form or high-precision reference values (tolerance `1e-4 m`).
2. **Elevation profile** — piecewise cubic continuity + extrapolation behavior.
3. **Point deduplication** — two adjacent sample points within `merge_tol_m` share an `Id`; just beyond tolerance they do not.

### 9.2 Town10HD.xodr integration tests

Load once per test fixture, then assert on the resulting `LaneletMap`:

4. **Loads cleanly** — `lanelet::load("resource/Town10HD.xodr")` returns a non-empty map with zero fatal errors and `errors.size() == expected_warning_count` (see below for expected classes of warnings).
5. **Road / junction counts** — number of lanelets matches the sum of driving-lane instances across `<laneSection>`s in the source; number of junctions matches `<junction>` count.
6. **Traffic-light classification** — every `<signal dynamic="yes" type="1000001">` produces exactly one `AutowareTrafficLight` regulatory element, **except** where a `<signalReference>` points to the same signal id from another road (in which case the element is shared). In Town10HD this is the dominant case; concrete spot checks:
   - signal id `944` (road that owns it + its `<signalReference>`s on sibling roads): single reg element, multiple attached lanelets.
   - signal id `943`: same pattern; used by `<signalReference>` at road offset `3.27e+1`.
7. **Non-light signals skipped** — `<signal>` ids `946`, `947`, `948` (stop signs, `type="206"`, `dynamic="no"`) and `963` (yield sign, `type="205"`) produce **no** regulatory elements and no warnings (non-dynamic path is silent).
8. **`orientation="none"` warning** — signal ids `955` and `956` (traffic lights with `orientation="none"`) each emit exactly one non-fatal warning of the "both-directions assumed" class, and still produce a reg element.
9. **Stop-line geometry** — for a spot-checked traffic light (e.g., signal `944`), the synthesized `ref_line` lies at the signal's `s` on its owner road, its endpoints coincide with the outer `t` of the lanes in `<validity>` (within `merge_tol_m`), and `type == "stop_line"`.
10. **`<signalReference>` dedup** — total `AutowareTrafficLight` reg-element count equals (unique dynamic signal ids with `type` in allowlist), **not** the sum of `<signal>` + `<signalReference>` occurrences.
11. **Controllers ignored** — the `<controller>` blocks at the end of Town10HD do not produce regulatory elements (v1), and do not emit errors.
12. **Factory registration** — `lanelet::load("resource/Town10HD.xodr")` works without naming the parser explicitly (extension-based dispatch).
13. **Traceability attributes** — every emitted lanelet carries `opendrive:road_id`, `opendrive:lane_section`, and `opendrive:lane_id`. Asserted by (a) picking a well-known `(road_id, lane_section, lane_id)` triple from Town10HD and checking that exactly one lanelet matches, and (b) reverse: no emitted lanelet has an `opendrive:road_id` that doesn't exist in the source file.

### 9.3 Error paths and config knobs

Small hand-written fixtures:

13. **Malformed XML** throws `lanelet::ParseError`.
14. **`<lateralProfile>`** (non-empty) emits a non-fatal error and parse continues.
15. **Dangling `<signalReference>`** (id not present in any road's `<signals>`) emits a non-fatal error and is skipped.
16. **Allowlist override** — a fixture with `<signal dynamic="yes" type="274">` is skipped with the default allowlist but becomes a traffic light when `autoware_opendrive/traffic_light_types="274"` is passed via config.
17. **Zero-width signal** — a light with `width=0` gets a fallback 0.3 m linestring and a non-fatal warning.

Expected-warning-count assertions in §9.2 are updated as new warning classes are added; the test keeps a single `expected_warning_count` constant sourced from a header to avoid brittle per-PR churn.

---

## 10. Dependencies

- Already available in the package build (no new deps in v1):
  - `pugixml` (used by `AutowareOsmParser`)
  - `lanelet2_core`, `lanelet2_io`
- Standard library only for Fresnel integrals and Simpson integration (see §5.3).

---

## 11. Limitations and future work

1. **Non-traffic-light signals** (speed limits, warning signs, priority signs, stand-alone stop signs) → regulatory elements / lanelet attributes. Traffic lights are already handled in v1; this extends the coverage.
2. **Stop lines from `<object type="roadMark" subtype="stopLine">`** → prefer the authored geometry over the synthesized line when present.
3. **Light bulbs** (`light_bulbs` role on `AutowareTrafficLight`) → requires a country/type table mapping `<signal @type,@subtype>` to bulb layouts.
4. **Controller groups** (`<controller>` + `<signal>/<dependency>`) → one regulatory element per controller, with the grouped lights as multiple `refers` members.
5. **Road marks** (`<lane>/<roadMark>`) → line-string type/color/width tags on lane bounds.
6. **Objects** (`<object>`) → barriers, crosswalks, speed bumps as regulatory elements or boundary annotations.
7. **Junction priority / right-of-way** → `RightOfWay` regulatory element.
8. **Superelevation** → z tilt on boundary points.
9. **`<speed>` records** → `speed_limit` attribute on lanelets.
10. **Traffic-hand override** via `<userData>` hint.
11. **Crosswalks and sidewalks** → promote `sidewalk` lanes or dedicated `<object type="crosswalk">` to Autoware crosswalk regulatory elements.
12. **Round-trip writer** (OpenDRIVE output) — out of scope indefinitely.
13. **Lane `border` records** → currently ignored; add once we have a test map that uses them.

---

## 12. File / module plan (delivery checklist)

- [ ] `include/autoware_lanelet2_extension/io/autoware_opendrive_parser.hpp`
- [ ] `lib/autoware_opendrive_parser.cpp` — class + factory registration
- [ ] `lib/opendrive/xodr_types.hpp`
- [ ] `lib/opendrive/xodr_reader.{hpp,cpp}` — pugixml → POD
- [ ] `lib/opendrive/fresnel.{hpp,cpp}`
- [ ] `lib/opendrive/geometry.{hpp,cpp}` — ref-line evaluators
- [ ] `lib/opendrive/lane_builder.{hpp,cpp}`
- [ ] `lib/opendrive/road_linker.{hpp,cpp}`
- [ ] `lib/opendrive/junction_linker.{hpp,cpp}`
- [ ] `lib/opendrive/signal_builder.{hpp,cpp}` — `<signals>` → `AutowareTrafficLight` + synthesized stop lines (see §5.12)
- [ ] `CMakeLists.txt` — add sources, target-link `pugixml`
- [ ] `test/test_opendrive_parser/*` + fixtures
- [ ] `package.xml` — no change expected (pugixml already transitively present; to confirm during implementation)

---

## 13. Resolved design decisions

All previously-open questions have been resolved in favor of the proposed defaults. The body of this spec already reflects each decision; they are restated here for traceability.

1. **Traffic hand.** Hard-coded right-hand traffic in v1 (§5.1). A future PR can read an `autoware:traffic_hand` hint from `<userData>`.
2. **Lane type `sidewalk`.** Skipped in v1 (§5.8) — no lanelet emitted; boundary linestrings only.
3. **`<geoReference>` mismatch.** Warn only (§5.2, §7) — the user-supplied `Projector` wins.
4. **Config plumbing.** Upstream `lanelet::io::Configuration` (§6.1) so `lanelet::load(..., config)` works.
5. **Sampling near discontinuities.** Unconditional break-point samples (§5.7).
6. **Traffic-light allowlist default.** `"1000001,1000002,1000013"` (§5.12, §6.1). Per-country presets deferred until we see a map that needs them. The `autoware_opendrive/traffic_light_types` knob handles one-off overrides.
7. **Stop-line placement.** Synthesized at the signal's `s` (§5.12). The known upstream offset from the painted stop line is documented; honoring `<object subtype="stopLine">` is future work (§11).
8. **Canonical test fixture.** [`resource/Town10HD.xodr`](../resource/Town10HD.xodr) is the primary integration fixture (§9.2). It is OpenDRIVE 1.4, so the parser must accept 1.4 input — see §2 caveat.
