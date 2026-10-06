# OpenBus Completion TODO

This is the ordered plan for deciding when the current prototype is fully
implemented. A feature is only considered complete when its runtime path,
configuration behavior, and regression evidence all exist.

## Status Legend

- `[x]` Implemented and validated in the current repository.
- `[~]` Runtime or parser work exists, but validation or edge-case coverage is
  still incomplete.
- `[ ]` Not implemented, or currently only parsed/consumed for compatibility.

## 0. Establish the completion baseline

- [x] Make `Docs/SCREENSHOT_VALIDATION_PLAN.md` truthful: environment maps and
  any other features marked fully supported must either have evidence or be
  moved to the incomplete section. Environment maps are now explicitly
  exploratory, and the alpha mode-2 depth gap is called out.
- [x] Add a short compatibility/status table to the README linking each major
  feature to its implementation and test evidence.

Done when a fresh checkout can be configured, built, launched, and evaluated
against one unambiguous support matrix.

## 1. Lock down physics and wheel stability

- [x] Fixed-rate ODE stepping with bounded catch-up.
- [x] Per-wheel suspension, steering, wheel rotation, braking, and telemetry.
- [x] Script-authored `Axle_Springfactor_*` values scale each ODE wheel's
  configured spring rate while preserving damper and anti-roll forces.
- [x] Independent axle suspension and live ODE wheel rendering mode.
- [x] Raw W/A/S/D input seeds player throttle, steering, and brake variables
  before each physics update. The live vehicle path runs scripts first, then
  applies the script-produced `M_Wheel` as wheel torque; the E400 scripts own
  engine state and Voith gear selection. The legacy throttle-based drivetrain
  remains only for standalone physics probes.
- [~] High-speed bump stability: local-axis damping, softened contact response,
  and steering-rate damping are implemented. A deterministic stress probe now
  exercises bump traversal, steering reversals, braking, and fast wheel contact
  with two- and three-axle configurations; broader solver/contact tuning remains.
- [x] Add a deterministic physics stress probe covering straight-line bumps,
  alternating bumps, hard steering transitions, braking over bumps, and
  high-speed wheel contact. Assert finite poses/velocities, bounded angular
  speed, no excessive suspension travel, and left/right symmetry.
- [ ] Use the stress probe to tune remaining ODE solver/contact parameters;
  do not accept visual smoothness alone as proof of stability.
- [~] Articulated configuration is parsed and exposed, but articulated
  multi-body physics is not complete.
- [ ] Implement articulated body sections, joints, collision, and rendering,
  or explicitly remove articulated vehicles from the supported scope.

Done when the physics probe is repeatable and reports numerical bounds, and
both supported vehicles pass it without wheel shake or explosive motion.

## 2. Complete the live wheel contract

- [x] Wheel bindings, axle mapping, steering, suspension, and rotation are
  connected to the live ODE wheel bodies.
- [x] Optional `OPENBUS_WHEELS_FROM_ODE=1` mode is documented and runnable.
- [~] ODE wheel translation and orientation were corrected and smoke-tested,
  but there is no dedicated renderer assertion for wheel-to-arch alignment.
- [ ] Add a renderer/pose diagnostic or capture test proving all configured
  wheels remain aligned with their authored pivots while steering, pitching,
  rolling, and suspending.
- [ ] Validate the default animation path and ODE path side by side so the
  opt-in mode cannot silently regress normal rendering.

Done when MAN DL05 and E400 wheel poses pass stationary, steering, bump, and
motion checks in both supported rendering modes.

## 3. Finish material and texture behavior

- [x] Base OBJ materials, alpha modes, no-Z flags, texture addressing,
  lightmaps, nightmaps, transmaps, free/script textures, UV translation,
  material changes, and environment-map shader plumbing have runtime paths.
- [~] Environment maps are wired through texture loading and a reflection
  shader, but need representative assets and repeatable visual validation.
- [~] Transparent and no-depth batches are sorted by render type and depth;
  validate difficult overlapping glass/decal cases and reflection passes.
- [ ] Match the documented `[matl_alpha]` mode-2 depth behavior. The current
  transparent pass disables depth writes, so blended surfaces do not occlude
  later transparent geometry as OMSI specifies; add an overlap regression.
- [~] Bump-map rendering now applies derivative-based normal-map shading from
  the configured texture and strength. Representative OMSI visual parity and
  parallax semantics still need validation.
- [ ] Verify `matl_change`, CTC, and texture-array layer selection with a
  real vehicle configuration, including activation-variable changes over time.
- [ ] Verify missing, delayed, and failed auxiliary textures do not leave
  stale GL bindings or permanently blank materials.
- [ ] Add automated or captured evidence for TGA, PNG/JPEG, DXT DDS, indexed/RLE
  TGA, alpha, address modes, light/night/trans/free textures, and envmaps.

Done when every supported material keyword has a documented runtime semantic,
representative assets exercise it, and screenshots/logs or focused tests prove
it.

## 4. Implement dynamic displays and lighting

- [x] Lua script texture storage and pixel operations exist in `ScriptRuntime`,
  and authored script-texture dimensions are initialized before scripts run.
- [x] Retain and connect `[scripttexture]` definitions to model materials,
  including dimensions, slot/index, and texture lifetime through the live
  script-texture upload path. Update cadence remains script-driven.
- [x] Implement `[texttexture]` and `[texttexture_enh]` runtime surfaces:
  bitmap text layout, color/alpha, sizing, material binding, and updates from
  script string variables. OFT fonts are loaded by internal name from the
  direct `Fonts/` directory, including glyph rectangles, color/alpha bitmaps,
  block-color mode, fallback glyphs, and `@` line breaks.
- [x] Implement `[usescripttexture]` and `[usetexttexture]` material binding
  to live texture surfaces, including Lua/native pixel, rectangle, text, load,
  lock, readback, and filtering operations.
- [x] Cache text source values and dynamic surface revisions so unchanged text
  is not rasterized or uploaded to OpenGL every frame.
- [~] Implemented the basic `[illumination_interior]`/`[interiorlight]`
  runtime path: assigned mesh groups now receive bounded controller-driven
  emissive colour. Positional falloff and `[light_enh]`, `[light_enh_2]`, and
  `[spotlight]` runtime semantics remain incomplete.
- [ ] Add a focused E400 display/light test using real model CFG data and
  screenshots at day/night or brightness changes.

Done when a configured display changes from script input and interior lights
change from their controlling variables without manual texture replacement.

### E400 OMSI/OpenBus comparison gaps

  of consuming them only for parser alignment; verify the dashboard LCD,
  odometer, and center display against the OMSI reference capture.
  substitution. The active E400 configuration uses these records for dashboard
  and body variants, so retaining their payloads is not sufficient for visual
  parity.
  assigned meshes, but `[light_enh_2]` and `[spotlight]` remain storage-only;
  the comparison still shows dashboard-lighting parity gaps.
  semantics, including strength and lighting interaction, then compare the
  dashboard binnacle and cab surfaces again.
  OMSI fonts, layout, and display-background/alpha semantics. Enhanced horizontal
  alignments 0–5, authored grid spacing, and `Refresh_Strings`-gated changed-string
  updates are implemented; the fallback rasterizer and remaining format details
  still differ from OMSI.
  the plate as `ident`, and rasterize its text texture on initialization; the
  default E400 plate is visible on the real model capture.
  odometer parity against the reference capture.
  that checks the dashboard LCD, odometer, warning lamps, illumination, and
  text surfaces independently of camera framing.
for the current parser. The remaining visual differences are therefore mostly
recognized-but-incomplete semantics, not missing keyword dispatch entries.

## 5. Complete vehicle/model configuration coverage

- [x] Core bus identity metadata (`[friendlyname]`, `[description]`, and
  `[type]`), mass, bounding box, center of gravity, axles, cameras, model
  assembly, animations, parents, visibility, and wheel data are loaded and
  exposed through `VehicleConfig` for the current vehicles.
- [~] `[mouseevent]` identifiers are retained on model parts, and W/A/S/D
  transitions are dispatched to the player script runtime as key state and
  optional key-specific entry points. Exact transformed triangle picking is
  active, including hand-cursor hover feedback, captured `trigger_<event>` /
  `trigger_<event>_drag` callbacks, matching `trigger_<event>_off` release
  callbacks, and the `I`-key clickable wireframe overlay. Scenery-object
  mouse-event dispatch is not connected; configurable key maps remain
  incomplete.
- [~] BUS asset references for `[paths]`, `[passengercabin]`, `[sound_ai]`,
  `[number]`, and `[registration_list]` are normalized and retained. The
  passenger-cabin CFG is now loaded for driver/passenger positions,
  illumination groups, and entry/exit path points. The `[paths]` CFG is now
  loaded for indexed path points/links, step-sound packs, and room-height
  transitions. `[number]` and `[registration_list]` files are now loaded
  separately in order and paired by entry index; AI sound selection remains
  incomplete.
- [~] `[registration_automatic]`, `[registration_free]`, and
  `[kmcounter_init]` now seed runtime registration and split odometer state.
  Explicit registration/index overrides and deterministic automatic numbers
  are available; persistence and full OMSI selection UI remain incomplete.
- [~] Bus-model `[collision_mesh]` records now build an ODE triangle collider;
  model `[boundingbox]` overrides the BUS box, `[nocollision]` filters a
  mesh-scoped collider, and the BUS box remains the fallback. Scenery-object
  collision records are still parse-only because scenery has no runtime.
- [~] Passenger-cabin CFG loading, driver/passenger positions, illumination
  groups, entry/exit path points, and basic passenger path graph loading are
  active. Boarding/alighting, passenger movement, passenger mass, route/path
  selection, and HOF/timetable integration remain incomplete.
- [ ] Complete registration persistence and full selection UI; implement or
  explicitly scope out AI/network sections, route/timetable/HOF data, and
  vehicle-specific view systems.
- [ ] Implement sound configuration coverage beyond the current player sound
  runtime path, including selecting `sound_ai` for AI vehicles.
- [ ] Add diagnostics that distinguish "recognized and active" from
  "recognized but ignored" for every parsed configuration record.

### Bus features still absent from the runtime

The following OMSI BUS/model features are currently absent or only represented
by parser alignment and should not be described as supported:

- Passenger-cabin connection records, boarding, alighting, passenger movement,
  and passenger mass. Basic `[passengercabin]` positions, illumination groups,
  and entries/exits are loaded and covered by `OpenBusVehicleConfigProbe`.
- Route/path selection, timetable/HOF lookup, stop announcements, and
  arrival-board data used by the script system. Basic `[paths]` point/link
  graphs and step metadata are loaded and covered by `OpenBusVehicleConfigProbe`.
- Registration-number persistence and full selection UI. Ordered number/plate
  pairing, runtime automatic/free selection, the distinct `number`/`ident`
  strings, initial plate-text rendering, and live odometer publication are
  active; parser coverage remains in
  `OpenBusVehicleConfigProbe`.
- AI/network vehicle sections, coupling/cable behavior, and articulated
  multi-body physics despite the articulated flag being parsed.
- Non-OMSI font edge cases such as Windows-1252 glyph validation and remaining
  text format semantics remain. `[texttexture_enh]` alignments 0–5 and grid
  spacing are handled by the text runtime. `[interiorlight]`, `[light_enh]`,
  `[light_enh_2]`, and `[spotlight]` light emission remain incomplete.
- Configurable keyboard binding files and the complete OMSI input action map;
  the current runtime dispatches physical W/A/S/D transitions to scripts.
- Remaining script system data providers such as route, terminus, ticket,
  passenger-count, ground-height, and arrival-board lookups; safe fallbacks
  remain in `ScriptRuntime`.

Done when unsupported configuration is visible in diagnostics and the README
claims only behavior that is active in the runtime.

### Documented keyword gap register

The reference documents cover more OMSI dialects than the current
vehicle-centric runtime consumes. The following records are documented but
are either unknown, parser-only, or missing their runtime behavior. Keep these
items separate from the active E400 completion work above so parsing a record
for alignment is not mistaken for supporting it.

#### BUS vehicle configuration

- [x] Preserve and expose `[friendlyname]`, `[description]`, and `[type]` in
  `VehicleConfig`.
- [ ] Implement BUS-level `[fixed]` and `[scriptshare]` semantics.
- [ ] Retain the labels and implement the view selection behavior for
  `[view_schedule]` and `[view_ticketselling]`; current parsing only records
  that the keyword occurred and validates that a driver camera exists.
- [ ] Parse and use `[cog]` in a consistent way with `[schwerpunkt]`.
- [ ] Store and apply `[rollwiderstand]` and `[rot_pnt_long]`; both are
  currently consumed without physics semantics.
- [x] Apply the parsed `[inv_min_turnradius]` steering constraint to the live
  physics steering limit.
- [ ] Apply the parsed `[ai_deltaheight]` correction to the AI vehicle path.
- [ ] Add the documented vehicle-family records `[rowdy_factor]`, `[boogies]`,
  `[ai_brakeperformance]`, `[ai_veh_type]`, and `[sinus]`, or explicitly mark
  them unsupported when their family-specific semantics cannot be preserved.
- [ ] Implement articulated/coupled records: `[coupling_front]`,
  `[coupling_back]`, `[coupling_front_character]`, `[couple_back]`, and
  `[couple_front_open_for_sound]`.
- [ ] Implement `[control_cable_front]` and `[control_cable_back]` for
  electrical/control-variable transfer between coupled vehicles.
- [ ] Implement `[new_attachment]` attachment transforms and the rail/trolley
  records `[contact_shoe]` and `[rail_body_osc]`, or explicitly scope them out
  with diagnostics.
- [ ] Complete `[sound_ai]` selection for AI vehicles; the path is retained,
  but the AI sound runtime is not selected.
- [ ] Complete `[registration_automatic]`, `[registration_free]`, and
  `[kmcounter_init]` runtime selection/persistence as tracked in the E400
  comparison section.

#### Bus-model configuration

- [~] Complete runtime material semantics for `[matl_bumpmap]`,
  `[matl_envmap]`, `[matl_freetex]`, `[matl_lightmap]`, and `[matl_nightmap]`;
  one or more shader, blending, controller, or texture-binding stages remain
  incomplete. `[matl_transmap]` alpha sampling is implemented, but its
  interaction with blended depth behavior still needs regression coverage.
- [ ] Define the behavior of `[matl_item]` instead of treating it as a
  no-op material marker.
- [ ] Apply full `[rendertype]`, `[fixed]`, and `[absheight]` semantics to
  geometry transforms and passes. `[rendertype]` currently recognizes
  `surface` and numeric values, but not the documented `presurface` and
  `on_surface` names; the scenery-object loader also discards the value.
- [~] `[isshadow]` meshes now render with their authored transparent material
  and a ground-plane projection; model collision meshes/bounds and
  `[nocollision]` now affect the bus ODE collider. Scenery-object collision
  construction and shadow grounding against elevated road surfaces remain
  incomplete.
- [~] Implement basic `[illumination_interior]`/`[interiorlight]` emission,
  controller variables, and mesh assignments. `[light_enh]`, `[light_enh_2]`,
  and `[spotlight]` geometry, falloff, and render ordering remain pending.
  Fix parsing as part of this work: `[light_enh]` currently treats its alpha
  bitmap path as numeric, and `[light_enh_2]` drops the time constant when the
  optional bitmap field is absent.
- [x] Correct `[viewpoint]` filtering: player exterior, player interior, and
  non-player vehicles now use their individual documented bits (`1`, `2`, and
  `4`). Model visibility and sound playback share zero-means-all,
  any-overlapping-bit mask matching; triggered sounds and ambient loops are
  filtered against the active player view. `OpenBusViewpointProbe` covers
  individual and combined masks, including exclusion of non-player-only meshes
  from player exterior views.
- [ ] Apply `origin_from_mesh` from mesh transformation metadata in every
  supported mesh path; the current origin operation is identity when that
  metadata is unavailable.
- [ ] Implement `[smoke]` exhaust/particle emission instead of consuming its
  nineteen fields for parser alignment.
- [ ] Apply `[VFDmaxmin]` display bounds and `[tex_detail_factor]` to display
  and texture rendering.
- [ ] Implement runtime selection/substitution for `[CTC]` and `[CTCTexture]`
  beyond retaining their parsed mappings.
- [ ] Add the documented texture-change records `[texchanges]`,
  `[newtexchangemaster]`, and `[entries]`, including count validation and
  runtime replacement behavior.
- [ ] Implement model `[terrainmapping]` when terrain/material CFGs are
  supported; it is documented but currently not dispatched.
- [ ] Finish OMSI-compatible `[texttexture]` and `[texttexture_enh]` font,
  vertical/layout, and alpha semantics; horizontal alignments 0–5 and authored
  grid spacing are supported, but the fallback font rasterizer and remaining
  enhanced-format flags are still not fully compatible.

#### General CFG dialects

- [ ] Add a surface/material CFG path for `[puddles]`, `[moisture]`,
  texture `[surface]`, and `[NightMapMode]`, including wetness and
  surface-type data. Scenery `[surface]` and `[nocollision]` are currently
  retained as flags without a runtime collision consumer; `[NightMapMode]` is
  parsed but discarded. `[maplight]` is also consumed without generating
  tile lightmaps.
- [ ] Implement passenger-cabin `[noticketsale]` eligibility for ticket
  interactions; the keyword is currently unhandled.
- [ ] Connect scenery-object `[model]`, `[script]`, `[varnamelist]`,
  `[stringvarnamelist]`, and `[mouseevent]` records to an active runtime, or
  diagnose them as unsupported instead of retaining parse-only state.
- [ ] Complete passenger-cabin behavior for `[linkToNextVeh]`,
  `[linkToPrevVeh]`, and `[stamper]`; basic `[drivpos]`, `[passpos]`,
  `[entry]`, `[exit]`, and illumination records do not yet provide passenger
  movement, boarding, or ticket interaction.
- [ ] Complete path runtime use of extended `[pathpnt]` records and
  `[stepsoundpack]`; loading the basic graph is not equivalent to passenger
  route selection or step-sound playback.
- [~] Player sound CFG runs untriggered `[sound]` records with `[volcurve]`
  controls as ambient loops and applies their configured base gain. This
  enables authored window-dependent ambience (for example, `cabwindowopen`)
  without window-specific mixer logic; the full OMSI sound contract remains
  incomplete.
- [ ] Complete sound spatialization and control semantics for `[loopsound]`
  and `[3d]` beyond the current player sound loading/trigger path.
- [ ] Add AI/network configuration support for `[ailist]`, `[aigroup_2]`,
  `[aigroup_depot]`, `[busstop]`, `[signalroute]`, and `[StnLink]`.
- [ ] Add environment configuration support for `[sky_textures]`,
  `[cloudtype]`, and `[startdate]`.
- [ ] Add fare/currency support for `[currency]` and `[coin]`.
- [ ] Add configurable input CFG support for `[game]`, `[ctrl]`, `[axis]`,
  and keyboard-context `[entry]`; current controls are hardcoded to the
  runtime key map.
- [ ] Define safe handling for general metadata `[friendlyname]`,
  `[description]`, and `[usrinfo]`; never copy `[usrinfo]` values into logs,
  diagnostics, or source control.

#### Documented script API gaps

The following documented system macros still dispatch to safe placeholder
handlers rather than real route, timetable, ticket, passenger, terrain, or
arrival-board data:

- [ ] HOF/route lookups: `GetTerminusIndex`, `GetTerminusCode`,
  `GetTerminusString`, `GetBusstopIndex`, `GetBusstopString`, `GetRouteIndex`,
  `GetRouteTerminusIndex`, `GetBusstopCount`, and `GetRouteBusstopIdent`.
- [ ] Timetable lookups: `GetTTLineString`, `GetTTTerminusIndex`,
  `GetTTBusstopCount`, `GetTTBusstopIndex`, `GetTTDelay`, `GetTTBusstopName`,
  `GetTTBusstopArr`, and `GetTTBusstopDep`.
- [ ] Ticket/passenger lookups: `GiveChangeCoin`, `GetTicketName`,
  `GetTicketValue`, `GetHumanCountOnPathLink`, and `GetHumanCountOnSeat`.
- [ ] World and depot lookups: `GetHeightAbovePoint`, `GetDepotStringGlobal`,
  `GetArrBusLine`, `GetArrBusTerminus`, and `GetArrBusTimeDiff`.
- [~] Verify OMSI parity for `NrSpecRandom`; OpenBus now provides deterministic
  seeded output, but its algorithm and range still need compatibility evidence.
- [ ] Add host-owned updates for default-only documented system variables:
  `Time`, `Day`, `Month`, `Year`, `DayOfYear`, `Pause`, `NoSound`,
  `PrecipType`, `PrecipRate`, `coll_pos_*`, `coll_energy`,
  `Weather_Temperature`, `Weather_AbsHum`, `AutoClutch`, `SunAlt`, and
  `wearlifespan`.
- [ ] Replace default-only values for documented variable families whose
  producers are absent: `AI_*`, `AI_Scheduled_*`, `PAX_Entry#_*`,
  `PAX_Exit#_*`, `GivenTicket`, `humans_count`, `FF_Vib_*`, `Snd_*`,
  `Cabinair_*`, `Dirt_*`, `TrafficPriority*`, `Axle_Brakeforce_*`,
  `Axle_SurfaceID_*`, `articulation_*`, `boogie_*`, and
  `contactshoe_*`.
- [ ] Add runtime producers for documented vehicle/scenery/human strings and
  state such as `act_route`, `act_busstop`, `SetLineTo`, `yard`,
  `file_schedule`, `Refresh_Strings`, `Switch`, `LastMovedDist`, `PAX_State`,
  and `HeightOfSeat`, or document their supported default-only behavior.

Each item above needs a parser/runtime test or an explicit unsupported-status
diagnostic before it can be moved to the implemented baseline.

## 6. Rendering and platform completeness

### Map support

Implemented foundation: `global.cfg` and tile/terrain parsing, configurable
entrypoint spawn, synchronous full-map terrain/scenery/road render construction with
distance/frustum culling, a 3x3 spawn-neighborhood terrain collision patch,
supported O3D scenery, tree preview cards, and a subset of `.sli` road profiles.
These are not equivalent to full OMSI map support; see the README's map scope
for the explicit boundary.

#### Keyword support audit (current implementation)

- `[map]` and `[entrypoints]` are loaded; entrypoint zero is the default, with
  optional index/name selection. `[groundtex]` records are retained; the first
  base texture is rendered by default. `OPENBUS_MAP_GROUNDTEX` can select a
  different base texture uniformly as a preview/debug override. Per-tile DDS
  sidecar paths are inventoried, but their suffix mapping, detail layers, and
  seasonal changes are not applied.
- `[version]`, `[terrain]`, `[object]`, `[spline]`, `[spline_h]`,
  `[splineAttachement]`, and `[splineAttachement_repeater]` have typed parser
  paths. Ordinary objects, road strips, and same-tile spline attachments have
  runtime paths, but object labels, spline skew/mirror behavior, cross-tile
  attachment chains, and repeater-master resolution remain incomplete.
- `[attachObj]` fixed fields and version-dependent IDs are parsed and retained,
  including non-destructive typed attach-point/rotation values while preserving
  raw fields and opaque label counts. Placement/rendering remains unsupported
  because parent anchor semantics are unresolved. `[water]` markers and binary sidecars are
  parsed/validated and summarized at load, but no water surfaces are rendered.
  `[variable_terrain]` and `[variable_terrainlightmap]` marker presence is
  parsed and reported, but dynamic terrain/lightmap rendering is unsupported.
  `[varparent]` IDs, four raw fields from `[rule]`/`[kill_rule]`, the
  `[spline_terrain_align]` marker, and `[spline_terrain_align_2]` spline data
  are retained on supported records, but none of these modifiers affects
  runtime placement or visibility yet.
- Chrono tile parsing now retains the complete source lines and comment/version,
  types `[selobject]` and `[selspline]` integer IDs before the opaque object
  payload, and preserves selector override lines as opaque data. Selector/date
  resolution is not applied to the map; `[delete]`, `[typ]`, and `[relabel]`
  have no independent runtime semantics.
  Global identity/description, version, background/map-camera data, marker flags,
  seasons, and traffic-density records are retained as metadata, but fields such
  as `[worldcoordinates]` have no runtime effect; timetable/HOF and AI-list map
  state remain out of scope. Unsupported `global.cfg` and tile sections retain
  their raw payload lines with source line numbers; their runtime semantics
  remain unimplemented.
- `.map.water` presence and count-plus-four-float records are decoded and
  validated; rendering remains deferred because the four heights' spatial
  order is not verified. Numeric-suffix `.map.<n>.dds` paths are inventoried
  per tile, but their payloads and suffix-to-groundtex mapping are unresolved.
  `.map.LM.bmp` and `.map.terrain_0.rdy` paths/presence are inventoried but
  their payloads are not decoded or rendered.
  Unimplemented bracketed sections in tile files and `global.cfg` now produce
  line-numbered warnings and preserve each section's opaque payload lines;
  no runtime meaning is inferred from them.

- [x] Add curved centerline tessellation for the recognized Freyfurt asphalt
  profiles. Signed-radius geometry is checked by `OpenBusMapSplineGeometryProbe`
  against linked Grande Porto spline endpoints in both turn directions; a
  zero-radius straight case and invalid inputs are covered. This verifies the
  centerline only, not cross-section sidewalks, markings, seams, or lighting.
- [ ] Add targeted, repeatable in-context screenshot evidence for curved-road
  geometry, sidewalks, markings, texture phase, and seams at linked spline
  boundaries.
- [ ] Load general scenery `.x` meshes, or keep them explicitly unsupported;
  tree SCO previews currently use crossed textured planes rather than their
  editor/helper mesh.
- [~] Read a subset of road profiles from `.sli` files (`[texture]`, `[profile]`,
  `[profilepnt]`); height profiles, complete texture/material semantics,
  banking, marking behavior, and representative visual evidence remain
  incomplete. Road UV V now includes a valid parsed v11+ spline chain offset;
  continuity and OMSI parity still need validation against installed linked
  spline assets.
- [x] Decode `[object]`, `[spline]`, and `[spline_h]` field layouts by tile
  version, including legacy spline links and v11+ chain offsets; version 5/7/11
  and current-version fixtures are covered by `OpenBusMapConfigProbe`.
- [x] Apply SCO/model `[absheight]` to ordinary map scenery so its vertical
  placement is not offset by terrain height; model and tree-SCO parsing have
  regression coverage in `OpenBusModelConfigProbe`.
- [~] Ordinary object rotation and same-tile spline attachment/repeater
  placement have runtime paths and parser/placement probes. A deterministic
  probe covers ordinary object x/y/z order, tile-origin translation, rotation
  retention, and relative/absolute terrain height; add visual transform
  regression evidence against installed map assets. Finish cross-tile chains,
  repeater-master resolution, and `[attachObj]` rendering. `[attachObj]`
  records now have typed IDs and preserved fixed/optional fields, covered by
  modern and legacy parser fixtures. `[varparent]` references are parsed on
  object, attached-object, and spline-attachment records; applying parent
  transforms remains open. `[attachObj]` label counts remain opaque and its
  optional tail is preserved independently of the count.
- [~] Add ODE road trimeshes for the runtime-visible supported `.sli` strips in
  the spawn tile's 3x3 neighborhood. Rendering and physics now share generated
  road-section geometry; a stability probe checks a raised static surface over
  terrain, and a geometry probe covers straight, curved, and elevated profiles.
  Still validate contact against representative installed curves/sidewalks and
  tune duplicate contacts where road strips overlap terrain; physics is not
  streamed as the bus leaves the neighborhood.
- [ ] Implement scenery collision honoring `.sco`/model `[collision_mesh]`,
  `[boundingbox]`, and `[nocollision]`; define safe static-world ownership and
  add placement/collision probes before connecting to ODE.
- [ ] Connect scenery scripts and mouse events only after their runtime
  semantics and ownership are defined.
- [~] Decode `.map.water` sidecars into per-surface four-height records;
  determine corner/quadrant ordering and implement water rendering. Per-tile/
  detail/seasonal ground textures, map lightmaps, and Chrono variant resolution
  remain. Chrono selector IDs and opaque override payload lines are parser-only.
- [~] Map rendering now applies the 500 m distance limit and view-frustum
  culling to resident terrain/road geometry, and groups scenery by spatial tile
  so rejected chunks do not scan every instance. All map tiles are still
  synchronously parsed/uploaded at startup and remain resident; there is no
  asynchronous map build, dynamic collision update, or GPU unload/streaming.
  Add runtime visibility/performance evidence and make map construction
  non-blocking before claiming streaming support.
- [ ] Update terrain and road collision as the player leaves the current 3x3
  spawn-neighborhood patch; preserve a bounded physics/collision residency
  window independently from render visibility.
- [x] Select map entrypoints at startup by zero-based index or case-insensitive
  name through `OPENBUS_MAP_ENTRY`; the default remains the first entrypoint.
  Selection and invalid-index behavior are covered by `OpenBusMapConfigProbe`.
- [ ] Add HOF/timetable-backed map state. Defer
  scheduled AI and signal behavior until route/signal formats provide enough
  verified data; the inspected Grande Porto signal-route file is only a
  header/template.

- [x] OpenGL 3.3 renderer, VBO model path, cameras, mirrors, frustum culling,
  LOD selection, texture cache, crash reports, tracing, and profiling hooks
  are present.
- [~] Reflection, async loading, texture scaling, and material batching are
  opt-in/performance-sensitive paths and need repeatable comparison runs.
- [ ] Compare the vehicle draw-list/material-selection reuse and static
  ground/grid VBO changes using repeated, uninstrumented runs; report frame
  p50/p95 and keep traced scope totals separate from frame time.
- [ ] Add renderer regression coverage for opaque/transparent list reuse,
  script-driven material changes, and view-specific reflection culling and
  transparent sorting before broadening those caches.
- [ ] Attribute `Vehicle::drawBatch.prepareAndSubmit` with finer-grained CPU
  profiling; test same-view transmap depth-prepass reuse only if material
  assembly is still material after the existing submission-state caches.
- [ ] Measure road-feature mesh construction and upload separately; cache its
  geometry only with explicit invalidation for changed `RoadBump` inputs.
- [ ] Recheck reflection setup cost in a longer capture. If viewport querying
  is confirmed as a bottleneck, track the app-owned viewport dimensions rather
  than querying GL state, and verify framebuffer/context transitions.
- [ ] Add GPU/presentation timing to reflection and draw benchmarks so CPU
  submission time is not mistaken for GPU execution or swap/present wait.
- [ ] Add repeatable performance captures for startup time, frame time, GPU
  workload, reflection intervals, texture scaling, and material batching.
- [x] Audit the renderer for OpenGL compatibility-profile API use; see
  `Docs/OPENGL_API_AUDIT.md` (no active compatibility API calls found).
- [ ] Remove the unused compatibility-era texture-environment enum fallback
  definitions in `src/RenderLoop.cpp` after confirming no platform header or
  generated source relies on them; retain constants that have active core-GL
  uses.
- [ ] Validate Windows and Linux helper scripts from clean build directories.
- [ ] Run clang-tidy/format checks and resolve relevant warnings in the final
  supported configuration.

Done when performance options have documented tradeoffs and both primary build
platforms have a reproducible build-and-smoke procedure.

## 7. Final release gate

- [ ] Run all focused probes, physics stress tests, parser/configuration tests,
  and material/display tests from a clean build.
- [ ] Run the complete screenshot matrix for MAN DL05 and E400, retaining the
  build revision, asset revision, logs, and expected results.
- [ ] Remove or label stale claims in README and the reference documents.
- [ ] Review generated JSON, texture logs, crash output, and profiling output
  for accidental repository artifacts.
- [ ] Mark this file complete only after every remaining item is either done or
  explicitly moved into an approved out-of-scope list.

## Already implemented baseline

The current code already provides the following baseline and these should not
be reimplemented as new work:

- ODE world, fixed stepping, bounded catch-up, configurable axle layouts, and
  simplified chassis/wheel physics.
- Suspension, anti-roll, steering, wheel rotation, braking, tyre friction,
  telemetry, and recent wheel/contact stability changes.
- OMSI bus/model parsing for the current MAN DL05 and E400 assets.
- OBJ conversion/loading, coordinate conversion, animation origins,
  `animparent`, visibility variables, LOD/viewpoint filtering, and live ODE
  wheel rendering.
- OpenGL VBO/shader rendering, base/auxiliary textures, alpha/no-Z behavior,
  mirrors, camera controls, tracing, profiling, and crash reporting.