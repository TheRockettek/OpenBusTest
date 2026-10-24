# OpenBus Completion TODO

This is the ordered plan for deciding when the current prototype is fully
implemented. A feature is only considered complete when its runtime path,
configuration behavior, and regression evidence all exist.

## Status Legend

- `[x]` Implemented and validated in the current repository.
- `[~]` Runtime or parser work exists, but validation or edge-case coverage is
  still incomplete.
- `[ ]` Not implemented, or currently only parsed/consumed for compatibility.
- `[o]` Explicitly outside the currently supported scope.

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
- [~] The wheel/static contact defaults use the more compliant settings, and
  the deterministic stress probe passes for both axle configurations (maximum
  angular speed 0.227 rad/s; suspension travel 0.237 m or less). Further solver
  tuning is blocked on a validated vehicle/reference envelope; the probe alone
  does not establish native OMSI parity. Do not accept visual smoothness alone
  as proof of stability.
- [o] Articulated configuration remains parseable for inspection, but
  articulated vehicles are outside the currently supported scope; runtime
  physics, collision, and rendering treat vehicles as single-body.

Done when the physics probe is repeatable and reports numerical bounds, and
both supported vehicles pass it without wheel shake or explosive motion.

## 2. Complete the live wheel contract

- [x] Wheel bindings, axle mapping, steering, suspension, and rotation are
  connected to the live ODE wheel bodies.
- [x] Optional `OPENBUS_WHEELS_FROM_ODE=1` mode is documented and runnable.
- [~] ODE wheel translation and orientation were corrected and smoke-tested.
  Static default/ODE app captures of MAN DL05 and Studio Polygon 400MMC show no
  obvious wheel-to-arch mismatch. `OpenBusCameraMathProbe` now asserts that the
  ODE wheel-render transform places each mesh center on its wheel mount under
  four chassis orientations; authored tire-to-arch clearance remains
  asset-specific and unverified.
- [x] Extend the deterministic two-/three-axle stress probe with authored
  wheel-pivot alignment bounds throughout steering, pitch, roll, and suspension
  motion; stationary alignment is checked separately.
- [~] Deterministic two-/three-axle pose bounds cover dynamic wheel placement,
  and static default/ODE captures now run from isolated working directories.
  Model-specific steering, bump, and motion comparisons remain unverified; the
  current capture routine writes screenshots relative to its working directory.

Done when MAN DL05 and E400 wheel poses pass stationary, steering, bump, and
motion checks in both supported rendering modes.

## 3. Finish material and texture behavior

- [x] Base OBJ materials, alpha modes, no-Z flags, texture addressing,
  lightmaps, nightmaps, transmaps, free/script textures, UV translation,
  material changes, and environment-map shader plumbing have runtime paths.
- [~] Environment maps are wired through texture loading and a reflection
  shader; a GPU probe now verifies directional sampling and blend opacity with
  a generated map. Representative assets and in-game visual parity remain.
- [~] Transparent batches sort back-to-front with render type as a tie-breaker,
  and transmap triangles sort back-to-front within each batch. Probes cover
  overlapping alpha/no-Z-write behavior, both ordering rules, and separate
  reflection triangle accounting; asset-level glass/decal overlaps and actual
  mirror-framebuffer parity remain to validate.
- [x] Match `[matl_alpha]` mode-2 depth writes unless `[matl_noZwrite]` is
  authored; the OpenGL overlap probe verifies that a blended surface occludes
  later geometry and that the explicit no-depth-write override still works.
- [~] Bump-map rendering now has GPU regression coverage for generated-map
  directional shading, zero strength, and strength clamping. Representative
  OMSI visual parity and parallax semantics still need validation.
- [~] Parser-built material states verify `matl_change` item selection, and a
  focused runtime-helper probe covers case-insensitive CTC replacement and
  texture-array layer clamping. Representative MAN DL05/E400 asset parity and
  end-to-end texture reload behavior remain unverified without those assets.
- [~] A GPU regression verifies that a missing auxiliary transmap binds
  texture zero after a prior material used a transmap. An asset-request probe
  covers delayed decode completion, terminal decode failure, and a later valid
  request; vehicle-side GL upload and same-entry retry/recovery still lack an
  injected runtime test.
- [~] Automated probes cover PNG RGBA, 32-bit alpha TGA, indexed/RLE TGA,
  DXT1/DXT4/DXT5 DDS, transmap alpha, lightmap, and free-texture rendering.
  JPEG decoding is covered by a compact grayscale fixture, and all five
  texture address modes are checked from CFG parsing through GL enum mapping.
  A GPU material probe verifies nightmap overlays, black-texel neutrality, and
  zero-strength behavior, plus directional environment-map sampling and blend
  opacity with a generated map.

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
- [x] Bind scenery `[usetexttexture]` and `[usescripttexture]` materials to
  independent per-placement runtime surfaces; generated text glyphs blend over
  transparent backgrounds. Runtime probes cover isolated text data, but visual
  validation against installed directional-sign assets remains.
- [~] Implemented the basic `[illumination_interior]`/`[interiorlight]`
  runtime path: assigned mesh groups receive controller-driven point-light
  shading. A GPU probe verifies light color isolation, zero strength, and
  distance attenuation. OMSI intensity parity and `[light_enh]`, `[light_enh_2]`,
  and `[spotlight]` runtime semantics remain incomplete.
- [~] A focused E400 display/light regression remains blocked on the external
  E400 model CFG/assets, which are not included in this checkout; parser and
  runtime probes do not replace the requested visual comparison.
- [~] In-game lighting flicker has been reported, but the bundled fixtures do
  not reproduce it and the affected light source (interior, nightmap, exterior,
  or scenery) is not identified. Diagnose against the installed model/map
  assets before changing light-controller timing or shader behavior.
- [x] Fix reproduced graphics-binding leaks: mirror target initialization and
  resizing now invalidate the texture cache, and primitive uploads explicitly
  bind their destination buffer after asset streaming. Reset the human-gait
  shader uniform before indexed scene draws so terrain and other indexed
  geometry cannot inherit pedestrian motion. GPU regressions verify that
  repeated mirror resizes preserve HUD/scene pixels, dynamic and cached
  overlays neither use nor overwrite streamed mesh geometry, and indexed scene
  geometry remains static after an animated human draw. These regressions do
  not establish that every reported in-game disappearance has the same cause;
  keep the separate lighting-flicker investigation open.

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
  separately in order and paired by entry index. AI vehicles select
  `[sound_ai]` when configured and fall back to `[sound]`; broader AI sound
  playback coverage remains incomplete.
- [~] `[registration_automatic]`, `[registration_free]`, and
  `[kmcounter_init]` now seed runtime registration and split odometer state.
  Explicit registration/index overrides and deterministic automatic numbers
  are available; persistence and full OMSI selection UI remain incomplete.
- [~] Bus-model `[collision_mesh]` records now build an ODE triangle collider;
  model `[boundingbox]` overrides the BUS box, `[nocollision]` filters a
  mesh-scoped collider, and the BUS box remains the fallback. Scenery-object
  collision records are still parse-only because scenery has no runtime.
- [~] Passenger-cabin CFG loading and a deterministic per-vehicle service
  runtime now cover waiting counts, timed boarding/alighting, dwell state, and
  seat occupancy. Legacy TTP calls and station-ID-only trips with complete
  directed StnLink paths can trigger scheduled AI-bus stops and boarding;
  passengers are assigned their next stop as a destination. Scheduled queues
  are shared by stop ID and render as scheduled humans independently of the
  ambient AI pedestrian cap. Scheduled AI departures replenish depleted
  queues to their initial size, and player buses replenish the served stop
  after departing; both preserve larger queues when demand remains. Reserved
  boarders animate from the roadside flank
  queue position through the configured cabin path into their assigned seat on
  both scheduled AI and player buses; the same passenger model identity is
  retained from the queue through boarding, the occupied seat, and alighting.
  Queue display and service reservation share one stop/queue-position identity
  mapping so the nearest visible rider is the one reserved to board.
  Terminal calls without an onward stop do not board riders or consume the
  shared stop queue.
  At destination stops, seated
  passengers now animate from their assigned seat to the roadside before the
  seat is freed; when cabin path links are connected, boarders and alighters
  follow the directed passenger-path graph between the configured entry/exit
  and their seat. Shared queue consumption gives simultaneous buses distinct
  passengers and preserves each rider identity through seating; first boarding
  priority rotates independently among vehicles sharing each stop queue. A
  scheduled AI bus now remains in service at its route end until its final-stop
  passenger alighting completes.
  Boarding animations retain the exact timetable call index, avoiding a
  mismatched stop position when a route repeats a stop ID or shares it with
  another route. The player-bus retrigger latch also follows that exact route
  call rather than a stop ID shared by unrelated calls.
  TTP station offsets project onto the correct preceding spline entry, including
  subdivided and reversed runtime route segments.
  Waiting queues shared by stop ID render at the currently served timetable
  call when a bus is boarding, or otherwise at the nearest visible call,
  independently of asynchronous AI bus model loading. Waiting passengers have
  a subtle idle gait before boarding; active boarders and alighters use the
  full movement animation.
  Disconnected
  or incomplete cabin graphs fall back to direct seat-to-curb animation.
  Walking passengers now face the active path segment during cabin movement
  and use the authored endpoint orientation when movement completes.
  Waiting queues and boarding/alighting positions use the correct passenger
  side for left- and right-hand traffic maps, using the map's north-zero road
  heading for curb and queue offsets.
  Player buses board and alight at nearby mapped calls when stopped and
  direction-aligned. Player stop discovery is registered from matched
  timetable routes independently of the scheduled AI bus population; it still
  requires a loadable bus configuration and routable route. Passenger mass,
  passenger ticket purchase, fare
  transactions, and physical/audible coin output remain incomplete. Map
  ticket-pack/currency records now feed ticket and
  change-coin script macros for player and AI buses.
- [ ] Complete registration persistence and full selection UI; implement or
  explicitly scope out AI/network sections, route/timetable/HOF data, and
  vehicle-specific view systems.
- [~] AI vehicles select the configured `sound_ai` bank with a player-bank
  fallback when it is absent; `VehicleConfigProbe` verifies both cases. Broader
  AI sound-event playback and sound-bank compatibility remain incomplete.
- [~] Unknown sound triggers are ignored rather than played; `SoundEngineProbe`
  verifies the unknown-event no-op. The listed Cotterell names and counts are
  absent from this checkout and its retained captures, so mapping them to
  configured events remains blocked on the source sound CFGs or an event trace.
- [ ] Add diagnostics that distinguish "recognized and active" from
  "recognized but ignored" for every parsed configuration record.

### Bus features still absent from the runtime

The following OMSI BUS/model features are currently absent or only represented
by parser alignment and should not be described as supported:

- Passenger mass and native cabin/door semantics. Timed boarding/alighting is
  connected to live scheduled
  AI/player bus stops, with movement following configured entry/exit path links
  when a connected cabin graph is available, fallback boarding/alighting
  animations between the roadside and assigned cabin seats, and occupied seats
  rendered. Basic   `[passengercabin]` seat positions, illumination groups, entries/exits, and
  path links are loaded and covered by `OpenBusVehicleConfigProbe`. When a
  connected graph exists, pathfinding follows one-way and bidirectional cabin
  links, and timetable offsets position stops within their mapped spline
  segments; detailed native door/aisle semantics and passenger mass remain
  unverified.
- Route/path selection, timetable/HOF lookup, stop announcements, and
  arrival-board data used by the script system. Basic `[paths]` point/link
  graphs and step metadata are loaded and covered by `OpenBusVehicleConfigProbe`.
- Registration-number persistence and full selection UI. Ordered number/plate
  pairing, runtime automatic/free selection, the distinct `number`/`ident`
  strings, initial plate-text rendering, and live odometer publication are
  active; parser coverage remains in
  `OpenBusVehicleConfigProbe`.
- AI/network vehicle sections and coupling/cable behavior remain absent.
  Articulated multi-body vehicles are explicitly outside the currently
  supported scope; the parsed flag does not enable articulated runtime behavior.
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
- [x] Apply `[rollwiderstand]` as an authored non-negative longitudinal force;
  absent values retain the legacy mass-scaled fallback, and explicit zero is
  preserved. Native OMSI physics parity remains unverified.
- [ ] Store and apply `[rot_pnt_long]`; it is currently consumed without
  physics semantics.
- [x] Apply the parsed `[inv_min_turnradius]` steering constraint to the live
  physics steering limit.
- [x] Apply `[ai_deltaheight]` to initial and map-route AI vehicle placement;
  player vehicles remain unaffected.
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
- [x] Select `[sound_ai]` for AI vehicle sound-bank loading, falling back to
  `[sound]` when no AI-specific path is authored. Full AI ambient/spatial sound
  behavior remains unverified.
- [ ] Complete `[registration_automatic]`, `[registration_free]`, and
  `[kmcounter_init]` runtime selection/persistence as tracked in the E400
  comparison section.

#### Bus-model configuration

- [~] Complete runtime material semantics for `[matl_bumpmap]`,
  `[matl_envmap]`, `[matl_freetex]`, `[matl_lightmap]`, and `[matl_nightmap]`;
  one or more shader, blending, controller, or texture-binding stages remain
  incomplete. `[matl_transmap]` alpha sampling is implemented, but its
  interaction with blended depth behavior still needs regression coverage.
- [~] Vehicle runtime selects controller-driven `[matl_item]` states and
  applies supported auxiliary material properties. Independent item texture
  replacement and complete native item scope remain unimplemented.
- [~] Apply complete `[rendertype]`, `[fixed]`, and `[absheight]` semantics.
  Scenery `[rendertype]` now routes recognized named/numeric categories, and
  model `[fixed]` suppresses animation-parent inheritance while preserving
  local animation. Native pass/transform parity and broader `[absheight]`
  semantics remain unverified.
- [~] `[isshadow]` meshes now render with their authored transparent material
  and a ground-plane projection; model collision meshes/bounds and
  `[nocollision]` now affect the bus ODE collider. Scenery-object collision
  construction and shadow grounding against elevated road surfaces remain
  incomplete.
- [~] Basic `[illumination_interior]`/`[interiorlight]` controller-driven
  point-light shading, mesh assignments, and distance attenuation are active;
  a GPU probe covers color, zero strength, and near/far attenuation. OMSI
  intensity parity remains unverified. `[light_enh]`, `[light_enh_2]`, and
  `[spotlight]` geometry and render ordering remain pending.
  `[light_enh]` and `[light_enh_2]` preserve all required numeric fields and
  accept their optional bitmap when present; enhanced-light runtime behavior is
  still pending.
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
- [~] `[CTCTexture]` replacements are applied case-insensitively to base mesh
  texture names and the resolver is unit-tested. `[CTC]` template/paint-scheme
  selection and representative-asset substitution remain unsupported.
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
- [ ] Implement passenger-cabin `[noticketsale]` eligibility and the observed
  `[ticket_sale]`, `[ticket_sale_money_point_2]`, and
  `[ticket_sale_change_point_2]` ticket-machine behavior/locations; these
  interactions are currently unhandled.
- [x] Resolve scenery-object `[model]` CFG references for static map geometry
  and material loading.
- [~] Scenery SCO `[script]`, `[varnamelist]`, `[stringvarnamelist]`, and
  `[constfile]` resources now feed per-placement `init`/`frame` runtimes, and
  scenery `[visible]` controllers are checked during map drawing. Script-driven
  animations, dynamic materials/textures, mouse-event dispatch, sound callbacks,
  and general `[varparent]` state inheritance remain incomplete. `[varparent]`
  is used to associate `[trafficlight]` signal heads with a parent controller's
  phase schedule; this does not provide general scenery-variable inheritance.
- [ ] Complete passenger-cabin behavior for `[linkToNextVeh]`,
  `[linkToPrevVeh]`, and `[stamper]`; basic `[drivpos]`, `[passpos]`,
  `[entry]`, `[exit]`, and illumination records support stop-service seating
  and boarding animation, and connected authored cabin path links are used for
  passenger movement. Linked-vehicle cabin traversal, stamper behavior, and
  ticket interaction are not yet implemented.
- [~] Load `[pathlink_oneway]` and preserve its directed-link flag alongside
  ordinary `[pathlink]` records; `OpenBusVehicleConfigProbe` distinguishes both
  forms. Passenger movement follows connected directed cabin paths; extended
  `[pathpnt]` records and `[stepsoundpack]` runtime use remain incomplete.
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
- [ ] Implement observed configuration records that remain unhandled in their
  current contexts: SCO `[approachdist]`, `[carpark_p]`, `[collision_mesh]`,
  `[complexity]`, `[crossingproblem]`, `[entrypoint]`, `[joinable]`, `[mass]`,
  `[momentofintertia]`, `[matl_texadress_border]`, `[newanim]`,
  `[nomaplighting]`, `[shadow]` per-mesh cast-shadow behavior,
  `[splinehelper]`, and `[terrainhole]`; model-config
  `[crashmode_pole]`, `[detail_factor]`, `[LightMapMapping]`,
  `[traffic_light]`, and `[traffic_lights_group]`; and spline-profile
  `[length]`, `[matl]`, `[nightmapmode]`, and `[rail_enh]`. Some names have
  implementations in other contexts (for example, `[traffic_light]` data in
  SCO files and `[matl]` model materials); the missing scope is the context
  shown by the runtime warning, not the keyword globally. `[newachse]`
  descriptive text lines are labels rather than configuration tags and are
  ignored; only identifier-like unknown axle fields are reported.
  Model-animation directives such as `anim_rot` and `anim_trans` are
  unbracketed operations inside `[newanim]`, not top-level CFG keywords.
  Likewise, labels such as `Scheiben` in an animation block are data/labels,
  not configuration tags; `anim_rot_x` is not a documented keyword in the
  checked-in format references. Do not add these strings to the unimplemented
  keyword backlog solely because a parser warning reports them. Model-animation
  divider comments beginning with `--` and descriptive labels in
  `[new_attachment]` blocks are ignored; only verified configuration tags or
  supported operation names belong in this register. The prior live warning
  count is not retained as a keyword count because warning tokens can include
  non-keyword text; re-audit source records before publishing a count.
- [ ] Define safe handling for general metadata `[friendlyname]`,
  `[description]`, and `[usrinfo]`; never copy `[usrinfo]` values into logs,
  diagnostics, or source control.

#### Documented script API gaps

The following documented system macros still dispatch to safe placeholder
handlers rather than real route, timetable, passenger-path, terrain, or
arrival-board data:

- [ ] HOF/route lookups: `GetTerminusIndex`, `GetTerminusCode`,
  `GetTerminusString`, `GetBusstopIndex`, `GetBusstopString`, `GetRouteIndex`,
  `GetRouteTerminusIndex`, `GetBusstopCount`, and `GetRouteBusstopIdent`.
- [ ] Timetable lookups: `GetTTLineString`, `GetTTTerminusIndex`,
  `GetTTBusstopCount`, `GetTTBusstopIndex`, `GetTTDelay`, `GetTTBusstopName`,
  `GetTTBusstopArr`, and `GetTTBusstopDep`.
- [~] Ticket/passenger macros: `GiveChangeCoin`, `GetTicketName`, and
  `GetTicketValue` use map ticket/currency data and `GetHumanCountOnSeat` reads
  the current seat-occupancy variables. Ticket purchasing/payment and
  `GetHumanCountOnPathLink` producers remain unimplemented.
- [ ] World and depot lookups: `GetHeightAbovePoint`, `GetDepotStringGlobal`,
  `GetArrBusLine`, `GetArrBusTerminus`, and `GetArrBusTimeDiff`.
- [~] Verify OMSI parity for `NrSpecRandom`; OpenBus now provides deterministic
  seeded output, but its algorithm and range still need compatibility evidence.
- [ ] Add host-owned updates for default-only documented system variables:
  `Time`, `Day`, `Month`, `Year`, `DayOfYear`, `Pause`, `NoSound`,
  `PrecipType`, `PrecipRate`, `coll_pos_*`, `coll_energy`,
  `Weather_Temperature`, `Weather_AbsHum`, `AutoClutch`, `SunAlt`, and
  `wearlifespan`.
- [~] `humans_count` now reports the passenger service's occupied-seat count,
  matching `passengers_onboard` for player and AI vehicles. Replace default-only
  values for documented variable families whose producers are absent:
  `AI_*`, `AI_Scheduled_*`, `PAX_Entry#_*`, `PAX_Exit#_*`, `GivenTicket`,
  `FF_Vib_*`, `Snd_*`,
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
entrypoint spawn, synchronous full-map terrain/scenery/road construction with
distance/frustum culling and a two-tile bus-centered draw/script active window;
supported O3D scenery (including external SCO `[model]` CFGs, transmaps, and
O3D material alpha), tree preview cards, and a subset of `.sli` road profiles.
These are not equivalent to full OMSI map support; see the README's map scope
for the explicit boundary.

#### Keyword support audit (current implementation)

- `[map]` and `[entrypoints]` are loaded; entrypoint zero is the default, with
  indexed selection through `OPENBUS_MAP_SPAWN` and an indexed spawn listing
  when it is unset. Legacy `OPENBUS_MAP_ENTRY` index/name selection remains
  available. The first `[groundtex]` record is the terrain
  base; verified numeric `.map.<index>.dds` sidecars are decoded as alpha masks
  and composite subsequent ground-texture layers per tile. `OPENBUS_MAP_GROUNDTEX`
  still provides a uniform preview/debug override. Repeated `[groundtex]` detail
  images are multiplied over their albedo. Variable terrain/lightmaps and
  seasonal changes remain unsupported.
- `[version]`, `[terrain]`, `[object]`, `[spline]`, `[spline_h]`,
  `[splineAttachement]`, and `[splineAttachement_repeater]` have typed parser
  paths. Ordinary objects, road strips, and same-tile spline attachments have
  runtime paths. SCOs marked `[onlyeditor]` are omitted from map rendering;
  object labels, spline skew behavior, cross-tile attachment chains,
  and repeater-master resolution remain incomplete.
- `[attachObj]` fixed fields and version-dependent IDs are parsed and retained,
  including non-destructive typed attach-point/rotation values while preserving
  raw fields and opaque label counts. Static rendering resolves parent IDs,
  ordered SCO/model `[new_attachment]` definitions, and child rotations; missing
  parents/anchors and cycles are skipped. `[water]` markers and binary sidecars are
  parsed/validated and rendered as provisional translucent tile quads. Their
  inferred SW/SE/NE/NW height ordering, appearance, and multiple-surface behavior
  still need comparison against OMSI; water remains visual-only, not collision.
  `[variable_terrain]` and `[variable_terrainlightmap]` marker presence is
  parsed and reported, but dynamic terrain/lightmap rendering is unsupported.
  `[varparent]` editor ordinals are retained separately from placement IDs and
  checked against tile `Object Nr.` labels; only the traffic-signal parent
  association is consumed at runtime. `[rule]`/`[kill_rule]` apply the supported
  group-specific speed, density, priority, and vehicle-access fields; their
  remaining native semantics are not implemented. `[spline_terrain_align]`
  provisionally tilts marked ordinary,
  attached, and spline-row scenery to the sampled terrain normal without
  changing placement height; OMSI parity is not yet verified.
  `[spline_terrain_align_2]` spline data remains preserved but unsupported.
- Chrono tile parsing retains the complete source lines and comment/version,
  types `[selobject]` and `[selspline]` integer IDs before the opaque object
  payload, and preserves selector override lines as opaque data. Date-ranged
  scenario tile files are now selected at startup by inclusive
  `[startdate]`/`[enddate]`; selector edits, `[delete]`, `[typ]`, and `[relabel]`
  still have no independent runtime semantics.
  Global identity/description, version, background/map-camera data, marker flags,
  seasons, and traffic-density records are retained as metadata, but fields such
  as `[worldcoordinates]` have no runtime effect; timetable/HOF and AI-list map
  state remain out of scope. Unsupported `global.cfg` and tile sections retain
  their raw payload lines with source line numbers; their runtime semantics
  remain unimplemented.
  `.map.water` presence and count-plus-four-float records are decoded and
  validated; rendering remains deferred because the four heights' spatial
  order is not verified. `.map.LM.bmp` and `.map.terrain_0.rdy` paths/presence
  are inventoried but their payloads are not decoded or rendered.
  Unimplemented bracketed sections in tile files and `global.cfg` now produce
  line-numbered warnings and preserve each section's opaque payload lines;
  no runtime meaning is inferred from them.

#### AI vehicle and human traffic

- [x] Use `[sound_ai]` for AI vehicles and apply `[ai_deltaheight]` to their
  placement. Focused vehicle configuration coverage verifies AI/player
  selection and preserves fallback to `[sound]`.
- [~] Spawn random road traffic from configured `.ovh` entries in modern
  `ailists.cfg` road groups on `[path]` type-0 road splines; `.bus` vehicles
  are reserved for the separate timetable-route AI bus pool. `.ttl` departure
  times are attached to their `.ttp` trip metadata and matched buses now wait
  for their next scheduled departure before endpoint admission. Midnight
  rollover is handled; an active bus advances to the next departure only after
  it departs, and missed trips are skipped when its route completes. When a
  route receives multiple pool buses, its sorted departures are partitioned
  across them so each scheduled departure has one owner. Native density
  units/interpolation remain provisional. Startup builds a bounded pool from
  connected road lanes with a forward continuation; isolated splines and
  terminal dead ends are excluded. Road vehicles can enter on connected
  internal lanes at clear, off-screen interior positions in the current active
  tile window, including tiles activated later during streaming. Candidate
  positions keep a vehicle-sized margin from spline endpoints, remain over
  50 m plus vehicle radius from the camera, and preserve collision clearance.
  Scheduled buses remain restricted to incoming endpoint lanes and their
  timetable. Population changes unload vehicles off-screen; route-complete
  actors are recycled from the bounded pool, resetting route progress, speed,
  and junction-wait state. The startup pool remains the hard capacity.
  When no complete timetable match exists, a route sharing at least two
  consecutive timetable splines may use an explicitly logged partial endpoint
  fallback; buses never spawn partway along a route.
- [~] AI vehicles advance with lookahead speed control along directed authored
  lane paths, including scenery junction paths, matching the cyan F2 geometry.
  Matching endpoints connect within 0.5 m horizontally and vertically; road
  vehicles blend small position and heading mismatches between adjacent road
  splines, while retaining authored junction-path curves; rendered yaw follows
  the sampled path tangent. Vehicles make seeded straight/left/right choices
  filtered by minimum turn radius. Normal acceleration/braking limits are
  1.6/3 m/s^2, bend speeds
  target at most 1.8 m/s^2 lateral acceleration, and curve speed has a 2 m/s
  floor (capped by cruise speed); routes beyond the minimum turn radius remain
  rejected at route selection. Following uses a speed-dependent
  1.5 s time headway and 2 m standstill buffer. Oriented swept footprints prevent advancement
  through player/AI/static vehicles. Same-route following respects route order
  across spline boundaries so a rear vehicle's reservation cannot block a
  leader on a curve. A gap- and relative-speed-based following target and a
  gentler signal approach envelope damp speed changes while retaining swept
  collision and stop-line safeguards. Conservative crossing reservations
  age blocked junction movements at 8 priority points per second (capped at 255);
  stopped actors queued at a signal or behind the player/AI bus reset their
  stuck timer, and pending removals are canceled while that wait remains valid.
  any vehicle part (front, center, or rear) inside a junction counts as
  committed. Approaches check the full junction connector and exit clearance
  before entry, preventing a queued merge exit from leaving a vehicle stopped
  across the main road. Committed vehicles use physical swept clearance within
  the junction instead of the larger approach headway, and accrued wait
  survives approach movement while a junction path is in lookahead. Regressions
  cover queued T-junction exits, right-turn wait aging, clearance, and no-overlap.
  Vehicles return to endpoint queues only after
  an unconnected endpoint is off-screen; population reductions happen off-screen.
  Regressions exercise actual
  left/right turns below 4.5 m/s on 10 m curves, radius rejection, following
  gaps at different speeds,
  stopped-player-bus avoidance, crossing progress/separation, long-frame
  blockage, crossing wait-age priority, right-turn merge clearance, and
  four-approach no-overlap/progress. Crossing reservations distinguish
  same-direction traffic on a curved shared path from conflicting movements.
  Fixed-cycle SCO signals linked to road paths hold AI traffic on non-permissive
  phases and release it on states 6-8; regressions cover route-start, loop-wrap,
  conditional stop timing, and bus-priority phase/distance/minimum-green rules.
  Cycle origin, stop-line
  fidelity, signs, native right-of-way and deadlock recovery at live junctions,
  overtaking, lane changing, articulated dynamics, and physical player-to-AI
  collision response are listed in the remaining-work checklist below.
- [x] Hold road AI vehicles at visible unconnected route ends and unload them
  only after the endpoint is outside the camera view and beyond the spawn
  distance threshold. Regression coverage verifies visible-end retention and
  hidden-end retirement.
- [~] Match timetable bus paths against the complete ordered `.ttr` track-entry
  sequence, including repeated scenery-object IDs and their path selectors, and
  try up to 16 feasible route seeds per endpoint using each bus model's minimum
  turning radius. Exact timetable-selected SCO paths can bridge up to 2.5 m of
  vertical endpoint offset when horizontal position and heading align; the
  route copy is height-aligned without changing ordinary traffic connections.
  Only routes with parsed departures are eligible for scheduled dispatch.
  Constrained route searches reject dead-end routes that fail to complete the
  required timetable sequence instead of treating a short common section as a
  complete trip.
  A bus now selects the earliest departure at or after the configured starting
  hour rather than waiting for the first time in the timetable on the next day.
  When no full route match exists, the best connected endpoint route sharing
  at least two consecutive timetable splines can be dispatched using that
  timetable's departures; startup logs identify this as an itinerary-inaccurate
  fallback. Complete timetable matches are now preferred over partial endpoint
  matches across all candidate endpoints, so an early partial match cannot hide
  a later complete route. Road traffic paths authored inside placed scenery
  objects retain the parent object's map ID and path index, allowing repeated
  timetable `.ttr` entries for junction paths to match the directed lane graph.
  Repeated checkpoints on ordinary map splines are collapsed to one route
  segment; distinct placed-scenery paths retain their individual selectors.
  A live Cotterell run still reports three complete routes and seven partial
  endpoint fallbacks, so unrepresented or disconnected map paths remain.
  A live Cotterell startup recognized complete routes for line 12, U28, and one
  U18 trip; other candidate trips remain explicitly logged as partial endpoint
  fallbacks rather than timetable-faithful. A one-bus live run departed line 12
  from endpoint spline 14089 at minute 483, emitted progress at 100 m intervals
  through 700 m, and reached its open route end after 785.68 m. Full OMSI
  schedule parity and the remaining partial map routes still need validation.
  The current seven fallback candidates match only timetable suffixes, omitting
  121 or 146 earlier checkpoints; startup diagnostics now report the matched
  offset, omitted prefix, and next expected spline/path rather than implying
  those are complete. A bounded authored-origin graph check now reports the
  longest ordered timetable prefix reachable from each unmatched trip's
  starting checkpoint. A live Cotterell startup found one X18 trip matching
  73/205 checkpoints from spline 10433, lane 5, before the graph disconnects
  at spline 10811; this confirms the fallback is not caused only by a suffix
  search. The direct-transition diagnostic found zero road lanes for spline
  10811, and an exact search of the installed Cotterell `.map`, `.sco`, and
  `.ttr` files found that ID only in `Uni_Ctrl.ttr`. The next represented
  checkpoint, spline 10813/path 5, is not reachable directly from spline 10810
  either. This points to an absent map source record and a real graph break,
  rather than a turn-radius limit; do not silently skip it and label the route
  complete.
- [x] Parse `drivers.txt` model paths and `.hum` fields, and distinguish
  type-1 human paths from type-0 road paths.
- [~] Spawn and move human actors on nearby type-1 spline routes using
  passenger-density settings. Configured `.hum` model first-LOD meshes now
  render at actor poses, uniformly scaled to the declared human height while
  preserving mesh proportions; the first `[humangeom]` value is foot distance,
  not body width. `.hum` and model CFG parsing, mesh parsing, and texture
  decoding run asynchronously, while OpenGL upload remains on the render
  thread. Native animation and visual parity remain incomplete. Active human
  footprints are included in road AI's swept-path obstacle set, so vehicles
  yield rather than pass through pedestrians. Pedestrians also stop/resume
  around occupied vehicle footprints, but have no physical collision response
  or configured model animation. Density reductions defer hiding a pedestrian
  until it is off-screen; increases admit preallocated actors only at clear
  off-screen path positions, advancing inactive actors invisibly until a safe
  admission point is available. Startup retries a bounded number of route
  candidates and wraps
  looping spawn distances before open-route endpoints; routes with no usable
  distance after the endpoint margin are rejected instead of stacking actors
  at the route start. The traffic runtime probe exercises repeated in-range AI
  and human placement, fills requested four-actor AI and human populations
  using bounded route candidates, and covers route-specific actor creation,
  movement, endpoint margins, unusably short routes, out-of-radius rejection,
  and vehicle yielding to pedestrian footprints.
- [~] Load and render `.hum`-referenced pedestrian models. Configuration,
  mesh parsing, and texture decoding run asynchronously, with OpenGL upload on
  the render thread; AI driver models and native animation remain unimplemented.
- [~] The `.hum` loader reads all 22 numeric coordinates in `[links]`;
  ambient-human gait phase uses the authored `[walk_param]` step length,
  upper-arm beta angle, arm-swing multiplier, hip-sway setting, and
  waist-animation setting; focused GPU regressions verify each animation
  control. The authored right-shoulder coordinates in `[links]` also set the
  arm pivot, mirrored for the other side, when valid; the right-hip coordinates
  similarly locate the approximate hip-sway pivot, and right-knee coordinates
  set the approximate lower-leg gait pivot. Right-elbow coordinates set an
  approximate forearm flex pivot during walking; right-hand coordinates
  similarly set an approximate wrist pivot for hand flex; neck Y/Z coordinates
  set an approximate head-stabilization pivot; and waist Y/Z coordinates anchor
  the gait-linked torso lean. Focused parser and GPU regressions cover the knee,
  elbow, hand, neck, and waist pivots. These are approximate mesh deformations,
  not native joint animation. Applying other link-driven joint positions, seat
  height, mass, and voice data to human actors remains incomplete; parsing
  alone is not runtime support.
- [~] Real-map benchmark smoke checks load type-1 human paths and spawn the
  configured population on installed Bad Karlstein and Cotterell maps. Bad
  Karlstein exposed 166 human-path splines and queued two actors, with one
  active at its sampled density of 0.68; Cotterell exposed 168 paths and
  activated both queued actors at density 1.0. Spawn logs confirmed placement
  on human splines at clear off-screen positions. These checks do not establish
  the meaning of every path field, continuity at every authored spline join,
  density interpolation parity, or representative in-game visual quality;
  those validations remain open.

- [~] Apply road and passenger traffic-density schedules to their respective
  capped populations using provisional linear interpolation and endpoint
  clamping. Density changes now defer population transitions until off-screen
  positions are available and route footprints are clear. The startup road
  pool reserves each unscheduled group up to its maximum active target across
  configured hours and weekday classes, rather than limiting the total queued
  pool to the simultaneous active population cap. This lets the same fixed,
  group-specific pool meet separate peaks at different times; the active
  population remains capped. Sampled split-peak regressions cover this
  invariant. Weekday schedules derive their starting weekday from the map
  system calendar (`Day`, `Month`, `Year`, and `DayOfYear`); the date and time
  variables advance with elapsed simulation time. Verify native units,
  interpolation, and time-of-day policy before treating the behavior as
  OMSI-compatible.

#### Implemented AI traffic enhancements

These enhancements are implemented and covered by synthetic probes, but do not
constitute full OMSI parity or in-game visual validation:

- [x] Admit road vehicles on clear, off-screen interior positions across
  connected road routes in the active tile window; connected internal lanes
  and circuits are eligible, while isolated splines and terminal dead ends
  are not. Scheduled buses remain endpoint-only. Retain vehicles at visible
  unconnected route ends and recycle them only after those ends are hidden.
  Reevaluate road candidates as tiles become active; vehicles are hidden and
  paused when their current tile becomes inactive.
- [x] Follow directed authored lanes and scenery junction paths, filter turns
  by vehicle turning radius, and apply bend-speed, acceleration, braking, and
  headway limits. Swept footprints make vehicles yield to the player, other AI,
  static vehicles, and pedestrians; any part of a vehicle entering a junction
  commits it to clear under physical swept-footprint checks, with reduced
  lookahead headway until it exits. Junction reservations age waiting movements,
  while committed actors clear before new approaches regardless of authored
  priority. Before entering, a vehicle must claim the junction group
  (scenery junction paths joined by connectors shorter than the vehicle):
  it waits at the entry line while any vehicle inside or already admitted
  would cross or merge into its swept path, so conflicting approaches never
  meet inside the box; same-lane followers share the leader's claim. Claims
  follow queue order: a vehicle cannot claim while an unclaimed leader is
  still between it and the entry, and never waits on a claim held by a
  vehicle queued behind it in its lane. A red
  signal ahead of the entry releases an unused claim. Routine vehicle queue, spawn, unload, and
  junction-entry logs are suppressed; a single stuck diagnostic, naming the
  blocking constraint, the exact blocking vehicle or obstacle, and world
  position, is emitted after 15 seconds without route progress, excluding
  detected red-signal waits and queues behind a bus.
  Following targets only leaders projected onto the vehicle's own path, so
  vehicles in adjacent lanes or on roads straight ahead of a turn are ignored.
  Moving in-lane leaders are handled by the damped following model rather than
  the parked-obstacle lookahead, so followers no longer brake to a crawl behind
  a moving car or stop short of a junction it is still driving through.
  The F2 overlay marks those active stalled vehicles red, excluding inactive
  tiles, route-end holding, and signal waits; deadlocked vehicles are removed
  from active traffic and not recycled into the population. A stopped vehicle
  is only treated as deadlocked when its chain of blockers ends in a cycle or
  an unknown blocker; chains ending at moving traffic, a signal, an AI bus, or
  the player bus are legitimate queues.
- [x] Show the number of currently spawned map AI vehicles in the on-screen
  status HUD.
- [x] Extend the F2 traffic overlay with vehicle and pedestrian behavior
  footprints, signal stop boxes colored by current permission, and sampled
  junction-path reservations; vehicle body boxes require drawable model
  geometry and depth-test against the scene, while signal and reservation
  diagnostics remain visible through objects. Deadlocked AI bodies are red,
  junction actors blue, and ordinary AI bodies green. The overlay log describes
  the color legend.
- [x] Attach timetable departures to complete endpoint-matched bus routes,
  handle midnight rollover, skip missed trips, and partition departures across
  pool buses without assigning a trip twice.
- [x] Parse fixed-cycle SCO traffic lights and attach them to linked paths.
  AI stops on non-green or invalid phases and resumes on states 6-8, including
  signalized route starts and loop-wrap approaches. Conditional stop controls
  use vehicle/pedestrian requests; `[traffic_light_jump]` bus-priority rules
  trigger from player/AI bus approaches, apply their target phase for the
  configured minimum, and wait for conflicting green groups to clear. Cycle
  origin and native stop-line fidelity remain open.
- [x] Move pedestrian actors on type-1 human paths, use parsed `[humangeom]`
  dimensions for proxy sizing and collision footprints, and stop pedestrians
  around occupied vehicle/human footprints. Configured `.hum` first-LOD models
  render asynchronously; a shader-driven walking gait follows actual route
  movement. Model `[smoothskin]`/`[setbone]` records are recognized, but native
  weighted skeletal animation and in-game visual validation remain open.
- [x] Defer population decreases until actors are off-screen and admit
  preallocated actors only at clear off-screen route positions. Density
  interpolation/native unit parity and active-population cap policy remain
  provisional; unscheduled group peak capacity is regression-tested.
- [x] Keep map AI vehicles out of unloaded tiles: admit them only from active
  entry tiles, hide and pause them outside the active tile window, and exclude
  paused vehicles from pedestrian obstacles and signal/junction actor updates.

#### Remaining AI traffic implementation

The entries above describe current behavior and its test coverage. These are
the outstanding implementation and validation tasks:

- [x] Recognize `.ovh` road vehicles in `NormalCar`/`NormalCars`, `Trucks`,
  `Commercial`/`Commercials`, and `Ambulance` AI-list groups. Preserve their
  `unsched_vehgroups.txt` indices and use the configured AI vehicle type (with
  trucks-group fallback) for route access checks.
- [x] Decode and apply group-specific map `[rule]` / `[kill_rule]` values on
  spline-profile and scenery-object paths: km/h speed limits, per-group
  traffic-density factors (zero excludes the group; positive factors weight
  entry and junction choices), authored priorities before wait-age tie-breaking
  among equal priorities,
  `no_cars`, `bus`, `trucks`, and `overtaking_prohib`. Scheduled buses bypass
  unscheduled access/density filters. Overtaking/lane changes are not
  implemented, so vehicles currently never overtake. `unsched_trafficdens.txt`
  group/day curves now drive the capped, time-varying road population target
  and weight startup selection; active group targets are recomputed as curves
  change, with safe off-screen admission and retirement. The startup pool is
  queued even when the starting hour has zero group density, so groups becoming
  active later in the day are not lost. The startup pool uses a rounding-safe
  upper bound for each group's active target across configured hours and
  weekday classes instead of sampling only the aggregate peak composition.
  Its queued pool may exceed the active population cap when group peaks happen
  at different times; only the simultaneously active population is capped.
  Sampled split-peak
  regressions verify reservations across weekday classes, and a targeted
  threshold regression covers an interior peak caused by integer
  apportionment at the active-cap transition. Where wildcard and
  day-specific schedules coexist, the most-specific valid matching curve
  takes precedence; an invalid curve does not suppress the wildcard fallback.
  Weekday masks with unknown bits are rejected with a diagnostic rather than
  silently treated as a schedule that never matches.
  Weekday selection comes from the map system calendar, which starts with the
  host local date and advances with simulation time.
  Native visual density-category parity remains unverified.
  Focused probes cover path and group indexing, parsing/validation, scheduled
  group selection/rebalancing and pool capacity, access, density weighting,
  zero-at-start/future-positive pool scheduling, disjoint peaks above the
  active cap, reservation coverage across sampled hours/weekdays, speed caps,
  priority ordering/fairness, and kill rules.
- [x] **Maintain target population:** recycle preallocated vehicle entries when
  routes end or the target increases, up to the startup pool cap. Re-entry is
  limited to clear, off-screen connected road positions in active tiles;
  scheduled buses remain limited to incoming endpoints and wait for their
  owned departure. Recycled actors reset route progress, speed, and
  junction-wait state.
- [~] **Complete scheduled bus routes:** complete timetable matches are now
  preferred over endpoint-only partial matches. Cotterell startup has queued
  endpoint buses for line 12, U28, and one complete U18 route; other trips
  remain partial fallbacks. Departure selection, midnight rollover, and the
  exact decimal-start-hour boundary are regression-tested; the boundary bug
  that could produce a negative due time is fixed. A one-bus live run confirmed
  a line 12 bus departed endpoint spline 14089 at timetable minute 483 once its
  tile was active, its footprint clear, and the camera rotated away, then
  reached the route end after 785.68 m. Progress diagnostics reported each
  100 m through 700 m. Earlier runs confirmed visible endpoints and inactive
  endpoint tiles correctly hold dispatch. Bounded diagnostics report changes
  in camera, clearance, and active-tile blockers, including camera distance.
  A bounded authored-origin search now identifies the longest connected
  timetable prefix and first disconnected checkpoint. Route search can traverse
  intermediate lanes and now consumes repeated consecutive timetable
  checkpoints consistently with the route matcher. A live Cotterell check
  confirms that this recovers the absent spline 10811 by following represented
  checkpoints in order, improving U18/X18 origin coverage from 73 to 110 of 204
  represented checkpoints. It then reaches a genuine lane-graph break between
  spline 14280 and represented spline 14292; neither an intermediary bridge nor
  unconstrained routes connect that pair. The nearest directed lane endpoints
  are 23.99 m apart horizontally and 0.92 m vertically, with heading alignment
  -1.0, so this is not a join-tolerance defect. Startup still finds 3 exact
  routes and 7 endpoint-only partial fallbacks. Do not classify those fallbacks
  as timetable-faithful or synthesize the disconnected transition; resolve it
  against valid map lane data before claiming full timetable coverage. When the
  configured bus-pool limit is smaller than the number of matched timetable
  routes, current initialization assigns one bus to routes in parser order
  until the pool is exhausted, so later routes receive no bus. With spare pool
  capacity, additional buses are allocated toward routes with the greatest
  departure count per assigned bus, then each route's departures are
  round-robin partitioned among its buses without duplication. Focused runtime
  probes cover undersized pools, departure-load balancing, routes without
  departures, and unique departure ownership. Validate an intended
  service-priority policy for undersized pools before changing which routes
  receive the initial buses.
- [ ] **Complete traffic control:** verify SCO signal cycle origin, approach
  distance and stop-line placement; implement traffic-sign rules.
- [~] **Improve junction control:** committed junction-path actors clear before
  new approaches; otherwise authored path/group priorities precede wait age,
  with wait aging fairly ordering equal-priority movements. Before claiming a
  junction, a vehicle on a lower-priority approach spline gives way to
  higher-priority traffic only when predicted arrival times overlap at a
  crossing or merge, and releases an unused claim if such traffic appears;
  priority traffic ignores that unused claim. Deadlock exemptions follow
  actual blocker chains instead of bus proximity. Synthetic
  regressions cover separation, progress, junction clearing, priority-over-
  wait, main-road priority at T-junctions, unobstructed turn transitions, and
  approach stop distance. Validate native right-of-way behavior and deadlock
  recovery at representative live junctions.
- [ ] **Add advanced vehicle behavior:** implement safe overtaking and lane
  changes, then extend swept-footprint handling to articulated vehicles.
- [~] **Render configured human models:** pedestrians now render the configured
  `.hum` first-LOD meshes instead of primitive proxies. `.hum`/model CFG
  parsing, mesh parsing, and texture decoding are asynchronous; OpenGL upload
  remains on the render thread. AI driver model rendering and configured
  skeletal/limb animation remain open.
- [ ] **Use remaining `.hum` behavior:** validate and apply links, walk
  parameters, seat height, mass, and voice; verify human-path field meanings,
  route continuity, density behavior, and placement on representative maps.
- [ ] **Define contact policy:** reconcile physical player-to-AI collision
  response with the requirement that AI and human traffic remain independent of
  the main bus's state; preserve avoidance without state mutation until that
  behavior is explicitly resolved.
- [ ] **Validate end-to-end behavior:** exercise live maps and in-game visuals
  for spawning, junctions, signals, pedestrian interactions, population
  transitions, and scheduled dispatch. Synthetic probes alone do not establish
  native OMSI parity.

- [ ] Implement or explicitly scope out the remaining runtime effects of retained
  `global.cfg` settings: `[description]`, `[NextIDCode]`,
  `[worldcoordinates]`, `[dynhelperactive]`, `[realrail]`, `[LHT]`,
  `[backgroundimage]`, `[mapcam]`, `[moneysystem]`, `[ticketpack]`,
  `[repair_time_min]`, `[years]`, `[realyearoffset]`, `[standarddepot]`, `[addseason]`,
  `[trafficdensity_road]`, and `[trafficdensity_passenger]` beyond the
  provisional population multiplier above.
- [~] Spline `skewStart`/`skewEnd` fields now drive a provisional longitudinal
  shear in rendered/collision road geometry. Verify the assumed units and
  formula against OMSI before treating it as compatible. `.sli`
  `[halfcantwidth]` now overrides the fallback width derived from valid
  `[heightprofile]` lateral extents for cant clamping; native visual parity and
  full authored height-profile behavior remain unresolved. The Cotterell log
  reports 951 profile strips with fewer than two points (including `pavement_a.sli`);
  verify whether these are intentional degenerate/editor strips or a
  profile-reader/geometry omission.
- [~] Provisional `[water]` tile quads are decoded, validated, rendered, and
  covered by `OpenBusMapWaterGeometryProbe`; OMSI parity and water collision
  remain incomplete. `[variable_terrain]` and `[variable_terrainlightmap]`,
  `[rule]`, `[kill_rule]`, and `[spline_terrain_align_2]` remain unsupported.
  Decode/render `.map.LM.bmp` and `.map.terrain_0.rdy` adjunct data where
  applicable. The Cotterell run skipped 27 variable-terrain tiles, 27
  variable-terrain-lightmap tiles, 2,212 `[rule]` records, and 1 `[kill_rule]`.
- [ ] Resolve Chrono map variants using `[selobject]`, `[selspline]`,
  `[delete]`, `[typ]`, and `[relabel]`; these records are currently parser-only
  or have no independent runtime effect.
- [x] Support Cotterell's intended `.map.8.dds` ground-texture layers: when a
  sidecar suffix equals the number of `[groundtex]` records, map it to the final
  record. Existing suffixes 1..N-1 retain their direct mapping; format semantics
  remain reverse-engineered rather than normative.
- [ ] Investigate why Cotterell visited 27 terrain tiles but uploaded only 26
  terrain meshes, despite the manifest reporting 27 terrain sidecars; identify
  the missing/rejected tile and ensure it is surfaced in diagnostics.
- [ ] Investigate the 7 spline attachments with no local placement in the
  Cotterell run (examples reference `splineIndex=154`, `102`, and `5`); determine
  whether they are unresolved repeater-master references, invalid local spline
  records, ambiguous IDs, or bad source placements. Unique reciprocal spline
  chains now resolve across tile boundaries for ordinary attachment rows.
- [~] Nonzero-LOD scenery parts now render using the vehicle renderer's existing
  threshold-to-distance mapping. Validate distance transitions and visual
  parity with representative installed scenery; 9 parts were skipped in the
  Cotterell run, including Streetobjects_RUE light models.
- [x] Recognize SCO `[groups]`, `[friendlyname]`, and `[helparrow]` as
  metadata/editor fields rather than malformed fixed-width model records. A
  parser regression covers the route-arrow-style one-line friendly name and
  group list. Route-arrow `text.bmp` textures were among 27 missing scenery
  textures; verify whether these should be generated by the text-texture path.
- [~] An empty-name/empty-path texture request is now treated as an intentionally
  untextured batch instead of a decode failure, and decode failures include the
  source mesh path. The parked Opel Manta DDS remains unverified because its
  model asset is not present in this checkout.

- [x] Add curved centerline tessellation for the recognized Freyfurt asphalt
  profiles. Signed-radius geometry is checked by `OpenBusMapSplineGeometryProbe`
  against linked Grande Porto spline endpoints in both turn directions; a
  zero-radius straight case and invalid inputs are covered. This verifies the
  centerline only, not cross-section sidewalks, markings, seams, or lighting.
- [ ] Add targeted, repeatable in-context screenshot evidence for curved-road
  geometry, sidewalks, markings, texture phase, and seams at linked spline
  boundaries.
- [~] Load static text, binary, and MSZIP-compressed scenery `.x` meshes with
  frame transforms, triangulation, normals, UVs, and inline materials.
  Skinning/animation and named external materials remain unsupported. Tree SCO
  previews still use crossed textured planes rather than their editor/helper
  mesh.
- [~] Read road cross-sections from `.sli` files (`[texture]`, `[profile]`,
  `[profilepnt]`), retain the four raw/numeric fields of `[heightprofile]`, and
  retain `[nocollision]` to exclude marked profiles from road collision while
  preserving their render geometry; native OMSI profile semantics are still
  unverified. Apply explicit `[halfcantwidth]` to cant clamping as well. Spline
  cant is provisionally interpolated as percentage crossfall in shared
  render/collision geometry;
  precedence and sign/unit parity need OMSI visual confirmation. Full authored
  height-profile, texture/material, and marking behavior and representative
  visual evidence remain incomplete. Road UV V includes a valid parsed v11+
  spline chain offset; continuity and OMSI
  parity still need validation against installed linked spline assets.
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
  regression evidence against installed map assets. Finish cross-tile chains
  and repeater-master resolution. Terrain-normal sampling/alignment and
  `[attachObj]` composition have deterministic placement probes; verify both
  visually against OMSI. `[attachObj]` label counts remain opaque and its
  optional tail is preserved independently of the count.
- [~] Stream ODE trimeshes for supported `.sli` strips and authored SCO/model
  `[collision_mesh]` O3Ds, including junctions, with terrain in a nearby tile
  neighborhood. Builder and simulation probes cover selected-tile construction,
  O3D placement/winding, `[nocollision]`, and add/remove resource ownership;
  validate contact while crossing representative installed tile seams and tune
  duplicate contacts where road strips overlap terrain. `[boundingbox]`-only
  scenery and non-O3D collision formats remain unsupported.
- [x] Bind per-placement `[usetexttexture]` and `[usescripttexture]` scenery
  materials to independent live runtime surfaces; generated text glyphs blend
  over transparent backgrounds. Runtime probes cover isolated text state, but
  visual validation against installed directional-sign assets remains.
- [~] Extend the per-placement scenery `init`/`frame` and `[visible]` path with
  script-driven animations, dynamic material changes, sound dispatch, and
  mouse events after those runtime semantics are verified. `[varparent]`
  script-variable inheritance remains distinct and unsupported.
- [~] Decode `.map.water` sidecars into per-surface four-height records;
  determine corner/quadrant ordering and implement water rendering. Per-tile/
  detail/seasonal ground textures, map lightmaps, and Chrono variant resolution
  remain. Chrono selector IDs and opaque override payload lines are parser-only.
- [~] Terrain, road, and scenery geometry is constructed/uploaded only for the
  two-tile bus-centered window; a shared-GL-context worker builds/uploads a
  replacement window while the old one remains drawable. Scenery script startup
  is deferred to the main thread (at most 32 new runtimes per frame), and state
  persists across windows. Collision mesh CPU preparation runs in a worker for
  the one-tile neighborhood constrained to successfully uploaded render geometry;
  ODE publication/removal remains on the simulation thread, one tile per update.
  Initial map construction and collision warm-up are synchronous, and renderer
  loading falls back to synchronous if shared-context creation fails. The global
  parsed tile/placement index remains CPU-resident. Implement per-tile rather
  than full-window rebuilds, measure ODE publication and boundary-frame latency,
  and validate in-game visual/contact continuity before claiming hitch-free
  streaming.
- [x] Select map entrypoints at startup by zero-based index or case-insensitive
  name through `OPENBUS_MAP_ENTRY`; the default remains the first entrypoint.
  Selection and invalid-index behavior are covered by `OpenBusMapConfigProbe`.
- [x] List map entrypoints with zero-based indexes and their resolved world
  coordinates/yaw when `OPENBUS_MAP_SPAWN` is unset; select and spawn by index
  when it is set. `OPENBUS_MAP_ENTRY` remains a compatibility fallback.
- [ ] Complete HOF-backed map state and validate remaining timetable/traffic
  control semantics. Scheduled AI dispatch uses parsed departures only when a
  route matches an endpoint. Linked SCO fixed-cycle signals are enforced from
  `[phase]` states; request-driven jump/stop timing and native cycle-origin
  semantics remain unverified. The inspected Grande Porto signal-route file is
  only a header/template.

- [x] OpenGL 3.3 renderer, VBO model path, cameras, mirrors, frustum culling,
  LOD selection, texture cache, crash reports, tracing, and profiling hooks
  are present.
- [x] Load deferred vehicle LODs from the camera eye (matching draw-time LOD
  selection), prefetch other LODs of the current view, and always load parts
  outside any `[LOD]` section. AI cars far from the bus but near the camera
  previously loaded only their far LOD and drew nothing while still blocking
  traffic; covered by `OpenBusVehiclePartLoadingProbe`.
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
- [x] Remove unused compatibility-era texture-environment enum fallback
  definitions from `src/render/GlCompat.h` after confirming no source or
  generated code uses them; active core-GL constants remain available.
- [ ] Validate Windows and Linux helper scripts from clean build directories.
- [~] Focused clang-format checks pass for changed ranges, and clang-tidy runs
  using the exported Windows compile database. The repository-wide checks still
  report existing formatting/lint warnings outside the touched ranges; Linux
  validation was not available in this Windows environment.

Done when performance options have documented tradeoffs and both primary build
platforms have a reproducible build-and-smoke procedure.

## 7. Final release gate

- [x] Run all configured probes, physics stress tests, parser/configuration
  tests, and material/display tests from a fresh Windows Clang release build;
  all 43 CTest tests pass.
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