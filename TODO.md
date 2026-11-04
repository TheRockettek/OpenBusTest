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

## 1. Lock down physics and wheel stability

- [~] High-speed bump stability: local-axis damping, softened contact response,
  and steering-rate damping are implemented. A deterministic stress probe now
  exercises bump traversal, steering reversals, braking, and fast wheel contact
  with two- and three-axle configurations; broader solver/contact tuning
  remains.
- [~] The wheel/static contact defaults use the more compliant settings, and
  the deterministic stress probe passes for both axle configurations (maximum
  chassis angular speed 0.297 rad/s; suspension travel 0.268 m or less). The
  probe now repeats each axle scenario and checks that all reported metrics
  match within 1e-9. Further solver tuning is blocked on a validated
  vehicle/reference envelope; the probe alone does not establish native OMSI
  parity. Do not accept visual smoothness alone as proof of stability.
- [o] Articulated configuration remains parseable for inspection, but
  articulated vehicles are outside the currently supported scope; runtime
  physics, collision, and rendering treat vehicles as single-body.

Done when the physics probe is repeatable and reports numerical bounds, and
both supported vehicles pass it without wheel shake or explosive motion.

## 2. Complete the live wheel contract

- [~] ODE wheel translation and orientation were corrected and smoke-tested.
  Static default/ODE app captures of MAN DL05 and Studio Polygon 400MMC show no
  obvious wheel-to-arch mismatch. `OpenBusCameraMathProbe` now asserts that the
  ODE wheel-render transform places each mesh center on its wheel mount under
  four chassis orientations; authored tire-to-arch clearance remains
  asset-specific and unverified.
- [~] Deterministic two-/three-axle pose bounds cover dynamic wheel placement,
  and the stress probe now checks dynamic simulated wheel poses through the
  render transform for both axle layouts. Static default/ODE captures run from
  isolated working directories, and `OPENBUS_CAPTURE_DIR` can direct captures
  to distinct output directories. Model-specific steering, bump, and motion
  comparisons remain unverified.

Done when MAN DL05 and E400 wheel poses pass stationary, steering, bump, and
motion checks in both supported rendering modes.

## 3. Finish material and texture behavior

- [~] Environment maps are wired through texture loading and a reflection
  shader; a GPU probe verifies opposite reflection directions and zero,
  partial, and full opacity with a generated map. Representative assets and
  in-game visual parity remain.
- [~] Transparent batches sort back-to-front with render type as a tie-breaker,
  and transmap triangles sort back-to-front within each batch. Probes cover
  overlapping alpha/no-Z-write behavior, both ordering rules, and separate
  reflection triangle accounting. A generated two-layer case checks sorted
  alpha blending in the reflection framebuffer; asset-level glass/decal
  overlaps and in-game mirror parity remain to validate.
- [~] Bump-map rendering now has GPU regression coverage for generated-map
  opposite-direction shading, zero strength, and strength clamping.
  Representative OMSI visual parity and parallax semantics still need
  validation.
- [~] Parser-built material states verify `matl_change` item selection, and a
  focused runtime-helper probe covers case-insensitive CTC replacement and
  texture-array layer clamping. A generated hidden-GL vehicle fixture switches
  item-specific lightmaps through two controller values, verifies distinct
  decoded/uploaded textures and restoration of the base material.
  Native independent primary-texture replacement semantics and representative
  MAN DL05/E400 asset parity remain unverified.
- [~] A GPU regression verifies that a missing auxiliary transmap binds
  texture zero after a prior material used a transmap. An asset-request probe
  covers delayed decode completion, terminal decode failure, and a later valid
  request. A hidden-GL vehicle probe injects DXT5 upload failure and verifies
  the decoded fallback renders after retrying the same cache entry.
- [~] Automated probes cover PNG RGBA, 32-bit alpha TGA, indexed/RLE TGA,
  DXT1/DXT4/DXT5 DDS, transmap alpha, lightmap, and free-texture rendering.
  JPEG decoding is covered by a compact grayscale fixture, and all five
  texture address modes are checked from CFG parsing through GL enum mapping.
  Inline SCO address directives are recognized by both scenery and model
  loaders; GPU coverage verifies border sampling on both axes and wrap-state
  changes on a shared texture. Representative scenery parity remains unverified.
  A GPU material probe verifies nightmap overlays, black-texel neutrality, and
  zero-strength behavior, plus directional environment-map sampling and blend
  opacity with a generated map.
- [~] Scenery material UV translation uses per-placement script variables for
  both axes. Generated rendering coverage exercises independent placements;
  representative OMSI visual parity remains unverified.
- [~] Scenery `[matl_lightmap]` and `[alphascale]` apply per-placement
  controllers. Base and selected item lightmaps have generated rendering
  coverage for interpolation, missing textures, and clamping. Other material
  item modifiers and native visual parity remain unverified.
- [~] Scenery `[matl_bumpmap]` uses the vehicle normal-map shader, with
  generated coverage for strength, map directions, and unavailable textures.
  Native bump/parallax behavior and representative visual parity remain
  unverified.
- [~] Scenery `[matl_envmap]` applies alpha-aware spherical reflections using
  the vehicle opacity factor. Generated coverage exercises both culling paths,
  strength clamping, and unavailable textures; dynamic material items and
  native visual parity remain unverified.
- [~] Scenery `[matl_nightmap]` supports base and script-selected item maps,
  with generated coverage for independent placements, visibility gates, and
  missing textures. Automatic `[NightMapMode]` scheduling and item modifiers
  other than nightmaps and lightmaps remain unsupported; native visual parity
  remains unverified.

Done when every supported material keyword has a documented runtime semantic,
representative assets exercise it, and screenshots/logs or focused tests prove
it.

## 4. Implement dynamic displays and lighting

- [~] Implemented the basic `[illumination_interior]`/`[interiorlight]`
  runtime path: assigned mesh groups receive controller-driven point-light
  shading. A vehicle-level probe verifies live controller updates and reset;
  GPU coverage checks color isolation, zero strength, and distance attenuation.
  `[light_enh]` now renders a camera-facing, alpha-masked bloom with authored
  color/size, controller multiplier, sunlight attenuation, and exponential
  time-constant smoothing. A hidden-GL vehicle probe verifies mask-channel
  handling and controller timing. `[light_enh_2]` renders directional and
  omnidirectional billboards with camera alignment modes and cone-angle
  brightness. A hidden-GL probe verifies authored color/mask and cone rejection.
  Starburst/fog/effect-only behavior, fog scattering, `[spotlight]` rendering,
  and OMSI intensity/sunlight parity remain incomplete. Parsed `[spotlight]`
  records now warn that their payload is retained but has no runtime renderer.
- [~] A focused E400 display/light regression remains blocked on the external
  E400 model CFG/assets, which are not included in this checkout; parser and
  runtime probes do not replace the requested visual comparison.
- [~] In-game lighting flicker has been reported, but the bundled fixtures do
  not reproduce it and the affected light source (interior, nightmap, exterior,
  or scenery) is not identified. Diagnose against the installed model/map
  assets before changing light-controller timing or shader behavior.

Done when a configured display changes from script input and interior lights
change from their controlling variables without manual texture replacement.

### E400 OMSI/OpenBus comparison gaps

  of consuming them only for parser alignment; verify the dashboard LCD,
  odometer, and center display against the OMSI reference capture.
  substitution. The active E400 configuration uses these records for dashboard
  and body variants, so retaining their payloads is not sufficient for visual
  parity.
  assigned meshes; `[light_enh_2]` now renders a base bloom, while `[spotlight]`
  remains storage-only with an explicit unsupported-runtime warning.
  E400-specific behavior and dashboard-lighting parity gaps still require the
  external model assets.
  semantics, including strength and lighting interaction, then compare the
  dashboard binnacle and cab surfaces again.
  OMSI fonts, layout, and display-background/alpha semantics. Enhanced
  horizontal alignments 0–5, authored grid spacing, and
  `Refresh_Strings`-gated changed-string
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

- [~] `[mouseevent]` identifiers are retained on model parts, and configured
  vehicle key bindings are dispatched to the player script runtime as key
  state and optional key-specific entry points. Exact transformed triangle
  picking is active, including hand-cursor hover feedback, captured
  `trigger_<event>` /
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
  are available; the spawn UI now preserves automatic plates when fleet-number
  lists exist, while persistence and full OMSI selection UI remain incomplete.
- [~] Bus-model `[collision_mesh]` records now build an ODE triangle collider;
  model `[boundingbox]` overrides the BUS box, `[nocollision]` filters a
  mesh-scoped collider, and the BUS box remains the fallback. Scenery
  collision also uses authored meshes, model geometry, or bounding-box
  fallback; installed-asset contact parity remains unverified.
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
  reserved boarder keeps its original queue slot while other buses board, and
  departures or service cancellation update the current shared count rather
  than restoring a stale per-bus snapshot. Returned riders retain their model
  identity in the shared queue, including when another bus has reserved a
  passenger, and keep it through reboarding and alighting. A
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
  requires a loadable bus configuration and a complete routable route.
  Station-only timetables retain their authored line and destination, and
  ordinary spline stops no longer require a scenery-path index. Many trips
  remain unavailable when AI road-route construction cannot connect their
  checkpoints; player timetable registration must be decoupled from that
  requirement without fabricating stop positions. The timetable panel caches
  HOF matching and course/trip choices until the player, map, or registered
  routes change. Catalog diagnostics report HOF data, rejection counts, and
  samples of unavailable routes rather than repeating matching every frame.
  Passenger mass,
  passenger ticket purchase, fare
  transactions, and physical/audible coin output remain incomplete. Map
  ticket-pack/currency records now feed ticket and
  change-coin script macros for player and AI buses.

  Passenger journey gaps to complete and validate:
  - [ ] Give waiting passengers an expected service and destination so a
    shared stop queue only boards an appropriate bus instead of assigning
    every rider to its next stop.
  - [ ] Have waiting passengers approach a nearby expected bus while it is
    moving without entering the road, and permit a safe road crossing only
    once it has stopped. Replace direct interpolation with road-aware walking
    for player and scheduled AI buses.
  - [~] Form a physical queue outside the entry and maintain single-passenger
    boarding after the bus stops and the selected door opens. Validate queue
    ordering and door gating for both player and scheduled AI buses.
  - [ ] When ticket selling is enabled, implement the passenger transaction:
    payment, change when owed, waiting for a ticket, and use of an applicable
    ticket machine. Parsed locations and script accounting alone are not a
    completed transaction.
  - [~] Validate walking to the assigned seat and sitting in game, including
    disconnected cabin-path fallback. Replace the fixed 0.75-second traversal
    with movement appropriate to the path length and seating transition.
  - [ ] Select an alighting stop within 200 m of each passenger's destination
    and trigger the appropriate stop-request and bus-stop script behavior at
    the relevant point in the journey; the current request at boarding and
    next-stop assignment do not provide destination-based behavior.
  - [~] Have alighters move to their predetermined exit and queue behind one
    another, including while waiting for the door to open. Support multiple
    alighters progressing concurrently while preserving queue order and the
    existing single-boarder constraint.
  - [ ] After leaving the bus, transfer each alighter to a nearby passenger
    spline leading toward their destination; if no suitable spline can be
    found, remove the passenger.
  - [ ] Add focused regression coverage and an in-game end-to-end validation
    for the journey, including moving-bus road safety, ticketing, the 200 m
    destination rule, concurrent alighting, and spline handoff.

- [~] The on-foot panel now selects a mapped line, ordered `.ttl` course,
  course trip, and first passenger-service stop. Player service follows that
  route and advances in `.ttl` order at the terminus; it stops at
  course end or when the player chooses to stop after the current trip.
  Disabling "Drive as an additional bus" targets one matching AI departure
  (route, departure time, and profile): a pending departure is skipped once,
  or an active bus retires at its next mapped stop; a bus without another
  mapped stop finishes its route. Indistinguishable duplicate
  departures cannot yet be matched to a particular course. Course parsing,
  trip progression, takeover-policy lifecycle, and stop-retirement handshakes
  have focused probes. A BRT Berlin UI smoke test covered selection, spawning,
  additional-bus mode, and the stop request; driving to the terminus remains
  unverified end to end.
- [~] The spawn UI supports fleet number/registration selection and free-form
  overrides; the player’s choices persist in `OpenBus_vehicle_preferences.txt`
  in the launch working directory. The player odometer is restored per bus
  identity and checkpointed atomically every 30 seconds and at shutdown in
  `OpenBus_odometer_preferences.txt`; at most the interval since the last
  checkpoint can be lost on abrupt process termination.
  AI/network sections, remaining route/timetable/HOF behavior, and
  native schedule/ticket panel behavior still need implementation or explicit
  scoping; configured driver-camera selection is now available.
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
  rendered. Basic `[passengercabin]` seat positions, illumination groups,
  entries/exits, and
  path links are loaded and covered by `OpenBusVehicleConfigProbe`. When a
  connected graph exists, pathfinding follows one-way and bidirectional cabin
  links, and timetable offsets position stops within their mapped spline
  segments; detailed native door/aisle semantics and passenger mass remain
  unverified.
- HOF-backed route/path selection, timetable macros beyond the map player-trip
  workflow, stop announcements, and arrival-board data used by the script
  system. Basic `[paths]` point/link graphs and step metadata are loaded and
  covered by `OpenBusVehicleConfigProbe`.
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
  and `[light_enh_2]` have partial light-emission paths; their OMSI parity and
  remaining flags/fog semantics are incomplete. `[spotlight]` has no runtime
  and emits an explicit retained-but-unsupported diagnostic.
- Complete OMSI input configuration and action semantics. Vehicle bindings
  can inherit `[vehicles]`/`[entry]` assignments from `Inputs/keyboard.cfg`,
  with OpenBus defaults for unsupported or absent assignments.
- Script macro HOF route, terminus, bus-stop, and global-string lookups are
  backed by the selected vehicle depot file. Ticket data comes from the active
  map ticket pack, vehicle seat counts from the passenger service, and timetable
  data from scheduled map AI buses. `GetHeightAbovePoint` raycasts through the
  live static collision world. Remaining providers are scenery bus-stop arrival
  queues; those macros return safe results and report unavailable provider data.
  `GetHumanCountOnPathLink` reports occupancy for the active simulated
  boarding/alighting passenger on authored vehicle-cabin path links. The
  simulation updates occupancy independently of rendering; map/scenery
  path-link occupancy remains unsupported.
- System trigger dispatch now reports chassis contacts and scheduled AI target
  and bus-stop changes. Passenger stop-request, rail-joint, repair, and
  refueling event sources remain unsupported.
- Predefined local variables now expose live ODE wheel/brake data, odometer and
  passenger counts, weather, collision details, and scheduled-map-bus values.
  Environmental lighting, surface IDs, fuel, door requests, train state, and
  other listed variables without a host simulation remain defaults or
  script-owned.

Done when unsupported configuration is visible in diagnostics and the README
claims only behavior that is active in the runtime.

### Documented keyword gap register

The reference documents cover more OMSI dialects than the current
vehicle-centric runtime consumes. The following records are documented but
are either unknown, parser-only, or missing their runtime behavior. Keep these
items separate from the active E400 completion work above so parsing a record
for alignment is not mistaken for supporting it.

#### BUS vehicle configuration

- [~] BUS-level `[fixed]` and `[scriptshare]` are explicitly diagnosed as
  unsupported; their runtime semantics are not implemented.
- [~] `[view_schedule]` and `[view_ticketselling]` entries retain their
  non-negative view indexes and labels, including repeated entries. The
  player timetable UI now offers selectors that jump to the associated driver
  cameras; native schedule/ticket panel behavior remains unimplemented.
- [~] `[cog]` coordinates are retained and its positive z value supplies the
  physics CG height when `[schwerpunkt]` is absent. `[schwerpunkt]` takes
  precedence regardless of record order, with a warning if heights disagree;
  nonzero horizontal offsets are explicitly reported but are not applied.
  Native precedence and horizontal CG physics remain unverified/incomplete.
- [~] `[rot_pnt_long]` is retained and diagnosed as not applied; implement
  its longitudinal rotation-point physics semantics.
- [~] Vehicle-family `[rowdy_factor]`, `[boogies]`, `[ai_brakeperformance]`,
  `[ai_veh_type]`, and `[sinus]` are explicitly diagnosed as unsupported;
  their family-specific behavior is not implemented.
- [~] `[coupling_front]` and `[coupling_back]` XYZ points are retained with
  diagnostics, and `[coupling_front_character]` angle limits/type are retained
  with validation. `[couple_back]` referenced vehicle path and boolean flag are
  retained; `[couple_front_open_for_sound]` remains explicitly unsupported.
  Articulated vehicle behavior is not implemented.
- [ ] Implement `[control_cable_front]` and `[control_cable_back]` for
  electrical/control-variable transfer between coupled vehicles. Both are
  diagnosed as unsupported.
- [ ] Implement `[new_attachment]` attachment transforms and the rail/trolley
  records `[contact_shoe]` and `[rail_body_osc]`, or explicitly scope them out
  with diagnostics. Vehicle-level records now report unsupported behavior.
- [~] `[registration_free]` supports editable vehicle-number and registration
  overrides in the spawn UI, including buses with configured fleet entries.
  Automatic registrations are no longer overridden by the selected fleet-list
  plate. Selections and odometer state persist; odometer writes use atomic
  replacement and periodic checkpoints. Native compatibility and testing
  against installed vehicle files remain outstanding as tracked in the E400
  comparison section.

#### Bus-model configuration

- [~] Complete runtime material semantics for `[matl_bumpmap]`,
  `[matl_envmap]`, `[matl_freetex]`, `[matl_lightmap]`, and `[matl_nightmap]`;
  one or more shader, blending, controller, or texture-binding stages remain
  incomplete. `[matl_transmap]` alpha sampling and its interaction with
  blended depth behavior, including `[matl_noZwrite]`, have regression
  coverage; representative
  OMSI visual parity remains unverified.
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
  `[nocollision]` now affect the bus ODE collider. Scenery collision has a
  separate supported mesh/box path; shadow grounding against elevated road
  surfaces remains incomplete.
- [~] Basic `[illumination_interior]`/`[interiorlight]` controller-driven
  point-light shading, mesh assignments, and distance attenuation are active;
  their retained records are serialized in model-config JSON. A GPU probe
  covers color, zero strength, and near/far attenuation. `[light_enh]` and
  `[light_enh_2]` render base alpha-masked bloom effects and preserve their
  numeric payloads/mesh assignments in JSON. `[spotlight]` retains its twelve
  values and emits an explicit unsupported-runtime warning. OMSI parity,
  enhanced-light flags/fog, and spotlight rendering remain incomplete.
- [~] `[origin_from_mesh]` uses source transform metadata when present and the
  parsed mesh bounds center as its pivot fallback; native pivot parity remains
  unverified.
- [~] `[smoke]` preserves its nineteen authored fields in model-config JSON;
  exhaust/particle emission remains unimplemented.
- [~] `[tex_detail_factor]` is retained as a finite model-config value and
  included in model-config JSON; detail-texture rendering remains unimplemented.
  `[VFDmaxmin]` bounds are validated and its optional view-context flag is
  retained verbatim; VFD display-region behavior remains unimplemented.
- [~] `[CTCTexture]` replacements are applied case-insensitively to base mesh
  texture names and the resolver is unit-tested; retained `[CTC]` template and
  `[CTCTexture]` mapping metadata is serialized in model-config JSON.
  `[CTC]` paint-scheme selection and representative-asset substitution remain
  unsupported.
- [~] `[texchanges]`, `[newtexchangemaster]`, and `[entries]` payloads are
  retained in source order and serialized in model-config JSON. Count
  validation and runtime replacement behavior remain unimplemented.
- [~] `[terrainmapping]` variable-length payloads are retained verbatim and
  serialized in model-config JSON; terrain/material rendering remains
  unsupported.
- [ ] Finish OMSI-compatible `[texttexture]` and `[texttexture_enh]` font,
  vertical/layout, and alpha semantics; horizontal alignments 0–5 and authored
  grid spacing are supported, but the fallback font rasterizer and remaining
  enhanced-format flags are still not fully compatible. Font-directory indexes
  now refresh on directory timestamp changes and do not cache missing
  directories; regression coverage includes added and removed definitions and
  case-distinct paths. The index depends on filesystem timestamp resolution and
  does not detect in-place changes to OFT font names; already-created script
  runtimes also retain their font results.

#### General CFG dialects

- [ ] Add a surface/material CFG path for `[puddles]`, `[moisture]`,
  texture `[surface]`, and `[NightMapMode]`, including wetness and
  surface-type data. Scenery `[surface]` and `[nocollision]` affect supported
  collision geometry, but do not implement texture surface/material physics;
  SCO `[NightMapMode]` is parsed and retained but not applied. `[maplight]`
  is consumed without generating tile lightmaps.
- [~] Passenger-cabin `[noticketsale]`, `[ticket_sale]`,
  `[ticket_sale_money_point_2]`, and `[ticket_sale_change_point_2]` locations
  and parameters are retained with unsupported-behavior diagnostics. Ticket
  sales eligibility, machine interaction, and money/change handling remain
  unimplemented.
- [~] Scenery SCO `[script]`, `[varnamelist]`, `[stringvarnamelist]`, and
  `[constfile]` resources now feed per-placement `init`/`frame` runtimes, and
  scenery `[visible]` controllers are checked during map drawing. The inline
  SCO model parser leaves these resource records and `[model]` to the SCO
  parser instead of reporting them as unknown model directives; external model
  CFG diagnostics are unchanged. Script-driven
  animations, dynamic materials/textures, mouse-event dispatch, sound callbacks,
  and general `[varparent]` state inheritance remain incomplete. `[varparent]`
  is used to associate `[trafficlight]` signal heads with a parent controller's
  phase schedule; this does not provide general scenery-variable inheritance.
- [~] `[stamper]` index and xyz position, and `[linkToNextVeh]`/
  `[linkToPrevVeh]` passenger-path point indexes, are retained with diagnostics;
  ticket-stamping interaction and linked-cabin traversal across coupled
  vehicles remain unimplemented. Basic `[drivpos]`, `[passpos]`,
  `[entry]`, `[exit]`, and illumination records support stop-service seating
  and boarding animation, and connected authored cabin path links are used for
  passenger movement. Ticket interaction remains unimplemented.
- [~] Load `[pathlink_oneway]` and preserve its directed-link flag alongside
  ordinary `[pathlink]` records; `OpenBusVehicleConfigProbe` distinguishes both
  forms. Passenger movement follows connected directed cabin paths; extended
  `[pathpnt]` records and `[stepsoundpack]` runtime use remain incomplete.
- [~] Player sound CFG runs untriggered `[sound]` records with `[volcurve]`
  controls as ambient loops and applies their configured base gain. This
  enables authored window-dependent ambience (for example, `cabwindowopen`)
  without window-specific mixer logic. Triggered sounds evaluate and multiply
  their named variable curves during playback for player and AI vehicles;
  special-curve semantics remain incomplete.
- [ ] Complete sound spatialization and control semantics for `[loopsound]`
  and `[3d]` beyond the current player sound loading/trigger path. Ambient
  definitions sharing a file retain independent positions and controls.
- [ ] Add AI/network configuration support for `[ailist]`, `[aigroup_2]`,
  `[aigroup_depot]`, `[busstop]`, `[signalroute]`, and `[StnLink]`.
- [ ] Add environment configuration support for `[sky_textures]`,
  `[cloudtype]`, and `[startdate]`.
- [~] Map-referenced `[currency]` and `[coin]` records provide coin values to
  change-coin script macros; coin rendering and passenger transactions remain
  incomplete.
- [~] Vehicle keyboard `[entry]` assignments are inherited from the
  `[vehicles]` section of `Inputs/keyboard.cfg` and exercised by
  `OpenBusVehicleInputBindingsProbe`; missing/unsupported bindings retain
  defaults. `[game]`, `[ctrl]`, `[axis]`, other input contexts, and complete
  native input behavior remain unsupported or unverified.
- [ ] Implement observed configuration records that remain unhandled in their
  current contexts: SCO `[carpark_p]`, `[crossingproblem]`,
  `[entrypoint]`, `[joinable]`, `[mass]`,
  `[momentofintertia]`, `[newanim]`,
  `[nomaplighting]`, `[shadow]` per-mesh cast-shadow behavior,
  `[splinehelper]`, and `[terrainhole]`; model-config
  `[crashmode_pole]`, `[detail_factor]`, `[LightMapMapping]`,
  `[traffic_light]`, and `[traffic_lights_group]`; and spline-profile
  `[length]`, `[matl]`, `[nightmapmode]`, and `[rail_enh]`. Some names have
  implementations in other contexts (for example, `[traffic_light]` data in
  SCO files and `[matl]` model materials); the missing scope is the context
  shown by the runtime warning, not the keyword globally. SCO `[approachdist]`
  is parsed for traffic signals, and SCO `[collision_mesh]` feeds supported
  scenery collision; native parity remains unverified. SCO `[complexity]`
  values are retained, but detail filtering is not implemented. `[newachse]`
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
- [~] BUS and map descriptions are retained; SCO descriptions are ignored by
  rendering and skipped as free-form blocks by both SCO and inline-model
  loaders, including keyword-shaped text. Define remaining metadata handling
  for `[friendlyname]` and `[usrinfo]`; never copy `[usrinfo]` values into
  logs, diagnostics, or source control.

#### Documented script API gaps

- [~] Timetable lookups: scheduled AI buses publish line and destination
  strings, ordered stop names, and selected `.ttp` arrival/departure profile
  times before script initialization. `.ttl` profile selections remain aligned
  with each departure, and `GetTTDelay` tracks schedule offset along the active
  route.
  `GetTTBusstopCount`, `GetTTBusstopIndex`, and indexed `GetTTBusstopName`
  return the active map timetable's passenger-stop data in both backends;
  `GetTTDelay`, `GetTTBusstopArr`, and `GetTTBusstopDep` now return schedule
  values in seconds. `GetTTTerminusIndex` uses the last mapped passenger stop as
  the route terminus, or `-1` when no stops are mapped. Native and Lua probes
  verify the lookups and stack contracts.
- [~] Ticket/passenger macros: `GiveChangeCoin`, `GetTicketName`, and
  `GetTicketValue` use map ticket/currency data and `GetHumanCountOnSeat` reads
  the current seat-occupancy variables. `GetHumanCountOnPathLink` reports the
  active boarding/alighting passenger's authored cabin pathlink, including
  when the bus is not drawn; scenery/world pathlink occupancy and ticket
  purchasing/payment remain unimplemented.
- [~] World lookup: `GetHeightAbovePoint` raycasts against the live static
  collision world and returns a safe zero with a warning when no provider is
  available. Vehicle probes cover provider results and stack behavior.
- [ ] Arrival-board lookups: `GetArrBusLine`, `GetArrBusTerminus`, and
  `GetArrBusTimeDiff` need the parent bus-stop association and arrival queue,
  which scenery script runtimes do not currently receive.
- [~] `int_haltewunsch` is dispatched when a simulated passenger completes
  boarding with the next stop assigned as their destination. Native-independent
  passenger request timing is not simulated.
- [~] Native and converted Lua backends now preserve the top string on chained
  `(S.$...)` stores, and native system-macro dispatch enforces object-kind
  availability like Lua registration. A cross-backend probe covers chained
  stores. Exact OMSI parity for string-store stack behavior and out-of-kind
  macro calls still needs independent verification.
- [~] Verify OMSI parity for `NrSpecRandom`; OpenBus now provides deterministic
  seeded output, and both backends reject non-finite/out-of-range seeds with a
  once-per-runtime warning instead of performing undefined conversions.
  Its algorithm and range still need compatibility evidence.
- [~] The selected simulation calendar updates `Time`, `Day`, `Month`, `Year`,
  and one-based `DayOfYear` across midnight. Traffic density weekday classes
  now change at the selected calendar's midnight, including weekend and week
  boundaries; focused clock and density probes cover a Friday-night start and
  fractional start-time carry across midnight.
  Selected precipitation initializes `PrecipType`/`PrecipRate` for both
  scripts and rendering, and active vehicles refresh their local counterparts;
  both script backends have regression coverage.
  `NoSound` reports OpenAL context availability at startup and each frame,
  with Linux null/unavailable-driver probes; device loss and per-clip failure
  are not detected. `Weather_Temperature` and `Weather_AbsHum` now publish
  validated host-configured values through `OPENBUS_WEATHER_TEMPERATURE` and
  `OPENBUS_WEATHER_ABSHUM`; `AutoClutch` reflects the opt-in
  `OPENBUS_AUTO_CLUTCH` setting, without implementing drivetrain assistance.
  `SunAlt` can be set with `OPENBUS_SUN_ALTITUDE`, but is not computed from map
  location and time. `Pause` is connected to the `P` toggle and stops physics,
  traffic, passenger service, the simulation clock, and script frames while
  rendering continues. `wearlifespan` is vehicle-local, not a system variable.
- [~] `humans_count` now reports the passenger service's occupied-seat count,
  matching `passengers_onboard` for player and AI vehicles. Replace default-only
  values for documented variable families whose producers are absent:
  `AI_*`, `AI_Scheduled_*`, `GivenTicket`,
  `FF_Vib_*`, `Snd_*`,
  `Cabinair_*`, `Dirt_*`, `TrafficPriority*`, `Axle_Brakeforce_*`,
  `Axle_SurfaceID_*`, `articulation_*`, `boogie_*`, and
  `contactshoe_*`. `PAX_Entry#_Open` and `PAX_Exit#_Open` remain script-owned;
  OpenBus consumes them for passenger service but does not derive them from
  door mechanics.
- [~] `PAX_Entry#_Req` and `PAX_Exit#_Req` follow the selected door even on
  multi-door buses. Boarding uses the first configured entry; each alighter
  uses a reachable exit selected from the seat's cabin path, with a nearest-door
  fallback when paths are disconnected. Movement and animation use the same
  selected door and wait for its `PAX_Entry#_Open` / `PAX_Exit#_Open` state.
  Entry requests stop when no seat is available or due to be freed by an
  alighter, while an already reserved boarder keeps its request.
  Selection is deterministic rather than a complete native multi-door policy;
  simultaneous passenger movement and selection of alternate entry doors
  remain unimplemented.
- [~] Add runtime producers for documented vehicle/scenery/human strings and
  state such as `act_route`, `act_busstop`, `SetLineTo`, `yard`,
  `file_schedule`, `Refresh_Strings`, `Switch`, `LastMovedDist`, `PAX_State`,
  and `HeightOfSeat`, or document their supported default-only behavior.
  Scheduled AI buses now hold at each passenger stop after requesting departure
  until the script clears `AI_Scheduled_AtStation` to 0; other names listed here
  remain unimplemented or default-only.

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
  and composite subsequent ground-texture layers per tile.
  `OPENBUS_MAP_GROUNDTEX`
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
  parents/anchors and cycles are skipped. `[water]` markers and binary
  sidecars are parsed/validated and rendered as provisional translucent tile
  quads. Their
  inferred SW/SE/NE/NW height ordering, appearance, and multiple-surface
  behavior
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
  `[startdate]`/`[enddate]`; selector edits, `[delete]`, `[typ]`, and
  `[relabel]`
  still have no independent runtime semantics.
  Global identity/description, version, background/map-camera data, marker
  flags,
  seasons, and traffic-density records are retained as metadata, but fields such
  as `[worldcoordinates]` have no runtime effect; timetable/HOF and AI-list map
  state remain out of scope. Unsupported `global.cfg` and tile sections retain
  their raw payload lines with source line numbers; their runtime semantics
  remain unimplemented.
  `.map.water` presence and count-plus-four-float records are decoded and
  validated; provisional tile quads use an unverified corner ordering.
  `.map.LM.bmp` and `.map.terrain_0.rdy` paths/presence
  are inventoried but their payloads are not decoded or rendered.
  Unimplemented bracketed sections in tile files and `global.cfg` now produce
  line-numbered warnings and preserve each section's opaque payload lines;
  no runtime meaning is inferred from them.

#### AI vehicle and human traffic

- [~] Spawn random road traffic from configured `.ovh` entries in modern
  `ailists.cfg` road groups on connected `[path]` type-0 road splines and
  point-backed scenery-object road paths; `.bus` vehicles
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
  tile window, including tiles activated later during streaming. Spawn
  candidate tiles are derived from sampled world positions rather than the
  owning spline tile, so splines and point-backed scenery paths crossing tile
  boundaries can use the active tile containing the candidate. Initial pool
  construction prioritizes
  routes with candidates in the current active tiles, verifies candidate
  positions against the active window before local-priority admission, then
  fills remaining capacity with map-wide routes. Candidate positions keep a
  vehicle-sized
  margin from spline endpoints, remain over 50 m plus vehicle radius from the
  camera, and preserve collision clearance. Scheduled buses remain restricted
  to connected endpoints or their authored timetable origins. Population
  changes unload vehicles off-screen; route-complete
  actors are recycled from the bounded pool, resetting route progress, speed,
  and junction-wait state. Road actors outside active tiles release their
  population slots, and inactive pool vehicles can rebuild routes from local
  connected lanes as tiles stream in. Scheduled buses retain their timetable
  routes and pause outside active tiles. The startup pool remains the hard
  capacity; validate long-distance streaming and population refill in-game.
  When no complete timetable match exists, a route following at least two
  consecutive entries from the timetable's first checkpoint may use an
  explicitly logged, itinerary-inaccurate prefix fallback. It stops at the
  matched prefix end and does not register player passenger stops for the
  incomplete itinerary; missing checkpoints and disconnected transitions are
  never skipped or synthesized.
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
  1.5 s time headway and 2 m standstill buffer. Oriented swept footprints
  prevent advancement
  through player/AI/static vehicles. Same-route following respects route order
  across spline boundaries so a rear vehicle's reservation cannot block a
  leader on a curve. A gap- and relative-speed-based following target and a
  gentler signal approach envelope damp speed changes while retaining swept
  collision and stop-line safeguards. Conservative crossing reservations
  age blocked junction movements at 8 priority points per second (capped at
  255);
  stopped actors queued at a signal or behind the player/AI bus reset their
  stuck timer, and pending removals are canceled while that wait remains valid.
  any vehicle part (front, center, or rear) inside a junction counts as
  committed. Approaches check the full junction connector and exit clearance
  before entry, preventing a queued merge exit from leaving a vehicle stopped
  across the main road. Committed vehicles use physical swept clearance within
  the junction instead of the larger approach headway, and accrued wait
  survives approach movement while a junction path is in lookahead. Regressions
  cover queued T-junction exits, right-turn wait aging, clearance, and
  no-overlap.
  Vehicles return to endpoint queues only after
  an unconnected endpoint is off-screen; population reductions happen
  off-screen.
  Regressions exercise actual
  left/right turns below 4.5 m/s on 10 m curves, radius rejection, following
  gaps at different speeds,
  stopped-player-bus avoidance, crossing progress/separation, long-frame
  blockage, crossing wait-age priority, right-turn merge clearance, and
  four-approach no-overlap/progress. Crossing reservations distinguish
  same-direction traffic on a curved shared path from conflicting movements.
  SCO signals linked to road paths now use native stop/jump conditions,
  initialize their shared cycle from map time, and treat dark state 12 as
  permissive; regressions cover route-start, loop-wrap, conditional stops,
  jumps, and player approach-distance requests. Native stop-line fidelity,
  sign-specific controls, right-of-way parity and deadlock recovery at live
  junctions,
  same-direction overtaking/lane changes, and physical player-to-AI
  collision response are listed in the remaining-work checklist below.
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
  When no full route match exists, a connected route following at least two
  consecutive entries from the timetable's first checkpoint can be dispatched
  using that timetable's departures; startup logs identify this as an
  itinerary-inaccurate prefix fallback. It stops at the matched prefix end and
  does not register player passenger stops for the incomplete itinerary.
  Complete timetable matches are preferred over these fallbacks, so a partial
  match cannot hide a later complete route. Road traffic paths authored inside
  placed scenery
  objects retain the parent object's map ID and path index, allowing repeated
  timetable `.ttr` entries for junction paths to match the directed lane graph.
  Repeated checkpoints on ordinary map splines are collapsed to one route
  segment; distinct placed-scenery paths retain their individual selectors.
  A live Cotterell run still reports three complete routes and seven partial
  itinerary fallbacks, so unrepresented or disconnected map paths remain.
  A live Cotterell startup recognized complete routes for line 12, U28, and one
  U18 trip; other candidate trips remain explicitly logged as partial
  timetable-origin prefix fallbacks rather than timetable-faithful. A one-bus
  live run departed line 12 from endpoint spline 14089 at minute 483, emitted
  progress at 100 m intervals
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
  A live HK West Kowloon 8P diagnostic matches only 5 of 53 represented
  checkpoints and reports a 68.3 m horizontal gap from spline 1596658 to
  1583971. Map-wide indexing currently adds map-spline lanes, but scenery
  object `[path]` lanes are added only while their tiles are active; scheduled
  routes can therefore lack off-window checkpoints. Index scenery traffic
  paths map-wide before evaluating this route further, then retain the
  disconnected-transition diagnostic rather than relaxing exact matching.
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
  elbow, hand, neck, and waist pivots. The runtime now forwards finger, neck,
  and waist values by name rather than mixing their positional arguments.
  Pedestrian O3D weights and negative `[setbone]` bindings now drive an
  approximate 13-bone GPU gait, with procedural fallback for meshes without
  usable weights. Invalid weight records are reported and skipped; some
  installed meshes use indices beyond the decoded vertex array, so their
  mapping and native weight semantics still need investigation. Hidden-GL
  smoke tests render installed Woman01, Woman03, and MeCaT04G02 models in both
  poses; validate joint alignment and visual parity in-game. Generic
  vehicle and scenery skeletal skinning remains unsupported. Seat height,
  mass, voice, and age remain for passenger boarding work.
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
  population targets using provisional linear interpolation and endpoint
  clamping. Density changes now defer population transitions until off-screen
  positions are available and route footprints are clear. The startup road
  pool reserves each unscheduled group up to its maximum active target across
  configured hours and weekday classes. There is no fixed active population
  cap; separate group peaks can make the queued pool larger than any one
  simultaneous target. Split-peak and uncapped interior-rounding regressions
  cover the reservation logic. Weekday schedules derive their starting
  weekday from the map
  system calendar (`Day`, `Month`, `Year`, and `DayOfYear`); the date and time
  variables advance with elapsed simulation time. Verify native units,
  interpolation, and time-of-day policy before treating the behavior as
  OMSI-compatible.

#### Remaining AI traffic implementation

The entries above describe current behavior and its test coverage. These are
the outstanding implementation and validation tasks:

- [~] **Complete scheduled bus routes:** complete timetable matches are now
  preferred over partial itinerary matches. Cotterell startup has queued
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
  routes and 7 partial itinerary fallbacks. These matches start at the
  timetable origin and stop at their connected prefix; do not classify them as
  timetable-faithful or synthesize a disconnected transition. Resolve it
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
- [~] **Complete traffic control:** SCO fixed-cycle signals now initialize from
  the selected map time, derive a zero group cycle from the longest configured
  light schedule, and run native-format conditional stop/jump controls from
  AI, pedestrian, and player requests. Dark phase 12 is permissive. Focused
  probes cover request conditions, jump/rewind behavior, player
  `[approachdist]`, and selected-time initialization. Tile `[rule]` path/group
  priorities are supported, but no separate sign-object behavior is inferred.
  Equal-priority, uncommitted straight crossings provisionally give precedence
  to the vehicle approaching from the right; after either actor waits 3 seconds,
  the preference yields to the existing tie-breakers. Native behavior for
  turning approaches, stop-line placement, and full OMSI parity remain
  unverified.
- [~] **Improve junction control:** committed junction-path actors clear before
  new approaches; otherwise authored path/group priorities precede wait age,
  with wait aging fairly ordering equal-priority movements except for the
  provisional right-before-left preference at straight crossings. Before
  claiming a junction, a vehicle on a lower-priority approach spline gives way
  to
  higher-priority traffic only when predicted arrival times overlap at a
  crossing or merge, and releases an unused claim if such traffic appears;
  priority traffic ignores that unused claim. Deadlock exemptions follow
  actual blocker chains instead of bus proximity. Synthetic
  regressions cover separation, progress, junction clearing, priority-over-
  wait, default straight-crossing right-of-way, main-road priority at
  T-junctions, unobstructed turn transitions, and approach stop distance.
  Validate native turning/right-of-way behavior and deadlock recovery at
  representative live junctions.
- [ ] **Add advanced vehicle behavior:** implement safe overtaking and lane
  changes, initially limited to verified same-direction adjacent lanes. Do not
  infer opposing-lane passes without validated visibility and prohibition
  semantics.
- [o] Extend swept-footprint handling to articulated vehicles; articulated
  runtime behavior remains outside the currently supported scope (see Section
  1).
- [~] **Render configured human models:** pedestrians now render the configured
  `.hum` first-LOD meshes instead of primitive proxies. `.hum`/model CFG
  parsing, mesh parsing, and texture decoding are asynchronous; OpenGL upload
  remains on the render thread. Pedestrian skinning uses an approximate gait;
  native animation parity and AI driver model rendering remain open.
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

- [ ] Implement or explicitly scope out the remaining runtime effects of
  retained
  `global.cfg` settings: `[description]`, `[NextIDCode]`,
  `[worldcoordinates]`, `[dynhelperactive]`, `[realrail]`, `[LHT]`,
  `[backgroundimage]`, `[mapcam]`, `[moneysystem]`, `[ticketpack]`,
  `[repair_time_min]`, `[years]`, `[realyearoffset]`, `[standarddepot]`,
  `[addseason]`,
  `[trafficdensity_road]`, and `[trafficdensity_passenger]` beyond the
  provisional population multiplier above.
- [~] Spline `skewStart`/`skewEnd` fields now drive a provisional longitudinal
  shear in rendered/collision road geometry. Verify the assumed units and
  formula against OMSI before treating it as compatible. `.sli`
  `[halfcantwidth]` now overrides the fallback width derived from valid
  `[heightprofile]` lateral extents for cant clamping; native visual parity and
  full authored height-profile behavior remain unresolved. The Cotterell log
  reports 951 profile strips with fewer than two points (including
  `pavement_a.sli`).
  Two currently installed `pavement_a.sli` copies each parse as two renderable
  strips with two points apiece and no invalid texture indices. This does not
  reproduce the retained log or identify its exact source asset/version; rerun
  a representative map and investigate any remaining short strips.
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
- [ ] Investigate the 7 spline attachments with no local placement in the
  Cotterell run (examples reference `splineIndex=154`, `102`, and `5`);
  determine
  whether they are unresolved repeater-master references, invalid local spline
  records, ambiguous IDs, or bad source placements. Unique reciprocal spline
  chains now resolve across tile boundaries for ordinary attachment rows.
  Cotterell is not present in the current Linux installation's map directory,
  so the retained report cannot be reproduced here.
- [~] Nonzero-LOD scenery parts now render using the vehicle renderer's existing
  threshold-to-distance mapping. Validate distance transitions and visual
  parity with representative installed scenery; 9 parts were skipped in the
  Cotterell run, including Streetobjects_RUE light models.
- [~] An empty-name/empty-path texture request is now treated as an
  intentionally
  untextured batch instead of a decode failure, and decode failures include the
  source mesh path. The parked Opel Manta DDS remains unverified because its
  model asset is not present in this checkout.

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
  match directives case-insensitively while preserving payload case and
  skipping disabled blocks, as in the shared CFG reader. A profile probe
  covers mixed-case tags, textures, road geometry, paths, and disabled records.
  Malformed points and profile indices now produce line-numbered diagnostics
  rather than being silently dropped; recovery and logging have focused probes.
  The earlier aggregate skipped-strip count has not been rechecked on a live
  map.
  Retain `[nocollision]` to exclude marked profiles from road collision while
  preserving their render geometry; native OMSI profile semantics are still
  unverified. Apply explicit `[halfcantwidth]` to cant clamping as well. Spline
  cant is provisionally interpolated as percentage crossfall in shared
  render/collision geometry;
  precedence and sign/unit parity need OMSI visual confirmation. Full authored
  height-profile, texture/material, and marking behavior and representative
  visual evidence remain incomplete. Road UV V includes a valid parsed v11+
  spline chain offset; continuity and OMSI
  parity still need validation against installed linked spline assets.
  Road collision now looks beside every referenced profile texture for the
  appended `.surf` companion (for example, `White.dds.surf`), decodes bitmap
  red-channel height data, and applies the provisional -2 cm to +2 cm range
  through UV-based collision subdivision. A generated black/white marking
  fixture verifies depression, elevation, and collision tessellation; native
  asset orientation, scale, filtering, and OMSI suspension parity remain to
  validate. Uniform-red `.surf` maps on affine road quads now retain the
  original road triangles with the same height offset rather than subdividing.
  A focused probe checks the reduced count and preserves subdivision on curved
  quads. Dense tile collision is split into multiple meshes instead of silently
  dropping later road sections at one million vertices; a generated tile
  exceeding that limit checks that all seven sections remain. A moving-wheel
  probe stays supported on synthetic flat and rough 0.125 m triangle grids
  without a fallback ground plane, including a 60 Hz regression measuring
  penetration and wheel motion throughout acceleration to above 40 km/h.
  Wheel contacts now select the deepest eight from up to 32 collision
  candidates per static mesh; the earlier speed-gated ray/force recovery was
  removed because it left substantial low-speed penetration. HK West Kowloon
  tiles (7,5) and (8,5) still
  contain 333,436 and 194,700 road triangles, mostly nonuniform `.surf`
  subdivisions, and had no sections skipped in those samples. The user
  confirmed that selecting deeper wheel contacts resolves the reported
  installed-map fall-through and bounce. Safe reduction of nonuniform height
  maps remains unverified. The `C` collision wireframe outlines
  `.surf`-mapped road triangles in cyan for visual inspection and shows dynamic
  `[crashmode_pole]` capsules,
  turning red while in contact. Pole radii are capped to keep collision close
  to the post shaft. `.cfg` surface behavior and material/friction semantics
  are not implemented.
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
  duplicate contacts where road strips overlap terrain. Supported scenery
  `[boundingbox]` fallback is probed; non-O3D collision formats remain
  unsupported.
- [~] Extend the per-placement scenery `init`/`frame` and `[visible]` path with
  script-driven animations, dynamic material changes, sound dispatch, and
  mouse events after those runtime semantics are verified. `[varparent]`
  script-variable inheritance remains distinct and unsupported.
- [~] Decode `.map.water` sidecars into per-surface four-height records;
  determine corner/quadrant ordering and validate provisional water rendering.
  Per-tile/
  detail/seasonal ground textures, map lightmaps, and Chrono variant resolution
  remain. Chrono selector IDs and opaque override payload lines are parser-only.
- [~] Map loading is being moved to background preparation with main-thread
  publication. Map-wide tile/traffic indexing now runs on the asset worker pool;
  map startup waits for that phase and for the initial active window. Newly
  activated tiles submit `.map` parsing, terrain-file parsing, and terrain mesh
  vertex/index construction to workers; decoded results are published and
  uploaded on the renderer thread, keeping OpenGL operations on the
  context-owning thread. The old active window remains visible until the new
  window is ready, and at most one terrain tile and one scene tile are published
  per frame. The start menu now submits the initial active window's tile,
  terrain, scenery, and road preparation to asset workers, publishes prepared
  resources on the GL-owning thread, and stays open until the spawn window is
  ready; gameplay then keeps later tile requests asynchronous. Initial startup
  preparation failures are reported to the menu instead of leaving it waiting
  indefinitely. Tile jobs are prioritized by distance to the requested center
  and
  capped to eight pending preparations per job type; map and terrain decoding
  overlap. Per-tile `.sco` and model configuration resolution/parsing is now
  scheduled on workers before scene publication; results are consumed and
  committed on the renderer thread, including attached-object parent assets.
  Scenery O3D/X mesh parsing and material-group vertex expansion are included
  in those worker results.
  Initial map load waits for configuration prep needed by the first active
  window. Road profile tessellation, road mesh generation, and texture path
  lookup plus water-sidecar parsing and surface geometry are now prepared on
  workers from tile/profile snapshots. Scenery texture path resolution is also
  captured in worker-prepared assets. Scenery material interpretation,
  bounding-radius calculation, and tree-preview geometry are prepared by asset
  workers; material batches reference existing mesh vertex storage rather than
  copying it. Later streamed road-buffer publication is limited to four
  batches per frame and the previous active window remains drawable until
  pending road batches drain. Menu startup also publishes incrementally; the
  non-menu startup path retains its synchronous initial-window wait.
  Scenery and terrain texture binding/creation, GL buffer creation/uploads,
  scene-tile publication, and batch registration remain synchronous on the
  renderer thread, so boundary stalls are still possible.
  Base ground-texture decoding now runs on the asset workers concurrently with
  map-wide indexing. Scenery- and road-referenced image decoding also runs
  during worker preparation. Active terrain jobs include ground-layer, detail,
  and mask image decoding in their prepared results; texture object creation
  and upload remain on the renderer thread.
  Road-geometry preparation is capped at eight tile-level worker jobs. Scenery
  configuration/model preparation is capped at eight unique asset jobs shared
  across tile requests, so assets referenced by multiple active tiles reuse the
  same in-flight result. Work already queued for a stale tile is allowed to
  finish, but results are discarded unless another requested tile still needs
  the result.
- [ ] Keep shared-cache mutation, placement state, and OpenGL resource
  creation/upload/destruction on their owning threads; bound publication work
  per frame and keep the previous active window drawable while new tile results
  are incomplete.
- [ ] Measure tile-boundary p95/p99 frame time, memory, and upload cost;
  validate visual and collision continuity in-game.
- [~] AI traffic driving, pedestrian movement, signal advancement, and
  population searches run on a dedicated worker using value snapshots for
  actors, routes, humans, obstacles, and signal requests. The worker advances
  the shared traffic-signal controllers; the owner thread applies
  generation-matched motion/pose results and the worker does not mutate
  vehicles, scripts, renderer state, or ODE objects. Fixed-step accumulation,
  generation changes, blocker identity, population planning, signal clocks,
  and worker progress under population load have focused probes. The worker is
  active by default; stress/replay determinism and shutdown/failure behavior
  under representative live traffic still need validation.
- [ ] Keep collision mesh CPU preparation on workers and ODE publication/removal
  on the simulation thread; measure publication cost and contact readiness,
  then define safe behavior for fast travel, teleports, worker failures, and
  contact continuity. Validate live driving across representative tile seams
  before claiming hitch-free streaming.
- [~] Complete HOF-backed map state and validate remaining timetable/traffic
  control semantics. Scheduled player trips use the spawned bus's HOF and
  matched timetable routes, with selection and cancellation covered by focused
  lifecycle probes. Scheduled AI dispatch uses parsed departures only when a
  route matches an endpoint; unmatched map itineraries remain unsupported.
  Linked SCO signals use fixed phases and native-format conditional stop/jump
  controls, with cycle initialization from selected map time. Native stop-line
  placement, `signalroutes.cfg` routing, remaining timetable matches, and full
  OMSI parity remain unverified; the inspected Grande Porto signal-route file
  is only a header/template.

### GPU visibility and draw submission

The current indirect path generates fixed-slot commands on the GPU but still
submits individual draws. Same-frame query occlusion covers ordinary opaque
scenery, with optional transparent scenery, bounded terrain/opaque-road and
water queries; vehicles are not query culled. CPU culling remains the default
and fallback, and both projected-size thresholds default to disabled.
See [COMPUTE_CULLING.md](Docs/COMPUTE_CULLING.md) for settings, tests, and
limitations.

- [ ] Reduce indirect submission overhead with compatible geometry/material
  batching, instancing or multi-draw and GPU-readable per-instance transforms.
  Preserve scripts, material changes, winding, authored render groups and
  ordering, with CPU/individual-draw fallback where batching is unsafe or
  unsupported. Verify actual draw-call reduction and pixel parity across
  main, mirror and capture views, both with and without query occlusion.
- [ ] Evaluate GPU command/instance compaction only after batching exists and
  profiling shows it is worthwhile. Preserve draw order where required and
  avoid synchronous visibility or draw-count readback; use GPU indirect-count
  functionality only when supported.
- [ ] Implement and validate conservative current-view depth-pyramid/Hi-Z
  occlusion without adding another application-selectable culling backend.
  Provide a sampleable depth source, correct maximum-depth reduction for
  standard depth testing, conservative projected coverage, per-view isolation,
  appropriate barriers, and visible fallback for near-plane or uncertain
  bounds and failures. Compare actual raster output against unculled views.
- [~] Opt-in bounded water queries have raster coverage; validate them in
  representative in-game and mirror views and measure their cost. Extend
  occlusion to vehicle parts where beneficial, accounting for animated and
  crashable bounds, transparency, depth-write/check exceptions, reflections
  and near-plane intersections; render normally when bounds are uncertain.
- [ ] Improve opaque render order only within compatible authored render
  groups, without moving cutouts, decals, transmap, special depth modes or
  transparent draws across ordering barriers. Add real-raster regressions for
  partially visible geometry and overlapping materials.
- [ ] Obtain repeatable London-map CPU+queries versus compute-indirect+queries
  measurements on representative Radeon hardware, including the main view and
  mirrors. Record frame-time distributions, CPU submission and GPU/presentation
  timings, configuration, resolution and asset warnings. The current short
  Mesa software runs show fewer submitted batches but do not establish a
  hardware performance improvement; submitted triangle counts do not measure
  geometry suppressed by GPU commands or conditional rendering.
- [ ] Re-run the focused CPU/GPU, query, traffic-light, vehicle and material
  probes after these changes, and validate representative in-game scenes and
  mirror views. Keep minimum-size cutoffs disabled unless explicitly set, and
  retain conservative fallback and resource/view isolation on errors.

### Broader performance and platform validation

- [~] Reflection, async loading, texture scaling, and material batching are
  opt-in/performance-sensitive paths and need repeatable comparison runs.
- [ ] Compare the vehicle draw-list/material-selection reuse and static
  ground/grid VBO changes using repeated, uninstrumented runs; report frame
  p50/p95 and keep traced scope totals separate from frame time.
- [~] Traffic driving reuses overlapping curvature samples and filters following
  and priority candidates with spatial indexes, retaining same-route leaders and
  conservative fallbacks. `OpenBusMapTrafficRuntimeProbe
  --benchmark-traffic-scale`
  covers 333 separated actors, while `--benchmark-junctions` hashes junction
  state on every update. Uninstrumented Linux release runs measured
  approximately 22.3-23.6 ms/update before and 20.0-20.8 ms/update after for
  the synthetic 333-actor case; junction results were approximately 4.1-4.6
  ms/update with matching state hashes. Re-profile representative in-game
  traffic and validate dense junctions and variable simulation timesteps.
- [~] Following searches use a tighter conservative radial candidate range and
  reuse heading calculations; arbitration reuses per-actor hold state and
  lookahead reuses each segment's vehicle speed limit. The runtime probe checks
  speed-limit changes at segment and loop boundaries and offers
  `--benchmark-traffic-dense` for 333 nearby parallel actors. Paired,
  uninstrumented Linux release runs showed overlapping timing ranges, so the
  performance impact on representative maps remains unconfirmed. Profiling
  now separates following candidate collection from evaluation, and initial
  footprint construction from active junction claims. Worker-rate adaptation
  now smooths requested-rate changes and only updates the worker when the
  configured rate has changed materially, avoiding per-render-frame lock and
  wakeup churn. Validate rate stability under real backlog and high-load
  captures;
  do not change timestep or backlog behavior without those measurements.
- [~] Traffic actors more than 500 m from the main camera use 2 m curve-speed
  sampling, promoted back to full sampling inside 400 m. Following, collision,
  signal, junction, obstacle-clearance, and authored speed-limit checks remain
  full detail; coarse curve samples use an extra braking margin. The 333-actor
  synthetic lookahead benchmark had matching distance/speed sums and median
  update time of 19.30 ms reduced-detail versus 21.35 ms full-detail across
  three paired runs. These are preliminary straight-route Linux measurements;
  validate curved routes and representative in-game traffic before claiming a
  general speedup.
- [~] Large groups sharing an open directed route now build a per-substep
  route-distance index and inspect only plausible leaders in the braking and
  response horizon, while retaining the spatial candidates and full checks.
  Small groups and closed/loop routes retain the previous exhaustive route scan
  to preserve seam behavior. The 1,000-actor shared-route benchmark completed
  at 24.75 ms/update in one Linux release run; compare repeated baseline and
  optimized captures before claiming a speedup, and profile loop-heavy traffic.
- [~] Same-route traffic skips redundant priority pairs and crossing reservation
  candidates, and reuses exact junction-preview footprints where available.
  `--benchmark-queue` exercises 104 queued vehicles and hashes their state;
  `OpenBusMapTrafficDenseQueueProbe` verifies clearance and braking. Use
  `OPENBUS_TRACE_TRAFFIC_DETAIL=1` for opt-in lookahead speed/clearance scopes.
  An uninstrumented RelWithDebInfo run measured 1.05 ms/update before versus
  0.81-0.83 ms/update in three later runs, with matching queue state hashes;
  timings are preliminary and unpaired. Measure crowded in-game routes and
  junctions; shared geometry and lane indexing across different routes remain
  possible future work.
- [~] Collision broad phases reuse per-substep vehicle/obstacle diagonals,
  and junction occupancy is cached until the actor moves. The dense and mixed
  dimension/rear-axle queue probes check braking and clearance. Uninstrumented
  RelWithDebInfo samples measured 0.84 ms/update before versus 0.70-0.76 ms
  afterward on the 104-vehicle queue, with matching state hashes; these runs
  are unpaired. Compare representative in-game frame times before estimating
  the benefit.
- [~] Transmap-transparent batches keep their original vertex buffer and draw
  triangles through a cached, view-sorted index buffer, avoiding full vertex
  reordering and upload for each view. Index buffers are bounded to three view
  directions per batch; profiler scopes separate cache hits, CPU sorting, and
  index uploads. Measure the new scopes in a crowded scene to quantify
  frame-time
  improvement.
- [~] Within each traffic substep, junction ranges and unchanged actor
  footprints
  are reused, and claim-section overlap checks are shared across samples.
  The traffic probe checks stationary leaders resuming across an occupancy-cell
  boundary. In three uninstrumented Linux release comparisons, the junction
  benchmark kept the same state hash and took 4.39-4.61 ms/update versus
  4.51-4.77 ms/update before; timings still need representative map validation.
- [~] `RenderLoopDraw.cpp` now filters scenery traffic segments to the active
  tile window once per debug draw and reuses that ordered view for endpoint
  lookup, path geometry, signal stop lines, and signal details. Traffic routes
  are filtered by their indexed owning tile before route construction; their
  active first segments are reused for endpoint lookup and debug geometry.
  A traffic-runtime probe compares the pre-filtered results with the previous
  route-first selection, including ambiguous IDs and multiple tiles. The
  behavior overlay reuses its per-frame current actor poses and shares each
  predicted pose between its trajectory and reservation boxes. Other diagnostic
  pose samples remain; measured overlay performance and visual parity remain
  to be validated on representative maps.
- [~] `OpenBusVehicleIndirectReuseProbe` checks opaque/transparent list reuse,
  script-variable-driven material reclassification, and view-dependent
  transparent sorting on CPU and compute-indirect paths. Reflection-pass
  culling and sorting still need equivalent vehicle-level regression coverage
  before broadening those caches.
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
- [ ] Validate Windows and Linux helper scripts from clean build directories.
- [~] Focused clang-format checks pass for changed ranges, and clang-tidy runs
  using the exported Windows compile database. The repository-wide checks still
  report existing formatting/lint warnings outside the touched ranges. Linux
  application builds and focused probes have since been run.

Done when performance options have documented tradeoffs and both primary build
platforms have a reproducible build-and-smoke procedure.

## 7. Final release gate

- [ ] Run the complete screenshot matrix for MAN DL05 and E400, retaining the
  build revision, asset revision, logs, and expected results.
- [ ] Remove or label stale claims in README and the reference documents.
- [ ] Review generated JSON, texture logs, crash output, and profiling output
  for accidental repository artifacts.
- [ ] Mark this file complete only after every remaining item is either done or
  explicitly moved into an approved out-of-scope list.
