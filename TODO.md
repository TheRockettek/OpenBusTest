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

- [ ] Define the supported scope explicitly: MAN DL05 and SP E400 MMC first;
  broader OMSI compatibility is a later goal.
- [ ] Record a clean baseline build and smoke run for both vehicles.
- [ ] Make `Docs/SCREENSHOT_VALIDATION_PLAN.md` truthful: environment maps and
  any other features marked fully supported must either have evidence or be
  moved to the incomplete section.
- [ ] Add a short compatibility/status table to the README linking each major
  feature to its implementation and test evidence.

Done when a fresh checkout can be configured, built, launched, and evaluated
against one unambiguous support matrix.

## 1. Lock down physics and wheel stability

- [x] Fixed-rate ODE stepping with bounded catch-up.
- [x] Per-wheel suspension, steering, wheel rotation, braking, and telemetry.
- [x] Independent axle suspension and live ODE wheel rendering mode.
- [~] High-speed bump stability: local-axis damping, softened contact response,
  and steering-rate damping are implemented, but there is no automated stress
  test for repeated bumps while accelerating and steering.
- [ ] Add a deterministic physics stress probe covering straight-line bumps,
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
- [ ] Implement bump-map rendering. The CFG parser stores the texture and
  strength, but the model material/shader contract does not currently apply a
  normal/parallax contribution.
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

- [~] Lua script texture storage and pixel operations exist in
  `ScriptRuntime`; model CFG records are still largely consumed for alignment.
- [ ] Retain and connect `[scripttexture]` definitions to model materials,
  including dimensions, slot/index, update cadence, and texture lifetime.
- [ ] Implement `[texttexture]` and `[texttexture_enh]` end to end: font/text
  layout, color/alpha, sizing, material binding, and updates from script
  variables.
- [ ] Implement `[usescripttexture]` and `[usetexttexture]` material binding
  instead of only consuming the slot index.
- [ ] Implement `[illumination_interior]` and `[interiorlight]` runtime
  lighting, including controller variables and the four group assignments.
- [ ] Add a focused E400 display/light test using real model CFG data and
  screenshots at day/night or brightness changes.

Done when a configured display changes from script input and interior lights
change from their controlling variables without manual texture replacement.

## 5. Complete vehicle/model configuration coverage

- [x] Core bus identity, mass, bounding box, center of gravity, axles,
  cameras, model assembly, animations, parents, visibility, and wheel data
  are loaded for the current vehicles.
- [~] `collision_mesh` and `nocollision` records are parsed/diagnosed, but
  physics still uses the simplified chassis box and does not build model
  collision geometry.
- [ ] Decide and document the collision policy. If model collision is in
  scope, load collision meshes and honor `nocollision`; otherwise mark these
  records unsupported and stop implying full CFG compatibility.
- [ ] Implement or explicitly scope out passenger cabin/path data,
  registration/odometer behavior, AI/network sections, route/timetable data,
  and vehicle-specific view systems.
- [ ] Implement sound configuration coverage beyond the current sound-runtime
  path, including `sound_ai` where relevant.
- [ ] Add diagnostics that distinguish "recognized and active" from
  "recognized but ignored" for every parsed configuration record.

Done when unsupported configuration is visible in diagnostics and the README
claims only behavior that is active in the runtime.

## 6. Rendering and platform completeness

- [x] OpenGL 3.3 renderer, VBO model path, cameras, mirrors, frustum culling,
  LOD selection, texture cache, crash reports, tracing, and profiling hooks
  are present.
- [~] Reflection, async loading, texture scaling, and material batching are
  opt-in/performance-sensitive paths and need repeatable comparison runs.
- [ ] Add repeatable performance captures for startup time, frame time, GPU
  workload, reflection intervals, texture scaling, and material batching.
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
