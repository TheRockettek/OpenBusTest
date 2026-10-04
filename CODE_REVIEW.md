data are not modeled. The previously identified bump-map gap is now resolved with a bounded
# OpenBus Code Review

**Review date:** 2026-10-04
**Scope:** First-party C++ source, headers, tests, and CMake/CTest configuration. Third-party
Lua/sol sources, generated files, build artifacts, and logs were excluded.

## Executive summary

The review found no confirmed critical-severity issue. The four previously documented
correctness/resource-handling findings have now been fixed and regression-tested: optional AI
configuration on Windows, script-texture bounds, the exterior camera's zero-distance
singularity, and invalid physics rates. No finding from the previous review remains open.

The main OMSI compatibility gap remains system macros that return safe fallback values where
route, timetable, passenger, ticket, depot, arrival-board, or world-height data is not modeled.
The active renderer has more material support than the earlier review recorded, but its
OpenGL/shader integration remains lightly covered by automated tests.

## Findings and remediation status

### Medium priority

#### 1. Optional AI configuration paths alias on Windows — resolved

**Original defect:** `openbus::getEnvironment()` returns a pointer into one thread-local `std::string`, and its
documented lifetime ends at the next call on that thread. `main()` retains the AI bus-config
pointer, calls `getEnvironment()` again for the AI model-config path, and only then consumes
both pointers. The second lookup overwrites the string, so the bus-config pointer now refers
to the model-config value. Supplying the paired AI environment variables can therefore send
the wrong path to bus-config loading.

**Locations:** `src/Environment.h` (`getEnvironment`); `src/main.cpp` (AI environment lookups
and path resolution).

**Resolution:** `main.cpp` now snapshots each optional path through `environmentValue()` into
an owning `std::string` immediately after lookup, before the next environment lookup.

#### 2. Script textures and drawing coordinates are not bounded — resolved

**Original defect:** `[scripttexture]` previously validated only that dimensions were positive.
The runtime could allocate `width * height * 4` bytes without a configured maximum, and
pixel/rectangle macros could grow textures from script coordinates. Those macros cast floats
directly to `int` without checking finiteness or representable range; `setPixel()` computed
`x + 1`/`y + 1`, and rectangle loops incremented through caller-provided endpoints.

**Current behavior:** `[scripttexture]` and the runtime enforce shared limits: dimensions no larger than 4096,
at most 4,194,304 pixels (16 MiB) per surface, at most 256 texture slots, and a 256 MiB
aggregate runtime texture budget. Textures loaded from files and text textures use the same
limits. Both native OSC and Lua fallback drawing paths validate finite, representable integer
arguments and slot indexes. Pixel growth is bounded; rectangle inputs are clipped and a fill
is rejected when its work area exceeds the pixel limit. Rectangle resizing occurs once before
an efficient bounded fill rather than expanding once per pixel.

**Locations:** `src/ModelConfigLoader.cpp` (`[scripttexture]` parser); `src/ScriptRuntime.cpp`
(`configuredScriptTexture`, `resizeScriptTexture`, `setPixel`, `safeScriptTexturePixel`, and
`safeScriptTextureRect`).

**Resolution:** Shared checked limits live in `src/ScriptTextureLimits.h`; the model parser,
runtime allocations, file-loaded surfaces, and text textures consume them. Regression probes
cover oversized configuration, non-representable native/Lua coordinates and indexes, and an
oversized rectangle.

#### 3. Exterior camera distance can reach a singular zero — resolved

**Original defect:** Right-mouse camera adjustment clamped `cameraDistance_` to `[0, 80]`. At zero, the exterior
camera computes identical eye and target positions. `lookAt()` divides the forward vector by
its length without handling a zero length, creating non-finite view-matrix values.

**Locations:** `src/RenderLoop.cpp` (right-mouse camera-distance clamp and exterior-camera
eye/target calculation); `src/CameraMath.cpp` (`lookAt`).

**Resolution:** Right-mouse exterior zoom now clamps distance to at least 0.1, and `lookAt()`
uses a finite fallback forward direction when eye and target coincide. `OpenBusCameraMathProbe`
checks that a coincident eye/target produces a finite matrix.

### Low priority

#### 4. Physics-rate validation accepts NaN and positive infinity — resolved

`BusSimulation::Impl` now rejects non-finite or non-positive rates before taking the reciprocal,
then verifies that the resulting fixed step is finite and positive. This also rejects positive
subnormal rates whose reciprocal overflows.

**Location:** `src/BusSimulation.cpp` (`BusSimulation::Impl` constructor).

**Resolution:** Constructor validation and `OpenBusPhysicsProbe` cover zero, negative, NaN,
positive infinity, and a denormal rate. No arbitrary maximum frequency is imposed; callers
remain responsible for selecting a practical rate.

## Compatibility limitations

### OMSI system macros with safe fallback behavior

`ScriptRuntime` still registers fallback handlers for several HOF/route, timetable,
passenger, ticket, depot, arrival-board, and ground-height queries. They return safe numeric,
index, or string defaults rather than consulting route, timetable, passenger, or world data.

**Locations:** `src/ScriptRuntime.cpp` (`safeNumericLookup`, `safeMissingIndex`,
`safeStringLookup`, `safeGetHeightAbovePoint`, `safeTicketName`, `safeArrivalString`, and
`safeArrivalTime`; system-macro registrations).

Scripts that depend on those data sources will not yet reproduce OMSI behavior. Keep these
fallbacks explicit and add focused tests when the underlying data models are implemented;
do not substitute guessed values.

### Approximate script text helpers

Script texture drawing and text-texture generation are present, but `getfontindex` returns a
constant and `textlength` uses a fixed-width byte-based estimate rather than font metrics.
These are compatibility limitations, not renderer safety defects.

**Location:** `src/ScriptRuntime.cpp` (`safeFontIndex`, `safeTextLength`).

## Architectural constraint

### `Variables` is currently single-thread owned

`Variables` stores numeric and string state in unsynchronized maps. Current frame updates,
script execution, and rendering access that state on the main/render thread; asset workers
decode assets without accessing `Variables`. This is not a demonstrated current data race,
but moving these operations to worker threads would require synchronization or a snapshot/
message-passing design.

## Coverage and verification

CMake currently registers 14 CTest probes:

- `OpenBusVariablesProbe`
- `OpenBusConfigurationParserProbe`
- `OpenBusTextureLoaderProbe`
- `OpenBusPhysicsProbe`
- `OpenBusStabilityProbe`
- `OpenBusCameraMathProbe`
- `OpenBusViewpointProbe`
- `OpenBusRoadFeaturesProbe`
- `OpenBusO3DLoaderProbe`
- `OpenBusModelConfigProbe`
- `OpenBusVehicleConfigProbe`
- `OpenBusBusConfigurationCollisionProbe`
- `OpenBusSoundEngineProbe`
- `OpenBusScriptTextureRuntimeProbe`

The probes cover parsing, variables, texture decoding, physics (including invalid-rate
rejection), camera math/viewpoints (including coincident look-at), road features,
O3D/model/vehicle configuration (including oversized script-texture rejection), sound, and
script-texture/runtime behavior (including bounded native and Lua macro inputs). They do
not exercise a full OpenGL context, shader compilation, material binding, transparent depth
prepasses, environment-map sampling, or end-to-end clickable mesh rendering. Add headless or
fixture-based renderer tests where practical and retain manual screenshot validation for
context-dependent behavior.

The latest full CMake build and CTest run after these remediations succeeded; all 14 registered
probes passed. CTest continues to emit non-fatal missing `DartConfiguration.tcl` messages.

## Verified fixes and stale findings

- `src/ConfigurationParser.cpp` strips a UTF-8 BOM from the first input line. The regression
  in `tests/VehicleConfigProbe.cpp` verifies that the first constant in a BOM-prefixed
  constant file is retained; this prevents the installed bus's first blink-timer constant
  from being silently skipped.
- `main.cpp` snapshots Windows environment-backed AI config paths before a subsequent lookup
  can overwrite the thread-local `getEnvironment()` buffer.
- `ScriptTextureLimits.h` centralizes dimensions, slot, per-surface, and aggregate-runtime
  limits; native and Lua texture macros share checked numeric conversions and bounded drawing.
- `BusSimulation::Impl` validates the physics rate and its reciprocal before storing a fixed
  step; `CameraMath::lookAt` remains finite for coincident eye/target inputs.
- Windows worker-thread COM initialization is guarded by RAII in `AssetRequestManager`.
- Texture block/decoded-size arithmetic, OBJ batch reservation multiplication, and material
  occurrence-index expansion have bounds checks in their current implementations.
- Transparent batches are sorted back-to-front with an alpha-tested transmap depth prepass.
- Bump-map data reaches a bounded derivative-based shader path; material batching is disabled
  for bump-mapped batches rather than silently omitting the effect.
- Environment-map texture loading and drawing are active; the shader epsilon clamp is a
	singularity guard, not evidence of an exploitable division defect.
- Temporary light-variable snapshots, native OSC instruction traces, and their
  `OPENBUS_DEBUG_LIGHTS` profile/documentation toggle have been removed. Profiling and ODE
  wireframe controls remain.

Earlier claims of renderer races, missing COM cleanup, absent environment/bump mapping,
unbounded material indices, and unchecked DXT/OBJ allocation arithmetic should not be
reintroduced without new evidence from the current implementation.

## Follow-up opportunities

1. Extend safe OMSI macro implementations only as their route, timetable, passenger, ticket,
   depot, arrival-board, and world-height backing data models become available.
2. Add headless or fixture-based renderer tests for OpenGL-context-dependent shader and draw
   behavior; continue manual screenshot validation where an actual context is required.
