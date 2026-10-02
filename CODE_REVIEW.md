# OpenBus Code Review

**Review date:** 2026-10-03
**Scope:** First-party C++ source, headers, tests, and CMake configuration. Third-party Lua/sol sources and build artifacts were excluded.

## Executive summary

The current checkout has a coherent single-threaded simulation/script/render update path,
guarded asynchronous asset decoding, and substantially more material support than the
previous review recorded. No confirmed critical memory-safety or data-race defect was
found during this review.

The main remaining compatibility gap is the set of OMSI system macros that still use
safe fallback values because route, timetable, passenger, ticket, depot, and arrival-board
data are not modeled. The previously identified bump-map gap is now resolved with a bounded
normal-map path in the model renderer.

## Confirmed findings

### Medium priority

#### 1. Several OMSI system macros still return fallback values

`ScriptRuntime` registers safe handlers for HOF/timetable, passenger, ticket, depot,
arrival-board, ground-height, and related lookups. The handlers intentionally return
`0`, `-1`, or an empty string and do not consult route, timetable, passenger, or world
geometry data.

**Locations:** `src/ScriptRuntime.cpp` (`safeNumericLookup`, `safeMissingIndex`,
`safeStringLookup`, `safeGetHeightAbovePoint`, `safeTicketName`, `safeArrivalString`,
and `safeArrivalTime`)

**Impact:** Scripts depending on stop indices/names, timetable state, passenger counts,
ticket data, depot strings, arrival boards, or vehicle-relative ground height cannot
reproduce OMSI behavior.

**Recommendation:** Keep these APIs explicitly documented as partial until the underlying
route/timetable/world systems exist. Add focused behavior tests as each data source is
implemented rather than replacing the safe fallbacks with guessed values.

### Low priority / coverage

#### 2. Renderer integration coverage is limited

The CTest suite exercises variables, configuration parsing, texture decoding, physics,
camera math, road features, O3D loading, model and vehicle configuration, and script
textures. It does not exercise an OpenGL context, shader compilation, material-state
binding, transparent depth prepasses, environment-map sampling, or end-to-end clickable
mesh rendering.

**Recommendation:** Add headless or fixture-based renderer tests where practical, and keep
manual screenshot validation for context-dependent behavior. Texture decoder fuzz/property
tests would also strengthen malformed-input coverage.

#### 3. Some script text helpers remain approximate

Script texture drawing and text-texture generation are implemented, but `getfontindex`
returns a constant value. `textlength` and the current font/text path intentionally use
fixed-width byte counts, matching the current OMSI compatibility scope rather than
counting Unicode code points. These are compatibility limitations rather than renderer
memory-safety issues.

**Location:** `src/ScriptRuntime.cpp` (`safeFontIndex`, `safeTextLength`)

#### 4. String handling is intentionally byte-oriented

All OMSI/configuration/script strings should remain fixed-width byte sequences for the
current implementation. Field counts, slicing, text lengths, font glyph lookup, and
serialization must use byte counts (`std::string::size()`/byte indexes); do not add UTF-8
decoding, Unicode normalization, wide-string conversion, or code-point counting. OMSI
does not require Unicode support for the current target vehicles, so accepting Unicode
would add semantics that are not part of the supported compatibility contract.

This is a scope constraint, not a defect. Any future Unicode work should be a deliberate
compatibility change with separate tests for byte-counted fields and authored OMSI text.

## Architectural constraints and risks

### `Variables` ownership is currently single-threaded

`Variables` stores values in unsynchronized `std::unordered_map` instances. The current
frame flow updates system variables, vehicle variables, scripts, and rendering from the
main/render thread (`RenderLoop::beginFrame`, `updateScripts`, and draw calls). The asset
worker pool decodes OBJ and texture data but does not access `Variables`.

This is therefore not a demonstrated current data race. It is an ownership constraint:
moving script execution, simulation updates, or variable-dependent rendering to worker
threads would require synchronization or a message/snapshot design. The existing TODO in
`src/Variables.h` should remain until that ownership model changes.

## Verified mitigations and resolved stale findings

- `AssetRequestManager` uses an RAII `ComInitializer` for Windows worker-thread COM
	initialization, including `CoUninitialize` when initialization succeeds.
- DXT block-count and decoded-buffer size arithmetic is checked before allocation or reads
	in `src/TextureLoader.cpp`.
- OBJ batch vertex reservation checks `source.size() * 3` for overflow before multiplying.
- Material-index expansion is capped at 10,000 before resizing occurrence state.
- Transparent batches are sorted back-to-front, with a transmap alpha-tested depth prepass
	and separate opaque/transparent passes.
- `matl_bumpmap` now flows from model configuration through asynchronous texture loading and
	batch submission into a bounded derivative-based normal-map shader path. Material batching
	is disabled for bump-mapped batches so the effect is not silently discarded.
- Environment maps have an active texture-loading and draw path; the environment shader's
	epsilon clamp is a normal singularity guard, not evidence of an exploitable division bug.
- Texture lookup/upload logging is opt-in through the verbose environment settings, avoiding
	the startup log flood noted by the earlier review.
- The unused `verboseMaterialChangeLogs` declaration was removed; the warning-enabled build is
	clean.
- Warning cleanup previously completed remains intact, including disabled trace builds,
	signed/unsigned comparisons, Windows time/environment APIs, and sound aggregate setup.

## Verification

The repository currently defines 11 CTest probes:

- `OpenBusVariablesProbe`
- `OpenBusConfigurationParserProbe`
- `OpenBusTextureLoaderProbe`
- `OpenBusPhysicsProbe`
- `OpenBusStabilityProbe`
- `OpenBusCameraMathProbe`
- `OpenBusRoadFeaturesProbe`
- `OpenBusO3DLoaderProbe`
- `OpenBusModelConfigProbe`
- `OpenBusVehicleConfigProbe`
- `OpenBusScriptTextureRuntimeProbe`

Run the configured build's CTest suite after source changes with
`ctest --test-dir build-ode -C Release --output-on-failure`.

## Suggested fix order

1. Define and document the supported OMSI compatibility scope, starting with the fallback
	 system macros that affect common vehicle scripts.
2. Add renderer/material integration coverage around shader compilation and representative
	 alpha, auxiliary-texture, and environment-map fixtures.
3. Preserve the current single-threaded `Variables` ownership rule unless a snapshot or
	 synchronization design is introduced deliberately.
