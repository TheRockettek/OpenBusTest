# OpenBus Code Review

**Review date:** 2026-10-07
**Scope:** First-party C++ architecture and hot paths: application timing/input, physics,
map parsing/placement/collision/rendering, vehicle and texture loading, script runtime,
renderer state/resource ownership, CMake/presets, regression probes, helper scripts, and
support documentation. The current working-tree changes were included. Third-party internals
and generated source were excluded; local build/test output was used for validation. This is
not an exhaustive audit of every asset format or a certification of OMSI compatibility.

## Executive summary

The four previously documented correctness/resource-handling findings have been fixed and
regression-tested: optional AI configuration on Windows, script-texture bounds, the exterior
camera's zero-distance singularity, and invalid physics rates. The current Windows Release
build succeeds, but **23 of 24 CTest probes pass**: the collision-streaming probe fails on an
invalid fixture before exercising its assertions.

The most important follow-ups are not just draw-call optimizations:

| Priority | Area | Main risk / next action |
| --- | --- | --- |
| High | Script timing | A one-tick cap silently loses time below the configured script frequency; extract and test the scheduler. |
| High | Collision-streaming latency | Tile crossings synchronously read and build colliders on the frame thread; measure boundary frames and stage reusable CPU data ahead of travel. |
| High | Advertised Linux support | The shared CMake project unconditionally links Windows Lua binaries. |
| Medium | Renderer correctness | Raw VAO/buffer binds bypass the state cache; DXT5 capability detection uses a query invalid in the requested core profile. |
| Medium | Memory / hot-path copies | Dynamic texture revisions are checked after copying pixels; asset caches retain CPU payloads and GPU resources without a byte-based residency policy. |
| Medium | Future map features | Fully resident rendering, duplicated tile decoding, and separate rendered/collision placement paths make streaming and feature parity harder. |
| Medium | Maintainability / verification | Large orchestration units and duplicated CMake source lists impede isolated tests; fix the failed streaming fixture and add scheduler/GL integration coverage. |

Findings below distinguish confirmed defects from architectural scaling risks. No fresh
render benchmark, GPU timing, memory profile, Linux build, sanitizer run, or full
OpenGL-context test was performed. No speedup percentages or measured hitch durations are
claimed. This update changes the review document only, not production code or test fixtures.

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

## Current review findings

### High priority

#### 1. The advertised Linux build links Windows-only Lua binaries

**Location:** `CMakeLists.txt` (`OpenBusLua` imported target and post-build copy).

The target unconditionally imports `src/osc/lua51.lib` and `src/osc/lua51.dll`, links that
target into OpenBus and the script-runtime probe, and copies the DLL. These are Windows
artifacts, while the README documents Linux builds using the same CMake project. A clean Linux
build therefore cannot link the advertised targets.

**Suggested fix:** Build or discover a platform-native Lua library and make the imported target
and runtime-copy step platform-specific; verify the Linux configure, build, and CTest path.

#### 2. One-tick script cap discards elapsed script time

**Location:** `src/RenderLoop.cpp:142` and `4605–4634`
(`MAX_SCRIPT_CATCH_UP_TICKS` and `RenderLoop::updateScripts`);
the stated fixed-rate behavior is in `README.md` under `OPENBUS_SCRIPT_HZ`.

The current cap is one tick per rendered frame. If scripts are configured at 60 Hz
while rendering at 30 FPS, each frame can accumulate about two ticks, but the loop runs one and
the `std::fmod` branch discards the other. Scripts then advance at about half the configured
rate while receiving `Timegap=1/60`; hitches drop still more time. This contradicts the documented
bounded catch-up behavior and affects vehicle and scenery script state. Both launch/profile
helpers currently set `OPENBUS_SCRIPT_HZ=60`, so this is relevant to normal launches, not just
an unusual API configuration.

**Suggested fix:** Retain bounded multi-tick catch-up for expected frame intervals, or explicitly
define dropped-time semantics and the relationship between script, physics, and wall time.
Do not substitute a variable timestep without considering script integration behavior.
Extract a context-free scheduler and test 30 FPS/60 Hz, 120 FPS/60 Hz, zero elapsed time,
a hitch, and input-event ordering for both backends. Expose executed ticks and dropped time.

### Medium priority

#### 3. Raw renderer binds leave the VAO/buffer state cache stale

**Locations:** `src/CoreRenderer.cpp:598–615` (`bindModelVertexBuffer`) and `1142–1170`
(`drawTextureQuad`); `src/RenderLoop.cpp:5524` (coordinate HUD), plus raw buffer binds in
`Vehicle::sortTransparentBatch` and `Vehicle::updateMaterialChange`.

`drawTextureQuad` changes the actual VAO and array-buffer bindings but neither updates nor
invalidates `currentVertexArray` / `currentArrayBuffer`. The next cached draw can skip a
required bind. For example, model A → HUD → model A leaves the quad VAO active while the
cache still reports the model VAO/buffer. Raw array-buffer uploads elsewhere create the same
invariant violation; texture invalidation does not repair vertex bindings. Which visual
symptoms appear depends on intervening draws; no visual reproduction was run in this review.

**Suggested fix:** Route draws and uploads through one coherent binding interface, or
explicitly invalidate all affected cached state after external binds. Make the cache
context-owned before adding additional windows/backends. Add a GL-call recorder regression
for model → HUD → same model, primitive → HUD → same primitive, and upload → cached draw,
then confirm with a core-profile smoke test.

#### 4. Model-config JSON does not escape every control character

**Location:** `src/ModelConfigJson.cpp` (`writeJsonString`).

Quotes, backslashes, newline, carriage return, and tab are escaped, but other bytes below
`0x20` are written raw. A configuration string containing one of those bytes produces invalid
JSON.

**Suggested fix:** Encode every remaining control byte as `\u00XX` and add serializer tests
covering the full control-byte range.

#### 5. Launch helpers forcibly terminate every `OpenBus.exe`

**Locations:** `run.bat`, `profile.bat`, `screenshot.bat`, and `benchmark_loading.ps1`.

The helpers run `taskkill /F /IM OpenBus.exe` before starting their work. This can forcibly stop
an unrelated or actively-debugged process, not just an instance launched by the helper.

**Suggested fix:** Track and stop only the child process started by the helper, or ask the user
to close an existing instance gracefully.

#### 6. Loading benchmark is out of sync with the runtime and can misreport results

**Locations:** `benchmark_loading.ps1` (executable path, per-run log removal, environment setup
and cleanup, `Get-MedianMs`); `CMakePresets.json` (NMake generator); `src/RenderLoop.cpp`
(current load-completion logging).

The benchmark expects `build-ode\Release\OpenBus.exe`, but the documented Windows preset uses
single-configuration NMake, which writes `build-ode\OpenBus.exe`; the current workspace has the
latter and not the former. Fixing that path alone is insufficient: the script requires an
`All textures loaded.` log marker that the current source does not emit, and sets the legacy
`OPENBUS_VEHICLE` selector rather than the current bus/model configuration variables.

`Get-MedianMs` casts `Count / 2` to `[int]`, which rounds rather than floors in PowerShell.
The isolated calculation for `[1, 2, 3]` selects index 2 and returns 3 instead of the median 2.
Three- and seven-sample runs select the element above the median; one- and five-sample runs
select the correct index because of ties-to-even rounding. Some odd run counts therefore
report the wrong statistic even after the launch/logging issues are fixed.

Each run also deletes the repository's `game.log`, and the script removes/overwrites
caller-provided `OPENBUS_*` values instead of restoring them. An exception can skip cleanup.

**Suggested fix:** Derive the executable path from the configured build or use the actual
single-config output path; use supported configuration selectors and explicit readiness
metrics. Floor the median index and test empty, one-, three-, five-, and even-sized samples.
Write each run to an isolated working/output directory, check exit/readiness status, and
snapshot/restore environment variables in `try/finally`.

#### 7. Clean setup instructions omit required OpenAL development files

**Locations:** `CMakeLists.txt` (`find_package(OpenAL CONFIG REQUIRED)`);
`README.md` (Windows vcpkg and Debian/Ubuntu prerequisites).

The README's dependency commands install ODE and GLFW but omit OpenAL, which configure requires.
A fresh environment following those commands can fail before building.

**Suggested fix:** Add the appropriate OpenAL development package/port to the setup instructions
and verify them from a clean configure.

#### 8. Windows build helpers and presets are tied to one machine's paths

**Locations:** `windows_clang_env.bat`, `CMakePresets.json`, and `lint.bat`.

LLVM, MSVC, Windows SDK, and vcpkg locations/versions are hard-coded. The environment helper
overwrites `OPENBUS_CLANG_BIN` before checking it, so its instruction to set that variable does
not work. `lint.bat` checks the default `C:\vcpkg` path before forwarded arguments can supply an
alternative toolchain. Separately, the README says CMake 3.20 is sufficient, but preset schema
version 3 requires CMake 3.21 or newer.

**Suggested fix:** Respect caller-provided paths and discover or document supported toolchain
versions. Honor the lint script's custom toolchain argument, and align the CMake minimum with
the preset schema or lower the schema version.

#### 9. `run.bat` assumes it is invoked from the repository root

**Location:** `run.bat` (relative `cmake -S .`, build, and executable paths).

Unlike the other project-relative helper patterns, this script does not switch to its own
directory. Invoking it by absolute path from another current directory makes CMake configure
the wrong source tree or fail.

**Suggested fix:** `pushd` to `%~dp0` before project-relative commands and `popd` on all exit
paths.

### Low priority

#### 10. `stb` is fetched from a mutable branch

**Location:** `CMakeLists.txt` (`FetchContent_Declare(stb)`).

`GIT_TAG master` allows identical source revisions to fetch different dependency contents over
time, reducing build reproducibility.

**Suggested fix:** Pin a reviewed release or immutable commit hash and update it deliberately.

#### 11. README and reference docs contradict current support

**Locations:** `README.md`, `Docs/OMSI_CFG_GENERAL_REFERENCE.md`,
`Docs/OMSI2_MAP_FORMAT_REFERENCE.md`, and the current helper scripts.

Some statements still describe per-tile ground-mask decoding/mapping or authored map-scenery
collision as unimplemented despite current implementation and probes. Other examples describe
old `run.bat` vehicle/map/AI settings, refer to a missing `convert_textures.ps1`, or use
machine-specific absolute documentation paths. The active profile helper also enables material
batching while the README says it is disabled there. Text-X scenery support and provisional
terrain alignment also contradict older support descriptions. The CPU flame-graph section
advertises collapsed-stack output absent from the current trace implementation, and the
benchmark description says its window is hidden while the constructor requests `GLFW_TRUE`.

**Suggested fix:** Reconcile feature status and examples against current code and scripts, remove
or implement missing commands, and use repository-relative documentation links. Keep supported,
parsed-only, and visually/contact-validated features distinct.

#### 12. Generated trace artifact is not covered by the ignore rule

**Location:** `.gitignore` (exact-name trace exclusions).

`.gitignore` excludes exact trace filenames, not copied/variant trace names, so a broad
`git add` can stage large generated artifacts. The previously mentioned
`openbus_trace - Copy.json` is **not present in the current working tree**; its old size and
untracked-file claim are not current evidence.

**Suggested fix:** Keep generated traces outside the repository or add a suitable trace-file
pattern to `.gitignore` without excluding intentional JSON fixtures/configuration.

#### 13. `bundle.bat` is an empty no-op

**Location:** `bundle.bat`.

The repository documents an `obj_bundle` target, but the checked-in batch helper is empty and
silently does nothing.

**Suggested fix:** Implement the wrapper with checked build/run steps or remove the misleading
empty helper.

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

## Performance and future-change findings

### P1. High: Collision residency is bounded, but tile-crossing work is synchronous

**Locations:** `src/MapCollisionStreamer.cpp:51–89`; `src/MapCollisionBuilder.cpp`
(`buildMapRoadCollisionImpl`, `readSceneryCollisionMeshes`); `src/main.cpp`
(streamer updates between physics and drawing in both loops).

On a center-tile change, `update()` scans the full manifest and performs terrain reads,
tile/config parsing, road/scenery geometry construction, and ODE insertion on the frame
thread. The builder's profile/terrain caches are recreated for each selected tile. Each
ordinary scenery placement rereads its SCO/model configuration and parses the declared
collision O3Ds; repeated placements do not share an asset-level collider cache. Returning
after eviction repeats that work. Rendering loads its own copy of the source data separately.

The 3×3 load neighborhood / two-tile unload hysteresis limits active collision residency,
not work or latency per crossing. This is a confirmed blocking path and a scaling risk,
not a measured hitch duration. A long boundary frame can also feed the existing physics
and script catch-up/drop policies on the next update.

**Recommendation:** First count file opens, cache hits, tile-build time, and p95/p99 crossing
frame time. Add a coordinate index and a bounded, revision-keyed cache of immutable terrain,
profiles, and asset-local collision meshes. Prepare upcoming tile CPU payloads asynchronously,
with priorities/cancellation; publish ODE objects only on the simulation owner thread between
steps. Readiness must cover the bus's reachable/swept volume, not just the camera view, and
teleports/failures need an explicit safe policy. Test travel in both directions, re-entry,
negative coordinates, rapid center changes, and contact continuity before enabling deferral.

### P2. Medium: Full-map construction and residency limit larger-map scalability

**Locations:** `src/MapRenderer.cpp:560–1787` (constructor), especially `loadAllTerrainTiles`
and the separate `loadMapTile` calls at `1206` and `1357`; `MapRenderer::Impl` resource vectors.

The constructor synchronously decodes/builds/uploads the listed terrain, scenery, and roads,
and retains the resulting GPU resources until destruction. Each tile's text is parsed once
to index global placement IDs and again to build content. Road staging geometry and terrain
grids also coexist during construction, increasing peak CPU memory. Existing distance/frustum
and chunk rejection saves draw work, but does not reduce load cost or GPU residency.

**Recommendation:** Separate a reusable CPU map/placement representation from GPU realization.
Parse each tile once per map revision, retain a lightweight global attachment index, and
construct bounded nearby payloads from shared immutable asset definitions. Preserve cross-tile
attachment dependencies when evicting tiles. Track startup/first-usable-frame time, peak CPU
bytes, GPU bytes, tile counts, and owner-thread upload budget before implementing streaming.
Keep visible startup content ready rather than replacing the current behavior with blank
geometry behind unbounded futures.

`MapRenderer::updateScripts()` also ticks every retained scenery runtime, not just visible
placements. Independent per-placement state is correct, and native compiled programs are
already cached; do not replace many mutable instances with one runtime per SCO. Attribute
cost by runtime count and script before evaluating a documented background cadence or
sleep/wake policy that preserves timers and externally observable state.

### P3. Medium: Dynamic texture revisions are checked after full pixel copies

**Locations:** `src/ScriptRuntime.cpp:2216–2241` (`copyScriptTexture` / `copyTextTexture`);
`src/RenderLoop.cpp:3296–3369` (`Vehicle::updateFreeTexture`);
`src/MapRenderer.cpp:1828–1899` (`refreshSceneryTexture`).

Both snapshot APIs copy the entire pixel vector. Their callers create a fresh snapshot and
only then compare the cached revision/dimensions. Thus an unchanged texture avoids a GPU
upload but still allocates and copies CPU pixels. Vehicle refresh runs per consuming batch
and can recur in depth, transparent, and reflection passes; scenery refresh repeats for
instances/batches even when the per-slot GPU texture is current. A permitted surface can
contain 16 MiB, so revision caching does not by itself bound this avoidable bandwidth.

**Recommendation:** Expose metadata/revision first, a copy-if-changed API, or a stable immutable
snapshot with an explicit lifetime. Resolve changed slots once per runtime/frame and reuse
them across views/batches. Preserve filtering and per-material address modes (potentially
with separate sampler state), and keep GL updates on its owner thread. Add byte-copy/upload
counters and a regression proving zero pixel copies/uploads for unchanged slots and one
content refresh after a revision change, including multi-batch/reflection and resize cases.

### P4. Medium: Asset caches have no explicit byte budget or reload generation

**Locations:** `src/AssetRequestManager.cpp` (`requestObj`, `loadTextureRequest`, tracking,
destructor); `src/RenderLoop.cpp` (`SharedVehicleDefinition`, cached display lists,
`findTexture`, `uploadTexture`); `src/ScriptRuntime.cpp` (`cachedNativeProgram`).

`parsedObjCache_` retains completed futures and their source mesh data for the manager's
lifetime. Texture requests and `decodedTextureCache_` strongly retain decoded/compressed
payloads after upload; tracked vehicle GL textures/buffers are released only when the manager
is destroyed. Parsed source arrays, expanded render vertices, and GPU buffers can therefore
remain resident together. `uploadTexture(path, Image image)` adds a full RGBA copy for cached
lvalue images even when no downscaling is requested.

Path-only process/static caches for definitions, script programs, and texture resolution
also have no change-generation invalidation. Missing-file results can remain cached after
an asset appears. This matters when adding vehicle unload/reload, map switching, an editor,
or long-running content browsing. It is retained residency by design, not a demonstrated
current leak or measured out-of-memory failure. `Batch::SharedVertices` already shares the
large CPU vertex vectors across identical vehicles; copying `displayLists` is **not** evidence
that every AI instance deep-copies its whole mesh.

**Recommendation:** Account separately for unique decoded CPU bytes, render CPU bytes,
in-flight staging, and GPU bytes. Define pinned users plus byte-based eviction, asset-generation
keys, and owner-thread deferred GPU deletion before adding removal/hot reload. Drop obsolete
source payloads only when all required consumers have finished; preserve CPU data used for
picking/transparency or replace it with an appropriate compact representation. Avoid clearing
shared payloads while workers/users still depend on them. Use a const upload view when scaling
is unnecessary; test load/unload/reload and shared-resource lifetimes under a small budget.

### P5. Medium: DXT5 capability detection is invalid in the requested GL core profile

**Locations:** `src/RenderLoop.cpp:3007–3048` (`uploadCompressedDds`), context creation
(`GLFW_OPENGL_CORE_PROFILE`); `src/TextureAssetLoader.cpp` (DXT5 selection);
`Vehicle::ensureTexture`, `ensureMaterialTexture`, and `ensureAuxiliaryTexture`.

The custom DXT5 upload path calls `glGetString(GL_EXTENSIONS)`. That legacy extension-list
query is invalid in an OpenGL 3.3 core context; extensions must be enumerated with
`glGetStringi`. A conforming driver returns no list, causing the support check to fail even
if S3TC is available. Base/material/environment paths can then reread/decode the DDS to RGBA
synchronously on the render thread. The auxiliary path has no corresponding fallback, so a
DXT5 lightmap/nightmap/transmap/bumpmap can remain unbound. The invalid query also leaves a GL
error for later error checks. Driver/runtime impact was not measured here.

**Recommendation:** Build one context capability table using core-profile enumeration and
reuse it across all upload paths. Choose compressed versus decoded CPU preparation before
publication, and apply a consistent worker-side fallback to base and auxiliary textures.
Add a real core-context DXT5 fixture test with supported/unsupported capability cases, inspect
GL errors, and verify every material role rather than only base textures.

### P6. Medium: Collision-debug GPU data ignores same-sized geometry changes

**Locations:** `src/RenderLoop.cpp:5432–5484` (collision revision / wireframe rebuild);
`src/CoreRenderer.cpp:1078–1140` (`drawStaticPrimitives`).

The CPU wireframe rebuilds when `collisionDebugRevision()` changes, but its GPU cache uploads
again only when the vertex count differs. Replacing one streamed terrain tile with another
of the same topology/count changes positions without changing size; the overlay can keep
showing the old collision geometry. This undermines the debugging evidence used to validate
streaming even though the actual ODE geometry has changed.

**Recommendation:** Pass an explicit content revision/dirty signal to the static buffer
update, or invalidate it on collision revision changes. Test equal-count replacements, empty
sets, and subsequent repopulation. The global raw-pointer registry of caller-owned
`StaticPrimitiveBuffer` objects also needs an explicit lifetime contract or owning handles
before callers can be removed independently of renderer shutdown.

### P7. Medium: Rendered scenery and collision use different placement semantics

**Locations:** `src/MapCollisionBuilder.cpp` (`tile.sceneryObjects` loop,
`transformSceneryVertex`); `src/MapRenderer.cpp` (`resolveMapSceneryPlacement`,
`applyTerrainAlignment`, attached-object and spline-attachment loops).

Rendering composes ordinary/attached/spline-attached placements and can apply a terrain-normal
orientation matrix. Collision construction considers ordinary `sceneryObjects` only and
reconstructs an Euler transform without that alignment. An aligned ordinary object with an
authored collision mesh can consequently be rendered and collided at different orientations;
attached/repeated objects do not receive the same authored-collider processing. These are
confirmed implementation differences, not a claim of complete OMSI placement support.

**Recommendation:** Generate a shared CPU placed-scene representation with stable IDs,
resolved world transforms, asset/collision flags, and dependency revisions. Consume it from
both renderer and collision preparation, leaving winding/backend conversion explicit. Until
parity is implemented, document/diagnose unsupported collider placement types. Add fixtures
for sloped terrain, `[absheight]`, nested anchors, and spline repeaters that compare rendered
and collision world vertices, plus contact tests where support is promised.

## Maintainability findings

### M1. Medium: Runtime orchestration is too tightly coupled for safe feature growth

**Locations:** `src/RenderLoop.cpp` (`Vehicle` plus `RenderLoop`), `src/MapRenderer.cpp`
(construction, placement resolution, scripts, GPU upload, and drawing).

`RenderLoop.cpp` combines input/picking, vehicle resource conversion, material selection,
script scheduling, sound dispatch, animation, reflections, camera, benchmarking, capture,
and HUD work. The map constructor likewise mixes filesystem/config parsing, global attachment
resolution, script initialization, and GL lifetime management. Behavior such as the scheduler
and dynamic snapshot policy cannot be tested in isolation through the current renderer path
without pulling in platform/context dependencies.

**Recommendation:** Extract behavior-preserving seams incrementally: a pure scheduler,
CPU placed-scene/asset preparation, a vehicle runtime independent of draw submission, and a
context-owned renderer/resource interface. Give each an explicit owner/thread contract and
small fixture tests before moving responsibilities. Share material/texture-address and
coordinate-conversion policies where appropriate, but keep vehicle/map source conventions
explicit; a broad rewrite or premature ECS/backend migration is not required.

### M2. Medium: CMake recompiles shared implementation and duplicates target policy

**Location:** `CMakeLists.txt` (application/probe source lists, compiler options, trace setup).

Production `.cpp` files are listed independently in many executable targets. The baseline
build compiled `MapRoadGeometry.cpp` separately for OpenBus, the road probe, collision-builder
probe, and collision-streamer probe; `BusSimulation.cpp`, parsers, and trace support have
similar duplication. A common edit repeats compilation across consumers, and adding a source
or changing options requires maintaining several lists. Warning options are applied to the
application only. Some probes compiling `PerfTrace.cpp` do not use
`openbus_configure_perf_trace`, so they use the header's default disabled instrumentation
even in this trace-enabled build. Not every probe needs tracing, but the policy is implicit.

**Recommendation:** Introduce small STATIC/OBJECT libraries along existing CPU parser/map,
scripting, physics, and renderer boundaries, with shared INTERFACE compile policy. Keep
headless probes free of GL/audio dependencies rather than linking one monolithic application
library. Make intentional trace-on/off variants explicit, gate probes with `BUILD_TESTING`,
and compare clean/incremental build times and compile commands after the refactor. Add
Debug/sanitizer and trace-disabled coverage; do not infer a specific build-time speedup from
the number of source-list duplicates alone.

### M3. Low: The interactive loop accumulates an unused input history

**Location:** `src/main.cpp:359–378` (`pendingKeyEvents`);
`src/RenderLoop.cpp` (`beginFrame`, `consumeKeyEvents`).

Every frame's consumed key events are appended to `pendingKeyEvents`, but that vector has
no consumer, drain, or size bound. Symbol references confirm only its declaration and insert
statement. Script key dispatch already occurs in `beginFrame`, so this retains an increasing
event history for the lifetime of an interactive run without changing gameplay.

**Recommendation:** Remove the dead backlog, or give replay/telemetry a deliberate bounded
or persisted event sink. Specify exactly-once dispatch and tick association before reusing
the queue for a new scheduler; test a long stream of press/release events.

### Measurement priorities, not yet demonstrated defects

- Keep the focused renderer optimization plan in `RENDER_REVIEW.md` separate from this
  correctness/architecture review. Existing chunk rejection, shared geometry, cached texture
  binds, and prepared vehicle draw lists are real optimizations; do not recommend them as
  absent features.
- Establish fixed assets, resolution/framebuffer size, camera, physics/script settings,
  warm-up, and at least five repeated runs before changing reflection quality or static
  opaque chunk batching/instancing. Report CPU inclusive/self scope time separately from
  GPU query time, plus visible/submitted work and memory bytes. Do not add inclusive nested
  scopes together or interpret aggregate call counts as one frame.
- Profile script/runtime counts and transparent sorting/upload bytes before lower-priority
  variable lookup/allocation micro-optimizations. Preserve transparency ordering, material
  semantics, and independently evolving instance state.
- Consolidate helper toolchain/process/path setup after correcting the concrete safety and
  benchmark findings. Keep performance launches isolated from normal logs and caller state.

## Coverage and verification

### Current local baseline

- **Build:** Successful full Windows Release build in `build-ode`, using `clang-cl` / NMake
  with `OPENBUS_ENABLE_PERF_TRACE=ON`. This validates the current source/configuration, not a
  fresh-machine dependency installation or another platform.
- **CTest:** **23 passed, 1 failed** out of 24 registered probes. The current map-config
  duplicate-tile/entrypoint-remapping and mirrored-road geometry probe changes passed.
- **Failed probe:** `OpenBusMapCollisionStreamerProbe` reports
  `No [terrain] section found in ...\OpenBusMapCollisionStreamerProbe\map\tile_-1.map`.
  `tests/MapCollisionStreamerProbe.cpp:13–17` writes only a `[version]` record, while
  `src/MapConfigLoader.cpp:1059–1060` requires a `[terrain]` marker for a normal tile even
  without a binary sidecar. The test exits during its first load, before neighborhood and
  hysteresis assertions. **Medium-priority test defect:** make the fixture valid rather
  than weakening the parser, rerun it, and extend the count-only fixture with actual terrain/
  road/scenery colliders, revision checks, and tile-transition contact coverage. The fixture
  and production code were deliberately not changed during this review.
- CTest emitted missing `DartConfiguration.tcl` messages on passing probes too; those are
  separate integration warnings, not the cause of the streaming-probe failure.
- No editor diagnostics were reported during source review. No full renderer/context,
  Linux, sanitizer, new performance, or memory validation was run.

CMake currently registers 24 CTest probes:

- `OpenBusVariablesProbe`
- `OpenBusConfigurationParserProbe`
- `OpenBusMapConfigProbe`
- `OpenBusMapChronoTileProbe`
- `OpenBusMapSplineGeometryProbe`
- `OpenBusMapRoadGeometryProbe`
- `OpenBusMapCollisionBuilderProbe`
- `OpenBusMapCollisionStreamerProbe`
- `OpenBusMapSceneryPlacementProbe`
- `OpenBusMapSplineProfileProbe`
- `OpenBusTextureLoaderProbe`
- `OpenBusPhysicsProbe`
- `OpenBusStabilityProbe`
- `OpenBusPhysicsStressProbe`
- `OpenBusCameraMathProbe`
- `OpenBusViewpointProbe`
- `OpenBusRoadFeaturesProbe`
- `OpenBusO3DLoaderProbe`
- `OpenBusXLoaderProbe`
- `OpenBusModelConfigProbe`
- `OpenBusVehicleConfigProbe`
- `OpenBusBusConfigurationCollisionProbe`
- `OpenBusSoundEngineProbe`
- `OpenBusScriptTextureRuntimeProbe`

The probes cover parsing, variables, texture decoding, physics (including invalid-rate
rejection and deterministic two-/three-axle bump, steering, and braking stress), map
configuration and signed-radius spline endpoint geometry, camera math/viewpoints
(including coincident look-at), road features,
O3D/model/vehicle configuration (including oversized script-texture rejection), sound, and
script-texture/runtime behavior (including bounded native and Lua macro inputs). They do
not exercise a full OpenGL context, shader compilation, material binding, transparent depth
prepasses, environment-map sampling, or end-to-end clickable mesh rendering. Add headless or
fixture-based renderer tests where practical and retain manual screenshot validation for
context-dependent behavior.

The registered suite also lacks an isolated script-scheduler regression and model-JSON
serialization coverage (`ModelConfigJson.cpp` is not compiled into the model-config probe).
No checked-in `.github` CI configuration was found. Automate the current probes with valid
fixtures, then add a clean-build matrix for the advertised platforms/configurations and
context smoke tests where available. Parser success alone must not be used as evidence of
rendered/collision parity or acceptable frame latency.

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
- Identical vehicle batches share large CPU vertex vectors through `Batch::SharedVertices`
  and initially share GPU geometry. Mutable instance animation/material/texture state remains
  independent. Native OSC programs are already cached by source path.
- Scenery drawing rejects whole chunk bounds before scanning their instances. `CoreRenderer`
  caches texture bindings; walking texture units does not imply unconditional texture binds.
- Vehicle sound callbacks reference owner objects that remain alive while the vehicles are
  destroyed in the current `RenderLoop` teardown. There is no evidence of the previously
  suggested sound callback race/use-after-free on this path.
- Texture block/decoded-size arithmetic, OBJ batch reservation multiplication, and material
  occurrence-index expansion have bounds checks in their current implementations.
- Vehicle transparent batches are sorted back-to-front with an alpha-tested transmap depth
  prepass; this does not establish globally sorted map-scenery transparency.
- Bump-map data reaches a bounded derivative-based shader path; material batching is disabled
  for bump-mapped batches rather than silently omitting the effect.
- Environment-map texture loading and drawing are active; the shader epsilon clamp is a
  singularity guard, not evidence of an exploitable division defect.
- The former categorical BMP signed-left-shift undefined-behavior finding is withdrawn:
  signed integer promotion alone does not establish that defect in the project's required
  C++23 mode. Explicit unsigned assembly and a top-down BMP fixture remain sensible clarity/
  portability improvements, not a confirmed current UB defect on that evidence.
- Temporary light-variable snapshots, native OSC instruction traces, and their
  `OPENBUS_DEBUG_LIGHTS` profile/documentation toggle have been removed. Profiling and ODE
  wireframe controls remain.

Earlier claims of renderer races, missing COM cleanup, absent environment/bump mapping,
unbounded material indices, and unchecked DXT/OBJ allocation arithmetic should not be
reintroduced without new evidence from the current implementation.

## Follow-up opportunities

1. Restore a green baseline by correcting the collision-streamer fixture; isolate and test
  script time accounting. Address VAO/buffer cache coherence and core-profile DXT5 detection
  before trusting visual/performance comparisons. Repair or narrow advertised Linux support.
2. Add per-frame/tile-boundary timings, tick/drop counters, snapshot-copy/upload bytes, and
  unique CPU/GPU residency accounting. Repair the loading benchmark before using its results.
3. Implement revision-aware dynamic snapshots and explicit resource lifetimes/budgets. Extract
  CPU scene/asset preparation and shared placement semantics, then introduce bounded async
  collision preparation and GPU tile residency with owner-thread publication.
4. Refactor CMake/common compile policy and orchestration seams incrementally; add isolated
  scheduler/serializer/resource tests and a platform/configuration validation matrix.
5. Only then benchmark reflection quality and chunk-local static opaque batching/instancing
  against repeatable workloads, with screenshot/contact checks to prevent semantic regressions.
6. Extend safe OMSI macro implementations only as their route, timetable, passenger, ticket,
  depot, arrival-board, and world-height backing data models become available.
