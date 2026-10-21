# OpenBus Code Review

**Review date:** 2026-10-21
**Scope:** First-party C++ architecture and hot paths: application timing/input, physics,
map parsing/placement/collision/rendering, vehicle and texture loading, script runtime,
renderer state/resource ownership, CMake/presets, regression probes, helper scripts, and
support documentation. The current working-tree changes were included; the source inventory
contains 142 first-party C++ source/header files. Third-party internals and generated source
were excluded. This is not an exhaustive audit of every asset format or a certification of
OMSI compatibility.

## Executive summary

The previously documented correctness/resource-handling fixes remain covered: optional AI
configuration on Windows, script-texture bounds, the exterior camera's zero-distance
singularity, and invalid physics rates. The current Windows Release build succeeds and all
**50 of 50 CTest probes pass**. Recent CMake module-library refactoring and focused material,
sorting, and texture-selection probes are included in this baseline.

The previously highest-priority process-safety defect is fixed: the launch helpers and loading
benchmark no longer terminate processes by image name, and they refuse to proceed while
OpenBus is already running. `run.bat` now performs its documented build-and-launch flow. Script
catch-up overflow is now explicitly bounded, accounted for, diagnosed, and regression-tested.

The most important remaining follow-ups are not just draw-call optimizations:

| Priority | Area | Main risk / next action |
| --- | --- | --- |
| Medium | Collision-streaming latency | Terrain/road/scenery mesh preparation is asynchronous, but ODE insertion/removal remains on the simulation thread and may still stall; measure publication and contact readiness. |
| Medium | Renderer correctness | Core-profile DXT5 capability detection still uses an invalid extension query. |
| Medium | Memory / hot-path copies | Vehicle dynamic textures still copy unchanged snapshots before revision checks; asset caches retain CPU payloads and GPU resources without a byte-based residency policy. |
| Medium | Map scalability | A shared-context worker rebuilds the full replacement geometry window on tile crossings; startup is synchronous, overlapping tiles are rebuilt, and global parsed map/placement data remains CPU-resident. |
| Medium | Build/docs verification | The production sources now use module libraries, but scheduler/serializer tests and clean Linux validation remain absent; several helper/setup instructions are still stale. |

Findings below distinguish confirmed defects from architectural scaling risks. The full
configured CTest suite includes material-rendering and compute-culling GPU probes, but no
full application/map-streaming context test, Linux build, sanitizer run, fresh benchmark,
GPU timing study, or memory profile was performed. No speedup percentages or measured hitch
durations are claimed. Active-window geometry residency, background renderer preparation, and
geometry-constrained background collision preparation are implemented, but no in-game
visual/performance/contact comparison has yet been run.

## Findings and remediation status

### Medium priority

#### 1. Optional AI configuration paths alias on Windows — resolved

**Original defect:** `openbus::getEnvironment()` returns a pointer into one thread-local `std::string`, and its
documented lifetime ends at the next call on that thread. `main()` retains the AI bus-config
pointer, calls `getEnvironment()` again for the AI model-config path, and only then consumes
both pointers. The second lookup overwrites the string, so the bus-config pointer now refers
to the model-config value. Supplying the paired AI environment variables can therefore send
the wrong path to bus-config loading.

**Locations:** `src/core/Environment.h` (`getEnvironment`); `src/app/main.cpp` (AI environment lookups
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

**Locations:** `src/config/ModelConfigLoader.cpp` (`[scripttexture]` parser); `src/scripting/ScriptRuntime.cpp`
(`configuredScriptTexture`, `resizeScriptTexture`, `setPixel`, `safeScriptTexturePixel`, and
`safeScriptTextureRect`).

**Resolution:** Shared checked limits live in `src/config/ScriptTextureLimits.h`; the model parser,
runtime allocations, file-loaded surfaces, and text textures consume them. Regression probes
cover oversized configuration, non-representable native/Lua coordinates and indexes, and an
oversized rectangle.

#### 3. Exterior camera distance can reach a singular zero — resolved

**Original defect:** Right-mouse camera adjustment clamped `cameraDistance_` to `[0, 80]`. At zero, the exterior
camera computes identical eye and target positions. `lookAt()` divides the forward vector by
its length without handling a zero length, creating non-finite view-matrix values.

**Locations:** `src/app/RenderLoop.cpp` (right-mouse camera-distance clamp and exterior-camera
eye/target calculation); `src/render/CameraMath.cpp` (`lookAt`).

**Resolution:** Right-mouse exterior zoom now clamps distance to at least 0.1, and `lookAt()`
uses a finite fallback forward direction when eye and target coincide. `OpenBusCameraMathProbe`
checks that a coincident eye/target produces a finite matrix.

### Low priority

#### 4. Physics-rate validation accepts NaN and positive infinity — resolved

`BusSimulation::Impl` now rejects non-finite or non-positive rates before taking the reciprocal,
then verifies that the resulting fixed step is finite and positive. This also rejects positive
subnormal rates whose reciprocal overflows.

**Location:** `src/physics/BusSimulation.cpp` (`BusSimulation::Impl` constructor).

**Resolution:** Constructor validation and `OpenBusPhysicsProbe` cover zero, negative, NaN,
positive infinity, and a denormal rate. No arbitrary maximum frequency is imposed; callers
remain responsible for selecting a practical rate.

## Current review findings

### High priority

#### 1. Platform-specific Lua linkage — Linux linkage is now conditional; Linux is unverified

**Location:** `CMakeLists.txt` (`OpenBusLua` imported target and post-build copy).

The earlier unconditional Windows import has been corrected. Windows uses the bundled
`lua51.lib`/`lua51.dll`; non-Windows builds now locate a system `lua5.1` or `lua51` library,
and the runtime-copy step is Windows-only. This removes the confirmed Windows-artifact link
failure, but no Linux configure/build/test was run in this review, so advertised Linux support
is not certified.

**Follow-up:** Validate the native Lua headers/library pairing with a clean Linux configure,
build, and CTest run.

#### 2. Resolved (2026-10-21): Script catch-up behavior is bounded and observable

**Locations:** `src/app/ScriptTickScheduler.h`, `src/app/RenderLoopSimulation.cpp`,
`tests/ScriptSchedulerProbe.cpp`, and `README.md` under `OPENBUS_SCRIPT_HZ`.

The scheduler now lives in a context-free helper. It clamps elapsed frame time to 250 ms,
executes at most eight fixed ticks per update, retains the fractional remainder, and reports
whole excess ticks and dropped seconds. `RenderLoop` aggregates and logs executed/dropped tick
counts once per second when drops occur. Invalid or non-finite `OPENBUS_SCRIPT_HZ` values are
logged and fall back to render-rate scripts. The README documents the deliberate drop policy,
so overload cannot silently masquerade as full fixed-rate catch-up.

**Validation:** `OpenBusScriptSchedulerProbe` covers 30 FPS/60 Hz, 120 FPS/60 Hz, zero elapsed
time, a 250 ms hitch, a high script rate, elapsed-time clamping, and invalid tick durations.
Input-event ordering and behavior of both script backends remain outside this scheduler probe.

### Medium priority

#### 3. Renderer cache finding — the reported texture-quad path is fixed

**Locations:** `src/render/CoreRenderer.cpp` (`drawTextureQuad` and renderer binding cache);
raw uploads remain in `src/app/vehicle/VehicleDraw.cpp` and
`src/app/vehicle/VehicleMaterials.cpp`.

The previous report that `drawTextureQuad` left the cached VAO and array-buffer IDs stale is
withdrawn: the current implementation updates both cache values after binding its quad VAO
and buffer. Vehicle raw buffer uploads remain, but `GL_ARRAY_BUFFER` is not VAO state and
vertex attribute pointers retain their captured buffer; no equivalent draw regression was
identified in the current paths. A GL call-recorder regression was not run.

**Status:** No active finding from the previously identified HUD/texture-quad sequence. Keep
state-cache ownership in view when adding additional contexts or render backends.

#### 4. Model-config JSON does not escape every control character

**Location:** `src/config/ModelConfigJson.cpp` (`writeJsonString`).

Quotes, backslashes, newline, carriage return, and tab are escaped, but other bytes below
`0x20` are written raw. A configuration string containing one of those bytes produces invalid
JSON.

**Suggested fix:** Encode every remaining control byte as `\u00XX` and add serializer tests
covering the full control-byte range.

#### 5. Resolved (2026-10-21): Launch helpers no longer terminate every `OpenBus.exe`

**Locations:** `run.bat`, `screenshot.bat`, and `benchmark_loading.ps1`.

The image-wide `taskkill` commands have been removed. `run.bat` and `screenshot.bat` now check
for an existing `OpenBus.exe` and stop with a message rather than terminating it;
`benchmark_loading.ps1` checks for existing processes and throws with their PIDs before
removing logs or starting a run. `run.bat` now changes to its own directory and performs the
documented Ninja configure/build/launch.

**Validation:** Static inspection confirmed that none of these three helpers contains a
process-termination command. The launch paths were not executed because they configure/build
and open the interactive application.

#### 6. Loading benchmark is out of sync with the runtime and can misreport results

**Locations:** `benchmark_loading.ps1` (executable path, per-run log removal, environment setup
and cleanup, `Get-MedianMs`); `CMakePresets.json` (NMake generator); `src/app/RenderLoop.cpp`
(current load-completion logging).

The benchmark expects `build-ode\Release\OpenBus.exe`, but the documented Windows preset uses
single-configuration NMake, which writes `build-ode\OpenBus.exe`; the current workspace has the
latter and not the former. Fixing that path alone is insufficient: the script requires an
`All textures loaded.` log marker that the current source does not emit, and sets
`OPENBUS_VEHICLE`, which the current application no longer reads.

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

#### 7. Windows setup instructions omit the required OpenAL vcpkg port

**Locations:** `CMakeLists.txt` (`find_package(OpenAL CONFIG REQUIRED)`);
`README.md` (Windows vcpkg and Debian/Ubuntu prerequisites).

The Debian/Ubuntu package list includes `libopenal-dev`, but the Windows vcpkg command installs
ODE, GLFW, and zlib without OpenAL. Since configure requires `OpenAL::OpenAL`, a clean Windows
vcpkg setup following the README may fail before building.

**Suggested fix:** Add the OpenAL vcpkg port to the Windows setup instructions and verify from
a clean configure.

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

#### 9. Some build helpers assume the caller's working directory is the repository root

**Location:** `profile.bat` (`cmake -S .` and relative build paths).

`profile.bat` calls `cmake -S .` without first switching to `%~dp0`. Invoking it by absolute
path from another current directory makes CMake configure the wrong source tree or fail.
`run.bat` and `screenshot.bat` now switch to their own directory.

**Suggested fix:** `pushd` to `%~dp0` before project-relative commands and `popd` on all exit
paths.

### Low priority

#### 10. `stb` is fetched from a mutable branch

**Location:** `CMakeLists.txt` (`FetchContent_Declare(stb)`).

`GIT_TAG master` allows identical source revisions to fetch different dependency contents over
time, reducing build reproducibility.

**Suggested fix:** Pin a reviewed release or immutable commit hash and update it deliberately.

#### 11. README examples and runtime instructions are stale

**Locations:** `README.md`, `Docs/OMSI_CFG_GENERAL_REFERENCE.md`,
`Docs/OMSI2_MAP_FORMAT_REFERENCE.md`, and the current helper scripts.

The README's `run.bat` description now matches its configure/build/launch behavior and MAN DL05
configuration. It still documents a missing `convert_textures.ps1` and claims the
render-benchmark window is hidden even though `RenderLoop` requests `GLFW_VISIBLE=GLFW_TRUE`.
The recent status-documentation updates correct several earlier feature-status contradictions;
these remaining tool instructions are directly reproducible.

**Suggested fix:** Remove or implement the missing conversion command and report benchmark
window visibility accurately. Keep supported, parsed-only, and visually/contact-validated
features distinct.

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

**Locations:** `src/scripting/ScriptRuntime.cpp` (`safeNumericLookup`, `safeMissingIndex`,
`safeStringLookup`, `safeGetHeightAbovePoint`, `safeTicketName`, `safeArrivalString`, and
`safeArrivalTime`; system-macro registrations).

Scripts that depend on those data sources will not yet reproduce OMSI behavior. Keep these
fallbacks explicit and add focused tests when the underlying data models are implemented;
do not substitute guessed values.

### Approximate script text helpers

Script texture drawing and text-texture generation are present, but `getfontindex` returns a
constant and `textlength` uses a fixed-width byte-based estimate rather than font metrics.
These are compatibility limitations, not renderer safety defects.

**Location:** `src/scripting/ScriptRuntime.cpp` (`safeFontIndex`, `safeTextLength`).

## Architectural constraint

### `Variables` is currently single-thread owned

`Variables` stores numeric and string state in unsynchronized maps. Current frame updates,
script execution, and rendering access that state on the main/render thread; asset workers
decode assets without accessing `Variables`. This is not a demonstrated current data race,
but moving these operations to worker threads would require synchronization or a snapshot/
message-passing design.

## Performance and future-change findings

### P1. High: Collision preparation is asynchronous, but ODE publication remains synchronous

**Locations:** `src/map/MapCollisionStreamer.cpp:51–89`; `src/map/MapCollisionBuilder.cpp`
(`buildMapRoadCollisionImpl`, `readSceneryCollisionMeshes`); `src/app/main.cpp`
(streamer updates between physics and drawing in both loops).

`MapCollisionStreamer` now schedules terrain reads, tile/config parsing, and road/scenery mesh
construction on a CPU worker. Results are accepted only while their tile still has resident
render geometry and is in the vehicle's load neighborhood. ODE geometry creation/removal stays
on the simulation thread, with at most one completed tile published per update. The initial
collision neighborhood is explicitly warmed before gameplay, so startup still waits. The
builder's profile/terrain caches are recreated for each selected tile. Each ordinary scenery
placement rereads its SCO/model configuration and parses declared collision O3Ds; repeated
placements do not share an asset-level collider cache. Rendering loads its own source data.

The one-tile load neighborhood / two-tile unload hysteresis limits active collision residency,
and collision is restricted to tiles with uploaded render geometry. These bounds limit residency,
not all frame-thread work: ODE publication/removal can still be expensive, and one-tile lookahead
cannot guarantee readiness for fast travel or worker delays. If preparation falls behind,
temporary contact gaps are possible. This is an architectural risk, not a measured hitch duration.

**Recommendation:** Measure build time, ODE publication cost, and p95/p99 frame time. Add
shared immutable asset-local collision caches and explicit readiness for the bus's reachable/
swept volume, not just its current tile. Define safe behavior for teleports and worker failures.
Test travel in both directions, re-entry, negative coordinates, rapid center changes, and contact
continuity; consider moving the entire ODE world to a dedicated owner thread if publication is
still costly.

### P2. Medium: Full-window background rebuilds still limit larger-map scalability

**Locations:** `src/render/map/MapRenderer.cpp` (`MapRenderer` constructor, `updateStreaming`, `draw`,
`updateScripts`); `MapRenderer::Impl` resource vectors.

At startup the constructor synchronously builds/uploads terrain, scenery, and roads for the
two-tile Chebyshev window. Later center-tile changes submit a candidate renderer to a worker with
a shared GLFW/OpenGL context; the old window remains drawable until the full replacement window
is ready, then its resources are released on the primary context. The worker also performs GPU
uploads, so driver/GPU contention is possible. If shared-context setup fails, the fallback is
synchronous. Overlapping tiles are rebuilt rather than reused. Newly encountered scenery runtimes
are initialized on the main script thread at a cap of 32 per frame. Collision CPU preparation is
also asynchronous, but ODE publication remains on the simulation thread. Startup parses tile
records once into the cached CPU-side representation/global placement index, which remains
resident for cross-tile attachment resolution. Road staging geometry and terrain grids coexist
during construction, increasing peak CPU memory.

**Recommendation:** Replace full-window reconstruction with per-tile entering/leaving realization
from the existing parsed cache. Preserve cross-tile attachment dependencies and script state while
evicting tile-owned resources. Measure startup/first-usable-frame time, peak CPU/GPU bytes, GPU
contention, ODE publication, and boundary-frame latency. Add a GL-context integration smoke test
and in-game visual/contact validation before claiming smooth streaming.

`MapRenderer::updateScripts()` now initializes and ticks scenery runtimes only when their owner
tile is active; each placement keeps independent mutable state, and native compiled programs are
already cached. Camera-frustum rejection does not pause scripts within active tiles: a script
can control `[visible]` or shared variables and needs a defined sleep/wake policy before
camera-based suspension is safe. Attribute cost by active runtime count and script before
changing cadence or timer semantics.

### P3. Medium: Dynamic texture revisions are checked after full pixel copies

**Locations:** `src/scripting/ScriptTextures.cpp` (`copy*TextureIfChanged`);
`src/app/vehicle/VehicleTextures.cpp` (`Vehicle::updateFreeTexture`);
`src/render/map/MapRendererDraw.cpp` (`refreshSceneryTexture`).

Revision-aware copy APIs now avoid copying pixels for unchanged snapshots, and scenery drawing
uses them after its first upload; script auxiliary textures do as well. The vehicle
`updateFreeTexture()` path still calls the unconditional copy APIs for text/script textures,
then compares the copied revision with its GPU cache. Since that refresh can occur per batch
and recur in depth, transparent, and reflection passes, unchanged vehicle snapshots can still
allocate and copy a full pixel vector before skipping the upload. A permitted surface can
contain 16 MiB, so this remaining path can produce avoidable CPU bandwidth.

**Recommendation:** Use the existing copy-if-changed APIs in the vehicle refresh path and
resolve changed slots once per runtime/frame for reuse across views/batches. Preserve filtering
and per-material address modes (potentially with separate sampler state), and keep GL updates
on its owner thread. Add byte-copy/upload counters and a regression proving zero pixel copies/
uploads for unchanged slots and one content refresh after a revision change, including
multi-batch/reflection and resize cases.

### P4. Medium: Asset caches have no explicit byte budget or reload generation

**Locations:** `src/render/AssetRequestManager.cpp` (`requestObj`, `loadTextureRequest`, tracking,
destructor); `src/app/vehicle/VehicleDefinition.cpp`, `src/app/vehicle/VehicleTextures.cpp`;
`src/scripting/ScriptRuntimeImpl.cpp` (`cachedNativeProgram`).

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

**Locations:** `src/app/vehicle/VehicleTextures.cpp` (`uploadCompressedDds`); context creation
(`GLFW_OPENGL_CORE_PROFILE`); `src/assets/TextureAssetLoader.cpp` (DXT5 selection);
vehicle base/material/auxiliary texture upload paths.

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

### P6. Resolved: Collision-debug GPU data ignored same-sized geometry changes

**Locations:** `src/app/RenderLoopDraw.cpp` (collision revision / wireframe rebuild);
`src/render/CoreRenderer.cpp` (`drawStaticPrimitives`).

The CPU wireframe rebuilds when `collisionDebugRevision()` changes. The GPU cache now receives
that revision and reuploads when either the vertex count or revision changes, so replacing a
streamed tile with same-sized geometry cannot leave stale orange lines. `OpenBusStaticPrimitiveBufferProbe`
covers unchanged contents, same-sized revision changes, and vertex-count changes. The buffer
is skipped for empty vertex sets and refreshed if geometry is later repopulated.

**Remaining note:** The global raw-pointer registry of caller-owned `StaticPrimitiveBuffer`
objects still needs an explicit lifetime contract or owning handles before callers can be
removed independently of renderer shutdown.

### P7. Medium: Rendered scenery and collision use different placement semantics

**Locations:** `src/map/MapCollisionBuilder.cpp` (`tile.sceneryObjects` loop,
`transformSceneryVertex`); `src/render/map/MapSceneTiles.cpp`
(`resolveMapSceneryPlacement`, `applyTerrainAlignment`, attached-object and spline-attachment
loops).

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

### M1. Medium: Runtime orchestration still lacks isolated scheduler and scene seams

**Locations:** `src/app/RenderLoop*.cpp`, `src/app/vehicle/Vehicle*.cpp`,
`src/render/map/MapRenderer*.cpp`, and `src/render/map/MapScene*.cpp`.

The former monolithic translation units have been split across render-loop, vehicle, and map
files, reducing file-level coupling. The application still owns cross-cutting input, script
scheduling, simulation, rendering, sound, capture, and lifecycle coordination, while map
scene preparation, placement resolution, script setup, and GL publication cross several
components. In particular, `updateScripts()` remains a `RenderLoop` member and has no pure
scheduler test; dynamic snapshot policies are also not isolated behind a context-free seam.

**Recommendation:** Continue extracting behavior-preserving seams incrementally: a pure
scheduler, CPU placed-scene/asset preparation, and context-owned renderer/resource interfaces.
Give each an explicit owner/thread contract and small fixture tests. Keep vehicle/map source
conventions explicit; a broad rewrite or premature ECS/backend migration is not required.

### M2. Resolved: Production CMake targets now share module libraries and compile policy

**Location:** `src/CMakeLists.txt` (`openbus_add_module` and production target dependencies).

Production implementation is now grouped into static module libraries (core, config, assets,
physics, map, scripting, audio, input, and render), and `OpenBus` links those modules rather
than recompiling every implementation source in its own source list. The shared module helper
applies compiler policy consistently. The former finding about duplicated production source
lists and app-only warnings is therefore stale.

**Remaining verification:** No before/after clean or incremental build-time comparison was
performed, and no sanitizer configuration was tested. Keep headless probes independent of
unneeded GL/audio dependencies.

### M3. Low: The interactive loop accumulates an unused input history

**Location:** `src/app/main.cpp` (`pendingKeyEvents` and frame-key event collection).

Every frame's consumed key events are appended to `pendingKeyEvents`, but current symbol
references show no consumer, drain, or size bound. Script key dispatch already occurs through
the frame input path, so this retains an increasing event history for the lifetime of an
interactive run without changing gameplay.

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
  with `OPENBUS_ENABLE_PERF_TRACE=ON`. The NMake environment was initialized through
  `windows_clang_env.bat`. This validates the current source/configuration, not a fresh-machine
  dependency installation or another platform.
- **CTest:** **50 of 50 passed** using `ctest --test-dir build-ode --output-on-failure -j 8`.
  This includes `OpenBusMapCollisionStreamerProbe`; the previously documented invalid fixture
  is no longer a current failure.
- GPU-backed `OpenBusMaterialRenderingProbe` and `OpenBusComputeCullingGpuProbe` passed, so the
  old assertion that no OpenGL context tests existed is stale. These focused probes do not
  constitute a full application/map-streaming context test or live in-game contact/render
  parity validation.
- No Linux build, sanitizer run, fresh dependency setup, loading/render benchmark, or memory
  profile was performed. No performance improvement or measured hitch duration is claimed.

The 50 registered tests cover perf-trace behavior, variables and configuration parsing,
scenery/vehicle/human configuration, input bindings, texture and model decoding, physics and
contact/stability/stress, map configuration/geometry/collision/traffic/placement, sound and
script runtimes, camera/viewpoint/reflection settings, asset requests, compute culling, GPU
material rendering, vehicle draw/material selection, weather, texture address modes, and
static primitive buffer revisions. Recent additions include
`OpenBusAssetRequestManagerProbe`, `OpenBusMaterialRenderingProbe`,
`OpenBusVehicleDrawSortingProbe`, `OpenBusVehicleMaterialSelectionProbe`,
`OpenBusWeatherEffectsProbe`, `OpenBusTextureAddressModeProbe`, and
`OpenBusScriptSchedulerProbe`.

No isolated script-scheduler regression or model-JSON serialization test is registered.
`ModelConfigJson.cpp` is now compiled into `openbus_config`, but its serializer has no direct
regression coverage. Parser and focused GPU probe success must not be used as evidence of full
OMSI parity, streamed-map contact continuity, or acceptable frame latency.

## Verified fixes and stale findings

- `src/config/ConfigurationParser.cpp` strips a UTF-8 BOM from the first input line. The regression
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
  `drawTextureQuad` now also updates the VAO/array-buffer cache, correcting the previously
  reported model → HUD → model cache mismatch.
- Production CMake sources are grouped into static module libraries with shared project
  compile settings; the previous duplicated-source-list finding is stale.
- CMake now selects bundled Lua artifacts only on Windows and locates a native Lua 5.1 library
  elsewhere; Linux behavior still requires independent validation.
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

1. Add model-JSON serialization tests.
2. Correct the loading benchmark's executable/readiness assumptions and ensure it preserves
  caller-owned logs and environment variables.
3. Validate the native Linux Lua/build path and add clean-build platform coverage. Keep the
  core-profile DXT5 extension query on the renderer correctness list.
4. Add per-frame/tile-boundary timings, tick/drop counters, snapshot-copy/upload bytes, and
  unique CPU/GPU residency accounting.
5. Implement revision-aware dynamic snapshots and explicit resource lifetimes/budgets. Extract
  CPU scene/asset preparation and shared placement semantics, then measure asynchronous
  collision preparation and GPU tile residency with owner-thread publication.
6. Benchmark reflection quality and chunk-local static opaque batching/instancing against
  repeatable workloads, with screenshot/contact checks to prevent semantic regressions.
7. Extend safe OMSI macro implementations only as their route, timetable, passenger, ticket,
  depot, arrival-board, and world-height backing data models become available.
