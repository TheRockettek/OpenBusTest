# OpenBus Code Review

**Review date:** 2026-10-25
**Scope:** First-party C++ architecture and hot paths: application timing/input, physics,
map parsing/placement/collision/rendering, vehicle and texture loading, script runtime,
renderer state/resource ownership, CMake/presets, regression probes, helper scripts, and
support documentation. The current working-tree changes were included; the source inventory
contains 151 first-party C++ source/header files. Third-party internals and generated source
were excluded. This is not an exhaustive audit of every asset format or a certification of
OMSI compatibility.

## Executive summary

This review records unresolved implementation defects, compatibility limitations, and
architectural/performance risks. The Windows Clang Release full-suite validation passed all
**64 of 64** configured CTest probes serially, including serializer and benchmark-statistics
probes.

The highest-priority remaining work is:

| Priority | Area | Main risk / next action |
| --- | --- | --- |
| Medium | Collision-streaming latency | Terrain/road/scenery mesh preparation is asynchronous, but ODE insertion/removal remains on the simulation thread and may still stall; measure publication and contact readiness. |
| Medium | Texture fallback cost | DXT5 fallback rereads and decodes textures on the render thread; dedicated fallback coverage is incomplete. |
| Medium | Memory residency | Asset caches retain CPU payloads and GPU resources without a byte-based residency policy. |
| Medium | Map scalability | Newly active tiles are loaded synchronously and retained; activation scans retained scenery/script state, while global parsed map/placement data remains CPU-resident. |
| Medium | Build verification | Clean Linux validation remains outstanding. |

Findings below distinguish confirmed defects from architectural scaling risks. The full
configured CTest suite includes material-rendering and compute-culling GPU probes, but no
full application/map-streaming context test, Linux build, sanitizer run, fresh benchmark,
GPU timing study, or memory profile was performed. No speedup percentages or measured hitch
durations are claimed.

## Current issues

### High priority

#### 1. Non-Windows Lua linkage has not been validated

**Location:** `CMakeLists.txt` (`OpenBusLua` imported target and post-build copy).

Non-Windows builds locate a system `lua5.1` or `lua51` library, but no Linux
configure/build/test was run in this review. The native Lua headers/library pairing and the
advertised Linux build path therefore remain unverified.

**Follow-up:** Validate the native Lua headers/library pairing with a clean Linux configure,
build, and CTest run.

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

### P2. Medium: Synchronous tile activation and retained map state limit larger-map scalability

**Locations:** `src/render/map/MapRenderer.cpp` (`updateStreaming`);
`src/render/map/MapSceneLoader.cpp` (`loadActiveTiles`); `MapRenderer::Impl` resource vectors.

The constructor loads the initial active tiles synchronously. Later center-tile changes also call
`loadActiveTiles` synchronously for newly encountered tiles. Already loaded tile content is
retained rather than rebuilt or evicted, so memory can grow as more of the map is explored.
`updateStreaming` then scans all retained scenery chunks and all scenery script states to build
active lists, even though only active owners are added to the lists. Global parsed map records
remain resident for cross-tile attachment resolution. Collision CPU preparation is asynchronous,
but ODE publication/removal remains on the simulation thread. These are scaling risks; no current
boundary hitch duration or memory-growth rate was measured.

**Recommendation:** Keep parsed map data available for dependency resolution, but add per-tile
indexes for retained scenery/script state so activation work scales with entering/leaving tiles
rather than the explored map. Measure boundary-frame latency and CPU/GPU residency during a
repeatable route before undertaking tile-resource eviction or a broader streaming rewrite. Add
a GL-context integration smoke test and in-game visual/contact validation.

`MapRenderer::updateScripts()` now initializes and ticks scenery runtimes only when their owner
tile is active; each placement keeps independent mutable state, and native compiled programs are
already cached. Camera-frustum rejection does not pause scripts within active tiles: a script
can control `[visible]` or shared variables and needs a defined sleep/wake policy before
camera-based suspension is safe. Attribute cost by active runtime count and script before
changing cadence or timer semantics.

### P3. Medium: Asset caches have no explicit byte budget or reload generation

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

### P4. Medium: DXT5 fallback decoding runs on the render thread

**Locations:** `src/app/vehicle/VehicleTextures.cpp` (compressed-texture fallback);
vehicle base/material/environment/auxiliary texture upload paths.

When compressed upload is unavailable, affected paths reread and decode the texture on the
render thread. This avoids a failed load but can add synchronous work during texture upload.
There is no dedicated supported/unsupported DXT5 upload fixture or driver performance
comparison.

**Recommendation:** Add focused fallback tests and measure the render-thread decode cost.
Reuse decoded payloads or move fallback decoding off-thread where ownership and upload ordering
permit.

### P5. Medium: Rendered scenery and collision use different placement semantics

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

### M1. Medium: Runtime orchestration still lacks isolated scene seams

**Locations:** `src/app/RenderLoop*.cpp`, `src/app/vehicle/Vehicle*.cpp`,
`src/render/map/MapRenderer*.cpp`, and `src/render/map/MapScene*.cpp`.

The former monolithic translation units have been split across render-loop, vehicle, and map
files, reducing file-level coupling. The application still owns cross-cutting input, script
scheduling, simulation, rendering, sound, capture, and lifecycle coordination, while map
scene preparation, placement resolution, script setup, and GL publication cross several
components. Vehicle dynamic-texture refresh skips unchanged snapshots, but changed snapshots
are not coalesced across batches and views.

**Recommendation:** Extract behavior-preserving CPU scene-preparation and context-owned
renderer/resource interfaces with explicit ownership/thread contracts and focused tests.
Coalesce changed texture snapshots where it is safe. Avoid a broad rewrite or premature
ECS/backend migration.

## Validation scope

The recorded `build-ninja` Windows Clang Release build succeeded, and
`ctest --test-dir build-ninja --output-on-failure -j 1` passed **64 of 64** tests. This does
not include a Linux build, sanitizer run, fresh dependency setup, loading/render benchmark, or
memory profile. The focused GPU probes do not validate full application/map-streaming
behavior, live in-game contact/render parity, or acceptable frame latency.
