# OpenBus

OpenBus is a C++ bus simulator prototype built on the
[Open Dynamics Engine (ODE)](https://www.ode.org/). It combines fixed-step bus
physics with OpenGL vehicle rendering and partial OMSI map rendering. This is
a prototype, not a fully OMSI-compatible simulator.

## Compatibility and validation status

This table summarizes the current implementation, not complete OMSI parity.
Parser probes establish only the parser behavior they exercise; they do not
prove end-to-end rendering or runtime semantics.

| Feature | Current status | Implementation | Regression evidence |
| --- | --- | --- | --- |
| Physics and wheel simulation | Fixed-step ODE and per-wheel simulation are active; the deterministic bump/steering/braking stress probe covers two- and three-axle fixtures. Broader tuning remains incomplete. | [`BusSimulation.cpp`](src/physics/BusSimulation.cpp) | [`BusSimulationSteeringProbe.cpp`](tests/BusSimulationSteeringProbe.cpp), [`BusSimulationStabilityProbe.cpp`](tests/BusSimulationStabilityProbe.cpp), [`BusSimulationStressProbe.cpp`](tests/BusSimulationStressProbe.cpp) |
| Vehicle/model loading | Core MAN DL05 and E400 configuration paths are implemented; broad OMSI configuration coverage remains partial. | [`VehicleConfigLoader.cpp`](src/config/VehicleConfigLoader.cpp), [`ModelConfigLoader.cpp`](src/config/ModelConfigLoader.cpp), [`RenderLoop.cpp`](src/app/RenderLoop.cpp) | [`VehicleConfigProbe.cpp`](tests/VehicleConfigProbe.cpp), [`ModelConfigProbe.cpp`](tests/ModelConfigProbe.cpp); visual procedure: [screenshot plan](Docs/SCREENSHOT_VALIDATION_PLAN.md) |
| Texture decoding and materials | Texture formats and selected material behaviors have focused parser/GPU probes, including alpha mode 2, bump maps, environment-map sampling, transparent sorting, CTC texture replacement, and array-layer bounds. Representative-asset parity and difficult overlap cases remain incomplete. | [`TextureLoader.cpp`](src/assets/TextureLoader.cpp), [`CoreRenderer.cpp`](src/render/CoreRenderer.cpp) | [`TextureLoaderProbe.cpp`](tests/TextureLoaderProbe.cpp), [`TextureAddressModeProbe.cpp`](tests/TextureAddressModeProbe.cpp), [`MaterialRenderingProbe.cpp`](tests/MaterialRenderingProbe.cpp), [`VehicleDrawSortingProbe.cpp`](tests/VehicleDrawSortingProbe.cpp), [`VehicleMaterialSelectionProbe.cpp`](tests/VehicleMaterialSelectionProbe.cpp), [`AssetRequestManagerProbe.cpp`](tests/AssetRequestManagerProbe.cpp); visual gaps: [incomplete visual checks](Docs/SCREENSHOT_VALIDATION_PLAN.md#incomplete-feature-visual-checks) |
| Map, terrain, scenery, and roads | Partial: only active-window terrain, scenery meshes, and road geometry are built/uploaded; on tile changes a shared-context worker builds the replacement window while the previous one remains drawable. Collision mesh preparation runs in a CPU worker and ODE publication stays on the simulation thread, limited to one tile per update. Startup window creation and initial collision warm-up remain synchronous; if a shared GL context cannot be created, renderer loading falls back to synchronous work. Startup still parses map tile text once to build the global attachment index. Static text/binary/MSZIP `.x` meshes and distance-selected scenery LODs now have runtime paths; full OMSI attachment/profile semantics and end-to-end visual/contact validation remain incomplete. | [`MapConfigLoader.cpp`](src/map/MapConfigLoader.cpp), [`MapSplineGeometry.cpp`](src/map/MapSplineGeometry.cpp), [`MapRoadGeometry.cpp`](src/map/MapRoadGeometry.cpp), [`MapCollisionBuilder.cpp`](src/map/MapCollisionBuilder.cpp), [`MapCollisionStreamer.cpp`](src/map/MapCollisionStreamer.cpp), [`MapRenderer.cpp`](src/render/map/MapRenderer.cpp), [`SceneryObjectConfigLoader.cpp`](src/config/SceneryObjectConfigLoader.cpp) | [`MapConfigLoaderProbe.cpp`](tests/MapConfigLoaderProbe.cpp), [`MapCollisionBuilderProbe.cpp`](tests/MapCollisionBuilderProbe.cpp), [`MapCollisionStreamerProbe.cpp`](tests/MapCollisionStreamerProbe.cpp), [`MapSplineGeometryProbe.cpp`](tests/MapSplineGeometryProbe.cpp), [`MapRoadGeometryProbe.cpp`](tests/MapRoadGeometryProbe.cpp); in-context visual validation remains |
| Scripts and dynamic displays | Vehicle script/text textures and scenery per-placement script/text texture surfaces, `init`/`frame`, and `[visible]` control are active; full OMSI behavior remains partial. | [`ScriptRuntime.cpp`](src/scripting/ScriptRuntime.cpp), [`MapRenderer.cpp`](src/render/map/MapRenderer.cpp) | [`ScriptTextureRuntimeProbe.cpp`](tests/ScriptTextureRuntimeProbe.cpp) |
| Weather | Partial first slice: configurable exterior rain/snow particles use the shared `PrecipType` and `PrecipRate` system variables; map weather schedules, fog, windshield effects, and accumulation are not implemented. | [`WeatherEffects.cpp`](src/render/WeatherEffects.cpp), [`RenderLoopDraw.cpp`](src/app/RenderLoopDraw.cpp) | [`WeatherEffectsProbe.cpp`](tests/WeatherEffectsProbe.cpp); rain and snow visually validated in app captures |
| Sound | Player sound runtime is active; AI vehicles select `[sound_ai]` when configured and otherwise fall back to `[sound]`. Full OMSI sound and AI event coverage remain incomplete. | [`VehicleSoundBank.cpp`](src/audio/VehicleSoundBank.cpp), [`VehicleConfigLoader.cpp`](src/config/VehicleConfigLoader.cpp) | [`SoundEngineProbe.cpp`](tests/SoundEngineProbe.cpp), [`VehicleConfigProbe.cpp`](tests/VehicleConfigProbe.cpp) |
| AI traffic, pedestrians, and routes | Partial runtime: asynchronous `.ovh` vehicle construction and model/script loading, asynchronous pedestrian `.hum`/model CFG and mesh loading, scheduled map buses, group-scoped path traffic rules, route-end retirement, sidewalk pedestrians, fixed-cycle plus request-aware SCO signal timing, and `unsched_trafficdens.txt` group/day curves for group-specific active targets are active. Group quotas change over time with safe off-screen admission/retirement; the startup pool remains fixed and may constrain extreme group shifts. AI driver models, overtaking, and full OMSI parity remain incomplete. | [traffic runtime](src/map/MapTrafficRuntime.cpp), [map configuration](src/map/MapConfigLoader.cpp), [render loop](src/app/RenderLoop.cpp) | [MapTrafficRuntimeProbe.cpp](tests/MapTrafficRuntimeProbe.cpp), [MapConfigLoaderProbe.cpp](tests/MapConfigLoaderProbe.cpp), [AssetRequestManagerProbe.cpp](tests/AssetRequestManagerProbe.cpp); see the [completion TODO](TODO.md) |

### Map runtime scope

The current map path loads `global.cfg` and uses its first entrypoint by default.
When `OPENBUS_MAP_SPAWN` is unset, startup prints all spawn points with their
zero-based indexes, names, world coordinates, and yaw. Set it to an index (for
example, `OPENBUS_MAP_SPAWN=1`) to spawn at that entrypoint's parsed position
and rotation. The older `OPENBUS_MAP_ENTRY` index/name selector remains
supported when `OPENBUS_MAP_SPAWN` is unset. Startup parses tile text once and
caches the parsed tile records/global placement-attachment index, then constructs
render geometry only for listed tiles in a two-tile Chebyshev window around the
bus (up to 5×5 tiles).
It uses the first `[groundtex]` entry as the
base and applies numeric per-tile `.map.<index>.dds` alpha masks to subsequent
`[groundtex]` layers. Set `OPENBUS_MAP_GROUNDTEX` to a zero-based
`[groundtex]` index to preview that texture uniformly, bypassing per-tile masks.
Detail-layer and seasonal selection remain unsupported. Terrain, road, and
scenery geometry and their window-owned GPU textures/buffers are created only
for that active window. On a tile crossing, a background worker builds and
uploads a complete replacement window using a GLFW shared OpenGL context; the
current window remains drawable until the replacement is ready, then its GPU
resources are released. If shared-context creation fails, loading falls back to
the synchronous path. Terrain and roads then use distance/frustum
rejection, scenery uses spatial chunks and per-instance culling, and `[visible]`
is checked per model part. Scenery runtimes are initialized/ticked only while
their source tile is active; their per-placement script state survives window
reloads. Scripts are not paused merely because the camera frustum excludes them.
Tile crossings still rebuild the full active window (rather than only the
entering tile), but construction and upload happen on the worker. Initial map
window construction at startup remains synchronous. Scenery script runtimes
created by a streamed window are initialized on the main script thread, at most
32 new runtimes per frame. Parsed tile records and the global placement index
remain CPU-cached for cross-tile attachment resolution. GPU contention and
boundary latency have not been measured.

ODE terrain and road/scenery collision is loaded separately around the player:
the load radius is one tile, and collision remains resident until a tile is
more than two tiles away, but only for tiles with successfully uploaded render
geometry in the current window. Terrain and road/scenery collision meshes are
prepared on a background CPU worker; ODE objects are published/removed on the
simulation thread, at most one completed tile per update.
Thus collision is independent of the camera frustum and scenery `[visible]`,
and collision residency also updates if the resident geometry set changes while
the player remains in the same center tile.
The initial collision neighborhood is warmed synchronously before gameplay;
runtime collision preparation is asynchronous, but ODE setup can still cost time
on publication. If preparation falls behind the player, temporary contact gaps
are possible. Physics residency follows loaded map tiles, not camera visibility.

Random map road traffic uses only `.ovh` entries from the `NormalCars`,
`Trucks`, and `Commercial`/`Commercials` `[aigroup_2]` groups. `.bus` vehicles
are excluded from this pool and spawned separately as scheduled AI buses using
bus-type configurations and timetable routes. Other groups and legacy
vehicle-list formats are not supported by this runtime. `OPENBUS_MAP_AI_COUNT` sets the
density-scaled road-vehicle base population (default `8`, maximum `200`).
`OPENBUS_MAP_AI_HOUR` sets the starting hour (default `12`); the in-game traffic
clock advances from it using elapsed simulation time and wraps at midnight.
Road and pedestrian populations follow their map density schedules as the
clock advances. A pool sized for each schedule's maximum is queued at
startup; vehicle instances are constructed and their scripts loaded on asset
workers, while model mesh parsing is deferred to asynchronous requests.
Pedestrian `.hum` and model CFG parsing, mesh parsing, and texture decoding
also use workers. OpenGL uploads and script initialization remain on the
main/render thread. Road vehicles and buses wait in an inactive queue until
their model and scripts are ready and they can be admitted at a connected
incoming lane end. Isolated splines, internal spline joins, and closed circuits
are never spawn points. Admission requires a clear vehicle footprint, an
endpoint outside the main camera frustum, and at least 50 m clearance from the
camera (plus the vehicle radius). No vehicles are distributed partway along
routes at startup or reactivation. Density reductions remove surplus vehicles
only off-screen. Vehicles are retired immediately when they reach an open
route end; subsequent spawns still obey the off-screen and clearance checks.
`.ttl` timetable departure times are
parsed and linked to `.ttp` trips. Matched buses wait for their next departure
before endpoint admission, including midnight rollover, while retaining the
off-screen and clearance checks. When multiple pooled buses serve a route, its
departure list is partitioned so each bus gets a disjoint subset of departure
times.
Pedestrians follow the authored human-path offsets and heights, render the
configured `.hum` model meshes and textures, and wait at scheduled pedestrian
crossing paths while their assigned signal phase is not green (states 6-8).
Skeletal walking animation and `[links]` behavior are not implemented. Active
pedestrian traffic actors also occupy collision footprints, so road AI
stops/yields rather than sweeping through them. Pedestrians likewise pause
before occupied vehicle footprints and resume when clear; this is kinematic
avoidance, not physical collision response.
Vehicles follow the authored lane offsets, heights, and travel directions shown
by the cyan F2 overlay, including scenery junction paths. Matching lane endpoints
join within 0.5 m horizontally and vertically; opposite-direction lanes are not
used as a forced return route. Road vehicles make reproducible seeded choices
among compatible straight, left, and right movements. Turns tighter than the
vehicle's minimum turning radius are rejected; authored radius data is used
when present, otherwise a conservative class/wheelbase estimate is used.
Genuine loops continue forward without reversing. Conflicting junction paths
use swept-footprint reservations; authored path/group priority takes precedence,
with wait age breaking ties among equal priorities. Synthetic regressions verify
that a waiting lower-priority movement yields to an approaching higher-priority
vehicle, crossing movements remain clear, and both make progress. Complete
native OMSI right-of-way behavior remains unverified.

Traffic accelerates at up to 1.6 m/s^2 and normally brakes at up to 3 m/s^2.
Lookahead reduces speed before bends using a 1.8 m/s^2 lateral-acceleration
limit. A 1.5-second following-headway target and a 2 m standstill buffer respond
to vehicles ahead, including the player bus. Oriented vehicle footprints and
small swept movement steps prevent AI from advancing through occupied space;
an emergency stop can exceed comfortable braking when a blockage appears too
close. Moving vehicles in the same lane are followed with a damped gap model
instead of being treated as parked obstacles, so a queue keeps flowing through a
junction behind a moving leader. Before entering a junction, a vehicle claims it
and waits at the entry line while any vehicle inside or already admitted would
cross or merge into its path, so two approaches never lock each other inside
the box. A vehicle approaching on a lower-priority spline gives way at its
entry line to any higher-priority vehicle whose path crosses or merges into its
junction movement and would arrive within the same conflict window (plus a
0.75-second safety margin accounting for vehicle occupancy); traffic that has
already cleared before the turn arrives does not hold it back;
higher-priority vehicles do not wait for such a vehicle's unused claim. Waiting
time only orders vehicles of equal priority. Crossing movements use
time-aware three-second lookahead reservations; road-sign right-of-way rules
beyond authored spline priority are not yet modeled.
Queues, actual endpoint spawns, junction entry speeds, off-screen population
unloads, route-end recycling, and lane-network statistics are logged.
Route planning tries an alternative branch if a later turn is impossible,
rejecting the entry route if no drivable exit or loop is reachable. Vehicles
exit only at unconnected lane ends; missing or misaligned authored connections
can still terminate a route.
AI buses are
selected separately from depot bus definitions and follow only timetable
routes whose `.ttp` trip names a destination and at least one station, and
whose `.ttr` spline chain contains the selected road route. They must also have
an eligible endpoint entry; maps without one do not receive fallback mid-route
bus spawns. Set
`OPENBUS_MAP_BUS_COUNT` to control their population (default `2`, maximum
`200`; `0` disables map AI buses). The bounded bus pool is created at startup
and only buses with a matched timetable route and parsed departures are
admitted at scheduled times. Maps without a complete endpoint-compatible
timetable route do not spawn scheduled buses. Traffic remains lane-constrained and kinematic:
signal/sign compliance, overtaking, lane changing, articulated swept-body
physics, and collision response when the player drives into AI are not implemented.

Road and timetable route networks are indexed from every tile listed in
`global.cfg` at map startup. AI vehicles retain those predefined lane routes while
rendering tiles stream in and out around the player; crossing a tile boundary
does not truncate or rebuild an actor's route.

Ordinary scenery gets a bilinearly sampled terrain height plus its authored
vertical placement value, unless its SCO/model declares `[absheight]`, in which
case the authored vertical coordinate is absolute.
Same-tile spline attachments/repeaters use typed spline-relative placement;
cross-tile chains are not complete. `[attachObj]` children resolve their parent
placement IDs across map tiles and use the indexed `[new_attachment]` point from
the parent SCO/model config, including the authored anchor and child rotations.
Missing parents/anchors and attachment cycles are skipped with diagnostics.
`[varparent]` editor object numbers are retained separately from map placement
IDs and checked against tile `Object Nr.` labels. Their scenery-script variable
inheritance effect is not evaluated, and they are not spatial attachment links.
`[spline_terrain_align]` currently tilts marked scenery so its local up axis
follows the sampled terrain normal, without changing its position. OMSI's exact
semantics for this marker have not been independently verified, so this is a
conservative renderer interpretation; `[spline_terrain_align_2]` remains
unsupported.
Scenery `[texttexture]` and `[scripttexture]` materials are bound per placed
instance, with independent live texture state and transparent text blending;
visual parity against installed directional-sign assets still needs validation.
Supported scenery meshes include O3D and static text, binary, and MSZIP-compressed `.x`;
configured nonzero LOD parts are selected by distance using the vehicle LOD
threshold mapping. Animated/skinned `.x` content remains unsupported, and
`[tree]` assets use a crossed-texture-plane preview rather than their helper mesh.
`.sli` rendering parses a subset of
`[texture]`, `[profile]`, and `[profilepnt]` records and tessellates straight
and signed-radius centerlines, but complete road/profile, banking, material,
and lane-marking behavior is not established. Road UV V uses a valid parsed
v11+ spline chain offset as its longitudinal origin; visual continuity and
OMSI parity still need confirmation against linked installed splines.

The following map behavior is **not implemented** or remains explicitly
partial:

- Animated/skinned DirectX `.x` content; static text, binary, and MSZIP-compressed
  `.x` meshes now load. The tree preview is not the original mesh. Cross-tile
  repeater-master resolution, full `[attachObj]` scripting,
  scenery script-driven animation/material changes, scenery mouse events, and
  non-authored scenery collision are incomplete. Per-placement scenery
  `init`/`frame`, `[visible]`, `[usetexttexture]`, and `[usescripttexture]`
  surfaces are active.
- Full `.sli` semantics and visual validation of curved sidewalks, road seams,
  and lane markings. Road-profile textures/material alpha, cant/banking, and
  texture phase across linked splines need representative asset tests. Parsed
  v11+ chain offsets now affect road texture V, but do not resolve links or
  prove visual parity.
- Supported `.sli` profile-strip road geometry and declared SCO/model
  `[collision_mesh]` O3D geometry are connected to ODE for tiles within one
  tile of the bus; terrain and road/scenery collision are unloaded beyond a
  two-tile radius. `[boundingbox]` is not yet used for map scenery, and
  overlapping road/terrain contacts still need tuning.
- Per-tile/detail/seasonal ground-texture rendering, water rendering, tile
  lightmaps, Chrono map variants, and asynchronous map construction are absent.
  Per-tile DDS filenames are inventoried, but their payloads and mapping to
  `[groundtex]` ordinals are unresolved. All map geometry stays resident after
  construction; active-window gating does not free GPU memory.
  `.map.water` records are size/finite-value validated and reported, but their
  spatial height ordering is unresolved, so no water geometry is drawn.
  Unimplemented bracketed sections in tile files and `global.cfg` are reported
  with line numbers during map loading. Global identity, description, seasons,
  and other documented metadata are retained; road/passenger density points
  drive provisional, separately capped AI populations.
  `[variable_terrain]` and `[variable_terrainlightmap]` markers are recognized
  and reported, but their dynamic terrain/lightmap behavior is not rendered.
  `[varparent]` and spline-terrain alignment data remain partial. `[rule]` and
  `[kill_rule]` apply supported group-specific speed, density, priority, and
  vehicle-access settings to road paths; see the [OMSI map format reference](Docs/OMSI2_MAP_FORMAT_REFERENCE.md).
  The separate `unsched_trafficdens.txt` group/day schedule applies its hourly
  curves to the capped road-traffic population target, startup pool composition,
  and per-group active targets. The pool remains fixed at startup and can
  constrain large changes in group proportions. The initial weekday comes from
  the host calendar because no map simulation-date source is available.
- Scheduled OMSI AI buses use matched timetable routes and departures; fixed
  SCO traffic-light schedules stop road traffic and pedestrians on linked
  paths. Signalized scenery instances expose the active phase through
  `trafficlightphase` for model scripts/visibility variables. Conditional
  stop/jump overrides and `signalroutes.cfg` routing remain incomplete, and
  model-specific lamp animation plus full in-game OMSI parity have not been
  validated.

The existing centerline probe validates linked endpoints and signed curve
directions, but does not validate cross-section profile placement, sidewalks,
markings, or visual seams. The [map TODOs](TODO.md#grande-porto-map-support)
track the missing runtime paths and visual evidence.

## Source layout

The engine is split into one static library per module (see `src/CMakeLists.txt`).
Headers are included by module-qualified paths, e.g. `#include "render/CoreRenderer.h"`.

| Folder | CMake target | Responsibility | Depends on |
| --- | --- | --- | --- |
| `src/core` | `openbus_core` | Logging, perf tracing, script variables, OMSI root discovery, shared value types | (none) |
| `src/osc` | `openbus_osc` | OMSI script (`.osc`) parser, bytecode compiler and Lua converter | (none) |
| `src/config` | `openbus_config` | OMSI configuration parsers (bus, model `.cfg`, scenery objects, vehicles) | core, osc (private) |
| `src/assets` | `openbus_assets` | Mesh (`.o3d`, `.x`, `.obj`) and texture loading, screenshot writing | core |
| `src/physics` | `openbus_physics` | ODE bus simulation and bus configuration | config, assets |
| `src/map` | `openbus_map` | Map loading, road/spline/water geometry, scenery placement, collision streaming | config, assets, physics |
| `src/scripting` | `openbus_scripting` | Script runtime (native bytecode and Lua backends), script textures | config, assets, osc/Lua (private) |
| `src/audio` | `openbus_audio` | OpenAL sound engine and vehicle sound banks | config |
| `src/input` | `openbus_input` | Keyboard/vehicle key bindings | core, GLFW |
| `src/render` | `openbus_render` | OpenGL renderer, asset requests, camera/matrix math, reflections; `render/map` builds and draws the map scene | assets, config, map, physics, scripting |
| `src/app` | `OpenBus` executable | Entry point, render loop (`RenderLoop*.cpp`), HUD and per-vehicle rendering in `app/vehicle` | all modules |

Large subsystems keep their public header small and place implementation details in an
internal header (`*Internal.h` / `*Impl.h`) shared by focused translation units:

- `app/RenderLoop*.cpp`: lifecycle, input/camera, simulation/scripts, drawing.
- `render/map/MapSceneLoader.*`: map scene construction (terrain, scenery assets, tile content);
  `MapRendererDraw.cpp`: per-frame map drawing.
- `scripting/Script{RuntimeImpl,Textures,NativeBackend,LuaBindings}.cpp`: runtime core,
  script textures, native OSC backend and Lua bindings.

Vendored code lives in `third_party/` (Lua 5.1, sol2) and is built separately from engine modules.

## Prerequisites

- CMake 3.20 or newer
- A C++23 compiler
- zlib
- ODE
- GLFW and OpenGL
- OpenAL
- Lua 5.1 on Linux (Windows uses the bundled Lua 5.1 runtime)

On Windows with vcpkg, install standalone LLVM (for `clang-cl.exe`) and the
Visual Studio C++ build tools (for the MSVC headers, libraries, and linker):

```powershell
vcpkg install ode:x64-windows glfw3:x64-windows zlib:x64-windows
```

On Debian or Ubuntu, install the native development packages:

```bash
sudo apt install build-essential cmake libode-dev libglfw3-dev libglu1-mesa-dev libopenal-dev liblua5.1-0-dev zlib1g-dev
```

The Linux build uses the system GLFW/ODE packages when available and falls
back to downloading GLFW during CMake configuration when GLFW is not found.

If `glfw3` is not available through `CMAKE_PREFIX_PATH`, the build now falls
back to downloading GLFW 3.4 during CMake configure. This still requires
internet access at configure time.

## Configure and build

The Windows helper uses the `build-ninja` directory, creating it if needed and
reusing it for subsequent incremental builds. It refuses to launch if an
`OpenBus.exe` process is already running; close that process yourself before
using the helper.

```powershell
.\run.bat
```

Native MSVC is rejected because its CMake C++23 mapping is
`/std:c++latest`, not an exact C++23 mode. The Windows `run.bat` helper
initializes the MSVC x64 environment and uses standalone Clang-cl with the
Visual Studio-bundled Ninja executable. Its `windows-clang-ninja` preset builds
in `build-ninja` using 12 parallel jobs; the original NMake `build-ode`
directory remains untouched. The first Ninja build compiles from scratch, while
subsequent builds recompile only changed sources. For a manual NMake configure
from `cmd.exe`:

```bat
call windows_clang_env.bat
cmake -S . -B build-ode -G "NMake Makefiles" -DCMAKE_CXX_COMPILER="%OPENBUS_CLANG_COMPILER%" -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake
cmake --build build-ode --target OpenBus
```

Release and RelWithDebInfo builds enable interprocedural/link-time optimization
for `OpenBus` by default. Disable it when comparing profiler call boundaries or
when using a toolchain that does not support IPO:

```bat
call windows_clang_env.bat
cmake -S . -B build-ode -G "NMake Makefiles" -DCMAKE_CXX_COMPILER="%OPENBUS_CLANG_COMPILER%" -DCMAKE_BUILD_TYPE=Release -DOPENBUS_ENABLE_IPO=OFF
```

On Linux, use a native build directory and the provided shell helpers:

```bash
chmod +x run.sh profile.sh screenshot.sh convert.sh
./run.sh
```

The helpers use `build-linux` by default. Set `BUILD_DIR` to choose another
build directory, and set `OPENBUS_VEHICLE=e400` to run the E400 model.

## Linting

Install LLVM so `clang-tidy` is available on `PATH`, then run:

```powershell
.\lint.bat -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake
```

This uses the separate `build-lint` directory and enables the project’s
`.clang-tidy` checks without changing the normal `build-ode` configuration.

To format the project’s C++ files with `clang-format`, install LLVM and run:

```powershell
.\format.bat
```

Run the prototype:

```powershell
.\build-ode\Release\OpenBus.exe
```

The executable opens a simple OpenGL preview window while the physics runs at
the configured fixed rate.

### OMSI configuration audit

On Windows, run the helper with an OMSI directory (or another directory to
inspect). It builds `OpenBusConfigAudit` in `build-ode` before scanning:

```powershell
.\audit.bat "C:\Path\To\OMSI 2"
```

Without an argument, the helper uses `OMSI_ROOT` if set, or prompts for a
directory. Set `OPENBUS_BUILD_DIR` to use a different existing CMake build
directory.

The auditor recursively checks `.bus` files, recognized model `.cfg` files,
and `.osc` scripts without writing translated Lua files. Build/output, vendor,
and virtual-environment directories are skipped. Results distinguish
`INVALID` from `UNSUPPORTED`; unknown `.cfg` families are reported as
unsupported instead of being parsed under the wrong grammar. OSC reported as
unsupported by OpenBus's native compiler is not treated as definitely invalid.
The process returns 1 if invalid files were found, 0 otherwise, and 2 for an
invalid invocation/root path.

### Optional binary mesh bundle

To reduce startup parsing and file-open overhead, generate an `.obx` bundle
from converted OBJ meshes:

```powershell
.\build-ode\Release\obj_bundle.exe MAN_DL05\Model\DL05_obj build-ode\DL05.obx
```

Copy or rename the resulting file to `openbus.obx` beside the model's
converted meshes. OpenBus uses matching bundle entries when available and
falls back to OBJ files for missing entries, so the bundle is optional.
The model CFG still controls mesh order, visibility, animations, and material
overrides.

The Windows build writes a symbolized `game.crash` report with exception
details, registers, module names, and source lines when matching PDB files are
available. It also writes `game.crash.dmp`, which can be opened in Visual
Studio or WinDbg with the matching executable and PDB. The Linux build writes
a `game.crash` stack trace for fatal POSIX signals and unexpected termination.
Build with debug symbols when detailed function names are needed, for example
with `-DCMAKE_BUILD_TYPE=Debug`.

## Current physics model

- ODE world with gravity and a hash-space collision broad phase
- Box chassis with configurable mass and dimensions
- Configurable axle layout, including arbitrary axle count, track width,
  steering, and driven-axle flags
- ODE contact joints for chassis/ground and wheel/ground collisions
- Per-wheel ODE bodies with cylinder geometry, suspension slider joints,
  steering hinges, and wheel rotation hinges
- Per-corner spring-damper suspension, anti-roll forces, tyre friction,
  braking, rolling resistance, aerodynamic drag, steering, bounded chassis
  attitude stabilization, and automatic gearing
- Wheel torque, wheelspin, and skid telemetry derived from the ODE wheel bodies
- Fixed-rate physics (60 Hz by default) with a bounded catch-up budget; if a
  frame exceeds that budget, excess accumulated time is dropped to avoid a
  spiral of death

Vehicle physics is loaded from the OMSI `.bus` file configured by
`OPENBUS_BUS_CONFIG`. Relative paths are resolved against the standard Steam
installation at `C:\Program Files (x86)\Steam\steamapps\common\OMSI 2`; set
`OPENBUS_OMSI_ROOT` when OMSI is installed elsewhere:

```powershell
$env:OPENBUS_OMSI_ROOT = "D:\SteamLibrary\steamapps\common\OMSI 2"
```

Vehicle values are not duplicated in C++.
Articulated vehicles are outside the currently supported scope. The
`openbus_articulated` configuration flag is retained for inspection only; it
does not create additional ODE bodies, joints, collision shapes, or rendered
sections, so articulated models must not be treated as supported vehicles.
The parsed configuration is written to `OpenBus_configuration.json` at startup.
Parsed model parts, materials, visibility variables, animation variables, and
wheel bindings are written to `OpenBus_model_configuration.json` as well. For
parts with `[animparent]`, `parent_animations` shows the variables inherited
from the referenced mesh, which makes door-glass control variables visible.

The model variable store initializes OMSI-style system variables with the local
system date at midday: `Time` is `43200`, `Day`, `Month`, and `Year` come from
the current date, and `DayOfYear` is the zero-based day index. Defaults include
`NoSound=1`, `Pause=0`, `PrecipType=0`, `PrecipRate=0`,
`Weather_Temperature=20`, `Weather_AbsHum=10`, `Envir_Brightness=1`,
`AutoClutch=0`, and `SunAlt=60`.
`Timegap`, `GetTime`, `mouse_x`, and `mouse_y` are updated from the renderer
each frame. Precipitation values are persistent shared system variables, so
configured weather and script overrides survive frame updates.

The initial weather slice renders rain or snow particles in exterior and free
camera views. Set `OPENBUS_WEATHER=rain` or `snow`; `clear` (the default)
disables precipitation. `OPENBUS_WEATHER_RATE` accepts `0`–`1` and defaults to
`0.5` for rain/snow. Weather settings populate the shared `PrecipType` and
`PrecipRate` system variables before scripts run, so scripts can override
them. This does not yet load OMSI map weather schedules or render fog,
windshield effects, wet roads, or snow accumulation.

`OPENBUS_BUS_CONFIG` accepts either an absolute path or a path relative to
`OPENBUS_OMSI_ROOT`. Set the optional `OPENBUS_MODEL_CONFIG` to select a `.cfg`
explicitly; otherwise the loader uses the `.bus` file's `[model]` entry. For
example, both paths below are relative to the OMSI installation root:

```powershell
$env:OPENBUS_BUS_CONFIG = "Vehicles/Caetano Levante/Caetano.bus"
$env:OPENBUS_MODEL_CONFIG = "Vehicles/Caetano Levante/Model/model_caetano.cfg"
./build-ode/Release/OpenBus.exe
```

Every rendered vehicle must provide a `VehiclePlacement` containing its world
`x/y/z` position and yaw in degrees. The renderer can load multiple vehicles
in one process by calling `RenderLoop::AddVehicle` once per `.bus`/`.cfg` pair.
The first vehicle is connected to the shared bus simulation; additional
vehicles are scripted AI models rendered at their declared placements. Set
`OPENBUS_AI_BUS_CONFIG` and `OPENBUS_AI_MODEL_CONFIG` together to add AI
vehicles. `OPENBUS_AI_COUNT` selects the number to spawn (defaults to 1 and is
limited to 200); multiple instances are arranged in a grid. `run.bat` selects
the MAN DL05 on Cotterell at spawn 0.

The loader uses `[mass]`, `[boundingbox]`, `[schwerpunkt]`, and `[newachse]`
records, including axle spring, damper, load, and driven flags. The
`bodyHalfLength`, `bodyHalfWidth`, `bodyHalfHeight`, collision dimensions, and
collision offsets are derived from `[boundingbox]`. Because standard OMSI
records do not define wheel thickness or articulation, each file also supplies
`[openbus_wheel_half_width]` and `[openbus_articulated]`. Axle steering is
derived from `Axle_Steering_X_L/R` variables in the referenced model.cfg.

## Renderer controls

Vehicle script bindings use named keyboard keys rather than OMSI numeric key
codes. Pressing an action calls `trigger_<action>`; releasing it calls
`trigger_<action>_off`. At startup, matching vehicle actions inherit their key
and modifier assignments from `Inputs/keyboard.cfg` under `OPENBUS_OMSI_ROOT`;
the documented table below is the fallback when that file is unavailable.

| Key | Vehicle actions |
| --- | --- |
| Num8 | `throttle` |
| Num2 | `brake` |
| Num4 / Num5 / Num6 | `steering_left` / `steering_neutral` / `steering_right` |
| Tab | `clutch` |
| Num+ | `throttle_amplify`, `bus_doorfront5` |
| T | `ticket_give` |
| Shift+T / Ctrl+T | `change_take` / `change_give` |
| `.` | `parking_brake_toggle` |
| Num7 / Num9 / Num. | `blinker_left_set` / `blinker_right_set` / `blinker_off` |
| B | `blinker_warn_toggle` |
| L / Shift+L | `kw_scheinwerfer_toggle` / `kw_standlicht_toggle` |
| F / Ctrl+F | `kw_fernlicht_toggle` / `taster_nebelschluss` |
| M | `kw_m_enginestart` |
| W / Shift+W / Ctrl+W | `kw_wipermode_up` / `cp_wischer_intervall_toggle` / `cp_wischer_wascher_button` |
| H | `horn` |
| Q | `cp_microphone` |
| E | `cp_batterietrennschalter_toggle` |
| 6 / 7 / 8 / 9 | `cp_fahrerlicht_toggle` / `cp_licht_untenrechts_toggle` / `cp_licht_oberdeck_toggle` / `cp_licht_unterdeck_toggle` |
| R / N | `kw_s_R`, `automatic_R` / `kw_s_N`, `automatic_N` |
| 1 / 2 / 3 / 4 / 5 / 6 | `kw_s_1` / `kw_s_2` / `kw_s_3` / `kw_s_4` / `kw_s_5` / `kw_s_6` |
| D | `automatic_D` |
| Num/ / Num* / Num- / Scroll Lock | `bus_doorfront0` / `bus_doorfront1` / `bus_dooraft` / `bus_20h-switch` |
| F5 / F6 / F7 / F8 | `bus_linie_minus` / `bus_ziel_minus` / `bus_ziel_plus` / `bus_linie_plus` |
| F12 | `cp_schalter_kinderwagen` |
| `[` / `/` | `kw_s_plus` / `kw_s_minus` |

Cashdesk, IBIS, and rollband actions are intentionally not assigned yet.
- `0`: outside orbit camera
- `1`-`9`: cameras from the selected `.bus` file in driver/passenger order
- Left/right arrow: step through driver/passenger cameras in file order
- Default camera: the driver camera selected by `[set_camera_std]`
- Hold middle mouse and drag: orbit outside view `0`, or look around the
  selected non-third-person viewport
- Press `F4` to toggle mouse look; while enabled, move the mouse to look around
  without holding a middle button. The cursor is captured until `F4` is pressed
  again, and arrow keys continue to select camera views. Set
  `OPENBUS_MOUSE_LOOK=1` to enable it at startup.
- Hold right mouse and drag vertically: adjust FOV in configured views, or
  move closer/farther in the outside orbit view
- Scroll up/down: zoom in/out
- Press `F3` to toggle the no-clip free camera. Use the up/down arrows to move
  forward/back and left/right to strafe; use `Page Up` / `Page Down` to move
  vertically. Middle-mouse drag looks around, the mouse wheel adjusts movement
  speed, and right-mouse drag adjusts FOV. Freecam does not move the bus or
  change its physics. The active map window and collision streaming remain
  centered on the bus; flying beyond the nearby active tiles will not trigger
  camera-centered map streaming.
- Press `I` to show clickable mesh bounds; the currently hovered clickable part
  is highlighted. `game.log` lists mesh dimensions, mouse events, and mapped
  script trigger availability when the overlay is enabled.
- Press `C` to show the active ODE terrain and road/scenery collision trimeshes
  as an orange wireframe, plus the bus fallback collision box. Static map
  collision streams around the bus (one-tile load radius, two-tile unload
  radius); the debug wireframe refreshes as tiles enter and leave that set.
- Press `F2` to show traffic paths in the active map tiles, including authored
  lane offsets and scenery-object junction paths. SCO `[path]` records are
  tessellated as local spline segments from their position, heading, radius,
  and length fields. Road/AI paths are cyan and human paths are magenta. Orange
  Xs show map-spline endpoints without reciprocal links or a matching map or
  scenery path endpoint within 0.5 m horizontally and vertically. Matching
  endpoints are joined in the overlay with a short colored connector when a
  visible gap remains; this does not change AI route connectivity.
- Map-driven road vehicles use deterministic starts on authored road routes
  containing at least two linked splines. AI buses are further restricted to
  linked spline chains present in timetable trips with station data. Vehicles
  follow authored link order; closed routes cycle as authored and open routes
  retire at their endpoint rather than reversing back along the path.
- Press `U` to raise the player bus and its physics bodies 5 m at the current
  X/Y location; it then falls naturally under gravity.
- VSync is enabled by default; set `OPENBUS_VSYNC=0`, `off`, or `false` before
  launching to run above the display refresh cadence
- Reflection mirrors render at `256x256` by default. Set
  `OPENBUS_REFLECTION_SIZE=256`, `512`, or `1024` to choose the square mirror
  target resolution.
- Mirrors update every frame by default. Set `OPENBUS_REFLECTION_INTERVAL=4`
  to update them once every four rendered frames; values below `1` are treated
  as `1`.
- Set `OPENBUS_REFLECTION_MAX_FPS=60` to cap mirror refreshes at 60 FPS without
  limiting the main game. The cap is unset by default; when both reflection
  controls are configured, the interval and time-based cap both apply.
- Mirror transparency is rendered by default. Set
  `OPENBUS_REFLECTION_TRANSPARENT=0` to render opaque geometry only and reduce
  mirror workload when transparent details are not needed. This setting is
  read once on the first mirror draw and retained for the process lifetime.
- Set `OPENBUS_MATERIAL_BATCHING=1` to use the experimental material-table
  shader path. It combines compatible opaque geometry and supports up to 8
  ordinary 2D base textures per group; transparent, animated-material, and
  auxiliary-texture batches remain on the standard path. It is disabled in
  `profile.bat` because the current E400 workload is GPU-bound and the extra
  material texture sampling is slower than the additional draw calls.
- Native OSC bytecode is the default script backend. Scripts that cannot be
  compiled fall back to Lua; set `OPENBUS_SCRIPT_BACKEND=lua` to force Lua for
  debugging.
- OSC numeric values use signed 32-bit floating-point precision in both
  backends. Conditions and logical inputs treat zero as false and any non-zero
  value as true; logical and comparison results are canonical `0` or `1`.
- Scripts normally follow the render rate. Set `OPENBUS_SCRIPT_HZ=30` to run
  vehicle and scenery scripts at 30 Hz while rendering continues at the display rate;
  fixed-rate script ticks receive `Timegap=1/30`. Each update runs at most eight catch-up
  ticks and clamps elapsed time to 250 ms. If more whole ticks remain, they are dropped to
  keep script work bounded; the fractional remainder is retained and dropped time is logged
  periodically. Invalid or non-finite rates are logged and scripts follow the render rate.
- Stationary-pointer hover checks refresh every 75 ms (about 13.33 Hz) by default.
  Set `OPENBUS_HOVER_HZ=30` to refresh at 30 Hz, or `OPENBUS_HOVER_HZ=0` to
  check every rendered input frame. Rates are clamped to 0–1000 Hz; invalid or
  non-finite values are logged and retain the default. Cursor movement, camera
  changes and clickable-state changes still invalidate the cache immediately;
  clicks are not throttled by this setting.
- Profiling retains at most `1,000,000` events by default. Set
  `OPENBUS_TRACE_MAX_EVENTS=250000` or another positive value to lower the
  memory ceiling. Trace events are flushed incrementally to the JSON file while
  profiling runs, rather than being held until shutdown.
- Trace scopes shorter than `1` microsecond are omitted by default. Set
  `OPENBUS_TRACE_MIN_US=5` to omit scopes shorter than five microseconds;
  values below `1` are treated as `1`.
- Optional texture downscaling: set `OPENBUS_TEXTURE_SCALE=0.5` before
  launching to upload half-resolution textures; valid range is `0.25`-`1.0`
  JPEG files. To create BC3 DDS siblings with AMD Compressonator, run
  `.\convert_textures.ps1 -Compressonator C:\path\to\CompressonatorCLI.exe`.
  selection, and sorted opaque/transparent batches to reduce off-screen work
  and state churn. Set `OPENBUS_FRUSTUM_CULLING=0` to disable culling for
  debugging. Opaque parts are front-to-back and transparent parts
  back-to-front; this is not hardware occlusion-query culling.
- Set `OPENBUS_WHEELS_FROM_ODE=1` to bypass wheel `newanim` transforms and
  place each wheel from its matching ODE body pose. The default is `0`.
- Doppler pitch shifting is disabled by default. Set `OPENBUS_DOPPLER=1` (also
  accepts `true`, `yes`, or `on`) before launch to enable OpenAL Doppler using
  listener and moving-loop velocity estimates. Velocities are reset after long
  frame gaps and capped to avoid pitch spikes on camera/position teleports.
- Model geometry is uploaded to OpenGL vertex buffer objects (VBOs) and drawn
  with shader-backed `glDrawArrays` calls in an OpenGL 3.3 core profile.
- Window and door glass remains material-driven in the VBO path; the temporary
  flat glass fallback is no longer used for normal model materials.
- Normal MAN DL05 wheel meshes are removed from the static model pass and
  rendered from the live ODE wheel poses, including suspension, steering, and
  wheel rotation; the wire wheel fallback is used only when an asset is absent.
- The driver camera follows the ODE chassis position, pitch, roll, and heading
- OMSI coordinates use `x=lateral`, `y=longitudinal`, `z=vertical`; OpenBus
  converts them to engine `X=longitudinal`, `Y=lateral`, `Z=vertical` with the
  expected lateral reflection. This applies to CFG positions, camera centers,
  collision offsets, OBJ vertices, normals, and mesh pivots.
- O3D pivot matrices are animation metadata, not additional mesh placement
  transforms. Mirrored meshes are detected from pivot determinant and
  face/normal agreement; only those meshes have winding correction and
  clockwise back-face culling enabled.
- `[newanim]` records and their origin commands are processed in configuration
  order. Origin-only records are valid. `anim_rot`/`anim_trans` use the
  converted local animation frame, `delay` is rate-based smoothing, and
  `maxspeed` limits movement per second. `[animparent]` composes parent
  transforms for body and wheel/detail meshes.
- When `MAN_DL05/Model/DL05.cfg` is available, the highest-detail `[LOD] 0.25`
  model block is rendered with driver-viewpoint mesh filtering
- OBJ texture coordinates follow the converter's existing V-axis conversion
  plus the final OpenGL image-origin correction;
- MAN DL05 OBJ lateral coordinates are mirrored into the simulator convention
  so the modeled right-side exit remains on the vehicle's right
  TGA, BMP, PNG/JPEG, and DXT1/DXT5 DDS texture data are decoded for model
  materials
- OBJ `mtllib`/`usemtl` groups are rendered with their material textures;
  converter-generated texture extensions are resolved by filename stem
- Window meshes use a depth-write-disabled, controlled translucent glass pass
  until the original script-driven OMSI material stack is implemented
- Script-controlled `WerbungInnen` advertising overlays are skipped until
  texture-state scripts are implemented, preventing opaque white overlays from
  covering the glass
- Door meshes retain their opaque panels while `fenster_*` door glass groups
  render through a separate static translucent pass
- Opaque DXT1 textures preserve color-index 3 as opaque, preventing roof
  materials from becoming transparent and exposing interior geometry
- Texture lookup diagnostics are written to `OpenBus_textures.log`; entries
  are deduplicated and record loaded files, missing source files, and decode
  failures
- The visible chassis and wheels use the live ODE body position and rotation,
  including chassis pitch/roll, suspension travel, steering, and wheel rotation
- A red three-axis marker shows the aggregate mass-weighted center of gravity
  at the configured 1.0 m bus CG height
- The configured 11,500 kg bus mass includes the wheel and suspension bodies;
  the chassis body mass is reduced accordingly so the complete ODE assembly
  totals 11,500 kg
- Close the window to exit

The ground uses a world-space grid with highlighted ten-metre lines.

## CPU profiling

Use `profile.bat` (Windows) or `profile.sh` (Linux) to launch a Release build
with tracing instrumentation. **Recording starts off.** Navigate to the place
you want to profile, then press **F10** to start recording. Press **F10** again
to pause and save; further presses resume/pause the same capture. Holding the
key does not repeatedly toggle it. Recording state changes are logged.

The Chrome trace is written to `openbus_trace.json`; `OPENBUS_TRACE_FILE`
overrides its path. Pausing drains pending events and leaves a valid JSON file
that can be opened in Perfetto or a Chrome trace viewer without exiting.
Resuming adds to that file; paused time and scopes spanning a pause are not
recorded. Timestamps retain the real gaps between recording intervals. The
event limit applies across all intervals in the run. The first recording in
a new run overwrites the output file; starting paused does not touch it.

For automatic startup/loading captures, launch an instrumented executable
directly with `OPENBUS_TRACE=1`. This opt-in preserves automated benchmark
capture behaviour. Normal builds with `OPENBUS_ENABLE_PERF_TRACE=OFF` do not
record; F10 logs that a profiling build is needed.

`RenderLoop::draw` includes separate scopes for ground-height sampling, collision
debug rendering, the status HUD (text update, rasterization, upload, and submission),
and window-title updates. The HUD is the final overlay and restores the scene's
depth-enabled, blend-disabled, culling-disabled state without querying the driver.
Trace durations are CPU wall times, not GPU execution times: driver backpressure
can appear in a GL call or in `RenderLoop::endFrame.swapBuffers`. Compare whole
`main` frame times rather than treating a shorter draw scope as an equal FPS gain.

## Repeatable render benchmark

Run `benchmark_rendering.bat` to build an isolated Release executable
with tracing enabled, then benchmark 720p, 1080p, 1440p, 4K, and 5120x1440.
The runner is `benchmark_rendering.py`, requires Python 3.9 or newer, and uses
only the standard library; set `OPENBUS_PYTHON` if `python` is not on PATH.
The runner performs five runs per resolution by default, in deterministic
seeded order; use `--runs 1` for a quick check or `--seed N` to change the order.
After building, the Python runner can also be launched directly with
`python benchmark_rendering.py`. Each run measures baseline rendering,
user-facing camera cycling, third-person zoom, driving controls, and dashboard
interaction. Benchmark mode creates a hidden GLFW window, uses a fixed 60 Hz
simulation step, and disables VSync. Hiding the window may avoid OS work-area
constraints; actual dimensions are checked and mismatched runs are skipped before
warm-up and measurement.
Normal simulator launches remain visible and retain the standard `build-ode`
build and `profile.bat` settings.

`--warmup-frames` is a total distributed across the five phases (60 by default,
12 per phase). Each phase is then measured twice per resolution run, with 120
fixed-step frames per visit by default (two seconds at 60 Hz), slowing camera,
zoom, steering, and dashboard-look progressions to half their previous rate.
Use `--phase-repeats` and `--phase-frames` to adjust those values. Results are written to
`render-benchmark-results/`. The `*_runs.csv` and `*_scopes.csv` files record
requested window, actual window, framebuffer sizes, framebuffer/window scale
factors, and flag any framebuffer that differs from the requested resolution.
`render_benchmark_frametimes.csv` reports one measured-frame summary row per
run, including average, p90, and p95 frame times in milliseconds across all
measured phases and repeats; the same metrics are also included in
`render_benchmark_runs.csv`. Percentiles use linear interpolation over the
sorted frame-time samples. Skipped runs remain in the run and frame-time summary
files with a status and reason but no timing metrics. Use `--require-exact-resolution`
to return a failure status if any run is clamped. The `*_frames.csv` file records
one row per main-frame or selected renderer-scope event, including timestamp,
inclusive duration, and self time. Nested calls stay separate so reflection
draws are not accidentally summed into a misleading per-frame total. High-volume
`Vehicle::drawBatch.prepareAndSubmit` events are included only when they last at
least 0.1 ms, keeping normal batch submissions from overwhelming the report.

The `*_scopes.csv` file groups Chrome trace scopes by phase and reports
occurrence count, summed inclusive elapsed wall time, summed self time, average,
and maximum duration. Durations are milliseconds converted from trace
microseconds; these are CPU-side elapsed scope timings, not GPU timings. Self
time subtracts the union of direct child intervals on the same thread, avoiding
double subtraction for overlaps. Raw per-run traces and logs are retained
alongside the CSV files.

## OMSI bus-file reference

The supplied OMSI examples are summarized in
[Docs/OMSI_BUS_FORMAT_REFERENCE.md](C:/Users/blane/Desktop/OpenBusTest/Docs/OMSI_BUS_FORMAT_REFERENCE.md).
It documents parser syntax, count-prefixed and terminated blocks, identity,
assets, scripts, cameras, physical data, axles, suspension, couplings, AI,
rail/trolley sections, and a minimal bus-file skeleton.

The CFG corpus is indexed in
[Docs/OMSI_CFG_FORMAT_REFERENCE.md](C:/Users/blane/Desktop/OpenBusTest/Docs/OMSI_CFG_FORMAT_REFERENCE.md).
Bus-only model construction is documented separately in
[Docs/OMSI_CFG_BUS_MODEL_REFERENCE.md](C:/Users/blane/Desktop/OpenBusTest/Docs/OMSI_CFG_BUS_MODEL_REFERENCE.md);
general surfaces, passenger cabins, paths, sounds, environment, AI/network,
controls, and metadata remain in
[Docs/OMSI_CFG_GENERAL_REFERENCE.md](C:/Users/blane/Desktop/OpenBusTest/Docs/OMSI_CFG_GENERAL_REFERENCE.md).
The original CFG files remain in place so their relative paths continue to
work.

The OMSI 2 map-package reference is in
[Docs/OMSI2_MAP_FORMAT_REFERENCE.md](C:/Users/blane/Desktop/OpenBusTest/Docs/OMSI2_MAP_FORMAT_REFERENCE.md).
It documents `global.cfg`, tile records and sidecars, Chrono variants, AI lists,
timetable files, parser limitations, and validation against the installed map.
