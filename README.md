# OpenBus

OpenBus is a C++ bus simulator prototype built on the [Open Dynamics Engine
(ODE)](https://www.ode.org/). It combines fixed-step bus physics with OpenGL
vehicle rendering and partial OMSI map rendering. This is a prototype, not a
fully OMSI-compatible simulator.

## Compatibility and validation status

This table summarizes the current implementation, not complete OMSI parity.
Parser probes establish only the parser behavior they exercise; they do not
prove end-to-end rendering or runtime semantics.

| Feature | Current status | Implementation | Regression evidence |
| --- | --- | --- | --- |
| Physics and wheel simulation | Fixed-step ODE and per-wheel simulation are active; the deterministic bump/steering/braking stress probe covers two- and three-axle fixtures. Broader tuning remains incomplete. | [`BusSimulation.cpp`](src/physics/BusSimulation.cpp) | [`BusSimulationSteeringProbe.cpp`](tests/BusSimulationSteeringProbe.cpp), [`BusSimulationStabilityProbe.cpp`](tests/BusSimulationStabilityProbe.cpp), [`BusSimulationStressProbe.cpp`](tests/BusSimulationStressProbe.cpp) |
| Vehicle/model loading | Core MAN DL05 and E400 configuration paths are implemented; broad OMSI configuration coverage remains partial. | [`VehicleConfigLoader.cpp`](src/config/VehicleConfigLoader.cpp), [`ModelConfigLoader.cpp`](src/config/ModelConfigLoader.cpp), [`RenderLoop.cpp`](src/app/RenderLoop.cpp) | [`VehicleConfigProbe.cpp`](tests/VehicleConfigProbe.cpp), [`ModelConfigProbe.cpp`](tests/ModelConfigProbe.cpp); visual procedure: [screenshot plan](Docs/SCREENSHOT_VALIDATION_PLAN.md) |
| Texture decoding and materials | Texture formats and selected material behaviors have focused parser/GPU probes, including alpha mode 2, bump maps, environment-map sampling, transparent sorting, CTC texture replacement, and array-layer bounds. Representative-asset parity and difficult overlap cases remain incomplete. | [`TextureLoader.cpp`](src/assets/TextureLoader.cpp), [`CoreRenderer.cpp`](src/render/CoreRenderer.cpp) | [`TextureLoaderProbe.cpp`](tests/TextureLoaderProbe.cpp), [`TextureAddressModeProbe.cpp`](tests/TextureAddressModeProbe.cpp), [`MaterialRenderingProbe.cpp`](tests/MaterialRenderingProbe.cpp), [`VehicleDrawSortingProbe.cpp`](tests/VehicleDrawSortingProbe.cpp), [`VehicleMaterialSelectionProbe.cpp`](tests/VehicleMaterialSelectionProbe.cpp), [`AssetRequestManagerProbe.cpp`](tests/AssetRequestManagerProbe.cpp); visual gaps: [incomplete visual checks](Docs/SCREENSHOT_VALIDATION_PLAN.md#incomplete-feature-visual-checks) |
| Map, terrain, scenery, and roads | Partial: map-wide traffic, lane, and attachment indexes are still built at startup, but terrain, scenery, water, and road render assets are loaded synchronously only as tiles enter the active window. Loaded render assets stay cached after leaving the window for faster re-entry, so memory can grow as more of the map is explored. Startup parses all tile text for the global indexes; active tile content is parsed again on demand. Collision mesh preparation runs in a CPU worker and ODE publication stays on the simulation thread, limited to one tile per update. Startup window creation and initial collision warm-up remain synchronous. Static text/binary/MSZIP `.x` meshes and distance-selected scenery LODs now have runtime paths; full OMSI attachment/profile semantics and end-to-end visual/contact validation remain incomplete. | [`MapConfigLoader.cpp`](src/map/MapConfigLoader.cpp), [`MapSplineGeometry.cpp`](src/map/MapSplineGeometry.cpp), [`MapRoadGeometry.cpp`](src/map/MapRoadGeometry.cpp), [`MapCollisionBuilder.cpp`](src/map/MapCollisionBuilder.cpp), [`MapCollisionStreamer.cpp`](src/map/MapCollisionStreamer.cpp), [`MapRenderer.cpp`](src/render/map/MapRenderer.cpp), [`SceneryObjectConfigLoader.cpp`](src/config/SceneryObjectConfigLoader.cpp) | [`MapConfigLoaderProbe.cpp`](tests/MapConfigLoaderProbe.cpp), [`MapCollisionBuilderProbe.cpp`](tests/MapCollisionBuilderProbe.cpp), [`MapCollisionStreamerProbe.cpp`](tests/MapCollisionStreamerProbe.cpp), [`MapSplineGeometryProbe.cpp`](tests/MapSplineGeometryProbe.cpp), [`MapRoadGeometryProbe.cpp`](tests/MapRoadGeometryProbe.cpp); in-context visual validation remains |
| Scripts and dynamic displays | Vehicle script/text textures and scenery per-placement script/text texture surfaces, `init`/`frame`, and `[visible]` control are active; full OMSI behavior remains partial. | [`ScriptRuntime.cpp`](src/scripting/ScriptRuntime.cpp), [`MapRenderer.cpp`](src/render/map/MapRenderer.cpp) | [`ScriptTextureRuntimeProbe.cpp`](tests/ScriptTextureRuntimeProbe.cpp) |
| Weather | Partial first slice: configurable exterior rain/snow particles use the shared `PrecipType` and `PrecipRate` system variables; map weather schedules, fog, windshield effects, and accumulation are not implemented. | [`WeatherEffects.cpp`](src/render/WeatherEffects.cpp), [`RenderLoopDraw.cpp`](src/app/RenderLoopDraw.cpp) | [`WeatherEffectsProbe.cpp`](tests/WeatherEffectsProbe.cpp); rain and snow visually validated in app captures |
| Sound | Player sound runtime is active; AI vehicles select `[sound_ai]` when configured and otherwise fall back to `[sound]`. Full OMSI sound and AI event coverage remain incomplete. | [`VehicleSoundBank.cpp`](src/audio/VehicleSoundBank.cpp), [`VehicleConfigLoader.cpp`](src/config/VehicleConfigLoader.cpp) | [`SoundEngineProbe.cpp`](tests/SoundEngineProbe.cpp), [`VehicleConfigProbe.cpp`](tests/VehicleConfigProbe.cpp) |
| AI traffic, pedestrians, and routes | Partial runtime: asynchronous `.ovh` vehicle construction and model/script loading, asynchronous pedestrian `.hum`/model CFG and mesh loading, scheduled map buses, group-scoped path traffic rules (including `no_unsched_traffic`) for unscheduled road vehicles, provisional right-before-left precedence at equal-priority straight crossings, route-end retirement, sidewalk pedestrians, fixed-cycle plus request-aware SCO signal timing, route-based AI turn indicators, and `unsched_trafficdens.txt` group/day curves for group-specific active targets are active. Pedestrian populations use passenger-density schedules on human paths; vehicle `[rule]` restrictions target unscheduled groups and do not apply to scheduled buses or pedestrians. Group quotas change over time with safe off-screen admission/retirement; the maximum reserved pool is fixed at startup, and unfilled reservations are retried as connected road lanes stream in. AI driver models, lane changing/overtaking, and full OMSI parity remain incomplete. | [traffic runtime](src/map/MapTrafficRuntime.cpp), [map configuration](src/map/MapConfigLoader.cpp), [render loop](src/app/RenderLoop.cpp) | [MapTrafficRuntimeProbe.cpp](tests/MapTrafficRuntimeProbe.cpp), [MapConfigLoaderProbe.cpp](tests/MapConfigLoaderProbe.cpp), [AssetRequestManagerProbe.cpp](tests/AssetRequestManagerProbe.cpp); see the [completion TODO](TODO.md) |

For a consolidated list of configuration keywords with incomplete support,
including bus model CFG keywords selected by `OPENBUS_MODEL_CONFIG`, see the
[configuration keyword gap index](Docs/UNIMPLEMENTED_CONFIGURATION_KEYWORDS.md).

### Map runtime scope

Startup progress is written to the console and `game.log`, including
configuration loading, renderer initialization, and map phases. Each log entry
includes elapsed milliseconds, a severity (`TRACE`, `DEBUG`, `INFO`, `WARNING`,
or `ERROR`), and its module, for example `[123] [WARNING] Game Texture missing`.
By default, `DEBUG` and more severe messages are written. Set
`OPENBUS_LOG_LEVEL=TRACE` to include detailed sound-source and per-tile map
messages, or use `DEBUG`, `INFO`, `WARNING`, or `ERROR` to set another minimum
severity. Set `OPENBUS_LOG_FILE` to change the log file path (the file is
replaced at startup). If the log file cannot be opened or written, the logger
reports the failure on standard error and continues writing entries to the
console. Periodic map progress summaries remain at `DEBUG`; detailed tile
messages include the current file and counts. These logs do not require
performance tracing.

Interactive startup opens an ImGui setup screen. It lists maps with a
`global.cfg` under the configured OMSI root and `.bus` vehicles under
`Vehicles`. Choose a vehicle by its `[friendlyname]` manufacturer and type, then
choose its default appearance or a discovered `.cti` livery, a timetable HOF
source, and a configured number/registration. CTI texture replacements apply to
the selected vehicle, and the default paint uses a matching CTI item when one is
available. The **Timetable HOF** setting can automatically choose the
installed HOF that matches the most map timetable routes, including HOFs shipped
with other bus models; a manual override can select any installed HOF or disable
HOF loading. The selected HOF path and name are exposed to vehicle scripts as
`hof_file` and `hof_name`. Some vehicle scripts may assume a particular HOF
format, so use the manual override if an automatically matched HOF does not work
with that bus. The bus picker defaults to **Spawn in bus**; turn it off to leave
the bus parked and keep walking. Choose a map entrypoint for the bus, defaulting
to the startup menu's selected entrypoint. When on foot, enter a nearby parked bus
using the **Enter bus** button; use **Leave Bus** in the pause menu to get out.
The pause menu can also teleport the bus to the selected entrypoint whenever no
timetable trip is active. After a bus is spawned, its **Player timetable**
panel offers **Drive a scheduled trip**. Select a line, tour/course, trip, and first
stop, then **Start selected trip**. Available routes match the selected HOF's
ordered stop sequence (stop identifiers or display names): the map may include
extra stops between HOF-listed stops, or a shorter map trip may be an ordered
subset of the HOF route. Longer routes with different stop spellings or detours
can also match when their endpoints and most ordered stops agree. A two-stop
HOF sequence for a longer trip must match its endpoints rather than just a
shared section. If an HOF route has no stop list, it is available only
when its line and destination/terminus match the map route. Public line numbers
come from the HOF, except that an `XXX` placeholder uses the map's public line
when available. Departure times come from the map timetable; HOF files do not
supply departure times, and the trip picker sorts departures by their scheduled
start times. Date-active Chrono timetables replace deactivated lines
and add or override timetable data for both player and AI services, so available
lines can change with the selected start date.
Player routes come from the map's schedules and known stop definitions,
independently of complete station links, AI road-route construction, AI depot
buses, or an active AI bus population. Missing or ambiguous stop definitions
are reported in the log. The panel warns if stop locations cannot be resolved:
the trip can still be started, but those stops cannot be serviced.
During an active trip, the outer amber ring marks the 50 m stop-visit radius
centered on the matching editor-only bus-stop object; entering it advances the
timetable independently of passenger boarding. If stops are close together,
the bus must leave the previous visit radius before another stop can count.
The editor-only object can locate a timetable stop even when it has no route
pose. The inner ring marks the
passenger-service area, which still requires slowing near the stop but does not
require opening doors. Passenger queues use authored waiting areas when
available and are generated beside the stop otherwise. The trip HUD reports
onboard passenger count and warns when human models are unavailable, in which
case boarding can proceed without visible passenger actors.
**Cancel selection** closes setup, **Cancel trip now** immediately
releases the assigned trip and its AI replacement policy, and **Stop driving after
this trip** ends the assignment at the terminus. Course progression also stays
within the bus's HOF. Choose a map and one of its entrypoints; the date/time
defaults to the host's current local date and time. **Start in bus** enters the
selected vehicle immediately, while **Start on foot** lets the player enter it
later. Use **WASD** to walk relative to your view and **Space** to jump while
grounded. **F4** toggles mouse look and releases the cursor again to use the
on-foot panel. Walking uses gravity and a collision capsule against terrain,
roads, and static scenery; it is not the bus's flying debug camera. Map and
collision streaming follow the walker. The hidden bus's collision outline is not
drawn on foot. Use the in-game **Spawn selected bus** button to enter the chosen
vehicle at the selected entrypoint. Interactive vehicle selection uses each
`.bus` file's own `[model]` entry; `OPENBUS_BUS_CONFIG` and
`OPENBUS_MODEL_CONFIG` remain available for benchmark/custom runs. Benchmark
mode bypasses the interactive menu and continues to honor the
environment-selected map, spawn, and `OPENBUS_MAP_AI_HOUR` values. ImGui manages
the cursor during setup and on foot; after a bus is spawned, the game manages
the steering crosshair and clickable-control cursors. Bus physics stays frozen
while on foot and resumes when the bus is spawned.

Startup parses tile text once and caches the parsed tile records/global
placement-attachment index, then constructs render geometry only for listed
tiles in a two-tile Chebyshev window around the selected spawn point (up to 5×5
tiles). It uses the first `[groundtex]` entry as the base and applies numeric
per-tile `.map.<index>.dds` alpha masks to subsequent `[groundtex]` layers. Set
`OPENBUS_MAP_GROUNDTEX` to a zero-based `[groundtex]` index to preview that
texture uniformly, bypassing per-tile masks. Detail-layer and seasonal selection
remain unsupported. Terrain, road, and scenery geometry and their window-owned
GPU textures/buffers are created only for that active window. On a tile
crossing, a background worker builds and uploads a complete replacement window
using a GLFW shared OpenGL context; the current window remains drawable until
the replacement is ready, then its GPU resources are released. If shared-context
creation fails, loading falls back to the synchronous path. Terrain and roads
then use distance/frustum rejection, scenery uses spatial chunks and
per-instance culling, and `[visible]` is checked per model part. Scenery
runtimes are initialized/ticked only while their source tile is active; their
per-placement script state survives window reloads. Scripts are not paused
merely because the camera frustum excludes them. Tile crossings still rebuild
the full active window (rather than only the entering tile), but construction
and upload happen on the worker. Initial map window construction at startup
remains synchronous. Scenery script runtimes created by a streamed window are
initialized on the main script thread, at most 32 new runtimes per frame. Parsed
tile records and the global placement index remain CPU-cached for cross-tile
attachment resolution. GPU contention and boundary latency have not been
measured.

ODE terrain and road/scenery collision is loaded separately around the player:
the load radius is one tile, and collision remains resident until a tile is more
than two tiles away, but only for tiles with successfully uploaded render
geometry in the current window. Terrain and road/scenery collision meshes are
prepared on a background CPU worker; ODE objects are published/removed on the
simulation thread, at most one completed tile per update. Thus collision is
independent of the camera frustum and scenery `[visible]`, and collision
residency also updates if the resident geometry set changes while the player
remains in the same center tile. The initial collision neighborhood is warmed
synchronously before gameplay; runtime collision preparation is asynchronous,
but ODE setup can still cost time on publication. If preparation falls behind
the player, temporary contact gaps are possible. Physics residency follows
loaded map tiles, not camera visibility.

Random map road traffic uses `.ovh` and `.bus` entries from supported
`[aigroup_2]` groups. `Normal`, `NormalCar(s)`, and `Taxi` use normal-car path
rules; `Emergency` and `Ambulance` use ambulance rules; `Trucks` and
`Commercial`/`Commercials` use their corresponding rules. Other groups and
legacy vehicle-list formats are not supported by this runtime. Depot `.bus`
entries continue to provide scheduled AI buses on timetable routes.
`OPENBUS_MAP_AI_COUNT` sets the density-scaled road-vehicle base population
(default `200`, maximum `1000`). Trucks are permitted on road paths by default
unless the map's path rules explicitly prohibit them; configured group and path
density rules still apply. For mirrored road splines, OpenBus flips
unidirectional lane directions along with their lateral positions, including
one-way carriageways. Bidirectional and pedestrian paths keep their authored
directions. This correction is an OpenBus compatibility policy, not verified
native OMSI behavior. Set `OPENBUS_MIRRORED_ROAD_DIRECTIONS=authored` to restore
the previous behavior for maps that intentionally use mirrored directions;
`correct` is the default. Connected type-0 road paths authored by scenery
objects also contribute traffic lanes and spawn positions. These point-backed
paths appear as unlabeled lines in the F2 path overlay, unlike spline routes
labeled `S<ID>` or `H<ID>`. The selected start date/time drives the in-game
traffic clock's `Time` system variable, which advances using elapsed simulation
time. The selected date sets the system variables `Day`, `Month`, and `Year`;
`DayOfYear` is computed from that date. Traffic weekday schedules derive their
starting weekday from this same calendar (Sunday `0` through Saturday `6`), and
the date advances with simulation time across midnight and year boundaries.
Chrono tile overrides are currently disabled; the base map tile files are used
regardless of the selected date. Chrono timetable scenarios are selected at
startup and are not reapplied when the simulation date advances. The setup
screen defaults to the host's local date and time. Selector-based
`[selobject]`/`[selspline]` edits are not yet applied. The current traffic clock is previewed as `TIME HH:MM` in the status
HUD. Road and pedestrian populations follow their map density schedules as the
clock advances, with active population targets recalculated once per simulated
minute rather than every rendered frame. A pool sized for each schedule's
maximum is queued at startup; vehicle instances are constructed and their
scripts loaded on asset workers, while model mesh parsing is deferred to
asynchronous requests. For unscheduled traffic, separate group peaks can require
a queued pool larger than any simultaneous target. There is no fixed active
population cap; available pool capacity and safe spawn positions can still limit
the active population. Active vehicle meshes use the per-frame upload budget in
order of distance to the player, so nearby traffic is prioritized over farther
vehicles. `OPENBUS_MAP_TRAFFIC_DENSITY_MULTIPLIER` scales the map's scheduled
density factor for road vehicles, buses, and pedestrians (default `1.0`, maximum
`1000.0`). The target is the base population times the map's density factor
times this multiplier; high values can request very large pools and consume
substantial resources. The `run.bat` testing launcher defaults it to `10.0`, and
`profile.bat` sets it to `100` for denser profiling traffic; `run.bat` respects
a value already set in the environment; set it to `1.0` for normal scheduled
density. Pedestrian `.hum` and model CFG parsing, mesh parsing, and texture
decoding also use workers. OpenGL uploads and script initialization remain on
the main/render thread. Road vehicles and buses wait in an inactive queue until
their model and scripts are ready. Road traffic may be admitted at clear,
off-screen interior positions on connected lanes; scheduled buses remain
restricted to connected endpoints or their authored timetable origins. Candidate
tile ownership is derived from each sampled world position, so interior
positions on splines crossing tile boundaries can be admitted when their actual
tile is active. Isolated splines and terminal dead ends are excluded from
road-traffic spawn candidates. Pool construction first prioritizes routes with
spawn candidates in the current active tile window for the initial target
population, then fills remaining capacity with map-wide routes; every route is
checked for an active-tile candidate before local priority admission. Among
eligible locations, the worker balances proximity to the player against nearby
traffic, and prefers ordinary road interiors over SCO junction paths and
approaches close to signal stop lines. Positions near actual network entry lanes
receive a modest preference. These are preferences, not additional hard
exclusions. Spawn proposals in the same population plan reserve their positions
so subsequent vehicles can choose different locations. Scheduled bus origins are
unchanged. The runtime active-tile guard remains in force. Admission requires a
clear vehicle footprint, an entry outside the main camera frustum, and at least
50 m clearance from the camera (plus the vehicle radius). Density reductions
remove surplus vehicles only off-screen. Vehicles are retired when they reach an
open route end; subsequent spawns still obey the off-screen and clearance
checks. For visual debugging, set `OPENBUS_MAP_AI_SPAWN_VISIBLE=1` to allow road
AI vehicles and scheduled buses to spawn while the camera can see their entry
positions. Active-tile and vehicle-clearance checks still apply; the default
remains off-screen-only. `.ttl` timetable departure times are parsed and linked
to `.ttp` trips. Matched buses wait for their next departure at or after the
configured starting hour before endpoint admission, including midnight rollover.
Admission waits until the endpoint tile is active, the endpoint is outside the
camera views, and its vehicle-sized clearance is available; a due bus remains
queued while any of those conditions is false. When multiple pooled buses serve
a route, its departure list is partitioned so each bus gets a disjoint subset of
departure times. Timetable `.ttr` entries retain repeated scenery-object IDs and
their selected path indexes, so scheduled route validation checks the complete
ordered path rather than matching only shared road sections. Repeated timetable
checkpoints on an ordinary map spline are matched to that spline's single route
segment, while repeated placed-scenery object IDs retain distinct path
selectors. Exact timetable-selected scenery paths can bridge small authored
vertical offsets (up to 2.5 m) when their horizontal endpoints and headings
align; ordinary road-traffic connectivity keeps its stricter limits. Cotterell
startup validation has recognized complete routes for line 12, U28, and one U18
trip; other trips may be explicitly logged as partial timetable-origin prefix
fallbacks when the map graph cannot reproduce their full authored sequence. A
departure that exactly matches the configured start hour is eligible
immediately, including when the hour is represented as a decimal value. A live
Cotterell check confirmed a minute-483 bus departs once its endpoint tile is
active, the entry is clear, and the camera no longer sees it. A one-bus run then
confirmed route progress at 100 m intervals through 700 m and arrival at the
open route end after 785.68 m. Pedestrian model choices come from the map's
`humans.txt`; `drivers.txt` is reserved for AI driver models and is not used as
a pedestrian roster. Pedestrians follow the authored human-path offsets and
heights, render the configured `.hum` model meshes and textures, and wait at
scheduled pedestrian crossing paths while their assigned signal phase is not
green (states 6-8). Procedural gait animation, including lowering outstretched
arms, is applied to loaded pedestrian meshes; full skeletal skinning and
`[links]` behavior are not implemented. OMSI `[setbone]` records, including
signed pedestrian bone indices, are accepted without applying skeletal
transforms. Active pedestrian traffic actors also occupy collision footprints,
so road AI stops/yields rather than sweeping through them. Pedestrians likewise
pause before occupied vehicle footprints and resume when clear; this is
kinematic avoidance, not physical collision response. Vehicles follow the lane
offsets, heights, and effective travel directions shown by the F2 overlay,
including scenery junction paths. Matching lane endpoints join within 0.5 m
horizontally and vertically; opposite-direction lanes are not used as a forced
return route. Road vehicles make reproducible seeded choices among compatible
straight, left, and right movements. Turns tighter than the vehicle's minimum
turning radius are rejected; authored radius data is used when present,
otherwise a conservative class/wheelbase estimate is used. Genuine loops
continue forward without reversing. Conflicting junction paths use
swept-footprint reservations; authored path/group priority takes precedence. AI
vehicle bodies follow the sampled road surface in all three rotations: yaw
tracks the route at the rear axle, pitch follows the authored grade, and roll
follows road cant. Pitch and roll use the vehicle's heading-relative axes so
uphill travel raises the nose regardless of compass direction. Lane positions
include the authored road skew and height offsets. The configured rearmost axle
follows the route and the vehicle body pivots around it; when axle data is
unavailable, the rear axle is estimated at 30% of the configured bounding-box
length behind the model origin. AI collision footprints and their F2
visualizations use the same body pose. At equal priority, straight crossing
approaches provisionally give precedence to the vehicle approaching from the
right. Once either approach has waited 3 seconds, that preference yields to the
normal tie-breakers to avoid persistent multi-approach cycles. Other
equal-priority movements order straight-like paths (under 45°) before
45–100° bends and sharper turns; faster approaches (under 3 m/s, under 8 m/s,
or more, based on cruise speed so braking does not reverse the ranking) go
before slower ones, then earlier junction arrival and wait age break ties.
Yielding vehicles leave an additional 2 m before conflict points. Right turns
yielding to straight-through traffic stop 4 m before the junction entry when
needed, keeping the waiting vehicle clear of the crossing path so through
traffic can pass instead of deadlocking against it. Synthetic regressions verify
that a waiting
lower-priority movement yields to an approaching higher-priority vehicle,
crossing movements remain clear, and both make progress. When a long turning
vehicle's front extends into the junction while its center is still alongside a
straight-going vehicle, it can yield to that vehicle if priority and the sampled
stopping distance permit; physical collision checks still apply. Complete native
OMSI right-of-way behavior remains unverified.

Traffic accelerates at up to 1.6 m/s^2 and normally brakes at up to 3 m/s^2. If
a signal becomes blocking inside a moving vehicle's braking distance (`speed^2 /
(2 * 3 m/s^2)`, measured to the front-clearance stop point), the vehicle commits
to passing that signal rather than stopping instantly. The decision lasts until
it enters the signalized path; following, junction priority, and collision
checks still apply. Vehicles with enough room to stop, and vehicles already
waiting at red, continue to obey the signal. Lookahead reduces speed before
bends only when the current speed would exceed the curve's safe speed, estimated
with a 4.0 m/s^2 lateral-acceleration limit. A speed-dependent following headway
(0.5 s at walking pace rising to 1.5 s at 13 m/s) plus a 2 m standstill buffer
respond to vehicles ahead, including the player bus, so slow and stopped queues
close up to about 2–3 m while faster traffic keeps a longer gap. Oriented
vehicle footprints and small swept movement steps prevent AI from advancing
through occupied space; an emergency stop can exceed comfortable braking when a
blockage appears too close. A vehicle can continue when its next swept step
increases separation from an external obstacle, so a pedestrian close behind
does not hold it in place while it moves away. Likewise, a close vehicle behind
cannot hold a leader that is moving away; physical separation is still checked
throughout the swept movement. Once a vehicle is committed to a junction, the
swept collision margin is reduced while still preventing physical footprint
overlap. All committed vehicle turns use a 0.15 m clearance from external
vehicle footprints instead of the normal 0.5 m, allowing close, collision-free
turns when the configured bounding boxes are conservative. Vehicles in the same
lane, moving or stopped, are followed with a damped gap model instead of being
treated as parked obstacles, so a queue keeps flowing through a junction behind
a moving leader and creeps up behind a stopped one. Before entering a junction,
a vehicle claims it and waits at the entry line while any vehicle inside or
already admitted would cross or merge into its path, so two approaches never
lock each other inside the box. Straight, separated opposing lanes can pass
without treating the whole junction as a single shared corridor; uncertain
geometry retains conservative checks. A vehicle approaching a junction
constraint below 2 km/h settles to a stop within 0.25 m of its stopping point
instead of creeping indefinitely. A vehicle approaching on a lower-priority
spline gives way at its entry line to any higher-priority vehicle whose path
crosses or merges into its junction movement and would arrive within the same
conflict window (plus a 0.75-second safety margin accounting for vehicle
occupancy); traffic that has already cleared before the turn arrives does not
hold it back; higher-priority vehicles do not wait for such a vehicle's unused
claim. Waiting time orders junction admission among vehicles of equal priority.
Approach priority uses the lowest priority across the current spline and
connected route splines within 150 m, so short low-priority connectors still
affect right-of-way. If a collision cycle nevertheless traps two vehicles inside
a junction, the lower-priority vehicle backs out to the approach when its swept
path is clear, then waits for the other vehicle to exit. Crossing movements use
time-aware three-second lookahead reservations; road-sign right-of-way rules
beyond authored spline priority are not yet modeled. Queues, actual endpoint
spawns, junction entry speeds, off-screen population unloads, route-end
recycling, and lane-network statistics are logged. Route planning tries an
alternative branch if a later turn is impossible, rejecting the entry route if
no drivable exit or loop is reachable. Vehicles exit only at unconnected lane
ends; missing or misaligned authored connections can still terminate a route. AI
buses are selected separately from depot bus definitions. When a connected route
matches a timetable's complete `.ttr` spline chain, buses follow that route and
use its parsed departures; complete matches take priority over fallbacks. If no
complete match exists, OpenBus can use an explicitly logged,
itinerary-inaccurate fallback only when it can follow at least two consecutive
timetable entries from the trip's first checkpoint. Such a bus follows only the
matched prefix, stops at that prefix's end, and does not register player
passenger stops as if the full itinerary were available. Fallbacks do not skip
missing checkpoints or invent connections. They start at the timetable origin
and still wait for a clear, off-screen admission position. Set
`OPENBUS_MAP_BUS_COUNT` to control their population (default `20`, maximum
`200`; `0` disables map AI buses). The bounded bus pool is created at startup
and buses are admitted at scheduled times. Traffic remains lane-constrained and
kinematic: linked SCO signal timing and stop/jump controls are active, but
native stop-line/sign behavior is incomplete. Safe overtaking and lane changes
are not implemented; opposing-lane passing is not inferred. Articulated runtime
is outside the current supported scope, and physical collision response when
the player drives into AI is not implemented.

Road and timetable route networks are indexed from every tile listed in
`global.cfg` at map startup. AI vehicles retain those predefined lane routes
while rendering tiles stream in and out around the player; crossing a tile
boundary does not truncate or rebuild an actor's route.

Ordinary scenery gets a bilinearly sampled terrain height plus its authored
vertical placement value, unless its SCO/model declares `[absheight]`, in which
case the authored vertical coordinate is absolute. Same-tile spline
attachments/repeaters use typed spline-relative placement; cross-tile chains are
not complete. `[attachObj]` children resolve their parent placement IDs across
map tiles and use the indexed `[new_attachment]` point from the parent SCO/model
config, including the authored anchor and child rotations. Missing
parents/anchors and attachment cycles are skipped with diagnostics.
`[varparent]` references are resolved against map placement IDs. Traffic-light
models marked `[trafficlight]` inherit the signal phase schedule from their
referenced parent so their scripts can drive the visible lamps. When a parent
defines multiple signal groups, the numeric placement label selects the matching
group; uncovered time at the end of a group's cycle is phase `12` (off). SCO
`[traffic_light_stop]` and `[traffic_light_jump]` entries use the native
`light, time, if_request[, jump_to]` layout. A stop holds the shared cycle
clock while its request condition is active; a jump moves that clock to
`jump_to`. A true `if_request` activates when no traffic actor requests the
light; false activates when one does. Vehicle, pedestrian, and player
requests use `[approachdist]`; a zero-distance approach requests only at the
line while stopped. A zero `[traffic_lights_group]` cycle falls back to the
longest configured light schedule, and the cycle starts from the selected map
time. State `12` remains dark and is permissive for vehicles. Signal requests
and stop-line placement are still approximations pending native OMSI
validation. Other scenery-script variable inheritance is not evaluated, and
`[varparent]` is not a spatial attachment link. `[spline_terrain_align]`
currently tilts marked scenery so its local up axis follows the sampled terrain
normal, without changing its position. OMSI's exact semantics for this marker
have not been independently verified, so this is a conservative renderer
interpretation; `[spline_terrain_align_2]` remains unsupported. Scenery
`[texttexture]` and `[scripttexture]` materials are bound per placed instance,
with independent live texture state and transparent text blending; visual parity
against installed directional-sign assets still needs validation. Supported
scenery meshes include O3D and static text, binary, and MSZIP-compressed `.x`;
configured nonzero LOD parts are selected by distance using the vehicle LOD
threshold mapping. Animated/skinned `.x` content remains unsupported, and
`[tree]` assets use a crossed-texture-plane preview rather than their helper
mesh. `.sli` rendering parses a subset of `[texture]`, `[profile]`, and
`[profilepnt]` records and tessellates straight and signed-radius centerlines,
but complete road/profile, banking, material, and lane-marking behavior is not
established. Road UV V uses a valid parsed v11+ spline chain offset as its
longitudinal origin; visual continuity and OMSI parity still need confirmation
against linked installed splines.

The following map behavior is **not implemented** or remains explicitly partial:

- Animated/skinned DirectX `.x` content; static text, binary, and
  MSZIP-compressed `.x` meshes now load. The tree preview is not the original
  mesh. Cross-tile repeater-master resolution, full `[attachObj]` scripting,
  scenery script-driven animation/material changes, scenery mouse events, and
  attached/repeater scenery collision are incomplete. Per-placement scenery
  `init`/`frame`, `[visible]`, `[usetexttexture]`, and `[usescripttexture]`
  surfaces are active.
- Full `.sli` semantics and visual validation of curved sidewalks, road seams,
  and lane markings. Road-profile textures/material alpha, cant/banking, and
  texture phase across linked splines need representative asset tests. Parsed
  v11+ chain offsets now affect road texture V, but do not resolve links or
  prove visual parity.
- Supported `.sli` profile-strip road geometry and SCO/model scenery collision
  are connected to ODE for tiles within one tile of the bus. Scenery prefers
  `[collision_mesh]`, falls back to `[boundingbox]`, then to first-LOD O3D model
  geometry; `[nocollision]` is honored unless the SCO also declares `[surface]`.
  Terrain and road/scenery collision are unloaded beyond a two-tile radius, and
  overlapping road/terrain contacts still need tuning.
- Per-tile/detail/seasonal ground-texture rendering, water rendering, tile
  lightmaps, Chrono map variants, and asynchronous map construction are absent.
  Per-tile DDS filenames are inventoried, but their payloads and mapping to
  `[groundtex]` ordinals are unresolved. All map geometry stays resident after
  construction; active-window gating does not free GPU memory. `.map.water`
  records are size/finite-value validated and reported, but their spatial height
  ordering is unresolved, so no water geometry is drawn. Unimplemented bracketed
  sections in tile files and `global.cfg` are reported with line numbers during
  map loading. Global identity, description, seasons, and other documented
  metadata are retained; road/passenger density points drive provisional,
  separate AI population targets. `[variable_terrain]` and
  `[variable_terrainlightmap]` markers are recognized and reported, but their
  dynamic terrain/lightmap behavior is not rendered. `[varparent]` and
  spline-terrain alignment data remain partial. `[rule]` and `[kill_rule]` apply
  supported group-specific speed, density, priority, and vehicle-access settings
  to road paths; see the [OMSI map format
  reference](Docs/OMSI2_MAP_FORMAT_REFERENCE.md). The separate
  `unsched_trafficdens.txt` group/day schedule applies its hourly curves to the
  road-traffic population target, startup pool composition, and per-group active
  targets. The maximum reserved pool is fixed at startup, but unfilled
  reservations are retried as connected road lanes stream in. Weekday schedules
  use the selected map calendar and change day at its midnight, rather than 24
  hours after startup.
- Scheduled OMSI AI buses use matched timetable routes and departures; fixed SCO
  traffic-light schedules stop road traffic and pedestrians on linked paths.
  Signalized scenery instances expose the active phase through
  `trafficlightphase` for model scripts/visibility variables. Visible light
  heads linked by `[varparent]` retain their selected controller schedule even
  when the junction controller is `[onlyeditor]`; its geometry remains hidden.
  The GPU traffic-light probe verifies red, green, yellow, and back to red for
  fixed-cycle and controller-driven schedules through the placement loader,
  model script, visibility, and rendered pixels. Conditional SCO signal
  controls use aggregated traffic requests; `signalroutes.cfg` routing,
  model-specific lamp animation, and full in-game OMSI parity remain
  unverified.

The existing centerline probe validates linked endpoints and signed curve
directions, but does not validate cross-section profile placement, sidewalks,
markings, or visual seams. The [map TODOs](TODO.md#grande-porto-map-support)
track the missing runtime paths and visual evidence.

## Source layout

The engine is split into one static library per module (see
`src/CMakeLists.txt`). Headers are included by module-qualified paths, e.g.
`#include "render/CoreRenderer.h"`.

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

Large subsystems keep their public header small and place implementation details
in an internal header (`*Internal.h` / `*Impl.h`) shared by focused translation
units:

- `app/RenderLoop*.cpp`: lifecycle, input/camera, simulation/scripts, drawing.
- `render/map/MapSceneLoader.*`: map scene construction (terrain, scenery
  assets, tile content); `MapRendererDraw.cpp`: per-frame map drawing.
- `scripting/Script{RuntimeImpl,Textures,NativeBackend,LuaBindings}.cpp`:
  runtime core, script textures, native OSC backend and Lua bindings.

Vendored code lives in `third_party/` (Lua 5.1, sol2) and is built separately
from engine modules.

## Prerequisites

- CMake 3.21 or newer (required for the configure-preset schema)
- A C++23 compiler
- zlib
- ODE
- GLFW and OpenGL
- OpenAL
- Lua 5.1 on Linux (Windows uses the bundled Lua 5.1 runtime)

On Windows with vcpkg, install standalone LLVM (for `clang-cl.exe`) and the
Visual Studio C++ build tools (for the MSVC headers, libraries, and linker):

```powershell
vcpkg install ode:x64-windows glfw3:x64-windows openal-soft:x64-windows zlib:x64-windows
```

Set `OPENBUS_VCPKG_ROOT` to the vcpkg installation directory when it is not
`C:\vcpkg`. The Windows helper discovers Visual Studio C++ tools, the Windows
SDK, LLVM on `PATH`, and Ninja where possible; set `OPENBUS_CLANG_BIN`,
`OPENBUS_CLANG_COMPILER`, `OPENBUS_VS_ROOT`, `OPENBUS_MSVC_ROOT`,
`OPENBUS_WINDOWS_KIT`, `OPENBUS_WINDOWS_KIT_VERSION`, or `OPENBUS_NINJA` to
select specific installations.

On Debian or Ubuntu, install the native development packages:

```bash
sudo apt install build-essential cmake libode-dev libglfw3-dev libglu1-mesa-dev libopenal-dev liblua5.1-0-dev zlib1g-dev
```

The Linux build uses the system GLFW/ODE packages when available and falls back
to downloading GLFW during CMake configuration when GLFW is not found.

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

Native MSVC is rejected because its CMake C++23 mapping is `/std:c++latest`, not
an exact C++23 mode. The Windows `run.bat` helper initializes the MSVC x64
environment and uses standalone Clang-cl with the Visual Studio-bundled Ninja
executable. Its `windows-clang-ninja` preset builds in `build-ninja` using 12
parallel jobs; the original NMake `build-ode` directory remains untouched. The
first Ninja build compiles from scratch, while subsequent builds recompile only
changed sources. For a manual NMake configure from `cmd.exe`, the helper's
environment variables and preset paths can be overridden before invoking it:

```bat
call windows_clang_env.bat
cmake -S . -B build-ode -G "NMake Makefiles" -DCMAKE_CXX_COMPILER="%OPENBUS_CLANG_COMPILER%" -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE="%OPENBUS_VCPKG_ROOT%/scripts/buildsystems/vcpkg.cmake"
cmake --build build-ode --target OpenBus
```

Release and RelWithDebInfo builds enable interprocedural/link-time optimization
for `OpenBus` by default. Disable it when comparing profiler call boundaries or
when using a toolchain that does not support IPO:

```bat
call windows_clang_env.bat
cmake -S . -B build-ode -G "NMake Makefiles" -DCMAKE_CXX_COMPILER="%OPENBUS_CLANG_COMPILER%" -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE="%OPENBUS_VCPKG_ROOT%/scripts/buildsystems/vcpkg.cmake" -DOPENBUS_ENABLE_IPO=OFF
```

On Linux, use a native build directory and the provided shell helpers:

```bash
chmod +x run.sh profile.sh screenshot.sh convert.sh
./run.sh
```

The helpers use `build-linux` by default. Set `BUILD_DIR` to choose another
build directory. Set `OPENBUS_BUS_CONFIG` to the OMSI-relative bus configuration
path to select a vehicle.

### Steam presence on Windows and Linux

OpenBus uses the Steamworks SDK 1.65 redistributable supplied in
[`third_party/steamworks/redistributable_bin`](third_party/steamworks/redistributable_bin)
to request OMSI 2 (app ID `252530`) presence. The build copies the matching
Windows x64 or Linux x64/arm64 runtime next to the executable; OpenBus loads it
from there rather than searching other installed games. On unsupported
architectures, including 32-bit builds, OpenBus runs without Steam presence.

To disable Steam presence, set `OPENBUS_STEAM_PRESENCE=0`. The previous
`OPENBUS_STEAM_ROOT` and `OPENBUS_STEAM_API_LIBRARY` discovery settings are
no longer used. Loading the bundled native library still executes code in the
OpenBus process; keep the project and build directories protected from
untrusted modifications. OpenBus does not verify its signature or hash at
runtime.

Steam must be running as the same user and the active account must own OMSI 2.
OpenBus sets `SteamAppId`
and `SteamGameId` to `252530` in its own process and restores their previous
values at shutdown; a `steam_appid.txt` in the working directory, if present,
must contain only `252530`. No files in the OMSI installation are modified.

OpenBus checks the app ID returned by Steam after initialization and runs
callbacks during the setup screen and normal frames. Startup loading may block
callbacks temporarily. Failures are logged under `Steam` and do not prevent
OpenBus from running. Successful initialization is not proof that Steam displays
the expected presence: check the Steam client while OpenBus is running.

## Running all test probes

Probe executables are excluded from the default build. Run and profile helpers
build only `OpenBus` and its dependencies.

On Windows, run `test.bat`, or select **Tasks: Run Test Task** in VS Code and
choose **Build and run all probes**. The helper configures the
`windows-clang-ninja` preset, builds all test probes, and runs the full CTest
suite with failure output. Build or test failures return a nonzero exit code.
The preset consumes the installation paths prepared by `windows_clang_env.bat`;
use that helper in the same `cmd.exe` session before invoking CMake presets
manually. You can override discovered installations with the environment
variables listed above.

For any configured build directory, the equivalent cross-platform command is:

```sh
cmake --build <build-directory> --target check-probes --parallel 12
```

The target builds probe dependencies before running tests, including probe
variants registered with CTest. GPU probes may be reported as skipped when their
required graphics context is unavailable. The opt-in bgfx lighting preview is a
developer tool, not part of the automated test suite.

## Linting

Install LLVM so `clang-tidy` is available on `PATH`, then run:

```powershell
.\lint.bat -DCMAKE_TOOLCHAIN_FILE=C:/path/to/vcpkg/scripts/buildsystems/vcpkg.cmake
```

This uses the separate `build-lint` directory and enables the project’s
`.clang-tidy` checks without changing the normal `build-ode` configuration.
Additional CMake arguments are passed through and can override the helper's
defaults.

To format the project’s C++ files with `clang-format`, install LLVM and run the
script for your platform:

```powershell
.\format.bat
```

```bash
./format.sh
```

Run the prototype:

```powershell
.\build-ode\OpenBus.exe
```

The executable opens a simple OpenGL preview window while the physics runs at
the configured fixed rate.

### OMSI configuration audit

On Windows, run the helper with an OMSI directory (or another directory to
inspect). It builds `OpenBusConfigAudit` in `build-ode` before scanning. The
audit validates map manifests (`global.cfg`), bus configurations and their
referenced models, model configuration files, and OSC scripts:

```powershell
.\audit.bat "C:\Path\To\OMSI 2"
```

Without an argument, the helper uses `OMSI_ROOT` if set, or prompts for a
directory. Set `OPENBUS_BUILD_DIR` to use a different existing CMake build
directory.

The auditor recursively checks `.bus` files, recognized model `.cfg` files, and
`.osc` scripts without writing translated Lua files. Build/output, vendor, and
virtual-environment directories are skipped. Results distinguish `INVALID` from
`UNSUPPORTED`; unknown `.cfg` families are reported as unsupported instead of
being parsed under the wrong grammar. OSC reported as unsupported by OpenBus's
native compiler is not treated as definitely invalid. The process returns 1 if
invalid files were found, 0 otherwise, and 2 for an invalid invocation/root
path.

### Optional binary mesh bundle

To reduce startup parsing and file-open overhead, generate an `.obx` bundle from
converted OBJ meshes. The wrapper configures and builds the bundle tool in
`build-ninja`; its arguments are the model root and optional output path:

```powershell
.\bundle.bat MAN_DL05\Model\DL05_obj
```

`bundle.bat` configures/builds `obj_bundle` in `build-ninja` and passes the
arguments to it. Copy or rename the resulting file to `openbus.obx` beside the
model's converted meshes. OpenBus uses matching bundle entries when available
and falls back to OBJ files for missing entries, so the bundle is optional. The
model CFG still controls mesh order, visibility, animations, and material
overrides.

The Windows build writes a symbolized `game.crash` report with exception
details, registers, module names, and source lines when matching PDB files are
available. It also writes `game.crash.dmp`, which can be opened in Visual Studio
or WinDbg with the matching executable and PDB. The Linux build writes a
`game.crash` stack trace for fatal POSIX signals and unexpected termination.
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
- Per-corner spring-damper suspension, anti-roll forces, tyre friction, braking,
  rolling resistance, aerodynamic drag, steering, bounded chassis attitude
  stabilization, and automatic gearing
- Wheel torque, wheelspin, and skid telemetry derived from the ODE wheel bodies
- Fixed-rate physics (60 Hz by default) with a bounded catch-up budget; if a
  frame exceeds that budget, excess accumulated time is dropped to avoid a
  spiral of death

Interactive vehicle physics is loaded from the `.bus` selected in the startup
menu. Benchmark/custom runs can select a bus with `OPENBUS_BUS_CONFIG`. Relative
paths are resolved against the standard Steam installation at `C:\Program Files
(x86)\Steam\steamapps\common\OMSI 2`; set `OPENBUS_OMSI_ROOT` when OMSI is
installed elsewhere:

```powershell
$env:OPENBUS_OMSI_ROOT = "D:\SteamLibrary\steamapps\common\OMSI 2"
```

Vehicle values are not duplicated in C++. Articulated vehicles are outside the
currently supported scope. The `openbus_articulated` configuration flag is
retained for inspection only; it does not create additional ODE bodies, joints,
collision shapes, or rendered sections, so articulated models must not be
treated as supported vehicles. The parsed configuration is written to
`OpenBus_configuration.json` at startup. Parsed model parts, materials,
visibility variables, animation variables, and wheel bindings are written to
`OpenBus_model_configuration.json` as well. For parts with `[animparent]`,
`parent_animations` shows the variables inherited from the referenced mesh,
which makes door-glass control variables visible.

The model variable store initializes OMSI-style system variables with the local
system date at midday: `Time` is `43200`, `Day`, `Month`, and `Year` come from
the current date, and `DayOfYear` uses OMSI's one-based convention. Defaults include
`NoSound=1`, `Pause=0`, `PrecipType=1` (clear), `PrecipRate=0`,
`Weather_Temperature=20`, `Weather_AbsHum=10`, `Envir_Brightness=1`,
`AutoClutch=0`, and `SunAlt=60`. `Timegap`, `GetTime`, `mouse_x`, and `mouse_y`
are updated from the renderer each frame. Precipitation values are persistent
shared system variables, so configured weather survives frame updates.

The initial weather slice renders rain or snow particles in exterior and free
camera views. Set `OPENBUS_WEATHER=rain` or `snow`; `clear` (the default)
disables precipitation. `OPENBUS_WEATHER_RATE` accepts `0`–`1` and defaults to
`0.5` for rain/snow. `OPENBUS_WEATHER_TEMPERATURE` sets `Weather_Temperature`
in degrees Celsius (default `20`), and `OPENBUS_WEATHER_ABSHUM` sets
`Weather_AbsHum` in g/m3 (default `10`). Temperature must be above absolute
zero and absolute humidity must be non-negative. Set `OPENBUS_AUTO_CLUTCH` to
`1`, `true`, `yes`, or `on` to publish `AutoClutch=1` (default `0`); this
reports the setting to scripts but does not yet implement automatic clutch
control. `OPENBUS_SUN_ALTITUDE` overrides `SunAlt` in degrees (default `60`,
valid range `-90` through `90`); it is not calculated from map location or
simulation time. Press `P` to pause/resume, or `Esc` to open the pause menu and
pause; rendering and free-camera movement continue while physics, traffic,
passenger service, the simulation clock, script frames, and vehicle key/mouse
script events are paused. The pause menu can stop a scheduled trip immediately
or at its terminus, remove or spawn the player bus when no trip is active, return
to the startup screen, or quit the game. Its Debug Menu can set the map date/time
and skip ahead to the scheduled departure when a player trip is early. Active key
and mouse interactions receive release events as pause begins. On foot, the Spawn
Bus panel searches model/manufacturer, bus type, and configuration filename, then
filters bus types after a model is selected. Configuration filenames distinguish
duplicate type labels. Weather settings populate the shared `PrecipType`,
`PrecipRate`, `Weather_Temperature`, and `Weather_AbsHum` system variables
before scripts run. This does not yet load OMSI map weather schedules or render
fog, windshield effects, wet roads, or snow accumulation.

For benchmark/custom runs, `OPENBUS_BUS_CONFIG` accepts an absolute path or a
path relative to `OPENBUS_OMSI_ROOT`. Set the optional `OPENBUS_MODEL_CONFIG` to
select a `.cfg` explicitly; otherwise the loader uses the `.bus` file's
`[model]` entry. These settings do not override interactive vehicle selection.
For example, both paths below are relative to the OMSI installation root:

```powershell
$env:OPENBUS_BUS_CONFIG = "Vehicles/Caetano Levante/Caetano.bus"
$env:OPENBUS_MODEL_CONFIG = "Vehicles/Caetano Levante/Model/model_caetano.cfg"
./build-ode/Release/OpenBus.exe
```

Set `OPENBUS_ODOMETER_MODE` before launching to choose the player odometer:

| Value | Behavior |
| --- | --- |
| `global` (default when unset) | One shared counter across all bus models, starting at zero. |
| `vehicle` | A raw-distance counter per normalized `.bus` configuration path, starting at zero. |
| `vehicle-default` | The same per-model raw distance plus the bus's `[kmcounter_init]` kilometres (zero if absent). |

Fleet numbers and registrations do not affect these counters. The global and
per-model counters are saved separately in `OpenBus_odometer_preferences.txt`;
only the selected counter advances. The two per-model modes share raw mileage,
so changing modes never adds the configured starting mileage twice. Existing
version-2 model distances are retained, with the new global counter starting at
zero. Invalid mode values produce a startup error.

```powershell
$env:OPENBUS_ODOMETER_MODE = "vehicle-default"
```

Every rendered vehicle must provide a `VehiclePlacement` containing its world
`x/y/z` position and yaw in degrees. The renderer can load multiple vehicles in
one process by calling `RenderLoop::AddVehicle` once per `.bus`/`.cfg` pair. The
first vehicle is connected to the shared bus simulation; additional vehicles are
scripted AI models rendered at their declared placements. Set
`OPENBUS_AI_BUS_CONFIG` and `OPENBUS_AI_MODEL_CONFIG` together to add AI
vehicles. `OPENBUS_AI_COUNT` selects the number to spawn (defaults to 1 and is
limited to 200); multiple instances are arranged in a grid. `run.bat` opens the
interactive vehicle selector and defaults the map to Cotterell. The launcher
supplies the MAN DL05 config defaults only when benchmark mode is enabled.

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
- Hold right mouse and drag vertically: adjust FOV in configured views, or move
  closer/farther in the outside orbit view
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
  tessellated as local spline segments from their position, heading, radius, and
  length fields. Road paths are colored by priority (0 purple, 64 blue, 128
  green, 192 yellow, and 255 red), with a matching F2 legend; human paths are
  magenta. Arrows on paths show their travel direction; bidirectional paths have
  arrows in both directions, and their spline labels show `[BIDIR]`. `H` labels
  identify pedestrian splines; `S` labels identify road splines. Green wireframe
  boxes mark the waiting-passenger slots at registered timetable stops within 75
  m. Their labels show the stop name, ID, and waiting count; an empty queue
  shows its first potential slot. Positions use nearby authored passenger areas
  when available; `[fallback]` identifies generated roadside positions when no
  area is matched. Amber wireframes show editor-only `[busstop]` objects, and
  cyan wireframes show editor-only passenger areas and their authored cabin
  positions. Labels show the map object ID and any stop name. Each passenger
  area is associated with the nearest editor bus stop within 30 m and 2 m
  vertically; this proximity rule is an approximation, not verified native OMSI
  linking. Waiting passengers use its authored positions first; excess
  passengers are distributed inside the authored passenger-area bounds. When no
  area is matched, the generated roadside queue is used. White arrows above
  active AI vehicles show the vehicle's actual facing direction for comparison.
  Orange squares mark unconnected directed starts; red Xs mark unconnected
  directed ends. Connected endpoints have no marker: they have reciprocal links
  or a matching map or scenery path endpoint within 0.5 m horizontally and
  vertically. Matching endpoints are joined in the overlay with a short colored
  connector when a visible gap remains; this does not change AI route
  connectivity. Traffic-light phase, cycle, and control-point
  diagnostics appear in an ImGui panel while F2 is active. The top-left ImGui
  status panel shows the simulation date and time to the second, player
  coordinates, the current map tile and whether it is active, along with
  map/traffic statistics. Signalized road paths show a stopline across the
  authored path width, placed 0.75 m before the controlled path entry (the AI
  stopping boundary). The ImGui traffic-light panel lists junction/object ID
  (`J`), path (`P`), light (`L`), current phase (`PH`), controller clock (`T`),
  cycle and each phase/duration (`C`/`S`), and conditional stop/jump controls
  (`CP`, with `J->` showing the jump target). Signal stoplines are red for stop
  phases, amber for changeover phases, and green for permissive phases.
  Editor-only junction controller records remain included in this diagnostic
  overlay. The status overlay also reports active traffic-light models (`TL`),
  the controller-driven subset (`C`), and counts by current `trafficlightphase`
  value (`PH`); `?:N` marks models whose phase is unavailable.
- Map-driven road vehicles use deterministic starts on authored road routes
  containing at least two linked splines. AI buses are further restricted to
  linked spline chains present in timetable trips with station data. Vehicles
  follow authored link order; closed routes cycle as authored and open routes
  retire at their endpoint rather than reversing back along the path.
- Press `U` to raise the player bus and its physics bodies 5 m at the current
  X/Y location; it then falls naturally under gravity.
- VSync is requested for the startup menu, regardless of `OPENBUS_VSYNC`.
  Gameplay also requests VSync by default; set `OPENBUS_VSYNC=0`, `off`, or
  `false` before launching to disable it for gameplay. The graphics driver or
  display compositor may override the requested swap interval. Set
  `OPENBUS_FPS_CAP=60` to limit rendered gameplay frames to 60 FPS independently
  of VSync; leave it unset for no software frame-rate cap.
- AI traffic, pedestrians, and traffic signals run on a dedicated worker loop.
  Set `OPENBUS_AI_HZ=60` to choose its update rate. If unset, AI follows the
  average rendered FPS over a rolling three-second window, divided by
  `OPENBUS_AI_HZ_DIVISOR` (default `1`); it starts at 60 Hz until an average is
  available. `OPENBUS_MAX_FPS` optionally caps the AI update rate only; it does
  not limit rendering. AI rate is also automatically reduced when the smoothed
  worker tick time exceeds 80% of its configured tick budget. The HUD reports
  the effective AI Hz and average worker tick time in milliseconds.
  `OPENBUS_FPS_CAP` is the separate software render cap and works independently
  of VSync.
- Reflection mirrors render at `256x256` by default. Set
  `OPENBUS_REFLECTION_SIZE=256`, `512`, or `1024` to choose the square mirror
  target resolution.
- Mirrors update every frame by default. Set `OPENBUS_REFLECTION_INTERVAL=4` to
  update them once every four rendered frames; values below `1` are treated as
  `1`.
- Set `OPENBUS_REFLECTION_MAX_FPS=60` to cap mirror refreshes at 60 FPS without
  limiting the main game. The cap is unset by default; when both reflection
  controls are configured, the interval and time-based cap both apply.
- Mirror transparency is rendered by default, including in the run/profile
  launchers, which preserve an explicitly supplied value. Set
  `OPENBUS_REFLECTION_TRANSPARENT=0` to render opaque geometry only and reduce
  mirror workload. This also removes alpha-tested and alpha-blended vehicle
  bodywork, not just glass: some AI vehicles will show only their wheels. Leave
  it enabled for complete vehicles in mirrors. The render benchmark explicitly
  disables it for its opaque-only workload. This setting is read once on the
  first mirror draw and retained for the process lifetime.
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
  vehicle and scenery scripts at 30 Hz while rendering continues at the display
  rate; fixed-rate script ticks receive `Timegap=1/30`. Each update runs at most
  eight catch-up ticks and clamps elapsed time to 250 ms. If more whole ticks
  remain, they are dropped to keep script work bounded; the fractional remainder
  is retained and dropped time is logged periodically. Invalid or non-finite
  rates are logged and scripts follow the render rate.
- Stationary-pointer hover checks refresh every 75 ms (about 13.33 Hz) by
  default. Set `OPENBUS_HOVER_HZ=30` to refresh at 30 Hz, or
  `OPENBUS_HOVER_HZ=0` to check every rendered input frame. Rates are clamped to
  0–1000 Hz; invalid or non-finite values are logged and retain the default.
  Cursor movement, camera changes and clickable-state changes still invalidate
  the cache immediately; clicks are not throttled by this setting.
- Profiling retains at most `1,000,000` events by default. Set
  `OPENBUS_TRACE_MAX_EVENTS=250000` or another positive value to lower the
  memory ceiling. Trace events are flushed incrementally to the JSON file while
  profiling runs, rather than being held until shutdown.
- Trace scopes shorter than `1` microsecond are omitted by default. Set
  `OPENBUS_TRACE_MIN_US=5` to omit scopes shorter than five microseconds; values
  below `1` are treated as `1`.
- Set `OPENBUS_TRACE_TRAFFIC_DETAIL=1` while tracing to split individual traffic
  lookahead samples into curve/speed-limit and clearance scopes. This can
  produce many events and add profiling overhead; leave it unset for normal
  profiling.
- Optional texture downscaling: set `OPENBUS_TEXTURE_SCALE=0.5` before launching
  to upload half-resolution textures; valid range is `0.25`-`1.0`. OpenBus does
  not include a texture-conversion helper; create BC3 DDS files with your
  preferred compatible tool if needed.
- Mesh rendering uses frustum selection and sorted opaque/transparent batches to
  reduce off-screen work and state churn. Set `OPENBUS_FRUSTUM_CULLING=0` to
  disable culling for debugging. Opaque parts are front-to-back and transparent
  parts back-to-front; this is not hardware occlusion-query culling.
- Set `OPENBUS_WHEELS_FROM_ODE=1` to bypass wheel `newanim` transforms and place
  each wheel from its matching ODE body pose. The default is `0`.
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
  face/normal agreement; only those meshes have winding correction and clockwise
  back-face culling enabled.
- `[newanim]` records and their origin commands are processed in configuration
  order. Origin-only records are valid. `anim_rot`/`anim_trans` use the
  converted local animation frame, `delay` is rate-based smoothing, and
  `maxspeed` limits movement per second. `[animparent]` composes parent
  transforms for body and wheel/detail meshes.
- When `MAN_DL05/Model/DL05.cfg` is available, the highest-detail `[LOD] 0.25`
  model block is rendered with driver-viewpoint mesh filtering
- OBJ texture coordinates follow the converter's existing V-axis conversion plus
  the final OpenGL image-origin correction;
- MAN DL05 OBJ lateral coordinates are mirrored into the simulator convention so
  the modeled right-side exit remains on the vehicle's right
- TGA, BMP, PNG/JPEG, TIFF (when libtiff is available), and DXT1/DXT5 DDS
  texture data are decoded for model materials, including BMP and PNG payloads
  with a `.dds` filename
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
- Texture lookup diagnostics are written to `OpenBus_textures.log`; entries are
  deduplicated and record loaded files, missing source files, and decode
  failures
- The visible chassis and wheels use the live ODE body position and rotation,
  including chassis pitch/roll, suspension travel, steering, and wheel rotation
- A red three-axis marker shows the aggregate mass-weighted center of gravity at
  the configured 1.0 m bus CG height
- The configured 11,500 kg bus mass includes the wheel and suspension bodies;
  the chassis body mass is reduced accordingly so the complete ODE assembly
  totals 11,500 kg
- Close the window to exit

The ground uses a world-space grid with highlighted ten-metre lines.

## CPU and memory profiling

Use `profile.bat` (Windows) or `profile.sh` (Linux) to launch an instrumented
build. The Linux script does not capture memory by default. Pass `--memory` to
run it under Heaptrack; install the `heaptrack` package if it is unavailable.
The Linux build uses `RelWithDebInfo` so allocation call stacks retain symbols.
Heaptrack's allocation capture runs for the entire application session. Its
output defaults to `openbus-memory.heaptrack` in the repository root;
`OPENBUS_MEMORY_TRACE_FILE` selects another path. Open the capture with
`heaptrack_gui` or inspect it with `heaptrack_print`.

For a lower-overhead memory capture, run `./profile.sh --memory-only`: this
disables automatic CPU trace recording, though F10 can still enable it. Run a
short scenario and close the application normally so Heaptrack can finish.
`./profile.sh --memory-only --raw` records raw Heaptrack data without running
the interpreter during the game; follow the conversion command printed by
Heaptrack before opening that capture. Deferring interpretation does not reduce
its memory requirements, and raw data can require substantial disk space.
Choose a different `OPENBUS_MEMORY_TRACE_FILE` to retain previous captures.

Map textures release their prepared CPU image after a successful GPU upload.
Concurrent scenery preparations share a decoded image while it is still needed;
the sharing index does not keep the image alive after its last owner releases it.
If the same texture is later needed with a different addressing mode, it may
be decoded from disk again. The initial map-load summary logs the number and
pixel-buffer capacity of prepared CPU images still held by the map loader;
these figures exclude GPU textures and images held by other owners.
The same summary includes scenery image decode attempts and shared-image uses,
which can help assess how often preparations avoided a duplicate decode.
Scenery key collection passes, path resolutions, and request batches are also
reported; collected tile keys are reused across preparation batches rather than
resolving every placement again for each batch.
The summary also reports scenery texture lookups versus resolutions and terrain
texture resolutions versus path reuses. Terrain paths are reused only within a
tile preparation; missing paths are resolved again rather than cached.

The Chrome CPU trace is separate. The Linux profiling script starts it
automatically unless `--memory-only` is set; other instrumented runs start
paused unless `OPENBUS_TRACE=1` is set. Press **F10** to pause or resume
recording. Holding the key does not repeatedly toggle it. Recording state
changes are logged.

The Chrome trace is written to `openbus_trace.json`; `OPENBUS_TRACE_FILE`
overrides its path. Pausing drains pending events and leaves a valid JSON file
that can be opened in Perfetto or a Chrome trace viewer without exiting.
Resuming adds to that file; paused time and scopes spanning a pause are not
recorded. Timestamps retain the real gaps between recording intervals. The event
limit applies across all intervals in the run. The first recording in a new run
overwrites the output file; starting paused does not touch it.

For automatic startup/loading captures, launch an instrumented executable
directly with `OPENBUS_TRACE=1`. This opt-in preserves automated benchmark
capture behaviour. Normal builds with `OPENBUS_ENABLE_PERF_TRACE=OFF` do not
record; F10 logs that a profiling build is needed.

`RenderLoop::draw` includes separate scopes for ground-height sampling,
collision debug rendering, the status HUD (text update, rasterization, upload,
and submission), and window-title updates. The HUD is the final overlay and
restores the scene's depth-enabled, blend-disabled, culling-disabled state
without querying the driver. Trace durations are CPU wall times, not GPU
execution times: driver backpressure can appear in a GL call or in
`RenderLoop::endFrame.swapBuffers`. Compare whole `main` frame times rather than
treating a shorter draw scope as an equal FPS gain.

While tracing, the periodic `Physics collision profile` log breaks down
`dCollide` calls and hit pairs by wheel/static, chassis/static, wheel/pole,
chassis/pole, and other geometry pairs. These are counts over the logged
interval, not elapsed times; compare them with the timing trace when
investigating collision costs.

Traffic captures break `RenderLoop::updateTrafficDriving.vehicleMovement` into
actor collection and runtime advance.
`MapTrafficRuntime::advanceMapTrafficDriving` then reports route classification,
substep occupancy/claims, priority-pair arbitration, and actor movement stages;
actor movement includes following and obstacle projection, route lookahead,
junction admission, swept movement, and reservations. Use these nested spans in
Perfetto to find which part dominates vehicle movement time.

AI traffic uses moving occupancy rather than fixed-distance blocks: following
targets account for leader speed, braking distance, and time headway, while
rolling junction claims and predicted crossing reservations protect merges and
intersections. Lookahead collision checks use spatial indexes for occupied
footprints and predicted movement reservations, then perform the full footprint
test on nearby candidates. Nearby candidates are checked in their original index
order, so the first conflict, yielding, and stall diagnostics match a full scan.

For a repeatable free-flow lookahead benchmark, run
`build-release-ninja\OpenBusMapTrafficRuntimeProbe.exe --benchmark-lookahead`
after building the `OpenBusMapTrafficRuntimeProbe` target in Release mode. It
reports average update time for 69 actors over 30 updates plus final distance
and speed totals for before/after comparisons. This synthetic case stresses
distant reservations; it is not a substitute for profiling a populated map.

`--benchmark-junctions` runs 72 actors through a 3×3 grid of junctions for 600
updates, so priority arbitration, junction admission and crossing reservations
are exercised. Besides the average update time it prints a `state_hash` of every
actor's distance, speed, junction wait, block reason and junction claim after
every update. An optimization that must not change behaviour should leave that
hash unchanged. `--benchmark-junction-substeps` uses the same grid with 80
updates of 0.2 seconds each, exercising four 0.05-second simulation substeps per
update. The `OpenBusMapTrafficJunctionSubstepProbe` regression compares combined
substeps with separate updates for actor movement, blocking and claims.

The focused T-junction regression simulates major-road through traffic and a
vehicle turning onto the minor road for 30 simulated seconds in 300 updates. It
checks vehicle clearance throughout and requires every vehicle to clear the
junction without stalling. Run it with
`build-release-ninja\OpenBusMapTrafficRuntimeProbe.exe --t-junction`, or select
the `OpenBusMapTrafficTJunctionProbe` CTest test.

Some traffic optimizations skip or defer route-sampling queries, for example
curve limits that cannot lower the target speed, or lazily computed arrival
windows. They are enabled only when every actor's route provably cannot fail to
sample. Otherwise `advanceMapTrafficDriving` evaluates those queries eagerly in
their original order, so a malformed route still raises the same exception at
the same update. The probe's malformed-route checks guard this behaviour.

## Loading benchmark

`benchmark_loading.ps1` runs the existing `build-ninja\OpenBus.exe` by default,
with `OPENBUS_BUS_CONFIG` set to MAN DL05 and benchmark mode enabled. Close the
application after each run finishes loading. It records the existing object-load
completion marker only; texture completion has no runtime marker. Per-run logs,
optional traces, and CSV results are kept under `render-benchmark-results`. Use
`-BuildDirectory`, `-BusConfig`, `-Map`, and `-Runs` to select the build,
vehicle, map, and sample count. The benchmark disables map and external AI
populations so the object-load measurement belongs to the selected bus. For
`-EnableTrace`, configure and build the `windows-clang-benchmark` preset; the
script rejects builds without tracing instrumentation and verifies that a trace
file was written.

## Repeatable render benchmark

Run `benchmark_rendering.bat` to build an isolated Release executable with
tracing enabled, then benchmark 720p, 1080p, 1440p, 4K, and 5120x1440. The
runner is `benchmark_rendering.py`, requires Python 3.9 or newer, and uses only
the standard library; set `OPENBUS_PYTHON` if `python` is not on PATH. The
runner performs five runs per resolution by default, in deterministic seeded
order; use `--runs 1` for a quick check or `--seed N` to change the order. After
building, the Python runner can also be launched directly with `python
benchmark_rendering.py`. Each run measures baseline rendering, user-facing
camera cycling, third-person zoom, driving controls, and dashboard interaction.
Benchmark mode currently creates a visible GLFW window, uses a fixed 60 Hz
simulation step, and disables VSync. Actual dimensions are checked and
mismatched runs are skipped before warm-up and measurement. Normal simulator
launches remain visible and retain the standard `build-ode` build. `profile.bat`
uses the separate `build-benchmark-ninja` directory for its tracing-enabled
benchmark build.

`--warmup-frames` is a total distributed across the five phases (60 by default,
12 per phase). Each phase is then measured twice per resolution run, with 120
fixed-step frames per visit by default (two seconds at 60 Hz), slowing camera,
zoom, steering, and dashboard-look progressions to half their previous rate. Use
`--phase-repeats` and `--phase-frames` to adjust those values. Results are
written to `render-benchmark-results/`. The `*_runs.csv` and `*_scopes.csv`
files record requested window, actual window, framebuffer sizes,
framebuffer/window scale factors, and flag any framebuffer that differs from the
requested resolution. `render_benchmark_frametimes.csv` reports one
measured-frame summary row per run, including average, p90, and p95 frame times
in milliseconds across all measured phases and repeats; the same metrics are
also included in `render_benchmark_runs.csv`. Percentiles use linear
interpolation over the sorted frame-time samples. Skipped runs remain in the run
and frame-time summary files with a status and reason but no timing metrics. Use
`--require-exact-resolution` to return a failure status if any run is clamped.
The `*_frames.csv` file records one row per main-frame or selected
renderer-scope event, including timestamp, inclusive duration, and self time.
Nested calls stay separate so reflection draws are not accidentally summed into
a misleading per-frame total. High-volume `Vehicle::drawBatch.prepareAndSubmit`
events are included only when they last at least 0.1 ms, keeping normal batch
submissions from overwhelming the report.

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
The original CFG files remain in place so their relative paths continue to work.

The OMSI 2 map-package reference is in
[Docs/OMSI2_MAP_FORMAT_REFERENCE.md](C:/Users/blane/Desktop/OpenBusTest/Docs/OMSI2_MAP_FORMAT_REFERENCE.md).
It documents `global.cfg`, tile records and sidecars, Chrono variants, AI lists,
timetable files, parser limitations, and validation against the installed map.
