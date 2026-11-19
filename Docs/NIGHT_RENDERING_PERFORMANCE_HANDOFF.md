# Night Rendering Performance Handoff

## Status

Night lighting is still GPU-expensive, and the requested three-category lighting
architecture is **not complete**. The renderer now retains static scenery-light
attributes on the GPU and supports more than 128 lights, but its forward
fragment path still loops over every selected scenery light per fragment.
An opt-in vertex-evaluated scenery-light path is available as an approximation.
Distance-based AI spotlight scheduling is implemented and configurable, but
unified Forward+ or clustered light assignment is not implemented.

Earlier user-provided GPU timer samples showed approximately 17.8-18.2 ms for
the main view and 8.4-8.7 ms per reflection sample in darkness, versus
2.76-2.80 ms and 0.44-0.45 ms at full brightness. These are historical
observations, not a controlled comparison of the current implementation.
Reflection samples are per target refresh, not per frame.

## Implemented lighting changes

- Player spotlights are prepared for every rendered view and ordered ahead of
  AI spotlights when an explicit vehicle-light limit applies. A spotlight
  whose influence sphere (four times its configured range) is outside the
  view frustum is culled; this cutoff matches the shader's range cutoff.
- Player spotlights remain fully dynamic. Route-AI spotlight preparation uses
  configurable distance tiers (defaults: 2 frames within 60 m, 6 frames within
  180 m, and 20 frames farther away); approaching or fast vehicles halve the
  interval, while light-selection changes and maneuver/state changes force an
  immediate refresh. Cached world-space light data is adjusted for vehicle
  movement between updates, while view transforms and culling remain per-view.
  A conservative influence-sphere test suspends refreshes in views that cannot
  be affected, including reflection views; a light returning after a missed
  frame refreshes immediately. `AI_SPOTLIGHT_SCHEDULE` logs refresh counts and
  observed intervals. Physics-simulated AI is refreshed every relevant frame.
- Scenery lights are registered map-wide at load time and conservatively
  indexed into tiles overlapped by their radius, with a global fallback for very
  large bounds. The default renderer evaluates all registered lights, regardless
  of that index, to preserve their inverse-square tails beyond the configured
  radius. Contributions to moving objects remain dynamic. This is not GPU
  tile/cluster assignment, and evaluating all map-wide sources can be expensive.
- Both scenery and vehicle spotlights use separate OpenGL 3.3 texture buffers.
  There is **no configured light-count cap by default**; capacity is limited by
  the GPU and invalid over-capacity collections produce an error. Explicit
  `OPENBUS_MAP_LIGHT_LIMIT` and `OPENBUS_VEHICLE_SPOTLIGHT_LIMIT` settings
  remain available, but lower limits omit sources and are quality trade-offs.
  The earlier 64-light defaults and 128-light shader-array maximum are obsolete.
- When the registered sources fit the GPU buffer and are valid, the default
  scenery path retains their color and radius in a persistent
  buffer. Active-tile selection changes only the view list; the source buffer
  is rebuilt when the source count or content revision changes. If the
  full set exceeds capacity or includes an invalid inactive source, the
  renderer uses an active-source buffer or a compact selected-source buffer.
  Each view still
  computes positions on the CPU using double-precision transforms and uploads
  a 16-byte position/index entry per selected light when the list changes.
  Static attributes use 16 bytes per source rather than the previous 32 bytes;
  failed static uploads and texture-buffer bindings report errors and do not
  mark their revisions as uploaded. Failed replacement uploads invalidate
  their prior cache entries, as do failures uploading direct map lights,
  vehicle spotlights, or spotlight tile masks, so a later retry cannot reuse
  potentially changed GPU data as if it were the old upload. Radii and ranges
  below the smallest positive normal GPU float are rejected explicitly rather
  than being converted to a value that may flush to zero in shader arithmetic.
  The direct path uploads 32 bytes per selected light per changed view.
  Moving vehicles and reflections evaluate the selected scenery lights
  dynamically; the lighting is not baked.
- Scenery view preparation avoids camera-distance calculation and sorting when
  the selected light limit includes every active source. If a limit omits
  sources, it partially sorts by camera distance and transforms only the
  nearest selected lights; a zero limit skips view-position preparation. The
  default remains unlimited, so no lights are removed.
- Repeated scenery draws with the same camera and active sources reuse the
  prepared view list. `LIGHTING_UPLOAD` reports `map_view_preparations` and
  `map_view_reuses`; the cache invalidates when the view, source selection,
  source count, light limit, or source revision changes.
- Set `OPENBUS_STATIC_MAP_LIGHT_BUFFER=0` to compare the direct view-space
  path; `1` is the default. Resident storage covers registered map-wide
  sources and can consume more GPU memory than the former active-tile-only
  buffer.
- Set `OPENBUS_MAP_LIGHT_VERTEX_EVALUATION=1` to evaluate all selected scenery
  lights per mesh vertex and interpolate their contributions. This keeps all
  lights and leaves vehicle spotlights per-fragment, but changes the shading
  approximation on large triangles. It is off by default pending a fixed-scene
  visual comparison.

The relevant implementation is in [CoreRenderer.cpp](../src/render/CoreRenderer.cpp),
[MapRendererDraw.cpp](../src/render/map/MapRendererDraw.cpp),
[StaticLightTileIndex.h](../src/render/map/StaticLightTileIndex.h), and
[RenderLoopDraw.cpp](../src/app/RenderLoopDraw.cpp).

## Recent measurements

After map-wide registration, a 1280x720 Cotterell 23:00 run with the player
400 MMC and 100 forced 400 MMC AI headlights averaged 98.86 ms/frame over 150
frames (p95 133.95 ms). It included up to 345 scenery lights and 101 vehicle
spotlights. A prior 85-scenery-light run averaged 76.26 ms/frame, but these
short runs share a GPU with another OpenBus process and do not isolate the
cost of map-wide registration. They do show that the under-5-ms target has
not been met. Map-wide exact tails increase the work performed; an exact
spatial cull is not possible for nonzero infinite tails. The opt-in vertex
approximation averaged 508.89 ms/frame in a separate short run of the same
workload; it is not a demonstrated improvement in this scene.
A subsequent 2560x1440 run of the same night workload averaged 112.73 ms/frame
over 75 frames (p95 141.09 ms). Another run with an intermediate terrain-cache
change that was subsequently reverted averaged 158.76 ms/frame. These are
shared-GPU measurements, not an isolated comparison.
A one-division falloff rewrite averaged 140.44 ms/frame in another short run;
the result did not demonstrate a benefit, so the shader rewrite was reverted.
After adding retry on source-tile publication, a 1280x720 night run with 100
forced 400 MMC AI headlights averaged 87.14 ms/frame over 75 frames (p95
119.56 ms). This remains a short observation, not a controlled comparison.
A subsequent 2560x1440 run including spline recovery averaged 132.83 ms/frame
over 75 frames (p95 171.50 ms); it predates the duplicate-ID recovery refinement.
After adding late asset registration, a further 2560x1440 run averaged
131.83 ms/frame over 75 frames (p95 168.78 ms). These short runs do not
establish a performance improvement or satisfy the under-5-ms target.

Three subsequent diagnostic runs used the same 1440p Cotterell night setup,
100 forced AI vehicles, 20 warmup frames and 75 measured frames. Temporarily
setting the light limits to zero yielded the following observations:

| Diagnostic setting | Average frame | p95 frame | Maximum selected lights |
| --- | ---: | ---: | --- |
| Scenery disabled | 74.83 ms | 109.66 ms | 0 scenery, 101 vehicle |
| Vehicle spotlights disabled | 102.21 ms | 132.85 ms | 345 scenery, 0 vehicle |
| Both disabled | 71.13 ms | 105.49 ms | 0 scenery, 0 vehicle |

These are diagnostic configurations, not proposed rendering defaults: disabling
either category changes the image. They were short, separate runs rather than a
controlled A/B experiment, so their differences are not precise category costs.
They do indicate a substantial baseline cost outside these two lighting loops
on the test machine. For example, sampled main-view GPU interval averages in
the both-disabled run ranged from 39.87 to 59.01 ms; in the scenery-enabled,
vehicle-disabled run they ranged from 69.34 to 92.45 ms. The latter run
reported 345 selected scenery lights and 353,280 bytes of view-list uploads
over one 2.08-second telemetry interval; the persistent source buffer did not
need another upload during that interval. Further profiling should separate
baseline rendering costs from exact scenery-light evaluation rather than
attributing the entire frame time to lighting.

The renderer now logs its OpenGL vendor, renderer, version and texture-buffer
capacity at initialization, so future benchmark logs identify the actual GPU
and driver rather than relying on the machine's installed-adapter list. A
subsequent run identified an NVIDIA GeForce RTX 3080 Ti with driver 621.14 and
OpenGL 4.3; its reported texture-buffer capacity was 268,435,456 texels.
The earlier diagnostic logs did not contain renderer identification, so this
does not independently establish which GPU those runs used.
The identified run kept both categories enabled and averaged 120.34 ms/frame
(p95 161.54 ms) over 75 measured frames, with up to 345 scenery lights and
101 vehicle spotlights. This remains well above the requested target and is
not evidence of an optimization from logging the device.

After adding upload-failure handling and finer tracing, another run averaged
121.60 ms/frame (p95 163.60 ms). The 20-us trace threshold captured 247
pre-upload OpenGL error checks in dynamic-light uploads, totaling 3,116 ms;
captured static view-upload scopes totaled 3.07 ms. These are CPU wall times,
not isolated GPU execution times. In an earlier trace without the dynamic
pre-check, similar waiting appeared in the static-light setup scope. The
apparent shift is consistent with graphics-driver backpressure moving to the
first synchronization point; it does not demonstrate that the checks or
uploads alone caused the GPU workload. Do not remove error reporting based on
these timings without an alternative that preserves failure detection.
After GPU-range validation was added, the same short night workload completed
at 124.20 ms/frame on average (p95 162.29 ms); this is a correctness check,
not evidence of a performance improvement.

A subsequent frame trace showed `uploadDynamicLightBuffer.checkPreviousError`
alone taking 45.94 ms over four calls (11.48 ms maximum per call). All
`glGetError()` polling was removed from the renderer, including the static
primitive upload path. When OpenGL 4.3 or `GL_KHR_debug` is available, a
synchronous debug callback reports API errors at the issuing call; upload
caches remain retryable when an error is reported. Contexts without debug
output log that upload errors cannot be detected. The material-rendering probe
injects upload and texture-binding failures to verify they are rejected and
retried without polling the GL error queue.

The clean post-change night benchmark used one 100-AI Cotterell configuration
at both resolutions, with map-generated traffic disabled and a separate trace
file for each:

| Resolution | Mean frame | p90 frame | p95 frame | Main-view GPU mean | Reflection GPU mean |
| --- | ---: | ---: | ---: | ---: | ---: |
| 1920x1080 | 75.02 ms | 107.17 ms | 109.51 ms | 68.86 ms | 2.58 ms |
| 2560x1440 | 84.46 ms | 110.79 ms | 113.19 ms | 77.87 ms | 2.65 ms |

Each resolution matched its requested framebuffer and recorded 600 frame
timings; asynchronous GPU means use 126/270 samples at 1080p and 127/270 at
1440p. Each run confirmed 100 configured AI vehicles plus the DL05 player,
zero map-AI spawns, and asynchronous OpenGL error reporting. The trace summary
did not contain per-upload error-poll scopes at the 20-us trace threshold.
This confirms the previous error-query stalls are gone from the hot path, but
the GPU main view remains much slower than the 5-ms target.

Raw results and distinct traces are in the session results folder
`cotterell-night-100-ai-1080p-1440p-final`: `trace_1920x1080_run01.json` and
`trace_2560x1440_run01.json`, alongside the matching logs, frame timings, GPU
samples and scope summaries.

## Cotterell day/night comparison

Two exact-1440p runs were made with the benchmark harness using the DL05 player
vehicle and 1024-pixel reflections refreshed every frame. Cotterell's
map-generated traffic, map AI buses and pedestrians were disabled in both
cases (`OPENBUS_MAP_AI_COUNT=0`, `OPENBUS_MAP_BUS_COUNT=0`,
`OPENBUS_MAP_HUMAN_COUNT=0`) so the zero-AI baseline did not silently include
map traffic. Spotlight definition 0 was forced on loaded vehicles in both
runs. The day run used map hour 12 with no configured AI; the night run used
hour 23 with 100 configured 400 MMC AI vehicles, for 101 total player-plus-AI
vehicles.

| Scenario | Player | Configured AI | Mean frame | p90 frame | p95 frame | Main-view GPU mean | Reflection GPU mean |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Day, map hour 12 | MAN DL05 | 0 | 5.30 ms | 7.03 ms | 8.05 ms | 2.42 ms | 0.69 ms |
| Night, map hour 23 | MAN DL05 | 100 400 MMC | 98.41 ms | 131.03 ms | 133.93 ms | 90.55 ms | 3.53 ms |

Both runs reported the same NVIDIA GeForce RTX 3080 Ti / driver 621.14 and
2560x1440 framebuffer. The benchmark collected 600 frame timings per run;
GPU means above are weighted from the asynchronous samples (123/266 main-view/
reflection samples in the day run and 130/270 in the night run). The night
run selected up to 345 scenery lights and 101 vehicle spotlights. The day run
selected no scenery lights because ambient brightness suppresses the dynamic
scenery-light contribution. These runs change both time of day and AI count,
so they describe the two requested scenarios but are not a controlled
measurement of the isolated cost of either variable. The night result is far
above the 5-ms target.

Raw benchmark artifacts are in the session results folder
`cotterell-day-night-ai-comparison`, in `day-zero-ai-map-traffic-off` and
`night-100-ai-map-traffic-off` subfolders. Each contains the run summary,
per-frame timings, GPU timing samples, scope trace and application log.

Independent review identified unresolved map-wide registration limitations:
an over-capacity selected set throws rather than rendering all lights, and a
failed terrain read or missing parent metadata can leave a light at a fallback
pose. Unresolved ordinary, attached and spline-attached sources are retried
before their source tile is published, and recovery updates the light and its
GPU/view cache revision. Ordinary, attached and spline sources whose metadata
was unavailable during indexing are registered when their tile is published if
the asset has since recovered. Sources whose dependencies remain unavailable
at publication still need a later recovery mechanism.
Successfully resolved light placements are now shared with streamed
geometry, including cross-tile attachments with unloaded parents; a regression
probe checks nonzero parent terrain height, and independently resolved spline
repeater poses are retained separately. The persistent buffer capacity now
uses the full reported texel count instead of the direct path's two-texel
limit, and oversized source sets use a compact selected-source buffer when the
selection fits, but this does not remove the hardware limit. A proposed persistent
terrain cache was reverted because it could fail the whole map on a remote
terrain error and introduce synchronous streaming reads. A retryable,
asynchronous placement pipeline and a capacity-safe exact rendering path
remain necessary.

Review-fix spot checks on Cotterell at 23:00 used a 400 MMC player and 100
explicit 400 MMC AI vehicles with forced headlights, 1280x720, 30 warmup and
150 measured frames. One baseline run averaged 74.13 ms/frame and one updated
run averaged 76.51 ms/frame. Both reported up to 101 vehicle spotlights and
85 scenery lights. This is not a controlled performance conclusion: another
OpenBus process was running, the runs were short, and the two executables used
different tracing-enabled build directories. In this dense scene every light
remained relevant, so the new AI view suspension was not exercised.

The updated `setStaticMapLights` scope averaged about 0.50 ms across 521
recorded calls at a 20-microsecond threshold, while the baseline had no calls
above that threshold. The newly added OpenGL error checks may expose driver
backpressure there; these wall times do not establish how much frame time the
checks add, and require a controlled profile. The updated player-only night
run confirmed that uploading 85 static sources used 1,360 bytes rather than
2,720 bytes. Changed view lists still upload 16 bytes per selected light.
In a subsequent comparable dense-scene run with view caching, the average was
76.26 ms/frame and periodic logs reported 22-35 view reuses alongside 88-121
view preparations. The scenery preparation scope averaged about 0.90 ms,
predominantly inside `setStaticMapLights`; the short, shared-GPU samples do not
establish a performance change. The existing GPU view-buffer comparison had
already avoided duplicate uploads, so this cache saves CPU preparation, not
those uploads.

Four short, interleaved runs used the same binary: Ingolstadt at 01:30,
1280x720, 20 cars and 5 buses configured, reflection size 1024 and interval 1,
no configured light limits, 30 warmup and 150 measured frames per run. GPU
timing was sampled every five main frames. Each run completed at the requested
resolution.

| Scenery path | Mean frame time, run A | Mean frame time, run B | Sampled main-view GPU interval averages |
|---|---:|---:|---:|
| Persistent | 25.893 ms | 25.557 ms | about 17.1-18.6 ms |
| Direct | 25.838 ms | 24.536 ms | about 16.4-17.7 ms |

These runs suggest a possible persistent-path GPU cost, but are too short and
variable to establish a general regression or performance win. GPU summaries
can include warmup or transitions. No vehicle spotlights were active, so the
runs do **not** benchmark player or AI headlights. The maximum selected scenery
count was 142, not 250.

In the sampled persistent runs, an initial upload of 142 static sources used
4,544 bytes and was not repeated in the later sampled interval. Changed view
lists used 16 bytes per selected light, versus 32 in the direct path. These
are submitted bytes, not measured GPU bandwidth. `LIGHTING_UPLOAD` totals
include reflections and other view/setter calls, so compare like-for-like
intervals carefully. Raw run CSVs, logs and traces are in the local session
directory
`C:\Users\blane\.copilot\session-state\74f7a7c0-7e21-4b75-8ddb-ed3da181d269\files`,
under `static-a`, `static-b`, `direct-a` and `direct-b`.

An exact-resolution Cotterell comparison used the MAN DL05, 2560x1440,
01:30, 100 active map AI vehicles, 600 warmup and 600 measured frames. It was
one run per mode with asynchronous GPU timing sampled every five frames.
Vehicle spotlights were not active (zero uploaded), and the lights-disabled
diagnostic set both map and vehicle limits to zero. Sampled main-view GPU
sample-weighted main-view GPU averages were 14.14 ms with fragment-evaluated scenery lights, 2.95 ms with
`OPENBUS_MAP_LIGHT_VERTEX_EVALUATION=1`, and 1.56 ms with the diagnostic
lights disabled. Subtracting the diagnostic baseline gives an approximate
scenery-light contribution of 12.61 ms versus 1.42 ms. A second vertex-mode
run measured 3.03 ms weighted main-view GPU time and 8.02 ms average CPU
frame time. This puts the measured
map-light contribution below 5 ms without lowering light counts, but it is a
single-run A/B proxy, not an isolated GPU pass. Full CPU frame averages were
19.02, 8.02 and 6.91 ms respectively for fragment, vertex and no-light runs.
The vertex path remains opt-in until matched captures establish acceptable
image error. The second vertex run is in
`render-benchmark-results/lighting-enabled`,
`render-benchmark-results/lighting-vertex-final` and
`render-benchmark-results/lighting-disabled-diagnostic`.

### 400 MMC at night

A one-run Cotterell comparison used the installed 400 MMC
(`E400MMC_ADL_10.9m_Voith_LowHeight`), 2560x1440, `OPENBUS_MAP_AI_HOUR=23`,
100 map AI configured, 60 warmup frames and 120 frames in each of the five
benchmark phases. VSync was disabled and the frame rate was uncapped. The
fragment-lighting run averaged 19.68 ms per measured frame (p90 24.04 ms);
the outside baseline phase averaged 17.25 ms between frame starts (about
58 FPS), and the driver-camera dashboard phase averaged 23.23 ms (about
43 FPS). Main-view GPU interval averages ranged from 13.15 to 15.70 ms.

With `OPENBUS_MAP_LIGHT_VERTEX_EVALUATION=1`, the same workload averaged
8.05 ms per measured frame (p90 10.59 ms), and main-view GPU interval averages
ranged from 2.68 to 3.22 ms. The outside baseline measured about 119 FPS and
the driver-camera dashboard phase about 211 FPS by frame-start intervals.
Both runs selected up to 85 scenery lights. Neither uploaded vehicle
spotlights, so these runs do not measure player or AI headlight rendering.

This confirms a large night-time scenery-light cost for the fragment path and
brings the main-view GPU time below 5 ms with vertex evaluation, but it does
not reproduce the reported 150-FPS-outside/60-FPS-inside behavior. These are
single-run whole-view measurements, not isolated lighting timings; the vertex
path remains opt-in pending fixed-scene image validation. The benchmark logs,
traces and CSVs are in
`render-benchmark-results/night-400mmc-default-filtered` and
`render-benchmark-results/night-400mmc-vertex`.

The benchmark runner now accepts an explicit AI bus/model configuration and
count so vehicle-headlight tests do not depend on map AI having spotlight
definitions. A follow-up Cotterell night test used 100 additional instances of
the 400 MMC in the runner's configured grid, with map AI, map buses and map
humans disabled. It ran at 2560x1440 with 60 warmup frames and 120 frames in
each benchmark phase. In the unforced fragment run, the selected vehicle-light
maximum was 100 per view, confirming that the AI buses naturally enabled
lights at 23:00. The forced run logged 101 loaded vehicles and selected up to
101 vehicle lights per view, adding the player's first spotlight.

| 100-AI test mode | Mean frame time | P90 | P95 | Sample-weighted main-view GPU |
|---|---:|---:|---:|---:|
| Fragment, AI lights as scripted | 71.57 ms | 102.63 ms | 103.69 ms | 58.78 ms |
| Fragment, spotlight 0 forced on all 101 vehicles | 79.19 ms | 101.67 ms | 102.93 ms | 66.01 ms |
| Vertex-evaluated scenery lights, AI lights as scripted | 112.73 ms | 160.76 ms | 162.41 ms | 107.67 ms |

These are single runs of an intentionally dense grid of identical buses, not
representative map traffic. The high frame times and wide interval variation
make the 7.6 ms difference between the two fragment runs inconclusive as a
spotlight-only cost. CPU `RenderLoop::prepareVehicleSpotlights` scopes averaged
0.076-0.092 ms per call; the renderer's scenery-light preparation scope
averaged about 0.011 ms. The vertex path was slower in this dense-bus run, so
the earlier single-bus vertex result must not be generalized to this workload
or used to change the default. Main-view GPU timing is interval-sampled and
includes whole-view work; it does not isolate lighting.

These runs confirm that AI headlights can be exercised and counted, but still
do not reproduce the reported 150-FPS-outside/60-FPS-inside result. The
100-bus stress test is not a matched reproduction of the user's map traffic
or bus/camera positions. Its logs, traces and CSVs are in
`render-benchmark-results/night-400mmc-100ai-no-forced-spotlights`,
`render-benchmark-results/night-400mmc-100ai-spotlights` and
`render-benchmark-results/night-400mmc-100ai-vertex`.

### AI spotlight scheduler A/B

A same-build night A/B used the same Cotterell 2560x1440 scene and 100
configured 400 MMC AI buses, with map AI, buses and humans disabled. Both runs
used 60 warmup frames and 600 measured frames. The scheduled run used the
defaults above; the control set all three distance-tier intervals to one
frame. The scheduler log showed 20-frame average/maximum intervals for the
farther vehicles after initialization, while the control reported one-frame
intervals.

| AI spotlight update mode | Mean frame time | P90 | P95 | Sample-weighted main-view GPU | Sample-weighted reflection GPU |
|---|---:|---:|---:|---:|---:|
| Distance scheduled | 77.57 ms | 110.30 ms | 115.74 ms | 64.90 ms | 3.24 ms |
| Every frame | 78.38 ms | 112.04 ms | 116.04 ms | 66.28 ms | 3.24 ms |

These are single runs with substantial frame-time variation; the small
difference does not demonstrate a meaningful performance gain. The scheduler
reduces far-vehicle refresh frequency as intended, but per-view transforms,
culling, and GPU lighting remain, and the dense-bus workload may be dominated
by other rendering work. The results are not representative of ordinary map
traffic and do not resolve the reported FPS gap. Logs, traces and CSVs are in
`render-benchmark-results/night-400mmc-100ai-scheduled` and
`render-benchmark-results/night-400mmc-100ai-every-frame`.

### 400 MMC camera-isolation night check

To separate player-bus rendering from traffic, two Cotterell runs used the
default 400 MMC player bus, 2560x1440, 23:00, no map AI/buses/humans, and the
benchmark-only forced player spotlight. Each used 60 warmup frames and two
repeats of 120 frames per camera phase. The measured lighting set contained up
to 85 scenery lights and one vehicle spotlight. The vertex run enabled
`OPENBUS_MAP_LIGHT_VERTEX_EVALUATION=1`; the fragment run kept the default
fragment path.

| Scenery-light path | Mean frame time | P90 | P95 | Sample-weighted main-view GPU | Sample-weighted reflection GPU |
|---|---:|---:|---:|---:|---:|
| Fragment | 20.76 ms | 25.50 ms | 26.52 ms | 15.48 ms | 1.87 ms |
| Vertex-evaluated | 8.77 ms | 11.73 ms | 12.44 ms | 3.37 ms | 1.92 ms |

In the vertex run, CPU `RenderLoop::draw` scopes averaged 0.76 ms in the
outside baseline phase and 0.91 ms in the driver-camera/dashboard phase;
`RenderLoop::draw.model` averaged 0.22 and 0.23 ms respectively. This does not
reproduce an outside/inside 150/60 FPS split. The aggregate main-view GPU
samples are not tagged by camera phase, so the data does not rule out a
camera-specific GPU cost in the user's setup. It does show the full main-view
GPU average below 5 ms with the player spotlight and 85 scenery lights active
in the opt-in vertex mode; CPU frame time remains above 5 ms, and the
appearance-changing vertex path remains opt-in pending image validation.
These are single runs, not a controlled repeated comparison. Artifacts are in
`render-benchmark-results/night-400mmc-camera-isolation` and
`render-benchmark-results/night-400mmc-camera-isolation-vertex`.

The same vertex workload with synchronous GPU completion before swap (the
setting enabled by `profile.bat`) averaged 10.05 ms per frame (P90 12.84 ms,
P95 13.62 ms) and 3.27 ms sample-weighted main-view GPU time. Its outside and
driver-camera `RenderLoop::draw` scopes remained 0.78 and 0.91 ms on average.
This setting added about 1.28 ms to the single-run mean here, so it is worth
disabling for normal profiling but does not explain a 60 FPS cockpit-only
result. Artifacts are in
`render-benchmark-results/night-400mmc-camera-isolation-finish-before-swap`.

### Experimental vehicle spotlight tile culling

The optional screen-space spotlight mask projects a conservative pyramid
around each finite spotlight cone, clipping it against the near plane before
calculating tile bounds. If bounds are unsafe or numerically ambiguous they
cover the viewport; when average coverage exceeds 75%, the original global
spotlight loop is used. Enable the experiment explicitly with
`OPENBUS_VEHICLE_SPOTLIGHT_TILE_CULLING=1`. It remains disabled by default
because no in-game performance benefit has been established.

A same-build night A/B used Cotterell at 23:00, a 400 MMC, 2560x1440, 100
configured 400 MMC AI vehicles plus the player, forced spotlight 0 on all
vehicles, no map AI/buses/humans, 1024-pixel reflections at interval 1, 60
warmup frames and 120 frames in each benchmark phase (one repeat). Trace events
shorter than 20 microseconds were omitted to keep the trace below its event
limit.

| Spotlight path | Mean frame time | P90 | P95 | Sample-weighted main-view GPU | Sample-weighted reflection GPU |
|---|---:|---:|---:|---:|---:|
| Global loop | 80.05 ms | 104.19 ms | 105.53 ms | 70.44 ms | 4.00 ms |
| Tiling enabled, density fallback | 84.47 ms | 105.50 ms | 107.57 ms | 72.87 ms | 4.59 ms |

Both runs selected up to 101 vehicle spotlights. The tiling run reported no
mask uploads and used the global fallback for every set: the bounds remained
too dense to cull. These are single runs with substantial frame-time and GPU
timing variation, so the difference is not a controlled estimate of fallback
overhead. The experiment provides no demonstrated benefit for this workload
and remains opt-in; sparse-scene benefit and in-game image correctness remain
unverified. Logs and traces are in
`render-benchmark-results/night-400mmc-100ai-tile-adaptive-off-20us` and
`render-benchmark-results/night-400mmc-100ai-tile-adaptive-on-20us`.

The coarse sphere preflight then reduced the observed setter trace duration in
another enabled run: median 0.107 ms and P90 9.42 ms across retained
`setVehicleSpotlights` trace events, versus 0.111 ms and 11.12 ms in the prior
enabled run. The new run averaged 83.18 ms/frame and 71.17 ms sample-weighted
main-view GPU time, and again used the global fallback for every set. This
small single-run CPU-scope change is suggestive only; the same-build global
control did not find the benchmark dashboard click target, so it is not a
valid paired frame-time comparison. The dense test remains far above the
5-ms goal and the preflight does not reduce GPU light evaluations. Artifacts
are in `render-benchmark-results/night-400mmc-100ai-tile-sphere-preflight`
and `render-benchmark-results/night-400mmc-100ai-tile-sphere-preflight-global`.

The coarse bound was subsequently tightened to a sphere enclosing the
finite-angle spotlight cone. This preserves a sparse unit-test case for a
narrow beam, but did not change the in-game fallback decision: a player-only
run (one light), a player plus one AI bus (two lights), and a 100-AI run (up to
101 lights) all reported zero mask uploads and fallback on every set. In the
player-plus-one-AI comparison, the one-run global and opt-in tiled modes
averaged 20.38 and 21.70 ms/frame, with sample-weighted main-view GPU timings
of 14.59 and 15.68 ms. These runs are too short and variable to attribute the
difference, and the mask path did no culling. In the latest 100-AI run, mean
frame time was 85.80 ms and sample-weighted main-view GPU time was 67.57 ms.
These historical results showed no in-game benefit for the tested 400 MMC
spotlights.
Artifacts are in
`render-benchmark-results/night-400mmc-player-only-tile`,
`render-benchmark-results/night-400mmc-1ai-tile-global`,
`render-benchmark-results/night-400mmc-1ai-tile-enabled` and
`render-benchmark-results/night-400mmc-100ai-tile-final`.

A focused OpenGL image probe now exercises an active sparse mask with a
nonzero viewport origin and a contributing light in the second 32-bit mask
word, including a cone crossing the near plane. For both model and material
shaders, the image matches the global-loop fallback within one 8-bit channel
value and includes lit pixels on both sides of a tile boundary. A randomized
cone-point test also checks that visible samples are never omitted. These
synthetic checks do not replace fixed-scene or reflection image comparisons.

After near-plane clipping replaced the sphere preflight, a 100-AI night run
still fell back for every set and averaged 85.05 ms/frame. A player-only run
now activated masks for most sets, but averaged 23.02 ms/frame versus 22.67 ms
for one same-build global-loop control; sampled main-view GPU averages were
16.63 and 16.70 ms. These short runs establish no performance win. The
player-only mask uploaded over 1 MB in some two-second intervals, despite
having just one spotlight. Artifacts are in the session files directory under
`night-400mmc-100ai-clipped-tiles`, `night-400mmc-player-clipped-tiles` and
`night-400mmc-player-clipped-global`.

The tiled shader was then changed to iterate only set mask bits, in increasing
light-index order. Image probes cover contributing indices 31 and 32. A
same-build player-only night pair measured 21.92 ms/frame with the mask and
21.75 ms with the global loop; sample-weighted main-view GPU averages were
15.60 and 16.01 ms. This single pair does not establish a performance win.
Artifacts are in `night-400mmc-player-bit-scan-tiles` and
`night-400mmc-player-bit-scan-global` in the session files directory.

### Fixed night image comparison

`OPENBUS_BENCHMARK_CAPTURE_DIR` now captures the standard camera views after
benchmark warmup. At 23:00 on Cotterell, a 400 MMC with a forced player
spotlight, no AI traffic, 2560x1440 output, and transparent reflections
enabled was captured in fragment, vertex, fragment order. All three reported
two readiness frames. The two fragment controls were byte-identical in all
six RGB images. Comparing vertex with fragment, the maximum per-channel
8-bit difference was 89 in the dashboard view and 78 in the front view;
1.04% and 8.09% of their pixels respectively differed by more than five
levels in at least one channel. The other views ranged from 0.09% to 7.03%.
This one scene demonstrates measurable differences, not an accepted visual
error limit, so vertex lighting remains opt-in.

Another capture with vehicle tile culling enabled was RGB-identical to the
fragment control in all six views. The capture log confirms the setting was
enabled, but does not record per-view mask activation; the synthetic probes
above separately verify active-mask image equivalence. Images and logs are
in the session files directory under `night-fixed-fragment-a`,
`night-fixed-fragment-b`, `night-fixed-vertex` and `night-fixed-tiled`.
These captures include visible mirrors but do not separately compare mirror
render targets or cover AI lights.

The no-limit scenery selection was also compiled and run in the same 100-AI
night stress scene. The 20-microsecond trace retained only two
`MapRenderer::prepareSceneryLights` events, which is not enough to claim a
CPU-time improvement; all 85 selected scenery lights remained active. The run
completed at 80.08 ms/frame. Artifacts are in
`render-benchmark-results/night-400mmc-100ai-static-selection`.

After making loaded scenery sources resident, one 23:00 Cotterell run with a
400 MMC player, 100 configured 400 MMC AI vehicles, forced spotlights,
2560x1440, 1024-pixel reflections every frame and 600 measured frames averaged
78.68 ms/frame (p95 106.50 ms). It selected up to 85 scenery lights and 101
vehicle spotlights. The initial static upload was 2,720 bytes and did not
recur in later logged intervals. This stationary run does not exercise tile
transitions or demonstrate an improvement over the previous run; it is not
evidence that the 5 ms target has been met. Artifacts are in the session
files directory under `night-400mmc-100ai-resident`. A follow-up run after
adding the inactive-invalid-source fallback averaged 87.66 ms/frame (p95
115.12 ms), but a normal build ran concurrently, so that result is not a
controlled performance comparison. Its artifacts are under
`night-400mmc-100ai-resident-final`.

An experimental shader branch rejected vehicle spotlights whose fragment was
clearly behind the cone, before normalizing the light ray. A same-build,
single-pair test in the same 100-AI night scenario averaged 90.13 ms/frame
without the branch and 91.65 ms with it; sample-weighted main-view GPU
interval averages were 78.50 and 79.70 ms respectively. The samples may
include warmup and the single pair is too variable to quantify a regression,
but it showed no benefit, so the experimental branch and its setting were
removed. Results are in the session files directory under
`night-400mmc-100ai-early-reject-off` and
`night-400mmc-100ai-early-reject-on`.

With influence-aware tile indexing, a further run of that night scenario
averaged 83.28 ms/frame (p95 107.50 ms), with up to 85 scenery lights and 101
vehicle spotlights. The sampled main-view GPU interval average was 72.81 ms;
these samples can include warmup. This run does not exercise a tile transition
and is not a controlled comparison. Its artifacts are in the session files
directory under `night-400mmc-100ai-influence-index`. Synthetic tests cover
selection and rendering across a tile boundary, including the large-radius
fallback. This historical run predates map-wide registration and exact
selection of registered inverse-square tails.

### Compute-indirect culling trace

A 400 MMC night trace reported 1.77 ms over three CPU-side indirect-culling
dispatch scopes and 4.85 ms in `glFinish`. These scopes overlap their parents
and do not isolate GPU lighting. A subsequent Cotterell 23:00, 2560x1440
player-only test used a forced player spotlight, no map AI/buses/humans,
1024-pixel reflections at interval 4, minimum-object thresholds of 1% in the
main view and 10% in mirrors, and two runs of 300 measured frames. With
finish-before-swap enabled, CPU culling averaged 17.63 and 17.66 ms/frame;
compute-indirect averaged 17.41 and 17.14 ms/frame. These sequential runs are
not the user's exact scene and do not demonstrate that CPU culling is faster.

Additional compute-dispatch trace scopes separated error checking and saved
state from actual buffer and dispatch calls. In another single 300-frame
compute-indirect run, the pre-dispatch `glGetError` loop averaged 0.439 ms per
call with finish-before-swap and 3.470 ms without it; both had 1,178 calls in
the measurement. The post-dispatch error check averaged 0.025 ms per call in
both runs. Whole-frame averages were 20.45 and 18.69 ms respectively. The
increased pre-dispatch wait without `glFinish` shows where the driver can
apply backpressure; it does not prove that removing error checking would save
that time rather than move the wait elsewhere. No preceding GL error warning
was logged. Preserve error attribution and fallback until a controlled
alternative is validated. Trace instrumentation itself adds overhead.
Artifacts are in `render-benchmark-results/night-400mmc-culling-detailed` and
`render-benchmark-results/night-400mmc-culling-detailed-no-finish`.

For future measurements, `OPENBUS_GPU_TIMING=1` enables asynchronous GPU
timings and `LIGHTING_UPLOAD` interval summaries. CPU `lighting` trace scopes
separate scenery, player and AI spotlight preparation. Experimental vehicle
tile-mask counts are logged when the opt-in path is enabled; they do not
represent per-fragment evaluations, and scenery per-tile or per-cluster counts
are not available yet. See [README.md](../README.md) for profiling controls.

## Validation completed

- `cmake --build build-ninja --target OpenBus OpenBusMaterialRenderingProbe OpenBusCameraMathProbe`
  passed.
- `ctest --test-dir build-ninja --output-on-failure -R '^(OpenBusCameraMathProbe|OpenBusMaterialRenderingProbe(\..*)?)$'`
  passed all 5 selected tests.
- `cmake --build build-benchmark-ninja --target OpenBus OpenBusVehicleSpotlightTileMaskProbe OpenBusMaterialRenderingProbe`
  passed after the adaptive vehicle-spotlight fallback change.
- `ctest --test-dir build-benchmark-ninja -R "OpenBusVehicleSpotlightTileMaskProbe|OpenBusMaterialRenderingProbe" --output-on-failure`
  passed all 8 selected tests, including the sparse-mask, dense-fallback,
  multi-word mask and active-mask image probes.
- GPU probes cover a contributing 250th light, explicit vehicle limits,
  combined scenery/player/AI lights, multiple view transforms, static-source
  changes and renderer lifecycle. A 250-light, 32x32 framebuffer comparison
  matched persistent and direct paths for two transforms and both material
  shaders. This is not a 250-light in-game benchmark.
- A fixed-scene in-game image comparison, including reflection captures, is
  still outstanding. The earlier capture script relied on a different
  instrumented build; the current benchmark runner disables startup capture.

## Reflection configuration

The current [RendererReflection.cpp](../src/render/RendererReflection.cpp)
uses defaults of 1024 pixels and interval 1 when the environment variables
are absent, as documented in [README.md](../README.md).
[profile.bat](../profile.bat) explicitly sets 1024 and interval 4. Set the
desired values explicitly in comparisons. A reflection target renders on its
first visible frame; subsequent refreshes are controlled per target.
Reflection shading separately applies a default `0.25` ambient floor, even at
night. `OPENBUS_REFLECTION_AMBIENT_FLOOR=0` disables this mirror-only floor
for visual comparisons; intermediate values are supported from 0 to 1. This
changes appearance, not lighting cost.

## Remaining work, in priority order

1. **Validate the persistent path in-game.** Capture matched fixed scenes and
   reflections for persistent and direct modes, then repeat longer,
   interleaved same-binary measurements. Include static-source reuploads
   during tile changes. Decide whether the default is worthwhile based on both
   image correctness and GPU/CPU/upload results.
2. **Design unified spatial assignment.** Include player, AI and scenery
   lights in the same per-view tile or cluster pipeline; support transparent
   surfaces, reflection views and capacity overflow without silently dropping
   lights. Scenery lights have nonzero inverse-square tails beyond their
   configured radius: radius-only culling is not exact. Preserve those tails
   or establish and validate an explicit error policy before culling them.
3. **Complete the benchmark matrix.** The explicit 100-AI headlight case and
   player-plus-AI forced-light case are now covered, but require repeated,
   interleaved runs. Still measure approximately 250 scenery lights, player
   headlights on their own, dense overlap and reflections. Record category
   CPU time, isolated GPU lighting time, upload volume, evaluated lights,
   tile/cluster distributions and average and maximum AI update frequency.
   Do not achieve gains by silently reducing light counts or disabling lights.
4. **Validate reflection trade-offs** with matched 1024/1 and alternative
   settings, including visible mirror quality. The existing benchmark runs
   used 1024/1 and do not establish the quality or performance of alternatives.

## Worktree note

No commit was created. The worktree contained unrelated modified and
untracked files before this handoff; preserve them when continuing.
