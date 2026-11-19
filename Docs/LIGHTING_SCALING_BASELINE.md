# Synthetic scenery-light scaling baseline

`OpenBusLightingBenchmark` measures the existing renderer with a deterministic
collection of view-space scenery point lights and a single full-frame material
draw. It validates that the renderer selected the requested light count and that
the draw covers all four framebuffer corners. The default persistent path uses
the same static-light buffer interface used by the map renderer; `--storage
direct` exercises the alternate direct buffer path.
`--layout sparse` distributes lights over a wider area; the default dense
layout concentrates them near the view.

Add `--vehicle-lights N` for N synthetic vehicle spotlights. They move on each
draw by default; `--move-vehicles 0` holds them still. The tool reports the
requested and selected counts, upload bytes and, when moving, setter-time
mean and p95. These synthetic lights do not exercise vehicle scripts, player
priority, AI scheduling or visibility selection. Upload bytes include the
initial set and warmup and measured draws.

The tool requests an OpenGL 4.3 core context and retries 3.3 if unavailable. It
reports the actual GPU, driver, and framebuffer size. It synchronously reads
each `GL_TIME_ELAPSED` result, so the measured GPU draw includes material work
as well as lighting and is not an asynchronous frame-time measurement. The
moving-vehicle GPU update timer includes buffer uploads and, when enabled,
GPU tile assignment; it is not an isolated culling timer. The
CPU initial setter time includes validation and uploads. With
`--move-map-lights 1`, additional per-view setter and GPU update timings are
reported; these still do not measure registration or complete frame time.
`--spacing-multiplier N` expands the chosen synthetic layout by N. No game
scene, vehicle geometry, reflection, or shadow is measured. Light counts are
synthetically submitted to the renderer, not registered map objects.
`--layout sparse-prefix` puts the first 100 lights at the same distant
view-space position and the rest far offscreen with small radii. It is an
intentionally artificial test of source-order bias, not a representative scene.

`OpenBusMaterialRenderingProbe.large-light-image` separately verifies that
the 3,001st scenery light and vehicle spotlight contribute. It compares
persistent and direct scenery-light framebuffer images for two view transforms
and both model and material shaders. This is a small synthetic image check,
not an in-game reflection or visual-quality comparison.

## Initial observation

One sequential run on an NVIDIA GeForce RTX 3080 Ti, OpenGL 4.3, driver 621.14,
used a 320x180 framebuffer, 10 warmup draws, 40 measured draws and persistent
storage. The single-run values are not a controlled comparison against an
optimized implementation or evidence of game-scene frame rates.

| Submitted and selected lights | GPU draw mean (ms) | GPU draw p95 (ms) | Initial CPU setter (ms) |
| ---: | ---: | ---: | ---: |
| 0 | 0.0042 | 0.0051 | 0.0145 |
| 500 | 0.2204 | 0.2376 | 0.0344 |
| 1,000 | 0.4606 | 0.5591 | 0.0409 |
| 2,000 | 0.9818 | 1.1203 | 0.0634 |
| 3,000 | 1.3748 | 1.5299 | 0.1388 |
| 4,000 | 2.0059 | 2.2159 | 0.1429 |

These observations support investigating the per-fragment global light loop;
they do not establish that the overall game meets any performance target.
Repeat and interleave runs on the target machine before drawing conclusions.

Three subsequent rounds interleaved the 500 through 4,000 counts in the same
order, with the same framebuffer and sample counts. The ranges below are
across the three per-run means, not percentiles or before/after comparisons.
The GPU was not isolated from other workloads.

| Submitted and selected lights | GPU draw mean range (ms) |
| ---: | ---: |
| 500 | 0.1924–0.2243 |
| 1,000 | 0.3909–0.4641 |
| 2,000 | 0.8492–1.0203 |
| 3,000 | 1.3382–1.6047 |
| 4,000 | 1.7894–2.0548 |

A separate run with 3,000 scenery lights and 100 moving synthetic vehicle
spotlights used the same resolution and sample counts. It reported a 1.5301 ms
mean GPU draw, 1.6988 ms p95, and 0.0044 ms mean vehicle-light setter time
(0.0085 ms p95). This was not an interleaved comparison and does not isolate
the cost of the vehicle lights. A later sparse-layout run with the same counts
reported 1.5522 ms mean GPU draw and 1.6876 ms p95. These runs are not
interleaved, but the sparse result illustrates that the existing scenery
shader still evaluates selected distant lights rather than using per-tile
light lists.

Spotlight selection now computes each candidate's view-space distance once,
then performs the existing stable player-first/distance sort. In three
interleaved standalone CPU microbenchmark pairs with 20 warmups and 200
measured sorts per run, the following ranges are per-run means:

| Candidates | Original sort (ms) | Cached-distance sort (ms) |
| ---: | ---: | ---: |
| 500 | 0.0608-0.0671 | 0.0253-0.0323 |
| 1,000 | 0.1292-0.1413 | 0.0298-0.0691 |
| 2,000 | 0.2924-0.3732 | 0.0665-0.1289 |
| 3,000 | 0.6034-0.6548 | 0.0972-0.1312 |

These are synthetic selection-only CPU timings on the local Windows machine;
copying input candidates, collecting lights, uploads and complete frame costs
are excluded. The process was not CPU-isolated. The current implementation
can be measured with `OpenBusSpotlightSelectionBenchmark 3000`; the original
results used the same deterministic workload before the change.

The experimental dynamic spotlight-shadow path is enabled by default when
the renderer supports it; set `OPENBUS_SPOTLIGHT_SHADOWS=0` to disable it.
It uses up to 64 atlas tiles for player and distance-prioritized AI lights,
renders opaque and supported cutout casters, and samples the atlas in the main
and reflection views. An adaptive frame-rate-based update budget ranges from
one to eight tiles per frame. Shadow visibility fades as slots become selected,
unselected or too stale, but moving cached shadows may still change abruptly
when their maps refresh. At the minimum budget one update in eight is reserved
for due AI shadows to prevent complete starvation; many AI shadows can still
remain stale and fade out under sustained overload. Reflection-frustum selection uses previously known
reflection views. The OpenGL probes cover tile isolation, atlas state,
projection and receiver sampling; a comprehensive in-game visual assessment
and representative performance validation are still required.

An initial, non-interleaved Cotterell night comparison at 1280x720 with 100
forced-light AI vehicles and 1024-pixel reflections every frame measured
139.0 ms mean frame time with shadows disabled versus 245.9 ms enabled (50
measured frames per run, 20 warmups, before the adaptive budget was added).
This is a heavy scenario, not an isolated shadow cost or proof of a stable
regression magnitude. A later run with the adaptive budget and corrected
shadow-record indexing measured 176.6 ms over 75 frames after 30 warmups;
the immediately preceding disabled run measured 135.2 ms with the same
settings. A further run with fairness scheduling measured 180.4 ms. These
single runs are not an interleaved or statistically controlled comparison.
The shadow GPU timers and counts are exported to the benchmark
CSV. Cotterell also reported 86 skipped scenery model configuration
diagnostics, so this run does not validate all scenery.
Enabling experimental GPU spotlight culling in another equivalent single run
produced no GPU tile-mask sets and 327 fallback sets; its 173.7 ms mean
therefore does not demonstrate a tiled-path speedup.
The short on/off night captures show a lit road but do not place a clearly
visible moving occluder in the beam; they do not establish shadow quality or
freedom from temporal artifacts in the game.

Moving the spotlight shader's distance square root after cone rejection was
tested and rejected. In three interleaved pairs with 3,000 moving spotlights,
no scenery lights, 320x180, 20 warmups and 60 draws, original shader means
were 0.7209, 0.6116 and 0.6170 ms; modified shader means were 0.6885, 0.6931
and 0.6963 ms. This did not demonstrate a consistent gain and the original
shader was retained. These timings include material work and are not frame
times; the GPU was not isolated from other workloads.

For the sparse-prefix workload, three interleaved exact/GPU pairs used
1280x720, 10 warmups and 60 measured draws. The table shows ranges of the
per-run means. Every GPU run activated 71 masks with no density fallback;
the former first-128 sample would reject this layout because the first 100
bounds cover the viewport. The GPU draw includes material work and the CPU
setter includes projection and upload. Neither measures whole-frame cost.
At 3,000 lights, estimated mean occupancy was 100 lights per tile.

| Lights | Exact GPU draw (ms) | GPU mask draw (ms) | Exact CPU setter (ms) | GPU mask CPU setter (ms) |
| ---: | ---: | ---: | ---: | ---: |
| 500 | 2.0961-2.3603 | 0.5604-0.5615 | 0.0059-0.0093 | 0.0517-0.0554 |
| 1,000 | 4.2303-4.2556 | 0.5730-0.5745 | 0.0147-0.0170 | 0.0891-0.0951 |
| 2,000 | 8.7177-8.7538 | 0.5884-0.5957 | 0.0183-0.0284 | 0.1829-0.2365 |
| 3,000 | 13.9440-14.0486 | 0.6092-0.6103 | 0.0471-0.0539 | 0.2769-0.2978 |

For a separate spotlight-only sparse workload, three interleaved rounds used
3,000 moving vehicle lights, no scenery lights, a 320x180 framebuffer, 20
warmups and 60 measured draws. With the existing optional CPU-built spotlight
tile mask disabled, GPU draw means were 0.6107–0.7183 ms and CPU vehicle setter
means were 0.0531–0.0592 ms. With
`OPENBUS_VEHICLE_SPOTLIGHT_TILE_CULLING=1`, GPU draw means were
0.0120–0.0124 ms and CPU setter means were 0.2059–0.2120 ms. This artificial
workload has many off-screen spotlights; it does not demonstrate a game-scene
frame-time improvement or isolate the cost of GPU uploads. The option remains
disabled by default and does not cull scenery lights.

An experimental compute path uses the same CPU-projected cone bounds and
density fallback, then constructs the tile bit masks on the GPU. Enable it
with both `OPENBUS_VEHICLE_SPOTLIGHT_TILE_CULLING=1` and
`OPENBUS_VEHICLE_SPOTLIGHT_GPU_CULLING=1`. OpenGL 4.3 is required; unsupported
contexts, conservative buffer/dispatch limits, and compute failures fall back
to CPU mask construction. The shader does not read back the mask during normal
rendering. The GPU mask is capacity-checked and has no fixed per-tile light
limit; dense masks still fall back to the global loop. GPU tile assignment
counters are separate from CPU mask counters; their total assignments divided
by their tile count gives mean occupancy, but maximum GPU tile occupancy is not
currently reported.

When `OPENBUS_GPU_TIMING=1` is also enabled, asynchronous timestamp pairs report
`spotlight_cull_samples`, `spotlight_cull_avg_ms`, `spotlight_cull_max_ms` and
`spotlight_cull_dropped` in `GPU_TIMING` log intervals. These measure compute
dispatch plus the texture-fetch barrier, not CPU projection or buffer uploads.
They use the existing frame-sampling stride and can coexist with main-view and
reflection elapsed queries. Samples are per dispatch, potentially multiple
per frame; a full query ring drops samples rather than blocking. These fields
are also exported by `render_benchmark_gpu_timing.csv`. CSV fields are empty
for older logs that do not contain spotlight timestamp measurements.

## Experimental GPU scenery masks

`OPENBUS_MAP_LIGHT_GPU_CULLING=1` enables a separate OpenGL 4.3 scenery-light
tile mask. It defaults off. CPU-projected conservative screen bounds are
assigned to 32x32 tiles by the shared GPU compute program; the fragment shader
visits only indices in each tile. Moving and static receivers use the same
lists, and each map draw, including reflection draws, builds its own view.
Vertex-evaluated map lighting, unsupported hardware, dense coverage, capacity
limits and compute failures use the exact global loop instead. The mask is a
bitset, not a fixed-size per-tile light list; normal rendering does not read it
back. The renderer can use this path alongside GPU spotlight masks.

Scenery tails are nonzero at arbitrary distance. When the tile path is active,
it smoothly fades each selected light between squared distance-to-radius ratios
of `S` and `4S`, where `S = max(1, 4 * sum(max(R,G,B)) / 0.001)`, using the
uploaded float colours. In ideal arithmetic, the total removed contribution per
channel before material, exposure and display transformations is at most
0.00025. The margin is intended to tolerate floating-point rounding; it is not
a display-space bound. Selecting a different set or falling back to exact
lighting can change the fade, so temporal image quality and high-exposure
scenes still need validation. The heuristic skips masks above 45% estimated
coverage and may reject obviously dense sets from a deterministic sample of
up to 128 indices distributed across the input. Its
decision also changes whether the fade is applied; the fallback remains exact,
so transitions between these modes need temporal validation.

The GPU map-tile image probe additionally compares tiled and exact rendering
in a floating-point framebuffer at several distances through and beyond the
fade region. It exercises both material paths and direct and persistent
uploads, and checks a nonzero fade and a per-channel error of at most 0.000251
for these synthetic cases, including 100 overlapping colored tails among
3,001 selected lights at five view-space distances, with and without alpha
blending. This is not a display-space or real-scene temporal quality test.
The overlapping lights are at the beginning of the source list; the probe
also verifies that the distributed sample does not falsely classify that
otherwise sparse set as dense.

After switching to the distributed sample, one non-interleaved repeat of the
wide sparse 1280x720 synthetic workload gave GPU draw means of 0.0454, 0.1370,
0.5643 and 1.2688 ms for 500, 1,000, 2,000 and 3,000 lights respectively;
all 71 sets per count used GPU masks. CPU setter means were 0.0492, 0.0940,
0.1925 and 0.2959 ms. These single-run results are not a controlled comparison
with the earlier sampling heuristic or a whole-frame measurement.

A subsequent Cotterell comparison at 23:00 used 1280x720, 100 AI vehicles,
forced spotlights, 1024-pixel reflections every frame, 20 warmup frames and 50
measured frames. The exact run averaged 134.239 ms/frame (190.330 ms p95);
the run with map GPU culling requested averaged 131.985 ms/frame (188.772 ms
p95). This single sequential pair does not establish a performance improvement:
the requested GPU path recorded zero tiled sets and 254 exact fallbacks.
Intervals reported up to 345 registered and 86 selected scenery lights. Both
runs logged a map-load summary of 86 skipped scenery model configuration
diagnostics. An earlier pair accidentally ran at noon with no selected scenery
lights and is not a meaningful scenery-light comparison.

A separate instrumented night-scene run with the same settings reported 313
map-mask fallbacks, all due to estimated density; there were no compute,
capacity, viewport, dark-light, vertex-lighting or unavailable-resource
fallbacks in the logged intervals. It averaged 134.122 ms/frame (191.271 ms
p95). These counters cover logging intervals, not only measured frames.
`LIGHTING_UPLOAD` now reports these fallback reasons individually alongside
the total, to distinguish the heuristic from hardware or compute failures.
After the distributed-sampling change, another matching run still recorded
zero tiled sets and 292 density fallbacks; its mean was 145.375 ms/frame
(203.748 ms p95). These separate runs are not an interleaved comparison and
do not establish the performance impact of the heuristic change.

Vehicle category counters in `LIGHTING_UPLOAD` report the maximum number of
spotlight definitions on eligible loaded player and AI vehicles, and the total
number of player and AI light candidates that passed view-frustum selection.
Definitions include inactive alternatives; they are not counts of simultaneously
lit lights. Candidate totals are summed across `vehicle_selection_views`,
including reflection views, before renderer-side light limits.
In one matching 100-AI diagnostic run, logged intervals contained 238 selection
views, 238 player candidates and 23,800 AI candidates; maximum definition
counts were four player and 400 AI. This run's frame mean was 136.525 ms
(191.006 ms p95), and is not an isolated cost measurement.

`LIGHTING_UPLOAD` reports map GPU tile sets, fallback sets, tile count and
assignments. Assignments divided by tiles is the exact mean occupancy of the
conservative rectangular masks. With GPU timing enabled, maximum tile
occupancy is computed on sampled frames on the CPU; `map_gpu_tile_max_samples`
indicates whether a maximum was measured. `GPU_TIMING` additionally reports
`map_light_cull_*` asynchronous timestamp pairs for the compute dispatch and
texture barrier, and these fields are exported to
`render_benchmark_gpu_timing.csv`. They do not measure CPU projection,
uploads, full lighting or frame time. Sampling can drop queries without
blocking, and maximum-occupancy profiling itself adds CPU work.

Three interleaved pairs on an RTX 3080 Ti at 1280x720 used the synthetic sparse
layout with `--spacing-multiplier 4 --move-map-lights 1`, 10 warmups and 60
samples. These are deliberately widely distributed lights, not a game scene:

| Lights | Exact GPU draw mean (ms) | GPU-tiled draw mean (ms) | Exact CPU setter mean (ms) | GPU-tiled CPU setter mean (ms) |
| ---: | :--- | :--- | :--- | :--- |
| 500 | 2.1022-2.3615 | 0.0383-0.0384 | 0.0053-0.0075 | 0.0478-0.0486 |
| 1,000 | 4.2337-4.3889 | 0.1171-0.1371 | 0.0144-0.0184 | 0.0898-0.0920 |
| 2,000 | 8.8208-8.8604 | 0.4861-0.4867 | 0.0237-0.0299 | 0.1815-0.1966 |
| 3,000 | 14.0567-14.3882 | 1.2776-1.4588 | 0.0457-0.0510 | 0.2738-0.2834 |
| 4,000 | 18.9304-19.0560 | 2.4354-2.4384 | 0.0494-0.0551 | 0.3777-0.4089 |

These are synchronous whole-draw and setter microbenchmarks, not whole-frame
or isolated lighting timings. GPU update work and CPU projection increase.
In a less dispersed 320x180 synthetic layout, 2,000 and 3,000 lights took the
dense fallback in every measured set; 3,000-light CPU setter means were
0.0206-0.0242 ms exact versus 0.0403-0.0453 ms with the experiment requested.
The option remains disabled by default pending representative-scene and
temporal quality validation.

Three interleaved CPU/GPU assignment pairs with 3,000 moving spotlights and the
same sparse layout at 1280x720 gave CPU setter means of 0.2585–0.2668 ms
for CPU masks and 0.2120–0.2337 ms for GPU masks. A repeated set after
adding explicit dispatch error checks measured 0.2505–0.2524 ms and
0.2181–0.2315 ms respectively. In that repeat, GPU update means, including
uploads, were 0.0127–0.0130 ms and 0.0505–0.0523 ms respectively; GPU draw
means were 0.1083–0.1086 ms in both cases. These synchronous microbenchmarks
do not establish whole-frame savings: the GPU path moves work rather than
eliminating it and remains disabled by default.

For one longer synthetic 720p run with 3,000 moving spotlights and a five-frame
sampling stride, two timestamp intervals each reported 640 completed culling
samples, means of 0.030 and 0.029 ms, and 338 and 354 dropped samples.
These are not whole-frame timings and the dropped samples limit interpretation.

## Static view preparation

The tool also measures `StaticLightViewCache::prepare` with an unlimited
selection and a changing view transform. At 3,000 lights, three sequential
runs of 300 measured preparations after 100 warmups reported means of
0.0427, 0.0438 and 0.0430 ms before replacing the general four-component
transform with a three-component transform. After the change, corresponding
means were 0.0169, 0.0171 and 0.0171 ms. The runs used the same Release
configuration but were not interleaved, so they are not a controlled estimate
of game-frame savings. These timings exclude GPU uploads, map registration
and limited-selection sorting. The GPU fragment loop remains unchanged.

## Static tile selection

The benchmark also measures CPU tile-index selection separately from the GPU
draw. It alternates between two sets of 25 active 25-metre tiles, using a
synthetic 20-metre influence radius. This radius is independent of the
0.5-metre radius in the draw workload. `tile_selected_map_lights` is the count
for the last tile selection, not the number of lights selected for the GPU
draw. Neither the tile setup nor its selection time represents actual map
streaming or an exact cutoff for the shader's inverse-square light tails.

At 3,000 lights, sequential Release runs with 100 warmups and 300 measured
selections reported means of approximately 0.249–0.256 ms for the dense layout
and 0.021–0.025 ms for the sparse layout before the generation-mark
deduplication change. Three later runs reported dense means of 0.0213, 0.0208
and 0.0211 ms, and sparse means of 0.0058, 0.0088 and 0.0059 ms. These are
non-interleaved microbenchmarks, not measurements of whole-frame savings.
The dense case selected 3,000 lights and the sparse case selected 474 in the
last tile set. Dense identifiers use bounded reusable generation marks;
sparse or unusually large identifiers retain the exact sort/unique path.
Neither path discards lights to meet a capacity limit.

With application GPU timing enabled, `LIGHTING_UPLOAD` interval logs include
`map_selection_calls`, `map_registered_max`, `map_tile_candidates_total`,
and `map_tile_candidates_max`. Registered lights are the map renderer's
registered sources; tile candidates are selected before the configured
per-view light limit. A selection call is a map draw, including reflection
draws, and contributes zero candidates when lighting or active tiles are
absent. Candidates are not guaranteed to be visible or to contribute to a
fragment. The total divided by selection calls is the average per draw, not
per frame. Existing `map_lights_*` fields count lights submitted after
selection and may include other renderer calls.

When performance tracing is enabled, the nested
`MapRenderer::selectSceneryLightTiles` and
`MapRenderer::transformSceneryLights` scopes distinguish CPU index selection
from view preparation, respectively. Their parent
`MapRenderer::prepareSceneryLights` also includes uploads and setup.
Trace duration filters can omit short scopes; use an appropriate threshold and
check for trace truncation before interpreting their absence.

## Representative night scene

A separate short Cotterell run used 1280x720, 1024-pixel reflections refreshed
every frame, a MAN DL05 player and 100 configured 400 MMC AI vehicles with
forced headlights. Map-generated traffic was disabled. The run measured 150
frames after 20 warmup frames and reported a 125.58 ms average frame and
192.77 ms p95. Interval telemetry reported up to 86 selected scenery lights
and 101 vehicle spotlights. This is not a 3,000-light scene and is not an
interleaved before/after comparison. GPU timing was sampled asynchronously every
five frames, so per-interval reflection samples correspond to target refreshes
rather than whole frames.

A later telemetry-validation run with the 400 MMC as both player and AI bus
reported up to 345 registered scenery lights and 86 tile candidates per map
draw, with up to 101 submitted vehicle spotlights. It used the same resolution,
reflection settings and measured frame count but is not a controlled comparison
to the earlier run. A 1-microsecond trace threshold exceeded the event cap and
truncated the trace; rerunning with a 50-microsecond threshold completed, at the
cost of omitting shorter scopes.

Two additional single runs enabled spotlight tiling, first with CPU assignment
and then with GPU assignment. Interval logs showed 562 and 536 spotlight sets,
respectively, and every logged set took the density fallback to the global
loop. Their 142.1 ms and 136.7 ms mean frame times therefore do not measure a
GPU tile-assignment benefit. More representative workloads and repeated
comparisons are needed before enabling either experimental option by default.

`OpenBusMaterialRenderingProbe.large-spotlight-tile-image` compares the
existing tiled and global spotlight paths for a contributing light at index
3,000, including near-plane crossing and both material shaders. The global
comparison deliberately forces the tile-density fallback using additional
zero-colour lights. This is a synthetic image check, not validation of moving
vehicles or reflections in a game scene.

## Rejected scenery tail-fade prototype

A prototype faded inverse-square tails inside the existing global shader loop,
but did not avoid per-light buffer fetches. Its fade started at a squared
distance-to-radius ratio of at least four times the sum of the maximum RGB
component of each selected light, divided by a chosen error budget; it reached
zero at four times that squared ratio. In ideal arithmetic, the total removed
contribution per channel before material, exposure and display transformations
would be at most one quarter of the budget. This is not a display-space error
bound, and temporal quality when the selected set changes was not validated.

On an RTX 3080 Ti at 320x180, with the synthetic sparse persistent workload,
three interleaved pairs comparing the original exact shader against the
prototype with a deliberately large budget of 0.1 produced these per-run mean
GPU draw times (milliseconds):

| Lights | Exact | Fade |
| ---: | :--- | :--- |
| 500 | 0.2257-0.2277 | 0.2375-0.2391 |
| 1,000 | 0.4538-0.4641 | 0.4913-0.4974 |
| 2,000 | 0.7987-0.9865 | 0.9033-1.0670 |
| 3,000 | 1.2828-1.3403 | 1.4226-1.4383 |

Every paired run regressed, so the prototype was removed. These are whole
synthetic draw times, not isolated lighting or frame times; they do not prove
that a finite-tail approximation coupled to spatial lists would regress.
An earlier comparison mistakenly used a modified exact shader and suggested a
speedup; the table uses the original exact shader and supersedes that result.

## Reproduce

Build `OpenBusLightingBenchmark` using the project's configured CMake build.
For example:

```powershell
cmake --build build-benchmark-ninja --target OpenBusLightingBenchmark
foreach ($count in @(0, 500, 1000, 2000, 3000, 4000)) {
    & .\build-benchmark-ninja\OpenBusLightingBenchmark.exe --lights $count `
        --storage persistent --width 320 --height 180 --warmup 10 --frames 40
}
```

Inherited renderer settings can affect the result. In particular, a light
limit that prevents the requested count from being selected causes the tool to
fail rather than silently report a misleading measurement. Other quality and
driver settings should also be held constant and recorded for comparisons.
The benchmark does not force the GPU to be exclusive or detect competing GPU
workloads.
