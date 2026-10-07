# Render performance review and optimization TODO

**Input:** single supplied profile capture, 2026-10-07.  
**Status:** planning only; no renderer behavior has been changed.

## Reading the capture correctly

The values below are inclusive trace-scope time. Parent and child rows overlap, so
**they must not be summed**. The `Count` column is also greater than one for many
scopes; this is an aggregate of the capture, not proof that one game frame executes
eight times. The supplied `main` scope is 14.06 ms (about 71 FPS if it represents one
unblocked CPU frame), but this does not include unmeasured GPU execution or present
blocking outside the scope.

| Priority | Scope | Inclusive time | What it indicates |
| --- | --- | ---: | --- |
| P0 | `RenderLoop::draw.ground` / `MapRenderer::draw` | 6.518 / 6.514 ms | These are effectively the same map-rendering work, about 46% of `main`. Do not count both. |
| P0 | `RenderLoop::draw.reflections` / `ReflectionRenderer::render` | 1.900 / 1.898 ms | One reflection update is a meaningful 13.5% of `main`; target drawing accounts for virtually all of it. |
| P1 | `RenderLoop::beginFrame.pollEvents` | 0.882 ms | Event-pump time, not rendering. It may be OS scheduling/window-system work; optimize only after a repeatable CPU-side cause is demonstrated. |
| P1 | `RenderLoop::updatePlayerVariables` / `RenderLoop::updateScripts` | 0.821 / 0.811 ms | Script/host-variable update is a separate simulation hot path, not a draw-call issue. |
| P2 | `RenderLoop::draw.model` / `Vehicle::draw` | 0.783 / 0.781 ms | Vehicle rendering is presently much smaller than map and reflections. |
| P2 | `Vehicle::draw.opaquePass` | 0.494 ms | Most vehicle draw time is opaque submission. |
| P2 | `MapRenderer::updateScripts` | 0.489 ms | Scenery script execution deserves attribution before caching or lowering its cadence. |
| P3 | `Vehicle::drawBatch.prepareAndSubmit` | 0.356 ms across 348 calls | CPU work per submission averages 1.023 µs; the count may still create driver/GPU overhead. |
| P3 | `Vehicle::draw.classifyParts` | 0.140 ms | Visibility/material classification is already comparatively cheap. |
| P3 | `Vehicle::drawBatch.resolveTextures` | 0.104 ms across 88 calls | Texture resolution averages 1.182 µs; avoid speculative texture-cache redesign. |

`CoreRenderer::drawModelBatch` (1.973 ms across 1,970 calls) and
`CoreRenderer::drawIndexedModelBatch` (0.121 ms across 120 calls) are low-cost CPU
calls individually, but their high submission count means CPU scopes alone cannot
rule out a driver/GPU draw-call bottleneck.

## Existing safeguards and constraints

- `MapRenderer::draw()` already rejects invisible terrain, roads, chunks, and
  individual scenery instances. It then performs a draw for each visible scenery
  instance after its material/dynamic-texture handling and transform.
- Vehicle parts already use reusable opaque/transparent lists when the view and
  material/animation generations match. Preserve invalidation and sorting semantics.
- Reflection targets already have configurable size, interval, and maximum update
  rate: `OPENBUS_REFLECTION_SIZE`, `OPENBUS_REFLECTION_INTERVAL`, and
  `OPENBUS_REFLECTION_MAX_FPS`. Defaults are 1024, every frame, and uncapped.
- Texture upload must remain on the OpenGL thread. Do not move GL calls to worker
  threads while reducing `resolveTextures` cost.
- Transparent ordering, material changes, script/text textures, and reflection
  contents are correctness-sensitive. Any caching/batching work needs visual
  regression evidence as well as timing evidence.

## Ordered TODO backlog

### P0 — establish a trustworthy baseline

- [ ] Reproduce this workload with the existing benchmark harness at a fixed exact
  framebuffer resolution, VSync off, a fixed camera route, fixed assets, and at least
  five runs. Retain raw trace JSON, logs, and the CSV summaries.
- [ ] Report `main` frame-time average, p90, and p95 separately from inclusive scope
  totals. Break results down by benchmark phase; do not compare a single supplied
  frame against aggregate scope totals.
- [ ] Add GPU timing around main-map drawing, each reflection target draw, vehicle
  drawing, and presentation (for example, delayed OpenGL timer queries). Record
  query availability/disjoint/failure and consume results without synchronously
  stalling the CPU.
- [ ] Add a per-frame visible-work counter to the trace or benchmark output:
  visible terrain tiles/layers, roads, scenery chunks/instances, scenery draw calls,
  triangles/indices submitted, vehicle batches, reflection targets, target sizes, and
  reflection draw calls. Counters explain *why* a timing changes.
- [ ] Add nested scopes inside `MapRenderer::draw`: terrain base/layers, roads,
  chunk rejection, instance rejection, dynamic texture refresh, transform/material
  setup, opaque draws, and blended draws. Keep tracing disabled or sampled in normal
  interactive runs.
- [ ] Split `MapRenderer::updateScripts` into script-runtime execution, changed
  variable publication, and dynamic text/script-texture refresh/upload. Likewise,
  split `RenderLoop::updateScripts` and `updatePlayerVariables` enough to identify
  script execution versus host work.
- [ ] Verify whether `pollEvents` is wall-clock blocking by comparing it with CPU
  sampling/profiling and a minimized/idle window. Do not busy-poll or change event
  cadence merely to make this trace look smaller.

**Exit criterion:** five comparable captures have stable p50/p95 values, visible-work
counters, and CPU + GPU timing for map and reflection work.

### P0 — quantify and tune reflection work first

- [ ] Run an A/B matrix at the representative camera views: reflections disabled
  (diagnostic only), 256/512/1024 target size, intervals 1/2/4, and optionally a
  capped maximum update rate. Measure image quality, main-frame p95, CPU time, GPU
  time, and reflection staleness.
- [ ] Choose and document a quality preset based on the result. A likely low-risk
  first candidate is 512² and/or interval 2 for non-capture gameplay, but do not
  change the default until the matrix establishes acceptable mirror quality.
- [ ] Add per-target profiling for visibility, resize, clear, and draw, plus target
  size and whether the target was skipped. The current capture says draw dominates,
  but cannot identify the expensive scene content.
- [ ] Profile the reflection callback by content type (map terrain/layers, road,
  scenery, vehicle opaque, transparent). Validate whether distant map scenery is
  unnecessary in a mirror view before tightening reflection-specific culling.
- [ ] If target drawing remains GPU-bound, test reflection-specific quality controls
  in this order: smaller target, lower update frequency, tighter valid reflection
  frustum/distance, then an explicitly approved reduced-content reflection pass.
  Keep vehicle/mirror correctness and transparent-content policy documented.
- [ ] Avoid per-frame target resize churn; it should occur only when a requirement
  changes. The new counters should prove whether it occurs in the measured path.

**Exit criterion:** a documented reflection preset meets visual acceptance and improves
p95 frame time without hidden CPU/GPU stalls.

### P0 — reduce map submission only after attribution

- [ ] Use the new counters/scopes to identify whether time is dominated by terrain
  layers, road batches, opaque scenery, blended scenery, per-instance culling, or
  script-backed materials. Capture a near/medium/far camera position because the
  answer should change with visible content.
- [ ] Inspect the ratio of visible scenery instances to `drawModelBatch` calls. If
  many visible instances share immutable mesh/material state, prototype static
  batching by batch/material and spatial chunk, or GPU instancing where the target
  OpenGL 3.3 path supports it.
- [ ] Preserve chunk-level and per-instance frustum culling. Build batches per
  spatial chunk (not globally) so batching does not turn rejected geometry into GPU
  work. Keep transparent scenery in a separately ordered path.
- [ ] Make static opaque scenery the first batching target. Exclude script-visible,
  animated, text/script-texture, reflection-sensitive, and blended batches until
  their invalidation and ordering rules are explicitly designed and tested.
- [ ] Consider state sorting only inside a correctness-safe opaque chunk/bucket.
  Do not reorder transparent, no-depth, or alpha-tested behavior without screenshot
  comparisons.
- [ ] For terrain layers, measure layer count and overdraw before merging geometry.
  If layered terrain is GPU-bound, test texture-array/atlas or shader composition
  only with correct masks, UV scaling, filtering, and seasonal/detail extensibility.

**Exit criterion:** an A/B prototype lowers map CPU submission and/or map GPU time at
near and far viewpoints, while map screenshots match the baseline.

### P1 — script and host update work

- [ ] Attribute player scripts, vehicle scripts, and scenery scripts individually:
  count active runtimes, callbacks, executed instructions/native operations, changed
  variables, and texture uploads.
- [ ] Confirm unchanged text/script texture revisions do not cause rasterization or
  GL uploads. The existing revision cache should be measured before redesigning it.
- [ ] Skip work only when its documented cadence and visual behavior permit it:
  inactive/out-of-range scenery runtime throttling, event-driven variable updates,
  and coalesced dynamic texture uploads are candidates. Do not lower all scripts to
  a slower global rate without gameplay validation.
- [ ] Add deterministic script performance probes for a dense map and a dynamic
  dashboard so a cache/cadence change cannot silently freeze visibility or displays.

**Exit criterion:** a repeatable script breakdown identifies the largest runtime class,
and any reduction preserves scripted visual changes and input behavior.

### P2 — vehicle submission follow-up

- [ ] Add subscopes/counters for opaque draw state changes, material-batch draws,
  auxiliary texture checks, dynamic free/text texture updates, environment-map
  overlay draws, transparent depth prepass, transparent sorting/reuploads, and actual
  GL draw calls.
- [ ] Count driver-facing state changes (program, buffer, texture/sampler, blend,
  depth, cull) and compare them with GPU time. The current ~1 µs CPU average per
  `prepareAndSubmit` call is not enough evidence to optimize it.
- [ ] Verify prepared draw-list reuse in the opaque-to-transparent same-view path
  with a focused regression test before expanding it. Its context, material, model
  view, projection, and animation invalidation conditions are intentional.
- [ ] Investigate per-frame transparent triangle sorting only if the new scope shows
  material cost. It rebuilds/uploads a VBO, so any cache needs camera-dependent
  invalidation and visual ordering tests.

**Exit criterion:** vehicle work is addressed only if it limits the target p95 after
map/reflection work or GPU/state counters demonstrate a bottleneck.

## Experiments that should not become permanent changes automatically

- [ ] Toggle material batching, reflection transparent rendering, and reflection
  settings one variable at a time; use them as diagnostic A/B cases, not broad
  default changes.
- [ ] Do not remove culling, force texture residency, disable scripts, or disable
  transparency solely to lower a timing. Such runs are useful only to attribute cost.
- [ ] Do not infer that `swapBuffers` is cheap GPU presentation from its 214.5 µs
  total in this capture. Validate with GPU timers and both VSync-on/off runs.
- [ ] Do not prioritize `BusSimulation::fixedUpdate` (134 µs) or individual indexed
  map draws (about 1 µs average) ahead of the dominant map/reflection paths.

## Validation checklist for every accepted optimization

- [ ] Build the Release benchmark configuration and run the focused automated probes.
- [ ] Benchmark at the affected resolution(s), with the same route/camera and five
  or more runs. Compare average, p90, p95, visible-work counters, CPU scope time,
  and GPU time; retain before/after artifacts.
- [ ] Capture representative near/far map views, mirrors/reflections, opaque vehicle
  surfaces, transparent glass/decals, scripted scenery, and text textures.
- [ ] Verify no new GL errors, target-resize churn, texture uploads on the wrong
  thread, stale dynamic textures, culling pop-in, transparent ordering regression,
  or changed script cadence.
- [ ] Record the selected trade-off and rollback condition in this file or the
  benchmark documentation.

## Suggested agent sequence

1. Implement the P0 counters and nested trace scopes without changing rendering.
2. Produce the repeatable CPU/GPU baseline and reflection A/B matrix.
3. Select reflection defaults/preset only when visual acceptance is recorded.
4. Use map attribution to prototype one static-opaque, chunk-local batching or
   instancing change behind a switch.
5. Add regression coverage/screenshots, compare five-run p95 results, and retain the
   change only if it wins at representative near and far views.
6. Profile scripts and vehicle submission next; repeat the same attribution → isolated
   experiment → correctness validation cycle.

This order deliberately targets the 6.5 ms map path and 1.9 ms reflection update
before sub-millisecond vehicle micro-optimizations.
