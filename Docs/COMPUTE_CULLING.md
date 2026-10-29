# Compute visibility culling

## Default backend and regression fix

**CPU culling is the default**, including on compute-capable GPUs. The previous
default dispatched compute and synchronously read visibility back for map,
vehicle, and reflection views. Each readback could drain queued GPU rendering;
the dispatch cost was much larger than these small CPU visibility tests.

Leaving `OPENBUS_CULLING_BACKEND` unset, or setting it to `cpu`, uses the original
CPU spatial tests and chunk rejection. No compute input lists, dispatches, or
visibility readbacks are performed. Frustum/distance/LOD culling remains enabled;
this does not disable culling or reduce reflection quality/update frequency.

`OPENBUS_CULLING_BACKEND=compute-indirect` selects the implemented **no-readback
GPU command backend** described below. Other values select CPU.
Backend selection occurs at renderer initialization. The log identifies the
selected backend. The old `compute-readback` setting is retired and logs a
warning before selecting CPU; diagnostic probes still exercise readback to
compare GPU visibility with the CPU reference. No environment variable is
required for normal operation.

The renderer requests an OpenGL 4.3 core context, retrying OpenGL 3.3 if
unavailable. Compute culling initializes only when the actual context version
and optional compute/SSBO entry points satisfy the requirements **and the
experimental backend is explicitly selected**. Shader,
allocation, or dispatch failures retain the CPU visibility implementation.

## Experimental compute scope

- Map terrain, water, road bounds, and scenery bounding spheres use batched
  compute frustum and 10 km XY-bounds distance tests.
- Conservative CPU frustum, LOD, and projected-size broad phases remove clearly
  rejected scenery and vehicle parts before preparing indirect draws; retained
  candidates still receive the GPU's final test.
- Scenery and vehicle mesh LOD selection runs in compute.
- Optional projected-size tests for scenery and vehicle parts run in compute,
  using the current view's projection scale and configured threshold; terrain,
  roads, and water are not affected by that cutoff.
- Animated vehicle rendering bounds use compute tests. Reflection-surface
  scheduling uses a small current-view CPU test in all backends.
- Active tile membership, script variables, viewpoints, reflection target sizing,
  material resolution, transparent sorting, and draw submission remain on CPU.
- This replaces spatial visibility tests, not rasterizer back-face selection or
  depth testing. Hi-Z is not implemented; the optional depth-query prototype is
  described below.

Map scenery instances and vehicle parts can use a projected-size cutoff,
configured independently for the main view and mirror/camera views:

- `OPENBUS_MIN_OBJECT_SIZE_MAIN_VIEW_PERCENT` sets the main-view threshold.
- `OPENBUS_MIN_OBJECT_SIZE_MIRRORS_PERCENT` sets the mirror/camera threshold.

Values are percentages of viewport height, from `0` through `100`. Unset,
empty, or `0` values disable the cutoff for that view. Invalid values are logged
and ignored. The CPU backend applies the cutoff before submission; the optional
GPU backend first excludes objects certainly below the cutoff on the CPU, then
calculates the final decision in compute and gates indirect commands. Its
conservative broad phase retains uncertain and near-threshold bounds.
The traffic-light raster probe exercises red, green, and yellow phases with both
cutoffs disabled, including CPU and indirect rendering with queries enabled.
The GPU test uses float inputs and a small conservative boundary margin, so
objects near the cutoff may be retained when the double-precision CPU test
would reject them. Spheres reaching the camera plane and degenerate bounds are
retained conservatively. Size culling is not occlusion
culling; fully covered objects can still be submitted when they exceed the
configured threshold. The test uses conservative part or scenery bounding
spheres, not the exact projected height of a complete vehicle or mesh.

The window title reports submitted **candidate** triangles. For indirect draws,
this includes candidates that compute later suppresses, not only triangles
executed by the GPU. Reflection counts aggregate the targets drawn on the most
recent reflection refresh and include repeated passes such as depth prepasses.

## Experimental depth-query occlusion

Set `OPENBUS_OCCLUSION_CULLING=queries` to enable an opt-in opaque-map-scenery
prototype. It front-to-back sorts contiguous runs of ordinary opaque scenery
within each render-type group, leaving cutouts and special depth modes in place,
draws a conservative bounding cube against the current view's depth buffer
without changing color or depth, and uses `GL_ANY_SAMPLES_PASSED` plus
conditional rendering to skip the object's draw when the proxy is fully hidden.
The query is performed in the same view and frame, including reflection targets;
no previous-frame visibility is reused and no query result is read back by the
CPU. Unsupported query entry points fall back to ordinary rendering.

Set `OPENBUS_OCCLUSION_TRANSPARENT=1` in addition to enable queries for ordinary
transparent scenery after the opaque and transmap depth passes of its current
render-type group. This is off by default: transparent candidates add more
proxy draws and queries, which can reduce performance. They are tested against
the depth already present in the same view, without changing their back-to-front
order or depth-write behavior.

Set `OPENBUS_OCCLUSION_MAP_GEOMETRY=1` alongside the query setting to also test
bounded terrain tiles and ordinary opaque road batches against the depth already
written at their original positions in the render order. Terrain base and ground
layers share one query. Unbounded, transparent, or special-depth roads and
near-plane-crossing or invalid bounds render normally. This is off by default
because large tile bounds often remain visible and the additional query overhead
can cost more than it saves.

Set `OPENBUS_OCCLUSION_WATER=1` alongside the query setting to test each water
tile's conservative bounds against depth already present at the water pass.
The existing back-to-front order, blending, depth test and disabled depth writes
are preserved. Invalid or near-plane-crossing bounds, query allocation failure,
and proxy submission failure fall back to ordinary rendering. This option is
off by default: large tile bounds and query overhead may make it slower. The
water raster probe covers visible, hidden, partially covered and layered water
in CPU and indirect modes, including a texture-backed framebuffer; full mirror
and in-game behavior still need validation.

One short Mesa software-rendered `DBC_Map` comparison with indirect culling and
scenery queries enabled averaged 204.7 ms per `frame/main` CPU trace scope with
geometry queries off and 201.8 ms with them on (15 measured frames per run).
The requested window was 640×360 but the framebuffer was 1280×720. Both runs
reported missing or unsupported map assets. This small, noisy difference is not
evidence of a hardware performance improvement; no GPU timing was collected.

A separate short software comparison of the indirect broad phase and dispatch
reuse, with scenery and map-geometry queries enabled, recorded 2,660 indexed
model-batch submissions before and 1,098 after over 15 measured frames. Mean
`frame/main` CPU scope duration was 206.7 ms before and 208.4 ms after. The
640×360 window again had a 1280×720 framebuffer; the map had missing or
unsupported assets. The submission reduction is measurable in this run, but
the timing difference does not establish a performance improvement.

Scenery query eligibility respects the existing depth-test and depth-write
behavior; objects with `noZcheck` or `noZwrite` are not query culled. Vehicles
are not query culled.
Because conditional rendering may serialize GPU work and each candidate
adds a proxy draw/query, this is a correctness/performance prototype, not an
assumed optimization. Compare frame time and triangle/shader workload with the
setting off and on before using it broadly. The existing CPU triangle counter
still reports candidate geometry submitted by the renderer, not geometry
suppressed by conditional rendering; use GPU timing/profiling to measure the
actual workload. The normal renderer remains unchanged unless explicitly enabled.
Proxies that intersect the near plane and animated crashable poles are rendered
without query culling. When combined with compute-indirect, the GPU-generated
visibility also gates the query proxy, although CPU query setup is still required.
Depth and color state used for queries is shared within each opaque render-type
pass rather than queried from the driver for every candidate.
If proxy-buffer upload fails, the candidate is rendered normally and the upload
is retried later; the map releases its proxy buffer when it is destroyed.

Centers are transformed into eye space in double precision before packing the
SSBO, preventing large map origins from erasing local detail. Normalized frustum
planes use a conservative 1 mm margin. LOD decisions use float precision on GPU,
so transitions can differ from double-precision CPU fallback at exact thresholds.

Indirect mode consumes GPU commands after a command barrier, without
downloading a mask. Main views, captures, and reflections never consume previous-frame
visibility. The prior GL program and buffer bindings are restored after compute
to preserve the renderer's cached state. Resources are released before context
destruction.

The shared LOD-boundary buffer is uploaded again only when its contents change;
unchanged successive views reuse the GPU storage. Changed contents replace the
storage so earlier queued dispatches retain their own boundaries.
Nested indirect scopes also reuse a snapshot of shader-storage bindings across
dispatches, while restoring the caller's bindings after each dispatch. Direct
shader-storage or indirect-target binding changes inside such a scope are not
supported; renderer-owned buffer deletion invalidates the snapshot.

## Performance limitation

The retired readback backend required synchronous GPU-to-CPU visibility readback.
The indirect backend removes that dependency, but still prepares materials and
submits individual indirect draws for all candidates, including GPU-rejected
candidates. Indirect did not beat CPU culling in the previously measured
scene; CPU remains the default. Capability alone is not a performance criterion.
Diagnostic probe readback is not used by application rendering.

### Short software-rendering check

On `DBC_Map` at 640×360 with queries enabled, 3 warmup frames and 15 measured
frames, a single Mesa software-rendered run before the scenery broad phases
averaged 357.6 ms per `frame/main` CPU trace scope and submitted 116,472 model
batch scopes. A subsequent run with the frustum and LOD broad phases averaged
171.5 ms and submitted 46,588 model batch scopes. Both runs completed; the map
reported missing/unsupported assets in both. These short, workload-sensitive
CPU trace measurements are not GPU timings or a representative London/Radeon
comparison. They do not establish a hardware performance improvement.

## Historical: what was the CPU waiting for?

**Culling does not inherently require a CPU wait. The legacy readback implementation
requires one because the CPU is the consumer of the GPU's answer.** It moves the
tests to compute but leaves the decision to issue each draw on the CPU. A GPU-driven
renderer would let the GPU consume its own answer instead.

### Legacy readback dependency chain

1. **CPU prepares inputs.** `makeCullingItem()` transforms sphere centers into eye
  space in double precision, then packs floats. Callers prepare flags, XY bounds,
  LOD boundaries, and the current view's six frustum planes. Animated vehicle
  transforms are evaluated on CPU. This work has not moved to compute.
2. **CPU submits uploads and compute.** `computeCull()` uses `pglBufferData()` for
  inputs, output storage, and LOD boundaries, then queues `pglDispatchCompute()`.
  Submission does not mean the GPU has already executed the shader.
3. **GPU produces a mask.** Each invocation reads one candidate and writes one
  32-bit `visible[index]` value. Depending on flags, it tests the sphere against
  six planes, checks XY bounds against a 10 km radius, and walks LOD boundaries
  to select a level. The LOD search is linear, not binary. It does not render
  geometry, read a depth buffer, test occlusion, or issue draw commands.
4. **GPU writes are made visible to the next consumer.**
  `pglMemoryBarrier(GL_BUFFER_UPDATE_BARRIER_BIT)` establishes the required
  shader-write visibility for the subsequent buffer readback. It is a GPU
  memory dependency, **not a request to copy results to CPU or a CPU completion
  wait like `glFinish()`**.
5. **CPU requests the actual bytes now.** `pglGetBufferSubData()` copies the output
  buffer into the CPU `visibility` vector. It cannot return usable bytes before
  the relevant GPU writes are complete and accessible. This is the explicit
  blocking point, measured by `ComputeCulling::readback.wait`.
6. **CPU branches on those bytes.** Callers skip rejected objects, construct draw
  lists, resolve materials/textures, sort batches, and issue ordinary draws.
  Only after this can the GPU render those submitted draws.

The cycle is **CPU submits culling → GPU computes → CPU reads and submits draws
→ GPU renders**. Repeating it for multiple views breaks the normal overlap in
which the CPU prepares work while the GPU renders previously submitted work.

### Why can a tiny mask take so long?

The cost is not just transferring four bytes per candidate. The GPU may still be
processing previously submitted work when the CPU asks for the result. The wait
can include scheduling/queue backlog, dependencies on earlier commands, input
availability, compute execution, and driver/readback transfer overhead.
Previously queued reflection or scene rendering can therefore contribute to the
observed readback latency. It is **not** necessarily a wait for every GPU task on
the machine, nor proof that the culling arithmetic itself took that long.

Uploads, allocation/reuse, and GL state queries can also cost CPU time or cause
driver synchronization; the explicit readback is not the only possible cost.
`ComputeCulling::dispatchAndReadback` includes setup and readback, while
`ComputeCulling::readback.wait` times the CPU duration of that readback call.
Neither is a GPU timestamp measurement of the compute shader. The earlier
`MapRenderer::draw.computeCulling` scope also includes CPU input construction and
pointer-to-index hash-table work. Do not label those totals "GPU culling time".

### Where the returned mask was consumed

| Path | What the former readback implementation did after receiving the result | Is that CPU dependency fundamental? |
| --- | --- | --- |
| `MapRenderer::draw()` | `gpuVisible()` indexes the mask for terrain, water, roads, and scenery; rejected objects are skipped before per-object submission. GPU LOD rejection replaces CPU LOD selection. | No. GPU-produced draw commands can encode these decisions. |
| `Vehicle::computePartVisibility(true)` → `Vehicle::draw()` | The `visible` lambda filters spatial/LOD visibility; CPU builds opaque and transparent lists and submits batches. Prepared lists may be reused for the transparent pass. | No. CPU can prepare candidates and material state without reading spatial visibility; GPU can suppress rejected commands. |
| `Vehicle::prepareFrameVisibility()` (previous design) | Reflection-bearing parts previously used a downloaded mask before CPU corner projection and reflection-target size calculation. | Removed: reflection scheduling now tests just reflective surfaces on CPU, with no compute dispatch. |
| `CoreRenderer` draw functions | Material bindings and uniforms are prepared on CPU; `glDrawArrays()` / `glDrawElements()` use CPU-supplied counts. | The present APIs need redesign to consume GPU commands, but CPU material binding itself does not inherently require reading GPU visibility. |

Script-variable/viewpoint eligibility, active tiles, and resource residency are
CPU-known inputs, not values the culling shader discovers. Likewise, sorting
keys and material selections can be calculated for a conservative candidate set
before knowing its final spatial mask. The indirect path does that work before
the GPU rejects candidates, which is an important remaining performance cost.

## Moving more work to GPU without waiting

The target is **GPU culling → GPU draw commands → GPU rendering**, with the CPU
queuing all three in order and never downloading a per-object visibility mask.
GPU work still has dependencies, but they stay on the GPU instead of making the
render thread wait for the answer.

### Implemented first stage: fixed-slot GPU commands

`IndirectCullingBatch` owns a reusable output buffer. Each input
`CullingItem` is 64 bytes (eye sphere, XY bounds, flags/LOD, geometry count).
Compute writes five words per candidate, plus a second five-word command when
query occlusion is enabled for that map view. The model command contains count,
instanceCount, firstIndex, baseVertex, baseInstance. The last three are zero;
the first four words also form a valid arrays-indirect command with zero first
and baseInstance. Rejected candidates have instanceCount zero. Counts/ranges are
CPU-known geometry metadata, not downloaded GPU results. The second command
draws the 36-vertex query proxy with the same instance count, so spatially
rejected indirect candidates do not rasterize that proxy. Compute stops further
visibility checks for a candidate once an earlier check rejects it.

`GL_COMMAND_BARRIER_BIT` makes these writes visible to indirect command fetch.
`CoreRenderer` model, indexed-model, material, and environment passes use
`drawSelectedIndirect()` to consume the selected slot. No GPU visibility or
draw-count download, fence wait, or previous-frame mask is involved. The CPU
still submits a fixed candidate list; this is not command compaction or MDI.

`IndirectCullingScope` restores selected-command state and the prior GL indirect
buffer binding. Batch destructors run with the rendering context current;
OpenGL defers storage release until queued commands finish. Each nonempty
dispatch replaces the buffer's output storage, so reflection/main views cannot
overwrite previously queued commands. Map draws reuse the buffer object, while
vehicles retain one for their matching opaque/transparent passes. The binding
is cached within each scope, avoiding repeated binds for
successive commands in the same batch; direct changes to the indirect-buffer
target are not permitted while a scope is active. The vehicle model phase shares
an outer scope, so nested vehicle draws reuse the tracked binding without
querying the driver for every pass. Re-dispatching a batch
reallocates its output store. Input/LOD uploads still use `glBufferData`;
allocation/driver costs are not eliminated.

Map terrain layers, sorted water, bounded roads, and scenery select their own
command slots through view-stamped records, avoiding a per-view pointer hash
table and preventing omitted candidates from reusing a previous view's slot.
Map culling inputs and LOD scratch vectors also retain capacity between views.
Map chunk broad-phase rejection remains CPU-side to avoid preparing materials
for entire off-screen groups. Conservative CPU frustum and LOD broad phases
also omit scenery candidates that cannot pass the GPU tests, retaining uncertain
and boundary candidates for the GPU's final decision. This avoids preparing
and submitting every candidate when the current individual-draw path cannot
compact them. Roads without bounds use ordinary draws. Vehicles emit one command per
eligible batch (parts can contain batches with different geometry counts).
Eligible batches carry their command slots directly rather than using a
per-view pointer hash table; ineligible slots are cleared on redispatch. Vehicle
input scratch capacity is reused on the render thread instead of being retained
separately by every vehicle. The same conservative frustum and LOD broad phases
omit clearly rejected vehicle parts before batch/material preparation; compute
still makes the final decision for the retained parts.
Script/viewpoint eligibility stays CPU-side. Vehicle opaque and transparent
passes reuse the same prepared commands and sorted candidate lists when view,
animation, material, frustum, and size settings match. Changed settings or
newly loaded geometry force a fresh dispatch. Transparent candidates are sorted
on CPU and submitted in that order; GPU rejection does not reorder them. The
same slot gates transmap depth and environment passes.

Initialization checks compute and both indirect draw entry points. Readback
entry points are **not required** for indirect mode. Allocation/validation or
dispatch failures use existing current-view CPU culling, never stale commands.
Triangle submission counters count candidate triangles, not post-culling GPU
triangles, in indirect mode. No shader API/material behavior was removed.

### Further batching roadmap (not implemented yet)

1. **Start with opaque static scenery.** Keep simulation, scripts, streaming,
  resource creation, and conservative active-tile eligibility on CPU. Store
  bounds, transforms, LOD ranges, and draw metadata on GPU. Cache static data
  rather than rebuilding/uploading every record for every view. Preserve
  camera-relative precision; world-space float conversion alone would regress
  the large-origin behavior we currently protect with double CPU transforms.
2. **Have compute write indirect commands instead of a CPU mask.** A fixed slot
  per candidate can contain the normal geometry range and an instance count of
  zero when rejected (one or more when visible). Use the correctly laid-out
  command format for arrays or indexed draws. Refresh every slot each view so
  stale nonzero counts cannot leak into a different camera/reflection.
3. **Consume commands on GPU.** After compute, issue
  `glMemoryBarrier(GL_COMMAND_BARRIER_BIT)` before indirect draw consumption.
  Add `GL_SHADER_STORAGE_BARRIER_BIT` if draw shaders also read compute-written
  SSBO data; other consumers require their corresponding barrier bits. Use
  `glDrawArraysIndirect()` / `glDrawElementsIndirect()` initially. The CPU can
  bind each known batch's shader/textures and issue its indirect draw even when
  that GPU command will draw zero instances. **No count readback is needed.**
4. **Then batch to reduce CPU submission.** Group compatible geometry/materials
  for instancing or multi-draw indirect, with shader-readable transform/material
  tables. Multi-draw groups share GL state; unrelated textures/programs cannot
  simply be packed into one call without a compatible representation such as
  texture arrays/atlases and indexed material data. A single indirect call per
  candidate removes readback but does not remove per-candidate CPU submission.
5. **Compact only when useful.** Fixed-count indirect arrays with zero-instance
  rejected commands avoid needing the CPU to know a GPU-generated draw count.
  Compact visible commands/instances in compute if profiling justifies it.
  GPU-sourced variable multi-draw counts need supported indirect-count
  functionality (OpenGL 4.6 or `ARB_indirect_parameters`), not assumed on the
  baseline 4.3 compute context. Do not reintroduce a synchronous count download.
6. **Extend after validating static geometry.** GPU transforms/animation and LOD
  generation can further reduce CPU packing, but must retain authored animation
  parents, wheel poses, script state, and valid conservative animated bounds.
  Later, optional depth-pyramid/Hi-Z occlusion can reject hidden objects; it is
  a separate feature with depth-production and conservative visibility rules.

The fixed-slot command generation and individual indirect consumption in steps
2–3 are implemented. GPU-resident static scene tables, instancing/multi-draw
grouping, compaction, GPU transforms, and Hi-Z remain future work. Do not assume
a performance gain until measured.

### Work that needs special handling

- **Transparency:** arbitrary command compaction destroys back-to-front order.
  Initially retain CPU depth sorting (and, if useful, CPU culling) for transparent
  batches. Alternatively sort all eligible candidates on CPU and let compute
  zero rejected commands without changing slot order. GPU sorting or a different
  transparency algorithm is a later project. Preserve text, cutout, transmap
  depth-prepass, no-Z-write/check, and environment-pass behavior.
- **Reflection scheduling:** keep cheap current-view CPU tests/projection for
  the small set of reflective surfaces that determines target allocation and
  updates. This does not prevent GPU culling of the much larger scene rendered
  into each reflection. Moving target scheduling to GPU is a separate design;
  rendering all possible targets is a conservative but potentially costly option.
- **Multiple views and passes:** each main/reflection/capture view needs commands
  matching its own camera and current transforms/LOD. Reuse requires matching
  inputs. Commands may be regenerated in queue order, or separate buffers may be
  used; CPU uploads must not overwrite storage still in use by the GPU.
- **Picking, physics, and streaming:** raster visibility is not automatically the
  right eligibility test for interaction/collision/residency. Keep these independent
  unless there is a specific, validated reason to couple them.

### Why not just add a fence or delay readback?

A fence can tell the CPU whether previously submitted work is finished. Waiting
on it immediately merely moves the stall. Polling later can avoid that immediate
wait, but a past frame's visibility mask is not the current frame's answer:
camera turns, animations, LOD transitions, and reflections can expose previously
rejected objects. Blindly reusing an old mask can hide visible geometry.

Deferred readback is useful for noncritical statistics and diagnostics. It is
not the proposed correctness-preserving replacement for current-view culling.
Persistent mapping likewise does not make unfinished GPU writes safe for CPU
consumption. GPU-side indirect consumption removes the need for that CPU answer
altogether, rather than reading it a few frames late.

### Acceptance criteria for a GPU-driven backend

- No synchronous visibility **or draw-count** readbacks in the normal frame path.
- Current-view results, including rapid camera changes, reflections, animated
  parts, large map origins, and exact LOD/frustum boundaries.
- Existing material, transparency, shadow, text, and interaction behavior retained.
- Separate CPU submission and GPU timestamp measurements, with same-settings
  frame-time comparisons across small vehicle lists and large scenery scenes.
- No promised FPS target based solely on dispatch timing or hardware support.

## Validation

### Indirect backend validation (2026-10-18)

All-target Release build and nine focused CTests pass. The GPU probe now checks
arrays and indexed raster output for visible/hidden/explicitly rejected/zero-count
candidates, successive view changes, nested selection scopes, invalid slots/LOD,
missing indirect functions, and command parity up to 4,097 candidates. Real draw
tests run with `pglGetBufferSubData == nullptr`. Only the parity test explicitly
downloads commands, after a buffer-update barrier; the renderer does not.

Same 1,200-frame benchmark settings as below, final revision with CPU broad phase:

| Backend | Mean `main` duration | Visibility readbacks |
| --- | ---: | ---: |
| `compute-indirect` | 7.305 ms | 0 |
| `cpu` | 3.504 ms | 0 |

Both runs completed cleanly. Indirect issued 6,624 dispatches averaging 39.03 us
of CPU submission time (not GPU shader time). Artifacts:
`build-ode/indirect-final-{compute-indirect,cpu}`. An earlier unfiltered indirect
revision measured 8.849 ms versus 3.692 ms CPU; restoring map chunk broad phase
and filtering CPU-ineligible vehicle candidates reduced, but did not solve,
the candidate-submission overhead. Legacy readback in that earlier comparison
measured 6.031 ms and 4,808 waits (reflection scheduling is now CPU-only).

**This implements the no-readback dependency replacement, not a performance
win or a fully batched GPU renderer.** CPU remains default deliberately. Next
work should target GPU-resident transform/material tables and compatible
instanced/multi-draw groups so rejected candidates do not incur individual
CPU material/submission work. These tests do not constitute exhaustive visual
parity for every authored asset or GPU timestamp profiling.

### Regression comparison (2026-10-18)

Release tracing build, Cotterell spawn 0, MAN DL05, 1280x720, VSync off,
native scripts at 60 Hz, material batching enabled, 512-pixel reflections every
four frames without reflection transparency. Each run used 60 warmup frames,
then 1,200 measured frames across five benchmark phases repeated twice.
Settings and binary were identical between backends; order was reversed for the
second pair. Times below are mean `main` frame scope durations, not GPU timings.

| Run order | Synchronous compute | CPU/default | CPU frame-time reduction |
| --- | ---: | ---: | ---: |
| Compute, then CPU | 6.112 ms | 3.394 ms | 44.5% |
| Default (unset), then compute | 5.429 ms | 3.390 ms | 37.6% |

Compute made 6,008 measured readbacks per run; CPU/default made zero. In the first
pair, vehicle visibility preparation averaged 314.19 microseconds with compute
versus 9.61 microseconds on CPU. Scope times are nested and must not be summed
as independent frame costs. All four runs completed and exited cleanly.

Artifacts are under `build-ode/culling-regression-*` and
`build-ode/culling-regression-repeat-*`; the user's root trace was not replaced.
These scripted mixed-view results do not establish recovery to 750 FPS in the
user's original view. The runs also exposed an existing texture-upload GL error:
compute clears/logs it before dispatch, whereas CPU logged a failed campus-logo
upload. That separate rendering issue is not resolved by backend selection.

`OpenBusComputeCullingProbe` tests CPU reference boundaries and uninitialized
fallback, plus the default-off/explicit-opt-in policy without needing a GL context.
`OpenBusComputeCullingGpuProbe` explicitly enables compute in a hidden OpenGL 4.3
context and compares GPU/reference masks, partial workgroups, camera changes, perspective
planes, state restoration, missing entry points, and lifecycle. The GPU probe is
marked skipped if GLFW cannot create a suitable context.