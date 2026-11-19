# Graphics settings

OpenBus reads `OpenBus_graphics.ini` beside the game executable. The file is optional: if it
does not exist, the renderer uses its existing defaults. Graphics settings are available from
both the startup menu and the in-game pause menu. The Graphics page can edit and save the
settings listed below. Saving uses an atomic replacement. If the file cannot
be read, or contains an invalid known value, OpenBus reports the error and disables saving
through the menu so the existing file is not overwritten.

The production application uses Diligent Vulkan with a GLFW no-API window.
OpenGL-specific renderer descriptions below refer to legacy code or standalone
probes, not the production presentation path. The production executable does not
link or import the OpenGL system library; legacy renderer tests still require it
until their coverage is ported.

```ini
[display]
mode = windowed
width = 1920
height = 1080
refresh_rate = 60
monitor = 0
vsync = true
fps_cap = 0

[reflections]
size = 1024
interval = 1
max_fps = 0

[visibility]
main_minimum_object_percent = 0
mirrors_minimum_object_percent = 0

[textures]
anisotropic_filtering = 1
generate_mipmaps = false
map_texture_scale = 1.0
vehicle_texture_scale = 1.0

[lighting]
scenery_light_limit = 0

[antialiasing]
mode = off

[rendering]
scale = 1.0

[advanced]
vehicle_frustum_culling = true
```

| Setting | Meaning | Application |
| --- | --- | --- |
| `display.mode` | `windowed`, `borderless`, or `exclusive` fullscreen | Live preview, 15-second confirmation |
| `display.width` | Window/exclusive mode width, `640`â€“`16384` pixels | Live preview, 15-second confirmation |
| `display.height` | Window/exclusive mode height, `640`â€“`16384` pixels | Live preview, 15-second confirmation |
| `display.refresh_rate` | Exclusive fullscreen refresh rate, `24`â€“`1000` Hz | Live preview, 15-second confirmation |
| `display.monitor` | Zero-based monitor index | Live preview, 15-second confirmation |
| `display.vsync` | `true` or `false` | Live |
| `display.fps_cap` | Frames per second; `0` disables the cap, otherwise `0.01`–`1000` | Live |
| `reflections.size` | Maximum mirror target size: `256`, `512`, or `1024` | Restart |
| `reflections.interval` | Render reflections every N frames, from `1` to `10000` | Restart |
| `reflections.max_fps` | Reflection update rate; `0` disables the cap, otherwise `1`–`1000` | Restart |
| `visibility.main_minimum_object_percent` | Minimum projected object size in the main view, `0`–`100` percent | Live |
| `visibility.mirrors_minimum_object_percent` | Minimum projected object size in mirror views, `0`–`100` percent | Live |
| `textures.anisotropic_filtering` | `1` (off), `2`, `4`, `8`, or `16` | Restart |
| `textures.generate_mipmaps` | Generate mipmaps for decoded static vehicle and map textures | Restart |
| `textures.map_texture_scale` | Map texture resolution factor, `0.25`–`1.0` | Restart |
| `textures.vehicle_texture_scale` | Vehicle texture resolution factor, `0.25`–`1.0` | Restart |
| `lighting.scenery_light_limit` | Maximum scenery lights affecting active map tiles; `0` is unlimited, otherwise `1`–`16384` nearest lights | Live after saving |
| `antialiasing.mode` | `off`, `fxaa`, `msaa` or `smaa` | Live after saving |
| `rendering.scale` | World render resolution scale, `0.5`–`1.0` | Live after saving |
| `advanced.vehicle_frustum_culling` | Reject vehicle parts outside the camera frustum | Restart |

Existing environment variables override the corresponding INI setting:
`OPENBUS_VSYNC`, `OPENBUS_FPS_CAP`, `OPENBUS_REFLECTION_SIZE`,
`OPENBUS_REFLECTION_INTERVAL`, `OPENBUS_REFLECTION_MAX_FPS`,
`OPENBUS_MIN_OBJECT_SIZE_MAIN_VIEW_PERCENT`, and
`OPENBUS_MIN_OBJECT_SIZE_MIRRORS_PERCENT`, `OPENBUS_TEXTURE_SCALE`,
`OPENBUS_FRUSTUM_CULLING`, and `OPENBUS_MAP_LIGHT_LIMIT`. An overridden control is disabled and
identifies the variable in the menu; its INI value remains unchanged. Remove the environment
variable and restart to use the INI value. For compatibility, `OPENBUS_MAP_LIGHT_LIMIT=0`
disables all scenery lights; in the INI, `scenery_light_limit = 0` means unlimited.

Display controls are marked **Preview** in both menus. Preview changes the mode immediately
and opens a 15-second confirmation dialog; if not confirmed, the previous display mode and window
geometry are restored. Confirm the preview before saving. Borderless fullscreen uses the selected
monitor's current desktop resolution. Exclusive fullscreen offers modes reported by GLFW for the
selected monitor and refuses unsupported resolution/refresh combinations. On startup, an
unsupported exclusive mode saved outside the menu logs a warning and falls back to the monitor's
current mode. If the configured monitor index is unavailable at startup, the primary monitor is
used.

The quality preset selector in either menu applies a starting point to implemented world render,
reflection, visibility, texture-resolution, and scenery-light controls:

| Preset | World render scale | Map texture scale | Vehicle texture scale | Scenery-light cap | Reflection target | Update interval | Reflection cap | Main/mirror culling threshold |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Low | 50% | 50% | 50% | 128 | 256 | 3 frames | 30 FPS | 0.25% / 0.5% |
| Medium | 75% | 75% | 75% | 512 | 512 | 2 frames | 45 FPS | 0.1% / 0.2% |
| High | 100% | 100% | 100% | Unlimited | 1024 | Every frame | Unlimited | Disabled |

Presets do not change display settings, VSync, the main frame-rate cap, anti-aliasing, anisotropic
filtering, mipmap generation, or advanced vehicle frustum culling. They skip values controlled by
environment-variable overrides. Any individual edits that no longer match a preset are shown as
**Custom**. Reflection, anisotropic filtering, mipmap generation, vehicle
texture resolution, and advanced vehicle culling changes require restarting OpenBus.

The scenery-light limit applies immediately after saving and controls the maximum number of
scenery lights affecting the currently active map-tile window in each view. Capped views prioritize
lights nearest to that view; distant lights may disappear, so this is a deliberate
lighting-quality/performance trade-off, not a lossless optimization. It is especially useful in
night scenes with thousands of scenery lights. The default and High preset remain unlimited to
preserve existing lighting. Low and Medium use caps of 128 and 512, respectively. It does not cap
vehicle spotlights or perform per-light view-frustum culling. An `OPENBUS_MAP_LIGHT_LIMIT`
environment override takes precedence.

Anisotropic filtering applies to static vehicle and map material textures. When enabled on a
device that supports the OpenGL anisotropy extension, mipmaps are generated for eligible decoded
static textures and the requested level is clamped to the device maximum. The separate
`textures.generate_mipmaps` setting enables mip generation without anisotropic filtering; active
anisotropic filtering enables mipmaps automatically. Precompressed textures keep their authored
mip chains. Dynamic script textures, render targets and UI textures are excluded. Generated mip
chains can add roughly one third to the base storage of eligible uncompressed textures. The menu
reports unsupported devices rather than pretending anisotropic filtering is active.

Vehicle texture scaling uses area averaging when downsampling decoded vehicle images, reducing
aliasing compared with selecting a single source texel. For precompressed DDS/GLI vehicle
textures, it selects the closest authored mip level when a mip chain is available; it does not
decode compressed textures or generate missing mips. Precompressed textures without a mip chain
remain at their authored dimensions, so this setting does not guarantee a uniform texture-memory
reduction across all vehicle assets. It takes effect after a restart because existing vehicle
assets are already loaded.

Map texture scaling downsamples decoded map images with area averaging before GPU upload.
`textures.map_texture_scale` affects map textures only; unlike vehicle DDS/GLI paths, map textures
are decoded before upload. It reduces GPU texture storage but not the CPU-side decoded-image cache,
and may increase map load time due to the extra resampling work. It also requires a restart.

The renderer environment-variable inventory also contains developer and experimental controls,
including occlusion-query experiments, benchmark/capture settings, and verbose asset diagnostics.
Those are not promoted to player settings until their behavior and performance impact are
validated. Vehicle frustum culling is exposed separately under **Advanced rendering** as a
troubleshooting option, with a warning that disabling it can increase draw work.

Anti-aliasing is applied to world color before the native-resolution interface is drawn. The
motion-vector, validity, and depth textures are not filtered by FXAA or SMAA.

- **FXAA** is a single fullscreen color-filter pass. It is the lightest anti-aliasing choice but can
  soften fine detail.
- **MSAA** renders the world into 4-sample color, motion, validity, and depth targets. Color is
  averaged across samples; motion, validity, and depth are selected together from the nearest
  covered sample so values from different surfaces are not mixed. If 4x targets are unavailable,
  the renderer tries 2x and then falls back to FXAA. Multisampling increases render-target memory
  and geometry fragment work.
- **SMAA** uses the upstream SMAA 1x high-quality preset: color-edge detection, blend-weight
  calculation, and neighborhood blending. Its area/search lookup tables and MIT license are in
  [`third_party/smaa`](../third_party/smaa/README.md). It adds two intermediate targets and three
  fullscreen passes.

The GPU probe verifies that FXAA and SMAA change the output around a synthetic triangle edge and
that MSAA preserves the nearest surface's motion and depth during resolve. Its timings remain
synthetic and are not representative of a full game scene.

World render scaling allocates the main-view color, motion, validity, and depth targets at the
selected fraction of the display dimensions, then linearly upscales color before the native-
resolution interface is drawn. It applies to the main world view; reflection views remain at their
existing resolutions. Motion and depth resources remain at render resolution, and a scale or
display-size change invalidates temporal history. The `rendering.scale` value is clamped by
validation to `0.5`–`1.0`; `1.0` is native resolution. At 50%, the MRT target pixel count and
associated target memory are approximately one quarter of native, excluding the full-resolution
upscale destination. One synthetic OpenGL probe run at 1280x720 on an NVIDIA GeForce RTX 3080 Ti
reported 0.0587 ms/frame GPU time at native resolution and 0.0362 ms/frame at 50% scale; the
estimated MRT storage changed from 11.43 MiB to 2.86 MiB. The probe uses one triangle, so these
timings do not predict full-scene performance; real workloads and other GPUs still need testing.

An initial synthetic probe on Windows with an NVIDIA GeForce RTX 3080 Ti (OpenGL 3.3), at
1280x720 with a single triangle and 120 measured frames per run, reported median GPU time of
0.0627 ms/frame for the motion-data color-blit path and 0.0594 ms/frame for FXAA. The apparent
-0.0033 ms/frame difference is within run-to-run variance and is not evidence that FXAA is faster;
this probe is only a renderer-path smoke test, not a representative game-scene benchmark. Full
scene GPU-time and visual-quality measurements remain necessary.

Comments and unknown sections or keys are retained when saving. Duplicate known keys and
invalid known values are errors. Sections and keys are case-sensitive. Values should not have
inline comments.

HDR output is not implemented, and no HDR toggle is presented. The production Diligent Vulkan
presentation path has no HDR tone-mapping/transfer stage, HDR monitor-capability query, or swap-chain
color-space selection. A higher-precision render target alone would not make the displayed output
HDR. A real option requires an HDR-capable presentation path and end-to-end validation with Windows
HDR and an HDR display; until then the setting remains intentionally absent.

SSAA, DLAA, DLSS, temporal FSR, and XeSS are not presented as available options. SSAA requires
rendering above native resolution and resampling temporal metadata, which would add significant
GPU cost. The production renderer uses Diligent Vulkan, but no vendor SDK integration for DLSS/DLAA,
temporal FSR, or XeSS is wired into the game. An opt-in Vulkan swapchain probe
now validates Vulkan device creation and presentation, but it does not render the game scene or
integrate any upscaler SDK. A separate opt-in Diligent Engine 2.5.6 Vulkan interop probe renders a
a synthetic triangle into an offscreen target and submits a CPU-prepared map-terrain mesh through
`DiligentMapTileAsset` and `DiligentSceneFrame`. It coordinates temporal history, SMAA/FXAA
composition, Diligent ImGui, and swap-chain presentation across consecutive and resized frames. It
also checks native Vulkan device, queue, command-buffer, and image access plus a tracked image-layout
handoff. It uses Diligent's pinned Vulkan-Headers and Volk submodules, so a Vulkan SDK installation
is not required on Windows; a working Vulkan runtime/driver is still required. Both probes are
diagnostics only: neither renders the game scene or integrates an upscaler SDK. These modes remain
unavailable until the renderer and SDK integrations are ported and validated. To run the original
Vulkan probe, configure with
`-DOPENBUS_BUILD_VULKAN_PROBE=ON`, build `OpenBusVulkanBackendProbe`, and run its CTest. To run the
Diligent POC, configure with `-DOPENBUS_BUILD_DILIGENT_VULKAN_PROBE=ON`, build
`OpenBusDiligentVulkanProbe`, and run its CTest. The normal OpenBus executable uses the Diligent
Vulkan renderer.

### Diligent renderer migration

The production `OpenBus` target requires `-DOPENBUS_BUILD_DILIGENT_RENDERER=ON` and uses the
Diligent Vulkan frame host unconditionally; there is no OpenGL runtime fallback or renderer
selector. The `OpenBusDiligentFrameRendererProbe` is a focused lifecycle probe, while the
asset-heavy Vulkan rendering probe remains gated by `OPENBUS_BUILD_DILIGENT_VULKAN_PROBE`.
`DiligentTexture2D` can opt decoded RGBA8 uploads into CPU-generated, area-averaged mip chains;
`DiligentVehicleAsset`, `DiligentSceneryAsset`, and `DiligentMapTileAsset` forward this option for
their decoded static textures. It defaults off and leaves compressed authored mip chains unchanged.
Diligent map tile assets expose a checked tile-origin transform and compose it
with the view-projection matrix so tile-local terrain, water, and road meshes can be placed on the
production map grid by the scene submitter.
Vehicle animation-origin and transform calculations now live in the backend-neutral matrix module
and the existing vehicle renderer uses the shared ordered animation-transform builder.
`DiligentVehicleAsset` can submit per-animation-group transforms and can build previous-frame motion
parameters from `MotionHistory` using stable object/group keys; its animation/history factory uses
the same transform composition order. A hierarchy helper resolves parent groups, rejects cycles,
and propagates geometry-history invalidity to descendants. The production vehicle path submits
Diligent draws; `prepareDiligentVehicleGroupTransforms` adapts each loaded production vehicle's
evaluated display parts into those groups without reimplementing its parent-animation rules.
The opt-in Vulkan frame component, `openbus_diligent_vulkan`, owns a Diligent Vulkan device,
immediate context, and swap chain for a caller-owned native window. It exposes the current
back-buffer view, waits for queued work before nonzero resizes, and presents after finishing the
frame. A zero-sized resize suspends presentation and makes the back-buffer accessor unavailable;
the first nonzero resize resumes the frame. Callers still skip scene rendering while suspended.
The Vulkan probe checks resize, suspension, restoration, and present behavior. A backend-neutral
`RendererFramePresenter` interface now covers framebuffer resize/suspension, VSync, frame
presentation, and waiting for idle work. The production loop reports framebuffer-size changes
through this interface and uses `DiligentVulkanFrame` for startup-menu and normal frame
presentation. No shared-context bridge or mixed-backend runtime is used. The standalone Vulkan
probe remains opt-in; the production renderer fetches its pinned DiligentCore dependency as a
required component.

The camera/object history implementation is isolated in the graphics-API-independent
`openbus_motion_data` library. Both OpenGL and Diligent can link the same `MotionHistory`
implementation without making Diligent depend on the OpenGL renderer. Rendered frame IDs must
increase strictly; skipped IDs invalidate temporal history, while duplicate or backward IDs are
rejected.

`DiligentMotionRenderTargets` owns scene-color RGBA8, RG32F motion vectors, R8 validity, D32 depth,
and optional R32F linear depth, each with shader-resource access. It defaults to single-sample
attachments and can instead allocate a supported multisample count. For MSAA, `resolve` averages
color and selects motion, validity, depth, and linear depth together from the nearest-depth sample;
shader-resource accessors expose the resolved single-sample views. Call resolve after scene draws
and reacquire borrowed views after resize. `clear` binds the draw targets and clears motion,
validity, and linear depth to zero; `resize` waits for the immediate context before atomically
replacing attachments. The caller still handles frame-history invalidation, camera cuts, pass
ordering, and skipping scene draws while the presenter is suspended.

`DiligentSceneFrame` coordinates these targets with the textured-mesh renderer and color
composition. It binds and clears scene attachments, exposes the current `MotionDataFrame` metadata
to scene submission, resolves and composites to a caller-provided destination, and enforces the
begin/compose/complete lifecycle. The caller remains responsible for presenting the destination,
drawing UI, submitting scene objects, and updating `MotionHistory`. The coordinator invalidates
history on the first frame and after target resize, and rejects non-increasing frame IDs or
backward timestamps; create one instance per independently rendered view. `abortFrame` releases an
incomplete scene lifecycle after submission failure and invalidates history before the next frame.

`DiligentFrameRenderer` is the Diligent-only application frame host. It owns the Vulkan presenter,
scene targets, and Vulkan-backed ImGui layer; coordinates resize/suspension, scene composition,
UI submission, and presentation; and rejects non-`GLFW_NO_API` windows. Its
`frameData` exposes the active frame metadata, and `invalidateHistory` lets callers reset temporal
data after camera discontinuities. Suspending at a zero-sized drawable and restoring it invalidates
history even when the previous dimensions return unchanged. The
`OpenBusDiligentFrameRendererProbe` validates this lifecycle. The host owns the Diligent scenery
asset cache and exposes asset and seasonal-texture request/lookup APIs plus render-thread
publication; publication is rejected during scene submission. It also owns a file-backed
`DiligentMapTileSet`, with map-window request and render-thread publication APIs that reject changes
during scene submission, guarded inactive-tile eviction, and read-only access to tile draw data
inside a frame. Callers may pass `MotionHistory` directly to `beginFrame`; the host completes it only
after presentation succeeds and invalidates it on failure or suspension. Its
`generateTextureMipmaps` construction option is applied to file-backed map tile assets. The
production loop uses the host for frame lifecycle, scene submission, UI, and synchronized
map/scenery resources.
The host forwards guarded terrain, water, and road pass draws to the active tile set; callers still
provide pass order, view/projection, fallback textures, and live material/temporal state.
`MapRenderer::activeSceneryLights` exposes map lights filtered by the active tile window, and
`buildDiligentMapPointLights` converts and distance-limits them for a Diligent view. The production
frame host now offers a guarded `setMapLights` API that uploads those per-view lights to the
Diligent mesh renderer, and clears point/spot counts at each frame boundary.

`DiligentMapTileAsset` uploads the production CPU-prepared terrain, water, and road mesh layouts.
Terrain can contain an ordered set of base/detail/transmap material layers that share one uploaded
geometry buffer; layer alpha mode and independent UV scales/Y-flip are retained. The component also
retains tile coordinates, vertical bounds, road texture/weather/alpha metadata, preserves road batch
order, and shares uploads when batches reference the same decoded image. Callers still supply tile
transforms, live terrain/weather materials, texture streaming overrides, culling, pass ordering, and
motion history. The production `MapRenderer` supplies CPU-prepared data to the Diligent tile set;
OpenGL uploads are disabled on the production path. `DiligentMapTileSet` owns a keyed set
of these tile assets, selects a deterministic active window using the production map-tile
coordinate rules, and submits active terrain layers with the corresponding tile transforms. It
retains inactive assets as a cache unless `removeInactiveTiles` is called. File-backed cache entries
are keyed by their selected terrain, water, texture-sidecar, and texture-definition sources, so a
chrono-selected source change replaces the cached tile rather than keeping stale terrain.
`updateTerrainWindow` remains a synchronous convenience API. `requestTerrainWindow` instead
schedules CPU-side terrain, water, and layer preparation with at most four concurrent jobs;
`publishReadyTerrainTiles` publishes completed work, advances the queued window, and performs all
Diligent resource creation/uploads on the calling thread. Call it regularly while streaming. Worker
failures are rethrown at publication, and results outside the latest requested window are discarded
before GPU upload.
`DiligentFrameRenderer` owns this handoff and the production frame loop requests, publishes, and
draws the active tile window. Callers remain responsible for selecting the appropriate road pass. Road sources can
upload either decoded images or resolved texture paths. `drawWater` sorts active water
batches back-to-front and submits them with the tile transforms. `drawRoads` filters opaque,
cutout, or transparent road batches and orders transparent batches back-to-front using their
prepared world-space bounds.
Road strip batching and vertex generation use the backend-neutral
`openbus::map::prepareMapRoadRenderBatches` helper from `openbus_map`. Diligent file-backed tile
preparation can also load a tile's spline profiles and roads when the caller provides a finalized
map-wide `MapRoadConnectionIndex`; the bounded async path retains that index through CPU preparation,
then creates road GPU resources during render-thread publication. The production `MapRenderer` now
exposes its map definition, asset root, base ground-texture index, and finalized shared road index,
and `DiligentFrameRenderer` has an overload that accepts that source shape and defers requests until
the road index is ready. The production `RenderLoop` supplies this data through its synchronized
map-stream update.
Tiles can be added from CPU-prepared data or through its file-backed terrain entry point, which
prepares meshes from the production `.terrain` format and loads base/detail/transmap resources
directly through Diligent, including compressed texture formats. Chrono terrain overlays are
loaded from the tile reference's currently selected terrain and sidecar paths after production date-
based map chrono selection. When a map tile has a water sidecar, the file-backed
path also prepares its water mesh and loads the conventional map water, bump, and environment
textures when available. It accepts the map's projected metric grid configuration and keeps
tile-local terrain, water, and prepared road geometry on the same global tile-origin convention as
the production renderer.

`DiligentVehicleAsset` uploads prepared vehicle material batches and shares decoded or compressed
uploads for base textures, texture arrays, and auxiliary light/night/trans/detail/free/text/
environment/normal maps.
Owned auxiliary maps are bound when the batch or runtime material does not supply its own view.
Per-material runtime material/texture overrides are supported. The asset preserves opaque source
order and sorts blended batches back-to-front using their transformed batch centers. It does not own
vehicle animation or frame history; the production Diligent scene path supplies its live transforms
and temporal history.
`DiligentRuntimeTexture2D` provides revision-aware updateable RGBA storage for script/text texture
snapshots; a vehicle material override can bind it directly and derives nearest/linear sampling from
the snapshot's filtered flag. `DiligentScriptTexture` connects a fixed script or text slot to that
storage using `ScriptRuntime`'s change-aware snapshot API. Per-material animation/material selection
and object history still need to be connected to Diligent vehicle submissions.

The primitive component renders opaque, colored triangle lists using the same `PrimitiveVertex`
layout as OpenBus's OpenGL primitives. It accepts a column-major OpenBus model-view-projection
matrix, converts OpenGL's clip-depth range to the active Diligent device's range, and draws to
one caller-bound color target and optional depth target with matching formats and sample counts. Its
dynamic upload buffers are reused and grow as needed. The caller owns target binding, frame
submission and synchronization; a draw changes the context's pipeline and resource bindings.
The GPU probe reads back a depth-tested synthetic scene and checks transformed geometry, near/far
clipping, invalid inputs and Diligent errors. It also loads a repository O3D fixture using the
existing asset loader, projects its triangles in a test-only view, and verifies pixels were rendered.

The textured-mesh component shares the 13-float `ModelVertex` layout used by map and vehicle
geometry. It uploads either a non-indexed triangle list or vertices with a zero-based 32-bit
triangle index list to immutable GPU buffers, then draws with a 2D or 2D-array base texture multiplied
by vertex RGBA. `DiligentMaterialState` supports a vertical UV flip, offset, and scale in the
same order as OpenGL (`(uv + offset) * scale`), with the array layer left unchanged. Mesh creation
rejects empty or incomplete triangle lists, non-finite vertex data,
and indices outside the vertex array. `DiligentTexture2D` also uploads
the existing `TextureLoader::Image` RGBA data to an immutable, single-mip RGBA8 shader resource.
Its `load` factory uses the shared `TextureAssetLoader` to load decoded images and supported BC3
DDS assets directly, and uploads compatible GLI-backed BC1-BC7 DDS textures; unsupported
compressed formats and load failures are reported explicitly.
`DiligentTexture2DArray` uploads equally sized decoded images as immutable, single-mip RGBA8 array
layers and uploads supported GLI BC1-BC7 arrays with their original mip chains and sRGB formats.
BC1 arrays with alpha are expanded to RGBA8 to preserve transparency. The helper exposes the layer
count and supports one-layer arrays; cubemaps remain unsupported. Model vertices select zero-based
layers through their `layer` coordinate; coordinates must be integral and within the array view's
slice range. Decoded-image uploads validate exact byte counts, while compressed uploads validate
block rows, mip dimensions, and the existing texture-buffer limit. `DiligentTexture2D` also accepts
`TextureLoader::CompressedDds`, validating and uploading authored DXT5/BC3 block-compressed mip
levels without CPU decompression. GLI-backed 2D textures support BC1/DXT1, BC2/DXT3, BC3/DXT5,
BC4/ATI1N, BC5/ATI2N, BC6H HDR, and BC7 formats; sRGB variants are retained where available.
DXT1 textures with alpha are expanded to RGBA8 because Diligent's BC1 Vulkan format is RGB-only.
Compressed arrays and cubemaps are not supported.
The caller may use either owned texture helper or supply a native single-sample 2D or 2D-array
Diligent shader-resource view and a column-major OpenGL model-view-projection matrix, bind matching
matching targets, and manage submission and synchronization. The owned mesh, texture, context,
and targets must belong to the same renderer device; drawing changes context state.
The component defaults to linear filtering with repeat addressing, no face culling, and optional depth
testing. `DiligentMaterialState` maps OpenBus's alpha conventions: opaque, cutout (`alpha < 0.5`
discard), blended (source-alpha/one-minus-source-alpha), and near-opaque (`alpha < 0.99` discard).
Its base color multiplies sampled texture and vertex RGBA. Visibility fade is clamped to [0, 1] and
affects cutout decisions, matching the existing OpenGL shader; it does not change blended alpha.
Specular color and power add directional Blinn-Phong highlights. The per-channel emission color is
a [0, 1] self-illumination mask that restores the unlit base color; it is not an additive emissive
term. Point and spot lights currently contribute diffuse light only.
Each draw may optionally receive a single-sample 2D tangent-space normal map view. The shader
derives its tangent frame from screen-space derivatives of view-space position and the transformed
UV coordinates, so mesh vertices do not need a tangent attribute. The normal map must contain
linear tangent-space data (do not sample it through an sRGB view); `normalMapStrength` is limited to [0, 1] and scales
the sampled normal's X/Y components. A zero value keeps the geometric normal. The mapped normal
affects directional diffuse/specular and point/spot diffuse lighting. Derivative frames can be
unreliable for degenerate UVs or discontinuities across UV seams; authored tangent frames,
normal-map array selection, and production material loading are not implemented.
Optional per-draw lightmap and nightmap views extend the same material path. A lightmap blends
shaded output back toward the unlit base color according to its RGB and strength, so white restores
the base and black preserves shading. A nightmap overlays its non-black RGB content, multiplied by
the material and vertex colors; both strengths must be within [0, 1]. These are
non-owning single-sample 2D views, use the mesh's original UV coordinates, and must be sampled as
linear data. An optional transmap replaces base-texture alpha before cutout/blend handling and
supports its own UV scale and vertical flip. All auxiliary views are non-owning, single-sample 2D
resources that must remain valid through the draw. Detail maps multiply base color using their own
UV scale. Snow coverage blends the detailed surface toward the production snow color before
lighting. An optional view-space environment sphere map blends after lighting and nightmaps using
the geometric (not normal-mapped) normal and caller-supplied weather brightness. Snow coverage and
environment strength are clamped to [0, 1]; brightness must be finite and nonnegative. The map is a
non-owning single-sample 2D view. Production material loading remains unported.
Depth writing is selected per draw; it defaults on to match the current OpenGL model alpha path,
and can be disabled for blended materials that must not update depth. Depth testing can be
disabled or use less/less-equal comparisons; disabling it also disables depth writes, as in
OpenGL. Optional back-face rejection uses clockwise front faces; callers must disable it
for mirrored views or instances when matching the production renderer. The white-ambient material
option keeps the texture's ambient reflectance independent of the material tint and applies snow to
both. Blended draw ordering remains the caller's responsibility. Optional
`DiligentLightingParameters` applies view-space Lambert
lighting using the mesh normal, object-to-view transform, and caller-supplied inverse-transpose
normal transform. Directional lighting is disabled by default, preserving the unlit output.
`setPointLights` accepts view-space point lights once per rendered view and uploads them to a
dynamically sized structured GPU buffer; point-light contribution uses constant falloff inside the
radius, inverse-square falloff outside it, and a `0.12 + 0.88 * Lambert` diffuse/ambient factor
multiplied by light color and intensity. Callers remain responsible for selecting and culling lights
for each view. `setSpotLights` uploads view-space spotlights using beam-forward directions and
inner/outer cone half-angles; it applies squared linear cone interpolation, range attenuation, and
Lambert diffuse contribution. Spotlight shadows are not implemented. The GPU probe checks unlit parity, specular/emission behavior,
directional front/back lighting, white ambient and snow, depth comparisons, back-face rejection,
transformed normals, tangent-space normal-map response and strength,
detail-map scaling, snow and environment-map shading, lightmap/nightmap blending, transmap alpha
blending, colored point/spot lights,
buffer growth, cone rejection, and invalid inputs. Cutout polygon offset
and production-scene lighting integration
remain unsupported. The GPU probe checks decoded TGA and BC1-BC7 mip-chain uploads, array uploads,
array-layer selection, UV
orientation, texture switching, UV flip/offset/scale, depth and clipping, transforms, vertex and material color,
visibility fade, alpha cutout and blending, indexed and non-indexed draws, 32-bit indices and
invalid inputs. Terrain, water, and roads use shared CPU geometry preparation for the production
OpenGL renderer and direct Diligent uploads; vertices do not pass through OpenGL on the Diligent
path. The probe draws these prepared meshes and the O3D fixture with synthetic textures; it does not
render a complete map or the asset's real material. Readback is for testing only.

Scenery asset preparation is also available in the graphics-independent
`openbus_scenery_preparation` library. It retains parsed meshes, CPU vertices, decoded images,
resolved texture paths, seasonal alternatives, and authored batch metadata without OpenGL handles.
`DiligentSceneryAssetCache` queues that CPU preparation with at most four concurrent jobs and
publishes GPU assets through `publishReady` on the calling thread. It also asynchronously decodes
seasonal textures with a separate four-job limit and uploads them during publication. It shares the
prepared-image cache across workers, reuses assets by normalized config and OMSI-root paths, and
rethrows worker failures rather than creating a fallback asset. Its caller-driven `retain` API can
evict stale GPU assets and seasonal textures; it defines the exact allowed request set, drops
unrequested queued work, and discards stale running CPU preparations instead of uploading them.
The frame host rejects retention changes during scene submission. Callers must update the retained
set before requesting assets and must not keep raw asset pointers across retention changes.
Production map-window streaming updates the retained asset set before requesting resources.
The production loader retains CPU-prepared data for the Diligent path; legacy OpenGL upload code
remains in the shared module but is disabled by the production app. `DiligentSceneryAsset` uploads
prepared meshes, trees, and decoded base and auxiliary textures directly to owned Diligent
resources, sharing repeated mesh/material and texture uploads within the asset. It retains batch
metadata and exposes diagnostics for missing textures and failed mesh preparation; callers must
inspect those diagnostics and retain the prepared asset's other scene-wide configuration where
needed. Seasonal alternatives are retained as paths for the caller's streaming system rather than
uploaded eagerly. The production renderer and Diligent scenery batches share a CPU-only resolver
for scripted material selection, light and night strengths, UV offsets, and alpha scale. Its selected
item is non-owning and requires the source batch's material selection to remain alive. Texture
binding and dynamic texture refresh remain the caller's responsibility. `DiligentSceneryBatch`
can submit opaque and alpha-cutout meshes, including script-controlled auxiliary maps and an
optional caller-supplied seasonal texture, to the color or motion renderer. The caller supplies
view-dependent transforms, culling decisions, pass ordering, lighting, and motion history.
Separate transparent and transmap-depth-prepass calls support alpha blending, near-opaque cutout
blending, partial visibility fades, sorted-pass depth behavior, and color-masked depth-only writes.
Transparent draws still require caller-controlled back-to-front ordering. The textured-mesh shader
can multiply caller-provided free/script and text-atlas views over the base texture; text uses raw
mesh UVs and discards empty atlas pixels, while free/script maps use scaled UVs and can apply caller
weather brightness. Scenery batches require these runtime views when their metadata selects a
dynamic texture, but texture refresh, script selection, and view lifetime remain caller-owned.
Tree batches also expose direct cutout draws with caller-provided world/view transforms and seasonal
texture overrides. The Vulkan probe exercises a prepared SCO/X mesh, resource sharing, scripted map
selection, seasonal binding, dynamic free/text maps, transparent blending and fades, transmap
prepass depth/color behavior, motion validity, tree draws, missing assets, and resource lifetime.
Production scene submission uses Diligent; dynamic texture refresh and seasonal streaming still have
caller-owned lifecycle requirements, and lighting behavior is not yet a complete production match.

Diligent's textured-mesh shader applies the model UV flip and script offset before sampling
normal, light, night, detail and transparency maps. Base, detail and transparency texture scales
are independent, as in the production model shader. The Vulkan probe checks these coordinates
including transparency cutouts in the combined motion and depth pass.

`DiligentHumanAsset` uploads CPU-prepared, material-grouped pedestrian meshes and decoded base
textures directly to Diligent. It retains separate standing and seated skin-weight buffers when
their weights differ, and its submesh draw accepts current and previous rendered-frame bone
palettes plus optional motion parameters. The caller owns pose history, material ordering,
fallback-texture selection, and pass transforms. The Vulkan probe verifies direct image upload,
standing/seated deformation, and rejection of mismatched skin data. The production human loader
retains CPU geometry and batch-to-material mapping for Diligent uploads without GPU readback or a
second model decode. Production pedestrian animation and scene submission use the Diligent path.

`DiligentVehicleAsset` accepts production-prepared `ModelVertex` batches and uploads decoded or
compressed base textures and auxiliary material maps directly to Diligent. Repeated texture
uploads are shared while batch order is preserved; decoded and compressed texture arrays retain
their authored layers and mip chains where supported. Draws accept caller-owned base-texture
replacements and material overrides for runtime script maps, normal maps, lighting, and optional
previous-frame motion data. The caller still owns vehicle animation, transparent sorting, and
rendered-frame history; production vehicle submission uses the Diligent path.

`DiligentObjModel` converts an existing `ParsedObj` into immutable GPU meshes grouped by material,
preserving first-seen material order and triangle order within each group. It resolves OBJ's
one-based position/UV/normal indices, deduplicates matching vertices for indexed drawing, generates
flat normals where normals are absent, and retains material and model-transform metadata. The
`DiligentObjSubmesh::materialState` maps diffuse color, alpha, and specular properties to the
textured-mesh renderer. MTL emission is retained as source metadata because it does not match the
renderer’s existing emission-mask semantics. Supplying a texture root loads relative material
texture references through `DiligentTexture2D::load`, shares repeated textures between batches, and
reports missing/unsupported textures as errors. With no root, texture binding remains caller-owned.
`DiligentObjSubmesh::draw` binds its loaded texture or a caller-provided fallback and applies the
mapped material state. Import options choose the parsed coordinate frame or the vehicle frame
(`x = parsed y`, `y = -parsed x`, `z = parsed z`) for positions and normals, and can flip
texture V; use parsed coordinates with V flipped for map scenery and vehicle coordinates with V
flipped for vehicle meshes. Conversion is directly from `ParsedObj` to Diligent buffers, without
OpenGL resources. The retained model transform is metadata; callers must interpret it according to
the asset and animation path rather than automatically applying it to every draw. Animation,
skinning, and production scene submission are not part of this adapter. Batching may reorder
triangles across material groups, so transparent groups must be sorted by the caller.

The map scenery loader's material-grouped `ModelVertex` preparation is shared independently of
graphics APIs. The Vulkan probe uploads those same CPU vertices directly into a Diligent mesh; it
does not convert an OpenGL buffer. This generic OBJ adapter is not the production map submission path; production map terrain and
road rendering use the Diligent tile pipeline.

Vehicle mesh loading also shares its CPU vertex preparation independently of graphics APIs.
`prepareVehicleMeshGeometry` preserves the vehicle coordinate and UV conventions, authored or
face-generated normals, vertex colors, input triangle order, and existing invalid-index behavior.
The Vulkan probe uploads those vertices directly; production vehicle batching, materials, animation,
and submission use the Diligent path and its vehicle-specific asset adapter.

`DiligentDynamicTexture2D` provides updateable single-mip RGBA8 textures for scripted or text
images. Same-sized updates retain the texture view; a size change replaces it, so callers must
reacquire borrowed views and synchronize use across contexts. `DiligentMaterialState` can select
nearest rather than linear filtering and independently select repeat, clamp, border, mirror, or
mirror-once addressing for the base texture, including texture arrays and motion draws. It can also
request 2x, 4x, 8x, or 16x base-texture anisotropy; the renderer reports the device-supported maximum
and clamps requests to the highest supported level. Nearest filtering takes precedence over
anisotropy. Pipelines are created on first use and cached for each combination. The dynamic resource
is not yet wired to vehicle script textures, and the remaining production material policies are
not yet integrated.

The textured-mesh renderer also has an optional motion-data pipeline. Create it with color,
motion-vector, and validity target formats (for example RGBA8, RG32F, and R8), bind three
compatible color targets with the renderer's sample count in that order, then use `drawMotion` with the current
model-view-projection matrix and `DiligentMotionParameters`. Use
`DiligentMotionParameters::fromHistory` with the shared `MotionDataFrame` and `MotionTransform` to
compose the previous model-view-projection matrix, viewport size, and combined camera/object
history validity. Vectors are previous-minus-current pixel displacement, with positive X
to the right and positive Y up; invalid history or a previous clip position behind the camera
writes a zero vector and zero validity. The caller owns target creation, clearing, and binding.
`DiligentColorPresent` composites a single-sample scene-color shader view into a compatible
single-sample target with clamp-to-edge linear scaling and optional FXAA or SMAA. The SMAA path
uses the same high-quality preset and area/search tables as OpenGL, with source-resolution edge and
weight targets followed by neighborhood blending to the destination. Its intermediate resources
are cleared each frame and recreated when source dimensions change. The source view determines
the color space used for edge detection; use a non-sRGB view to match the current RGBA8 scene
target. The Vulkan probe checks both diagonal directions, flat colors, alpha, scaling, frame
clearing, and filter switching. These are synthetic checks, not a full-scene visual comparison.
Composition performs no GPU readback; callers must order it after scene rendering and before UI
and presentation. Tone mapping and production frame orchestration remain unimplemented.

`DiligentMultisampleTargets` provides color-only multisampled RGBA8 and D32 depth attachments plus a
GPU-resolved single-sample color texture for composition. The primitive and color-only textured
mesh renderers can use a matching supported sample count. Call `resolve` after drawing and
reacquire the color view after resize; the caller owns frame ordering and synchronization.
Unsupported sample counts are rejected rather than silently downgraded. The Vulkan probe checks
depth-tested primitive and textured draws, partial sample coverage, resolve, composition and
resize when the device supports 2x or 4x MSAA. The motion-target path separately tests both sample
counts, with and without linear depth, including overlapping geometry and partial sample coverage.
These components are not yet integrated into the production frame loop or its MSAA fallback policy.

The production Diligent UI path uses GLFW's Vulkan platform backend and a Diligent renderer
for ImGui draw lists, fonts, clipping, callbacks, registered texture views, and large meshes.
The renderer retains registered views until unregistered and requires callers to rebuild the
font texture after changing the atlas. UI is composited after scene color and the production game
uses this Diligent path.
`DiligentTexturedMesh` also accepts per-vertex human skin weights, and `drawSkinned` and
`drawSkinnedMotion` use the engine's 13-bone palette for positions and normals. Skinned motion
uses the caller-provided previous rendered palette; if it is absent, motion is marked invalid.
The shared `prepareHumanMeshGeometry` path expands parsed human triangles into the same
material-grouped vertices and source-position-aligned skin weights used by the production
loader. Those groups can be uploaded directly to `DiligentTexturedMesh` without an OpenGL
intermediate. The caller supplies the common multi-part bounds and chooses authored or inferred
seated weights; material setup and draw ordering remain its responsibility.
The production scene adapter supplies pedestrian poses and motion history; this reusable mesh
component does not retain poses automatically or implement procedural gait deformation.

An optional linear-depth output is enabled by supplying a linear-depth render-target format to the
renderer constructor. It stores positive axial view-space depth (`max(-viewZ, 0)`), not
device-normalized depth. The depth is relative to `DiligentLightingParameters::objectToView`, so
callers needing camera-relative values must supply the view transform even when lighting is
disabled. Bind the color target followed by the depth target for regular draws; when combined with
motion, bind color, motion vectors, validity, then linear depth. The caller owns target creation,
clearing (including the background depth value), and synchronization. The GPU probe checks the
linear-depth value and constructs the combined motion/depth pipeline.

The production game now renders through Diligent Vulkan; the APIs described above include both
production paths and reusable components that are not necessarily used by every material or pass.
OpenGL renderer sources and link dependencies remain in shared modules and tests, so build-system
removal is still pending. Representative visual/performance parity and supported-device validation
remain outstanding. No DLSS or FSR SDK is integrated.
