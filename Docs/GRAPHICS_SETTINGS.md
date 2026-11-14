# Graphics settings

OpenBus reads `OpenBus_graphics.ini` beside the game executable. The file is optional: if it
does not exist, the renderer uses its existing defaults. The pause menu's Graphics section can
edit and save the settings listed below. Saving uses an atomic replacement. If the file cannot
be read, or contains an invalid known value, OpenBus reports the error and disables saving
through the menu so the existing file is not overwritten.

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
| `antialiasing.mode` | `off` or `fxaa` | Live after saving |
| `rendering.scale` | World render resolution scale, `0.5`–`1.0` | Live after saving |
| `advanced.vehicle_frustum_culling` | Reject vehicle parts outside the camera frustum | Restart |

Existing environment variables override the corresponding INI setting:
`OPENBUS_VSYNC`, `OPENBUS_FPS_CAP`, `OPENBUS_REFLECTION_SIZE`,
`OPENBUS_REFLECTION_INTERVAL`, `OPENBUS_REFLECTION_MAX_FPS`,
`OPENBUS_MIN_OBJECT_SIZE_MAIN_VIEW_PERCENT`, and
`OPENBUS_MIN_OBJECT_SIZE_MIRRORS_PERCENT`, `OPENBUS_TEXTURE_SCALE`, and
`OPENBUS_FRUSTUM_CULLING`. An overridden control is disabled and identifies the variable in the
menu; its INI value remains unchanged. Remove the environment variable and restart to use the INI
value.

Display controls are marked **Preview** in the pause menu. Preview changes the mode immediately
and opens a 15-second confirmation dialog; if not confirmed, the previous display mode and window
geometry are restored. Confirm the preview before saving. Borderless fullscreen uses the selected
monitor's current desktop resolution. Exclusive fullscreen offers modes reported by GLFW for the
selected monitor and refuses unsupported resolution/refresh combinations. On startup, an
unsupported exclusive mode saved outside the menu logs a warning and falls back to the monitor's
current mode. If the configured monitor index is unavailable at startup, the primary monitor is
used.

The pause menu's quality preset selector applies a starting point to the implemented world
render scale, reflection, and visibility controls:

| Preset | World render scale | Map texture scale | Vehicle texture scale | Reflection target | Update interval | Reflection cap | Main/mirror culling threshold |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Low | 50% | 50% | 50% | 256 | 3 frames | 30 FPS | 0.25% / 0.5% |
| Medium | 75% | 75% | 75% | 512 | 2 frames | 45 FPS | 0.1% / 0.2% |
| High | 100% | 100% | 100% | 1024 | Every frame | Unlimited | Disabled |

Presets do not change display settings, VSync, the main frame-rate cap, anti-aliasing, anisotropic
filtering, mipmap generation, or advanced vehicle frustum culling. They skip values controlled by
environment-variable overrides. Any individual edits that no longer match a preset are shown as
**Custom**. Reflection, anisotropic-filtering, mipmap-generation, vehicle texture-resolution, and
advanced vehicle culling changes require restarting OpenBus.

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

FXAA runs as a full-resolution color pass after the world color and motion-data pass. It does not
filter or resize the motion-vector, validity, or depth buffers, and the UI is drawn afterward at
native resolution. FXAA replaces the color blit with a fullscreen filter draw. It allocates no
additional render target, but adds texture samples across the frame. MSAA is intentionally not
offered because the renderer does not yet have a surface-correct resolve for motion vectors,
validity, and depth.

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

HDR output is not implemented, and no HDR toggle is presented. The current presentation path
creates a GLFW OpenGL window and presents with `glfwSwapBuffers`; the main scene target is
`GL_RGBA8`, and the renderer has no HDR tone-mapping/transfer stage, HDR monitor-capability query,
or swap-chain color-space selection. The GLFW framebuffer hints used by this project expose
component bit depths and sRGB capability, but not an end-to-end HDR output mode. A higher-
precision render target alone would therefore not make the displayed output HDR. A real option
requires an HDR-capable native presentation path and end-to-end validation with Windows HDR and
an HDR display; until then the setting remains intentionally absent.

MSAA is unavailable; only Off and FXAA are currently supported. No placeholder controls are
presented.
